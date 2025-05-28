#include "CatParser.h"
#include "esp_log.h"
#include <algorithm>

namespace radio {
    CatParser::CatParser() {
        ESP_LOGD(CatParser::CatParser::TAG, "CatParser initialized");
    }

    void CatParser::parseMessage(const std::string_view message, CommandSource source, CommandCallback callback) {
        if (message.empty()) {
            ESP_LOGW(CatParser::TAG, "parseMessage called with empty message");
            return;
        }

        stats_.totalMessagesParsed++;
        if (source == CommandSource::UsbCdc0 || source == CommandSource::UsbCdc1 || source == CommandSource::Display || source == CommandSource::Panel) {
            stats_.localCommandsParsed++;
        } else {
            stats_.remoteCommandsParsed++;
        }

        ESP_LOGV(CatParser::TAG, "Parsing %s message: '%.*s' (length=%zu)",
                 (source == CommandSource::UsbCdc0 ? "Usb0" : (source == CommandSource::UsbCdc1 ? "Usb1" :
                 source == CommandSource::Display ? "Display" :
                 source == CommandSource::Panel ? "Panel" : "Remote")),
                 static_cast<int>(message.length()), message.data(), message.length());

        // Split message into individual CAT frames
        // ReSharper disable once CppDFAUnusedValue
        size_t frameCount = cat::ParserUtils::splitCATMessage(
            message, [this, source, &callback](const std::string_view frame) {
                if (const auto command = parseFrame(frame, source)
                    ; command.has_value()) {
                    callback(command.value());
                }
            });

        ESP_LOGV(CatParser::TAG, "Parsed %zu frames from message", frameCount);
    }

    std::optional<RadioCommand> CatParser::parseFrame(std::string_view frame, const CommandSource source) {
        // Sanitize frame by removing radio firmware quirks (e.g., "?;FA;" or "FA?;")
        // This is now handled at the protocol layer, not the transport layer
        frame = cat::ParserUtils::sanitizeFrame(frame);

        // Handle special cases first
        if (frame.empty()) {
            return std::nullopt;
        }

        // Handle wakeup/terminator
        if (frame == ";") {
            RadioCommand command("", CommandType::Set, source, std::string(frame));
            command.command = ";";
            return command;
        }

        // Handle error responses
        // Radio sometimes emits bare "?;" as a benign/ambiguous error marker. We suppress it.
        if (source == CommandSource::Remote && frame == "?;") {
            ESP_LOGD(CatParser::TAG, "Ignoring remote '?;' frame from radio");
            return std::nullopt;
        }
        if (isErrorResponse(frame)) {
            RadioCommand command(std::string(frame.substr(0, frame.length() - 1)), CommandType::Answer, source,
                                 std::string(frame));
            return command;
        }

        // Validate basic CAT format
        if (!cat::ParserUtils::isValidCAT(frame)) {
            reportError("Invalid CAT format: " + std::string(frame));
            stats_.parseErrors++;
            return std::nullopt;
        }

        // Extract command prefix
        std::string_view prefix = cat::ParserUtils::getPrefix(frame);
        if (prefix.empty()) {
            reportError("Missing command prefix: " + std::string(frame));
            stats_.parseErrors++;
            return std::nullopt;
        }

        // UI meta commands use 4-character prefixes (e.g., UIPC, UIXD)
        // Extend prefix if this is a UI command with at least 4 chars before semicolon
        if (prefix == "UI" && frame.length() >= 5) {
            prefix = frame.substr(0, 4);
        }

        // Determine command type based on content and source
        const CommandType type = determineCommandType(frame, source);

        // Create RadioCommand with original message
        RadioCommand command(std::string(prefix), type, source, std::string(frame));

        // Parse parameters if present (uses SSO inline storage for zero-allocation)
        if (type == CommandType::Set || type == CommandType::Answer || prefix == "EX") {
            parseParameters(command, frame, prefix);
        }

        // Update statistics
        switch (type) {
            case CommandType::Set:
                stats_.setCommands++;
                break;
            case CommandType::Read:
                stats_.readCommands++;
                break;
            case CommandType::Answer:
                stats_.answerCommands++;
                break;
            default:
                break;
        }

        ESP_LOGV(CatParser::TAG, "Parsed command: %s", command.describe().c_str());
        return command;
    }

    CommandType CatParser::determineSetCommandType(const CommandSource source, const std::string_view prefix,
                                                   const std::string_view params) {
        if (RadioCommand::isLocalSource(source) && setOnlyCommandPrefixes(prefix) == CommandType::Set) {
            return CommandType::Set;
        }

        // if local origin & params, it's a SET, we don't send ANSWER's
        if (RadioCommand::isLocalSource(source) && !params.empty()) {
            return CommandType::Set;
        }

        // Remote source with parameters is typically an ANSWER
        if (source == CommandSource::Remote && !params.empty()) {
            return CommandType::Answer;
        }
        
        // Remote source with empty parameters (shouldn't happen for AG commands but handle gracefully)
        if (source == CommandSource::Remote && params.empty()) {
            ESP_LOGI(CatParser::TAG, "Received CAT command prefix: %.*s with CommandSource::Remote with empty params",
                     static_cast<int>(prefix.size()), prefix.data());
        }

        return CommandType::Unknown;
    }

    /**
     * @brief Checks whether command prefix should be treated as SET even without parameters
     * @ref ts590sg_cat_commands_v3.json
     *
     * Two categories:
     * 1. Pure set-only commands (no read/answer in spec): BD, BU, CH, DN, EM, MK, MW, QD, QI, RC, SR, SV, UP, VV
     * 2. Action commands where no-params still triggers action: RX (receive), TX (transmit)
     *
     * Note: VS1/VS2 are set-only but 3-char prefixes not handled here
     */
    CommandType CatParser::setOnlyCommandPrefixes(const std::string_view prefix) {
        // Pure set-only commands per JSON spec
        if (prefix == "BD" || prefix == "BU" || prefix == "CH" || prefix == "DN" ||
            prefix == "EM" || prefix == "MK" || prefix == "MW" || prefix == "QD" ||
            prefix == "QI" || prefix == "RC" || prefix == "SR" || prefix == "SV" ||
            prefix == "UP" || prefix == "VV") {
            return CommandType::Set;
        }
        // Action commands - sending without params triggers the action
        if (prefix == "RX" || prefix == "TX") {
            return CommandType::Set;
        }
        return CommandType::Unknown;
    }

    CommandType CatParser::determineCommandType(const std::string_view frame, const CommandSource source) {
        // Extract parameters to check if frame has data
        std::string_view prefix = cat::ParserUtils::getPrefix(frame);
        std::string_view params = cat::ParserUtils::getParameters(frame);

        // UI meta commands use 4-character prefixes (e.g., UIPC, UIXD, UIDE)
        // Adjust prefix and params to account for extended prefix length
        if (prefix == "UI" && frame.length() >= 5) {
            prefix = frame.substr(0, 4);  // Extend to 4 chars: "UIDE", "UIPC", etc.
            params = frame.substr(4, frame.length() - 5);  // Params start at char 4
        }

        // Query format: command with no parameters (e.g., "FA;", "UIDE;")
        // Exception: some commands are action commands that should be Set even without params
        if (params.empty()) {
            // For LOCAL sources: Action commands that only support SET operations 
            if (RadioCommand::isLocalSource(source)) {
                if (setOnlyCommandPrefixes(prefix) == CommandType::Set) {
                    return CommandType::Set;
                }
                // Empty parameters from local sources are READ/query commands
                return CommandType::Read;
            }
            
            // For REMOTE sources: Empty parameters are typically ANSWER responses
            // (Radio sending status updates like "RX;", "TX0;", "FA14070000;" etc.)
            // EXCEPTION: PS; from RRC-1258 is a QUERY, not an answer!
            if (prefix == "PS") {
                return CommandType::Read;
            }
            return CommandType::Answer;
        }

        // Special cases where a minimal selector parameter still indicates a Read query
        // CD0; / CD1; / CD2; are queries selecting sub-function, not sets
        if (prefix == "CD" && params.length() == 1u) {
            return CommandType::Read;
        }

        if (prefix == "SM" && params == "0") {
            return CommandType::Read;
        }

        // EX commands: 7 chars = READ (menu + "0000"), 8+ chars depends on source
        if (prefix == "EX") {
            if (params.length() == 7) {
                return CommandType::Read;
            }
            // 8+ chars: Usb/Display=SET, Remote=ANSWER (radio response)
            if (RadioCommand::isLocalSource(source)) {
                return CommandType::Set;
            }
            return CommandType::Answer;
        }

        // Special handling for AG commands based on CAT specification
        if (prefix == "AG") {
            // AG with single parameter (AG0;) = read command
            // AG with multiple parameters (AG0123;) = set command
            if (params.length() == 1) {
                return CommandType::Read;
            }
            return determineSetCommandType(source, prefix, params);
        }

        // Frame has parameters - determine if it's Set or Answer based on source and context
        if (RadioCommand::isLocalSource(source)) {
            // Commands that arrive from USB CDC, the display or our panel interface are likely set commands
            return CommandType::Set;
        }

        // Check if this might be an unsolicited update from radio
        // For now, treat all remote frames with parameters as Answer
        return CommandType::Answer;
    }

    void CatParser::parseParameters(RadioCommand& command, const std::string_view frame,
                                     const std::string_view commandPrefix) {
        // UI meta commands have 4-char prefix, extract params starting at position 4
        std::string_view paramStr;
        if (commandPrefix.starts_with("UI") && commandPrefix.length() == 4) {
            // UI commands: extract after 4-char prefix, before semicolon
            if (frame.length() > 5 && frame.ends_with(";")) {
                paramStr = frame.substr(4, frame.length() - 5);
            }
        } else {
            paramStr = cat::ParserUtils::getParameters(frame);
        }

        if (paramStr.empty()) {
            return;
        }

        // Commands that preserve string format (frequencies, multi-field data)
        if (commandPrefix == "FA" || commandPrefix == "FB" || commandPrefix == "SP" || commandPrefix == "RU" ||
            commandPrefix == "RD" || commandPrefix == "XO" || commandPrefix == "EX" || commandPrefix == "AN" ||
            commandPrefix == "EQ" || commandPrefix == "SS") {
            // Frequency/extended parameters - keep as string to preserve leading zeros
            command.addParam(paramStr);
        } else if (commandPrefix == "CD") {
            // CD: Compact subcommand format CDx<value> - split selector and value
            command.addParam(paramStr.substr(0, 1)); // Selector ('0','1','2')
            if (paramStr.length() > 1) {
                command.addParam(paramStr.substr(1));
            }
        } else if (commandPrefix == "MD" || commandPrefix == "DA" ||
                   commandPrefix == "AG" || commandPrefix == "RG") {
            // Numeric parameters - try to parse as int
            if (const int value = cat::ParserUtils::parseInt(paramStr); value >= 0) {
                command.addParam(value);
            } else {
                command.addParam(paramStr);
            }
        } else if (commandPrefix == "LM") {
            // LM: VGS-1 recorder - LMP1P2; (P1=mode, P2=channel)
            if (paramStr.length() >= 2) {
                command.addParam(paramStr.substr(0, 1));
                command.addParam(paramStr.substr(1, 1));
            } else if (paramStr.length() == 1) {
                command.addParam(paramStr);
            }
        } else if (commandPrefix == "PA" || commandPrefix == "NT") {
            // PA: Pre-amplifier - SET=PAP1; ANSWER=PAP1P2; (P1=on/off, P2=always 0)
            // NT: Notch Filter - SET=NTP1P2; ANSWER=NTP1P2; (P1=mode, P2=bandwidth)
            // Both have width-1 P1 and P2, total 2 chars in answer
            if (paramStr.length() >= 2) {
                command.addParam(paramStr.substr(0, 1)); // P1
                command.addParam(paramStr.substr(1, 1)); // P2
            } else if (paramStr.length() == 1) {
                command.addParam(paramStr); // SET format with single param
            }
        } else if (commandPrefix == "RA") {
            // RA: RF Attenuator - SET=RAP1P1; ANSWER=RAP1P1P2P2; (P1=2 chars, P2=2 chars)
            if (paramStr.length() >= 4) {
                command.addParam(paramStr.substr(0, 2)); // P1 (width 2)
                command.addParam(paramStr.substr(2, 2)); // P2 (width 2)
            } else if (paramStr.length() >= 2) {
                command.addParam(paramStr); // SET format
            }
        } else if (commandPrefix == "MC") {
            // MC: Memory channel - preserve space for 100-series channels
            command.addParam(paramStr);
        } else if (commandPrefix == "PR") {
            // PR: Speech Processor enable - try int first
            if (const int value = cat::ParserUtils::parseInt(paramStr); value >= 0) {
                command.addParam(value);
            } else {
                command.addParam(paramStr);
            }
        } else if (commandPrefix == "AS") {
            // AS: Auto Mode channel - SET(16 chars) or READ(3 chars) format
            if (paramStr.length() >= 16) {
                command.addParam(paramStr.substr(0, 1));   // P1: const
                command.addParam(paramStr.substr(1, 2));   // P2: channel
                command.addParam(paramStr.substr(3, 11));  // P3: frequency
                command.addParam(paramStr.substr(14, 1));  // P4: mode
                command.addParam(paramStr.substr(15, 1));  // P5: data (uses overflow + params)
            } else if (paramStr.length() >= 3) {
                command.addParam(paramStr.substr(0, 1));
                command.addParam(paramStr.substr(1, 2));
            } else {
                command.addParam(paramStr);
            }
        } else if (commandPrefix == "PL") {
            // PL: Speech Processor levels - PLP1P1P1P2P2P2;
            if (paramStr.length() >= 6) {
                command.addParam(paramStr.substr(0, 3));
                command.addParam(paramStr.substr(3, 3));
            } else {
                command.addParam(paramStr);
            }
        } else if (commandPrefix == "IF") {
            // IF: Information command - 15 parameters per TS-590SG spec
            // Format: P1(11)P2(5)P3(5)P4(1)P5(1)P6(1)P7(2)P8(1)P9(1)P10(1)P11(1)P12(1)P13(1)P14(2)P15(1)
            if (paramStr.length() >= 34) {
                command.addParam(paramStr.substr(0, 11));   // P1: frequency
                command.addParam(paramStr.substr(11, 5));   // P2: 5 spaces
                command.addParam(paramStr.substr(16, 5));   // P3: RIT/XIT offset
                command.addParam(paramStr.substr(21, 1));   // P4: RIT status
                command.addParam(paramStr.substr(22, 1));   // P5: XIT status (overflow)
                command.addParam(paramStr.substr(23, 1));   // P6: mem ch hundreds (overflow)
                command.addParam(paramStr.substr(24, 2));   // P7: mem ch tens+ones (overflow)
                command.addParam(paramStr.substr(26, 1));   // P8: TX/RX status (overflow)
                command.addParam(paramStr.substr(27, 1));   // P9: mode (overflow)
                command.addParam(paramStr.substr(28, 1));   // P10: VFO/Memory (overflow)
                command.addParam(paramStr.substr(29, 1));   // P11: scan (overflow)
                command.addParam(paramStr.substr(30, 1));   // P12: split (overflow)
                command.addParam(paramStr.substr(31, 1));   // P13: tone status (overflow)
                command.addParam(paramStr.substr(32, 2));   // P14: tone freq idx (overflow)
                command.addParam(paramStr.substr(34, 1));   // P15: reserved (overflow)
            } else {
                command.addParam(paramStr);
            }
        } else {
            // Default: try int first, fall back to string
            if (const int value = cat::ParserUtils::parseInt(paramStr); value >= 0) {
                command.addParam(value);
            } else {
                command.addParam(paramStr);
            }
        }
    }

    bool CatParser::isErrorResponse(const std::string_view frame) {
        return frame == "?;" || frame == "E;" || frame == "O;";
    }

    void CatParser::reportError(const std::string_view error) const {
        ESP_LOGW(CatParser::TAG, "%s", error.data());
        if (errorCallback_) {
            errorCallback_(error);
        }
    }
} // namespace radio
