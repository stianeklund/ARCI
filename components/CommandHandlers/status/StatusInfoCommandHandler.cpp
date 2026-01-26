#include "StatusInfoCommandHandler.h"
#include "RadioManager.h"
#include "RadioState.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include <array>
#include <charconv>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include "sdkconfig.h"

namespace {

constexpr size_t IF_RESPONSE_RESERVE = 48;

void appendZeroPadded(std::string &out, uint64_t value, size_t width) {
    const uint64_t original = value;
    std::array<char, 32> buffer{};
    size_t writePos = width;
    while (writePos > 0) {
        buffer[--writePos] = static_cast<char>('0' + (value % 10));
        value /= 10;
    }

    if (value != 0) {
        std::array<char, 32> fallback{};
        auto [ptr, ec] = std::to_chars(fallback.data(), fallback.data() + fallback.size(), original);
        if (ec == std::errc()) {
            out.append(fallback.data(), ptr);
            return;
        }
    }

    out.append(buffer.data(), width);
}

void appendNumber(std::string &out, int value) {
    std::array<char, 16> buffer{};
    auto [ptr, ec] = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    if (ec == std::errc()) {
        out.append(buffer.data(), ptr);
    } else {
        out.push_back('0');
    }
}

} // namespace

namespace radio {

StatusInfoCommandHandler::StatusInfoCommandHandler()
    : BaseCommandHandler({"TY", "RS", "SM", "RM", "IF", "ID", "BY", "FV", "RI", "XI"}, "Status & Information Commands") {
}

bool StatusInfoCommandHandler::handleCommand(const RadioCommand& command,
                                           ISerialChannel& radioSerial,
                                           ISerialChannel& usbSerial,
                                           RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling status/info command: %s", command.command.c_str());
    
    if (command.command == "SM") {
        return handleSM(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "RM") {
        return handleRM(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "IF") {
        return handleIF(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "ID") {
        return handleID(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "BY") {
        return handleBY(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "RS") {
        respondToSource(command, buildCommand("RS", "0"), usbSerial, radioManager);
        return true;
    }
    if (command.command == "TC") {
        respondToSource(command, buildCommand("TC ", "1"), usbSerial, radioManager);
        return true;
    }
        if (command.command == "TY") {
        // TY: Type (undocumented) -> "TYK00;" Possibly for TYPE Kenwood or TYPE K (USA), TYPE E (Europe)?
        if (isQuery(command)) {
            // Fixed-answer: do not traverse RRC; always answer locally
            respondToSource(command, buildCommand("TYK", "00"), usbSerial, radioManager);
            return true;
        }
        if (command.type == CommandType::Answer) {
            // Normalize to expected answer format
            routeAnswerResponse(command, buildCommand("TYK", "00"), usbSerial, radioManager);
            return true;
        }
        return false;
    }
    if (command.command == "XI") {
        // XI: Transmit frequency/mode read
        if (isQuery(command)) {
            if (command.isCatClient()) {
                const auto& state = radioManager.getState();

                const auto selectVfoFrequency = [&state](const int vfo) -> uint64_t {
                    switch (vfo) {
                        case 0:
                            return state.vfoAFrequency.load();
                        case 1:
                            return state.vfoBFrequency.load();
                        default:
                            break;
                    }
                    return 0ULL;
                };

                uint64_t txFrequency = selectVfoFrequency(state.currentTxVfo.load());
                if (txFrequency == 0) {
                    // Fallback to RX VFO if TX frequency is unavailable
                    txFrequency = selectVfoFrequency(state.currentRxVfo.load());
                }
                if (txFrequency == 0) {
                    // Absolute fallback to VFO A/B if state not yet populated
                    txFrequency = selectVfoFrequency(0);
                    if (txFrequency == 0) {
                        txFrequency = selectVfoFrequency(1);
                    }
                }

                // Apply transverter offset for display if enabled (controlled by UIXD1/UIXD0)
                if (state.transverterOffsetEnabled && state.transverter) {
                    const uint64_t offset = state.transverterOffsetHz;
                    const bool plus = state.transverterOffsetPlus;
                    if (offset > 0) {
                        if (plus) {
                            txFrequency += offset;
                        } else {
                            txFrequency = (txFrequency > offset) ? (txFrequency - offset) : 0ULL;
                        }
                    }
                }

                // Build XI response directly to avoid stack-heavy std::ostringstream
                std::string response;
                response.reserve(17); // "XI" + 11 freq + mode + datamode + "00" + ";"
                response.append("XI");
                // P1: 11-digit TX frequency derived from current TX VFO
                appendZeroPadded(response, txFrequency, 11);
                // P2: mode (1 digit)
                response += static_cast<char>('0' + static_cast<int>(state.mode.load()));
                // P3: data mode (0/1)
                response += static_cast<char>('0' + (state.dataMode.load() ? 1 : 0));
                // P4P4: always 00
                response.append("00;");
                respondToSource(command, response, usbSerial, radioManager);
            } else if (shouldSendToRadio(command)) {
                ESP_LOGD(TAG, "XI query being sent to radio");
                sendToRadio(radioSerial, buildCommand("XI"));
            }
            return true;
        }
        if (command.type == CommandType::Answer) {
            // Synchronize TX-side VFO information from the radio's response
            const auto &state = radioManager.getState();
            const int txVfo = state.currentTxVfo.load();

            if (command.originalMessage.length() >= 13 && command.originalMessage.rfind("XI", 0) == 0) {
                const std::string_view payload(command.originalMessage);
                const std::string_view freqView = payload.substr(2, 11);
                uint64_t txFrequency = 0;

                if (!freqView.empty()) {
                    const auto result = std::from_chars(freqView.data(), freqView.data() + freqView.size(), txFrequency);
                    if (result.ec == std::errc{} && txFrequency > 0) {
                        if (txVfo == 0) {
                            radioManager.updateVfoAFrequency(txFrequency);
                        } else if (txVfo == 1) {
                            radioManager.updateVfoBFrequency(txFrequency);
                        } else {
                            ESP_LOGW(TAG, "XI answer received but TX VFO %d is unsupported", txVfo);
                        }
                    }
                }

                // Update TX mode (P2) and data mode (P3) for completeness
                const size_t modeIndex = 2 + 11;
                if (payload.length() > modeIndex) {
                    const char modeChar = payload[modeIndex];
                    if (modeChar >= '0' && modeChar <= '9') {
                        radioManager.updateMode(modeChar - '0');
                    }
                }

                const size_t dataIndex = modeIndex + 1;
                if (payload.length() > dataIndex) {
                    const char dataChar = payload[dataIndex];
                    if (dataChar == '0' || dataChar == '1') {
                        radioManager.updateDataMode(dataChar - '0');
                    }
                }
            }

            // Update cache to prevent unnecessary polling
            const uint64_t currentTime = esp_timer_get_time();
            radioManager.getState().commandCache.update("XI", currentTime);
            
            ESP_LOGD(TAG, "*** XI ANSWER RECEIVED: '%s' - CACHE UPDATED ***", command.originalMessage.c_str());
            
            // Route via unified policy (original formatting)
            routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
            return true;
        }
        return false;
    }
    if (command.command == ";") {
        ESP_LOGD(TAG, "Received ; from: %d 0 local 1 remote", command.source);
        // Don't send ?; error marker to USB client - just ignore bare semicolon
        return true;
    }

    if (command.command == "FV") {
        // Firmware version
        if (isQuery(command)) {
            // Fixed-answer: do not traverse RRC; always answer locally
            const auto& state = radioManager.getState();
            std::string ver;
#ifdef CONFIG_RADIOCORE_EMULATE_TS2000
            ver = "1.20";
#else
            ver = (state.firmwareVersion[0] == '\0') ? std::string(CONFIG_RADIOCORE_FV_STRING) : std::string(state.firmwareVersion.data());
#endif
            respondToSource(command, buildCommand("FV", ver), usbSerial, radioManager);
            return true;
        }
        if (command.type == CommandType::Answer) {
            std::string ver;
#ifdef CONFIG_RADIOCORE_EMULATE_TS2000
            ver = "1.20";
#else
            ver = getStringParam(command, 0, CONFIG_RADIOCORE_FV_STRING);
#endif
            routeAnswerResponse(command, buildCommand("FV", ver), usbSerial, radioManager);
            return true;
        }
        return false;
    } else if (command.command == "RI") {
        return handleRI(command, radioSerial, usbSerial, radioManager);
    }

    return false;
}

bool StatusInfoCommandHandler::handleSM(const RadioCommand& command,
                                       ISerialChannel& radioSerial,
                                       ISerialChannel& usbSerial,
                                       RadioManager& radioManager) const {
    // SM: S-meter reading
    if (isQuery(command)) {
        const auto &state = radioManager.getState();

        // AI-mode aware handling: Check AI mode of requesting source
        uint8_t aiMode = 0;
        if (command.source == CommandSource::UsbCdc0) {
            aiMode = state.usbCdc0AiMode.load();
        } else if (command.source == CommandSource::UsbCdc1) {
            aiMode = state.usbCdc1AiMode.load();
        } else if (command.source == CommandSource::Display) {
            aiMode = state.displayAiMode.load();
        }

        // In AI2/AI4 modes: answer immediately from cached value, don't poll radio
        if (aiMode == 2 || aiMode == 4) {
            const std::string response = formatSMeterResponse(state.meterSmRaw);
            respondToSource(command, response, usbSerial, radioManager);
            ESP_LOGD(TAG, "SM query in AI%d mode: answered from cache (%d) without polling",
                     aiMode, state.meterSmRaw);
            return true;
        }

        // For AI0/AI1: use standard cache+refresh logic with appropriate TTL
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "SM", TTL_REALTIME, // 500ms TTL for fresh data
            [this](const RadioState& state) {
                // Return real cached SM value from radio
                return formatSMeterResponse(state.meterSmRaw);
            },
            "0" // SM READ requires parameter 0
        );
    }
    
    if (command.type == CommandType::Answer) {
        ESP_LOGV(TAG, "🔍 SM ANSWER HANDLER called: originalMsg='%s', source=%d",
                 command.originalMessage.c_str(), (int)command.source);

        // Parse and store real S-meter value from radio (P2: 4-digit value 0000-9999)
        if (command.params.size() >= 2) {
            std::string valueStr = getStringParam(command, 1, "");
            if (!valueStr.empty() && valueStr.length() <= 4) {
                int smValue = std::stoi(valueStr);
                radioManager.getState().meterSmRaw = smValue;
                ESP_LOGD(TAG, "SM value stored: %d", smValue);
            }
        }

        // Always update cache timestamp when we receive answer from radio
        radioManager.getState().commandCache.update("SM", esp_timer_get_time());

        // Use unified routing for SM answers
        routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
        return true;
    }
    
    return false;
}

bool StatusInfoCommandHandler::handleRM(const RadioCommand& command,
                                       ISerialChannel& radioSerial,
                                       ISerialChannel& usbSerial,
                                       RadioManager& radioManager) const {
    // RM: Meter function (set) and reading (query)
    if (isSet(command)) {
        int func = getIntParam(command, 0, -1);
        if (func < 0 || func > 3) {
            ESP_LOGW(TAG, "Invalid RM function: %d", func);
            return false;
        }
        radioManager.getState().meterFunction = func;
        
        // Update cache timestamp when state changes
        uint64_t currentTime = esp_timer_get_time();
        radioManager.getState().commandCache.update("RM", currentTime);
        
        // Forward set to radio if local
        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("RM", std::to_string(func));
            sendToRadio(radioSerial, cmdStr);
        }
        return true;
    }

    // Query: return current meter selection with meter reading
    if (isQuery(command)) {
        const auto &state = radioManager.getState();

        // Helper lambda to build RM response
        auto buildRmResponse = [](const RadioState& state) {
            int currentFunc = state.meterFunction;
            int meterValue = 0;
            switch (currentFunc) {
                case 1: meterValue = state.meterSwr; break;
                case 2: meterValue = state.meterComp; break;
                case 3: meterValue = state.meterAlc; break;
                default: meterValue = 0; break;
            }
            // Build RM parameter directly to avoid stack-heavy std::ostringstream
            std::string param;
            param.reserve(5); // func(1) + meter(4)
            param += static_cast<char>('0' + currentFunc);
            appendZeroPadded(param, static_cast<uint64_t>(meterValue), 4);
            return BaseCommandHandler::buildCommand("RM", param);
        };

        // AI-mode aware handling: Check AI mode of requesting source
        uint8_t aiMode = 0;
        if (command.source == CommandSource::UsbCdc0) {
            aiMode = state.usbCdc0AiMode.load();
        } else if (command.source == CommandSource::UsbCdc1) {
            aiMode = state.usbCdc1AiMode.load();
        } else if (command.source == CommandSource::Display) {
            aiMode = state.displayAiMode.load();
        }

        // In AI2/AI4 modes: answer immediately from cached value, don't poll radio
        if (aiMode == 2 || aiMode == 4) {
            const std::string response = buildRmResponse(state);
            respondToSource(command, response, usbSerial, radioManager);
            ESP_LOGD(TAG, "RM query in AI%d mode: answered from cache without polling", aiMode);
            return true;
        }

        // For AI0/AI1: use standard cache+refresh logic with appropriate TTL
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "RM", TTL_STATUS,
            buildRmResponse
        );
    }
    
    // Handle Answer commands from radio
    if (command.type == CommandType::Answer) {
        auto &state = radioManager.getState();

        // Parse meter function (P1) and meter value (P2: 4-digit value 0000-9999)
        if (command.params.size() >= 2) {
            int func = getIntParam(command, 0, -1);
            std::string valueStr = getStringParam(command, 1, "");

            if (func >= 0 && func <= 3) {
                state.meterFunction = func;

                // Store meter value in appropriate field based on function
                if (!valueStr.empty() && valueStr.length() <= 4) {
                    int meterValue = std::stoi(valueStr);
                    switch (func) {
                        case 1:
                            state.meterSwr = meterValue;
                            ESP_LOGD(TAG, "RM SWR value stored: %d", meterValue);
                            break;
                        case 2:
                            state.meterComp = meterValue;
                            ESP_LOGD(TAG, "RM COMP value stored: %d", meterValue);
                            break;
                        case 3:
                            state.meterAlc = meterValue;
                            ESP_LOGD(TAG, "RM ALC value stored: %d", meterValue);
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        // Update cache timestamp for AI mode compatibility
        uint64_t currentTime = esp_timer_get_time();
        radioManager.getState().commandCache.update("RM", currentTime);

        // Use unified routing for RM answers
        routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
        return true;
    }
    
    return false;
}

bool StatusInfoCommandHandler::handleIF(const RadioCommand& command,
                                       ISerialChannel& radioSerial,
                                       ISerialChannel& usbSerial,
                                       RadioManager& radioManager) const {
    // IF: Information (frequency, mode, RIT/XIT, etc.)
    
    // Handle queries
    if (isQuery(command)) {
        const auto &state = radioManager.getState();
        const uint64_t nowUs = esp_timer_get_time();
        const uint64_t lastIfUpdateUs = state.commandCache.get("IF");
        const uint64_t ageMs = lastIfUpdateUs ? (nowUs - lastIfUpdateUs) / 1000 : 0;
        ESP_LOGD(TAG,
                 "IF QUERY: src=%d tx=%d tune=%d vfoA=%llu vfoB=%llu age=%llums",
                 static_cast<int>(command.source),
                 state.isTx.load(),
                 state.isTuning.load(),
                 static_cast<unsigned long long>(state.vfoAFrequency.load()),
                 static_cast<unsigned long long>(state.vfoBFrequency.load()),
                 static_cast<unsigned long long>(ageMs));

        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "IF", TTL_HIGH_FREQ,
            [this](const RadioState& st) {
                return formatIFResponse(st);
            }
        );
    }
    
    if (command.type == CommandType::Answer) {
        // Parse IF answer parameters and update state
        if (command.params.size() >= 5) {
            auto &state = radioManager.getState();

            const int txRxState = getIntParam(command, 7, -1);  // P8: 0=RX, 1=TX
            const bool isTransmitFrame = (txRxState == 1);
            const int reportedVfo = getIntParam(command, 9, -1); // P10: VFO/Memory selection (0=A,1=B,2=Memory)

            // P1: 11-digit frequency (skip state update while local tuning is active)
            if (std::string freqStr = getStringParam(command, 0, ""); !freqStr.empty() && freqStr.length() >= 11) {
                uint64_t frequency = 0;
                const auto freqParseResult = std::from_chars(freqStr.data(), freqStr.data() + freqStr.size(), frequency);

                if (freqParseResult.ec == std::errc{} && frequency > 0) {
                    int targetVfo = reportedVfo;

                    if (!state.isTuning.load()) {

                        if (targetVfo < 0 || targetVfo > 2) {
                            const int fallbackVfo = isTransmitFrame ? state.currentTxVfo.load()
                                                                     : state.currentRxVfo.load();
                            targetVfo = fallbackVfo;
                        }

                        bool updated = false;
                        switch (targetVfo) {
                            case 0:
                                updated = radioManager.updateVfoAFrequency(frequency);
                                break;
                            case 1:
                                updated = radioManager.updateVfoBFrequency(frequency);
                                break;
                            default:
                                ESP_LOGD(TAG, "IF answer frequency for unsupported VFO %d - skipping cache update", targetVfo);
                                break;
                        }

                        ESP_LOGD(TAG, "IF ANS: vfo%c=%llu %s",
                                 targetVfo == 0 ? 'A' : 'B',
                                 static_cast<unsigned long long>(frequency),
                                 updated ? "(changed)" : "(unchanged)");
                    } else {
                        ESP_LOGV(TAG, "IF ANSWER ignored due to tuning (freq=%s, targetVfo=%d)", freqStr.c_str(), targetVfo);
                    }
                } else {
                    ESP_LOGW(TAG, "IF answer contained invalid frequency payload: %s", freqStr.c_str());
                }
            }

            if (reportedVfo >= 0 && reportedVfo <= 2) {
                if (isTransmitFrame) {
                    radioManager.updateTxVfo(reportedVfo);
                } else {
                    radioManager.updateRxVfo(reportedVfo);
                }
            }

            // P3: RIT/XIT offset (5 chars, signed)
            if (std::string offsetStr = getStringParam(command, 2, ""); offsetStr.length() == 5) {
                int offset = 0;
                if (offsetStr[0] == ' ') {
                    offset = std::stoi(offsetStr.substr(1));
                } else if (offsetStr[0] == '-') {
                    offset = -std::stoi(offsetStr.substr(1));
                }
                radioManager.updateRitOffset(offset);
            }
            
            // P4: RIT status
            if (std::string ritStr = getStringParam(command, 3, ""); !ritStr.empty()) {
                bool ritOn = ritStr == "1";
                radioManager.updateRitEnabled(ritOn);
            }
            
            // P5: XIT status
            if (std::string xitStr = getStringParam(command, 4, ""); !xitStr.empty()) {
                bool xitOn = xitStr == "1";
                radioManager.updateXitEnabled(xitOn);
            }
        }
        
        // Process additional parameters if available (P6-P15)
        if (command.params.size() >= 15) {
            auto& state = radioManager.getState();
            
            // P6+P7: Memory channel (P6=hundreds, P7=tens+ones)
            std::string hundredsStr = getStringParam(command, 5, "");
            if (std::string tensOnesStr = getStringParam(command, 6, ""); !hundredsStr.empty() && !tensOnesStr.empty()) {
                int channel = std::stoi(hundredsStr + tensOnesStr);
                state.memoryChannel.store(channel);
            }
            
            // P8: TX/RX status
            if (std::string txRxStr = getStringParam(command, 7, ""); !txRxStr.empty()) {
                bool radioTxState = txRxStr == "1";
                const uint64_t currentTime = esp_timer_get_time();

                // Check if this IF response was solicited (someone recently queried IF)
                const bool ifWasQueried = state.queryTracker.wasRecentlyQueried("IF", currentTime);
                const bool hasCurrentOwner = (state.getTxOwner() != -1);

                if (radioTxState) {
                    // Radio reports TX - only manage ownership if:
                    // 1. IF was specifically queried, OR
                    // 2. No one currently owns TX (orphaned state)
                    if (ifWasQueried || !hasCurrentOwner) {
                        if (!state.tryAcquireTx(CommandSource::Remote, currentTime)) {
                            ESP_LOGW(TAG, "IF: Radio TX detected, forcing ownership for Remote source (queried=%s, hasOwner=%s)",
                                     ifWasQueried ? "yes" : "no", hasCurrentOwner ? "yes" : "no");
                            state.forceReleaseTx(currentTime);
                            state.tryAcquireTx(CommandSource::Remote, currentTime);
                        }
                    } else {
                        // Unsolicited IF in AI mode - don't steal ownership, just update state
                        // But DO update txActivationTime so the timeout watchdog can detect stuck TX
                        ESP_LOGD(TAG, "IF: Radio TX detected but not stealing ownership (unsolicited AI response, owner=%d)",
                                 state.getTxOwner());
                        state.isTx.store(true);
                        // Update activation time if not already set, so timeout watchdog works
                        if (state.txActivationTime.load() == 0) {
                            state.txActivationTime.store(currentTime);
                            ESP_LOGD(TAG, "IF: Set txActivationTime for timeout watchdog (unsolicited TX)");
                        }
                    }
                } else {
                    // Radio reports RX - decide how to handle based on source and state
                    // The radio is the ultimate authority on TX/RX state
                    const bool fromRadio = (command.source == CommandSource::Remote);
                    const bool locallyTrackedAsTx = state.isTx.load();
                    const int currentOwner = state.getTxOwner();

                    // Force release TX ownership if:
                    // 1. Response came from radio, AND
                    // 2. We locally think we're in TX (state mismatch), AND
                    // 3. Either IF was queried, OR owner is Remote, OR no owner (orphaned state)
                    const bool shouldForceRelease = fromRadio && locallyTrackedAsTx &&
                                                   (ifWasQueried || currentOwner == static_cast<int>(CommandSource::Remote) || currentOwner == -1);

                    if (shouldForceRelease) {
                        ESP_LOGI(TAG, "IF: Radio RX detected, force releasing TX ownership (queried=%s, owner=%d)",
                                 ifWasQueried ? "yes" : "no", currentOwner);
                        state.forceReleaseTx(currentTime);

                        // Broadcast RX; to local interfaces so external controllers
                        // (e.g., Remoterig front-ends) release PTT state
                        const std::string rxMsg = buildCommand("RX");

                        if (const esp_err_t err = usbSerial.sendMessage(rxMsg); err != ESP_OK) {
                            // Only log non-backpressure errors (backpressure already logged by CdcSerialHandler)
                            if (err != ESP_ERR_NO_MEM) {
                                ESP_LOGW(TAG, "IF: Failed to echo RX; to USB0: %s", esp_err_to_name(err));
                            }
                        }

                        // Forward to display interface (if available)
                        radioManager.sendToDisplay(rxMsg);
                    } else if (fromRadio && locallyTrackedAsTx) {
                        // Radio says RX but we have a local owner - force release anyway
                        // The radio is authoritative, even if ownership tracking is inconsistent
                        ESP_LOGW(TAG, "IF: Radio RX but local owner %d exists (queried=%s) - forcing TX release",
                                 currentOwner, ifWasQueried ? "yes" : "no");
                        state.forceReleaseTx(currentTime);

                        // Broadcast RX; to local interfaces
                        const std::string rxMsg = buildCommand("RX");
                        if (const esp_err_t err = usbSerial.sendMessage(rxMsg); err != ESP_OK) {
                            if (err != ESP_ERR_NO_MEM) {
                                ESP_LOGW(TAG, "IF: Failed to echo RX; to USB0: %s", esp_err_to_name(err));
                            }
                        }
                        radioManager.sendToDisplay(rxMsg);
                    } else if (!radioTxState) {
                        // Normal case: update TX state to match radio
                        state.isTx.store(false);
                        state.txActivationTime.store(0);
                        ESP_LOGD(TAG, "IF: Radio RX confirmed, state updated (queried=%s, owner=%d)",
                                 ifWasQueried ? "yes" : "no", currentOwner);
                    }
                }
            }
            
            // P9: Operating mode
            if (std::string modeStr = getStringParam(command, 8, ""); !modeStr.empty()) {
                int mode = std::stoi(modeStr);
                radioManager.updateMode(mode);
            }
            
            // P10: VFO/Memory selection (0=VFO, 1=Memory)
            // No additional action required here; VFO selection is handled earlier.

            // P11: Scan status
            if (std::string scanStr = getStringParam(command, 10, ""); !scanStr.empty()) {
                int scanStatus = std::stoi(scanStr);
                state.scanStatus.store(scanStatus);
            }
            
            // P12: Split status
            if (std::string splitStr = getStringParam(command, 11, ""); !splitStr.empty()) {
                bool split = splitStr == "1";
                radioManager.updateSplitEnabled(split);
            }
            
            // P13: Tone status
            if (std::string toneStr = getStringParam(command, 12, ""); !toneStr.empty()) {
                int toneStatus = std::stoi(toneStr);
                state.toneState = toneStatus;
                state.toneStatus = toneStatus;
            }
            
            // P14: Tone frequency index
            if (std::string toneFreqStr = getStringParam(command, 13, ""); !toneFreqStr.empty()) {
                int toneFreq = std::stoi(toneFreqStr);
                state.toneFrequency = toneFreq;
            }
            
            // P15: Reserved/constant - no action needed
        }
        
        // Update cache timestamp when IF answer is received
        uint64_t currentTime = esp_timer_get_time();
        radioManager.getState().commandCache.update("IF", currentTime);
        
        // Handle transverter offset before routing using centralized helper
        std::string responseMessage = command.originalMessage;

        if (radioManager.isTransverterOffsetActive()) {
            // Parse frequency from original message and apply offset
            if (responseMessage.length() >= 13) { // "IF" + 11 digit frequency
                std::string freqStr = responseMessage.substr(2, 11);
                uint64_t baseFreq = 0;
                auto res = std::from_chars(freqStr.data(), freqStr.data() + 11, baseFreq);
                if (res.ec == std::errc{}) {
                    const uint64_t displayFreq = radioManager.baseToDisplayFrequency(baseFreq);
                    if (displayFreq != baseFreq) {
                        // Build frequency string directly to avoid stack-heavy std::ostringstream
                        std::string newFreqStr;
                        newFreqStr.reserve(11);
                        appendZeroPadded(newFreqStr, displayFreq, 11);

                        // Replace frequency in original message
                        responseMessage.replace(2, 11, newFreqStr);
                    }
                }
            }
        }

        // Use unified routing for IF answers (with transverter offset applied)
        routeAnswerResponse(command, responseMessage, usbSerial, radioManager);
        return true;
    }
    
    return false;
}


bool StatusInfoCommandHandler::handleID(const RadioCommand& cmd,
                                       ISerialChannel& radioSerial,
                                       ISerialChannel& usbSerial,
                                       const RadioManager& rm) {
    // ID: Radio identification
    ESP_LOGI("StatusInfo", "=== ID COMMAND HANDLER ===");
        ESP_LOGI("StatusInfo", "Type: %s, Source: %s",
                 cmd.type == CommandType::Read? "Read" : "Answer",
                 cmd.isLocal() ? "Local" : "Remote");

    const std::string idStr = CONFIG_RADIOCORE_ID_STRING;

    if (isQuery(cmd)) {
        // Return configured identifier immediately
        ESP_LOGI("StatusInfo", "ID query handled locally (not sent to radio)");
        const std::string response = buildCommand("ID", idStr);
        respondToSource(cmd, response, usbSerial, rm);
        ESP_LOGI("StatusInfo", "ID response sent to origin: %s", response.c_str());
        return true;
    }

    if (cmd.type == CommandType::Answer) {
        const std::string response = buildCommand("ID", idStr);
        routeAnswerResponse(cmd, response, usbSerial, rm);
        return true;
    }
    return false;
}

// =============================================================================
// Helper functions
// =============================================================================

std::string StatusInfoCommandHandler::formatSMeterResponse(int level) {
    // Clamp to valid range
    level = std::max(MIN_SMETER, std::min(MAX_SMETER, level));

    // Build SM response directly to avoid stack-heavy std::ostringstream
    std::string response;
    response.reserve(7); // "SM" + 4 digits + ";"
    response.append("SM");
    appendZeroPadded(response, static_cast<uint64_t>(level), 4);
    response.push_back(';');
    return response;
}

std::string StatusInfoCommandHandler::formatIFResponse(const RadioState& state) {
    std::string response;
    response.reserve(IF_RESPONSE_RESERVE);
    response.append("IF");

    auto selectVfoFrequency = [&state](int vfo) -> uint64_t {
        switch (vfo) {
            case 0:
                return state.vfoAFrequency.load();
            case 1:
                return state.vfoBFrequency.load();
            default:
                break;
        }
        if (const uint64_t rxFreq = state.vfoAFrequency.load(); rxFreq > 0) {
            return rxFreq;
        }
        return state.vfoBFrequency.load();
    };

    const bool isTx = state.isTx.load();
    const int rxVfo = state.currentRxVfo.load();
    const int txVfo = state.currentTxVfo.load();

    int activeVfo = isTx ? txVfo : rxVfo;
    uint64_t activeFrequency = selectVfoFrequency(activeVfo);

    if (activeFrequency == 0) {
        if (const uint64_t vfoAFreq = selectVfoFrequency(0); vfoAFreq > 0) {
            activeFrequency = vfoAFreq;
            activeVfo = 0;
        } else if (const uint64_t vfoBFreq = selectVfoFrequency(1); vfoBFreq > 0) {
            activeFrequency = vfoBFreq;
            activeVfo = 1;
        }
    }

    // Apply transverter offset for display if enabled (controlled by UIXD1/UIXD0)
    // NOTE: This mirrors RadioManager::baseToDisplayFrequency() logic - keep in sync
    const bool offsetActive = state.transverterOffsetEnabled && state.transverter && state.transverterOffsetHz > 0;
    if (offsetActive) {
        if (state.transverterOffsetPlus) {
            activeFrequency += state.transverterOffsetHz;
        } else {
            activeFrequency = (activeFrequency > state.transverterOffsetHz)
                ? (activeFrequency - state.transverterOffsetHz) : 0ULL;
        }
    }

    appendZeroPadded(response, activeFrequency, 11);

    response.append("     ");

    int ritOffset = state.ritXitOffset.load();
    if (ritOffset > 9990) {
        ritOffset = 9990;
    } else if (ritOffset < -9990) {
        ritOffset = -9990;
    }
    if (ritOffset >= 0) {
        response.push_back(' ');
        appendZeroPadded(response, static_cast<uint64_t>(ritOffset), 4);
    } else {
        response.push_back('-');
        appendZeroPadded(response, static_cast<uint64_t>(-ritOffset), 4);
    }

    response.push_back(state.ritOn.load() ? '1' : '0');
    response.push_back(state.xitOn.load() ? '1' : '0');

    int memChannel = state.memoryChannel.load();
    if (memChannel == 0) {
        memChannel = 22;
    }
    int memHundreds = memChannel / 100;
    if (memHundreds < 0) {
        memHundreds = 0;
    } else if (memHundreds > 9) {
        memHundreds = 9;
    }
    response.push_back(static_cast<char>('0' + memHundreds));

    int memTensOnes = memChannel % 100;
    if (memTensOnes < 0) {
        memTensOnes = 0;
    } else if (memTensOnes > 99) {
        memTensOnes = 99;
    }
    appendZeroPadded(response, static_cast<uint64_t>(memTensOnes), 2);

    response.push_back(isTx ? '1' : '0');

    appendNumber(response, static_cast<int>(state.mode.load()));

    if (activeVfo < 0 || activeVfo > 2) {
        activeVfo = 0;
    }
    response.push_back(static_cast<char>('0' + activeVfo));

    response.push_back('0');

    response.push_back(state.split.load() ? '1' : '0');

    appendNumber(response, state.toneState);

    int toneIdx = state.toneFrequency;
    if (toneIdx < 0) {
        toneIdx = 0;
    } else if (toneIdx > 99) {
        toneIdx = 99;
    }
    appendZeroPadded(response, static_cast<uint64_t>(toneIdx), 2);

    response.push_back('0');
    response.push_back(';');
    return response;
}

bool StatusInfoCommandHandler::handleBY(const RadioCommand& cmd,
                                      ISerialChannel& radioSerial,
                                      ISerialChannel& usbSerial,
                                      RadioManager& rm) const {
    // BY: BUSY status (Sky Command)
    if (isQuery(cmd)) {
        if (cmd.isCatClient()) {
            // Return busy status - typically 0 (not busy) for testing
            const auto response = buildCommand("BY", "00"); // P1=0 (not busy), P2=0 (reserved)
            respondToSource(cmd, response, usbSerial, rm);
        } else {
            const uint64_t timestamp = esp_timer_get_time();
            rm.getState().queryTracker.recordQuery("BY", timestamp);
            rm.noteQueryOrigin("BY", cmd.source, timestamp);
            sendToRadio(radioSerial, buildCommand("BY"));
        }
        return true;
    }
    
    if (cmd.type == CommandType::Answer) {
        if (rm.shouldForwardToUSB(cmd.originalMessage)) {
            // Forward the answer from radio to USB
            const int busy = getIntParam(cmd, 0, 0);
            const int reserved = getIntParam(cmd, 1, 0);
            const std::string params = std::to_string(busy) + std::to_string(reserved);
            const auto response = buildCommand("BY", params);
            respondToSource(cmd, response, usbSerial, rm);
        }
        return true;
    }
    
    return false;
}

bool StatusInfoCommandHandler::handleRI(const RadioCommand &command,
                                        ISerialChannel &radioSerial,
                                        ISerialChannel &usbSerial,
                                        RadioManager &radioManager) const {
    // RI: RX frequency and mode (read)
    if (isQuery(command)) {
        // For RI queries, respond from cached state when the source is Usb.
        // Only forward to radio when the query originated from the radio side.
        if (command.source == CommandSource::Remote) {
            sendToRadio(radioSerial, buildCommand("RI"));
        } else {
            // Build RI response from current state - avoid stack-heavy std::ostringstream
            const auto &state = radioManager.getState();
            std::string response;
            response.reserve(17); // "RI" + 11 freq + mode + datamode + "00" + ";"
            response.append("RI");

            // P1: RX frequency (11 digits in Hz)
            const uint64_t rxFreq = state.split.load() ? state.vfoBFrequency.load() : state.vfoAFrequency.load();
            appendZeroPadded(response, rxFreq, 11);

            // P2: RX mode (1=LSB, 2=USB, 3=CW, etc.)
            response += static_cast<char>('0' + static_cast<int>(state.mode.load()));

            // P3: Data mode (0=OFF, 1=ON)
            response += static_cast<char>('0' + (state.dataMode.load() ? 1 : 0));

            // P4: Data sub mode (00-03) - use 00 as default since dataSubMode not implemented
            response.append("00;");

            respondToSource(command, response, usbSerial, radioManager);
        }
        return true;
    }

    if (command.type == CommandType::Answer) {
        // Forward the answer via unified policy
        const std::string paramStr = getStringParam(command, 0, "");
        const std::string response = buildCommand("RI", paramStr);
        routeAnswerResponse(command, response, usbSerial, radioManager);

        // Update state from RI answer
        if (!command.params.empty()) {
            if (const std::string paramStr = getStringParam(command, 0, ""); paramStr.length() >= 11) {
                if (const uint64_t frequency = std::stoull(paramStr.substr(0, 11)); frequency > 0) {
                    // Update RX frequency (which VFO depends on split state)
                    if (const auto &state = radioManager.getState(); state.split.load()) {
                        radioManager.updateVfoBFrequency(frequency);
                    } else {
                        radioManager.updateVfoAFrequency(frequency);
                    }
                }

                if (paramStr.length() >= 12) {
                    const int mode = std::stoi(paramStr.substr(11, 1));
                    radioManager.updateMode(mode);
                }
            }
        }
        return true;
    }

    return false;
}

} // namespace radio
