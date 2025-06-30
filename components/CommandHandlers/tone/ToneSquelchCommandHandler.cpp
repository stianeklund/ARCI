#include "ToneSquelchCommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"

namespace radio {

ToneSquelchCommandHandler::ToneSquelchCommandHandler()
    : BaseCommandHandler({
        "TO", "TN", "CT", "SQ", "DQ", "QC", "CN"
    }, "Tone & Squelch Control Commands") {
}

bool ToneSquelchCommandHandler::handleCommand(const RadioCommand& command,
                                            ISerialChannel& radioSerial,
                                            ISerialChannel& usbSerial,
                                            RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling tone/squelch command: %s", command.describe().c_str());
    
    if (command.command == "TO") {
        return handleTO(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "TN") {
        return handleTN(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "CT") {
        return handleCT(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "CN") {
        // CN: CTCSS frequency number (alias of TN for some flows)
        if (isQuery(command)) {
            // Use same TTL as TN since they share state
            return handleLocalQueryStandard(
                command, radioSerial, usbSerial, radioManager,
                "CN", TTL_STATUS,
                [](const RadioState& st) {
                    return formatResponse2D("CN", static_cast<int>(st.toneFrequency));
                }
            );
        }

        if (isSet(command)) {
            int toneNumber = parseToneNumber(command);
            if (toneNumber < MIN_TONE_NUMBER || toneNumber > MAX_TONE_NUMBER) {
                ESP_LOGW(TAG, "Invalid tone number in CN command: %d", toneNumber);
                return false;
            }
            auto& state = radioManager.getState();
            state.toneFrequency = static_cast<uint8_t>(toneNumber);
            if (shouldSendToRadio(command)) {
                sendToRadio(radioSerial, formatResponse2D("CN", toneNumber));
            }
            return true;
        }

        if (command.type == CommandType::Answer) {
            int toneNumber = parseToneNumber(command);
            if (toneNumber >= MIN_TONE_NUMBER && toneNumber <= MAX_TONE_NUMBER) {
                auto& state = radioManager.getState();
                state.toneFrequency = static_cast<uint8_t>(toneNumber);
                routeAnswerResponse(command, formatResponse2D("CN", toneNumber), usbSerial, radioManager);
            }
            return true;
        }
        return false;
    }
    if (command.command == "SQ") {
        return handleSQ(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "DQ") {
        return handleDQ(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "QC") {
        return handleQC(command, radioSerial, usbSerial, radioManager);
    }

    return false;
}

bool ToneSquelchCommandHandler::handleTO(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // TO - Tone on/off
    if (isQuery(command)) {
        if (command.isUsb()) {
            // For local queries, check if cached data is fresh
            if (isCacheFresh(radioManager, "TO", TTL_STATUS)) {
                // Cache is fresh - respond with cached data immediately
                auto& state = radioManager.getState();
                int currentTone = state.toneState;
                std::string response = buildCommand("TO", std::to_string(currentTone));
                respondToSource(command, response, usbSerial, radioManager);
            } else {
                // Cache is stale - check if we have valid cached data
                auto& state = radioManager.getState();
                int currentTone = state.toneState;
                if (currentTone >= 0 && currentTone <= 2) {
                    // Have valid cached data - return immediately + background refresh
                    std::string response = buildCommand("TO", std::to_string(currentTone));
                    respondToSource(command, response, usbSerial, radioManager);
                    // Background refresh to update cache
                    radioManager.getState().queryTracker.recordQuery("TO", esp_timer_get_time());
                    sendToRadio(radioSerial, buildCommand("TO"));
                } else {
                    // No valid cached data - query radio first
                    radioManager.getState().queryTracker.recordQuery("TO", esp_timer_get_time());
                    sendToRadio(radioSerial, buildCommand("TO"));
                }
            }
        } else {
            // Non-local query - forward to radio
            radioManager.getState().queryTracker.recordQuery("TO", esp_timer_get_time());
            sendToRadio(radioSerial, buildCommand("TO"));
        }
        return true;
    }

    if (isSet(command)) {
        int toneState = parseOnOffValue(command);
        if (toneState < 0) {
            ESP_LOGW(TAG, "Invalid tone state in TO command");
            return false;
        }

        // Update state
        auto& state = radioManager.getState();
        state.toneState = toneState;
        
        // Update cache timestamp when state changes
        uint64_t currentTime = esp_timer_get_time();
        radioManager.getState().commandCache.update("TO", currentTime);

        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("TO", std::to_string(toneState));
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Tone %s", toneState ? "enabled" : "disabled");
        return true;
    }

    if (command.type == CommandType::Answer) {
        int toneState = parseOnOffValue(command);
        if (toneState >= 0) {
            // Update state from radio response
            auto& state = radioManager.getState();
            state.toneState = toneState;
            
            // Update cache timestamp for AI mode compatibility
            uint64_t currentTime = esp_timer_get_time();
            radioManager.getState().commandCache.update("TO", currentTime);
            
            ESP_LOGD(TAG, "Updated tone state from radio: %d", toneState);
            const std::string response = buildCommand("TO", std::to_string(toneState));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool ToneSquelchCommandHandler::handleTN(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // TN - Tone frequency number
    if (isQuery(command)) {
        // Tone frequency changes infrequently - use 5s TTL
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "TN", TTL_STATUS,
            [](const RadioState& st) {
                return formatResponse2D("TN", static_cast<int>(st.toneFrequency));
            }
        );
    }

    if (isSet(command)) {
        int toneNumber = parseToneNumber(command);
        if (toneNumber < MIN_TONE_NUMBER || toneNumber > MAX_TONE_NUMBER) {
            ESP_LOGW(TAG, "Invalid tone number in TN command: %d", toneNumber);
            return false;
        }

        // Update local state
        auto& state = radioManager.getState();
        state.toneFrequency = static_cast<uint8_t>(toneNumber);

        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse2D("TN", toneNumber));
        }

        ESP_LOGD(TAG, "Tone number set to %d", toneNumber);
        return true;
    }

    if (command.type == CommandType::Answer) {
        int toneNumber = parseToneNumber(command);
        if (toneNumber >= MIN_TONE_NUMBER && toneNumber <= MAX_TONE_NUMBER) {
            // Update state from radio response
            auto& state = radioManager.getState();
            state.toneFrequency = static_cast<uint8_t>(toneNumber);
            routeAnswerResponse(command, formatResponse2D("TN", toneNumber), usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool ToneSquelchCommandHandler::handleCT(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // CT - Tone control state (0/1/2), not frequency. Tests expect single-digit formatting.
    if (isQuery(command)) {
        // Do not send to radio or USB on local query (per tests)
        if (!command.isUsb()) {
            sendToRadio(radioSerial, buildCommand("CT"));
        }
        return true;
    }

    if (isSet(command)) {
        int stateVal = parseOnOffValue(command);
        if (stateVal < 0 || stateVal > 2) {
            ESP_LOGW(TAG, "Invalid CT state: %d", stateVal);
            return false;
        }
        
        // Update local state
        radioManager.getState().toneStatus = static_cast<int8_t>(stateVal);
        
        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("CT", std::to_string(stateVal));
            sendToRadio(radioSerial, cmdStr);
        }
        return true;
    }

    if (command.type == CommandType::Answer) {
        int stateVal = parseOnOffValue(command);
        if (stateVal >= 0 && stateVal <= 2) {
            std::string response = buildCommand("CT", std::to_string(stateVal));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool ToneSquelchCommandHandler::handleSQ(const RadioCommand& cmd,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& rm) const {
    // SQ - Squelch level
    if (isQuery(cmd)) {
        // Squelch changes occasionally - use 5s TTL
        return handleLocalQueryStandard(
            cmd, radioSerial, usbSerial, rm,
            "SQ", TTL_STATUS,
            [](const RadioState& st) {
                // Return default mid-level (state not tracked)
                return buildCommand("SQ", "0050");
            },
            "0"  // Query parameter required by SQ
        );
    }

    if (isSet(cmd)) {
        int squelchLevel = parseSquelchLevel(cmd);
        if (squelchLevel < MIN_SQUELCH_LEVEL || squelchLevel > MAX_SQUELCH_LEVEL) {
            ESP_LOGW(TAG, "Invalid squelch level in SQ command: %d", squelchLevel);
            return false;
        }

        if (shouldSendToRadio(cmd)) {
            sendToRadio(radioSerial, formatResponseSub3D("SQ", 0, squelchLevel));
        }

        ESP_LOGD(TAG, "Squelch level set to %d", squelchLevel);
        return true;
    }

    if (cmd.type == CommandType::Answer) {
        int squelchLevel = parseSquelchLevel(cmd);
        if (squelchLevel >= MIN_SQUELCH_LEVEL && squelchLevel <= MAX_SQUELCH_LEVEL) {
            routeAnswerResponse(cmd, formatResponseSub3D("SQ", 0, squelchLevel), usbSerial, rm);
        }
        return true;
    }

    return false;
}

bool ToneSquelchCommandHandler::handleDQ(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // DQ - DCS function on/off
    if (isQuery(command)) {
        // DCS state changes infrequently - use 5s TTL
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "DQ", TTL_STATUS,
            [](const RadioState& st) {
                // Return default off state (state not tracked)
                return buildCommand("DQ", "0");
            }
        );
    }

    if (isSet(command)) {
        int dcsState = parseOnOffValue(command);
        if (dcsState < 0) {
            ESP_LOGW(TAG, "Invalid DCS state in DQ command");
            return false;
        }

        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("DQ", std::to_string(dcsState));
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "DCS %s", dcsState ? "enabled" : "disabled");
        return true;
    }

    if (command.type == CommandType::Answer) {
        int dcsState = parseOnOffValue(command);
        if (dcsState >= 0) {
            const std::string response = buildCommand("DQ", std::to_string(dcsState));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool ToneSquelchCommandHandler::handleQC(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // QC - DCS code
    if (isQuery(command)) {
        // DCS code changes infrequently - use 5s TTL
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "QC", TTL_STATUS,
            [](const RadioState& st) {
                // Return default DCS 023 (state not tracked)
                return buildCommand("QC", "023");
            }
        );
    }

    if (isSet(command)) {
        int dcsCode = parseDcsCode(command);
        if (dcsCode < MIN_DCS_CODE || dcsCode > MAX_DCS_CODE) {
            ESP_LOGW(TAG, "Invalid DCS code in QC command: %d", dcsCode);
            return false;
        }

        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse3D("QC", dcsCode));
        }

        ESP_LOGD(TAG, "DCS code set to %d", dcsCode);
        return true;
    }

    if (command.type == CommandType::Answer) {
        int dcsCode = parseDcsCode(command);
        if (dcsCode >= MIN_DCS_CODE && dcsCode <= MAX_DCS_CODE) {
            routeAnswerResponse(command, formatResponse3D("QC", dcsCode), usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

int ToneSquelchCommandHandler::parseToneNumber(const RadioCommand& command) const {
    return getIntParam(command, 0, -1);
}

int ToneSquelchCommandHandler::parseSquelchLevel(const RadioCommand& command) const {
    return getIntParam(command, 0, -1);
}

int ToneSquelchCommandHandler::parseDcsCode(const RadioCommand& command) const {
    return getIntParam(command, 0, -1);
}

} // namespace radio
