#include "AntennaCommandHandler.h"
#include "RadioManager.h"
#include "RadioState.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace radio {

bool AntennaCommandHandler::handleCommand(const RadioCommand& command,
                                          ISerialChannel& radioSerial,
                                          ISerialChannel& usbSerial,
                                          RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling antenna command: %s", command.describe().c_str());
    
    if (command.command == "AC") {
        return handleAC(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "AN") {
        return handleAN(command, radioSerial, usbSerial, radioManager);
    }

    return false;
}

bool AntennaCommandHandler::handleAC(const RadioCommand& command,
                                     ISerialChannel& radioSerial,
                                     ISerialChannel& usbSerial,
                                     RadioManager& radioManager) {
    auto &state = radioManager.getState();

    // Handle query (AC;) - use base class standardized handling
    if (isQuery(command)) {
        return handleLocalQueryStandard(command, radioSerial, usbSerial, radioManager,
                                        "AC", TTL_STATUS,
                                        [this](const RadioState &s) { return formatACResponse(s); });
    }

    switch (command.type) {
        case CommandType::Set: {
            const auto [rxAt, txAt, tuning, valid] = parseACParams(command);
            if (!valid) {
                ESP_LOGW(TAG, "Invalid AC parameters");
                return false;
            }

            // Update local state
            state.rxAtIn = rxAt == 1;
            state.txAtIn = txAt == 1;
            state.atTuning = tuning == 1;
            state.commandCache.update("AC", esp_timer_get_time());

            ESP_LOGD(TAG, "Set AC: RX-AT=%s, TX-AT=%s, Tuning=%s",
                     rxAt ? "IN" : "THRU",
                     txAt ? "IN" : "THRU",
                     tuning ? "START" : "STOP");

            // Forward to radio if from local source
            if (command.shouldSendToRadio()) {
                const std::string cmdStr = "AC" + std::to_string(rxAt) +
                                           std::to_string(txAt) +
                                           std::to_string(tuning) + ";";
                radioSerial.sendMessage(cmdStr);
                ESP_LOGV(TAG, "Sent to radio: %s", cmdStr.c_str());
            }
            return true;
        }

        case CommandType::Read:
            // This case should not be reached with the new query pattern
            ESP_LOGW(TAG, "Unexpected CommandType::Read for AC");
            return false;

        case CommandType::Answer: {
            auto params = parseACParams(command);
            if (params.valid) {
                state.rxAtIn = params.rxAt == 1;
                state.txAtIn = params.txAt == 1;
                state.atTuning = params.tuning == 1;
                state.commandCache.update("AC", esp_timer_get_time());
                ESP_LOGD(TAG, "Updated AC from radio");
            }

            // Route via unified policy
            {
                const std::string response = formatACResponse(state);
                routeAnswerResponse(command, response, usbSerial, radioManager);
            }
            return true;
        }
        default: return false;
    }
}

bool AntennaCommandHandler::handleAN(const RadioCommand& command,
                                     ISerialChannel& radioSerial,
                                     ISerialChannel& usbSerial,
                                     RadioManager& radioManager) {
    auto& state = radioManager.getState();
    
    // Handle query (AN;) - use base class standardized handling
    if (isQuery(command)) {
        return handleLocalQueryStandard(command, radioSerial, usbSerial, radioManager,
                                        "AN", TTL_STATUS, 
                                        [this](const RadioState& s) { return formatANResponse(s); });
    }
    
    switch (command.type) {
        case CommandType::Set: {

            auto [mainAnt, rxAnt, drvOut, valid] = parseANParams(command);

            if (!valid) {
                ESP_LOGW(TAG, "Invalid AN parameters");
                return false;
            }
            
            // Update local state (9 means no change)
            if (mainAnt != 9) {
                state.mainAntenna = mainAnt; // Store antenna number directly
            }
            if (rxAnt != 9) {
                state.rxAnt = rxAnt == 1;
            }
            if (drvOut != 9) {
                state.drvOut = drvOut == 1;
            }
            state.commandCache.update("AN", esp_timer_get_time());
            
            ESP_LOGD(TAG, "Set AN: MainAnt=%d, RxAnt=%d, DrvOut=%d", mainAnt, rxAnt, drvOut);
            
            // Forward to radio if from local source
            if (command.shouldSendToRadio()) {
                const std::string cmdStr = "AN" + std::to_string(mainAnt) +
                                   std::to_string(rxAnt) +
                                   std::to_string(drvOut) + ";";

                radioSerial.sendMessage(cmdStr);
                ESP_LOGV(TAG, "Sent to radio: %s", cmdStr.c_str());
            }
            return true;
        }
        
        case CommandType::Read:
            // This case should not be reached with the new query pattern
            ESP_LOGW(TAG, "Unexpected CommandType::Read for AN");
            return false;
        
        case CommandType::Answer: {
            if (auto [mainAnt, rxAnt, drvOut, valid] = parseANParams(command); valid) {
                if (mainAnt != 9) {
                    state.mainAntenna = mainAnt; // Store antenna number directly
                }
                if (rxAnt != 9) {
                    state.rxAnt = rxAnt == 1;
                }
                if (drvOut != 9) {
                    state.drvOut = drvOut == 1;
                }
                state.commandCache.update("AN", esp_timer_get_time());
                ESP_LOGD(TAG, "Updated AN from radio");
            }
            
            // Route via unified policy
            {
                const std::string response = formatANResponse(state);
                routeAnswerResponse(command, response, usbSerial, radioManager);
            }
            return true;
        }
        default: return false;
    }
}


AntennaCommandHandler::ACParams AntennaCommandHandler::parseACParams(const RadioCommand& command) const {
    ACParams result = {0, 0, 0, false};
    
    if (command.params.empty()) {
        return result;
    }
    
    std::string paramStr;
    if (std::holds_alternative<std::string>(command.params[0])) {
        paramStr = std::get<std::string>(command.params[0]);
    } else if (std::holds_alternative<int>(command.params[0])) {
        paramStr = std::to_string(std::get<int>(command.params[0]));
    } else {
        return result;
    }
    
    // Handle both single digit and 3-digit formats
    if (paramStr.length() == 1) {
        // Single digit - interpret as "00X" format
        result.rxAt = 0;
        result.txAt = 0;
        result.tuning = paramStr[0] - '0';
    } else if (paramStr.length() == 3) {
        // 3-digit format: P1P2P3
        result.rxAt = paramStr[0] - '0';
        result.txAt = paramStr[1] - '0';  
        result.tuning = paramStr[2] - '0';
    } else {
        return result;
    }
    
    // Validate ranges
    if (result.rxAt >= 0 && result.rxAt <= 1 &&
        result.txAt >= 0 && result.txAt <= 1 &&
        result.tuning >= 0 && result.tuning <= 1) {
        result.valid = true;
    }
    
    return result;
}

AntennaCommandHandler::ANParams AntennaCommandHandler::parseANParams(const RadioCommand& command) const {
    ANParams result = {1, 0, 0, false};
    
    if (command.params.empty()) {
        return result;
    }
    
    std::string paramStr;
    if (std::holds_alternative<std::string>(command.params[0])) {
        paramStr = std::get<std::string>(command.params[0]);
    } else if (std::holds_alternative<int>(command.params[0])) {
        paramStr = std::to_string(std::get<int>(command.params[0]));
    } else {
        return result;
    }
    
    // Expect 3-digit format: P1P2P3
    if (paramStr.length() != 3) {
        return result;
    }
    
    // Parse each digit
    result.mainAnt = paramStr[0] - '0';
    result.rxAnt = paramStr[1] - '0';
    result.drvOut = paramStr[2] - '0';
    
    // Validate ranges (0-1 for main antenna per JSON spec, 0-1 or 9 for others)
    if (((result.mainAnt >= 0 && result.mainAnt <= 1) || result.mainAnt == 9) &&
        ((result.rxAnt >= 0 && result.rxAnt <= 1) || result.rxAnt == 9) &&
        ((result.drvOut >= 0 && result.drvOut <= 1) || result.drvOut == 9)) {
        result.valid = true;
    }
    
    return result;
}


std::string AntennaCommandHandler::formatACResponse(const RadioState& state) const {
    std::string result;
    result.reserve(6); // "AC" + 3 digits + ";"
    result.append("AC");
    result.push_back(state.rxAtIn ? '1' : '0');
    result.push_back(state.txAtIn ? '1' : '0');
    result.push_back(state.atTuning ? '1' : '0');
    result.push_back(';');
    return result;
}

std::string AntennaCommandHandler::formatANResponse(const RadioState& state) const {
    std::string result;
    result.reserve(6); // "AN" + 3 digits + ";"
    result.append("AN");
    result.push_back('0' + state.mainAntenna); // Direct antenna number (1 or 2)
    result.push_back(state.rxAnt ? '1' : '0');
    result.push_back(state.drvOut ? '1' : '0');
    result.push_back(';');
    return result;
}

} // namespace radio
