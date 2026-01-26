#include "VisualScanCommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"

namespace radio {

VisualScanCommandHandler::VisualScanCommandHandler()
    : BaseCommandHandler({
        "VS"
    }, "Visual Scan Commands") {
}

bool VisualScanCommandHandler::handleCommand(const RadioCommand& command,
                                           ISerialChannel& radioSerial,
                                           ISerialChannel& usbSerial,
                                           RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling visual scan command: %s", command.describe().c_str());
    
    if (command.command == "VS") {
        return handleVS(command, radioSerial, usbSerial, radioManager);
    }
    
    return false;
}

bool VisualScanCommandHandler::handleVS(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // VS - Dispatch based on first parameter character (0..4), works for query/set/answer
    const std::string paramStr = getStringParam(command, 0, "");
    if (!paramStr.empty()) {
        
        // Handle VS0X format (e.g., "01" -> VS0 with status 1)
        if (paramStr.length() >= 2 && paramStr[0] == '0') {
            return handleVS0(command, radioSerial, usbSerial, radioManager);
        }
        
        // Handle single digit dispatch for other VS commands
        switch (paramStr[0]) {
            case '0': return handleVS0(command, radioSerial, usbSerial, radioManager);
            case '1': 
                // Special case: if this is an Answer and looks like VS0 status, route to VS0
                if (command.type == CommandType::Answer && paramStr.length() == 1) {
                    return handleVS0(command, radioSerial, usbSerial, radioManager);
                }
                return handleVS1(command, radioSerial, usbSerial, radioManager);
            case '2': 
                // Special case: if this is an Answer and looks like VS0 status, route to VS0
                if (command.type == CommandType::Answer && paramStr.length() == 1) {
                    return handleVS0(command, radioSerial, usbSerial, radioManager);
                }
                return handleVS2(command, radioSerial, usbSerial, radioManager);
            case '3': 
                // For VS3, this could be either VS0 status 3 or VS3 command
                if (command.type == CommandType::Answer && paramStr.length() == 1) {
                    return handleVS0(command, radioSerial, usbSerial, radioManager);
                }
                return handleVS3(command, radioSerial, usbSerial, radioManager);
            case '4': return handleVS4(command, radioSerial, usbSerial, radioManager);
            default: 
                break;
        }
    }
    // If no specific subcommand detected, treat as general VS answer/forward
    if (command.type == CommandType::Answer) {
        routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
        return true;
    }
    if (shouldSendToRadio(command)) {
        if (command.isCatClient()) {
            radioManager.getState().queryTracker.recordQuery("VS", esp_timer_get_time());
        }
        sendToRadio(radioSerial, command.originalMessage);
        return true;
    }
    return false;
}

bool VisualScanCommandHandler::handleVS0(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // VS0 - Visual Scan start/stop/pause status
    if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("VS0", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("VS0"));
        } else {
            std::string response = "VS00;"; // Default to OFF
            respondToSource(command, response, usbSerial, radioManager);
        }
        return true;
    }

    if (isSet(command)) {
        int status = parseVS0Status(command);
        if (status < VS0_OFF || status > VS0_RESTART) {
            ESP_LOGW(TAG, "Invalid VS0 status: %d", status);
            return false;
        }

        // Update local state
        radioManager.getState().visualScanStatus = status;

        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("VS0", std::to_string(status));
            sendToRadio(radioSerial, cmdStr);
        }

        const char* statusStr[] = {"OFF", "ON", "Pause", "Restart"};
        ESP_LOGD(TAG, "Visual scan status set to %s (%d)", statusStr[status], status);
        return true;
    }

    if (command.type == CommandType::Answer) {
        int status = parseVS0Status(command);
        if (status >= VS0_OFF && status <= VS0_RESTART) {
            // Update local state from radio response
            radioManager.getState().visualScanStatus = status;
            ESP_LOGD(TAG, "Updated visual scan status from radio: %d", status);
            
            // Route answer response
            std::string response = buildCommand("VS0", std::to_string(status));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool VisualScanCommandHandler::handleVS1(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // VS1 - Set Visual Scan center frequency (set only)
    if (isSet(command)) {
        uint64_t centerFreq = parseVS1Frequency(command);
        if (centerFreq == 0) {
            ESP_LOGW(TAG, "Invalid frequency in VS1 command");
            return false;
        }

        if (shouldSendToRadio(command)) {
            // Format: VS1 + 11 digit frequency in Hz, zero padded
            std::string cmdStr;
            cmdStr.reserve(16);
            cmdStr.append("VS1");
            constexpr uint64_t divisors[] = {
                10000000000ULL, 1000000000ULL, 100000000ULL, 10000000ULL, 1000000ULL,
                100000ULL, 10000ULL, 1000ULL, 100ULL, 10ULL, 1ULL
            };
            for (uint64_t div : divisors) {
                cmdStr.push_back('0' + static_cast<char>((centerFreq / div) % 10));
            }
            cmdStr.push_back(';');
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Visual scan center frequency set to %llu Hz", centerFreq);
        return true;
    }

    return false;
}

bool VisualScanCommandHandler::handleVS2(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // VS2 - Set Visual Scan span (set only)
    if (isSet(command)) {
        int span = parseVS2Span(command);
        if (span < VS2_MIN_SPAN || span > VS2_MAX_SPAN) {
            ESP_LOGW(TAG, "Invalid span in VS2 command: %d", span);
            return false;
        }

        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("VS2", std::to_string(span));
            sendToRadio(radioSerial, cmdStr);
        }

        const char* spanStr[] = {"20kHz", "50kHz", "100kHz", "200kHz", "500kHz", "1MHz", "2MHz"};
        ESP_LOGD(TAG, "Visual scan span set to %s (%d)", spanStr[span], span);
        return true;
    }

    return false;
}

bool VisualScanCommandHandler::handleVS3(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // VS3 - Read Visual Scan frequencies and span (read only)
    if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("VS3", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("VS3"));
        } else {
            // Default response: lower freq (11 digits) + center freq (11 digits) + upper freq (11 digits) + span (1 digit)
            std::string response = "VS3000000000000000000000000000000000000;"; // All zeros as default
            respondToSource(command, response, usbSerial, radioManager);
        }
        return true;
    }

    if (command.type == CommandType::Answer) {
        // Route VS3 answer responses - use the full command string from radio
        std::string fullResponse = buildCommand("VS3", getStringParam(command, 0, ""));
        routeAnswerResponse(command, fullResponse, usbSerial, radioManager);
        return true;
    }

    return false;
}

bool VisualScanCommandHandler::handleVS4(const RadioCommand& command,
                                        ISerialChannel& radioSerial,
                                        ISerialChannel& usbSerial,
                                        RadioManager& radioManager) {
    // VS4 - Read Visual Scan sweep frequency and signal level (read only)
    if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("VS4", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("VS4"));
        } else {
            // Default response: sweep frequency (11 digits) + signal level (4 digits)
            std::string response = "VS4000000000000000;"; // All zeros as default
            respondToSource(command, response, usbSerial, radioManager);
        }
        return true;
    }

    if (command.type == CommandType::Answer) {
        // Route VS4 answer responses - use the full command string from radio
        std::string fullResponse = buildCommand("VS4", getStringParam(command, 0, ""));
        routeAnswerResponse(command, fullResponse, usbSerial, radioManager);
        return true;
    }

    return false;
}

int VisualScanCommandHandler::parseVS0Status(const RadioCommand& command) const {
    // For VS0X commands, we need to extract X from the parameter
    // Handles both "01" format and "1" format (when parser converts to int)
    const std::string paramStr = getStringParam(command, 0, "");
    
    if (paramStr.empty()) {
        return -1;
    }
    
    // Case 1: VS0X format like "01" -> extract the second character
    if (paramStr.length() >= 2 && paramStr[0] == '0') {
        const char statusChar = paramStr[1];
        if (statusChar >= '0' && statusChar <= '3') {
            return statusChar - '0';
        }
    }
    
    // Case 2: Single digit format like "1" (parser converted "01" to int 1, then back to string "1")
    if (paramStr.length() == 1 && paramStr[0] >= '0' && paramStr[0] <= '3') {
        return paramStr[0] - '0';
    }
    
    // Fallback to regular int parsing
    return getIntParam(command, 0, -1);
}

uint64_t VisualScanCommandHandler::parseVS1Frequency(const RadioCommand& command) const {
    // VS1 takes an 11-digit frequency in Hz
    std::string params = getStringParam(command, 0, "");
    if (params.size() < 11) {
        return 0;
    }
    
    // Convert string to uint64_t manually without exceptions
    uint64_t result = 0;
    for (char c : params) {
        if (c < '0' || c > '9') {
            return 0; // Invalid character
        }
        result = result * 10 + (c - '0');
    }
    return result;
}

int VisualScanCommandHandler::parseVS2Span(const RadioCommand& command) const {
    return getIntParam(command, 0, -1);
}

} // namespace radio
