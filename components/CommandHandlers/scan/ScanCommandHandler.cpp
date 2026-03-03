#include "ScanCommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"
#include <iomanip>
#include <sstream>

namespace radio {

ScanCommandHandler::ScanCommandHandler()
    : BaseCommandHandler({
        "SC", "SS", "SU"
    }, "Scan & Sweep Commands") {
}

bool ScanCommandHandler::handleCommand(const RadioCommand& command,
                                     ISerialChannel& radioSerial,
                                     ISerialChannel& usbSerial,
                                     RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling scan command: %s", command.describe().c_str());
    
    if (command.command == "SC") {
        return handleSC(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "SS") {
        return handleSS(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "SU") {
        return handleSU(command, radioSerial, usbSerial, radioManager);
    }
    
    return false;
}

bool ScanCommandHandler::handleSC(const RadioCommand& command,
                                 ISerialChannel& radioSerial,
                                 ISerialChannel& usbSerial,
                                 RadioManager& radioManager) {
    // SC - Scan start/stop
    if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("SC", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("SC"));
        } else {
            // Return current scan state (0 = off, 1 = on)
            std::string response = "SC0;"; // Default to off
            respondToSource(command, response, usbSerial, radioManager);
        }
        return true;
    }

    if (isSet(command)) {
        int scanState = parseOnOffValue(command);
        if (scanState < 0) {
            ESP_LOGW(TAG, "Invalid scan state in SC command");
            return false;
        }

        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("SC", std::to_string(scanState));
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Scan %s", scanState ? "started" : "stopped");
        return true;
    }

    if (command.type == CommandType::Answer) {
        int scanState = parseOnOffValue(command);
        if (scanState >= 0) {
            std::string response = buildCommand("SC", std::to_string(scanState));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        } else {
            routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool ScanCommandHandler::handleSU(const RadioCommand& command,
                                 ISerialChannel& radioSerial,
                                 ISerialChannel& usbSerial,
                                 RadioManager& radioManager) {
    // SU - Scan group read/write
    if (isSet(command)) {
        if (shouldSendToRadio(command)) {
            // SU read format (SU0;) has P1 param, so parser classifies it as Set.
            // Record query so the response gets forwarded in AI0 mode.
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("SU", esp_timer_get_time());
            }
            std::string params = getStringParam(command, 0, "");
            std::string cmdStr = params.empty() ? std::string("SU;") : std::string("SU") + params + ";";
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Scan group command executed");
        return true;
    }
    if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("SU", esp_timer_get_time());
            }
            std::string params = getStringParam(command, 0, "");
            std::string cmdStr = params.empty() ? std::string("SU;") : std::string("SU") + params + ";";
            sendToRadio(radioSerial, cmdStr);
        }
        return true;
    }
    if (command.type == CommandType::Answer) {
        routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
        return true;
    }

    return false;
}

bool ScanCommandHandler::handleSS(const RadioCommand& command,
                                 ISerialChannel& radioSerial,
                                 ISerialChannel& usbSerial,
                                 RadioManager& radioManager) {
    // SS - Program Slow Scan Frequency (forward parameters as-is)
    if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("SS", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("SS"));
        } else {
            // No local state; default to 0
            respondToSource(command, buildCommand("SS", "000"), usbSerial, radioManager );
        }
        return true;
    }

    if (isSet(command)) {
        std::string params = getStringParam(command, 0, "");
        std::string cmdStr = params.empty() ? std::string("SS;") : std::string("SS") + params + ";";
        if (shouldSendToRadio(command)) {
            // SS read format (SS00;) has params, so parser classifies it as Set.
            // Record query so the response gets forwarded in AI0 mode.
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("SS", esp_timer_get_time());
            }
            sendToRadio(radioSerial, cmdStr);
        }
        return true;
    }

    if (command.type == CommandType::Answer) {
        routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
        return true;
    }

    return false;
}

} // namespace radio
