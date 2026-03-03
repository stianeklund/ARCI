#include "AudioEqualizerCommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"
#include <iomanip>
#include <sstream>

namespace radio {
    AudioEqualizerCommandHandler::AudioEqualizerCommandHandler()
        : BaseCommandHandler({
                                 "EQ", "UR", "UT"
                             }, "Audio Processing & Equalizer Commands") {
    }

    bool AudioEqualizerCommandHandler::handleCommand(const RadioCommand &command,
                                                     ISerialChannel &radioSerial,
                                                     ISerialChannel &usbSerial,
                                                     RadioManager &radioManager) {
        ESP_LOGV(TAG, "Handling audio/equalizer command: %s", command.describe().c_str());

        if (command.command == "EQ") {
            return handleEQ(command, radioSerial, usbSerial, radioManager);
        }
        if (command.command == "UR") {
            return handleUR(command, radioSerial, usbSerial, radioManager);
        }
        if (command.command == "UT") {
            return handleUT(command, radioSerial, usbSerial, radioManager);
        }

        return false;
    }

    bool AudioEqualizerCommandHandler::handleEQ(const RadioCommand &command,
                                                ISerialChannel &radioSerial,
                                                ISerialChannel &usbSerial,
                                                RadioManager &radioManager) const {
        // EQ - Audio equalizer mode
        if (isQuery(command)) {
            // Use standardized TTL caching for audio config (5s TTL)
            return handleLocalQueryStandard(
                command, radioSerial, usbSerial, radioManager,
                "EQ", TTL_STATUS,
                [](const RadioState& st) {
                    const int eqMode = st.rxEqualizer;
                    return buildCommand("EQ", std::to_string(eqMode));
                }
            );
        }

        if (isSet(command)) {
            // Accept string parameters per spec; tests expect raw forwarding and minimal state update
            const std::string param = getStringParam(command, 0, "");
            // Update a simple RX EQ indicator using last digit if available
            if (!param.empty() && std::isdigit(param.back())) {
                const int rxEq = param.back() - '0';
                radioManager.getState().rxEqualizer = rxEq;
            }
            radioManager.getState().commandCache.update("EQ", esp_timer_get_time());
            if (shouldSendToRadio(command)) {
                // EQ read format (EQ00;) has params, so parser classifies it as Set.
                // Record query so the response gets forwarded in AI0 mode.
                if (command.isCatClient()) {
                    radioManager.getState().queryTracker.recordQuery("EQ", esp_timer_get_time());
                }
                const auto cmdStr = buildCommand("EQ", param);
                sendToRadio(radioSerial, cmdStr);
            }
            ESP_LOGD(TAG, "Equalizer set param=%s", param.c_str());
            return true;
        }

        if (command.type == CommandType::Answer) {
            const int eqMode = parseEqualizerMode(command);
            if (eqMode >= MIN_EQ_MODE && eqMode <= MAX_EQ_MODE) {
                radioManager.getState().commandCache.update("EQ", esp_timer_get_time());
                const auto response = buildCommand("EQ", std::to_string(eqMode));
                routeAnswerResponse(command, response, usbSerial, radioManager);
            } else {
                // Still route original if unparsable
                routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
            }
            return true;
        }

        return false;
    }

    bool AudioEqualizerCommandHandler::handleUR(const RadioCommand &command,
                                                ISerialChannel &radioSerial,
                                                ISerialChannel &usbSerial,
                                                RadioManager &radioManager) const {
        // UR - RX equalizer (18-band user levels)
            if (isQuery(command)) {
            if (shouldSendToRadio(command)) {
                radioManager.getState().queryTracker.recordQuery("UR", esp_timer_get_time());
                sendToRadio(radioSerial, buildCommand("UR"));
            } else {
                // Default response: all bands set to 06 (0 dB)
                std::string defaultLevels = "060606060606060606060606060606060606060606060606060606060606060606060606";
                const auto response = buildCommand("UR", defaultLevels);
                respondToSource(command, response, usbSerial, radioManager);
            }
            return true;
        }

        if (isSet(command)) {
            std::string bands = parseEqualizerBands(command);
            if (bands.length() != 36) {  // 18 bands × 2 digits each = 36 characters
                ESP_LOGW(TAG, "Invalid UR equalizer band data length: %zu", bands.length());
                return false;
            }

            if (shouldSendToRadio(command)) {
                const auto cmdStr = buildCommand("UR", bands);
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "RX equalizer bands set");
            return true;
        }

        if (command.type == CommandType::Answer) {
            // Use unified routing for UR answers
            std::string bands = parseEqualizerBands(command);
            if (bands.length() == 36) {
                const auto response = buildCommand("UR", bands);
                routeAnswerResponse(command, response, usbSerial, radioManager);
            }
            return true;
        }

        return false;
    }

    bool AudioEqualizerCommandHandler::handleUT(const RadioCommand &command,
                                                ISerialChannel &radioSerial,
                                                ISerialChannel &usbSerial,
                                                RadioManager &radioManager) const {
        // UT - TX equalizer (18-band user levels)
            if (isQuery(command)) {
            if (shouldSendToRadio(command)) {
                radioManager.getState().queryTracker.recordQuery("UT", esp_timer_get_time());
                sendToRadio(radioSerial, buildCommand("UT"));
            } else {
                // Default response: all bands set to 06 (0 dB)
                std::string defaultLevels = "060606060606060606060606060606060606060606060606060606060606060606060606";
                const auto response = buildCommand("UT", defaultLevels);
                respondToSource(command, response, usbSerial, radioManager);
            }
            return true;
        }

        if (isSet(command)) {
            std::string bands = parseEqualizerBands(command);
            if (bands.length() != 36) {  // 18 bands × 2 digits each = 36 characters
                ESP_LOGW(TAG, "Invalid UT equalizer band data length: %zu", bands.length());
                return false;
            }

            if (shouldSendToRadio(command)) {
                const auto cmdStr = buildCommand("UT", bands);
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "TX equalizer bands set");
            return true;
        }

        if (command.type == CommandType::Answer) {
            // Use unified routing for UT answers
            std::string bands = parseEqualizerBands(command);
            if (bands.length() == 36) {
                const auto response = buildCommand("UT", bands);
                routeAnswerResponse(command, response, usbSerial, radioManager);
            }
            return true;
        }

        return false;
    }

    int AudioEqualizerCommandHandler::parseEqualizerMode(const RadioCommand &command) const {
        return getIntParam(command, 0, -1);
    }

    std::string AudioEqualizerCommandHandler::parseEqualizerBands(const RadioCommand &command) const {
        // For UR/UT commands, the parameters are a 36-character string of 18 bands (2 digits each)
        return getStringParam(command, 0, "");
    }
} // namespace radio
