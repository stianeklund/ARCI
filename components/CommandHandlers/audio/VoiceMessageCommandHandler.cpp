#include "VoiceMessageCommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"
#include <iomanip>
#include <sstream>

namespace radio {

VoiceMessageCommandHandler::VoiceMessageCommandHandler()
    : BaseCommandHandler({"LM", "PB", "VR", "ME"}, "Voice & Message Playback/Recording Commands") {
}

bool VoiceMessageCommandHandler::handleCommand(const RadioCommand& command,
                                             ISerialChannel& radioSerial,
                                             ISerialChannel& usbSerial,
                                             RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling voice/message command: %s", command.command.c_str());
    
    if (command.command == "LM") {
        return handleLM(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "PB") {
        return handlePB(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "VR") {
        return handleVR(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "ME") {
        // ME: Message status (not fully specified) — forward/query passthrough
        if (isQuery(command)) {
            respondToSource(command, buildCommand("ME", "0000"), usbSerial, radioManager);
            return true;
        }
        if (command.type == CommandType::Answer) {
            routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
            return true;
        }
        // For sets or others, just forward to radio if local
        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, command.originalMessage);
            return true;
        }
        return false;
    }

    return false;
}

bool VoiceMessageCommandHandler::handleLM(const RadioCommand& command,
                                         ISerialChannel& radioSerial,
                                         ISerialChannel& usbSerial,
                                         RadioManager& radioManager) const {
    // LM: VGS-1 recorder control/status
    if (isQuery(command)) {
        // Use standardized TTL caching for voice recorder status (5s TTL)
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "LM", TTL_STATUS,
            [](const RadioState& st) {
                // Format: target (P1) + status (P2)
                const int target = st.vgs1Target;
                const int status = st.vgs1Status;
                const std::string params = std::to_string(target) + std::to_string(status);
                return buildCommand("LM", params);
            }
        );
    }
    
    if (isSet(command)) {
        // LM set command expects 2 parameters: target (P1) and control (P2)
        if (command.params.size() < 2) {
            ESP_LOGW(TAG, "LM set command needs 2 parameters, got: %zu", command.params.size());
            return false;
        }

        const int target = std::stoi(getStringParam(command, 0));
        const int control = std::stoi(getStringParam(command, 1));
        
        // Validate parameters according to spec
        if (target < 0 || target > 5) {
            ESP_LOGW(TAG, "Invalid LM target: %d", target);
            return false;
        }
        
        if (control < 0 || control > 5) {
            ESP_LOGW(TAG, "Invalid LM control: %d", control);
            return false;
        }
        
        ESP_LOGD(TAG, "VGS-1 control: target=%d, control=%d", target, control);
        // Update state according to LM format: P1=target, P2=control/status
        auto& state = radioManager.getState();
        state.vgs1Target = target;
        state.vgs1Status = control;
        
        if (shouldSendToRadio(command)) {
            const auto cmdStr = buildCommand("LM", std::to_string(target) + std::to_string(control));
            sendToRadio(radioSerial, cmdStr);
        }
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        const int mode = getIntParam(command, 0, LM_STOP);
        const int channel = getIntParam(command, 1, 1);
        const auto response = buildCommand("LM", std::to_string(mode) + std::to_string(channel));
        routeAnswerResponse(command, response, usbSerial, radioManager);
        return true;
    }
    
    return false;
}

    bool VoiceMessageCommandHandler::handleVR(const RadioCommand &command,
                                              ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial,
                                              RadioManager &radioManager) const {
        // VR: Sets and reads the VOX gain (VGS-1 status/voice guide control)
        if (isQuery(command)) {
            // Voice guide state changes infrequently - use 5s TTL
            return handleLocalQueryStandard(
                command, radioSerial, usbSerial, radioManager,
                "VR", TTL_STATUS,
                [](const RadioState& st) {
                    // Return default voice guide state (off) - state not tracked
                    return buildCommand("VR", "0");
                }
            );
        }

        if (isSet(command)) {
            const int vgsState = getIntParam(command, 0, -1);
            if (!isValidVgsState(vgsState)) {
                ESP_LOGW(TAG, "Invalid VR voice guide state: %d", vgsState);
                return false;
            }

            const char *stateName = (vgsState == VR_OFF) ? "OFF" : (vgsState == VR_ON) ? "ON" : "ENHANCED";
            ESP_LOGD(TAG, "Voice guide control: %s", stateName);

            if (shouldSendToRadio(command)) {
                const auto cmdStr = buildCommand("VR", std::to_string(vgsState));
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

    if (command.type == CommandType::Answer) {
        const int vgsState = getIntParam(command, 0, VR_OFF);
        const auto response = buildCommand("VR", std::to_string(vgsState));
        routeAnswerResponse(command, response, usbSerial, radioManager);
        return true;
    }

        return false;
    }

// =============================================================================
// Helper functions
// =============================================================================

bool VoiceMessageCommandHandler::isValidMessageNumber(const int msgNum) {
    return msgNum >= MIN_MESSAGE && msgNum <= MAX_MESSAGE;
}

bool VoiceMessageCommandHandler::isValidPlaybackState(const int state) {
    return state >= PB_STOP && state <= PB_PLAY_MSG4;
}

bool VoiceMessageCommandHandler::isValidVgsState(const int state) {
    return state >= VR_OFF && state <= VR_ENHANCED;
}



bool VoiceMessageCommandHandler::handlePB(const RadioCommand& command,
                                         ISerialChannel& radioSerial,
                                         ISerialChannel& usbSerial,
                                         RadioManager& radioManager) const {
    // PB per ts590sg_cat_commands_v3.json:
    // set:    PBP1;   (P1=0 stop, 1-4 play CH1-CH4, 5=constant)
    // read:   PB;
    // answer: PBP2P3P4P5; where P2=channel(1-5), P3..P5=queue buffers (0-5)

        if (isQuery(command)) {
            // Playback status changes during operation - use 2s dynamic TTL
            return handleLocalQueryStandard(
                command, radioSerial, usbSerial, radioManager,
                "PB", TTL_DYNAMIC,
                [](const RadioState& st) {
                    // Compose from state; clamp to valid ranges
                    int ch = st.playbackChannel;
                    int q1 = st.playbackQueue[0];
                    int q2 = st.playbackQueue[1];
                    int q3 = st.playbackQueue[2];
                    if (ch < 0 || ch > 5) ch = 0;
                    if (q1 < 0 || q1 > 5) q1 = 0;
                    if (q2 < 0 || q2 > 5) q2 = 0;
                    if (q3 < 0 || q3 > 5) q3 = 0;
                    return std::string("PB") + static_cast<char>('0' + ch) +
                           static_cast<char>('0' + q1) + static_cast<char>('0' + q2) +
                           static_cast<char>('0' + q3) + ";";
                }
            );
        }

    if (isSet(command)) {
        const int action = getIntParam(command, 0, -1);
        if (action < PB_STOP || action > PB_PLAY_CONSTANT) {
            ESP_LOGW(TAG, "Invalid PB action: %d", action);
            return false;
        }
        // Forward to radio if local
        if (shouldSendToRadio(command)) {
            const auto cmdStr = buildCommand("PB", std::to_string(action));
            sendToRadio(radioSerial, cmdStr);
        }
        // Update local playbackChannel for non-zero actions
        auto& state = radioManager.getState();
        if (action == PB_STOP) {
            state.playbackChannel = 0;
        } else {
            // 1..4 or 5 (constant)
            state.playbackChannel = action;
        }
        return true;
    }

    if (command.type == CommandType::Answer) {
        // Extract the 4 digits P2..P5 as a single param; parser may give int or string
        std::string params = getStringParam(command, 0, "");
        // Defensive: ensure only digits and at least 1 char
        // Expected 4 chars, but some radios might return shorter; pad/truncate safely
        if (params.size() < 4) {
            // Try to reconstruct from ints if parser gave separate ints; fallback already handled by getStringParam
            // Pad with zeros
            while (params.size() < 4) params.push_back('0');
        }
        // Take first 4 characters
        params = params.substr(0, 4);
        int ch = params[0] - '0';
        int q1 = params[1] - '0';
        int q2 = params[2] - '0';
        int q3 = params[3] - '0';
        if (ch < 0 || ch > 5) ch = 0;
        if (q1 < 0 || q1 > 5) q1 = 0;
        if (q2 < 0 || q2 > 5) q2 = 0;
        if (q3 < 0 || q3 > 5) q3 = 0;

        auto& state = radioManager.getState();
        state.playbackChannel = ch;
        state.playbackQueue[0] = q1;
        state.playbackQueue[1] = q2;
        state.playbackQueue[2] = q3;

        const std::string response = std::string("PB") + static_cast<char>('0' + ch) + static_cast<char>('0' + q1) +
                                   static_cast<char>('0' + q2) +
                                   static_cast<char>('0' + q3) + ";";
        routeAnswerResponse(command, response, usbSerial, radioManager);
        return true;
        }

        return false;
    }
} // namespace radio
