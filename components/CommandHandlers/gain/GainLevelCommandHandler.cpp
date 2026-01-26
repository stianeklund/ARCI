#include "GainLevelCommandHandler.h"
#include "RadioManager.h"
#include "ForwardingPolicy.h"
#include "esp_log.h"
#include <charconv>

namespace radio {

GainLevelCommandHandler::GainLevelCommandHandler()
    : BaseCommandHandler({"AG", "RG", "MG", "CG", "ML", "VG"}, "Gain & Level Control Commands") {
}

bool GainLevelCommandHandler::canHandle(const RadioCommand& command) const {
    const std::string& cmd = command.command;
    return cmd == "AG" || cmd == "RG" || cmd == "MG" || 
           cmd == "CG" || cmd == "ML" || cmd == "VG";
}

bool GainLevelCommandHandler::handleCommand(const RadioCommand& command,
                                          ISerialChannel& radioSerial,
                                          ISerialChannel& usbSerial,
                                          RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling gain/level command: %s", command.command.c_str());

    if (command.command == "AG") {
        return handleAG(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "RG") {
        return handleRG(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "MG") {
        return handleMG(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "CG") {
        return handleCG(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "ML") {
        return handleML(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "VG") {
        return handleVG(command, radioSerial, usbSerial, radioManager);
    }
    return false;
}

// =============================================================================
// Existing commands from GainCommandHandler
// =============================================================================

bool GainLevelCommandHandler::handleAG(const RadioCommand& command,
                                      ISerialChannel& radioSerial,
                                      ISerialChannel& usbSerial,
                                      RadioManager& radioManager) const {
    auto& state = radioManager.getState();
    
    // Debug AG command patterns - log all AG commands with values
    static uint32_t agCommandCount = 0;
    static uint64_t lastAgLogTime = 0;
    const uint64_t now = esp_timer_get_time();
    agCommandCount++;
    
    // Extract gain value for logging
    int gainValue = -1;
    if (command.type == CommandType::Set || command.type == CommandType::Answer) {
        gainValue = parseGainValue(command);
    }
    
    if (now - lastAgLogTime > 1000000) { // Log every second
        ESP_LOGV(TAG, "🔍 AG DEBUG: Received %u AG commands in last second, type=%s, source=%s",
                 agCommandCount,
                 (command.type == CommandType::Set ? "SET" :
                  command.type == CommandType::Read ? "READ" : "ANSWER"),
                 command.isUsb() ? "USB" : (command.isTcp() ? "TCP" : "RADIO"));
        agCommandCount = 0;
        lastAgLogTime = now;
    }
    
    // Log individual AG commands with values for detailed analysis
    if (command.type == CommandType::Set || command.type == CommandType::Answer) {
        ESP_LOGV(TAG, "🎚️ AG %s: gain=%d, source=%s, originalMsg='%.*s'",
                 (command.type == CommandType::Set ? "SET" : "ANSWER"),
                 gainValue,
                 command.isUsb() ? "USB" : (command.isTcp() ? "TCP" : "RADIO"),
                 static_cast<int>(command.originalMessage.length()), command.originalMessage.data());
    }
    
    // Also log READ commands to trace the flow
    if (command.type == CommandType::Read) {
        ESP_LOGV(TAG, "🎚️ AG READ: source=%s, originalMsg='%.*s'",
                 command.isUsb() ? "USB" : (command.isTcp() ? "TCP" : "RADIO"),
                 static_cast<int>(command.originalMessage.length()), command.originalMessage.data());
    }
    
    if (isSet(command)) {
        const int gain = parseGainValue(command);
        if (!isValidGainValue(gain)) {
            ESP_LOGW(TAG, "Invalid AF gain value: %d", gain);
            return false;
        }

        // Update local state
        state.afGain.store(gain);
        state.commandCache.update("AG", esp_timer_get_time());
        ESP_LOGV(TAG, "Set AF gain to %d", gain);

        // Forward to radio if from local source
        if (shouldSendToRadio(command)) {
            const auto cmdStr = formatResponseSub3D("AG", 0, gain);
            sendToRadio(radioSerial, cmdStr);
            ESP_LOGV(TAG, "Sent to radio: %s", cmdStr.c_str());
        }
        
        // Route AG SET commands to AI-enabled interfaces (especially for potentiometer changes)
        const std::string agMsg = formatResponseSub3D("AG", 0, gain);
        routeSetCommandToAIInterfaces(command, agMsg, usbSerial, radioManager);
        ESP_LOGD(TAG, "Routed AG SET to AI interfaces: %s", agMsg.c_str());
        
        return true;
    }

    if (isQuery(command)) {
        // AG; - Query AF gain using standardized pattern
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "AG", TTL_STATUS,
            [this](const RadioState& st) {
                const int gain = st.afGain.load();
                return formatResponseSub3D("AG", 0, gain);
            },
            "0"  // AG read parameter: AG0; instead of AG;
        );
    }

    if (command.type == CommandType::Answer) {
        // Check if we're actively adjusting AF gain via potentiometer
        const uint64_t now = esp_timer_get_time();
        const uint64_t adjustTime = state.afGainAdjustTime.load();
        constexpr uint64_t ADJUST_TIMEOUT_US = 500000; // 500ms timeout
        
        if (state.isAdjustingAfGain.load() && (now - adjustTime) < ADJUST_TIMEOUT_US) {
            ESP_LOGD(TAG, "Skipping AG answer update during potentiometer adjustment (prevents flicker)");
            // Clear flag after timeout
            if ((now - adjustTime) >= ADJUST_TIMEOUT_US) {
                state.isAdjustingAfGain.store(false);
            }
            return true; // Command was handled, just not processed
        }
        
        const int gain = parseGainValue(command);
        if (isValidGainValue(gain)) {
            state.afGain.store(gain);
            state.commandCache.update("AG", esp_timer_get_time());
            ESP_LOGD(TAG, "Updated AF gain from radio: %d", gain);
        }

        // Route via unified policy
        {
            const auto response = formatResponseSub3D("AG", 0, gain);
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool GainLevelCommandHandler::handleRG(const RadioCommand& command,
                                      ISerialChannel& radioSerial,
                                      ISerialChannel& usbSerial,
                                      RadioManager& radioManager) const {
    auto& state = radioManager.getState();

    if (isSet(command)) {
        const int gain = parseGainValue(command);
        if (!isValidGainValue(gain)) {
            ESP_LOGW(TAG, "Invalid RF gain value: %d", gain);
            return false;
        }

        // Update local state and cache timestamp
        state.rfGain.store(gain);
        state.commandCache.update("RG", esp_timer_get_time());
        ESP_LOGV(TAG, "HandleRG: Set RF gain to %d", gain);

        // Forward to radio if from local source
        if (shouldSendToRadio(command)) {
            const auto cmdStr = formatResponse3D("RG", gain);
            sendToRadio(radioSerial, cmdStr);
            ESP_LOGV(TAG, "Sent to radio: %s", cmdStr.c_str());
        }
        
        // Route RG SET commands to AI-enabled interfaces (especially for potentiometer changes)
        const std::string rgMsg = formatResponse3D("RG", gain);
        routeSetCommandToAIInterfaces(command, rgMsg, usbSerial, radioManager);
        ESP_LOGD(TAG, "Routed RG SET to AI interfaces: %s", rgMsg.c_str());
        
        return true;
    }

    if (isQuery(command)) {
        // RG; - Query RF gain using standardized pattern
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "RG", TTL_STATUS,
            [this](const RadioState& st) {
                const int gain = st.rfGain.load();
                return formatResponse3D("RG", gain);
            }
        );
    }

    if (command.type == CommandType::Answer) {
        // Check if we're actively adjusting RF gain via potentiometer
        const uint64_t now = esp_timer_get_time();
        const uint64_t adjustTime = state.rfGainAdjustTime.load();
        constexpr uint64_t ADJUST_TIMEOUT_US = 500000; // 500ms timeout
        
        if (state.isAdjustingRfGain.load() && (now - adjustTime) < ADJUST_TIMEOUT_US) {
            ESP_LOGD(TAG, "Skipping RG answer update during potentiometer adjustment (prevents flicker)");
            // Clear flag after timeout
            if ((now - adjustTime) >= ADJUST_TIMEOUT_US) {
                state.isAdjustingRfGain.store(false);
            }
            return true; // Command was handled, just not processed
        }
        
        const int gain = parseGainValue(command);
        if (isValidGainValue(gain)) {
            state.rfGain.store(gain);
            state.commandCache.update("RG", esp_timer_get_time());
            ESP_LOGV(TAG, "Updated RF gain from radio: %d", gain);
        }

        // Route via unified policy - preserve original formatting for consistency
        routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
        return true;
    }

    return false;
}



// =============================================================================
// New commands
// =============================================================================

bool GainLevelCommandHandler::handleMG(const RadioCommand& command,
                                      ISerialChannel& radioSerial,
                                      ISerialChannel& usbSerial,
                                      RadioManager& radioManager) const {
    // MG: Microphone gain
    if (isQuery(command)) {
        return handleLocalQueryStandard(
            command, radioSerial, usbSerial, radioManager,
            "MG", TTL_STATUS,
            [this](const RadioState& st) {
                int gain = st.microphoneGain;
                if (gain < 0) {
                    gain = 0;
                }
                if (gain > 999) {
                    gain = 999;
                }
                return formatResponse3D("MG", gain);
            }
        );
    }

    if (isSet(command)) {
        const int gain = parseGainValue(command);
        if (!isValidGainValue(gain)) {
            ESP_LOGW(TAG, "Invalid microphone gain value: %d", gain);
            return false;
        }

        // Update local state
        radioManager.getState().microphoneGain = gain;

        // Update cache timestamp when state changes
        const uint64_t currentTime = esp_timer_get_time();
        radioManager.getState().commandCache.update("MG", currentTime);

        if (shouldSendToRadio(command)) {
            const auto cmdStr = formatResponse3D("MG", gain);
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Set microphone gain to %d", gain);
        return true;
    }

    if (command.type == CommandType::Answer) {
        const int gain = parseGainValue(command);
        if (isValidGainValue(gain)) {
            // Update state from radio response
            radioManager.getState().microphoneGain = gain;

            // Update cache timestamp for AI mode compatibility
            const uint64_t currentTime = esp_timer_get_time();
            radioManager.getState().commandCache.update("MG", currentTime);

            ESP_LOGD(TAG, "Updated microphone gain from radio: %d", gain);
        }

        // Route via unified policy
        {
            const auto response = formatResponse3D("MG", gain);
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool GainLevelCommandHandler::handleCG(const RadioCommand& command,
                                      ISerialChannel& radioSerial,
                                      ISerialChannel& usbSerial,
                                      RadioManager& radioManager) {
    // CG: Carrier level (for AM/FM modes)
    if (isQuery(command)) {
        if (command.isCatClient()) {
            // Return current or default carrier level directly to USB/TCP
            const int level = radioManager.getState().carrierLevel;
            const auto response = formatResponse3D("CG", level);
            respondToSource(command, response, usbSerial, radioManager);
        } else {
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("CG", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("CG"));
        }
        return true;
    }

    if (isSet(command)) {
        const int level = parseGainValue(command);
        if (level < MIN_CARRIER_LEVEL || level > MAX_CARRIER_LEVEL) {
            ESP_LOGW(TAG, "Invalid carrier level value: %d", level);
            return false;
        }

        // Update local state
        radioManager.getState().carrierLevel = level;
        radioManager.getState().commandCache.update("CG", esp_timer_get_time());

        if (shouldSendToRadio(command)) {
            const auto cmdStr = formatResponse3D("CG", level);
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Set carrier level to %d", level);
        return true;
    }

    if (command.type == CommandType::Answer) {
        const int level = parseGainValue(command);
        // Update local state from radio response
        radioManager.getState().carrierLevel = level;

        // Route via unified policy
        {
            const auto response = formatResponse3D("CG", level);
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool GainLevelCommandHandler::handleML(const RadioCommand& command,
                                      ISerialChannel& radioSerial,
                                      ISerialChannel& usbSerial,
                                      RadioManager& radioManager) {
    // ML: TX Monitor output level
    if (isQuery(command)) {
        if (command.isCatClient()) {
            // For local queries, check if cached data is fresh
            if (isCacheFresh(radioManager, "ML", TTL_STATUS)) {
                // Cache is fresh - respond with cached data immediately
                const int level = radioManager.getState().txMonitorLevel;
                const auto response = formatResponse3D("ML", level);
                respondToSource(command, response, usbSerial, radioManager);
            } else {
                // Cache is stale - check if we have valid cached data
                const int level = radioManager.getState().txMonitorLevel;
                if (level >= 0 && level <= 100) {
                    // Have valid cached data - return immediately + background refresh
                    const auto response = formatResponse3D("ML", level);
                    respondToSource(command, response, usbSerial, radioManager);
                    // Background refresh to update cache
                    radioManager.getState().queryTracker.recordQuery("ML", esp_timer_get_time());
                    sendToRadio(radioSerial, buildCommand("ML"));
                } else {
                    // No valid cached data - query radio first
                    radioManager.getState().queryTracker.recordQuery("ML", esp_timer_get_time());
                    sendToRadio(radioSerial, buildCommand("ML"));
                }
            }
        } else {
            // Non-local query - forward to radio
            radioManager.getState().queryTracker.recordQuery("ML", esp_timer_get_time());
            sendToRadio(radioSerial, buildCommand("ML"));
        }
        return true;
    }

    if (isSet(command)) {
        const int level = parseGainValue(command);
        if (!isValidGainValue(level)) {
            ESP_LOGW(TAG, "Invalid monitor level value: %d", level);
            return false;
        }

        // Update local state
        radioManager.getState().txMonitorLevel = level;

        // Update cache timestamp when state changes
        const uint64_t currentTime = esp_timer_get_time();
        radioManager.getState().commandCache.update("ML", currentTime);

        if (shouldSendToRadio(command)) {
            const auto cmdStr = formatResponse3D("ML", level);
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Set monitor level to %d", level);
        return true;
    }

    if (command.type == CommandType::Answer) {
        const int level = parseGainValue(command);
        if (isValidGainValue(level)) {
            // Update state from radio response
            radioManager.getState().txMonitorLevel = level;

            // Update cache timestamp for AI mode compatibility
            const uint64_t currentTime = esp_timer_get_time();
            radioManager.getState().commandCache.update("ML", currentTime);

            ESP_LOGD(TAG, "Updated monitor level from radio: %d", level);
        }

        // Route via unified policy
        {
            const auto response = formatResponse3D("ML", level);
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}
    bool GainLevelCommandHandler::handleVG(const RadioCommand& command,
                                         ISerialChannel& radioSerial,
                                         ISerialChannel& usbSerial,
                                         RadioManager& radioManager) {
        // VG: VOX gain
        if (isQuery(command)) {
            if (shouldSendToRadio(command)) {
                if (command.isCatClient()) {
                    radioManager.getState().queryTracker.recordQuery("VG", esp_timer_get_time());
                }
                sendToRadio(radioSerial, buildCommand("VG"));
            } else {
                // Return default VOX gain
                respondToSource(command, buildCommand("VG", "005"), usbSerial, radioManager); // Mid-range default
            }
            return true;
        }

        if (isSet(command)) {
            const int gain = getIntParam(command, 2, 5);
            if (!isValidVoxLevel(gain)) {
                ESP_LOGW(TAG, "Invalid VG VOX gain: %d", gain);
                return false;
            }

            // Update cache timestamp
            radioManager.getState().commandCache.update("VG", esp_timer_get_time());

            if (shouldSendToRadio(command)) {
                const std::string cmdStr = buildCommand("VG", std::to_string(gain));
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "Set VOX gain to %d", gain);
            return true;
        }

        if (command.type == CommandType::Answer) {
        {
            const int gain = getIntParam(command, 2, 5);
            const std::string response = buildCommand("VG", std::to_string(gain));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

        return false;
    }

// =============================================================================
// Helper functions
// =============================================================================

int GainLevelCommandHandler::parseGainValue(const RadioCommand& command) const {
    return getIntParam(command, 0, -1);
}


bool GainLevelCommandHandler::isValidGainValue(int value) const {
    return value >= MIN_GAIN && value <= MAX_GAIN;
}



bool GainLevelCommandHandler::handlePC(const RadioCommand& command,
                                      ISerialChannel& radioSerial,
                                      ISerialChannel& usbSerial,
                                      RadioManager& radioManager) {
    // PC: Output power control
    auto& state = radioManager.getState();

    if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            if (command.isCatClient()) {
                radioManager.getState().queryTracker.recordQuery("PC", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("PC"));
        } else {
            // Return current power level from state
            const int power = state.transmitPower;
            const auto response = formatResponse3D("PC", power);
            respondToSource(command, response, usbSerial, radioManager);
        }
        return true;
    }

    if (isSet(command)) {
        const int power = parseGainValue(command);
        if (!isValidPowerLevel(power)) {
            ESP_LOGW(TAG, "Invalid PC power level: %d", power);
            return false;
        }

        // Update local state
        state.transmitPower = power;
        state.commandCache.update("PC", esp_timer_get_time());

        if (shouldSendToRadio(command)) {
            const auto cmdStr = formatResponse3D("PC", power);
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Set output power to %d watts", power);
        return true;
    }

    if (command.type == CommandType::Answer) {
        const int power = parseGainValue(command);
        if (isValidPowerLevel(power)) {
            state.transmitPower = power;
            ESP_LOGD(TAG, "Updated power level from radio: %d", power);
        }

        {
            const auto response = formatResponse3D("PC", power);
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }
    
    return false;
}
    bool GainLevelCommandHandler::isValidVoxLevel(const int level) const {
    return level >= MIN_VOX_LEVEL && level <= MAX_VOX_LEVEL;
}

bool GainLevelCommandHandler::isValidPowerLevel(int power) const {
    return power >= 0 && power <= 100; // 0-100 watts for TS-590SG
}

}// namespace radio
