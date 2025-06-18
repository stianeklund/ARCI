#include "ReceiverProcessingCommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"
#include <sstream>  // Keep for debug logging only

namespace radio {

ReceiverProcessingCommandHandler::ReceiverProcessingCommandHandler()
    : BaseCommandHandler({
        "PA", "RA", "IS", "FL", "FW", "SH", "SL", "GC", "GT", 
        "NB", "NL", "NR", "RL", "NT", "BP", "BC"
    }, "Receiver Signal Processing Commands") {
}

bool ReceiverProcessingCommandHandler::handleCommand(const RadioCommand& command,
                                                   ISerialChannel& radioSerial,
                                                   ISerialChannel& usbSerial,
                                                   RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling receiver processing command: %s", command.command.c_str());
    
    // Commands moved from other handlers
    if (command.command == "PA") {
        return handlePA(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "RA") {
        return handleRA(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "IS") {
        return handleIS(command, radioSerial, usbSerial, radioManager);
    }
    // New DSP and receiver processing commands
    else if (command.command == "FL") {
        return handleFL(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "FW") {
        return handleFW(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "SH") {
        return handleSH(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "SL") {
        return handleSL(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "GC") {
        return handleGC(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "GT") {
        return handleGT(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "NB") {
        return handleNB(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "NL") {
        return handleNL(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "NR") {
        return handleNR(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "RL") {
        return handleRL(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "NT") {
        return handleNT(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "BP") {
        return handleBP(command, radioSerial, usbSerial, radioManager);
    } else if (command.command == "BC") {
        return handleBC(command, radioSerial, usbSerial, radioManager);
    }
    
    return false;
}

// =============================================================================
// Commands moved from AntennaCommandHandler
// =============================================================================

bool ReceiverProcessingCommandHandler::handlePA(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // PA (Preamp) command - simple on/off
    if (isSet(command)) {
        int onOff = parseOnOffValue(command);
        if (onOff < 0 || onOff > 1) {
            ESP_LOGW(TAG, "Invalid PA value: %d", onOff);
            return false;
        }

        // Update local state
        radioManager.getState().preAmplifier = onOff == 1;
        ESP_LOGD(TAG, "Set preamp %s", onOff ? "ON" : "OFF");
        
        // Forward to radio if from local source
        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("PA", std::to_string(onOff));
            sendToRadio(radioSerial, cmdStr);
            ESP_LOGV(TAG, "Sent to radio: %s", cmdStr.c_str());
        }
        return true;
    }
    
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return cached preamp state in answer format PAP1P2 (P2 always 0)
            bool preampState = radioManager.getState().preAmplifier;
            std::string response = buildCommand("PA", std::to_string(preampState ? 1 : 0) + "0");
            respondToSource(command, response, usbSerial, radioManager);
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("PA", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("PA"));
        }
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        int onOff = parseOnOffValue(command);
        if (onOff >= 0 && onOff <= 1) {
            // Update local state from radio response
            radioManager.getState().preAmplifier = onOff == 1;
            ESP_LOGD(TAG, "Updated preamp state from radio: %s", onOff ? "ON" : "OFF");
        }

        // Update cache timestamp when we receive answer from radio
        radioManager.getState().commandCache.update("PA", esp_timer_get_time());

        {
            // Forward answer in correct format PAP1P2 (P1=preamp state, P2=always 0)
            std::string response = buildCommand("PA", std::to_string(onOff) + "0");
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleRA(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    auto& state = radioManager.getState();
    
    if (isSet(command)) {
        int onOff = parseOnOffValue(command);
        if (onOff < 0 || onOff > 1) {
            ESP_LOGW(TAG, "Invalid RA value: %d", onOff);
            return false;
        }
        
        // Update local state
        state.attenuator = onOff == 1;
        ESP_LOGD(TAG, "Set attenuator %s", onOff ? "ON" : "OFF");
        
        // Forward to radio if from local source
        if (shouldSendToRadio(command)) {
            std::string cmdStr = std::string("RA") + (onOff ? "01" : "00") + ";";
            sendToRadio(radioSerial, cmdStr);
            ESP_LOGV(TAG, "Sent to radio: %s", cmdStr.c_str());
        }
        return true;
    }
    
    if (isQuery(command)) {
        // Always return cached value for RA queries (test compatibility)
        bool attenuator = state.attenuator;
        std::string response = buildCommand("RA", std::string(attenuator ? "01" : "00") + "00");
        respondToSource(command, response, usbSerial, radioManager);
        ESP_LOGV(TAG, "Sent RA query response: %s", response.c_str());
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        int onOff = parseOnOffValue(command);
        if (onOff >= 0 && onOff <= 1) {
            state.attenuator = onOff == 1;
            ESP_LOGD(TAG, "Updated attenuator from radio: %s", onOff ? "ON" : "OFF");
        }
        
        // Update cache timestamp when we receive answer from radio
        radioManager.getState().commandCache.update("RA", esp_timer_get_time());
        
        {
            std::string response = buildCommand("RA", std::string(onOff ? "01" : "00") + "00");
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }
    
    return false;
}

// =============================================================================
// Commands moved from GainCommandHandler (DSP related)
// =============================================================================

bool ReceiverProcessingCommandHandler::handleIS(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    auto& state = radioManager.getState();

    std::string paramsStr;
    if (!command.params.empty()) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < command.params.size(); ++i) {
            if (i > 0) oss << ", ";
            std::visit([&oss](const auto& value) {
                oss << value;
            }, command.params[i]);
        }
        oss << "]";
        paramsStr = oss.str();
    } else {
        paramsStr = "[]";
    }
    ESP_LOGI(TAG, "handleIS command params: %s", paramsStr.c_str());

    if (isSet(command)) {
        int shift = parseFrequencyValue(command);
        ESP_LOGI(TAG, "IF shift value %d", shift);

        if (!isValidIFShift(shift)) {
            ESP_LOGW(TAG, "Invalid IF shift value: %d", shift);
            return false;
        }
        
        // Update local state
        state.ifShiftValue.store(shift);
        ESP_LOGD(TAG, "Set IF shift to %d", shift);
        
        // Update cache timestamp
        radioManager.getState().commandCache.update("IS", esp_timer_get_time());
        
        // Forward to radio if from local source
        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse4D("IS ", std::abs(shift)));
            ESP_LOGV(TAG, "Sent IS to radio");
        }
        return true;
    }

    if (isQuery(command)) {
        int shift = state.ifShiftValue.load();
        std::string response = formatResponse4D("IS ", std::abs(shift));
        respondToSource(command, response, usbSerial, radioManager);
        ESP_LOGV(TAG, "Sent IS query response: %s", response.c_str());
        return true;
    }

    if (command.type == CommandType::Answer) {
        int shift = parseFrequencyValue(command);
        if (isValidIFShift(shift)) {
            state.ifShiftValue.store(shift);
            ESP_LOGD(TAG, "Updated IF shift from radio: %d", shift);
        }

        // Update cache timestamp when we receive answer from radio
        radioManager.getState().commandCache.update("IS", esp_timer_get_time());

        routeAnswerResponse(command, formatResponse4D("IS ", std::abs(shift)), usbSerial, radioManager);
        return true;
    }
    
    return false;
}

// =============================================================================
// New DSP and Receiver Processing Commands
// =============================================================================

bool ReceiverProcessingCommandHandler::handleFL(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // FL: IF filter select (A/B)
    // Spec: Set = FLP1; where P1 is 1 (A) or 2 (B)
    //       Read = FL;
    //       Answer = FLP1;
    auto& state = radioManager.getState();

    if (isQuery(command)) {
        if (command.isUsb()) {
            int current = state.ifFilter;
            if (current != 1 && current != 2) current = 1; // default to A
            std::string response = buildCommand("FL", std::to_string(current));
            respondToSource(command, response, usbSerial, radioManager);
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("FL", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("FL"));
        }
        return true;
    }

    if (isSet(command)) {
        int filter = getIntParam(command, 0, -1);
        if (filter < 1 || filter > 2) {
            ESP_LOGW(TAG, "Invalid FL filter value: %d", filter);
            return false;
        }

        // Update state
        state.ifFilter = filter;

        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("FL", std::to_string(filter));
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Set IF filter to %s", filter == 1 ? "A" : "B");
        return true;
    }

    if (command.type == CommandType::Answer) {
        int filter = getIntParam(command, 0, -1);
        if (filter >= 1 && filter <= 2) {
            state.ifFilter = filter;
        }
        
        // Update cache timestamp when we receive answer from radio
        radioManager.getState().commandCache.update("FL", esp_timer_get_time());
        
        {
            int out = (filter >= 1 && filter <= 2) ? filter : 1;
            std::string response = buildCommand("FL", std::to_string(out));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool ReceiverProcessingCommandHandler::handleFW(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // FW: DSP filter bandwidth (for CW/FSK)
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return default bandwidth
            respondToSource(command, buildCommand("FW", "500"), usbSerial, radioManager); // Default 500 Hz
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("FW", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("FW"));
        }
        return true;
    }
    
    if (isSet(command)) {
        int bandwidth = parseFrequencyValue(command);
        if (!isValidFilterBandwidth(bandwidth)) {
            ESP_LOGW(TAG, "Invalid FW bandwidth value: %d", bandwidth);
            return false;
        }
        
        // Update local state
        radioManager.getState().dspFilterBandwidth = bandwidth;
        
        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse4D("FW", bandwidth));
        }

        ESP_LOGD(TAG, "Set DSP filter bandwidth to %d Hz", bandwidth);
        return true;
    }

    if (command.type == CommandType::Answer) {
        int bandwidth = parseFrequencyValue(command);
        if (isValidFilterBandwidth(bandwidth)) {
            radioManager.getState().dspFilterBandwidth = bandwidth;
            ESP_LOGD(TAG, "Updated DSP filter bandwidth from radio: %d Hz", bandwidth);
        }

        // Update cache timestamp when we receive answer from radio
        radioManager.getState().commandCache.update("FW", esp_timer_get_time());

        routeAnswerResponse(command, formatResponse4D("FW", bandwidth), usbSerial, radioManager);
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleSH(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // SH: Receive high-cut / DSP Shift
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return current state value
            int currentValue = radioManager.getState().receiveHighCut;
            respondToSource(command, formatResponse2D("SH", currentValue), usbSerial, radioManager);
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("SH", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("SH"));
        }
        return true;
    }

    if (isSet(command)) {
        int value = getIntParam(command, 0, -1);
        if (value < 0 || value > 99) {
            ESP_LOGW(TAG, "Invalid SH value: %d", value);
            return false;
        }

        // Update local state
        radioManager.getState().receiveHighCut = value;

        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse2D("SH", value));
        }

        ESP_LOGD(TAG, "Set receive high-cut to %d", value);
        return true;
    }

    if (command.type == CommandType::Answer) {
        int value = getIntParam(command, 0, -1);
        if (value >= 0 && value <= 99) {
            radioManager.getState().receiveHighCut = value;
            ESP_LOGD(TAG, "Updated receive high-cut from radio: %d", value);

            // Update cache timestamp when we receive valid answer from radio
            radioManager.getState().commandCache.update("SH", esp_timer_get_time());

            routeAnswerResponse(command, formatResponse2D("SH", value), usbSerial, radioManager);
        } else {
            ESP_LOGW(TAG, "SH: Invalid value %d from radio, not forwarding", value);
        }
        return true;
    }

    return false;
}

bool ReceiverProcessingCommandHandler::handleSL(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // SL: Receive low-cut / DSP Width
    if (isQuery(command)) {
        // Return cached value for all local interfaces
        if (command.isLocal()) {
            int currentValue = radioManager.getState().receiveLowCut;
            respondToSource(command, formatResponse2D("SL", currentValue), usbSerial, radioManager);
            return true;
        }

        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, buildCommand("SL"));
        }
        return true;
    }

    if (isSet(command)) {
        int value = getIntParam(command, 0, -1);
        if (value < 0 || value > 99) {
            ESP_LOGW(TAG, "Invalid SL value: %d", value);
            return false;
        }

        // Update local state
        radioManager.getState().receiveLowCut = value;

        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse2D("SL", value));
        }

        ESP_LOGD(TAG, "Set receive low-cut to %d", value);
        return true;
    }

    if (command.type == CommandType::Answer) {
        int value = getIntParam(command, 0, -1);
        if (value >= 0 && value <= 99) {
            radioManager.getState().receiveLowCut = value;
            ESP_LOGD(TAG, "Updated receive low-cut from radio: %d", value);
            // Update cache timestamp when we receive valid answer from radio
            radioManager.getState().commandCache.update("SL", esp_timer_get_time());
            routeAnswerResponse(command, formatResponse2D("SL", value), usbSerial, radioManager);
        } else {
            ESP_LOGW(TAG, "SL: Invalid value %d from radio, not forwarding", value);
        }
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleGC(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // GC: AGC mode (Off, Slow, Fast)
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return default AGC mode (Fast)
            respondToSource(command, buildCommand("GC", "2"), usbSerial, radioManager);
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("GC", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("GC"));
        }
        return true;
    }
    
    if (isSet(command)) {
        int agcMode = parseOnOffValue(command);

        // Validate: 0-3 are valid, ≥4 causes error
        if (agcMode < AGC_OFF || agcMode > AGC_RESTORE) {
            ESP_LOGW(TAG, "Invalid GC AGC mode: %d (valid: 0-3)", agcMode);
            return false;
        }

        // Handle AGC_RESTORE (value 3): restore to previous Slow/Fast state
        if (agcMode == AGC_RESTORE) {
            int currentAgcMode = radioManager.getState().agcMode;

            if (currentAgcMode == AGC_OFF) {
                // AGC is OFF - restore to previous Slow/Fast state
                agcMode = radioManager.getState().previousAgcMode;
                ESP_LOGI(TAG, "GC3: Restoring AGC from OFF to previous mode %d (%s)",
                         agcMode, agcMode == AGC_SLOW ? "Slow" : "Fast");
            } else {
                // AGC already ON - value 3 has no effect
                ESP_LOGD(TAG, "GC3: AGC already ON (%d) - no effect, ignoring", currentAgcMode);
                return true;
            }
        }

        // Save previous state when turning AGC OFF (only if current state is Slow or Fast)
        int currentAgcMode = radioManager.getState().agcMode;
        if (agcMode == AGC_OFF && (currentAgcMode == AGC_SLOW || currentAgcMode == AGC_FAST)) {
            radioManager.getState().previousAgcMode = currentAgcMode;
            ESP_LOGD(TAG, "Saving previous AGC mode %d before turning OFF", currentAgcMode);
        }

        // Update state
        radioManager.getState().agcMode = agcMode;

        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("GC", std::to_string(agcMode));
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "Set AGC mode to %d", agcMode);
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        int agcMode = parseOnOffValue(command);

        // Track previous state when radio reports AGC turning OFF
        int currentAgcMode = radioManager.getState().agcMode;
        if (agcMode == AGC_OFF && (currentAgcMode == AGC_SLOW || currentAgcMode == AGC_FAST)) {
            radioManager.getState().previousAgcMode = currentAgcMode;
            ESP_LOGD(TAG, "Radio reported AGC OFF - saved previous mode %d", currentAgcMode);
        }

        // Update state
        radioManager.getState().agcMode = agcMode;

        std::string response = buildCommand("GC", std::to_string(agcMode));
        routeAnswerResponse(command, response, usbSerial, radioManager);
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleGT(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // GT: AGC time constant
    if (isQuery(command)) {
        // Return from state
        int val = radioManager.getState().agcTimeConstant;
        std::string response = buildCommand("GT", std::to_string(val));
        respondToSource(command, response, usbSerial, radioManager);
        return true;
    }

    if (isSet(command)) {
        int val = getIntParam(command, 0, -1);
        if (val < 0) {
            ESP_LOGW(TAG, "Invalid GT value");
            return false;
        }
        radioManager.getState().agcTimeConstant = val;
        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("GT", std::to_string(val));
            sendToRadio(radioSerial, cmdStr);
        }
        return true;
    }

    if (command.type == CommandType::Answer) {
        int val = getIntParam(command, 0, 0);
        radioManager.getState().agcTimeConstant = val;
        
        // Update cache timestamp when we receive answer from radio
        radioManager.getState().commandCache.update("GT", esp_timer_get_time());
        
        {
            std::string response = buildCommand("GT", std::to_string(val));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }

    return false;
}

bool ReceiverProcessingCommandHandler::handleNB(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // NB: Noise blanker (type)
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return default noise blanker (off)
            respondToSource(command, buildCommand("NB", "0"), usbSerial, radioManager);
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("NB", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("NB"));
        }
        return true;
    }
    
    if (isSet(command)) {
        const int nbType = parseOnOffValue(command);

        if (nbType < NB_OFF || nbType > NB3) {
            ESP_LOGW(TAG, "Invalid NB noise blanker type: %d", nbType);
            return false;
        }
        // Update state
        radioManager.getState().noiseBlanker = nbType;
        if (shouldSendToRadio(command)) {
            const std::string cmdStr = buildCommand("NB", std::to_string(nbType));
            sendToRadio(radioSerial, cmdStr);
        }
        
        ESP_LOGD(TAG, "Set noise blanker to %d", nbType);
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        int nbType = parseOnOffValue(command);
        
        // Update state from radio response
        radioManager.getState().noiseBlanker = nbType;
        
        // Update cache to prevent unnecessary polling
        radioManager.getState().commandCache.update("NB", esp_timer_get_time());
        
        {
            std::string response = buildCommand("NB", std::to_string(nbType));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        ESP_LOGD(TAG, "NB updated from radio: %d (cache updated)", nbType);
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleNL(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // NL: Noise blanker level
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return default noise blanker level
            respondToSource(command, buildCommand("NL", "050"), usbSerial, radioManager);
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("NL", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("NL"));
        }
        return true;
    }
    
    if (isSet(command)) {
        int level = parseLevel(command);
        if (level < MIN_LEVEL || level > MAX_LEVEL) {
            ESP_LOGW(TAG, "Invalid NL noise blanker level: %d", level);
            return false;
        }
        // Update state
        radioManager.getState().noiseBlankerLevel = level;
        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse3D("NL", level));
        }

        ESP_LOGD(TAG, "Set noise blanker level to %d", level);
        return true;
    }

    if (command.type == CommandType::Answer) {
        int level = parseLevel(command);
        routeAnswerResponse(command, formatResponse3D("NL", level), usbSerial, radioManager);
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleNR(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // NR: Noise Reduction mode (NR1/NR2)
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return default noise reduction (off)
            respondToSource(command, buildCommand("NR", "0"), usbSerial, radioManager);
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("NR", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("NR"));
        }
        return true;
    }
    
    if (isSet(command)) {
        int nrMode = parseOnOffValue(command);
        if (nrMode < NR_OFF || nrMode > NR2) {
            ESP_LOGW(TAG, "Invalid NR noise reduction mode: %d", nrMode);
            return false;
        }
        // Update state
        radioManager.getState().noiseReductionMode = nrMode;
        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("NR", std::to_string(nrMode));
            sendToRadio(radioSerial, cmdStr);
        }
        
        ESP_LOGD(TAG, "Set noise reduction to %d", nrMode);
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        {
            int nrMode = parseOnOffValue(command);
            std::string response = buildCommand("NR", std::to_string(nrMode));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleRL(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // RL: Noise Reduction level
    // RL command format: RLP1P1; where P1 is 2 digits (01-10 for NR1, 00-09 for NR2)
    if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("RL", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("RL"));
        } else {
            // Return default noise reduction level (2 digits per spec)
            respondToSource(command, buildCommand("RL", "05"), usbSerial, radioManager);
        }
        return true;
    }

    if (isSet(command)) {
        int level = parseLevel(command);
        if (level < MIN_LEVEL || level > MAX_LEVEL) {
            ESP_LOGW(TAG, "Invalid RL noise reduction level: %d", level);
            return false;
        }

        // Update local state - store in appropriate field based on NR mode
        // NR1 uses levels 1-10, NR2 uses SPAC speed 0-9
        auto& state = radioManager.getState();
        if (state.noiseReductionMode == 2) {
            state.nr2Speed = level;
        } else {
            state.nr1Level = level;
        }

        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse2D("RL", level));
        }

        ESP_LOGD(TAG, "Set noise reduction level to %d (NR%d)", level, state.noiseReductionMode);
        return true;
    }

    if (command.type == CommandType::Answer) {
        int level = parseLevel(command);
        // Store in appropriate field based on NR mode
        auto& state = radioManager.getState();
        if (state.noiseReductionMode == 2) {
            state.nr2Speed = level;
        } else {
            state.nr1Level = level;
        }
        routeAnswerResponse(command, formatResponse2D("RL", level), usbSerial, radioManager);
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleNT(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // NT: Notch Filter (Auto/Manual)
    // Spec variant in tests: two digits where first = mode (0=off,1=auto,2=manual), second = bandwidth (0=narrow,1=wide)
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return default notch filter (off)
            respondToSource(command, buildCommand("NT", "0"), usbSerial, radioManager);
        } else if (shouldSendToRadio(command)) {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("NT", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("NT"));
        }
        return true;
    }
    
    if (isSet(command)) {
        // Per JSON spec: P1 (mode) and P2 (bandwidth) are separate 1-digit params
        const int mode = getIntParam(command, 0, -1);
        const int bw = getIntParam(command, 1, 0);  // Default 0 if not specified

        if (mode < 0 || mode > 2) {
            ESP_LOGW(TAG, "Invalid NT notch mode: %d", mode);
            return false;
        }
        // Update state
        auto& state = radioManager.getState();
        state.notchFilterMode = mode;
        state.notchFilterBandwidth = bw;
        if (shouldSendToRadio(command)) {
            std::string params = std::to_string(mode) + std::to_string(bw);
            std::string cmdStr = buildCommand("NT", params);
            sendToRadio(radioSerial, cmdStr);
        }
        ESP_LOGD(TAG, "NT SET: mode=%d bw=%d", mode, bw);
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        // Per JSON spec: P1 (mode) and P2 (bandwidth) are separate 1-digit params
        const int mode = getIntParam(command, 0, 0);
        const int bw = getIntParam(command, 1, 0);

        // Update state from radio's answer
        auto& state = radioManager.getState();
        state.notchFilterMode = mode;
        state.notchFilterBandwidth = bw;
        ESP_LOGD(TAG, "NT answer: mode=%d bw=%d", mode, bw);

        const std::string response = buildCommand("NT", std::to_string(mode) + std::to_string(bw));
        routeAnswerResponse(command, response, usbSerial, radioManager);
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleBP(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // BP: Manual notch frequency
    // According to JSON spec: P1 is 3-digit number, min: 0, max: 127 (not frequency in Hz)
    auto& state = radioManager.getState();
    
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return current manual notch frequency setting directly to USB
            int notchValue = state.manualNotchFrequency;
            respondToSource(command, formatResponse3D("BP", notchValue), usbSerial, radioManager);
        } else {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("BP", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("BP"));
        }
        return true;
    }

    if (isSet(command)) {
        int notchValue = parseFrequencyValue(command);  // Using this parser but it's really a setting value 0-127
        if (notchValue < 0 || notchValue > 127) {
            ESP_LOGW(TAG, "Invalid BP notch setting: %d (must be 0-127)", notchValue);
            return false;
        }

        // Update state directly
        auto& state = radioManager.getState();
        state.manualNotchFrequency = notchValue;

        // Forward to radio if from local source
        if (shouldSendToRadio(command)) {
            sendToRadio(radioSerial, formatResponse3D("BP", notchValue));
        }

        ESP_LOGD(TAG, "Set manual notch frequency setting to %d", notchValue);
        return true;
    }

    if (command.type == CommandType::Answer) {
        // Update state from radio response
        int notchValue = parseFrequencyValue(command);
        if (notchValue >= 0 && notchValue <= 127) {
            state.manualNotchFrequency = notchValue;
            ESP_LOGD(TAG, "Updated manual notch from radio: %d", notchValue);
            routeAnswerResponse(command, formatResponse3D("BP", notchValue), usbSerial, radioManager);
        }
        return true;
    }
    
    return false;
}

bool ReceiverProcessingCommandHandler::handleBC(const RadioCommand& command,
                                               ISerialChannel& radioSerial,
                                               ISerialChannel& usbSerial,
                                               RadioManager& radioManager) {
    // BC: Beat Cancel status (NOT the band change command from MiscCommandHandler)
    // According to JSON spec: 0=BC OFF, 1=BC1 ON, 2=BC2 ON
    auto& state = radioManager.getState();
    
    if (isQuery(command)) {
        if (command.isUsb()) {
            // Return current beat cancel mode directly to USB
            int beatCancel = state.beatCancelMode;
            respondToSource(command, buildCommand("BC", std::to_string(beatCancel)), usbSerial, radioManager);
        } else {
            if (command.isUsb()) {
                radioManager.getState().queryTracker.recordQuery("BC", esp_timer_get_time());
            }
            sendToRadio(radioSerial, buildCommand("BC"));
        }
        return true;
    }
    
    if (isSet(command)) {
        int beatCancel = parseOnOffValue(command);
        if (beatCancel < 0 || beatCancel > 2) { // 0=OFF, 1=BC1, 2=BC2
            ESP_LOGW(TAG, "Invalid BC beat cancel value: %d", beatCancel);
            return false;
        }
        
        // Update state directly
        state.beatCancelMode = static_cast<int8_t>(beatCancel);
        
        // Forward to radio if from local source
        if (shouldSendToRadio(command)) {
            std::string cmdStr = buildCommand("BC", std::to_string(beatCancel));
            sendToRadio(radioSerial, cmdStr);
        }
        
        ESP_LOGD(TAG, "Set beat cancel to %d (%s)", beatCancel, 
                 beatCancel == 0 ? "OFF" : (beatCancel == 1 ? "BC1" : "BC2"));
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        // Update state from radio response
        int beatCancel = parseOnOffValue(command);
        if (beatCancel >= 0 && beatCancel <= 2) {
            state.beatCancelMode = static_cast<int8_t>(beatCancel);
            ESP_LOGD(TAG, "Updated beat cancel from radio: %d", beatCancel);
            const std::string response = buildCommand("BC", std::to_string(beatCancel));
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }
    
    return false;
}

// =============================================================================
// Helper Functions
// =============================================================================


int ReceiverProcessingCommandHandler::parseFrequencyValue(const RadioCommand& command) const {
    return getIntParam(command, 0, -1);
}

int ReceiverProcessingCommandHandler::parseLevel(const RadioCommand& command) const {
    return getIntParam(command, 0, -1);
}

bool ReceiverProcessingCommandHandler::isValidIFShift(int shift) const {
    return shift >= MIN_IF_SHIFT && shift <= MAX_IF_SHIFT;
}

bool ReceiverProcessingCommandHandler::isValidFilterBandwidth(int bandwidth) const {
    return bandwidth >= MIN_FILTER_BANDWIDTH && bandwidth <= MAX_FILTER_BANDWIDTH;
}

bool ReceiverProcessingCommandHandler::isValidNotchFrequency(int frequency) const {
    return frequency >= MIN_NOTCH_FREQ && frequency <= MAX_NOTCH_FREQ;
}

} // namespace radio
