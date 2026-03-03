#include "ModeCommandHandler.h"
#include <array>
#include <string_view>
#include "RadioManager.h"
#include "esp_log.h"

namespace radio
{
    static constexpr std::array validModes = {
        false, true,  true,  true, true,  true, // 0-5: invalid, LSB, USB, CW, FM, AM
        true,  true,  true,  true, false, false, // 6-11: FSK, CW-R, PSK, FSK-R, invalid, invalid
        false, false, false, false // 12-15: all invalid
    };

    // Performance: Constexpr mode name lookup
    static constexpr std::array<std::string_view, 16> modeNames = {
        "INVALID", "LSB",   "USB",     "CW",      "FM",      "AM",      "FSK",     "CW-R",
        "PSK",     "FSK-R", "INVALID", "INVALID", "INVALID", "INVALID", "INVALID", "INVALID"};

    ModeCommandHandler::ModeCommandHandler() :
        BaseCommandHandler({"MD", "DA", "MK", "AS", "BU", "BD"}, "Mode & Band Control Commands")
    {
    }

    bool ModeCommandHandler::handleCommand(const RadioCommand &command, ISerialChannel &radioSerial,
                                           ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // Performance optimization: Use jump table for O(1) dispatch instead of cascading if-else
        // Most common commands (MD, DA) are checked first via likely branch prediction

        const std::string_view cmdView = command.command;

        // Fast path for most common commands
        if (cmdView == "MD")
        {
            return handleMD(command, radioSerial, usbSerial, radioManager);
        }
        if (cmdView == "DA")
        {
            return handleDA(command, radioSerial, usbSerial, radioManager);
        }

        // Search jump table for remaining commands
        for (const auto &entry : commandTable_)
        {
            if (entry.command.empty())
                break; // End of table

            if (cmdView == entry.command)
            {
                if (entry.fullHandler)
                {
                    return (this->*entry.fullHandler)(command, radioSerial, usbSerial, radioManager);
                }
                if (entry.simpleHandler)
                {
                    return (this->*entry.simpleHandler)(command, radioSerial, radioManager);
                }
            }
        }

        // Special cases that don't fit the pattern
        if (cmdView == "AS")
        {
            return handleAS(command, radioSerial, usbSerial, radioManager);
        }

        return false;
    }

    bool ModeCommandHandler::handleMD(const RadioCommand &command, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                                      RadioManager &radioManager) const
    {
        if (isQuery(command))
        {
            // MD; - Query current mode using standardized pattern
            return handleLocalQueryStandard(command, radioSerial, usbSerial, radioManager, "MD", TTL_MID_FREQ,
                                            [](const RadioState &st)
                                            {
                                                const int mode = st.mode.load();
                                                return buildCommand("MD", std::to_string(mode));
                                            });
        }

        if (isSet(command))
        {
            // MD[mode]; - Set operating mode
            if (command.params.empty())
            {
                ESP_LOGW(ModeCommandHandler::TAG, "MD set command missing mode parameter");
                return false;
            }

            if (!radioManager.acquirePrimaryControl(command.source))
            {
                ESP_LOGW(ModeCommandHandler::TAG, "MD control lease denied for source %d (owner=%d)",
                         static_cast<int>(command.source), radioManager.currentPrimaryControlOwner());

                if (command.isLocal())
                {
                    const int currentMode = radioManager.getMode();
                    const std::string response = buildCommand("MD", std::to_string(currentMode));
                    respondToSource(command, response, usbSerial, radioManager);
                }
                return true;
            }

            const int mode = getIntParam(command, 0, -1);
            if (!isValidMode(mode))
            {
                ESP_LOGW(ModeCommandHandler::TAG, "MD command has invalid mode: %d", mode);
                return false;
            }

            // Update the radio manager state
            radioManager.updateMode(mode);
            ESP_LOGD(ModeCommandHandler::TAG, "Mode set to: %s (%d)", getModeName(mode).data(), mode);

            // Always send to radio when requested, regardless of state change
            if (shouldSendToRadio(command))
            {
                const std::string cmd = buildCommand("MD", std::to_string(mode));
                sendToRadio(radioSerial, cmd);
            }

            return true;
        }

        if (command.type == CommandType::Answer)
        {
            // This is a response from the radio - update our state
            if (command.params.empty())
            {
                ESP_LOGW(ModeCommandHandler::TAG, "MD answer missing mode parameter");
                return false;
            }

            const int mode = getIntParam(command, 0, -1);
            if (!isValidMode(mode))
            {
                ESP_LOGW(ModeCommandHandler::TAG, "MD answer has invalid mode: %d", mode);
                return false;
            }

            // Update state
            radioManager.updateMode(mode);

            // Use unified routing for MD answers - preserve original formatting
            routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
            ESP_LOGD(ModeCommandHandler::TAG, "Mode updated from radio: %s (%d)", getModeName(mode).data(), mode);
            return true;
        }

        return false;
    }

    bool ModeCommandHandler::handleDA(const RadioCommand &command, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                                      RadioManager &radioManager) const
    {
        if (isQuery(command))
        {
            // Data mode changes infrequently - use 5s TTL like MD
            return handleLocalQueryStandard(command, radioSerial, usbSerial, radioManager, "DA", TTL_STATUS,
                                            [](const RadioState &st)
                                            {
                                                const int dataMode = st.dataMode.load();
                                                return buildCommand("DA", std::to_string(dataMode));
                                            });
        }

        if (isSet(command))
        {
            // DA[mode]; - Set data mode
            if (command.params.empty())
            {
                ESP_LOGW(ModeCommandHandler::TAG, "DA set command missing data mode parameter");
                return false;
            }

            const int dataMode = getIntParam(command, 0, -1);
            if (dataMode < 0 || dataMode > 1)
            {
                // 0=off, 1=on
                ESP_LOGW(ModeCommandHandler::TAG, "DA command has invalid data mode: %d", dataMode);
                return false;
            }

            // Update the radio manager state
            if (radioManager.updateDataMode(dataMode))
            {
                ESP_LOGD(ModeCommandHandler::TAG, "Data mode set to: %s", dataMode ? "ON" : "OFF");
                if (shouldSendToRadio(command))
                {
                    const std::string cmd = buildCommand("DA", std::to_string(dataMode));
                    sendToRadio(radioSerial, cmd);
                }
            }

            return true;
        }

        if (command.type == CommandType::Answer)
        {
            // This is a response from the radio - update our state
            if (command.params.empty())
            {
                ESP_LOGW(ModeCommandHandler::TAG, "DA answer missing data mode parameter");
                return false;
            }

            const int dataMode = getIntParam(command, 0, -1);
            if (dataMode < 0 || dataMode > 1)
            {
                ESP_LOGW(ModeCommandHandler::TAG, "DA answer has invalid data mode: %d", dataMode);
                return false;
            }

            // Update state
            radioManager.updateDataMode(dataMode);

            // Update cache timestamp for consistent behavior
            radioManager.getState().commandCache.update("DA", esp_timer_get_time());

            // Use unified routing for DA answers - preserve original formatting
            routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
            ESP_LOGD(ModeCommandHandler::TAG, "Data mode updated from radio: %s (cache updated)",
                     dataMode ? "ON" : "OFF");
            return true;
        }

        return false;
    }

    bool ModeCommandHandler::handleMK(const RadioCommand &command, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                                      RadioManager &radioManager) const
    {
        // MK: Mode key operation (virtual key press)

        if (isQuery(command))
        {
            return handleLocalQueryStandard(command, radioSerial, usbSerial, radioManager, "MK", TTL_DYNAMIC,
                                            [](const RadioState &st)
                                            {
                                                const int key = st.modeKey;
                                                return BaseCommandHandler::buildCommand("MK", std::to_string(key));
                                            });
        }

        if (isSet(command))
        {
            // Update local state if a numeric parameter is provided
            if (!command.params.empty())
            {
                if (const int key = getIntParam(command, 0, -1); key >= 0)
                {
                    radioManager.getState().modeKey = key;

                    // Update cache timestamp when state changes
                    const uint64_t currentTime = esp_timer_get_time();
                    radioManager.getState().commandCache.update("MK", currentTime);
                }
            }

            // Forward to radio if needed
            if (shouldSendToRadio(command))
            {
                std::string cmdStr = buildCommand("MK");
                if (!command.params.empty())
                {
                    cmdStr = buildCommand("MK", getStringParam(command, 0));
                }
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

        // Handle Answer commands from radio
        if (command.type == CommandType::Answer)
        {
            // Update state from radio response if we have a valid parameter
            if (!command.params.empty())
            {
                if (const int key = getIntParam(command, 0, -1); key >= 0)
                {
                    radioManager.getState().modeKey = key;

                    // Update cache timestamp for AI mode compatibility
                    const uint64_t currentTime = esp_timer_get_time();
                    radioManager.getState().commandCache.update("MK", currentTime);

                    ESP_LOGD(TAG, "Updated mode key from radio: %d", key);
                }
            }

            std::string response = buildCommand("MK");
            if (!command.params.empty())
            {
                response = buildCommand("MK", getStringParam(command, 0));
            }
            routeAnswerResponse(command, response, usbSerial, radioManager);
            return true;
        }

        return true;
    }

    bool ModeCommandHandler::handleAS(const RadioCommand &command, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                                      RadioManager &radioManager) const
    {
        // AS: Auto Mode channel entry
        // Format: ASP1P2P2P3P3P3P3P3P3P3P3P3P3P3P4P5;
        // P1=0 (const), P2=channel (00-31), P3=freq (11 digits), P4=mode (1-9), P5=data (0-1)

        if (isQuery(command))
        {
            // AS query - forward to radio
            if (shouldSendToRadio(command))
            {
                if (command.isCatClient())
                {
                    radioManager.getState().queryTracker.recordQuery("AS", esp_timer_get_time());
                }
                std::string cmdStr = buildCommand("AS");
                if (!command.params.empty())
                {
                    cmdStr = buildCommand("AS", getStringParam(command, 0));
                }
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

        if (isSet(command))
        {
            // AS command with multi-parameter parsing
            if (command.params.size() == 5)
            {
                // SET command: AS<P1><P2P2><P3...><P4><P5>;
                return handleASSet(command, radioSerial, usbSerial, radioManager);
            }
            if (command.params.size() == 2)
            {
                // QUERY command: AS<P1><P2P2>; - read format has params so parser classifies as Set.
                // Forward to radio and record query so response gets forwarded in AI0 mode.
                if (shouldSendToRadio(command))
                {
                    if (command.isCatClient())
                    {
                        radioManager.getState().queryTracker.recordQuery("AS", esp_timer_get_time());
                    }
                    const std::string params = getStringParam(command, 0) + getStringParam(command, 1);
                    const std::string cmdStr = buildCommand("AS", params);
                    sendToRadio(radioSerial, cmdStr);
                }
                return true;
            }
            ESP_LOGW(TAG, "AS command needs 2 (query) or 5 (set) parameters, got: %zu", command.params.size());
            return false;
        }

        if (command.type == CommandType::Answer)
        {
            // Route AS answers from radio to requesting interfaces
            routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
            return true;
        }

        return false;
    }

    bool ModeCommandHandler::handleASSet(const RadioCommand &command, ISerialChannel &radioSerial,
                                         ISerialChannel &usbSerial, RadioManager &radioManager) const
    {
        // P1: const (should be "0")
        const std::string p1 = getStringParam(command, 0);
        if (p1 != "0")
        {
            ESP_LOGW(TAG, "AS command P1 must be 0, got: %s", p1.c_str());
            return false;
        }

        // P2: channel (00-31)
        const int channel = std::stoi(getStringParam(command, 1));
        if (channel < 0 || channel > 31)
        {
            ESP_LOGW(TAG, "AS command invalid channel: %d", channel);
            return false;
        }

        // P3: frequency (11 digits in Hz)
        const uint64_t frequency = std::stoull(getStringParam(command, 2));

        // P4: mode (1-9)
        const int mode = std::stoi(getStringParam(command, 3));
        if (!isValidMode(mode))
        {
            ESP_LOGW(TAG, "AS command invalid mode: %d", mode);
            return false;
        }

        // P5: data mode flag (0-1)
        const int dataMode = std::stoi(getStringParam(command, 4));
        if (dataMode < 0 || dataMode > 1)
        {
            ESP_LOGW(TAG, "AS command invalid data mode: %d", dataMode);
            return false;
        }

        // Update state directly for auto mode channels
        auto &state = radioManager.getState();
        if (channel < 32)
        {
            state.autoModeChannels[channel].frequency.store(frequency);
            state.autoModeChannels[channel].mode.store(mode);
            state.autoModeChannels[channel].dataMode.store(dataMode == 1);
        }

        // Forward to radio if from local source
        if (shouldSendToRadio(command))
        {
            const std::string params = p1 + getStringParam(command, 1) + getStringParam(command, 2) +
                getStringParam(command, 3) + getStringParam(command, 4);
            const std::string cmdStr = buildCommand("AS", params);
            sendToRadio(radioSerial, cmdStr);
        }

        ESP_LOGD(TAG, "AS command: channel=%d, freq=%llu, mode=%d, data=%d", channel, frequency, mode, dataMode);

        return true;
    }

    bool ModeCommandHandler::handleBU(const RadioCommand &command, ISerialChannel &radioSerial,
                                      RadioManager &radioManager) const
    {
        // BU: Band select (up) - per TS-590SG specification, requires 2-digit band parameter
        auto &state = radioManager.getState();

        if (isSet(command))
        {
            // BU requires a band parameter per specification (format: BUP1P1;)
            if (command.params.empty())
            {
                ESP_LOGW(TAG, "BU command requires band parameter per TS-590SG specification");
                return false;
            }

            const int band = parseNumericValue(command);
            if (band < 0 || band > MAX_BAND)
            {
                ESP_LOGW(TAG, "Invalid BU band number: %d", band);
                return false;
            }

            // Forward to radio if from local source
            if (shouldSendToRadio(command))
            {
                sendToRadio(radioSerial, formatResponse2D("BU", band));
                ESP_LOGV(TAG, "Sent BU band command: %d", band);
            }

            // Update band state
            state.bandNumber.store(band, std::memory_order_relaxed);
            return true;
        }

        // BU does not support query mode per TS-590SG specification
        ESP_LOGW(TAG, "BU command only supports set operation with band parameter");
        return false;
    }

    bool ModeCommandHandler::handleBD(const RadioCommand &command, ISerialChannel &radioSerial,
                                      RadioManager &radioManager) const
    {
        // BD: Band select (down) - per TS-590SG specification, requires 2-digit band parameter
        auto &state = radioManager.getState();

        if (isSet(command))
        {
            // BD requires a band parameter per specification (format: BDP1P1;)
            if (command.params.empty())
            {
                ESP_LOGW(TAG, "BD command requires band parameter per TS-590SG specification");
                return false;
            }

            const int band = parseNumericValue(command);
            if (band < 0 || band > MAX_BAND)
            {
                ESP_LOGW(TAG, "Invalid BD band number: %d", band);
                return false;
            }

            // Forward to radio if from local source
            if (shouldSendToRadio(command))
            {
                sendToRadio(radioSerial, formatResponse2D("BD", band));
                ESP_LOGV(TAG, "Sent BD band command: %d", band);
            }

            // Update band state
            state.bandNumber.store(band, std::memory_order_relaxed);
            return true;
        }

        // BD does not support query mode per TS-590SG specification
        ESP_LOGW(TAG, "BD command only supports set operation with band parameter");
        return false;
    }

    bool ModeCommandHandler::isValidMode(const int mode)
    {
        // Performance: Use constexpr lookup table for O(1) validation
        return mode >= 0 && mode < static_cast<int>(validModes.size()) && validModes[mode];
    }

    bool ModeCommandHandler::isValidBand(const int band) { return band >= MIN_BAND && band <= MAX_BAND; }

    std::string_view ModeCommandHandler::getModeName(const int mode)
    {
        if (mode >= 0 && mode < static_cast<int>(modeNames.size()))
        {
            return modeNames[mode];
        }
        return "UNKNOWN";
    }

    int ModeCommandHandler::parseNumericValue(const RadioCommand &command) const { return getIntParam(command, 0, -1); }
} // namespace radio
