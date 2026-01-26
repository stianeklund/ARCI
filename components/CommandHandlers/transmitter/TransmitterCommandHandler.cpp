#include "TransmitterCommandHandler.h"
#include <charconv>
#include "ForwardingPolicy.h"
#include "RadioManager.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace radio
{

    static const char *sourceToString(CommandSource src)
    {
        switch (src)
        {
        case CommandSource::UsbCdc0:
            return "USB0";
        case CommandSource::UsbCdc1:
            return "USB1";
        case CommandSource::Tcp0:
            return "TCP0";
        case CommandSource::Tcp1:
            return "TCP1";
        case CommandSource::Display:
            return "Display";
        case CommandSource::Panel:
            return "Panel";
        case CommandSource::Macro:
            return "Macro";
        case CommandSource::Remote:
            return "Remote";
        default:
            return "Unknown";
        }
    }

    TransmitterCommandHandler::TransmitterCommandHandler() :
        BaseCommandHandler({"TX", "RX", "PC", "TP", "PR", "PL", "VX", "VD"}, "Transmitter Control Commands")
    {
    }

    bool TransmitterCommandHandler::handleCommand(const RadioCommand &command, ISerialChannel &radioSerial,
                                                  ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        ESP_LOGV(TAG, "Handling transmitter command: %s", command.command.c_str());

        // TX/RX control commands (merged from TxRxControlCommandHandler)
        if (command.command == "TX")
        {
            return handleTX(command, radioSerial, usbSerial, radioManager);
        }
        else if (command.command == "RX")
        {
            return handleRX(command, radioSerial, usbSerial, radioManager);
        }
        // Power and processor commands
        // PS moved to InterfaceSystemCommandHandler
        // Other transmitter control commands
        else if (command.command == "PC")
        {
            return handlePC(command, radioSerial, usbSerial, radioManager);
        }
        else if (command.command == "TP")
        {
            return handleTP(command, radioSerial, usbSerial, radioManager);
        }
        else if (command.command == "PR")
        {
            return handlePR(command, radioSerial, usbSerial, radioManager);
        }
        else if (command.command == "PL")
        {
            return handlePL(command, radioSerial, usbSerial, radioManager);
        }
        else if (command.command == "VX")
        {
            return handleVX(command, radioSerial, usbSerial, radioManager);
        }
        else if (command.command == "VD")
        {
            return handleVD(command, radioSerial, usbSerial, radioManager);
        }

        return false;
    }

    bool TransmitterCommandHandler::handlePC(const RadioCommand &command, ISerialChannel &radioSerial,
                                             ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // PC: Output power control (0-100)
        auto &state = radioManager.getState();

        if (isQuery(command))
        {
            // PC; - Query output power using standardized pattern
            return handleLocalQueryStandard(command, radioSerial, usbSerial, radioManager, "PC", TTL_STATUS,
                                            [this](const RadioState &st)
                                            {
                                                const int power = st.transmitPower;
                                                return formatResponse3D("PC", power);
                                            });
        }

        if (isSet(command))
        {
            int power = getIntParam(command, 0, -1);
            if (!isValidPowerLevel(power))
            {
                ESP_LOGW(TAG, "Invalid PC power level: %d", power);
                return false;
            }
            state.transmitPower = power;
            if (shouldSendToRadio(command))
            {
                auto cmdStr = formatResponse3D("PC", power);
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            int power = getIntParam(command, 0, -1);
            if (isValidPowerLevel(power))
            {
                state.transmitPower = power;
                // Update cache timestamp when we receive answer from radio
                state.commandCache.update("PC", esp_timer_get_time());
            }
            // Use unified routing for PC answers - preserve original formatting
            routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
            return true;
        }

        return false;
    }

    // =============================================================================
    // Commands moved from other handlers
    // =============================================================================

    bool TransmitterCommandHandler::handleTX(const RadioCommand &command, ISerialChannel &radioSerial,
                                             ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // TX: Transmit control
        auto &state = radioManager.getState();

        if (isQuery(command))
        {
            // TX is a SET-only command per CAT spec - should not have Query path
            // Use standardized pattern for any query attempts (though TX; should be classified as SET)
            return handleLocalQueryStandard(command, radioSerial, usbSerial, radioManager, "TX", TTL_STATUS,
                                            [this](const RadioState &st)
                                            {
                                                int txMode = TX_MODE_SEND; // Default to SEND mode (not RX!)
                                                if (st.isTx.load())
                                                {
                                                    txMode = st.isTuning.load() ? TX_MODE_TUNE : TX_MODE_SEND;
                                                }
                                                return formatTXResponse(txMode);
                                            });
        }

        if (isSet(command))
        {
            // Per CAT spec: TX; with no parameter defaults to 0 (SEND/TX mode)
            int mode = getIntParam(command, 0, 0);
            ESP_LOGI(TAG, "🔧 TX SET: Setting TX mode to %d (source: %s, cmd: %s)", mode,
                     sourceToString(command.source), command.originalMessage.c_str());

            if (!isValidTxMode(mode))
            {
                ESP_LOGW(TAG, "Invalid TX mode: %d", mode);
                return false;
            }

            // Try to acquire TX ownership
            const uint64_t currentTime = esp_timer_get_time();

            // Panel (physical MOX button) can always force TX acquisition
            // This is a safety feature - physical controls override software state
            if (command.source == CommandSource::Panel)
            {
                const int previousOwner = state.getTxOwner();
                if (previousOwner != -1 && previousOwner != static_cast<int>(CommandSource::Panel))
                {
                    ESP_LOGI(TAG, "🔘 Panel TX: Physical button forces TX acquisition (previous owner: %d)", previousOwner);
                    state.forceReleaseTx(currentTime);
                }
                if (!state.tryAcquireTx(command.source, currentTime))
                {
                    ESP_LOGE(TAG, "Panel TX acquisition failed unexpectedly after force release");
                    return false;
                }
            }
            else if (!state.tryAcquireTx(command.source, currentTime))
            {
                ESP_LOGW(TAG, "❌ TX DENIED: Source %s cannot acquire TX (owned by source %d)",
                         sourceToString(command.source), state.getTxOwner());
                return false;
            }

            // Update state based on mode (TX ownership acquired successfully)
            switch (mode)
            {
            case TX_MODE_SEND:
                state.isTuning.store(false);
                ESP_LOGI(TAG, "✅ TX STATE: Set to SEND (MIC) mode by %s", sourceToString(command.source));
                break;

            case TX_MODE_DATA_SEND:
                state.isTuning.store(false);
                ESP_LOGI(TAG, "✅ TX STATE: Set to DATA SEND (ACC2/USB) mode by %s", sourceToString(command.source));
                break;

            case TX_MODE_TUNE:
                state.isTuning.store(true);
                ESP_LOGI(TAG, "✅ TX STATE: Set to TUNE mode by %s", sourceToString(command.source));
                break;
            default:
                ESP_LOGW(TAG, "Unknown TX mode:%i", mode);
                break;
            }

            const bool willSendToRadio = shouldSendToRadio(command);
            ESP_LOGI(TAG, "🔍 TX ROUTING: shouldSendToRadio=%s", willSendToRadio ? "true" : "false");

            if (willSendToRadio)
            {
                std::string cmdStr = formatTXResponse(mode);
                ESP_LOGI(TAG, "📤 TX SEND: Sending to radio: %s", cmdStr.c_str());
                sendToRadio(radioSerial, cmdStr);
            }
            else
            {
                ESP_LOGW(TAG, "❌ TX BLOCKED: Command not sent to radio (check cache/source policy)");
            }
            // Route TX state to AI-enabled interfaces (mimicking radio Answer behavior)
            const std::string txMsg = formatTXResponse(mode);
            routeSetCommandToAIInterfaces(command, txMsg, usbSerial, radioManager);
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            int mode = getIntParam(command, 0, TX_MODE_SEND);
            if (isValidTxMode(mode))
            {
                // Radio TX answer - the radio is the ultimate authority
                const uint64_t currentTime = esp_timer_get_time();

                ESP_LOGW(TAG, "TXMON: radio asserted TX (mode=%d); monitoring outbound CAT", mode);

                // Force acquire TX ownership for "Remote" source when radio reports TX
                // This ensures the radio always has control when it's actually transmitting
                if (!state.tryAcquireTx(CommandSource::Remote, currentTime))
                {
                    // If we can't acquire (someone else owns), force release and try again
                    ESP_LOGW(TAG, "Radio TX answer: forcing release of current ownership");
                    state.forceReleaseTx(currentTime);
                    state.tryAcquireTx(CommandSource::Remote, currentTime);
                }

                switch (mode)
                {
                case TX_MODE_SEND:
                    state.isTuning.store(false);
                    break;
                case TX_MODE_DATA_SEND:
                    state.isTuning.store(false);
                    break;
                case TX_MODE_TUNE:
                    state.isTuning.store(true);
                    break;
                default:
                    break;
                }
                ESP_LOGD(TAG, "Updated TX mode from radio: %d", mode);

                // Invalidate IF cache so next query rebuilds with updated TX state
                state.commandCache.record("IF", 0);
                ESP_LOGD(TAG, "🔄 TX ANSWER: Invalidated IF cache to reflect TX state");
            }

            // Use unified routing for TX answers
            std::string response = formatTXResponse(mode);
            routeAnswerResponse(command, response, usbSerial, radioManager);
            return true;
        }

        return false;
    }

    bool TransmitterCommandHandler::handleRX(const RadioCommand &command, ISerialChannel &radioSerial,
                                             ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // RX: Receive (shorthand for TX0)
        auto &state = radioManager.getState();

        ESP_LOGI(TAG, "🔧 RX SET: Setting to RX mode (source: %s, cmd: %s)", sourceToString(command.source),
                 command.originalMessage.c_str());

        if (isSet(command) || (command.isLocal() && command.params.empty()))
        {
            // Try to release TX ownership
            const uint64_t currentTime = esp_timer_get_time();

            const int previousOwner = state.getTxOwner();

            // Panel (physical buttons) can always force RX regardless of ownership
            // Other sources must respect ownership unless they are local (USB/display/panel) panic overrides
            if (command.source == CommandSource::Panel)
            {
                ESP_LOGI(TAG, "🔘 Panel RX: Physical button forces TX release");
                state.forceReleaseTx(currentTime);
            }
            else if (!state.releaseTx(command.source, currentTime))
            {
                if (command.isLocal())
                {
                    ESP_LOGW(TAG, "⚠️ RX override: %s requested RX while owned by source %d — forcing release",
                             sourceToString(command.source), previousOwner);
                    state.forceReleaseTx(currentTime);
                }
                else
                {
                    ESP_LOGW(TAG, "❌ RX DENIED: Source %s cannot release TX (owned by source %d)",
                             sourceToString(command.source), previousOwner);
                    return false;
                }
            }

            // Set to receive mode (TX ownership released successfully or forced)
            state.isTuning.store(false);
            radioManager.forceReleasePrimaryControl();
            ESP_LOGI(TAG, "✅ RX STATE: Set to RX mode by %s", sourceToString(command.source));

            const bool willSendToRadio = shouldSendToRadio(command);
            ESP_LOGI(TAG, "🔍 RX ROUTING: shouldSendToRadio=%s", willSendToRadio ? "true" : "false");

            if (willSendToRadio)
            {
                ESP_LOGI(TAG, "📤 RX SEND: Sending to radio: RX;");
                sendToRadio(radioSerial, buildCommand("RX"));
            }
            else
            {
                ESP_LOGW(TAG, "❌ RX BLOCKED: Command not sent to radio (check cache/source policy)");
            }
            // Route RX state to AI-enabled interfaces (mimicking radio Answer behavior)
            const std::string rxMsg = buildCommand("RX");
            routeSetCommandToAIInterfaces(command, rxMsg, usbSerial, radioManager);
            return true;
        }

        if (command.type == CommandType::Read && command.source == CommandSource::Remote)
        {
            // Remote Read RX: This is actually a confirmation/echo from the radio
            // The radio sends RX; as a Read when it has switched to RX mode
            ESP_LOGI(TAG, "📡 RX read from radio: Confirming RX mode state");

            // Force release TX ownership when radio confirms RX
            const uint64_t currentTime = esp_timer_get_time();
            if (state.forceReleaseTx(currentTime))
            {
                ESP_LOGI(TAG, "✅ RX STATE: Confirmed RX mode from radio - TX ownership force released");
            }

            state.isTuning.store(false);
            radioManager.forceReleasePrimaryControl();

            // Route this confirmation to AI-enabled interfaces as an Answer
            const std::string rxMsg = buildCommand("RX");
            routeAnswerResponse(command, rxMsg, usbSerial, radioManager);
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            // RX Answer from radio - force release TX ownership
            // The radio is the ultimate authority for TX/RX state
            ESP_LOGI(TAG, "📡 RX ANSWER: Received from radio, routing to AI-enabled interfaces");

            const uint64_t currentTime = esp_timer_get_time();
            if (state.forceReleaseTx(currentTime))
            {
                ESP_LOGI(TAG, "✅ RX ANSWER: TX ownership force released due to radio RX answer");

                // Invalidate IF cache so next query rebuilds with updated TX/RX state
                state.commandCache.record("IF", 0);
                ESP_LOGD(TAG, "🔄 RX ANSWER: Invalidated IF cache to reflect RX state");
            }

            radioManager.forceReleasePrimaryControl();
            routeAnswerResponse(command, buildCommand("RX"), usbSerial, radioManager);
            return true;
        }

        return false;
    }

    // Note: No TQ command in TS-590SG; use IF; (P8) to infer TX status.

    // PS moved to InterfaceSystemCommandHandler

    // =============================================================================
    // New transmitter control commands
    // =============================================================================


    bool TransmitterCommandHandler::handleTP(const RadioCommand &command, ISerialChannel &radioSerial,
                                             ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // TP: TX Tune power
        auto &state = radioManager.getState();

        if (isQuery(command))
        {
            // TP; - Query TX tune power using standardized pattern
            return handleLocalQueryStandard(command, radioSerial, usbSerial, radioManager, "TP", TTL_STATUS,
                                            [this](const RadioState &st)
                                            {
                                                int currentPower = st.txTunePower;
                                                if (currentPower == 0)
                                                    currentPower = 10; // Default to 10W if not set
                                                return formatResponse3D("TP", currentPower);
                                            });
        }

        if (isSet(command))
        {
            int power = parseLevel(command);
            if (!isValidPowerLevel(power))
            {
                ESP_LOGW(TAG, "Invalid TP tune power: %d", power);
                return false;
            }

            // Update state
            state.txTunePower = power;

            // Update cache timestamp when state changes
            uint64_t currentTime = esp_timer_get_time();
            radioManager.getState().commandCache.update("TP", currentTime);

            if (shouldSendToRadio(command))
            {
                auto cmdStr = formatResponse3D("TP", power);
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "Set tune power to %d watts", power);
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            int power = parseLevel(command);
            if (power >= 0)
            {
                // Update state from radio response
                state.txTunePower = power;

                // Update cache timestamp for AI mode compatibility
                uint64_t currentTime = esp_timer_get_time();
                radioManager.getState().commandCache.update("TP", currentTime);

                ESP_LOGD(TAG, "Updated tune power from radio: %d", power);
            }

            // Use unified routing for TP answers - preserve original formatting
            routeAnswerResponse(command, command.originalMessage, usbSerial, radioManager);
            return true;
        }

        return false;
    }

    bool TransmitterCommandHandler::handlePR(const RadioCommand &command, ISerialChannel &radioSerial,
                                             ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // PR: Speech Processor enable
        if (isQuery(command))
        {
            if (command.isUsb())
            {
                // Return default speech processor state (off)
                respondToSource(command, buildCommand("PR", "0"), usbSerial, radioManager);
            }
            else if (shouldSendToRadio(command))
            {
                if (command.isUsb())
                {
                    radioManager.getState().queryTracker.recordQuery("PR", esp_timer_get_time());
                }
                sendToRadio(radioSerial, buildCommand("PR"));
            }
            return true;
        }

        if (isSet(command))
        {
            int enabled = parseNumericValue(command);
            if (enabled < 0 || enabled > 1)
            {
                ESP_LOGW(TAG, "Invalid PR speech processor state: %d", enabled);
                return false;
            }

            // Update local state
            radioManager.getState().processor = enabled == 1;

            if (shouldSendToRadio(command))
            {
                std::string cmdStr = buildCommand("PR", std::to_string(enabled));
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "Set speech processor %s", enabled ? "ON" : "OFF");
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            // Use unified routing for PR answers
            int enabled = parseNumericValue(command);
            std::string response = buildCommand("PR", std::to_string(enabled));
            routeAnswerResponse(command, response, usbSerial, radioManager);
            return true;
        }

        return false;
    }

    bool TransmitterCommandHandler::handlePL(const RadioCommand &command, ISerialChannel &radioSerial,
                                             ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // PL: Speech Processor input/output levels (format: PL050060; = input 50, output 60)
        auto &state = radioManager.getState();

        if (isQuery(command))
        {
            if (command.isUsb())
            {
                // Return default levels
                int inLevel = state.speechProcessorInLevel;
                int outLevel = state.speechProcessorOutLevel;
                auto response = formatResponseDual3D("PL", inLevel, outLevel);
                respondToSource(command, response, usbSerial, radioManager);
            }
            else if (shouldSendToRadio(command))
            {
                radioManager.getState().queryTracker.recordQuery("PL", esp_timer_get_time());
                sendToRadio(radioSerial, buildCommand("PL"));
            }
            return true;
        }

        if (isSet(command))
        {
            // PL set command expects 2 parameters: input level (3 digits) and output level (3 digits)
            if (command.params.size() < 2)
            {
                ESP_LOGW(TAG, "PL set command needs 2 parameters, got: %zu", command.params.size());
                return false;
            }

            int inLevel = std::stoi(getStringParam(command, 0));
            int outLevel = std::stoi(getStringParam(command, 1));

            if (inLevel < 0 || inLevel > 100 || outLevel < 0 || outLevel > 100)
            {
                ESP_LOGW(TAG, "Invalid PL levels: input=%d, output=%d", inLevel, outLevel);
                return false;
            }

            // Update local state
            state.speechProcessorInLevel = inLevel;
            state.speechProcessorOutLevel = outLevel;

            if (shouldSendToRadio(command))
            {
                auto cmdStr = formatResponseDual3D("PL", inLevel, outLevel);
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "Set speech processor levels: input=%d, output=%d", inLevel, outLevel);
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            // Parse response and update state
            std::string paramStr = getStringParam(command, 0);
            if (paramStr.length() == 6)
            {
                int inLevel = std::stoi(paramStr.substr(0, 3));
                int outLevel = std::stoi(paramStr.substr(3, 3));
                state.speechProcessorInLevel = inLevel;
                state.speechProcessorOutLevel = outLevel;
            }

            // Use unified routing for PL answers
            auto response = buildCommand("PL", paramStr);
            routeAnswerResponse(command, response, usbSerial, radioManager);
            return true;
        }

        return false;
    }

    bool TransmitterCommandHandler::handleVX(const RadioCommand &command, ISerialChannel &radioSerial,
                                             ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // VX: VOX on/off / CW break-in
        if (isQuery(command))
        {
            if (shouldSendToRadio(command))
            {
                radioManager.getState().queryTracker.recordQuery("VX", esp_timer_get_time());
                sendToRadio(radioSerial, buildCommand("VX"));
            }
            else
            {
                // Return default VOX state (off)
                respondToSource(command, buildCommand("VX", "0"), usbSerial, radioManager);
            }
            return true;
        }

        if (isSet(command))
        {
            int voxMode = parseNumericValue(command);
            if (voxMode < 0 || voxMode > 2)
            { // 0=off, 1=VOX, 2=CW break-in
                ESP_LOGW(TAG, "Invalid VX VOX mode: %d", voxMode);
                return false;
            }

            if (shouldSendToRadio(command))
            {
                std::string cmdStr = buildCommand("VX", std::to_string(voxMode));
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "Set VOX mode to %d", voxMode);
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            // Use unified routing for VX answers
            int voxMode = parseNumericValue(command);
            std::string response = buildCommand("VX", std::to_string(voxMode));
            routeAnswerResponse(command, response, usbSerial, radioManager);
            return true;
        }

        return false;
    }


    bool TransmitterCommandHandler::handleVD(const RadioCommand &command, ISerialChannel &radioSerial,
                                             ISerialChannel &usbSerial, RadioManager &radioManager)
    {
        // VD: VOX delay time
        if (isQuery(command))
        {
            if (shouldSendToRadio(command))
            {
                if (command.isUsb())
                {
                    radioManager.getState().queryTracker.recordQuery("VD", esp_timer_get_time());
                }
                sendToRadio(radioSerial, buildCommand("VD"));
            }
            else
            {
                // Return default VOX delay
                auto response = formatResponse4D("VD", 500); // 500ms default
                respondToSource(command, response, usbSerial, radioManager);
            }
            return true;
        }

        if (isSet(command))
        {
            int delay = parseLevel(command);
            if (!isValidVoxDelay(delay))
            {
                ESP_LOGW(TAG, "Invalid VD VOX delay: %d", delay);
                return false;
            }

            if (shouldSendToRadio(command))
            {
                auto cmdStr = formatResponse4D("VD", delay);
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "Set VOX delay to %d ms", delay);
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            // Use unified routing for VD answers
            int delay = parseLevel(command);
            auto response = formatResponse4D("VD", delay);
            routeAnswerResponse(command, response, usbSerial, radioManager);
            return true;
        }

        return false;
    }

    // =============================================================================
    // Helper functions
    // =============================================================================

    int TransmitterCommandHandler::parseNumericValue(const RadioCommand &command) const
    {
        return getIntParam(command, 0, -1);
    }

    int TransmitterCommandHandler::parsePowerState(const RadioCommand &command) const
    {
        return getIntParam(command, 0, -1);
    }

    int TransmitterCommandHandler::parseLevel(const RadioCommand &command) const { return getIntParam(command, 0, -1); }

    bool TransmitterCommandHandler::isValidPowerLevel(int power) const
    {
        return power >= MIN_POWER && power <= MAX_POWER;
    }

    bool TransmitterCommandHandler::isValidVoxDelay(int delay) const
    {
        return delay >= MIN_VOX_DELAY && delay <= MAX_VOX_DELAY;
    }

    // TX/RX helper functions (from TxRxControlCommandHandler)
    bool TransmitterCommandHandler::isValidTxMode(int mode) const
    {
        return mode >= TX_MODE_SEND && mode <= TX_MODE_TUNE;
    }

    std::string TransmitterCommandHandler::formatTXResponse(int mode) const
    {
        return buildCommand("TX", std::to_string(mode));
    }

    // formatTQResponse removed: TQ is not a real TS-590SG command.

} // namespace radio
