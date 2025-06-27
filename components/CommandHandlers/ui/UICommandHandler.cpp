#include "UICommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdio>

namespace radio
{

    UICommandHandler::UICommandHandler() :
        BaseCommandHandler({"UIPC", "UIML", "UIRL", "UIRS", "UINL", "UIPI", "UIPO", "UINF", "UIIS", "UIDA", "UIRI", "UIMN", "UIBL", "UIXD", "UIPS", "UIDE"}, "UI Meta Commands (Panel-Display Only)")
    {
    }

    bool UICommandHandler::handleCommand(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                         ISerialChannel &usbSerial, RadioManager &rm)
    {
        ESP_LOGV(TAG, "Handling UI command: %s", cmd.command.c_str());

        // UI commands are NEVER forwarded to radio
        // They exist solely for panel-display communication

        if (cmd.command == "UIPC")
        {
            return handleUIPC(cmd, rm);
        }
        if (cmd.command == "UIML")
        {
            return handleUIML(cmd, rm);
        }
        if (cmd.command == "UIRL")
        {
            return handleUIRL(cmd, rm);
        }
        if (cmd.command == "UIRS")
        {
            return handleUIRS(cmd, rm);
        }
        if (cmd.command == "UINL")
        {
            return handleUINL(cmd, rm);
        }
        if (cmd.command == "UIPI")
        {
            return handleUIPI(cmd, rm);
        }
        if (cmd.command == "UIPO")
        {
            return handleUIPO(cmd, rm);
        }
        if (cmd.command == "UINF")
        {
            return handleUINF(cmd, rm);
        }
        if (cmd.command == "UIIS")
        {
            return handleUIIS(cmd, rm);
        }
        if (cmd.command == "UIDA")
        {
            return handleUIDA(cmd, rm);
        }
        if (cmd.command == "UIRI")
        {
            return handleUIRI(cmd, rm);
        }
        if (cmd.command == "UIMN")
        {
            return handleUIMN(cmd, rm);
        }
        if (cmd.command == "UIBL")
        {
            return handleUIBL(cmd, rm);
        }
        if (cmd.command == "UIXD")
        {
            return handleUIXD(cmd, rm);
        }
        if (cmd.command == "UIPS")
        {
            return handleUIPS(cmd, rm);
        }
        if (cmd.command == "UIDE")
        {
            return handleUIDE(cmd, rm);
        }

        return false;
    }

    bool UICommandHandler::handleUIPC(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIPC: Power Control UI command
        // Format: UIPCnnn; where nnn is 005-100 (power in watts)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            // Query: return current UI power value or cached PC value
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::Power
                              ? state.uiState.currentValue.load()
                              : state.transmitPower;
            const std::string response = formatUIPC(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidPowerValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIPC value: %d (must be 5-100)", value);
                return false;
            }

            // Update UI state
            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            // Forward to display for UI update
            const std::string response = formatUIPC(value);
            ESP_LOGD(TAG, "UIPC set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        // Answer from display (unlikely but handle it)
        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidPowerValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIPC answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIML(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIML: Carrier/Monitor Level UI command
        // Format: UIMLnnn; where nnn is 000-020 (level)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::CarrierLevel
                              ? state.uiState.currentValue.load()
                              : state.txMonitorLevel;
            const std::string response = formatUIML(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidCarrierValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIML value: %d (must be 0-20)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUIML(value);
            ESP_LOGD(TAG, "UIML set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidCarrierValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIML answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIRL(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIRL: NR1 Level UI command
        // Format: UIRLnnn; where nnn is 001-010 (level)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::NrLevel
                              ? state.uiState.currentValue.load()
                              : state.nr1Level;
            const std::string response = formatUIRL(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidNrLevelValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIRL value: %d (must be 0-10)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUIRL(value);
            ESP_LOGD(TAG, "UIRL set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidNrLevelValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIRL answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIRS(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIRS: NR2 SPAC Speed UI command
        // Format: UIRSnnn; where nnn is 000-009 (speed 2-20ms in 2ms steps)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::Nr2Speed
                              ? state.uiState.currentValue.load()
                              : state.nr2Speed;
            const std::string response = formatUIRS(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidNr2SpeedValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIRS value: %d (must be 0-9)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUIRS(value);
            ESP_LOGD(TAG, "UIRS set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidNr2SpeedValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIRS answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUINL(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UINL: Noise Blanker Level UI command
        // Format: UINLnnn; where nnn is 000-010 (level)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::NbLevel
                              ? state.uiState.currentValue.load()
                              : state.noiseBlankerLevel;
            const std::string response = formatUINL(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidNbLevelValue(value))
            {
                ESP_LOGW(TAG, "Invalid UINL value: %d (must be 0-10)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUINL(value);
            ESP_LOGD(TAG, "UINL set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidNbLevelValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UINL answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIPI(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIPI: Speech Processor Input Level UI command
        // Format: UIPInnn; where nnn is 000-100 (level)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::ProcInputLevel
                              ? state.uiState.currentValue.load()
                              : state.speechProcessorInLevel;
            const std::string response = formatUIPI(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidProcLevelValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIPI value: %d (must be 0-100)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUIPI(value);
            ESP_LOGD(TAG, "UIPI set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidProcLevelValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIPI answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIPO(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIPO: Speech Processor Output Level UI command
        // Format: UIPOnnn; where nnn is 000-100 (level)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::ProcOutputLevel
                              ? state.uiState.currentValue.load()
                              : state.speechProcessorOutLevel;
            const std::string response = formatUIPO(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidProcLevelValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIPO value: %d (must be 0-100)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUIPO(value);
            ESP_LOGD(TAG, "UIPO set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidProcLevelValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIPO answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUINF(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UINF: Manual Notch Frequency UI command
        // Format: UINFnnn; where nnn is 000-127 (frequency index)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::NotchFrequency
                              ? state.uiState.currentValue.load()
                              : state.manualNotchFrequency;
            const std::string response = formatUINF(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidNotchFreqValue(value))
            {
                ESP_LOGW(TAG, "Invalid UINF value: %d (must be 0-127)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUINF(value);
            ESP_LOGD(TAG, "UINF set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidNotchFreqValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UINF answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIIS(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIIS: IF Shift UI command
        // Format: UIISnnnn; where nnnn is 0000-9999 (Hz)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::IfShift
                              ? state.uiState.currentValue.load()
                              : state.ifShiftValue.load();
            const std::string response = formatUIIS(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidIfShiftValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIIS value: %d (must be 0-9999)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUIIS(value);
            ESP_LOGD(TAG, "UIIS set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidIfShiftValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIIS answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIDA(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIDA: Data Mode UI command
        // Format: UIDAn; where n is 0 (OFF) or 1 (ON)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::DataMode
                              ? state.uiState.currentValue.load()
                              : state.dataMode.load();
            const std::string response = formatUIDA(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidDataModeValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIDA value: %d (must be 0 or 1)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUIDA(value);
            ESP_LOGD(TAG, "UIDA set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidDataModeValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIDA answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIRI(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIRI: RIT/XIT Offset UI command
        // Format: UIRIsnnnn; where s is sign (+/-) and nnnn is 0000-9999 (Hz)
        // Range: -9999 to +9999 Hz
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const int value = state.uiState.isActive() &&
                              state.uiState.getActiveControl() == UIControl::RitXitOffset
                              ? state.uiState.currentValue.load()
                              : state.ritXitOffset.load();
            const std::string response = formatUIRI(value);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            // Parse signed value from parameter (format: +0000 or -0000)
            const std::string param = getStringParam(cmd, 0, "0");
            int value = 0;

            if (!param.empty())
            {
                value = std::stoi(param);
            }

            if (!isValidRitXitValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIRI value: %d (must be -9999 to +9999)", value);
                return false;
            }

            state.uiState.currentValue.store(static_cast<int16_t>(value));
            state.uiState.lastUpdateTime.store(esp_timer_get_time());

            const std::string response = formatUIRI(value);
            ESP_LOGD(TAG, "UIRI set: %d, sending to display: %s", value, response.c_str());
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const std::string param = getStringParam(cmd, 0, "0");
            int value = 0;

            if (!param.empty())
            {
                value = std::stoi(param);
            }

            if (isValidRitXitValue(value))
            {
                state.uiState.currentValue.store(static_cast<int16_t>(value));
                ESP_LOGD(TAG, "UIRI answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIMN(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIMN: Menu/UI state command
        // Format: UIMNn; where n is 0 (dismissed) or 1 (active)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const bool active = state.uiState.isActive();
            const std::string response = formatUIMN(active);
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (value == 0)
            {
                // Dismiss UI - clear state
                ESP_LOGI(TAG, "UI dismissed via UIMN0");
                state.uiState.clear();
                sendToDisplayOnly(formatUIMN(false), rm);
            }
            else if (value == 1)
            {
                // UI active - just acknowledge (actual control type set elsewhere)
                ESP_LOGD(TAG, "UI active acknowledged via UIMN1");
                sendToDisplayOnly(formatUIMN(true), rm);
            }
            else
            {
                ESP_LOGW(TAG, "Invalid UIMN value: %d (must be 0 or 1)", value);
                return false;
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIBL(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIBL: Display backlight level command
        // Format: UIBLnnn; where nnn is 000-255 (backlight level)
        // Query: UIBL; → Response: UIBLnnn;
        // Set: UIBLnnn;

        if (isQuery(cmd))
        {
            // Query current backlight level from display
            // Since we don't track backlight state in RadioManager, just forward query
            const std::string response = formatUIBL(0); // Default to 0, display should respond
            sendToDisplayOnly(response, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (!isValidBacklightValue(value))
            {
                ESP_LOGW(TAG, "Invalid UIBL value: %d (must be 0-255)", value);
                return false;
            }

            // Forward to display to set backlight level
            const std::string command = formatUIBL(value);
            ESP_LOGD(TAG, "UIBL set: %d, sending to display: %s", value, command.c_str());
            sendToDisplayOnly(command, rm);
            return true;
        }

        // Answer from display (backlight level response)
        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (isValidBacklightValue(value))
            {
                ESP_LOGD(TAG, "UIBL answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIXD(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIXD: Transverter Display Offset toggle
        // Format: UIXDn; where n is 0 (disabled) or 1 (enabled)
        // When enabled, FA/FB/IF responses to USB include the XO offset
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const bool enabled = state.transverterOffsetEnabled;
            const std::string response = formatUIXD(enabled);
            sendToDisplayOnly(response, rm);
            ESP_LOGI(TAG, "UIXD query: transverter display offset is %s", enabled ? "enabled" : "disabled");
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (value != 0 && value != 1)
            {
                ESP_LOGW(TAG, "Invalid UIXD value: %d (must be 0 or 1)", value);
                return false;
            }

            state.transverterOffsetEnabled = (value == 1);
            ESP_LOGI(TAG, "UIXD set: transverter display offset %s", value ? "enabled" : "disabled");

            // Send confirmation to display
            const std::string response = formatUIXD(value == 1);
            sendToDisplayOnly(response, rm);

            // Refresh display with updated frequency (apply or remove offset)
            // This ensures display immediately shows correct frequency when UIXD is toggled
            rm.refreshDisplayFrequencies();

            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (value == 0 || value == 1)
            {
                state.transverterOffsetEnabled = (value == 1);
                ESP_LOGD(TAG, "UIXD answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIPS(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIPS: Panel Status / Display Wake command
        // Format: UIPSn; where n is 0 (idle/sleep) or 1 (active/awake)
        // - Panel sends UIPS1; on local activity to wake display
        // - Display can query UIPS; to check panel activity state
        // - Display can respond with current wake state
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            // Query: Check if there's recent panel activity
            constexpr uint64_t ACTIVITY_WINDOW_US = 500000; // 500ms
            const uint64_t now = esp_timer_get_time();
            const uint64_t lastEncoder = state.lastEncoderActivityTime.load(std::memory_order_relaxed);
            const uint64_t lastButton = state.lastButtonActivityTime.load(std::memory_order_relaxed);

            const bool recentActivity =
                (lastEncoder > 0 && (now - lastEncoder) < ACTIVITY_WINDOW_US) ||
                (lastButton > 0 && (now - lastButton) < ACTIVITY_WINDOW_US);

            const std::string response = formatUIPS(recentActivity);
            sendToDisplayOnly(response, rm);
            ESP_LOGD(TAG, "UIPS query: panel is %s", recentActivity ? "active" : "idle");
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (value != 0 && value != 1)
            {
                ESP_LOGW(TAG, "Invalid UIPS value: %d (must be 0 or 1)", value);
                return false;
            }

            // UIPS1 from panel = activity signal (already sent automatically)
            // UIPS0 from panel = explicit idle (optional, display manages own timeout)
            ESP_LOGD(TAG, "UIPS set: panel %s", value ? "active" : "idle");

            // Forward to display
            const std::string response = formatUIPS(value == 1);
            sendToDisplayOnly(response, rm);
            return true;
        }

        // Answer from display (display reporting its wake state)
        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (value == 0 || value == 1)
            {
                ESP_LOGD(TAG, "UIPS answer from display: %s", value ? "awake" : "asleep");
            }
            return true;
        }

        return false;
    }

    bool UICommandHandler::handleUIDE(const RadioCommand &cmd, RadioManager &rm) const
    {
        // UIDE: Display Communication Enable/Disable command
        // Format: UIDEn; where n is 0 (disabled) or 1 (enabled)
        // When disabled, all display communication is blocked (except UIDE itself)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            const bool enabled = state.displayCommunicationEnabled.load(std::memory_order_relaxed);
            const std::string response = formatUIDE(enabled);
            sendToDisplayOnly(response, rm);
            ESP_LOGD(TAG, "UIDE query: display communication is %s", enabled ? "enabled" : "disabled");
            return true;
        }

        if (isSet(cmd))
        {
            const int value = getIntParam(cmd, 0, -1);
            if (value != 0 && value != 1)
            {
                ESP_LOGW(TAG, "Invalid UIDE value: %d (must be 0 or 1)", value);
                return false;
            }

            const bool newValue = (value == 1);
            const bool oldValue = state.displayCommunicationEnabled.load(std::memory_order_relaxed);

            // Only log if state actually changed
            if (newValue != oldValue)
            {
                state.displayCommunicationEnabled.store(newValue, std::memory_order_relaxed);
                ESP_LOGI(TAG, "UIDE: display communication %s", newValue ? "enabled" : "disabled");
            }
            else
            {
                ESP_LOGD(TAG, "UIDE: display communication already %s (no change)", newValue ? "enabled" : "disabled");
            }

            // Send confirmation to display (this always goes through even if disabled)
            const std::string response = formatUIDE(newValue);
            sendToDisplayOnly(response, rm);

            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int value = getIntParam(cmd, 0, -1);
            if (value == 0 || value == 1)
            {
                state.displayCommunicationEnabled.store(value == 1, std::memory_order_relaxed);
                ESP_LOGD(TAG, "UIDE answer from display: %d", value);
            }
            return true;
        }

        return false;
    }

    // =============================================================================
    // Static helper functions
    // =============================================================================

    std::string UICommandHandler::formatUIPC(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UIPC%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIML(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UIML%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIRL(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UIRL%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIRS(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UIRS%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUINL(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UINL%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIPI(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UIPI%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIPO(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UIPO%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUINF(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UINF%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIIS(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UIIS%04d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIDA(const int value)
    {
        return value ? "UIDA1;" : "UIDA0;";
    }

    std::string UICommandHandler::formatUIRI(const int value)
    {
        // Format: UIRIsNNNN; where s is sign (+/-) and NNNN is 0000-9999
        char buf[16];
        snprintf(buf, sizeof(buf), "UIRI%+05d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIMN(const bool active)
    {
        return active ? "UIMN1;" : "UIMN0;";
    }

    std::string UICommandHandler::formatUIBL(const int value)
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "UIBL%03d;", value);
        return std::string(buf);
    }

    std::string UICommandHandler::formatUIXD(const bool enabled)
    {
        return enabled ? "UIXD1;" : "UIXD0;";
    }

    std::string UICommandHandler::formatUIPS(const bool active)
    {
        return active ? "UIPS1;" : "UIPS0;";
    }

    std::string UICommandHandler::formatUIDE(const bool enabled)
    {
        return enabled ? "UIDE1;" : "UIDE0;";
    }

    void UICommandHandler::sendToDisplayOnly(const std::string &cmd, RadioManager &rm)
    {
        if (auto *disp = rm.getDisplaySerial(); disp)
        {
            disp->sendMessage(cmd);
        }
    }

    bool UICommandHandler::isValidPowerValue(const int value)
    {
        return value >= 5 && value <= 100;
    }

    bool UICommandHandler::isValidCarrierValue(const int value)
    {
        return value >= 0 && value <= 20;
    }

    bool UICommandHandler::isValidNrLevelValue(const int value)
    {
        return value >= 0 && value <= 10;
    }

    bool UICommandHandler::isValidNbLevelValue(const int value)
    {
        return value >= 0 && value <= 10;
    }

    bool UICommandHandler::isValidNr2SpeedValue(const int value)
    {
        return value >= 0 && value <= 9;
    }

    bool UICommandHandler::isValidProcLevelValue(const int value)
    {
        return value >= 0 && value <= 100;
    }

    bool UICommandHandler::isValidNotchFreqValue(const int value)
    {
        return value >= 0 && value <= 127;
    }

    bool UICommandHandler::isValidIfShiftValue(const int value)
    {
        return value >= 0 && value <= 9999;
    }

    bool UICommandHandler::isValidDataModeValue(const int value)
    {
        return value == 0 || value == 1;
    }

    bool UICommandHandler::isValidRitXitValue(const int value)
    {
        return value >= -9999 && value <= 9999;
    }

    bool UICommandHandler::isValidBacklightValue(const int value)
    {
        return value >= 0 && value <= 255;
    }

} // namespace radio
