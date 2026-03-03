#include "InterfaceSystemCommandHandler.h"
#include <algorithm>
#include <atomic>
#include "RadioManager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
    struct AiLogTracker
    {
        std::atomic<int> lastCdc0{-1};
        std::atomic<int> lastCdc1{-1};
        std::atomic<int> lastDisplay{-1};
        std::atomic<int> lastRadio{-1};
        std::atomic<int64_t> lastLogUs{0};
    };

    constexpr int64_t AI_LOG_INTERVAL_US = 10 * 1000 * 1000; // 10 seconds
    AiLogTracker g_aiLogTracker;

    void maybeLogAiCoordination(const radio::RadioState &state, const int radioAiMode)
    {
        const int cdc0 = state.usbCdc0AiMode.load();
        const int cdc1 = state.usbCdc1AiMode.load();
        const int display = state.displayAiMode.load();

        const int prevCdc0 = g_aiLogTracker.lastCdc0.load();
        const int prevCdc1 = g_aiLogTracker.lastCdc1.load();
        const int prevDisplay = g_aiLogTracker.lastDisplay.load();
        const int prevRadio = g_aiLogTracker.lastRadio.load();

        const bool changed =
            (cdc0 != prevCdc0) || (cdc1 != prevCdc1) || (display != prevDisplay) || (radioAiMode != prevRadio);

        const int64_t nowUs = esp_timer_get_time();
        const int64_t lastUs = g_aiLogTracker.lastLogUs.load();
        const bool due = (lastUs == 0) || (nowUs - lastUs >= AI_LOG_INTERVAL_US);

        if (!(changed || due))
        {
            return;
        }

        const char *reason = changed ? " (changed)" : " (periodic)";
        ESP_LOGI(radio::InterfaceSystemCommandHandler::TAG, "AI coordination: CDC0=%d CDC1=%d Display=%d -> Radio=%d%s",
                 cdc0, cdc1, display, radioAiMode, reason);

        g_aiLogTracker.lastCdc0.store(cdc0);
        g_aiLogTracker.lastCdc1.store(cdc1);
        g_aiLogTracker.lastDisplay.store(display);
        g_aiLogTracker.lastRadio.store(radioAiMode);
        g_aiLogTracker.lastLogUs.store(nowUs);
    }
} // namespace

namespace radio
{
    InterfaceSystemCommandHandler::InterfaceSystemCommandHandler() :
        BaseCommandHandler({"AI", "PS", "SR", "LK", "TC"}, "Interface & System Control Commands")
    {
    }

    bool InterfaceSystemCommandHandler::handleCommand(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                                      ISerialChannel &usbSerial, RadioManager &rm)
    {
        ESP_LOGV(TAG, "Handling interface/system command: %s", cmd.command.c_str());

        if (cmd.command == "AI")
        {
            return handleAI(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "PS")
        {
            return handlePS(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "SR")
        {
            return handleSR(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "LK")
        {
            return handleLK(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "TC")
        {
            return handleTC(cmd, radioSerial, usbSerial, rm);
        }
        return false;
    }

    // Helper function to calculate radio AI mode from all client preferences
    int InterfaceSystemCommandHandler::calculateRadioAIMode(const RadioState &state)
    {
        const int cdc0 = state.usbCdc0AiMode.load();
        const int cdc1 = state.usbCdc1AiMode.load();
        const int tcp0 = state.tcp0AiMode.load();
        const int tcp1 = state.tcp1AiMode.load();
        const int display = state.displayAiMode.load();

        // If ALL clients want AI off, radio is off. Otherwise, use highest requested mode.
        return (cdc0 == 0 && cdc1 == 0 && tcp0 == 0 && tcp1 == 0 && display == 0) ? 0 : std::max({cdc0, cdc1, tcp0, tcp1, display});
    }

    void InterfaceSystemCommandHandler::logAiCoordinationSnapshot(const RadioState &state)
    {
        const int radioAiMode = calculateRadioAIMode(state);
        maybeLogAiCoordination(state, radioAiMode);
    }

    // Helper function to update client AI preference and coordinate radio
    void InterfaceSystemCommandHandler::updateClientAIMode(CommandSource source, int mode, ISerialChannel &radioSerial,
                                                           RadioManager &radioManager) const
    {
        auto &state = radioManager.getState();

        // Update client-specific AI preference
        switch (source)
        {
        case CommandSource::UsbCdc0:
            state.usbCdc0AiMode.store(mode);
            break;
        case CommandSource::UsbCdc1:
            state.usbCdc1AiMode.store(mode);
            break;
        case CommandSource::Tcp0:
            state.tcp0AiMode.store(mode);
            break;
        case CommandSource::Tcp1:
            state.tcp1AiMode.store(mode);
            break;
        case CommandSource::Display:
            state.displayAiMode.store(mode);
            break;
        default:
            ESP_LOGW(TAG, "Unexpected AI set source: %d", (int)source);
            return;
        }

        // Auto-sync display AI mode with USB AI modes for forwarding
        // When any USB interface enables AI2/AI4, enable display forwarding too
        // NOTE: We only auto-enable, never auto-disable. Display maintains its own
        // preference (default AI2) to ensure the radio stays in AI2 mode for rapid
        // display updates, even when USB clients request AI0.
        if (source == CommandSource::UsbCdc0 || source == CommandSource::UsbCdc1)
        {
            const int maxUsbAiMode = std::max(state.usbCdc0AiMode.load(), state.usbCdc1AiMode.load());
            if (maxUsbAiMode >= 2)
            {
                state.displayAiMode.store(maxUsbAiMode);
                ESP_LOGD(TAG, "Auto-enabled display AI mode %d (following USB AI mode)", maxUsbAiMode);
            }
            // Intentionally do NOT auto-disable display AI when USB goes to AI0
            // This preserves display's preference and keeps radio in AI2 mode
        }

        // Calculate and apply coordinated radio AI mode
        const int radioAiMode = calculateRadioAIMode(state);
        state.aiMode.store(radioAiMode);

        maybeLogAiCoordination(state, radioAiMode);

        // Don't send AI commands to radio if power is off - this keeps the radio awake
        if (state.powerOffRequestTime.load() > 0) {
            ESP_LOGD(TAG, "⚡ Power off: Not sending AI mode to radio");
            return;
        }

        // Macro-in-progress guard: suppress AI coordination during macro execution
        // This prevents AI command storms during transverter/split/band macros
        if (state.macroInProgress.load()) {
            ESP_LOGD(TAG, "🔧 Macro in progress: Suppressing AI coordination (mode=%d)", radioAiMode);
            return;
        }

        // Debounce AI coordination: prevent rapid-fire AI set+query sequences
        // Only send commands if enough time has elapsed since last coordination
        constexpr uint64_t AI_DEBOUNCE_INTERVAL_US = 200000; // 200ms minimum interval
        const uint64_t now = esp_timer_get_time();
        const uint64_t lastCoordination = state.lastAiCoordinationTime.load();
        if (lastCoordination > 0 && (now - lastCoordination) < AI_DEBOUNCE_INTERVAL_US) {
            ESP_LOGD(TAG, "⏱️ AI coordination debounced (%.1f ms since last, min=200ms)",
                     (now - lastCoordination) / 1000.0);
            return;
        }

        // Update debounce timestamp
        state.lastAiCoordinationTime.store(now);

        // Send coordinated AI mode to radio
        const std::string aiSetCmd = formatAIResponse(radioAiMode);
        ESP_LOGI(TAG, "📤 Sending AI mode to radio: %s", aiSetCmd.c_str());
        sendToRadio(radioSerial, aiSetCmd);
        ESP_LOGI(TAG, "📤 Querying radio AI mode: AI;");
        sendToRadio(radioSerial, buildCommand("AI")); // Verify radio received it
    }

    bool InterfaceSystemCommandHandler::handleAI(const RadioCommand &command, ISerialChannel &radioSerial,
                                                 ISerialChannel &usbSerial, RadioManager &radioManager) const
    {
        auto &state = radioManager.getState();

        if (isQuery(command))
        {
            int responseMode;

            // Each client gets their own AI preference back, not the radio's mode
            switch (command.source)
            {
            case CommandSource::UsbCdc0:
                responseMode = state.usbCdc0AiMode.load();
                respondToSource(command, formatAIResponse(responseMode), usbSerial, radioManager);
                break;
            case CommandSource::UsbCdc1:
                responseMode = state.usbCdc1AiMode.load();
                respondToSource(command, formatAIResponse(responseMode), usbSerial, radioManager);
                break;
            case CommandSource::Tcp0:
                responseMode = state.tcp0AiMode.load();
                radioManager.sendToSource(CommandSource::Tcp0, formatAIResponse(responseMode));
                break;
            case CommandSource::Tcp1:
                responseMode = state.tcp1AiMode.load();
                radioManager.sendToSource(CommandSource::Tcp1, formatAIResponse(responseMode));
                break;
            case CommandSource::Display:
                responseMode = state.displayAiMode.load();
                radioManager.sendToDisplay(formatAIResponse(responseMode));
                break;
            default:
                ESP_LOGW(TAG, "Unexpected AI query from source: %d", (int)command.source);
                return false;
            }
            return true;
        }

        if (isSet(command))
        {
            const int mode = getIntParam(command, 0, -1);
            if (!isValidAIMode(mode))
            {
                ESP_LOGW(TAG, "Invalid AI mode: %d", mode);
                return false;
            }

            updateClientAIMode(command.source, mode, radioSerial, radioManager);
            return true;
        }

        if (command.type == CommandType::Answer)
        {
            const int mode = getIntParam(command, 0, -1);
            const int expectedMode = calculateRadioAIMode(state);

            if (isValidAIMode(mode))
            {
                state.aiMode.store(mode);
                if (mode == expectedMode)
                {
                    ESP_LOGI(TAG, "📻 Radio AI answer: AI%d; (✓ matches expected)", mode);
                }
                else
                {
                    ESP_LOGW(TAG, "📻 Radio AI answer: AI%d; (✗ expected AI%d - radio may be rejecting mode)",
                             mode, expectedMode);
                }
                routeAnswerResponse(command, formatAIResponse(mode), usbSerial, radioManager);
            }
            else
            {
                ESP_LOGE(TAG, "📻 Radio AI answer: INVALID mode %d in message: %s, not forwarding",
                         mode, command.originalMessage.c_str());
            }
            return true;
        }

        return false;
    }

    bool InterfaceSystemCommandHandler::handlePS(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                                 ISerialChannel &usbSerial, RadioManager &rm) const
    {
        // PS: Power control
        const auto &state = rm.getState();

        if (isQuery(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                // Check if we have fresh cached power state
                if (isCacheFresh(rm, "PS", TTL_STATUS))
                {
                    // Use cached power state
                    const PowerState cachedState = rm.getPowerState();
                    const int powerValue = (cachedState == On) ? PS_ON : PS_OFF;
                    const std::string response = formatPSResponse(powerValue);
                    // Refresh PS cache timestamp since we just served a valid answer
                    rm.getState().commandCache.update("PS", esp_timer_get_time());
                    respondToSource(cmd, response, usbSerial, rm);
                }
                else
                {
                    // Cache is stale, query the radio
                    ESP_LOGI(TAG, "PS cache stale/empty, forwarding query '%s' to radio", cmd.originalMessage.c_str());
                    // Record query when it originates from USB/TCP source
                    if (cmd.isCatClient())
                    {
                        const uint64_t timestamp = esp_timer_get_time();
                        state.queryTracker.recordQuery("PS", timestamp);
                        rm.noteQueryOrigin("PS", cmd.source, timestamp);
                    }
                    sendToRadio(radioSerial, cmd.originalMessage);
                }
            }
            else if (cmd.source == CommandSource::Remote)
            {
                // Remote-origin PS query (from RRC-1258): reply with our interface state
                const auto &state = rm.getState();

                // If power state not yet established from radio, assume ON to maintain RRC keepalive.
                // This is safer than responding PS0 which might cause RRC to stop polling.
                // Once we receive PS answer from radio, powerStateEstablished becomes true.
                const bool stateEstablished = state.powerStateEstablished.load();
                const bool ifaceOn = stateEstablished ? state.powerOn.load() : true;
                const int powerValue = ifaceOn ? PS_ON : PS_OFF;
                const std::string response = formatPSResponse(powerValue);

                ESP_LOGD(TAG, "RRC PS query -> %s", response.c_str());
                // Refresh PS cache timestamp based on local state we reported
                rm.getState().commandCache.update("PS", esp_timer_get_time());
                sendToRadio(radioSerial, response);
            }
            else
            {
                // Display or other local surfaces: no defaulting; let higher layers decide if needed
                ESP_LOGI(TAG, "PS query from non-USB/TCP source %d ignored (no passthrough)", static_cast<int>(cmd.source));
            }
            return true;
        }

        if (isSet(cmd))
        {
            const int powerState = getIntParam(cmd, 0, -1);
            if (!isValidPowerState(powerState))
            {
                ESP_LOGW(TAG, "Invalid power state: %d", powerState);
                return false;
            }

            rm.updatePowerState(powerState == PS_ON);
            // User explicitly setting power state establishes it
            rm.getState().powerStateEstablished.store(true);

            // Update keepAlive state ONLY for LOCAL user actions (not radio responses)
            if (cmd.isCatClient() || cmd.source == CommandSource::Panel)
            {
                if (powerState == PS_ON)
                {
                    ESP_LOGD(TAG, "Enabling keepAlive due to local PS1 cmd (user initiated)");
                    rm.getState().keepAlive.store(true);
                    // Clear power-off debounce since user explicitly wants power on
                    rm.getState().powerOffRequestTime.store(0);
                }
                else
                {
                    ESP_LOGD(TAG, "Disabling keepAlive due to local PS0 cmd (user initiated)");
                    rm.getState().keepAlive.store(false);
                    // Set flag to ignore ALL PS1 responses from radio until user explicitly powers on
                    // This prevents AI-mode broadcasts or stale responses from overriding user intent
                    rm.getState().powerOffRequestTime.store(esp_timer_get_time());
                }
            }

            // Note: Actual power control should be handled by hardware layer
            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = formatPSResponse(powerState);
                if (powerState == PS_OFF)
                {
                    // Send PS0 multiple times to ensure radio actually powers off
                    // TS-590SG may need multiple commands during busy periods
                    ESP_LOGI(TAG, "⚡ Sending PS0 to radio (3x to ensure power off)");
                    for (int i = 0; i < 3; ++i)
                    {
                        sendToRadio(radioSerial, cmdStr);
                        if (i < 2)
                        {
                            vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between sends
                        }
                    }
                }
                else
                {
                    ESP_LOGD(TAG, "sending to radio: %s", cmdStr.c_str());
                    sendToRadio(radioSerial, cmdStr);
                }
            }
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int powerState = getIntParam(cmd, 0, PS_ON);

            // Mark power state as established - we now have authoritative info from radio
            rm.getState().powerStateEstablished.store(true);

            // Special case: If radio reports PS1 (power on), automatically turn on interface
            // This allows the interface to come online when radio is already powered
            if (powerState == PS_ON)
            {
                // If user explicitly requested power off, debounce stale PS1 responses
                // but allow radio to override after a short debounce window (prevents front-panel
                // or RRC-1258 power-on from being blocked indefinitely)
                const uint64_t powerOffTime = rm.getState().powerOffRequestTime.load();
                constexpr uint64_t DEBOUNCE_WINDOW_US = 2000000; // 2 seconds

                if (powerOffTime > 0)
                {
                    const uint64_t currentTime = esp_timer_get_time();
                    const uint64_t elapsedUs = currentTime - powerOffTime;

                    if (elapsedUs < DEBOUNCE_WINDOW_US)
                    {
                        ESP_LOGI(TAG, "Ignoring stale PS1 from radio - debounce window active (elapsed=%llu us)",
                                 elapsedUs);
                        // Still route the response to USB clients, but don't trigger power-on sequence
                        const std::string response = formatPSResponse(powerState);
                        routeAnswerResponse(cmd, response, usbSerial, rm);
                        // Echo to radioSerial for RRC keepalive even during debounce
                        sendToRadio(radioSerial, response);
                        return true;
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Radio PS1 accepted - debounce expired (elapsed=%llu us), clearing powerOffRequestTime",
                                 elapsedUs);
                        rm.getState().powerOffRequestTime.store(0);
                    }
                }

                // Always ensure keepAlive is enabled when radio reports PS1
                // This handles cases where powerOn is already true but keepAlive was false
                const bool wasOff = !rm.getState().keepAlive.load();
                rm.getState().keepAlive.store(true);

                // Only run initialization sequence if state actually changes from OFF to ON
                if (rm.updatePowerState(true) || wasOff)
                {
                    ESP_LOGI(TAG, "Radio reported PS1 - enabling interface keepAlive and boot sequence");
                    rm.performBootSequence();
                    rm.syncTransverterMenuSettings();
                }
                else
                {
                    ESP_LOGD(TAG, "Radio already reported PS1 - state unchanged");
                }
            }
            else
            {
                // Radio confirmed power-off - do NOT clear powerOffRequestTime flag
                // User still wants power off; only their explicit PS1 command clears it

                // For PS0 (power off), only update if keepAlive is active
                // This prevents the radio from forcing interface off if user disabled monitoring
                if (rm.getState().keepAlive.load())
                {
                    ESP_LOGI(TAG, "Radio is OFF (PS0) - turning off interface (keepAlive was active)");
                    rm.updatePowerState(false);
                    rm.getState().keepAlive.store(false);
                }
                else
                {
                    ESP_LOGV(TAG, "Ignoring radio PS0 response - keepAlive inactive (user controls interface)");
                }
            }

            // Use unified routing for PS answers
            const std::string response = formatPSResponse(powerState);
            routeAnswerResponse(cmd, response, usbSerial, rm);

            // Also echo PS response back to radioSerial for RRC-1258 keepalive
            // The RRC client needs to see PS1; responses to maintain its connection
            sendToRadio(radioSerial, response);
            return true;
        }

        return false;
    }

    bool InterfaceSystemCommandHandler::handleSR(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                                 ISerialChannel &usbSerial, RadioManager &rm)
    {
        // SR: Reset the transceiver
        if (isSet(cmd) || cmd.params.empty())
        {
            ESP_LOGW(TAG, "Transceiver reset requested - this would normally reset the radio");

            // In a real implementation, this would trigger a hardware reset
            // For now, we just log and forward to radio
            if (shouldSendToRadio(cmd))
            {
                sendToRadio(radioSerial, buildCommand("SR"));
            }

            return true;
        }

        return false;
    }

    bool InterfaceSystemCommandHandler::handleLK(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                                 ISerialChannel &usbSerial, RadioManager &rm) const
    {
        // LK: Panel lock
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            if (cmd.isCatClient())
            {
                // Return current panel lock state (default to unlocked)
                const std::string response = formatLKResponse(LK_OFF);
                respondToSource(cmd, response, usbSerial, rm);
            }
            else if (shouldSendToRadio(cmd))
            {
                const uint64_t timestamp = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("LK", timestamp);
                rm.noteQueryOrigin("LK", cmd.source, timestamp);
                sendToRadio(radioSerial, cmd.originalMessage);
            }
            return true;
        }

        if (isSet(cmd))
        {
            // Parse LKP1P2; format where P1=lock state (0/1), P2=reserved (always 0)
            std::string param = getStringParam(cmd, 0, "");
            int lockState = -1;

            if (param.length() >= 1 && std::isdigit(param[0]))
            {
                lockState = param[0] - '0'; // Extract P1 (lock state)
                // P2 is ignored as per spec (always 0)
            }
            else
            {
                // Fallback to single parameter format for backward compatibility
                lockState = getIntParam(cmd, 0, -1);
            }

            if (!isValidLockState(lockState))
            {
                ESP_LOGW(TAG, "Invalid lock state: %d", lockState);
                return false;
            }

            ESP_LOGD(TAG, "Panel lock: %s", lockState == LK_ON ? "LOCKED" : "UNLOCKED");
            // Update state
            state.panelLock.store(lockState == LK_ON, std::memory_order_relaxed);

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = formatLKResponse(lockState);
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            // Use unified routing for LK answers
            const int lockState = getIntParam(cmd, 0, LK_OFF);
            const std::string response = formatLKResponse(lockState);
            routeAnswerResponse(cmd, response, usbSerial, rm);
            return true;
        }

        return false;
    }

    bool InterfaceSystemCommandHandler::handleTC(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                                 ISerialChannel &usbSerial, const RadioManager &rm) const
    {

        // TC: Terminal Control (indicates a program like ARCP is connected)
        if (isQuery(cmd))
        {
            // Fixed-answer: do not traverse RRC; always answer locally
            const std::string response = formatTCResponse(TC_ON);
            respondToSource(cmd, response, usbSerial, rm);
            return true;
        }

        if (isSet(cmd))
        {
            const int tcState = getIntParam(cmd, 0, -1);
            ESP_LOGD(TAG, "Terminal control state: %d", tcState);

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = formatTCResponse(tcState);
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            // Use unified routing for TC answers
            const int tcState = getIntParam(cmd, 0, TC_ON);
            const std::string response = formatTCResponse(tcState);
            routeAnswerResponse(cmd, response, usbSerial, rm);
            return true;
        }

        return false;
    }

    // =============================================================================
    // Helper functions
    // =============================================================================

    bool InterfaceSystemCommandHandler::isValidAIMode(const int mode)
    {
        return mode == AI_OFF || mode == AI_ON || mode == AI_BACKUP_ON;
    }

    bool InterfaceSystemCommandHandler::isValidPowerState(const int state) { return state == PS_OFF || state == PS_ON; }

    bool InterfaceSystemCommandHandler::isValidLockState(const int state) { return state == LK_OFF || state == LK_ON; }

    std::string InterfaceSystemCommandHandler::formatAIResponse(const int mode)
    {
        return buildCommand("AI", std::to_string(mode));
    }

    std::string InterfaceSystemCommandHandler::formatPSResponse(const int state)
    {
        return buildCommand("PS", std::to_string(state));
    }

    std::string InterfaceSystemCommandHandler::formatLKResponse(const int state)
    {
        // LKP1P2; format where P2=0 (always 0)
        return buildCommand("LK", std::to_string(state) + "0");
    }

    std::string InterfaceSystemCommandHandler::formatTCResponse(const int state)
    {
        // TC uses "TC P1;" format (space before parameter) per TS-590SG spec
        return buildCommand("TC ", std::to_string(state));
    }
} // namespace radio
