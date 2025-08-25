#include "include/RadioMacroManager.h"
#include "../RadioCore/include/CATHandler.h"
#include "esp_log.h"
#include "RadioManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace radio {
    RadioMacroManager::RadioMacroManager(RadioManager &radioManager)
        : radioManager_(radioManager) {
        ESP_LOGD(TAG, "RadioMacroManager initialized");
    }

    bool RadioMacroManager::executeTransverterMacro(const bool enable) {
        ESP_LOGI(TAG, "Executing transverter macro: %s", enable ? "ENABLE" : "DISABLE");

        if (!validateRadioState()) {
            setError("Radio not in valid state for transverter macro");
            return false;
        }

        // Set macro-in-progress flag to suppress AI coordination during execution
        auto &state = radioManager_.getState();
        state.macroInProgress.store(true);
        ESP_LOGD(TAG, "🔧 Macro started: AI coordination suppressed");

        // First read current state to populate cache
        if (const auto readCommands = readTransverterState(); !sendCommandSequence(readCommands)) {
            setError("Failed to read transverter state");
            state.macroInProgress.store(false);
            ESP_LOGD(TAG, "🔧 Macro failed: AI coordination restored");
            return false;
        }

        // Wait for read responses to be processed (allow time for radio to respond)
        vTaskDelay(pdMS_TO_TICKS(300));

        // Then generate and send configuration commands based on current state

        if (const auto configCommands = generateTransverterCommands(enable, radioManager_);
            sendCommandSequence(configCommands)) {
            ESP_LOGI(TAG, "Transverter macro executed successfully");
            state.macroInProgress.store(false);
            ESP_LOGD(TAG, "🔧 Macro completed: AI coordination restored");
            return true;
        }
        setError("Failed to execute transverter macro command sequence");
        state.macroInProgress.store(false);
        ESP_LOGD(TAG, "🔧 Macro failed: AI coordination restored");
        return false;
    }

    bool RadioMacroManager::executeBandChangeMacro(int band) {
        ESP_LOGW(TAG, "Band change macro not yet implemented (band: %d)", band);
        setError("Band change macro not implemented");
        return false;
    }

    bool RadioMacroManager::executeContestModeMacro() {
        ESP_LOGW(TAG, "Contest mode macro not yet implemented");
        setError("Contest mode macro not implemented");
        return false;
    }

    bool RadioMacroManager::executeSplitMacro(bool enable, bool copyVfoBeforeEnable) {
        ESP_LOGD(TAG, "Executing split macro: %s", enable ? "ENABLE" : "DISABLE");

        if (!validateRadioState()) {
            setError("Radio not in valid state for split macro");
            return false;
        }

        // Set macro-in-progress flag to suppress AI coordination during execution
        auto &state = radioManager_.getState();
        state.macroInProgress.store(true);

        std::vector<std::string> commands;
        if (enable) {
            if (copyVfoBeforeEnable) {
                commands.emplace_back("VV;"); // Copy VFO A → VFO B
            }

            if (radioManager_.getRxVfo() == 0) {
                commands.emplace_back("FT1;");
            }
            if (radioManager_.getRxVfo() == 1) {
                commands.emplace_back("FT0;");
            }
        }

        if (!sendCommandSequence(commands)) {
            setError("Failed to execute split macro command sequence");
            state.macroInProgress.store(false);
            return false;
        }

        state.macroInProgress.store(false);
        return true;
    }

    bool RadioMacroManager::canExecuteMacro(std::string_view macroName) {
        if (macroName == "transverter") {
            return validateRadioState();
        }
        if (macroName == "band_change" || macroName == "contest_mode") {
            // Not yet implemented
            return false;
        }

        ESP_LOGW(TAG, "Unknown macro: %.*s", (int)macroName.size(), macroName.data());
        return false;
    }

    bool RadioMacroManager::sendCommandSequence(const std::vector<std::string> &commands, const int delayMs) const {
        ESP_LOGI(TAG, "Sending command sequence of %zu commands", commands.size());

        for (size_t i = 0; i < commands.size(); ++i) {
            const auto &command = commands[i];
            ESP_LOGI(TAG, "Sending command %zu/%zu: %s", i + 1, commands.size(), command.c_str());

            // Use dispatchMessage for proper mutex serialization
            radioManager_.dispatchMessage(radioManager_.getMacroCATHandler(), command);

            // CRITICAL FIX: Always yield CPU to prevent starving TCA8418 task during rapid command sequences
            // This ensures button processing continues during macro execution
            vTaskDelay(pdMS_TO_TICKS(5)); // Minimum 5ms yield for task scheduling

            // Add additional delay between commands if specified
            if (i < commands.size() - 1 && delayMs > 5) {
                vTaskDelay(pdMS_TO_TICKS(delayMs - 5)); // Subtract the base yield time
            }
        }

        ESP_LOGV(TAG, "Command sequence completed successfully");
        return true;
    }

    bool RadioMacroManager::validateRadioState() {
        // For now, just return true. In the future, we could check:
        // - Radio is powered on (PS state)
        // - CAT interface is responsive
        // - No critical operations in progress
        // - Valid frequency/mode settings

        ESP_LOGV(TAG, "Radio state validation passed");
        return true;
    }

    void RadioMacroManager::setError(const std::string &error) {
        lastError_ = error;
        ESP_LOGW(TAG, "Error: %s", error.c_str());
    }

    std::vector<std::string> RadioMacroManager::readTransverterState() {
        ESP_LOGD(TAG, "Generating transverter state read commands");
        return {
            "AN;", // Read antenna/DRV status
            "EX0560000;", // Read EX056: Transverter enable state
            "EX0850000;", // Read EX085: DRV connector function
            "EX0590000;", // Read EX059: HF linear amplifier control
            "EX0600000;", // Read EX060: VHF linear amplifier control
            "XO;" // Read transverter offset (if enabled)
        };
    }

    std::vector<std::string> RadioMacroManager::generateTransverterCommands(
        const bool enable, RadioManager &radioManager) {
        std::vector<std::string> commands;
        const auto &state = radioManager.getState();
        if (enable) {
            // Target state for enable
            [[maybe_unused]] constexpr bool targetTransverter = true; // EX056 should be enabled (any non-zero value)
            constexpr bool targetDrvOut = true; // AN P3: DRV OUT ON

            if (!state.transverter) {
                commands.emplace_back("EX05600001;"); // Enable transverter (menu 056, value 0001)
            }
            // Always set DRV connector to DRO mode explicitly when enabling transverter
            // Tests expect this command even if already configured
            commands.emplace_back("EX08500000;"); // DRV connector to DRO mode (menu 085, value 0000)
            if (constexpr int targetHFLinear = 3; state.hfLinearAmpControl != targetHFLinear) {
                commands.emplace_back("EX05900003;"); // HF linear amp control (menu 059, value 0003)
            }
            if (constexpr int targetVhfLinear = 3; state.vhfLinearAmpControl != targetVhfLinear) {
                commands.emplace_back("EX06000003;"); // VHF linear amp control (menu 060, value 0003)
            }
            if (constexpr bool targetRxAnt = true;
                state.rxAnt != targetRxAnt || state.drvOut != targetDrvOut) {
                commands.emplace_back("AN911;"); // P1=9(no change), P2=1(RX ANT ON), P3=1(DRV OUT ON)
            }
            // Enable transverter display offset on display
            commands.emplace_back("UIXD1;");
        } else {
            // Target state for disable
            [[maybe_unused]] constexpr bool targetTransverter = false; // EX056: OFF
            constexpr bool targetDrvOut = false; // AN P3: DRV OUT OFF

            // Tests expect explicit disable commands even if state already matches
            commands.emplace_back("EX05600000;"); // Disable transverter (menu 056, value 0000)
            commands.emplace_back("AN900;"); // P1=9(no change), P2=0(RX ANT OFF), P3=0(DRV OUT OFF)
            // Disable transverter display offset on display
            commands.emplace_back("UIXD0;");

            // Note: We intentionally don't change EX059/EX060 (HF/VHF amp settings) on disable
            // These are crucial for PTT detection on other equipment and should be preserved
        }

        return commands;
    }
} // namespace radio
