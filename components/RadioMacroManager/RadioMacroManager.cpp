#include "include/RadioMacroManager.h"
#include "../RadioCore/include/CATHandler.h"
#include "esp_log.h"
#include "RadioManager.h"
#include "MacroStorage.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <array>
#include <cstdarg>
#include <cstring>

namespace radio {

RadioMacroManager::RadioMacroManager(RadioManager &radioManager)
    : radioManager_(radioManager) {
    ESP_LOGI(TAG, "RadioMacroManager initialized");
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

            if (!state.transverter.load(std::memory_order_relaxed)) {
                commands.emplace_back("EX05600001;"); // Enable transverter (menu 056, value 0001)
            }
            // Always set DRV connector to DRO mode explicitly when enabling transverter
            // Tests expect this command even if already configured
            commands.emplace_back("EX08500000;"); // DRV connector to DRO mode (menu 085, value 0000)
            if (constexpr int targetHFLinear = 3; state.hfLinearAmpControl.load(std::memory_order_relaxed) != targetHFLinear) {
                commands.emplace_back("EX05900003;"); // HF linear amp control (menu 059, value 0003)
            }
            if (constexpr int targetVhfLinear = 3; state.vhfLinearAmpControl.load(std::memory_order_relaxed) != targetVhfLinear) {
                commands.emplace_back("EX06000003;"); // VHF linear amp control (menu 060, value 0003)
            }
            if (constexpr bool targetRxAnt = true;
                state.rxAnt.load(std::memory_order_relaxed) != targetRxAnt || state.drvOut.load(std::memory_order_relaxed) != targetDrvOut) {
                commands.emplace_back("AN911;"); // P1=9(no change), P2=1(RX ANT ON), P3=1(DRV OUT ON)
            }
            // Enable transverter display offset on display
            commands.emplace_back("UIXD1;");
        } else {
            // Target state for disable
            [[maybe_unused]] constexpr bool targetTransverter = false; // EX056: OFF
            [[maybe_unused]] constexpr bool targetDrvOut = false; // AN P3: DRV OUT OFF

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

// =============================================================================
// User-Defined Macro Execution (merged from MacroExecutor)
// =============================================================================

esp_err_t RadioMacroManager::executeUserMacro(uint8_t macroId) {
    ESP_LOGD(TAG, "executeUserMacro(%d): entry", macroId);

    // Fetch macro from storage
    storage::MacroDefinition macro{};
    esp_err_t err = storage::MacroStorage::instance().getMacro(macroId, macro);
    if (err != ESP_OK) {
        setStatus("Macro %d not found", macroId);
        ESP_LOGW(TAG, "executeUserMacro(%d): NOT FOUND in storage", macroId);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGD(TAG, "executeUserMacro(%d): loaded macro '%s', command='%s'",
             macroId, macro.name, macro.command);

    // Parse command sequence
    std::vector<std::string> commands = parseCommandSequence(macro.command);
    if (commands.empty()) {
        setStatus("Macro %d has no commands", macroId);
        ESP_LOGW(TAG, "executeUserMacro(%d): empty command sequence", macroId);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Executing macro %d ('%s') with %zu command(s)",
             macroId, macro.name, commands.size());

    // Execute commands with inter-command delay
    for (size_t i = 0; i < commands.size(); i++) {
        const std::string& cmd = commands[i];

        ESP_LOGD(TAG, "executeUserMacro(%d): cmd %zu/%zu = '%s'",
                 macroId, i + 1, commands.size(), cmd.c_str());

        err = executeSingleCommand(cmd);
        if (err != ESP_OK) {
            setStatus("Failed to execute command %zu of %zu", i + 1, commands.size());
            ESP_LOGW(TAG, "executeUserMacro(%d): FAILED cmd %zu '%s': %s",
                     macroId, i, cmd.c_str(), esp_err_to_name(err));
            return err;
        }

        // Add inter-command delay (50ms) if not the last command
        if (i < commands.size() - 1) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    setStatus("Macro %d executed successfully (%zu commands)", macroId, commands.size());
    ESP_LOGI(TAG, "Macro %d execution complete", macroId);
    return ESP_OK;
}

esp_err_t RadioMacroManager::executeSlot(uint8_t slot) {
    // Track execution entry for debugging
    static uint32_t executionCounter = 0;
    executionCounter++;
    const uint32_t thisExecution = executionCounter;

    ESP_LOGI(TAG, ">>> MACRO SLOT ENTRY #%lu: slot=%d (F%d)",
             thisExecution, slot, (slot % 6) + 1);

    if (slot >= storage::kMacroSlotCount) {
        setStatus("Invalid slot: %d", slot);
        ESP_LOGE(TAG, "<<< MACRO SLOT EXIT #%lu: Invalid slot %d (valid: 0-%zu)",
                 thisExecution, slot, storage::kMacroSlotCount - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // Get slot assignment
    std::array<uint8_t, storage::kMacroSlotCount> assignments{};
    esp_err_t err = storage::MacroStorage::instance().getSlotAssignments(assignments);
    if (err != ESP_OK) {
        setStatus("Failed to get slot assignments");
        ESP_LOGE(TAG, "<<< MACRO SLOT EXIT #%lu: Failed to get slot assignments: %s",
                 thisExecution, esp_err_to_name(err));
        return err;
    }

    uint8_t macroId = assignments[slot];
    ESP_LOGD(TAG, "Slot assignments: [%d,%d,%d,%d,%d,%d], slot %d -> macroId=%d",
             assignments[0], assignments[1], assignments[2],
             assignments[3], assignments[4], assignments[5],
             slot, macroId);

    if (macroId == 0) {
        setStatus("No macro assigned to slot %d (F%d)", slot, (slot % 6) + 1);
        ESP_LOGD(TAG, "<<< MACRO SLOT EXIT #%lu: No macro assigned to slot %d",
                 thisExecution, slot);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Executing macro from slot %d: macroId=%d", slot, macroId);

    err = executeUserMacro(macroId);
    ESP_LOGI(TAG, "<<< MACRO SLOT EXIT #%lu: result=%s", thisExecution, esp_err_to_name(err));
    return err;
}

bool RadioMacroManager::isReady() const {
    // Always ready since we use RadioManager reference (no separate init needed)
    return true;
}

esp_err_t RadioMacroManager::executeSingleCommand(const std::string& cmd) {
    if (cmd.empty()) {
        return ESP_OK;
    }

    std::string command = cmd;

    // Ensure command ends with semicolon
    if (command.back() != ';') {
        command += ';';
    }

    ESP_LOGI(TAG, "Macro dispatch START: '%s'", command.c_str());

    // Dispatch via RadioManager using the panel CAT handler
    const bool result = radioManager_.dispatchMessage(radioManager_.getPanelCATHandler(), command);
    if (!result) {
        ESP_LOGW(TAG, "Macro dispatch TIMEOUT or FAILED: '%s'", command.c_str());
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Macro dispatch END: '%s' (success)", command.c_str());
    return ESP_OK;
}

std::vector<std::string> RadioMacroManager::parseCommandSequence(const std::string& commands) {
    std::vector<std::string> result;
    std::string current;

    for (char c : commands) {
        if (c == '|') {
            if (!current.empty()) {
                result.push_back(current);
                current.clear();
            }
        } else {
            current += c;
        }
    }

    // Add any remaining command
    if (!current.empty()) {
        result.push_back(current);
    }

    return result;
}

void RadioMacroManager::setStatus(const char* fmt, ...) {
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    lastStatus_ = buffer;
}

} // namespace radio
