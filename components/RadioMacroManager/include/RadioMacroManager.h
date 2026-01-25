#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <cstdint>
#include "esp_err.h"

namespace radio {

class RadioManager;

/**
 * @brief Unified macro execution engine for all macro types
 *
 * RadioMacroManager handles both:
 * 1. Semantic macros: Complex, stateful operations (transverter, split, band change)
 * 2. User-defined macros: Stored CAT command sequences assigned to F-buttons
 *
 * All macros use the proper command system through CATHandler for consistency
 * and proper state management.
 *
 * Examples:
 * - Transverter mode: Enable transverter menu + configure antennas + DRV out
 * - User macro: "FA00014074000|MD2|DA1" -> switch to 20M FT8
 */
class RadioMacroManager {
public:
    /**
     * @brief Construct RadioMacroManager with RadioManager dependency
     * @param radioManager Reference to RadioManager for dispatching commands
     */
    explicit RadioMacroManager(RadioManager& radioManager);

    // =========================================================================
    // Semantic Macros (hardcoded complex operations)
    // =========================================================================

    /**
     * @brief Execute transverter mode macro
     *
     * Configures the radio for transverter operation by:
     * - Setting EX056 menu item (transverter enable/disable)
     * - Configuring RX antenna (AN00x)
     * - Setting DRV out (AN99x)
     *
     * @param enable true to enable transverter mode, false to disable
     * @return true if all commands executed successfully
     */
    bool executeTransverterMacro(bool enable);

    /**
     * @brief Execute band change macro (future implementation)
     * @param band Target band index
     * @return true if successful
     */
    bool executeBandChangeMacro(int band);

    /**
     * @brief Execute contest mode macro (future implementation)
     * @return true if successful
     */
    bool executeContestModeMacro();

    /**
     * @brief Enable/disable split operation as a macro
     *
     * Uses FR/FT (and optional VV) to configure RX/TX VFOs for split without
     * exposing CAT details to UI code.
     *
     * @param enable true to enable split (RX=A, TX=B), false to disable (RX=A, TX=A)
     * @param copyVfoBeforeEnable if true, issue VV; prior to enabling to copy A→B
     * @return true if command sequence was dispatched
     */
    bool executeSplitMacro(bool enable, bool copyVfoBeforeEnable = true);

    // =========================================================================
    // User-Defined Macros (stored in MacroStorage)
    // =========================================================================

    /**
     * @brief Execute a user-defined macro by ID
     *
     * Fetches macro definition from MacroStorage and executes its command sequence.
     * Commands are separated by '|' in storage and executed with ';' terminator.
     *
     * @param macroId ID of macro to execute (1-50)
     * @return ESP_OK if execution successful, ESP_ERR_NOT_FOUND if macro not found
     */
    esp_err_t executeUserMacro(uint8_t macroId);

    /**
     * @brief Execute a macro assigned to an F-button slot
     *
     * Looks up macro ID for slot and executes it.
     * Slots 0-5: F1-F6 short press
     * Slots 6-11: F1-F6 long press
     *
     * @param slot Slot number (0-11)
     * @return ESP_OK if execution started, ESP_ERR_NOT_FOUND if no macro assigned
     */
    esp_err_t executeSlot(uint8_t slot);

    /**
     * @brief Check if macro manager is ready to execute commands
     * @return true if initialized and ready
     */
    bool isReady() const;

    /**
     * @brief Get last status message from macro execution
     * @return Status string (empty if no error)
     */
    std::string getLastStatus() const { return lastStatus_; }

    // =========================================================================
    // State and Diagnostics
    // =========================================================================

    /**
     * @brief Check if a semantic macro can be safely executed
     * @param macroName Name of the macro to check
     * @return true if macro can be executed safely
     */
    static bool canExecuteMacro(std::string_view macroName);

    /**
     * @brief Get last error message from failed macro execution
     * @return Error message string, empty if no error
     */
    std::string getLastError() const { return lastError_; }

private:
    RadioManager& radioManager_;
    std::string lastError_;
    std::string lastStatus_;

    /**
     * @brief Send a sequence of commands as an atomic operation
     * @param commands Vector of command strings to send
     * @param delayMs Delay between commands in milliseconds (default 20ms)
     * @return true if all commands sent successfully
     */
    bool sendCommandSequence(const std::vector<std::string>& commands, int delayMs = 20) const;

    /**
     * @brief Execute a single CAT command
     * @param command Command string (with or without trailing semicolon)
     * @return ESP_OK on success, ESP_ERR_TIMEOUT on dispatch failure
     */
    esp_err_t executeSingleCommand(const std::string& command);

    /**
     * @brief Parse pipe-separated command string into vector
     * @param commands Pipe-separated command string (e.g., "FA00014074000|MD2|DA1")
     * @return Vector of individual command strings
     */
    static std::vector<std::string> parseCommandSequence(const std::string& commands);

    /**
     * @brief Set status message for execution tracking
     * @param fmt Format string
     * @param ... Arguments
     */
    void setStatus(const char* fmt, ...);

    /**
     * @brief Validate basic radio state before macro execution
     * @return true if radio is in a valid state for macro execution
     */
    static bool validateRadioState();

    /**
     * @brief Set error message for failed operations
     * @param error Error message to store
     */
    void setError(const std::string& error);

    // Transverter macro helpers
    static std::vector<std::string> readTransverterState();
    static std::vector<std::string> generateTransverterCommands(bool enable, RadioManager& radioManager);

    static constexpr const char* TAG = "RadioMacroManager";
};

} // namespace radio
