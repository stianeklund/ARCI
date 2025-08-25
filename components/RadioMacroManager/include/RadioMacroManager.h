#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <functional>

namespace radio {
    class RadioManager;

    // Forward declaration
class CATHandler;

/**
 * @brief Manages complex sequences of CAT commands as semantic macros
 * 
 * The RadioMacroManager provides high-level semantic operations that execute
 * multiple CAT commands as atomic operations. This is ideal for complex
 * radio configuration changes that require multiple coordinated commands.
 * 
 * Examples:
 * - Transverter mode: Enable transverter menu + configure antennas + DRV out
 * - Contest mode: Set memory channels + function keys + power levels
 * - Band change: Set frequency + filters + power + antenna
 * 
 * All macros use the proper command system through CATHandler for consistency
 * and proper state management.
 */
class RadioMacroManager {
public:
    /**
     * @brief Construct RadioMacroManager with RadioManager dependency
     * @param radioManager Reference to RadioManager for dispatching commands
     */
    explicit RadioMacroManager(RadioManager& radioManager);

    // Core macro operations
    
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

    // State validation and diagnostics
    
    /**
     * @brief Check if a macro can be safely executed
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
    RadioManager& radioManager_;      ///< Reference to RadioManager for command dispatch
    std::string lastError_;           ///< Last error message for diagnostics

    /**
     * @brief Send a sequence of commands as an atomic operation
     * @param commands Vector of command strings to send
     * @param delayMs Delay between commands in milliseconds (default 20ms)
     * @return true if all commands sent successfully
     */
    bool sendCommandSequence(const std::vector<std::string>& commands, int delayMs = 20) const;

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

    // Static configuration data
    
    /**
     * @brief Read current transverter-related state from radio
     * @return Vector of read commands to populate cache
     */
    static std::vector<std::string> readTransverterState();
    
    /**
     * @brief Generate configuration commands based on current vs desired state
     * @param enable true for enable sequence, false for disable
     * @param radioManager Reference to radio manager for accessing current state
     * @return Vector of configuration command strings (only for settings that need change)
     */
    static std::vector<std::string> generateTransverterCommands(bool enable, RadioManager& radioManager);

    static constexpr const char* TAG = "RadioMacroManager";
};

} // namespace radio
