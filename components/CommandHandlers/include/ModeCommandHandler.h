#pragma once

#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for mode and band control CAT commands
 * 
 * Handles operating mode setting and querying commands, enabling data mode,
 * and switching between amateur bands.
 * 
 * Commands handled:
 * - MD: Operating mode
 * - DA: DATA mode flag  
 * - MK: Mode key operation (virtual key press)
 * - AS: Auto Mode channel entry
 * - BU: Band select (up)
 * - BD: Band select (down)
 * 
 * Performance Optimizations:
 * - Jump table for O(1) command dispatch
 * - Constexpr lookup tables for mode validation
 * - String view usage for zero-copy operations
 * - Branch prediction hints for common paths
 */
class ModeCommandHandler final : public BaseCommandHandler {
public:
    ModeCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    // Performance: Jump table for O(1) command dispatch
    using HandlerFunc = bool (ModeCommandHandler::*)(const RadioCommand&, ISerialChannel&, ISerialChannel&, RadioManager&) const;
    using SimpleHandlerFunc = bool (ModeCommandHandler::*)(const RadioCommand&, ISerialChannel&, RadioManager&) const;
    
    static constexpr size_t COMMAND_TABLE_SIZE = 8;
    struct CommandEntry {
        std::string_view command;
        HandlerFunc fullHandler;
        SimpleHandlerFunc simpleHandler;
        bool usesUSB;
    };
    

    /**
     * @brief Handle MD (Mode) command
     */
    bool handleMD(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;

    /**
     * @brief Handle DA (Data Mode) command
     */
    bool handleDA(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;

    /**
     * @brief Handle MK (Mode Key) command - NEW
     */
    bool handleMK(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;

    /**
     * @brief Handle AS (Auto Mode) command - NEW
     */
    bool handleAS(const RadioCommand &command, ISerialChannel &radioSerial,
                  ISerialChannel &usbSerial, RadioManager &radioManager) const;

    /**
     * @brief Handle AS SET operation (5-parameter variant)
     */
    bool handleASSet(const RadioCommand &command, ISerialChannel &radioSerial,
                     ISerialChannel &usbSerial, RadioManager &radioManager) const;

    /**
     * @brief Handle BU (Band Up) command - MOVED from MiscCommandHandler
     */
    bool handleBU(const RadioCommand& command, ISerialChannel& radioSerial,
                  RadioManager& radioManager) const;

    /**
     * @brief Handle BD (Band Down) command - MOVED from MiscCommandHandler
     */
    bool handleBD(const RadioCommand& command, ISerialChannel& radioSerial,
                  RadioManager& radioManager) const;

    /**
     * @brief Validate mode value is within acceptable range
     * @param mode Mode value (0-9 typically)
     * @return true if valid
     */
    static bool isValidMode(int mode);

    /**
     * @brief Validate band number is within acceptable range
     * @param band Band number (0-17 typically for HF bands)
     * @return true if valid
     */
    [[nodiscard]] static bool isValidBand(int band);

    /**
     * @brief Get mode name for logging
     * @param mode Mode number
     * @return Human-readable mode name
     */
    static std::string_view getModeName(int mode);

    static constexpr CommandEntry commandTable_[COMMAND_TABLE_SIZE] = {
        {"MD", &ModeCommandHandler::handleMD, nullptr, true},
        {"DA", &ModeCommandHandler::handleDA, nullptr, true},
        {"MK", &ModeCommandHandler::handleMK, nullptr, true},
        {"AS", nullptr, nullptr, true}, // Special case - non-const method
        {"BU", nullptr, &ModeCommandHandler::handleBU, false},
        {"BD", nullptr, &ModeCommandHandler::handleBD, false},
        {"", nullptr, nullptr, false}, // Sentinel
        {"", nullptr, nullptr, false}  // Padding for alignment
    };

    // TS-590SG mode values
    static constexpr int MODE_LSB = 1;
    static constexpr int MODE_USB = 2;
    static constexpr int MODE_CW = 3;
    static constexpr int MODE_FM = 4;
    static constexpr int MODE_AM = 5;
    static constexpr int MODE_FSK = 6;
    static constexpr int MODE_CW_REV = 7;
    static constexpr int MODE_PSK = 8;
    static constexpr int MODE_FSK_REV = 9;

    static constexpr int MIN_MODE = 1;
    static constexpr int MAX_MODE = 9;
    
    // Band constants - per TS-590SG specification (BD/BU commands)
    static constexpr int MIN_BAND = 0;
    static constexpr int MAX_BAND = 10;
    
    static constexpr const char* TAG = "ModeCommandHandler";
    
    // Helper functions
    int parseNumericValue(const RadioCommand& command) const;
};

} // namespace radio