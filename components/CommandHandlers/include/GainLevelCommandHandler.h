#pragma once

#include "AntennaCommandHandler.h"
#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for gain and level control commands
 * 
 * This group manages all gain and level settings. These related parameters
 * often interact (e.g., AGC and RF Gain).
 * 
 * Commands handled:
 * - AG: AF (Audio Frequency) gain
 * - RG: RF (Radio Frequency) Gain
 * - MG: Microphone gain - NEW
 * - CG: Carrier level (for AM/FM modes) - NEW  
 * - ML: TX Monitor output level - NEW
 * - PC: Power control (from TransmitterCommandHandler)
 * - SQ: Squelch level (moved from tone/squelch to avoid conflict)
 * - IS: IF shift control
 */
class GainLevelCommandHandler : public BaseCommandHandler {
public:
    GainLevelCommandHandler();

    // ICommandHandler interface
    bool canHandle(const RadioCommand& command) const override;
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    // Existing commands from GainCommandHandler
    bool handleAG(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;
    bool handleRG(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;

    // New commands to implement
    bool handleMG(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;
    bool handleCG(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleML(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handlePC(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleVG(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);

    // Helper functions
    int parseGainValue(const RadioCommand& command) const;
    bool isValidGainValue(int value) const;
    bool isValidPowerLevel(int power) const;
    bool isValidVoxLevel(int level) const;

    static constexpr int MIN_VOX_LEVEL = 0;
    static constexpr int MAX_VOX_LEVEL = 9;


    // Constants for gain/level ranges
    static constexpr int MIN_GAIN = 0;
    static constexpr int MAX_GAIN = 255;
    

    static constexpr int MIN_CARRIER_LEVEL = 0;
    static constexpr int MAX_CARRIER_LEVEL = 100;
    
    static constexpr const char* TAG = "GainLevelCommandHandler";
};

} // namespace radio