#pragma once

#include "AntennaCommandHandler.h"
#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for CW (Continuous Wave) operation commands
 * 
 * This group contains all commands that are specific to CW operation, including
 * the electronic keyer, speed settings, and the built-in Morse code decoder.
 * 
 * Commands handled:
 * - KS: Keying speed - NEW
 * - SD: CW break-in delay - NEW
 * - KY: Morse keyer (send text) - NEW
 * - CA: CW TUNE (auto-tune) - NEW
 * - CD0: Morse code decoder (enable) - NEW
 * - CD1: Morse code decoder threshold - NEW
 * - CD2: Morse code decoder output (read-only answer) - NEW
 */
class CwCommandHandler : public BaseCommandHandler {
public:
    CwCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    // CW operation commands
    bool handleKS(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleSD(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleKY(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleCA(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    
    // Morse code decoder command (unified)
    bool handleCD(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    
    // Private helper methods for CD subcommands
private:
    bool handleCD0(const RadioCommand& command, ISerialChannel& radioSerial,
                   ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleCD1(const RadioCommand& command, ISerialChannel& radioSerial,
                   ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleCD2(const RadioCommand& command, ISerialChannel& radioSerial,
                   ISerialChannel& usbSerial, RadioManager& radioManager);

public:

    // Helper functions
    int parseSpeed(const RadioCommand& command) const;
    int parseDelay(const RadioCommand& command) const;
    int parseThreshold(const RadioCommand& command) const;
    std::string parseText(const RadioCommand& command) const;
    bool isValidSpeed(int speed) const;
    bool isValidDelay(int delay) const;
    bool isValidThreshold(int threshold) const;
    bool isValidMorseText(std::string_view text) const;

    // Constants for CW operation
    static constexpr int MIN_CW_SPEED = 4;     // 4 WPM minimum
    static constexpr int MAX_CW_SPEED = 60;    // 60 WPM maximum
    static constexpr int DEFAULT_CW_SPEED = 20; // 20 WPM default
    
    static constexpr int MIN_BREAK_IN_DELAY = 15;   // 15ms minimum
    static constexpr int MAX_BREAK_IN_DELAY = 3000; // 3000ms maximum
    static constexpr int DEFAULT_BREAK_IN_DELAY = 100; // 100ms default
    
    static constexpr int MIN_DECODER_THRESHOLD = 1;  // Minimum threshold
    static constexpr int MAX_DECODER_THRESHOLD = 20; // Maximum threshold
    static constexpr int DEFAULT_DECODER_THRESHOLD = 10; // Default threshold
    
    // Maximum length for KY command text
    static constexpr size_t MAX_MORSE_TEXT_LENGTH = 24;
    
    static constexpr const char* TAG = "CwCommandHandler";
};

} // namespace radio