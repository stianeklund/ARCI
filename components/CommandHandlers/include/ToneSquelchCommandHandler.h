#pragma once

#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for tone and squelch system commands
 * 
 * Handles CTCSS, DCS, and squelch control commands for the TS-590SG.
 * 
 * Commands handled:
 * - TO: Tone on/off
 * - TN: Tone frequency
 * - CT: CTCSS tone
 * - SQ: Squelch level
 * - DQ: DCS function
 * - QC: DCS code
 */
class ToneSquelchCommandHandler : public BaseCommandHandler {
public:
    ToneSquelchCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    static constexpr const char* TAG = "ToneSquelchCommandHandler";
    
    bool handleTO(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleTN(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleCT(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleSQ(const RadioCommand& cmd,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& rm) const;
    
    bool handleDQ(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleQC(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    // Helper functions
    int parseToneNumber(const RadioCommand& command) const;
    int parseSquelchLevel(const RadioCommand& command) const;
    int parseDcsCode(const RadioCommand& command) const;
    
    // Constants
    static constexpr int MIN_TONE_NUMBER = 0;   // TN command accepts 00-42 per JSON spec
    static constexpr int MAX_TONE_NUMBER = 42;  // TS-590SG supports 43 CTCSS tones (00-42)
    static constexpr int MIN_SQUELCH_LEVEL = 0;
    static constexpr int MAX_SQUELCH_LEVEL = 255;
    static constexpr int MIN_DCS_CODE = 23;     // Standard DCS range
    static constexpr int MAX_DCS_CODE = 754;
};

} // namespace radio