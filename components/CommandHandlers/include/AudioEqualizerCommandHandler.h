#pragma once

#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for audio processing and equalizer commands
 * 
 * Handles audio equalization commands for the TS-590SG according to specification.
 * 
 * Commands handled:
 * - EQ: Audio equalizer mode
 * - UR: RX equalizer (18-band user levels)
 * - UT: TX equalizer (18-band user levels)
 */
class AudioEqualizerCommandHandler : public BaseCommandHandler {
public:
    AudioEqualizerCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    static constexpr const char* TAG = "AudioEqualizerCommandHandler";
    
    bool handleEQ(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager) const;
    
    bool handleUR(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager) const;
    
    bool handleUT(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager) const;
    
    // Helper functions
    int parseEqualizerMode(const RadioCommand& command) const;
    std::string parseEqualizerBands(const RadioCommand& command) const;
    
    // Constants
    static constexpr int MIN_EQ_MODE = 0;
    static constexpr int MAX_EQ_MODE = 7;  // TS-590SG has 8 EQ settings (0-7)
    static constexpr int MIN_EQ_BAND_LEVEL = 0;
    static constexpr int MAX_EQ_BAND_LEVEL = 30;  // 18-band equalizer levels (00-30)
    static constexpr int EQ_BAND_COUNT = 18;
};

} // namespace radio