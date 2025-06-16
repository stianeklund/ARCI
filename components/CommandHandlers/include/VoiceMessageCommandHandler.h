#pragma once

#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for voice and message playback/recording commands
 * 
 * This group handles the control of the optional VGS-1 unit for voice 
 * announcements and message recording/playback.
 * 
 * Commands handled:
 * - LM: VGS-1 recorder control/status
 * - PB: Voice/CW message playback
 * - VR: VGS-1 status/voice guide control
 */
class VoiceMessageCommandHandler : public BaseCommandHandler {
public:
    VoiceMessageCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    bool handleLM(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;
    bool handlePB(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;
    bool handleVR(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;

    // Helper functions
    static bool isValidMessageNumber(int msgNum);

    static bool isValidPlaybackState(int state);

    static bool isValidVgsState(int state);

    static std::string formatLMResponse(int mode, int channel);

    static std::string formatPBResponse(int state);

    static std::string formatVRResponse(int state);
    
    // Constants
    
    // LM (VGS-1 recorder) modes
    static constexpr int LM_STOP = 0;         // Stop
    static constexpr int LM_RECORD = 1;       // Record
    static constexpr int LM_PLAY = 2;         // Play
    static constexpr int LM_FAST_FORWARD = 3; // Fast forward
    static constexpr int LM_FAST_REWIND = 4;  // Fast rewind
    
    // PB (Playback) states
    static constexpr int PB_STOP = 0;         // Stop playback
    static constexpr int PB_PLAY_MSG1 = 1;    // Play message 1
    static constexpr int PB_PLAY_MSG2 = 2;    // Play message 2
    static constexpr int PB_PLAY_MSG3 = 3;    // Play message 3
    static constexpr int PB_PLAY_MSG4 = 4;    // Play message 4
    static constexpr int PB_PLAY_CONSTANT = 5; // Play constant recording (per JSON spec)
    
    // VR (Voice guide) states
    static constexpr int VR_OFF = 0;          // Voice guide off
    static constexpr int VR_ON = 1;           // Voice guide on
    static constexpr int VR_ENHANCED = 2;     // Enhanced voice guide
    
    // Message limits
    static constexpr int MIN_MESSAGE = 1;
    static constexpr int MAX_MESSAGE = 4;
    static constexpr int MIN_VGS_CHANNEL = 1;
    static constexpr int MAX_VGS_CHANNEL = 4;
    
    static constexpr const char* TAG = "VoiceMessageCommandHandler";
};

} // namespace radio
