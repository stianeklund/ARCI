#pragma once

#include "BaseCommandHandler.h"
#include "RadioState.h"

namespace radio {

/**
 * @brief Handler for status and information commands
 * 
 * This group provides various status readings and radio information.
 * These commands are typically used for monitoring and diagnostics.
 * 
 * Commands handled:
 * - SM: S-meter reading (signal strength)
 * - RM: Signal meter reading (same as SM)
 * - IF: Information (frequency, mode, etc.)
 * - ID: Radio identification
 * - BY: BUSY status (Sky Command)
 */
class StatusInfoCommandHandler final : public BaseCommandHandler {
public:
    StatusInfoCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

    // Exposed for testing and other components needing a synthesized IF frame
    static std::string formatIFResponse(const RadioState& state);

private:
    bool handleSM(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;
    bool handleRM(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;
    bool handleIF(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;

    static bool handleID(const RadioCommand& cmd, ISerialChannel& radioSerial,
                         ISerialChannel& usbSerial, const RadioManager& rm);
    bool handleBY(const RadioCommand& cmd, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& rm) const;
    bool handleRI(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager) const;

    // Helper functions
    static std::string formatSMeterResponse(int level);
    
    // Constants
    static constexpr int MIN_SMETER = 0;
    static constexpr int MAX_SMETER = 30; // S9+60dB
    static constexpr auto RADIO_ID = "023"; // TS-590SG ID
    
    static constexpr auto TAG = "StatusInfoCommandHandler";
};

} // namespace radio
