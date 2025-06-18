#pragma once

#include "AntennaCommandHandler.h"
#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for receiver signal processing commands
 * 
 * This is a large but critical group that includes all commands related to the 
 * receiver's analog and digital signal processing chain. A handler for this group 
 * manages the state of the transceiver's various filtering and noise-mitigation features.
 * 
 * Commands handled:
 * - PA: Pre‑amplifier (moved from AntennaCommandHandler)
 * - RA: RF attenuator status (moved from AntennaCommandHandler)
 * - FL: IF filter select (A/B) - NEW
 * - FW: DSP filter bandwidth (for CW/FSK) - NEW
 * - SH: Receive high-cut / DSP Shift - NEW
 * - SL: Receive low-cut / DSP Width - NEW
 * - IS: IF shift (CW/CW-R only) - moved here for DSP grouping
 * - GC: AGC mode (Off, Slow, Fast) - NEW
 * - GT: AGC time constant - NEW
 * - NB: Noise blanker (type) - NEW
 * - NL: Noise blanker level - NEW
 * - NR: Noise Reduction mode (NR1/NR2) - NEW
 * - RL: Noise Reduction level - NEW
 * - NT: Notch Filter (Auto/Manual) - NEW
 * - BP: Manual notch frequency - NEW
 * - BC: Beat Cancel status - NEW (conflicts with band change in MiscCommandHandler)
 */
class ReceiverProcessingCommandHandler : public BaseCommandHandler {
public:
    ReceiverProcessingCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    // Commands moved from AntennaCommandHandler
    bool handlePA(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleRA(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);

    // Commands moved from GainCommandHandler (related to DSP processing)
    bool handleIS(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);

    // New DSP and receiver processing commands
    bool handleFL(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleFW(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleSH(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleSL(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleGC(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleGT(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleNB(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleNL(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleNR(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleRL(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleNT(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleBP(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleBC(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);

    // Helper functions
    int parseFrequencyValue(const RadioCommand& command) const;
    int parseLevel(const RadioCommand& command) const;
    bool isValidIFShift(int shift) const;
    bool isValidFilterBandwidth(int bandwidth) const;
    bool isValidNotchFrequency(int frequency) const;

    // Constants
    static constexpr int MIN_IF_SHIFT = 0;      // Hz (per TS-590SG spec: 0-9999)
    static constexpr int MAX_IF_SHIFT = 9999;   // Hz (per TS-590SG spec)

    static constexpr int MIN_FILTER_BANDWIDTH = 50;   // Hz
    static constexpr int MAX_FILTER_BANDWIDTH = 5000; // Hz
    
    static constexpr int MIN_NOTCH_FREQ = 300;   // Hz
    static constexpr int MAX_NOTCH_FREQ = 3000;  // Hz
    
    static constexpr int MIN_LEVEL = 0;
    static constexpr int MAX_LEVEL = 255;
    
    // AGC modes
    static constexpr int AGC_OFF = 0;
    static constexpr int AGC_SLOW = 1;
    static constexpr int AGC_FAST = 2;
    static constexpr int AGC_RESTORE = 3;  // Restore to previous Slow/Fast state

    // Noise blanker types
    static constexpr int NB_OFF = 0;
    static constexpr int NB1 = 1;
    static constexpr int NB2 = 2;
    static constexpr int NB3 = 3;

    // Noise reduction modes
    static constexpr int NR_OFF = 0;
    static constexpr int NR1 = 1;
    static constexpr int NR2 = 2;
    
    static constexpr const char* TAG = "ReceiverProcessingCommandHandler";
};

} // namespace radio