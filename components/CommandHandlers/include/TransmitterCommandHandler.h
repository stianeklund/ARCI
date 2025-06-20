#pragma once

#include "AntennaCommandHandler.h"
#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for transmitter control commands
 * 
 * This group includes commands for keying the transmitter, setting power output,
 * and configuring transmission-related features like the speech processor and VOX.
 * 
 * Commands handled:
 * - TX: Transmit control (TX0=RX, TX1=TX, TX2=TUNE) - merged from TxRxControlCommandHandler
 * - RX: Receive (shorthand for TX0) - merged from TxRxControlCommandHandler
 * - TQ: TX status query - merged from TxRxControlCommandHandler
 * - PC: Output power control (moved from GainLevel)
 * - PB: Playback (voice/message)
 * - TP: TX Tune power
 * - PR: Speech Processor enable
 * - PL: Speech Processor input/output levels
 * - VX: VOX on/off / CW break-in
 * - VG: VOX gain
 * - VD: VOX delay time
 * 
 * Note: PC (output power) moved to GainLevelCommandHandler
 */
class TransmitterCommandHandler : public BaseCommandHandler {
public:
    TransmitterCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand& command,
                       ISerialChannel& radioSerial,
                       ISerialChannel& usbSerial,
                       RadioManager& radioManager) override;

private:
    // TX/RX control commands (merged from TxRxControlCommandHandler)
    bool handleTX(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleRX(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleTQ(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    
    // Power and processor commands
    bool handlePC(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);

    // New transmitter control commands
    bool handleTP(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handlePR(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handlePL(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleVX(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);
    bool handleVD(const RadioCommand& command, ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial, RadioManager& radioManager);

    // Helper functions
    int parseNumericValue(const RadioCommand& command) const;
    int parsePowerState(const RadioCommand& command) const;
    int parseLevel(const RadioCommand& command) const;
    bool isValidPowerLevel(int power) const;
    bool isValidVoxDelay(int delay) const;
    
    // TX/RX helper functions (from TxRxControlCommandHandler)
    bool isValidTxMode(int mode) const;
    std::string formatTXResponse(int mode) const;
    std::string formatTQResponse(int status) const;

    // Constants
    static constexpr int MIN_POWER = 5;     // 5 watts minimum
    static constexpr int MAX_POWER = 100;   // 100 watts maximum

    static constexpr int MIN_VOX_DELAY = 0;    // 0 ms
    static constexpr int MAX_VOX_DELAY = 3000; // 3000 ms
    
    // TX modes per TS-590SG CAT specification
    static constexpr int TX_MODE_SEND = 0;       // SEND (MIC) - default TX mode
    static constexpr int TX_MODE_DATA_SEND = 1;  // DATA SEND (ACC2/USB)
    static constexpr int TX_MODE_TUNE = 2;       // TX Tune

    static constexpr const char* TAG = "TransmitterCommandHandler";
};

} // namespace radio
