#pragma once

#include "AntennaCommandHandler.h"
#include "BaseCommandHandler.h"
#include "RadioState.h"

namespace radio {

/**
 * @brief Handler for memory and channel management commands
 * 
 * Handles all interactions with the transceiver's 120 memory channels 
 * and its Quick Memory feature.
 * 
 * Commands handled:
 * - MC: Memory channel selection/recall
 * - MW: Memory write (store current settings to memory)
 * - MR: Memory read (recall settings from memory) 
 * - QI: Quick memory store (store to quick memory)
 * - QD: Quick memory delete/clear
 * - QR: Quick memory recall
 * - SV: Memory Transfer (VFO to Memory) - NEW
 */
class MemoryCommandHandler : public BaseCommandHandler {
public:
    MemoryCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    static constexpr const char* TAG = "MemoryCommandHandler";
    
    bool handleMC(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleMW(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleMR(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleQI(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  RadioManager& radioManager);
    
    bool handleQD(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  RadioManager& radioManager);
    
    bool handleQR(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleSV(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    // Memory channel parsing (handles both 3-digit and variable formats)
    struct MemoryChannel {
        int channel;
        bool valid;
    };
    
    MemoryChannel parseMemoryChannel(const RadioCommand& command) const;
    std::string formatMCResponse(int channel) const;
    std::string formatMRResponse(const RadioState& state, int channel, char splitSelector = '0') const;
    
    // Memory channel limits
    static constexpr int MIN_MEMORY_CHANNEL = 0;
    static constexpr int MAX_MEMORY_CHANNEL = 999;
    static constexpr int QUICK_MEMORY_START = 1000; // QMR channels typically start at 1000
};

} // namespace radio