#pragma once

#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for scan and sweep operation commands
 * 
 * Handles all scan, sweep and related search operations on the TS-590SG.
 * 
 * Commands handled (consolidated to avoid prefix collisions):
 * - SC: Scan status/control
 * - SS: Program Slow Scan Frequency
 * - SU: Scan group setup / step
 */
class ScanCommandHandler : public BaseCommandHandler {
public:
    ScanCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    static constexpr const char* TAG = "ScanCommandHandler";
    
    bool handleSC(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleSS(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleSU(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    // No extra helpers; parameters forwarded as-is to radio for SS/SU
};

} // namespace radio
