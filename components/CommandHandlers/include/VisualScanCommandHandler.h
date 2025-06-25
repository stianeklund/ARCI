#pragma once

#include "BaseCommandHandler.h"

namespace radio {

/**
 * @brief Handler for visual scan commands
 * 
 * Handles visual scan functionality for the TS-590SG according to specification.
 * 
 * Commands handled:
 * - VS0: Visual Scan start/stop/pause status
 * - VS1: Set Visual Scan center frequency
 * - VS2: Set Visual Scan span
 * - VS3: Read Visual Scan frequencies and span
 * - VS4: Read Visual Scan sweep frequency and signal level
 */
class VisualScanCommandHandler : public BaseCommandHandler {
public:
    VisualScanCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    static constexpr const char* TAG = "VisualScanCommandHandler";
    
    bool handleVS(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleVS0(const RadioCommand& command,
                   ISerialChannel& radioSerial,
                   ISerialChannel& usbSerial,
                   RadioManager& radioManager);
    
    bool handleVS1(const RadioCommand& command,
                   ISerialChannel& radioSerial,
                   ISerialChannel& usbSerial,
                   RadioManager& radioManager);
    
    bool handleVS2(const RadioCommand& command,
                   ISerialChannel& radioSerial,
                   ISerialChannel& usbSerial,
                   RadioManager& radioManager);
    
    bool handleVS3(const RadioCommand& command,
                   ISerialChannel& radioSerial,
                   ISerialChannel& usbSerial,
                   RadioManager& radioManager);
    
    bool handleVS4(const RadioCommand& command,
                   ISerialChannel& radioSerial,
                   ISerialChannel& usbSerial,
                   RadioManager& radioManager);
    
    // Helper functions
    int parseVS0Status(const RadioCommand& command) const;
    uint64_t parseVS1Frequency(const RadioCommand& command) const;
    int parseVS2Span(const RadioCommand& command) const;
    
    // Constants for VS0 (Visual Scan status)
    static constexpr int VS0_OFF = 0;
    static constexpr int VS0_ON = 1;
    static constexpr int VS0_PAUSE = 2;
    static constexpr int VS0_RESTART = 3;
    
    // Constants for VS2 (Visual Scan span)
    static constexpr int VS2_MIN_SPAN = 0;
    static constexpr int VS2_MAX_SPAN = 6;
};

} // namespace radio