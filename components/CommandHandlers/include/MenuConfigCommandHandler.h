#pragma once

#include "BaseCommandHandler.h"
#include "ExtendedCommandHandler.h"
#include <memory>

namespace radio {

/**
 * @brief Handler for menu and configuration commands
 * 
 * Commands for programmatic access to the transceiver's menu system.
 * 
 * Commands handled:
 * - MF: Menu bank select (A/B)
 * - EX: Extended menu item access (TS-590SG)
 * - ES: Advanced startup option (TS-590S)
 * 
 * Note: Other commands moved to appropriate categories:
 * - LK moved to InterfaceSystemCommandHandler
 * - LM moved to VoiceMessageCommandHandler  
 * - MG/ML moved to GainLevelCommandHandler
 */

class MenuConfigCommandHandler final : public BaseCommandHandler {
public:
    MenuConfigCommandHandler();

    // ICommandHandler interface
    bool handleCommand(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager) override;

private:
    std::unique_ptr<ExtendedCommandHandler> extendedCommandHandler_;
    static constexpr const char* TAG = "MenuConfigCommandHandler";
    
    bool handleMF(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleEX(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    bool handleES(const RadioCommand& command,
                  ISerialChannel& radioSerial,
                  ISerialChannel& usbSerial,
                  RadioManager& radioManager);
    
    // Helper functions
    [[nodiscard]] bool isValidMenuBank(int bank) const;
    [[nodiscard]] bool isValidMenuNumber(int menuNum) const;
    [[nodiscard]] std::string formatMFResponse(int bank) const;
    [[nodiscard]] std::string formatEXResponse(std::string_view params) const;
    [[nodiscard]] std::string formatESResponse(int option) const;
    
    // Constants for parameter validation
    static constexpr int MF_BANK_A = 0;      // Menu bank A
    static constexpr int MF_BANK_B = 1;      // Menu bank B
    
    static constexpr int MIN_MENU_NUMBER = 0;
    static constexpr int MAX_MENU_NUMBER = 99;
    
    static constexpr int ES_NORMAL = 0;      // Normal startup
    static constexpr int ES_ADVANCED = 1;    // Advanced startup
};

} // namespace radio