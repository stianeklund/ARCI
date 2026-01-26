#include "MenuConfigCommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"

namespace radio {

MenuConfigCommandHandler::MenuConfigCommandHandler()
    : BaseCommandHandler({"MF", "EX", "ES"}, "Menu & Configuration Commands"),
      extendedCommandHandler_(std::make_unique<ExtendedCommandHandler>()) {
}

bool MenuConfigCommandHandler::handleCommand(const RadioCommand& command,
                                           ISerialChannel& radioSerial,
                                           ISerialChannel& usbSerial,
                                           RadioManager& radioManager) {
    ESP_LOGV(TAG, "Handling menu/config command: %s", command.command.c_str());
    
    if (command.command == "MF") {
        return handleMF(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "EX") {
        return extendedCommandHandler_->handleCommand(command, radioSerial, usbSerial, radioManager);
    }
    if (command.command == "ES") {
        return handleES(command, radioSerial, usbSerial, radioManager);
    }

    return false;
}

bool MenuConfigCommandHandler::handleMF(const RadioCommand& command,
                                       ISerialChannel& radioSerial,
                                       ISerialChannel& usbSerial,
                                       RadioManager& radioManager) {
    // MF: Menu bank select (A/B)
    auto& state = radioManager.getState();
    
    if (isQuery(command)) {
        if (command.isCatClient()) {
            // For local queries, check if cached data is fresh
            if (isCacheFresh(radioManager, "MF", TTL_STATIC_CONFIG)) {
                // Cache is fresh - respond with cached data immediately
                int bank = state.menuBank;
                std::string response = formatMFResponse(bank);
                respondToSource(command, response, usbSerial, radioManager);
            } else {
                // Cache is stale - check if we have valid cached data
                int bank = state.menuBank;
                if (bank >= 0 && bank <= 1) {
                    // Have valid cached data - return immediately + background refresh
                    std::string response = formatMFResponse(bank);
                    respondToSource(command, response, usbSerial, radioManager);
                    // Background refresh to update cache
                    radioManager.getState().queryTracker.recordQuery("MF", esp_timer_get_time());
                    sendToRadio(radioSerial, buildCommand("MF"));
                } else {
                    // No valid cached data - query radio first
                    radioManager.getState().queryTracker.recordQuery("MF", esp_timer_get_time());
                    sendToRadio(radioSerial, buildCommand("MF"));
                }
            }
        } else {
            // Non-local query - forward to radio
            radioManager.getState().queryTracker.recordQuery("MF", esp_timer_get_time());
            sendToRadio(radioSerial, buildCommand("MF"));
        }
        return true;
    }
    
    if (isSet(command)) {
        int bank = getIntParam(command, 0, -1);
        if (!isValidMenuBank(bank)) {
            ESP_LOGW(TAG, "Invalid menu bank: %d", bank);
            return false;
        }
        
        // Update local state
        state.menuBank = bank;
        
        // Update cache timestamp when state changes
        uint64_t currentTime = esp_timer_get_time();
        radioManager.getState().commandCache.update("MF", currentTime);
        
        ESP_LOGD(TAG, "Set menu bank to %s", bank == MF_BANK_A ? "A" : "B");
        
        if (shouldSendToRadio(command)) {
            std::string cmdStr = formatMFResponse(bank);
            sendToRadio(radioSerial, cmdStr);
        }
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        int bank = getIntParam(command, 0, MF_BANK_A);
        if (bank >= 0 && bank <= 1) {
            // Update state from radio response
            state.menuBank = bank;
            
            // Update cache timestamp for AI mode compatibility
            uint64_t currentTime = esp_timer_get_time();
            radioManager.getState().commandCache.update("MF", currentTime);
            
            ESP_LOGD(TAG, "Updated menu bank from radio: %s", bank == MF_BANK_A ? "A" : "B");
        }
        
        {
            std::string response = formatMFResponse(bank);
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }
    
    return false;
}



bool MenuConfigCommandHandler::handleES(const RadioCommand& command,
                                       ISerialChannel& radioSerial,
                                       ISerialChannel& usbSerial,
                                       RadioManager& radioManager) {
    // ES: Advanced startup option (TS-590S)
        if (isQuery(command)) {
        if (shouldSendToRadio(command)) {
            radioManager.getState().queryTracker.recordQuery("ES", esp_timer_get_time());
            sendToRadio(radioSerial, buildCommand("ES"));
        } else {
            // Return default startup option (normal)
            std::string response = formatESResponse(ES_NORMAL);
            respondToSource(command, response, usbSerial, radioManager);
        }
        return true;
    }
    
    if (isSet(command)) {
        int option = getIntParam(command, 0, -1);
        if (option != ES_NORMAL && option != ES_ADVANCED) {
            ESP_LOGW(TAG, "Invalid ES startup option: %d", option);
            return false;
        }
        
        ESP_LOGD(TAG, "Set startup option to %s", option == ES_NORMAL ? "NORMAL" : "ADVANCED");
        
        if (shouldSendToRadio(command)) {
            std::string cmdStr = formatESResponse(option);
            sendToRadio(radioSerial, cmdStr);
        }
        return true;
    }
    
    if (command.type == CommandType::Answer) {
        {
            int option = getIntParam(command, 0, ES_NORMAL);
            std::string response = formatESResponse(option);
            routeAnswerResponse(command, response, usbSerial, radioManager);
        }
        return true;
    }
    
    return false;
}

// =============================================================================
// Helper functions
// =============================================================================

bool MenuConfigCommandHandler::isValidMenuBank(int bank) const {
    return bank == MF_BANK_A || bank == MF_BANK_B;
}

bool MenuConfigCommandHandler::isValidMenuNumber(int menuNum) const {
    return menuNum >= MIN_MENU_NUMBER && menuNum <= MAX_MENU_NUMBER;
}

std::string MenuConfigCommandHandler::formatMFResponse(int bank) const {
    return buildCommand("MF", std::to_string(bank));
}

std::string MenuConfigCommandHandler::formatEXResponse(std::string_view params) const {
    return buildCommand("EX", params);
}

std::string MenuConfigCommandHandler::formatESResponse(int option) const {
    return buildCommand("ES", std::to_string(option));
}

} // namespace radio
