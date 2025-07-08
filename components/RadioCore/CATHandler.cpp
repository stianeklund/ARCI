#include "CATHandler.h"
#include "CatParser.h"
#include "CommandDispatcher.h"
#include "RadioManager.h"
#include "ISerialChannel.h"
#include "esp_log.h"
#include <memory>

namespace radio {

CATHandler::CATHandler(CommandDispatcher& dispatcher, RadioManager& manager,
                       ISerialChannel& radioSerial, ISerialChannel& usbSerial, const CommandSource source)
    : dispatcher_(dispatcher), manager_(manager), radioSerial_(radioSerial), 
      usbSerial_(usbSerial), source_(source) {
    const char* sourceStr = source == CommandSource::UsbCdc0 ? "Usb0" :
                            source == CommandSource::UsbCdc1 ? "Usb1" :
                            source == CommandSource::Display ? "Display" :
                            source == CommandSource::Panel ? "Panel" : "Remote";
    ESP_LOGD(CATHandler::TAG, "CATHandler initialized in %s mode", sourceStr);
    
    // Set error callback (string_view to avoid copies)
    unifiedParser_.setErrorCallback([this](std::string_view error) {
        this->reportError(error);
    });
}

CATHandler::~CATHandler() = default;

bool CATHandler::parseMessage(const std::string_view message) {
    if (message.empty()) {
        return false;
    }

    logMessage(message);
    stats_.totalMessagesParsed++;

    // Track if any commands were handled
    bool anyCommandHandled = false;

    // Use the source directly
    const CommandSource source = source_;
    
    // Parse message using unified parser
    unifiedParser_.parseMessage(message, source, [this, &anyCommandHandled](const RadioCommand& command) {
        // Forward command directly to CommandDispatcher

        if (dispatcher_.dispatchCommand(command, radioSerial_, usbSerial_, manager_)) {
            stats_.totalCommandsParsed++;
            anyCommandHandled = true;
            if (command.type == CommandType::Read) {
                stats_.queryCommands++;
            } else {
                stats_.setCommands++;
            }
        } else {
            stats_.unknownCommands++;
            ESP_LOGW(CATHandler::TAG, "Command not handled by dispatcher: %s", command.describe().c_str());
        }
    });
    
    // Update our stats from the parser stats
    const auto& parserStats = unifiedParser_.getStatistics();
    stats_.parseErrors = parserStats.parseErrors;
    
    return anyCommandHandled;
}

std::optional<RadioCommand> CATHandler::parseFrame(const std::string_view frame) const {
    const CommandSource source = source_;
    
    // Create a temporary parser instance to parse single frame
    CatParser tempParser;
    std::optional<RadioCommand> result;
    
    tempParser.parseMessage(frame, source, [&result](const RadioCommand& command) {
        result = command;
    });
    
    return result;
}

void CATHandler::reportError(std::string_view error) const {
    const char* sourceStr = source_ == CommandSource::UsbCdc0 ? "Usb0" :
                            source_ == CommandSource::UsbCdc1 ? "Usb1" :
                            source_ == CommandSource::Display ? "Display" :
                            source_ == CommandSource::Panel ? "Panel" : "Remote";
    ESP_LOGW(CATHandler::TAG, "%s mode error: %.*s", sourceStr, static_cast<int>(error.size()), error.data());
    if (errorCallback_) {
        errorCallback_(error);
    }
}

void CATHandler::logMessage(std::string_view message) const {
    const char* sourceStr = source_ == CommandSource::UsbCdc0 ? "Usb0" :
                            source_ == CommandSource::UsbCdc1 ? "Usb1" :
                            source_ == CommandSource::Display ? "Display" :
                            source_ == CommandSource::Panel ? "Panel" : "Remote";
    ESP_LOGV(CATHandler::TAG, "%s message: '%.*s'", sourceStr,
             static_cast<int>(message.length()), message.data());
}

} // namespace radio
