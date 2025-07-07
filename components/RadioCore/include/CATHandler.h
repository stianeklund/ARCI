
#pragma once

#include "RadioCommand.h"
#include <string_view>
#include <functional>
#include <optional>
#include <memory>
#include "CatParser.h"

class ISerialChannel;

namespace radio {

class RadioManager;
class CommandDispatcher;

/**
 * @brief Unified CAT handler that processes commands from both local (USB) and remote (radio) sources
 * 
 * This replaces the old separate LocalCATHandler and RadioCATHandler with a single,
 * unified handler that uses the CatParser and CommandDispatcher system.
 */
class CATHandler {
public:
    CATHandler(CommandDispatcher& dispatcher, RadioManager& manager, 
               ISerialChannel& radioSerial, ISerialChannel& usbSerial, CommandSource source);
    ~CATHandler();

    CATHandler(const CATHandler&) = delete;
    CATHandler& operator=(const CATHandler&) = delete;
    CATHandler(CATHandler&&) = delete;
    CATHandler& operator=(CATHandler&&) = delete;

    /**
     * @brief Parse a message containing one or more CAT commands/responses
     * @param message The raw CAT message string
     * @return true if any commands in the message were handled by a command handler
     *
     * IRAM_ATTR: This is a hot-path function called for every CAT message.
     * Placing it in IRAM (vs flash) reduces latency by ~20% on ESP32-S3.
     */
    bool parseMessage(std::string_view message) __attribute__((hot));

    /**
     * @brief Set error callback for parse errors
     */
    void setErrorCallback(std::function<void(std::string_view)> callback) {
        errorCallback_ = std::move(callback);
    }

    struct Statistics {
        size_t totalMessagesParsed{0};
        size_t totalCommandsParsed{0};
        size_t queryCommands{0};
        size_t setCommands{0};
        size_t parseErrors{0};
        size_t unknownCommands{0};
        size_t multiCommandMessages{0};
    };

    [[nodiscard]] Statistics getStatistics() const { return stats_; }
    void resetStatistics() { stats_ = Statistics{}; }
    
    /**
     * @brief Get reference to the RadioManager
     * @return Reference to RadioManager
     */
    RadioManager& getRadioManager() { return manager_; }

private:
    /**
     * @brief Parse a single CAT frame (for compatibility/testing)
     */
    std::optional<RadioCommand> parseFrame(std::string_view frame) const;

    CommandDispatcher& dispatcher_;
    RadioManager& manager_;
    ISerialChannel& radioSerial_;
    ISerialChannel& usbSerial_;
    CommandSource source_;
    Statistics stats_;
    std::function<void(std::string_view)> errorCallback_;
    CatParser unifiedParser_;

    void reportError(std::string_view error) const;
    void logMessage(std::string_view message) const;

    static constexpr const char* TAG = "CATHandler";
};

} // namespace radio
