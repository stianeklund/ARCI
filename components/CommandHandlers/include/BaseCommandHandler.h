#pragma once

#include "ICommandHandler.h"
#include "ISerialChannel.h"
#include "ParserUtils.h"
#include <string>
#include <string_view>
#include <initializer_list>
#include <vector>
#include <functional>
#include "esp_timer.h"

namespace radio {

// Forward declarations
class RadioManager;
class RadioState;

/**
 * @brief Base implementation for command handlers
 * 
 * Provides common functionality used by most command handlers including
 * command prefix matching, parameter parsing, and radio/USB communication.
 */
class BaseCommandHandler : public ICommandHandler {
public:
    /**
     * @brief Constructor
     * @param supportedCommands List of command prefixes this handler supports (e.g., {"FA", "FB"})
     * @param description Human-readable description of what this handler does
     */
    BaseCommandHandler(std::initializer_list<std::string_view> supportedCommands, 
                      std::string_view description);

    virtual ~BaseCommandHandler() = default;

    // ICommandHandler interface
    bool canHandle(const RadioCommand& command) const override;
    std::string_view getDescription() const override { return description_; }
    std::vector<std::string_view> getPrefixes() const override;

protected:
    /**
     * @brief Check if command should be sent to radio
     * Uses the new CommandSource information to make the decision
     * @param command The command to check
     * @return true if should be sent to radio
     */
    static bool shouldSendToRadio(const RadioCommand& command) {
        return command.shouldSendToRadio();
    }
    

    /**
     * @brief Send a command string to the radio
     * @param radioSerial Radio serial interface
     * @param commandStr The command string to send (including semicolon)
     */
    static void sendToRadio(ISerialChannel& radioSerial, std::string_view commandStr);

    /**
     * @brief Send a response to USB/host
     * @param usbSerial USB serial interface
     * @param responseStr The response string to send (including semicolon)
     */
    static void sendToUSB(ISerialChannel& usbSerial, std::string_view responseStr);

    /**
     * @brief Send response to originating interface based on command source
     * Routes response back to the interface that sent the original command
     * @param command Original command to determine source interface
     * @param response Response string to send (including semicolon)
     * @param usbSerial USB serial interface
     * @param radioManager Radio manager for display interface access
     */
    static void respondToSource(const RadioCommand& command, std::string_view response,
                              ISerialChannel& usbSerial, const RadioManager& radioManager);

    /**
     * @brief Unified response routing for Answer commands using ForwardingPolicy
     * Handles all forwarding logic for radio responses in one place
     * @param command Original command (should be Answer type from Remote source)
     * @param response Response string to send (including semicolon)
     * @param usbSerial USB serial interface
     * @param radioManager Radio manager for state and display access
     */
    static void routeAnswerResponse(const RadioCommand& command, std::string_view response,
                                   ISerialChannel& usbSerial, const RadioManager& radioManager);

    /**
     * @brief Build a CAT command string from prefix and parameters
     * @param prefix Command prefix (e.g., "FA")
     * @param params Command parameters (will be concatenated)
     * @return Complete command string with semicolon
     */
    static std::string buildCommand(std::string_view prefix, std::string_view params = "");

    /**
     * @brief Parse an integer parameter from the command
     * @param command The RadioCommand containing parameters
     * @param index Parameter index (0-based)
     * @param defaultValue Value to return if parameter missing or invalid
     * @return Parsed integer value
     */
    int getIntParam(const RadioCommand& command, size_t index, int defaultValue = -1) const;

    /**
     * @brief Parse a string parameter from the command
     * @param command The RadioCommand containing parameters
     * @param index Parameter index (0-based)
     * @param defaultValue Value to return if parameter missing
     * @return String parameter value
     */
    static std::string getStringParam(const RadioCommand& command, size_t index, std::string_view defaultValue = "") ;

    /**
     * @brief Check if command is a query (Read type)
     */
    static bool isQuery(const RadioCommand& command) {
        return command.type == CommandType::Read;
    }

    /**
     * @brief Check if command is a set operation (Set type)
     */
    static bool isSet(const RadioCommand& command) {
        return command.type == CommandType::Set;
    }

    /**
     * @brief Parse an on/off (0/1) parameter from the command
     * @param command The RadioCommand containing parameters
     * @return 0 for off, 1 for on, -1 for invalid/missing
     */
    int parseOnOffValue(const RadioCommand& command) const {
        return getIntParam(command, 0, -1);
    }

    /**
     * @brief Route SET command responses to AI-enabled interfaces
     * Mimics radio Answer behavior for local SET commands to maintain UI sync.
     * Call this after processing a SET command to notify other interfaces.
     * @param originCommand Original command (to avoid echoing back to origin)
     * @param response Response string to route (including semicolon)
     * @param usbSerial USB serial interface
     * @param radioManager Radio manager for state and display access
     */
    static void routeSetCommandToAIInterfaces(const RadioCommand& originCommand,
                                              std::string_view response,
                                              ISerialChannel& usbSerial,
                                              const RadioManager& radioManager);

    /**
     * @brief Format a CAT response with prefix and 2-digit zero-padded value
     * @param prefix Command prefix (e.g., "TN", "CN")
     * @param value Integer value to format (0-99)
     * @return Formatted response string with semicolon
     */
    static std::string formatResponse2D(std::string_view prefix, int value) {
        std::string result;
        result.reserve(prefix.size() + 3); // 2 digits + semicolon
        result.append(prefix);
        if (value >= 0 && value <= 99) {
            result.push_back('0' + (value / 10));
            result.push_back('0' + (value % 10));
        } else {
            // Fallback for out-of-range
            result.append(std::to_string(value));
        }
        result.push_back(';');
        return result;
    }

    /**
     * @brief Format a CAT response with prefix and 3-digit zero-padded value
     * @param prefix Command prefix (e.g., "AG", "RG")
     * @param value Integer value to format (0-999)
     * @return Formatted response string with semicolon
     */
    static std::string formatResponse3D(const std::string_view prefix, const int value) {
        // Performance: Use constexpr lookup table for faster digit formatting
        static constexpr char digits[] = "000001002003004005006007008009"
                                        "010011012013014015016017018019"
                                        "020021022023024025026027028029"
                                        "030031032033034035036037038039"
                                        "040041042043044045046047048049"
                                        "050051052053054055056057058059"
                                        "060061062063064065066067068069"
                                        "070071072073074075076077078079"
                                        "080081082083084085086087088089"
                                        "090091092093094095096097098099";
        
        std::string result;
        result.reserve(prefix.size() + 4); // 3 digits + semicolon
        result.append(prefix);
        
        if (value >= 0 && value <= 999) {
            // Fast path: use lookup table for values 0-99, manual for 100-999
            if (value < 100) {
                result.append(&digits[value * 3], 3);
            } else {
                result.push_back('0' + (value / 100));
                const int remainder = value % 100;
                result.push_back('0' + (remainder / 10));
                result.push_back('0' + (remainder % 10));
            }
        } else {
            // Fallback for out-of-range values
            result.append(std::to_string(value));
        }
        
        result.push_back(';');
        return result;
    }

    /**
     * @brief Format a CAT response with prefix, subcode, and 3-digit value (e.g., AG0255;)
     * @param prefix Command prefix
     * @param subcode Single character/digit subcode (0-9)
     * @param value Integer value to format (0-999)
     * @return Formatted response string with semicolon
     */
    static std::string formatResponseSub3D(const std::string_view prefix, const int subcode, const int value) {
        std::string result;
        result.reserve(prefix.size() + 5); // 1 subcode + 3 digits + semicolon
        result.append(prefix);
        result.push_back('0' + subcode); // Faster than std::to_string for single digits
        
        // Reuse the optimized 3-digit formatting from formatResponse3D
        if (value >= 0 && value <= 999) {
            if (value < 100) {
                result.push_back('0');
                if (value < 10) {
                    result.push_back('0');
                    result.push_back('0' + value);
                } else {
                    result.push_back('0' + (value / 10));
                    result.push_back('0' + (value % 10));
                }
            } else {
                result.push_back('0' + (value / 100));
                const int remainder = value % 100;
                result.push_back('0' + (remainder / 10));
                result.push_back('0' + (remainder % 10));
            }
        } else {
            // Fallback for out-of-range values
            result.append(std::to_string(value));
        }
        
        result.push_back(';');
        return result;
    }

    /**
     * @brief Format a CAT response with prefix and signed 4-digit zero-padded value
     * @param prefix Command prefix
     * @param value Signed integer value
     * @return Formatted response string with sign and semicolon
     */
    static std::string formatResponseSigned4D(const std::string_view prefix, const int value) {
        std::string result;
        result.reserve(prefix.size() + 6); // sign + 4 digits + semicolon
        result.append(prefix);
        result.push_back(value >= 0 ? '+' : '-');
        const int abs_value = std::abs(value);
        // Fast digit extraction without std::to_string
        result.push_back('0' + (abs_value / 1000) % 10);
        result.push_back('0' + (abs_value / 100) % 10);
        result.push_back('0' + (abs_value / 10) % 10);
        result.push_back('0' + abs_value % 10);
        result.push_back(';');
        return result;
    }

    /**
     * @brief Format a CAT response with prefix and two 3-digit zero-padded values
     * @param prefix Command prefix
     * @param value1 First integer value
     * @param value2 Second integer value
     * @return Formatted response string with semicolon
     */
    static std::string formatResponseDual3D(const std::string_view prefix, const int value1, const int value2) {
        std::string result;
        result.reserve(prefix.size() + 7); // 3 + 3 digits + semicolon
        result.append(prefix);
        // Fast 3-digit formatting without std::to_string
        result.push_back('0' + (value1 / 100) % 10);
        result.push_back('0' + (value1 / 10) % 10);
        result.push_back('0' + value1 % 10);
        result.push_back('0' + (value2 / 100) % 10);
        result.push_back('0' + (value2 / 10) % 10);
        result.push_back('0' + value2 % 10);
        result.push_back(';');
        return result;
    }

    /**
     * @brief Format a CAT response with prefix and 4-digit zero-padded value
     * @param prefix Command prefix
     * @param value Integer value to format
     * @return Formatted response string with semicolon
     */
    static std::string formatResponse4D(const std::string_view prefix, const int value) {
        std::string result;
        result.reserve(prefix.size() + 5); // 4 digits + semicolon
        result.append(prefix);
        // Fast 4-digit formatting without std::to_string
        result.push_back('0' + (value / 1000) % 10);
        result.push_back('0' + (value / 100) % 10);
        result.push_back('0' + (value / 10) % 10);
        result.push_back('0' + value % 10);
        result.push_back(';');
        return result;
    }

    /**
     * @brief Check if cached data for a command is fresh (within TTL)
     * @param radioManager Reference to radio manager for state access
     * @param command CAT command prefix (e.g., "FA", "FB", "MD")  
     * @param ttlUs TTL in microseconds (default: 3 seconds)
     * @return true if cache is fresh (within TTL)
     */
    static bool isCacheFresh(const RadioManager& radioManager, std::string_view command,
                             uint64_t ttlUs = 3000000);

    /**
     * @brief Standardized local READ handling (Option A):
     *        - If cache is fresh: reply immediately using provided builder
     *        - If cache is stale/missing: record query and send READ to radio; no immediate reply
     *        For non-local sources, forwards the READ to radio when appropriate.
     * @param command Incoming READ command
     * @param radioSerial Radio serial interface
     * @param usbSerial USB serial interface
     * @param radioManager Radio manager reference
     * @param key Cache key/prefix (e.g., "IF", "RM")
     * @param ttlUs TTL in microseconds to consider cache fresh
     * @param buildResponse Functor that builds a response string from current RadioState
     * @param readParam Optional parameter for the READ command (e.g., "0" for SM0;)
     * @return true when handled (always returns true)
     */
    static bool handleLocalQueryStandard(const RadioCommand& command,
                                         ISerialChannel& radioSerial,
                                         ISerialChannel& usbSerial,
                                         RadioManager& radioManager,
                                         std::string_view key,
                                         uint64_t ttlUs,
                                         const std::function<std::string(const RadioState&)>& buildResponse,
                                         std::string_view readParam = {});

public:
    // TTL constants for different command categories (in microseconds)
    // Internal polling has been removed; these are retained only to guide
    // cache freshness decisions for local USB responses.
    static constexpr uint64_t TTL_STATIC_CONFIG = 30000000;  // 30s - Menu/config commands (EX, MF, AN)
    static constexpr uint64_t TTL_STATUS = 5000000;          // 5s - Status commands (RM, TO, TP, TS, MG, ML)
    static constexpr uint64_t TTL_DYNAMIC = 2000000;         // 2s - Dynamic operational (SD, MK)  
    static constexpr uint64_t TTL_MID_FREQ = 950000;         // 0.95s - Mid-frequency polling (MD)
    static constexpr uint64_t TTL_REALTIME = 500000;         // 0.5s - Real-time data (SM meters)
    static constexpr uint64_t TTL_HIGH_FREQ = 450000;        // 0.45s - High-frequency polling (IF)
    static constexpr uint64_t TTL_BURST = 20000;             // 0.02s - kept for compatibility

private:
    static constexpr size_t MAX_COMMANDS = 32; // Maximum commands per handler (ensure no truncation)
    std::string_view supportedCommands_[MAX_COMMANDS];
    size_t numCommands_;
    std::string description_;

    static constexpr const char *TAG = "BaseCommandHandler";
};

} // namespace radio
