#pragma once

#include "RadioCommand.h"
#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include <atomic>
#include "rtos_mutex.h"

class ISerialChannel;

namespace radio {
    class RadioManager;

    // Forward declarations
class ICommandHandler;
using CommandHandlerPtr = std::unique_ptr<ICommandHandler>;

/**
 * @brief Statistics for command dispatching with error response tracking
 */
struct DispatcherStatistics {
    size_t totalCommandsDispatched{0};
    size_t commandsHandled{0};
    size_t commandsUnhandled{0};
    size_t handlerErrors{0};
    std::atomic<int32_t> currentProcessingDepth{0};  // Current commands being processed simultaneously (signed to catch underflow)
    std::atomic<int32_t> maxProcessingDepth{0};      // Peak processing depth observed
    
    // Error response tracking for diagnostics
    size_t totalErrorResponses{0};
    size_t questionMarkErrors{0};        // "?;" responses
    size_t eErrors{0};                   // "E;" responses  
    size_t oErrors{0};                   // "O;" responses
    uint64_t lastErrorTime{0};           // Last error timestamp (microseconds)
    uint64_t lastCommandTime{0};         // Last command sent timestamp (microseconds)
    std::string lastCommandBeforeError;  // Command that preceded the error
    uint64_t lastCommandBeforeErrorTime{0}; // Timestamp of the command that preceded the error (microseconds)
    uint64_t averageErrorInterval{0};    // Running average of error intervals
    size_t errorBursts{0};               // Number of error bursts detected

    // Default constructor (required since we have custom copy constructor)
    DispatcherStatistics() = default;

    // Copy constructor to handle atomic members
    DispatcherStatistics(const DispatcherStatistics& other) 
        : totalCommandsDispatched(other.totalCommandsDispatched),
          commandsHandled(other.commandsHandled),
          commandsUnhandled(other.commandsUnhandled),
          handlerErrors(other.handlerErrors),
          currentProcessingDepth(other.currentProcessingDepth.load()),
          maxProcessingDepth(other.maxProcessingDepth.load()),
          totalErrorResponses(other.totalErrorResponses),
          questionMarkErrors(other.questionMarkErrors),
          eErrors(other.eErrors),
          oErrors(other.oErrors),
          lastErrorTime(other.lastErrorTime),
          lastCommandTime(other.lastCommandTime),
          lastCommandBeforeError(other.lastCommandBeforeError),
          lastCommandBeforeErrorTime(other.lastCommandBeforeErrorTime),
          averageErrorInterval(other.averageErrorInterval),
          errorBursts(other.errorBursts) {}

    // Assignment operator to handle atomic members
    DispatcherStatistics& operator=(const DispatcherStatistics& other) {
        if (this != &other) {
            totalCommandsDispatched = other.totalCommandsDispatched;
            commandsHandled = other.commandsHandled;
            commandsUnhandled = other.commandsUnhandled;
            handlerErrors = other.handlerErrors;
            currentProcessingDepth.store(other.currentProcessingDepth.load());
            maxProcessingDepth.store(other.maxProcessingDepth.load());
            totalErrorResponses = other.totalErrorResponses;
            questionMarkErrors = other.questionMarkErrors;
            eErrors = other.eErrors;
            oErrors = other.oErrors;
            lastErrorTime = other.lastErrorTime;
            lastCommandTime = other.lastCommandTime;
            lastCommandBeforeError = other.lastCommandBeforeError;
            lastCommandBeforeErrorTime = other.lastCommandBeforeErrorTime;
            averageErrorInterval = other.averageErrorInterval;
            errorBursts = other.errorBursts;
        }
        return *this;
    }

    void reset() {
        totalCommandsDispatched = 0;
        commandsHandled = 0;
        commandsUnhandled = 0;
        handlerErrors = 0;
        currentProcessingDepth.store(0);
        maxProcessingDepth.store(0);
        totalErrorResponses = 0;
        questionMarkErrors = 0;
        eErrors = 0;
        oErrors = 0;
        lastErrorTime = 0;
        lastCommandTime = 0;
        lastCommandBeforeError.clear();
        lastCommandBeforeErrorTime = 0;
        averageErrorInterval = 0;
        errorBursts = 0;
    }
    
    void recordError(std::string_view errorType, std::string_view lastCmd, const uint64_t currentTime) {
        totalErrorResponses++;
        if (errorType == "?") questionMarkErrors++;
        else if (errorType == "E") eErrors++;  
        else if (errorType == "O") oErrors++;
        
        // Calculate interval and detect patterns
        if (lastErrorTime > 0) {
            uint64_t interval = currentTime - lastErrorTime;
            if (averageErrorInterval == 0) {
                averageErrorInterval = interval;
            } else {
                // Simple moving average
                averageErrorInterval = (averageErrorInterval * 3 + interval) / 4;
            }
            
            // Detect error bursts (errors < 5 seconds apart)
            if (interval < 5000000) {
                errorBursts++;
            }
        }
        
        lastErrorTime = currentTime;
        lastCommandBeforeError = lastCmd;
        lastCommandBeforeErrorTime = lastCommandTime;  // Store when the problematic command was sent
    }
    
    void recordCommandSent(const uint64_t currentTime) {
        lastCommandTime = currentTime;
    }
};

/**
 * @brief Central dispatcher for RadioCommand objects
 * 
 * Replaces the massive switch statement in RadioManager by delegating
 * command processing to specialized handlers registered for specific
 * command types or prefixes.
 */
class CommandDispatcher {
public:
    CommandDispatcher();
    ~CommandDispatcher();

    // Non-copyable, non-movable for now
    CommandDispatcher(const CommandDispatcher&) = delete;
    CommandDispatcher& operator=(const CommandDispatcher&) = delete;
    CommandDispatcher(CommandDispatcher&&) = delete;
    CommandDispatcher& operator=(CommandDispatcher&&) = delete;

    /**
     * @brief Register a command handler for specific commands
     * @param handler Unique pointer to the handler (dispatcher takes ownership)
     * @return true if registration successful
     */
    bool registerHandler(CommandHandlerPtr handler);

    /**
     * @brief Dispatch a command to the appropriate handler
     * @param command The RadioCommand to process
     * @param radioSerial Serial interface to the radio hardware
     * @param usbSerial Serial interface to the USB/host
     * @param radioManager Reference to the radio manager for state access
     * @return true if command was handled successfully, false if no handler found or error
     *
     * Hot path optimization: Marked as frequently called for better code placement
     */
    bool dispatchCommand(const RadioCommand& command,
                        ISerialChannel& radioSerial,
                        ISerialChannel& usbSerial,
                        RadioManager& radioManager) __attribute__((hot));

    /**
     * @brief Get current dispatcher statistics (thread-safe snapshot)
     * @return Copy of statistics, safe to access from any task
     */
    DispatcherStatistics getStatistics() const;

    /**
     * @brief Reset statistics (thread-safe)
     */
    void resetStatistics();

    /**
     * @brief Get list of registered handlers (for debugging/diagnostics)
     */
    std::vector<std::string> getRegisteredHandlers() const;

private:
    // Compact perfect hash table for O(1) command lookup with zero allocations
    // CAT commands are always 2-char uppercase ASCII (e.g., "FA", "MD", "PS")
    // Observation: All commands use chars A-Z (26 letters) in both positions
    // Hash: (c1-'A')*26 + (c2-'A') maps AA..ZZ to 0..675 (26*26 = 676 entries)
    // This gives us a 676-entry table (5.4KB) vs 64KB for full ASCII space
    // Using direct array indexing instead of std::unordered_map eliminates:
    //  - Hash computation overhead (~10-15 CPU cycles per lookup)
    //  - Hash collision handling
    //  - Heap allocations for bucket storage
    //  - Cache misses from pointer chasing
    static constexpr size_t HASH_TABLE_SIZE = 26 * 26; // 676 entries, ~5.4KB
    static constexpr size_t compactHash(char c1, char c2) {
        // Assumes uppercase A-Z; for safety, mask to valid range
        const size_t idx1 = (static_cast<unsigned char>(c1) - 'A') & 0x1F; // 0-25
        const size_t idx2 = (static_cast<unsigned char>(c2) - 'A') & 0x1F; // 0-25
        return idx1 * 26 + idx2;
    }

    std::vector<CommandHandlerPtr> handlers_;
    std::array<ICommandHandler*, HASH_TABLE_SIZE> commandMap_{};  // Zero-initialized
    DispatcherStatistics stats_;
    mutable RtosMutex statsMutex_;  // Protects stats_ string fields for thread-safe access

    static constexpr const char* TAG = "CommandDispatcher";
};

} // namespace radio
