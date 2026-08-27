#pragma once

#include "RadioCommand.h"
#include <vector>
#include <unordered_map>
#include <string>
#include <string_view>
#include <memory>
#include <atomic>
#include <array>
#include <cstring>
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
    // Plain counters are written from multiple tasks without the mutex, so they are
    // atomic. statsMutex_ (in CommandDispatcher) covers only the non-atomic error /
    // timing / string fields below.
    std::atomic<uint32_t> totalCommandsDispatched{0};
    std::atomic<uint32_t> commandsHandled{0};
    std::atomic<uint32_t> commandsUnhandled{0};
    std::atomic<uint32_t> handlerErrors{0};
    std::atomic<int32_t> currentProcessingDepth{0};  // Current commands being processed simultaneously (signed to catch underflow)
    std::atomic<int32_t> maxProcessingDepth{0};      // Peak processing depth observed
    
    // Error response tracking for diagnostics
    size_t totalErrorResponses{0};
    size_t questionMarkErrors{0};        // "?;" responses
    size_t eErrors{0};                   // "E;" responses  
    size_t oErrors{0};                   // "O;" responses
    uint64_t lastErrorTime{0};           // Last error timestamp (microseconds)
    uint64_t lastCommandTime{0};         // Last command sent timestamp (microseconds)
    static constexpr size_t CMD_BUF_SIZE = 32;  // CAT commands are short (typically 3-15 chars)
    char lastCommandBeforeError[CMD_BUF_SIZE]{};  // Command that preceded the error (fixed buffer, no heap)
    char lastCommandSource[CMD_BUF_SIZE]{};       // Source interface of the command (fixed buffer, no heap)
    uint64_t lastCommandBeforeErrorTime{0}; // Timestamp of the command that preceded the error (microseconds)
    uint64_t firstErrorTime{0};          // First error timestamp, used for the true overall interval
    uint64_t averageErrorInterval{0};    // Overall mean interval between errors
    size_t errorBursts{0};               // Number of distinct <5s burst groups detected
    bool inErrorBurst{false};            // Prevents counting every error inside one burst as a new burst
    static constexpr size_t ERROR_RATE_SAMPLE_COUNT = 10;
    static constexpr uint64_t ERROR_RATE_WINDOW_US = 5'000'000;
    std::array<uint64_t, ERROR_RATE_SAMPLE_COUNT> recentErrorTimes{};
    size_t recentErrorWriteIndex{0};
    size_t recentErrorSamples{0};
    bool highErrorRateActive{false};

    // Helper to get string_view of the fixed buffers
    std::string_view lastCommandBeforeErrorView() const { return {lastCommandBeforeError}; }
    std::string_view lastCommandSourceView() const { return {lastCommandSource}; }

    // Default constructor (required since we have custom copy constructor)
    DispatcherStatistics() = default;

    // Copy constructor to handle atomic members
    DispatcherStatistics(const DispatcherStatistics& other)
        : totalCommandsDispatched(other.totalCommandsDispatched.load()),
          commandsHandled(other.commandsHandled.load()),
          commandsUnhandled(other.commandsUnhandled.load()),
          handlerErrors(other.handlerErrors.load()),
          currentProcessingDepth(other.currentProcessingDepth.load()),
          maxProcessingDepth(other.maxProcessingDepth.load()),
          totalErrorResponses(other.totalErrorResponses),
          questionMarkErrors(other.questionMarkErrors),
          eErrors(other.eErrors),
          oErrors(other.oErrors),
          lastErrorTime(other.lastErrorTime),
          lastCommandTime(other.lastCommandTime),
          lastCommandBeforeErrorTime(other.lastCommandBeforeErrorTime),
          firstErrorTime(other.firstErrorTime),
          averageErrorInterval(other.averageErrorInterval),
          errorBursts(other.errorBursts),
          inErrorBurst(other.inErrorBurst),
          recentErrorTimes(other.recentErrorTimes),
          recentErrorWriteIndex(other.recentErrorWriteIndex),
          recentErrorSamples(other.recentErrorSamples),
          highErrorRateActive(other.highErrorRateActive) {
        std::memcpy(lastCommandBeforeError, other.lastCommandBeforeError, CMD_BUF_SIZE);
        std::memcpy(lastCommandSource, other.lastCommandSource, CMD_BUF_SIZE);
    }

    // Assignment operator to handle atomic members
    DispatcherStatistics& operator=(const DispatcherStatistics& other) {
        if (this != &other) {
            totalCommandsDispatched.store(other.totalCommandsDispatched.load());
            commandsHandled.store(other.commandsHandled.load());
            commandsUnhandled.store(other.commandsUnhandled.load());
            handlerErrors.store(other.handlerErrors.load());
            currentProcessingDepth.store(other.currentProcessingDepth.load());
            maxProcessingDepth.store(other.maxProcessingDepth.load());
            totalErrorResponses = other.totalErrorResponses;
            questionMarkErrors = other.questionMarkErrors;
            eErrors = other.eErrors;
            oErrors = other.oErrors;
            lastErrorTime = other.lastErrorTime;
            lastCommandTime = other.lastCommandTime;
            std::memcpy(lastCommandBeforeError, other.lastCommandBeforeError, CMD_BUF_SIZE);
            std::memcpy(lastCommandSource, other.lastCommandSource, CMD_BUF_SIZE);
            lastCommandBeforeErrorTime = other.lastCommandBeforeErrorTime;
            firstErrorTime = other.firstErrorTime;
            averageErrorInterval = other.averageErrorInterval;
            errorBursts = other.errorBursts;
            inErrorBurst = other.inErrorBurst;
            recentErrorTimes = other.recentErrorTimes;
            recentErrorWriteIndex = other.recentErrorWriteIndex;
            recentErrorSamples = other.recentErrorSamples;
            highErrorRateActive = other.highErrorRateActive;
        }
        return *this;
    }

    static void storeToBuf(char (&buf)[CMD_BUF_SIZE], std::string_view src) {
        const size_t n = std::min(src.size(), CMD_BUF_SIZE - 1);
        std::memcpy(buf, src.data(), n);
        buf[n] = '\0';
    }

    void reset() {
        totalCommandsDispatched.store(0);
        commandsHandled.store(0);
        commandsUnhandled.store(0);
        handlerErrors.store(0);
        currentProcessingDepth.store(0);
        maxProcessingDepth.store(0);
        totalErrorResponses = 0;
        questionMarkErrors = 0;
        eErrors = 0;
        oErrors = 0;
        lastErrorTime = 0;
        lastCommandTime = 0;
        lastCommandBeforeError[0] = '\0';
        lastCommandSource[0] = '\0';
        lastCommandBeforeErrorTime = 0;
        firstErrorTime = 0;
        averageErrorInterval = 0;
        errorBursts = 0;
        inErrorBurst = false;
        recentErrorTimes.fill(0);
        recentErrorWriteIndex = 0;
        recentErrorSamples = 0;
        highErrorRateActive = false;
    }

    // Returns true only when a live burst first crosses 10 errors in 5 seconds.
    bool recordError(std::string_view errorType, std::string_view lastCmd, std::string_view lastSource, const uint64_t currentTime) {
        totalErrorResponses++;
        if (errorType == "?") questionMarkErrors++;
        else if (errorType == "E") eErrors++;
        else if (errorType == "O") oErrors++;

        if (firstErrorTime == 0) {
            firstErrorTime = currentTime;
        }

        // Calculate the true overall mean interval and detect distinct burst groups.
        // The former 3:1 EWMA was dominated by the most recent intra-burst gaps, so
        // diagnostics could claim "every 52 ms" while the actual lifetime rate was
        // only about two errors per minute.
        if (lastErrorTime > 0) {
            uint64_t interval = currentTime - lastErrorTime;
            averageErrorInterval = (currentTime - firstErrorTime) / (totalErrorResponses - 1);

            // Count a run of closely spaced errors as one burst, not one burst per
            // additional error in that run.
            if (interval < 5000000) {
                if (!inErrorBurst) {
                    errorBursts++;
                    inErrorBurst = true;
                }
            } else {
                inErrorBurst = false;
            }
        }

        lastErrorTime = currentTime;
        recentErrorTimes[recentErrorWriteIndex] = currentTime;
        recentErrorWriteIndex = (recentErrorWriteIndex + 1) % ERROR_RATE_SAMPLE_COUNT;
        if (recentErrorSamples < ERROR_RATE_SAMPLE_COUNT) recentErrorSamples++;

        bool highRateNow = false;
        if (recentErrorSamples == ERROR_RATE_SAMPLE_COUNT) {
            // Once full, writeIndex points at the oldest retained sample.
            const uint64_t oldest = recentErrorTimes[recentErrorWriteIndex];
            highRateNow = currentTime >= oldest && currentTime - oldest <= ERROR_RATE_WINDOW_US;
        }
        const bool highRateStarted = highRateNow && !highErrorRateActive;
        highErrorRateActive = highRateNow;
        // A radio rejection should follow its command promptly. Do not pin a random
        // command from tens of minutes ago onto an unsolicited/corrupt '?;' frame.
        static constexpr uint64_t COMMAND_ASSOCIATION_WINDOW_US = 2'000'000;
        if (lastCommandTime > 0 && currentTime >= lastCommandTime &&
            currentTime - lastCommandTime <= COMMAND_ASSOCIATION_WINDOW_US) {
            storeToBuf(lastCommandBeforeError, lastCmd);
            storeToBuf(lastCommandSource, lastSource);
            lastCommandBeforeErrorTime = lastCommandTime;
        } else {
            lastCommandBeforeError[0] = '\0';
            lastCommandSource[0] = '\0';
            lastCommandBeforeErrorTime = 0;
        }
        return highRateStarted;
    }

    size_t recentErrorCount(const uint64_t currentTime) const {
        size_t count = 0;
        for (size_t i = 0; i < recentErrorSamples; ++i) {
            const uint64_t timestamp = recentErrorTimes[i];
            if (timestamp > 0 && currentTime >= timestamp &&
                currentTime - timestamp <= ERROR_RATE_WINDOW_US) {
                count++;
            }
        }
        return count;
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
     * @brief Record that a command was sent to the radio (for error diagnostics)
     * Called from RadioManager::sendRadioCommand to track all commands sent to the radio,
     * not just those dispatched through handlers.
     * Only records the timestamp; command/source strings are captured lazily on error.
     * @param command The command string (stored in fixed buffer, no heap allocation)
     * @param source Human-readable source identifier (e.g. "Internal", "UsbCdc0")
     */
    void recordCommandSentToRadio(std::string_view command, std::string_view source = "Internal");

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
    mutable RtosMutex statsMutex_;  // Protects only the non-atomic error/timing/string fields of stats_ (counters are atomic)

    static constexpr const char* TAG = "CommandDispatcher";
};

} // namespace radio
