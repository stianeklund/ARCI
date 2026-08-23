#pragma once

#include <cstdint>
#include <string_view>
#include "RadioState.h"
#include "RadioCommand.h"

namespace radio
{

    // Forward declarations
    class RadioState;

    /**
     * @brief Centralized policy class for CAT command routing decisions
     *
     * This class encapsulates all the logic for determining when and where
     * CAT responses should be forwarded, consolidating the complex routing
     * logic that was previously scattered across RadioManager and handlers.
     */
    class ForwardingPolicy
    {
    public:
        /**
         * @brief Determines if a response should be forwarded to USB interface
         * @param response The response message to evaluate
         * @param state Reference to current radio state
         * @param currentTime Current timestamp in microseconds
         * @return true if should forward to USB
         */
        static bool shouldForwardToUSB(std::string_view response, const RadioState &state, uint64_t currentTime);

        // Per-CDC decisions for USB interfaces
        static bool shouldForwardToUsbCdc0(std::string_view response, const RadioState &state, uint64_t currentTime);
        static bool shouldForwardToUsbCdc1(std::string_view response, const RadioState &state, uint64_t currentTime);

        // Per-port decisions for TCP interfaces
        static bool shouldForwardToTcp0(std::string_view response, const RadioState &state, uint64_t currentTime);
        static bool shouldForwardToTcp1(std::string_view response, const RadioState &state, uint64_t currentTime);

        /**
         * @brief Determines if a response should be forwarded to display interface
         * @param response The response message to evaluate
         * @param state Reference to current radio state
         * @param currentTime Current timestamp in microseconds
         * @return true if should forward to display
         */
        static bool shouldForwardToDisplay(std::string_view response, const RadioState &state, uint64_t currentTime);

        /**
         * @brief Undo a dedup commit for one sink after delivery of `response` failed
         *
         * shouldForward*() commits the dedup slot (see handleDeduplication) BEFORE the
         * caller actually writes the frame to the sink. If that write fails (e.g. the
         * display serial queue is full), the slot has already recorded "delivered" for
         * a value that was never actually sent, so an identical follow-up value would be
         * wrongly suppressed forever. This resets only the slot matching `response`'s
         * prefix to a sentinel that no real value can match, so the next attempt (retry
         * or next natural update) is forwarded instead of deduped away.
         * Unknown/untracked prefixes are a no-op.
         *
         * @param response The frame that failed to be delivered
         * @param sinkState The forwarding state for the sink the delivery failed on
         */
        static void invalidateDedup(std::string_view response, RadioState::InterfaceForwardState &sinkState);

    private:
        // Helper methods for specific forwarding logic
        static bool shouldForwardInAIMode(std::string_view response, uint8_t aiMode, const RadioState &state,
                                          RadioState::InterfaceForwardState &sinkState, uint64_t currentTime,
                                          CommandSource sink = CommandSource::UsbCdc0);

        static bool shouldForwardInNonAIMode(std::string_view response, const RadioState &state,
                                             RadioState::InterfaceForwardState &sinkState, uint64_t currentTime,
                                             uint8_t aiMode = 0);

        static bool handleDeduplication(std::string_view response, RadioState::InterfaceForwardState &sinkState,
                                        uint64_t currentTime, bool commit = false);

        static bool handleRateLimiting(std::string_view response, RadioState::InterfaceForwardState &sinkState,
                                       uint64_t currentTime, CommandSource sink = CommandSource::UsbCdc0);

        static std::string_view getCommandPrefix(std::string_view response);

        // Constants
        static constexpr uint64_t FAIRNESS_WINDOW_US = 10000; // 10ms fairness window
        static constexpr uint64_t SM_MIN_INTERVAL_US = 100000; // 100ms = 10 per second
        static constexpr uint64_t SM_MIN_INTERVAL_DISPLAY_US = 20000; // 20ms for display = 50 per second
    };

} // namespace radio
