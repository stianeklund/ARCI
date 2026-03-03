#include "ForwardingPolicy.h"
#include <cctype>
#include <charconv>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include "DisplayLatencyProfiler.h"
#include "RadioCommand.h"
#include "RadioState.h"
#include "esp_log.h"

namespace radio
{

    static const char *TAG = "ForwardingPolicy";

    namespace
    {

        constexpr uint64_t FNV_OFFSET_BASIS = 1469598103934665603ULL;
        constexpr uint64_t FNV_PRIME = 1099511628211ULL;

        /**
         * @brief Parse numeric payload after 2-char command prefix
         * @tparam T Numeric type (int, uint64_t, etc.)
         * @param response Full CAT response (e.g., "FA00014074000;")
         * @param prefixLen Characters to skip (default 2 for standard commands)
         * @return Parsed value or nullopt on error
         */
        template<typename T>
        std::optional<T> parseNumericPayload(std::string_view response, size_t prefixLen = 2)
        {
            if (response.length() <= prefixLen + 1)
            {
                return std::nullopt;
            }

            const char *begin = response.data() + prefixLen;
            const char *ptr = begin;
            const char *end = response.data() + response.length();

            while (ptr < end && *ptr != ';')
            {
                if (!std::isdigit(static_cast<unsigned char>(*ptr)))
                {
                    return std::nullopt;
                }
                ++ptr;
            }

            if (ptr == begin)
            {
                return std::nullopt; // no digits found
            }

            T value = 0;
            const auto result = std::from_chars(begin, ptr, value);
            if (result.ec != std::errc())
            {
                return std::nullopt;
            }
            return value;
        }

        // Convenience aliases for common types
        inline std::optional<uint64_t> parseFrequencyPayload(std::string_view response)
        {
            return parseNumericPayload<uint64_t>(response);
        }

        inline std::optional<int> parseSmValue(std::string_view response)
        {
            return parseNumericPayload<int>(response);
        }

        // Parse RM response: RMP1P2P2P2P2; where P1 is meter type, P2 is 4-digit value
        // Returns {meter_type (1-3), value (0-30)} or nullopt on error
        std::optional<std::pair<int, int>> parseRmValue(std::string_view response)
        {
            // Minimum: RM + P1 + P2P2P2P2 = 7 chars
            if (response.length() < 7)
            {
                return std::nullopt;
            }

            char meterType = response[2];
            if (meterType < '1' || meterType > '3')
            {
                return std::nullopt; // Only interested in meter types 1, 2, 3
            }

            // Parse 4-digit meter value starting at position 3
            const char *begin = response.data() + 3;
            const char *end = begin + 4;

            int value = 0;
            const auto result = std::from_chars(begin, end, value);
            if (result.ec != std::errc())
            {
                return std::nullopt;
            }

            return std::make_pair(meterType - '0', value);
        }

        uint64_t fnv1a64(std::string_view data)
        {
            uint64_t hash = FNV_OFFSET_BASIS;
            for (unsigned char ch : data)
            {
                hash ^= ch;
                hash *= FNV_PRIME;
            }
            return hash;
        }

    } // namespace

    bool ForwardingPolicy::shouldForwardToUSB(std::string_view response, const RadioState &state, uint64_t currentTime)
    {
        // Backward-compat: default to CDC0 decision
        return shouldForwardToUsbCdc0(response, state, currentTime);
    }

    bool ForwardingPolicy::shouldForwardToUsbCdc0(std::string_view response, const RadioState &state,
                                                  uint64_t currentTime)
    {
        const uint8_t ai = state.usbCdc0AiMode.load();
        auto &sinkState = state.accessForwardState(CommandSource::UsbCdc0);

        // DEBUG: Add special logging for IF responses
        if (response.length() >= 2 && response.substr(0, 2) == "IF")
        {
            ESP_LOGV(TAG, "🔍 IF DECISION CDC0: ai=%d, response='%.*s'", ai, (int)response.length(), response.data());
        }

        ESP_LOGV(TAG, "shouldForwardToUsbCdc0: response='%.*s', ai=%d", (int)response.length(), response.data(), ai);
        if (response.length() < 2)
            return false;

        bool result;
        if (ai == 2 || ai == 4)
        {
            result = shouldForwardInAIMode(response, ai, state, sinkState, currentTime);
        }
        else
        {
            result = shouldForwardInNonAIMode(response, state, currentTime, ai);
        }

        // DEBUG: Add result logging for IF responses
        if (response.length() >= 2 && response.substr(0, 2) == "IF")
        {
            ESP_LOGV(TAG, "🔍 IF RESULT CDC0: %s (ai=%d)", result ? "FORWARD" : "BLOCK", ai);
        }

        return result;
    }

    bool ForwardingPolicy::shouldForwardToUsbCdc1(std::string_view response, const RadioState &state,
                                                  uint64_t currentTime)
    {
        const uint8_t ai = state.usbCdc1AiMode.load();
        auto &sinkState = state.accessForwardState(CommandSource::UsbCdc1);

        // DEBUG: Add special logging for IF responses
        if (response.length() >= 2 && response.substr(0, 2) == "IF")
        {
            ESP_LOGV(TAG, "🔍 IF DECISION CDC1: ai=%d, response='%.*s'", ai, (int)response.length(), response.data());
        }

        ESP_LOGV(TAG, "shouldForwardToUsbCdc1: response='%.*s', ai=%d", (int)response.length(), response.data(), ai);
        if (response.length() < 2)
            return false;

        bool result;
        if (ai == 2 || ai == 4)
        {
            result = shouldForwardInAIMode(response, ai, state, sinkState, currentTime);
        }
        else
        {
            result = shouldForwardInNonAIMode(response, state, currentTime, ai);
        }

        // DEBUG: Add result logging for IF responses
        if (response.length() >= 2 && response.substr(0, 2) == "IF")
        {
            ESP_LOGV(TAG, "🔍 IF RESULT CDC1: %s (ai=%d)", result ? "FORWARD" : "BLOCK", ai);
        }

        return result;
    }

    bool ForwardingPolicy::shouldForwardToTcp0(std::string_view response, const RadioState &state,
                                               uint64_t currentTime)
    {
        const uint8_t ai = state.tcp0AiMode.load();
        auto &sinkState = state.accessForwardState(CommandSource::Tcp0);

        ESP_LOGV(TAG, "shouldForwardToTcp0: response='%.*s', ai=%d", (int)response.length(), response.data(), ai);
        if (response.length() < 2)
            return false;

        if (ai == 2 || ai == 4)
        {
            return shouldForwardInAIMode(response, ai, state, sinkState, currentTime, CommandSource::Tcp0);
        }
        else
        {
            return shouldForwardInNonAIMode(response, state, currentTime, ai);
        }
    }

    bool ForwardingPolicy::shouldForwardToTcp1(std::string_view response, const RadioState &state,
                                               uint64_t currentTime)
    {
        const uint8_t ai = state.tcp1AiMode.load();
        auto &sinkState = state.accessForwardState(CommandSource::Tcp1);

        ESP_LOGV(TAG, "shouldForwardToTcp1: response='%.*s', ai=%d", (int)response.length(), response.data(), ai);
        if (response.length() < 2)
            return false;

        if (ai == 2 || ai == 4)
        {
            return shouldForwardInAIMode(response, ai, state, sinkState, currentTime, CommandSource::Tcp1);
        }
        else
        {
            return shouldForwardInNonAIMode(response, state, currentTime, ai);
        }
    }

    bool ForwardingPolicy::shouldForwardToDisplay(std::string_view response, const RadioState &state,
                                                  uint64_t currentTime)
    {
        const uint8_t displayAiMode = state.displayAiMode.load();
        auto &sinkState = state.accessForwardState(CommandSource::Display);

        ESP_LOGV(TAG, "shouldForwardToDisplay: response='%.*s', displayAiMode=%d", (int)response.length(),
                 response.data(), displayAiMode);

        // Get command prefix (first 2 characters)
        if (response.length() < 2)
        {
            return false;
        }

        // Tuning suppression: Block IF/FA/FB updates during active tuning to prevent display flicker
        const std::string_view prefix = getCommandPrefix(response);
        if (prefix == "IF" || prefix == "FA" || prefix == "FB")
        {
            constexpr uint64_t TUNING_DEBOUNCE_US = 300000;  // 300ms
            const bool isTuning = state.isTuning.load();
            const uint64_t lastEncoderActivity = state.lastEncoderActivityTime.load();
            const bool withinDebounce = lastEncoderActivity > 0 &&
                                        (currentTime - lastEncoderActivity) < TUNING_DEBOUNCE_US;

            if (isTuning || withinDebounce)
            {
                ESP_LOGV(TAG, "Tuning suppression: blocking %.*s (tuning=%d, debounce=%d)",
                         static_cast<int>(prefix.length()), prefix.data(), isTuning, withinDebounce);
                if (prefix == "FA")
                {
                    DisplayLatencyProfiler::instance().markDisplaySuppressed(
                        DisplayLatencyProfiler::SuppressReason::TuningDebounce);
                }
                return false;
            }
        }

        // For display AI2/AI4 modes, use AI mode logic with display-specific rate limits
        if (displayAiMode == 2 || displayAiMode == 4)
        {
            return shouldForwardInAIMode(response, displayAiMode, state, sinkState, currentTime, CommandSource::Display);
        }

        // For non-AI modes, use query tracking logic
        return shouldForwardInNonAIMode(response, state, currentTime, displayAiMode);
    }

    bool ForwardingPolicy::shouldForwardInAIMode(std::string_view response, uint8_t aiMode, const RadioState &state,
                                                 RadioState::InterfaceForwardState &sinkState, uint64_t currentTime,
                                                 CommandSource sink)
    {
        const std::string_view prefix = getCommandPrefix(response);

        // Even in AI mode, some commands should not be forwarded unsolicited
        // EX commands (menu items) should only be forwarded if specifically queried
        if (prefix == "EX")
        {
            if (state.queryTracker.wasRecentlyQueried(std::string(prefix), currentTime))
            {
                ESP_LOGV(TAG, "📡 AI mode %d: forwarding EX - was queried", aiMode);
                return true;
            }
            ESP_LOGV(TAG, "📡 AI mode %d: not forwarding EX - not queried", aiMode);
            return false;
        }

        if (!handleDeduplication(response, sinkState, currentTime, false))
        {
            ESP_LOGV(TAG, "📡 AI mode %d: skipping %.*s (dedup)", aiMode, static_cast<int>(prefix.length()),
                     prefix.data());
            if (sink == CommandSource::Display && prefix == "FA")
            {
                DisplayLatencyProfiler::instance().markDisplaySuppressed(
                    DisplayLatencyProfiler::SuppressReason::Deduplication);
            }
            return false;
        }

        if (!handleRateLimiting(response, sinkState, currentTime, sink))
        {
            ESP_LOGV(TAG, "📡 AI mode %d: skipping %.*s (rate limited)", aiMode, static_cast<int>(prefix.length()),
                     prefix.data());
            if (sink == CommandSource::Display && prefix == "FA")
            {
                DisplayLatencyProfiler::instance().markDisplaySuppressed(
                    DisplayLatencyProfiler::SuppressReason::RateLimited);
            }
            return false;
        }

        // Commit dedup state now that the frame will be forwarded
        (void)handleDeduplication(response, sinkState, currentTime, true);

        ESP_LOGV(TAG, "📡 AI mode %d: forwarding %.*s", aiMode, static_cast<int>(prefix.length()), prefix.data());
        return true;
    }

    bool ForwardingPolicy::shouldForwardInNonAIMode(std::string_view response, const RadioState &state,
                                                    uint64_t currentTime, uint8_t aiMode)
    {
        const std::string_view prefix = getCommandPrefix(response);

        // Check if interface recently queried this command type (5 second TTL)
        const bool recentlyQueried = state.queryTracker.wasRecentlyQueried(std::string(prefix), currentTime);

        if (aiMode == 0)
        {
            // AI0 = no unsolicited updates, but query responses MUST still be forwarded.
            // A real TS-590SG in AI0 mode still responds to explicit queries.
            if (recentlyQueried)
            {
                ESP_LOGI(TAG, "AI0 forwarding query response: %.*s", (int)prefix.length(), prefix.data());
                return true;
            }
            ESP_LOGD(TAG, "AI0 suppressing unsolicited: %.*s", (int)prefix.length(), prefix.data());
            return false;
        }

        // DEBUG: Add special logging for IF responses
        if (prefix == "IF")
        {
            ESP_LOGE(TAG, "🔍 IF NON-AI CHECK: prefix='%.*s', recentlyQueried=%s", (int)prefix.length(), prefix.data(),
                     recentlyQueried ? "YES" : "NO");
        }

        // DEBUG: Add special logging for XO responses
        if (prefix == "XO")
        {
            ESP_LOGI(TAG, "🔍 XO NON-AI CHECK: prefix='%.*s', recentlyQueried=%s, aiMode=%d", (int)prefix.length(),
                     prefix.data(), recentlyQueried ? "YES" : "NO", aiMode);
        }

        if (recentlyQueried)
        {
            ESP_LOGV(TAG, "shouldForwardInNonAIMode: Forwarding %.*s - recently queried", (int)prefix.length(),
                     prefix.data());
            return true;
        }

        // Forward some rare but requested readouts by default to mimic TS-590SG behavior
        // Note: EX commands removed - they should only be forwarded if specifically queried
        // TX/RX are status notifications that should always be forwarded (not query responses)
        if (prefix == "VV" || prefix == "SS" || prefix == "SU" || prefix == "TX" || prefix == "RX")
        {
            return true;
        }

        // For other commands without specific query tracking,
        // don't forward in non-AI modes to avoid unsolicited responses
        ESP_LOGV(TAG, "shouldForwardInNonAIMode: Not forwarding %.*s - not queried", (int)prefix.length(),
                 prefix.data());
        return false;
    }

    bool ForwardingPolicy::handleDeduplication(std::string_view response, RadioState::InterfaceForwardState &sinkState,
                                               uint64_t /*currentTime*/, bool commit)
    {
        const std::string_view prefix = getCommandPrefix(response);

        if (prefix == "FA" || prefix == "FB")
        {
            const auto frequency = parseFrequencyPayload(response);
            if (!frequency.has_value())
            {
                return true; // Unable to parse; forward to be safe
            }

            auto &slot = (prefix == "FA") ? sinkState.lastFAMessage : sinkState.lastFBMessage;
            const uint64_t previous = slot.load(std::memory_order_relaxed);
            if (previous == *frequency)
            {
                return false;
            }
            if (commit)
            {
                slot.store(*frequency, std::memory_order_relaxed);
            }
            return true;
        }

        if (prefix == "IF")
        {
            const uint64_t hash = fnv1a64(response);
            auto &slot = sinkState.lastIFMessage;
            const uint64_t previous = slot.load(std::memory_order_relaxed);
            if (previous == hash)
            {
                return false;
            }
            if (commit)
            {
                slot.store(hash, std::memory_order_relaxed);
            }
            return true;
        }

        if (prefix == "SM")
        {
            const auto smValue = parseSmValue(response);
            if (!smValue.has_value())
            {
                return true;
            }
            auto &slot = sinkState.lastSMValue;
            const int previous = slot.load(std::memory_order_relaxed);
            if (previous == *smValue)
            {
                return false;
            }
            if (commit)
            {
                slot.store(*smValue, std::memory_order_relaxed);
            }
            return true;
        }

        if (prefix == "RM")
        {
            const auto rmValue = parseRmValue(response);
            if (!rmValue.has_value())
            {
                return true; // Unable to parse; forward to be safe
            }

            const auto [meterType, value] = *rmValue;
            std::atomic<int> *slot = nullptr;

            switch (meterType)
            {
            case 1:
                slot = &sinkState.lastRM1Value;
                break;
            case 2:
                slot = &sinkState.lastRM2Value;
                break;
            case 3:
                slot = &sinkState.lastRM3Value;
                break;
            default:
                return true; // Unknown meter type, forward anyway
            }

            const int previous = slot->load(std::memory_order_relaxed);
            if (previous == value)
            {
                ESP_LOGV(TAG, "📉 RM%d dedup: value=%d unchanged", meterType, value);
                return false;
            }
            if (commit)
            {
                slot->store(value, std::memory_order_relaxed);
            }
            return true;
        }

        // Default: no dedup performed
        return true;
    }

    bool ForwardingPolicy::handleRateLimiting(std::string_view response, RadioState::InterfaceForwardState &sinkState,
                                              uint64_t currentTime, CommandSource sink)
    {
        const std::string_view prefix = getCommandPrefix(response);

        if (prefix == "SM")
        {
            // Use display-specific rate limit for display sink, standard limit for others
            const uint64_t smRateLimit = (sink == CommandSource::Display) ? SM_MIN_INTERVAL_DISPLAY_US : SM_MIN_INTERVAL_US;

            const uint64_t lastSmTime = sinkState.lastSMTime.load(std::memory_order_relaxed);
            if (lastSmTime != 0 && (currentTime - lastSmTime) < smRateLimit)
            {
                ESP_LOGV(TAG, "📉 SM rate limited: Δ%lluµs < %lluµs (sink=%d)",
                         static_cast<unsigned long long>(currentTime - lastSmTime),
                         static_cast<unsigned long long>(smRateLimit),
                         static_cast<int>(sink));
                return false;
            }

            const uint64_t lastForward = sinkState.lastForwardTime.load(std::memory_order_relaxed);
            if (lastForward != 0 && (currentTime - lastForward) < FAIRNESS_WINDOW_US &&
                sinkState.lastForwardedWasSM.load(std::memory_order_relaxed))
            {
                ESP_LOGV(TAG, "📉 SM fairness drop: Δ%lluµs < %lluµs",
                         static_cast<unsigned long long>(currentTime - lastForward),
                         static_cast<unsigned long long>(FAIRNESS_WINDOW_US));
                return false;
            }

            sinkState.lastSMTime.store(currentTime, std::memory_order_relaxed);
            sinkState.lastForwardedWasSM.store(true, std::memory_order_relaxed);
        }
        else
        {
            sinkState.lastForwardedWasSM.store(false, std::memory_order_relaxed);
        }

        sinkState.lastForwardTime.store(currentTime, std::memory_order_relaxed);
        return true;
    }

    std::string_view ForwardingPolicy::getCommandPrefix(std::string_view response)
    {
        if (response.length() >= 2)
        {
            return response.substr(0, 2);
        }
        return "";
    }

} // namespace radio
