#pragma once

#include <atomic>
#include <cstdint>
#include "esp_log.h"
#include "esp_timer.h"

namespace radio
{

/**
 * @brief Lightweight profiler for display update latency diagnostics.
 *
 * Accumulates statistics with minimal overhead (atomic operations only).
 * Call logAndReset() periodically (e.g., every 10s) to emit summary.
 */
class DisplayLatencyProfiler
{
  public:
    // Suppression reason codes
    enum class SuppressReason : uint8_t
    {
        None = 0,
        IsTuning,
        GracePeriod,
        TuningDebounce,
        Deduplication,
        RateLimited,
        PendingValidation
    };

    static DisplayLatencyProfiler &instance()
    {
        static DisplayLatencyProfiler inst;
        return inst;
    }

    // Call at start of FA Answer processing
    void markFaAnswerStart() { lastFaStartUs_.store(esp_timer_get_time(), std::memory_order_relaxed); }

    // Call after state update, before routing
    void markFaStateUpdated()
    {
        const uint64_t now = esp_timer_get_time();
        const uint64_t start = lastFaStartUs_.load(std::memory_order_relaxed);
        if (start > 0)
        {
            const uint64_t elapsed = now - start;
            updateStats(faProcessUs_, elapsed);
        }
    }

    // Call when FA is forwarded to display
    void markDisplayForwarded()
    {
        const uint64_t now = esp_timer_get_time();
        const uint64_t start = lastFaStartUs_.load(std::memory_order_relaxed);
        if (start > 0)
        {
            const uint64_t elapsed = now - start;
            updateStats(faToDisplayUs_, elapsed);
        }
        displayForwardCount_.fetch_add(1, std::memory_order_relaxed);
    }

    // Call when FA is suppressed from display
    void markDisplaySuppressed(SuppressReason reason)
    {
        displaySuppressCount_.fetch_add(1, std::memory_order_relaxed);
        switch (reason)
        {
        case SuppressReason::IsTuning:
            suppressIsTuning_.fetch_add(1, std::memory_order_relaxed);
            break;
        case SuppressReason::GracePeriod:
            suppressGracePeriod_.fetch_add(1, std::memory_order_relaxed);
            break;
        case SuppressReason::TuningDebounce:
            suppressTuningDebounce_.fetch_add(1, std::memory_order_relaxed);
            break;
        case SuppressReason::Deduplication:
            suppressDedup_.fetch_add(1, std::memory_order_relaxed);
            break;
        case SuppressReason::RateLimited:
            suppressRateLimit_.fetch_add(1, std::memory_order_relaxed);
            break;
        case SuppressReason::PendingValidation:
            suppressPending_.fetch_add(1, std::memory_order_relaxed);
            break;
        default:
            break;
        }
    }

    // Call when USB CDC0 receives FA
    void markUsbCdc0Forwarded() { usbCdc0ForwardCount_.fetch_add(1, std::memory_order_relaxed); }

    // Track interval between display forwards
    void markDisplayForwardInterval()
    {
        const uint64_t now = esp_timer_get_time();
        const uint64_t last = lastDisplayForwardUs_.exchange(now, std::memory_order_relaxed);
        if (last > 0)
        {
            const uint64_t interval = now - last;
            updateStats(displayIntervalUs_, interval);
        }
    }

    // Track direct encoder-to-display sends (bypasses radio echo path)
    void markEncoderDirectSend()
    {
        encoderDirectCount_.fetch_add(1, std::memory_order_relaxed);
        const uint64_t now = esp_timer_get_time();
        const uint64_t last = lastEncoderSendUs_.exchange(now, std::memory_order_relaxed);
        if (last > 0)
        {
            const uint64_t interval = now - last;
            updateStats(encoderIntervalUs_, interval);
        }
    }

    // Enable/disable logging output (tracking still occurs for minimal overhead)
    static constexpr bool LOGGING_ENABLED = false;

    // Log summary and reset counters. Call from low-priority task every ~10s.
    void logAndReset()
    {
        static constexpr const char *TAG = "DisplayProfiler";

        if constexpr (!LOGGING_ENABLED)
        {
            // Still reset counters to prevent overflow, but don't log
            displayForwardCount_.store(0, std::memory_order_relaxed);
            displaySuppressCount_.store(0, std::memory_order_relaxed);
            usbCdc0ForwardCount_.store(0, std::memory_order_relaxed);
            encoderDirectCount_.store(0, std::memory_order_relaxed);
            suppressIsTuning_.store(0, std::memory_order_relaxed);
            suppressGracePeriod_.store(0, std::memory_order_relaxed);
            suppressTuningDebounce_.store(0, std::memory_order_relaxed);
            suppressDedup_.store(0, std::memory_order_relaxed);
            suppressRateLimit_.store(0, std::memory_order_relaxed);
            suppressPending_.store(0, std::memory_order_relaxed);
            resetStats(faProcessUs_);
            resetStats(faToDisplayUs_);
            resetStats(displayIntervalUs_);
            resetStats(encoderIntervalUs_);
            return;
        }

        // Snapshot and reset atomics
        const uint32_t fwdCount = displayForwardCount_.exchange(0, std::memory_order_relaxed);
        const uint32_t supCount = displaySuppressCount_.exchange(0, std::memory_order_relaxed);
        const uint32_t usbCount = usbCdc0ForwardCount_.exchange(0, std::memory_order_relaxed);
        const uint32_t encCount = encoderDirectCount_.exchange(0, std::memory_order_relaxed);

        // Suppression breakdown
        const uint32_t supTuning = suppressIsTuning_.exchange(0, std::memory_order_relaxed);
        const uint32_t supGrace = suppressGracePeriod_.exchange(0, std::memory_order_relaxed);
        const uint32_t supDebounce = suppressTuningDebounce_.exchange(0, std::memory_order_relaxed);
        const uint32_t supDedup = suppressDedup_.exchange(0, std::memory_order_relaxed);
        const uint32_t supRate = suppressRateLimit_.exchange(0, std::memory_order_relaxed);
        const uint32_t supPending = suppressPending_.exchange(0, std::memory_order_relaxed);

        // Latency stats
        const auto procStats = resetStats(faProcessUs_);
        const auto fwdStats = resetStats(faToDisplayUs_);
        const auto intervalStats = resetStats(displayIntervalUs_);
        const auto encIntervalStats = resetStats(encoderIntervalUs_);

        // Only log if there was any activity
        if (fwdCount == 0 && supCount == 0 && usbCount == 0 && encCount == 0)
        {
            return;
        }

        // Main summary line - now includes encoder direct sends
        ESP_LOGI(TAG, "FA Display: fwd=%lu sup=%lu enc=%lu | USB0: %lu",
                 static_cast<unsigned long>(fwdCount),
                 static_cast<unsigned long>(supCount),
                 static_cast<unsigned long>(encCount),
                 static_cast<unsigned long>(usbCount));

        // Suppression breakdown (only if suppressions occurred)
        if (supCount > 0)
        {
            ESP_LOGI(TAG, "  Suppress: tuning=%lu grace=%lu debounce=%lu dedup=%lu rate=%lu pending=%lu",
                     static_cast<unsigned long>(supTuning), static_cast<unsigned long>(supGrace),
                     static_cast<unsigned long>(supDebounce), static_cast<unsigned long>(supDedup),
                     static_cast<unsigned long>(supRate), static_cast<unsigned long>(supPending));
        }

        // Encoder interval stats (only if encoder sends occurred)
        if (encCount > 0 && encIntervalStats.count > 0)
        {
            const uint32_t avgEncInterval = static_cast<uint32_t>(encIntervalStats.sum / encIntervalStats.count);
            ESP_LOGI(TAG, "  Encoder->Display(ms): avg=%lu min=%lu max=%lu (count=%lu)",
                     static_cast<unsigned long>(avgEncInterval / 1000),
                     static_cast<unsigned long>(encIntervalStats.min / 1000),
                     static_cast<unsigned long>(encIntervalStats.max / 1000),
                     static_cast<unsigned long>(encCount));
        }

        // Radio echo latency stats (only if forwards occurred)
        if (fwdCount > 0)
        {
            const uint32_t avgProc = procStats.count > 0 ? static_cast<uint32_t>(procStats.sum / procStats.count) : 0;
            const uint32_t avgFwd = fwdStats.count > 0 ? static_cast<uint32_t>(fwdStats.sum / procStats.count) : 0;
            const uint32_t avgInterval =
                intervalStats.count > 0 ? static_cast<uint32_t>(intervalStats.sum / intervalStats.count) : 0;

            ESP_LOGI(TAG, "  RadioEcho(us): process=%lu fwd=%lu | Interval(ms): avg=%lu min=%lu max=%lu",
                     static_cast<unsigned long>(avgProc), static_cast<unsigned long>(avgFwd),
                     static_cast<unsigned long>(avgInterval / 1000),
                     static_cast<unsigned long>(intervalStats.min / 1000),
                     static_cast<unsigned long>(intervalStats.max / 1000));
        }
    }

  private:
    DisplayLatencyProfiler() = default;

    struct LatencyStats
    {
        std::atomic<uint64_t> sum{0};
        std::atomic<uint64_t> min{UINT64_MAX};
        std::atomic<uint64_t> max{0};
        std::atomic<uint32_t> count{0};
    };

    struct StatsSnapshot
    {
        uint64_t sum;
        uint64_t min;
        uint64_t max;
        uint32_t count;
    };

    void updateStats(LatencyStats &stats, uint64_t value)
    {
        stats.sum.fetch_add(value, std::memory_order_relaxed);
        stats.count.fetch_add(1, std::memory_order_relaxed);

        // Update min (relaxed CAS loop)
        uint64_t curMin = stats.min.load(std::memory_order_relaxed);
        while (value < curMin && !stats.min.compare_exchange_weak(curMin, value, std::memory_order_relaxed))
        {
        }

        // Update max (relaxed CAS loop)
        uint64_t curMax = stats.max.load(std::memory_order_relaxed);
        while (value > curMax && !stats.max.compare_exchange_weak(curMax, value, std::memory_order_relaxed))
        {
        }
    }

    StatsSnapshot resetStats(LatencyStats &stats)
    {
        StatsSnapshot snap;
        snap.sum = stats.sum.exchange(0, std::memory_order_relaxed);
        snap.min = stats.min.exchange(UINT64_MAX, std::memory_order_relaxed);
        snap.max = stats.max.exchange(0, std::memory_order_relaxed);
        snap.count = stats.count.exchange(0, std::memory_order_relaxed);
        return snap;
    }

    // Timestamps
    std::atomic<uint64_t> lastFaStartUs_{0};
    std::atomic<uint64_t> lastDisplayForwardUs_{0};
    std::atomic<uint64_t> lastEncoderSendUs_{0};

    // Counters
    std::atomic<uint32_t> displayForwardCount_{0};
    std::atomic<uint32_t> displaySuppressCount_{0};
    std::atomic<uint32_t> usbCdc0ForwardCount_{0};
    std::atomic<uint32_t> encoderDirectCount_{0};

    // Suppression breakdown
    std::atomic<uint32_t> suppressIsTuning_{0};
    std::atomic<uint32_t> suppressGracePeriod_{0};
    std::atomic<uint32_t> suppressTuningDebounce_{0};
    std::atomic<uint32_t> suppressDedup_{0};
    std::atomic<uint32_t> suppressRateLimit_{0};
    std::atomic<uint32_t> suppressPending_{0};

    // Latency accumulators
    LatencyStats faProcessUs_;
    LatencyStats faToDisplayUs_;
    LatencyStats displayIntervalUs_;
    LatencyStats encoderIntervalUs_;
};

} // namespace radio
