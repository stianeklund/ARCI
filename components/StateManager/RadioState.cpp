#include "RadioState.h"
#include <cinttypes>
#include "RadioCommand.h" // For CommandSource enum
#include "esp_log.h"

namespace radio
{

    static const char *TAG = "RadioState";

    bool RadioState::tryAcquireTx(CommandSource source, uint64_t currentTime)
    {
        RtosLockGuard<RtosMutex> lock(txMutex);

        // Check if TX is already owned by someone else
        int currentOwner = txOwner.load();
        if (currentOwner != -1 && currentOwner != static_cast<int>(source))
        {
            // Check if current ownership has timed out
            if (!isTxTimedOut(currentTime))
            {
                ESP_LOGD(TAG, "TX acquisition denied: owned by source %d, requested by %d", currentOwner,
                         static_cast<int>(source));
                return false;
            }
            // Current ownership timed out, force release
            ESP_LOGI(TAG, "TX ownership timed out for source %d, allowing acquisition by %d", currentOwner,
                     static_cast<int>(source));
        }

        // Check if ownership is actually changing
        const bool ownershipChanged = (currentOwner != static_cast<int>(source));
        const bool txWasInactive = !isTx.load();

        // Acquire TX ownership
        txOwner.store(static_cast<int>(source));
        txActivationTime.store(currentTime);
        isTx.store(true);

        // Only log INFO when ownership changes or TX goes from inactive to active
        if (ownershipChanged) {
            ESP_LOGI(TAG, "✅ TX ownership changed: source %d → %d", currentOwner, static_cast<int>(source));
        } else if (txWasInactive) {
            ESP_LOGI(TAG, "✅ TX activated by source %d", static_cast<int>(source));
        } else {
            ESP_LOGD(TAG, "TX re-acquired by source %d (no change)", static_cast<int>(source));
        }
        return true;
    }

    bool RadioState::releaseTx(CommandSource source, uint64_t currentTime)
    {
        RtosLockGuard<RtosMutex> lock(txMutex);

        int currentOwner = txOwner.load();

        // Only the owner (or forced release) can release TX
        if (currentOwner != static_cast<int>(source))
        {
            ESP_LOGW(TAG, "TX release denied: owned by source %d, requested by %d", currentOwner,
                     static_cast<int>(source));
            return false;
        }

        // Release TX ownership
        txOwner.store(-1);
        txActivationTime.store(0);
        isTx.store(false);

        ESP_LOGI(TAG, "✅ TX released by source %d at time %" PRIu64, static_cast<int>(source), currentTime);
        return true;
    }

    bool RadioState::forceReleaseTx(uint64_t currentTime)
    {
        RtosLockGuard<RtosMutex> lock(txMutex);

        if (!isTx.load())
        {
            return false; // Nothing to release
        }

        int currentOwner = txOwner.load();

        // Force release TX ownership
        txOwner.store(-1);
        txActivationTime.store(0);
        isTx.store(false);

        ESP_LOGW(TAG, "⚠️ TX force released (previous owner: %d) at time %" PRIu64, currentOwner, currentTime);
        return true;
    }

    RadioState::InterfaceForwardState &RadioState::accessForwardState(CommandSource sink) const
    {
        // Members are declared mutable, so direct access from const method is valid
        switch (sink)
        {
        case CommandSource::UsbCdc0:
            return usb0ForwardState;
        case CommandSource::UsbCdc1:
            return usb1ForwardState;
        case CommandSource::Tcp0:
            return tcp0ForwardState;
        case CommandSource::Tcp1:
            return tcp1ForwardState;
        case CommandSource::Display:
            return displayForwardState;
        default:
            // Panel/Remote reuse USB0 state to avoid null references; they do not rely on AI dedup
            return usb0ForwardState;
        }
    }

    bool RadioState::tryAcquireControlLease(CommandSource source, int priority, uint64_t currentTime,
                                            uint64_t leaseDurationUs)
    {
        RtosLockGuard<RtosMutex> lock(controlLeaseMutex);

        const int currentOwner = controlLeaseOwner.load(std::memory_order_relaxed);
        const uint64_t currentExpiry = controlLeaseExpiry.load(std::memory_order_relaxed);

        if (currentOwner == static_cast<int>(source))
        {
            controlLeaseExpiry.store(currentTime + leaseDurationUs, std::memory_order_relaxed);
            return true;
        }

        const bool expired = currentOwner == -1 || currentExpiry == 0 || currentExpiry <= currentTime;
        if (expired)
        {
            controlLeaseOwner.store(static_cast<int>(source), std::memory_order_relaxed);
            controlLeasePriority.store(priority, std::memory_order_relaxed);
            controlLeaseExpiry.store(currentTime + leaseDurationUs, std::memory_order_relaxed);
            return true;
        }

        const int existingPriority = controlLeasePriority.load(std::memory_order_relaxed);
        if (priority > existingPriority)
        {
            controlLeaseOwner.store(static_cast<int>(source), std::memory_order_relaxed);
            controlLeasePriority.store(priority, std::memory_order_relaxed);
            controlLeaseExpiry.store(currentTime + leaseDurationUs, std::memory_order_relaxed);
            return true;
        }

        // Allow equal-priority takeover when the current lease is nearly expired
        constexpr uint64_t GRACE_THRESHOLD_US = 200000ULL; // 200 ms before expiry
        if (priority == existingPriority && currentExpiry > currentTime &&
            (currentExpiry - currentTime) <= GRACE_THRESHOLD_US)
        {
            controlLeaseOwner.store(static_cast<int>(source), std::memory_order_relaxed);
            controlLeasePriority.store(priority, std::memory_order_relaxed);
            controlLeaseExpiry.store(currentTime + leaseDurationUs, std::memory_order_relaxed);
            return true;
        }

        return false;
    }

    bool RadioState::refreshControlLease(CommandSource source, uint64_t currentTime, uint64_t leaseDurationUs)
    {
        RtosLockGuard<RtosMutex> lock(controlLeaseMutex);
        if (controlLeaseOwner.load(std::memory_order_relaxed) != static_cast<int>(source))
        {
            return false;
        }

        controlLeaseExpiry.store(currentTime + leaseDurationUs, std::memory_order_relaxed);
        return true;
    }

    void RadioState::releaseControlLease(CommandSource source)
    {
        RtosLockGuard<RtosMutex> lock(controlLeaseMutex);
        if (controlLeaseOwner.load(std::memory_order_relaxed) == static_cast<int>(source))
        {
            controlLeaseOwner.store(-1, std::memory_order_relaxed);
            controlLeasePriority.store(-1, std::memory_order_relaxed);
            controlLeaseExpiry.store(0, std::memory_order_relaxed);
        }
    }

    void RadioState::forceReleaseControlLease()
    {
        RtosLockGuard<RtosMutex> lock(controlLeaseMutex);
        controlLeaseOwner.store(-1, std::memory_order_relaxed);
        controlLeasePriority.store(-1, std::memory_order_relaxed);
        controlLeaseExpiry.store(0, std::memory_order_relaxed);
    }

    bool RadioState::isControlLeaseActive(CommandSource source, uint64_t currentTime) const
    {
        const int owner = controlLeaseOwner.load(std::memory_order_relaxed);
        const uint64_t expiry = controlLeaseExpiry.load(std::memory_order_relaxed);
        if (owner == -1 || expiry == 0 || expiry <= currentTime)
        {
            return false;
        }
        return owner == static_cast<int>(source);
    }

} // namespace radio
