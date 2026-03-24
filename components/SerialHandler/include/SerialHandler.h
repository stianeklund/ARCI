#pragma once

#include "ISerialChannel.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
#include <string>
#include <string_view>
#include <functional>

class SerialHandler : public ISerialChannel {
public:
    explicit SerialHandler(uart_port_t uart_num);
    virtual ~SerialHandler();

public:
    static constexpr size_t MAX_MESSAGE_LENGTH = 64;
    esp_err_t setupUart(int baud_rate);
    esp_err_t setupUart(int baud_rate, int tx_pin, int rx_pin);
    esp_err_t setupUart(int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin);

    // ISerialChannel interface implementation
    bool hasMessage() const override;
    std::pair<esp_err_t, std::string> getMessage() override;
    std::pair<esp_err_t, std::string_view> getMessageView() override;
    esp_err_t sendMessage(std::string_view message) override;
    esp_err_t sendMessage(std::string_view message1, std::string_view message2) override;
    uint32_t getSendFailureCount() const override { return sendFailures_.load(); }
    void setOnFrameCallback(std::function<void()> cb) override { onFrameCallback_ = std::move(cb); }

    // UART-specific methods (not part of ISerialChannel interface)
    void processReceivedData(const uint8_t* data, size_t len);

private:
    uart_port_t m_uart_num;
    char m_accum[MAX_MESSAGE_LENGTH]{};
    size_t m_accum_len{0};

    static constexpr size_t QUEUE_CAPACITY = 64;  // Increased to handle high traffic in AI2/AI4 modes
    struct MsgSlot {
        char data[MAX_MESSAGE_LENGTH]{};
        size_t len{0};
        int64_t timestamp{0};  // Time when message was queued (microseconds)
    };
    MsgSlot m_queue[QUEUE_CAPACITY]{};
    size_t m_head{0};
    size_t m_tail{0};
    size_t m_count{0};
    mutable portMUX_TYPE m_queueSpinlock;  // Protects m_head, m_tail, m_count, m_queue access
    TaskHandle_t m_uartTaskHandle;
    QueueHandle_t m_uartQueue;
    static constexpr char END_MARKER = ';';
    static constexpr size_t RX_BUF_SIZE = 1024;
    static constexpr size_t TX_BUF_SIZE = 1024;
    static constexpr int64_t QUEUE_TIMEOUT_US = 1500000;  // 1.5 second timeout for queue entries

    static void uartEventTask(void* pvParameters);
    void clearExpiredMessages();  // Remove messages older than QUEUE_TIMEOUT_US

    void recordSendFailure(esp_err_t error, const char* reason, size_t attemptedLen, esp_log_level_t level);

    // Optional callback to signal higher-level code that a frame is ready
    std::function<void()> onFrameCallback_{};

    std::atomic<uint32_t> sendFailures_{0};

    // Debug tracking for last frame sent (used for correlating radio errors)
    std::string lastSentFrame_;
    std::atomic<uint64_t> lastSentTimestampUs_{0};

    // --- Queue health diagnostics (atomic, zero overhead in hot path) ---
    std::atomic<uint32_t> queueHighWatermark_{0};  // Max queue depth seen
    std::atomic<uint32_t> totalExpired_{0};         // Messages dropped due to age
    std::atomic<uint32_t> totalOverflowDrops_{0};   // Messages dropped due to full queue
    std::atomic<uint64_t> maxDequeueAgeUs_{0};      // Worst-case message age at dequeue
    std::atomic<uint64_t> totalDequeueAgeUs_{0};    // Sum of all dequeue ages (for average)
    std::atomic<uint32_t> totalDequeued_{0};         // Count of dequeued messages

public:
    /// Snapshot of queue health counters (call periodically, e.g., every 10s)
    struct QueueStats {
        uint32_t highWatermark;
        uint32_t expired;
        uint32_t overflowDrops;
        size_t   currentDepth;
        uint64_t maxAgeUs;       // Worst-case message age at dequeue (µs)
        uint64_t avgAgeUs;       // Average message age at dequeue (µs)
        uint32_t dequeued;       // Total messages dequeued in period
    };
    QueueStats resetQueueStats();

    /// Current queue depth (lock-free read of atomic)
    size_t getQueueDepth() const {
        portENTER_CRITICAL(&m_queueSpinlock);
        const size_t d = m_count;
        portEXIT_CRITICAL(&m_queueSpinlock);
        return d;
    }

    /// Message age (µs) of the head slot, or 0 if empty.
    /// Useful for checking how stale the oldest pending message is.
    int64_t peekHeadAgeUs() const {
        portENTER_CRITICAL(&m_queueSpinlock);
        if (m_count == 0) {
            portEXIT_CRITICAL(&m_queueSpinlock);
            return 0;
        }
        const int64_t age = esp_timer_get_time() - m_queue[m_head].timestamp;
        portEXIT_CRITICAL(&m_queueSpinlock);
        return age;
    }
};
