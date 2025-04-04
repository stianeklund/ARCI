#ifndef MOCKSERIALHANDLER_H
#define MOCKSERIALHANDLER_H
#include <vector>
#include <string>
#include <string_view>
#include <functional>
#include <atomic>
#include "ISerialChannel.h"
#include "driver/uart.h"
#include "esp_err.h"

class MockSerialHandler : public ISerialChannel {
public:
    explicit MockSerialHandler(uart_port_t uart_num = UART_NUM_0);
    ~MockSerialHandler() override;

    // Mock-specific UART setup methods (not part of interface)
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
    
    // Test helper methods
    std::vector<std::string> sentMessages;
    void queueReceivedMessage(std::string_view message);
    void clearSentMessages() { sentMessages.clear(); }
    void clearReceivedMessages() { m_receivedMessages.clear(); }

private:
    std::vector<std::string> m_receivedMessages;
    mutable size_t m_nextMessageIndex{0};
    mutable std::string m_lastMessageView;
    std::atomic<uint32_t> sendFailures_{0};
    std::function<void()> onFrameCallback_{};
};

class ErrorMockSerialHandler : public MockSerialHandler {
public:
    esp_err_t sendMessage(std::string_view message) override {
        return ESP_FAIL;
    }
};
#endif //MOCKSERIALHANDLER_H
