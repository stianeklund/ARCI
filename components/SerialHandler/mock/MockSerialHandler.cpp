#include "include/MockSerialHandler.h"

MockSerialHandler::MockSerialHandler(uart_port_t uart_num) {
    // uart_num parameter retained for API compatibility with SerialHandler
    // but not used in mock implementation
    (void)uart_num;
}

MockSerialHandler::~MockSerialHandler() {
    // Clear callbacks to prevent dangling function pointers
    setOnFrameCallback(nullptr);
}

esp_err_t MockSerialHandler::setupUart(int baud_rate) {
    return ESP_OK;
}

esp_err_t MockSerialHandler::setupUart(int baud_rate, int tx_pin, int rx_pin) {
    return ESP_OK;
}

esp_err_t MockSerialHandler::setupUart(int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin) {
    return ESP_OK;
}

bool MockSerialHandler::hasMessage() const {
    return m_nextMessageIndex < m_receivedMessages.size();
}

std::pair<esp_err_t, std::string> MockSerialHandler::getMessage() {
    if (m_nextMessageIndex >= m_receivedMessages.size()) {
        return {ESP_FAIL, ""};
    }
    std::string msg = m_receivedMessages[m_nextMessageIndex];
    m_nextMessageIndex++;
    return {ESP_OK, msg};
}

std::pair<esp_err_t, std::string_view> MockSerialHandler::getMessageView() {
    if (m_nextMessageIndex >= m_receivedMessages.size()) {
        return {ESP_FAIL, std::string_view{}};
    }
    m_lastMessageView = m_receivedMessages[m_nextMessageIndex];
    m_nextMessageIndex++;
    return {ESP_OK, std::string_view{m_lastMessageView}};
}

esp_err_t MockSerialHandler::sendMessage(std::string_view message) {
    sentMessages.emplace_back(message);
    return ESP_OK;
}

esp_err_t MockSerialHandler::sendMessage(std::string_view message1, std::string_view message2) {
    std::string combined;
    combined.reserve(message1.length() + message2.length());
    combined.append(message1);
    combined.append(message2);
    sentMessages.emplace_back(combined);
    return ESP_OK;
}

void MockSerialHandler::queueReceivedMessage(std::string_view message) {
    m_receivedMessages.emplace_back(message);
}
