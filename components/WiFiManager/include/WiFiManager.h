#pragma once

#include <string>
#include <atomic>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

namespace wifi {

/**
 * @brief WiFi connection status
 */
enum class WiFiStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    FAILED
};

/**
 * @brief WiFi Manager for ESP32 Station Mode
 * 
 * Handles WiFi initialization, connection, and status monitoring.
 * Configured via KConfig for SSID and password.
 */
class WiFiManager {
public:
    static const char* TAG;
    
    WiFiManager();
    ~WiFiManager();
    
    // Non-copyable, non-movable
    WiFiManager(const WiFiManager&) = delete;
    WiFiManager& operator=(const WiFiManager&) = delete;
    WiFiManager(WiFiManager&&) = delete;
    WiFiManager& operator=(WiFiManager&&) = delete;
    
    /**
     * @brief Initialize WiFi subsystem
     * @return true if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Start WiFi connection
     * Uses SSID/password from KConfig
     * @return true if connection initiated successfully
     */
    bool connect();
    
    /**
     * @brief Disconnect from WiFi
     */
    void disconnect();
    
    /**
     * @brief Get current WiFi status
     * @return Current connection status
     */
    WiFiStatus getStatus() const;
    
    /**
     * @brief Check if WiFi is connected
     * @return true if connected and has IP address
     */
    bool isConnected() const;
    
    /**
     * @brief Get IP address as string
     * @return IP address string, or empty if not connected
     */
    std::string getIpAddress() const;
    
    /**
     * @brief Wait for WiFi connection with timeout
     * @param timeoutMs Timeout in milliseconds
     * @return true if connected within timeout
     */
    bool waitForConnection(uint32_t timeoutMs = 10000);

    /**
     * @brief Scan for available WiFi networks (diagnostic)
     * Logs all visible networks with RSSI signal strength.
     * Must be called after initialize() but before connect().
     * @return Number of networks found, or -1 on error
     */
    int scanNetworks();

private:
    static constexpr int WIFI_CONNECTED_BIT = BIT0;
    static constexpr int WIFI_FAIL_BIT = BIT1;
    
    EventGroupHandle_t wifiEventGroup_;
    esp_netif_t* netifSta_;
    std::atomic<WiFiStatus> status_;
    bool initialized_;
    int retryCount_;
    
    static void wifiEventHandler(void* arg, esp_event_base_t eventBase, 
                                int32_t eventId, void* eventData);
    static void ipEventHandler(void* arg, esp_event_base_t eventBase, 
                              int32_t eventId, void* eventData);
    
    void handleWifiEvent(esp_event_base_t eventBase, int32_t eventId, void* eventData);
    void cleanup();
};

} // namespace wifi