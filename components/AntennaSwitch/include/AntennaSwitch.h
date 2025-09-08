#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <memory>
#include <chrono>
#include <unordered_map>
#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "cJSON.h"
#include "AntennaWebSocketClient.h"

namespace antenna {

/**
 * @brief Antenna information structure
 */
struct AntennaInfo {
    int id;
    std::string name;
    std::vector<std::string> compatibleBands;
    bool isActive;
    
    AntennaInfo() : id(-1), isActive(false) {}
    AntennaInfo(int antennaId, const std::string& antennaName) 
        : id(antennaId), name(antennaName), isActive(false) {}
};

/**
 * @brief Transport type for antenna switch communication
 */
enum class TransportType {
    Http,
    WebSocket,
    Auto  // Try WebSocket first, fallback to HTTP
};

/**
 * @brief Configuration for antenna switch API
 */
struct AntennaConfig {
    bool enabled = false;
    std::string baseUrl = "http://192.168.1.100";
    int timeoutMs = 1000;  // Request timeout
    bool fallbackToRadio = true;
    int maxRetries = 2;     // POST retries on failure
    std::string statusEndpoint = "/status";
    std::string controlEndpoint = "/api/antenna/switch";
    
    // Performance optimization settings
    bool useAsyncMode = true;
    int cacheExpirationMs = 60000;  // Cache antenna status for 60 seconds
    int connectionPoolSize = 1;     // Keep connection alive
    bool enableCompression = false; // Small responses: no compression needed
    
    // WebSocket configuration
    TransportType transportType = TransportType::Auto;
    std::string wsUrl = "ws://192.168.1.100/ws";
    bool enableWebSocketEvents = true;
    int wsReconnectTimeoutMs = 5000;
    int wsKeepAliveMs = 30000;
};

/**
 * @brief ESP32 Band Decoder Antenna Switch API Client
 * 
 * This class provides integration with the ESP32 Band Decoder antenna switch
 * system via HTTP API calls. It allows automatic antenna switching based on
 * frequency compatibility and provides fallback to radio's built-in antenna ports.
 */
class AntennaSwitch {
public:
    /**
     * @brief Constructor
     */
    AntennaSwitch();
    
    /**
     * @brief Destructor
     */
    ~AntennaSwitch();
    
    // Non-copyable, non-movable for safety
    AntennaSwitch(const AntennaSwitch&) = delete;
    AntennaSwitch& operator=(const AntennaSwitch&) = delete;
    AntennaSwitch(AntennaSwitch&&) = delete;
    AntennaSwitch& operator=(AntennaSwitch&&) = delete;
    
    /**
     * @brief Initialize the antenna switch with configuration
     * @param config Configuration parameters
     * @return true if initialization successful
     */
    bool initialize(const AntennaConfig& config);
    
    /**
     * @brief Switch to the next compatible antenna for current frequency
     * @param frequencyHz Current operating frequency in Hz
     * @return true if switch successful
     */
    bool switchToNextAntenna(uint64_t frequencyHz);
    
    /**
     * @brief Switch to a specific antenna by ID
     * @param antennaId Ignored - API uses "next" action for cycling
     * @return true if switch command sent successfully
     */
    bool switchToAntenna(int antennaId);
    
    /**
     * @brief Get list of antennas compatible with given frequency
     * @param frequencyHz Frequency in Hz to check compatibility
     * @return Vector of compatible antennas
     */
    std::vector<AntennaInfo> getCompatibleAntennas(uint64_t frequencyHz);
    
    /**
     * @brief Get current active antenna information
     * @return Current antenna info, or empty if unavailable
     */
    AntennaInfo getCurrentAntenna();
    
    /**
     * @brief Check if the antenna switch API is available
     * @return true if API responds to status requests
     */
    bool isApiAvailable();
    
    /**
     * @brief Check if network connectivity is ready
     * @return true if network interface has IP address
     */
    bool isNetworkReady();
    
    /**
     * @brief Get current configuration
     * @return Current antenna configuration
     */
    const AntennaConfig& getConfig() const { return config_; }
    
    /**
     * @brief Update configuration
     * @param config New configuration
     */
    void updateConfig(const AntennaConfig& config);
    
    /**
     * @brief Get last error message
     * @return Last error message string
     */
    std::string getLastError() const { return lastError_; }

private:
    static const char* TAG;
    
    AntennaConfig config_;
    esp_http_client_handle_t httpClient_;
    std::unique_ptr<AntennaWebSocketClient> wsClient_;
    std::string lastError_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> apiAvailable_{false};
    std::atomic<bool> wsAvailable_{false};
    std::atomic<TransportType> activeTransport_{TransportType::Http};
    
    // HTTP response buffer with optimized size
    std::string responseBuffer_;
    static const size_t MAX_RESPONSE_SIZE = 2048;  // Reduced buffer size
    
    // Performance optimization members
    struct CacheEntry {
        std::vector<AntennaInfo> antennas;
        AntennaInfo currentAntenna;
        std::chrono::steady_clock::time_point timestamp;
        uint64_t frequencyHz;
    };
    
    mutable CacheEntry cache_;
    std::atomic<bool> networkReady_{false};
    std::chrono::steady_clock::time_point lastNetworkCheck_;
    
    // Async operation support
    QueueHandle_t requestQueue_;
    TaskHandle_t workerTask_;
    std::atomic<bool> stopWorker_{false};
    
    /**
     * @brief HTTP event handler callback
     */
    static esp_err_t httpEventHandler(esp_http_client_event_t *evt);
    
    /**
     * @brief Create HTTP client configuration
     * @return HTTP client configuration
     */
    esp_http_client_config_t createHttpConfig();
    
    /**
     * @brief Perform HTTP GET request
     * @param endpoint API endpoint (relative to base URL)
     * @param response Reference to store response
     * @return true if request successful
     */
    bool httpGet(const std::string& endpoint, std::string& response);
    
    /**
     * @brief Perform HTTP POST request  
     * @param endpoint API endpoint (relative to base URL)
     * @param jsonPayload JSON payload to send
     * @param response Reference to store response
     * @return true if request successful
     */
    bool httpPost(const std::string& endpoint, const std::string& jsonPayload, std::string& response);
    
    /**
     * @brief Parse antenna status from JSON response using cJSON for efficiency
     * @param jsonResponse JSON response string
     * @return Vector of antenna information
     */
    std::vector<AntennaInfo> parseAntennaStatus(const std::string& jsonResponse, uint64_t frequencyHz = 14000000);
    
    /**
     * @brief Fast JSON parsing using cJSON library
     * @param jsonResponse JSON response string
     * @return Vector of antenna information
     */
    std::vector<AntennaInfo> parseAntennaStatusFast(const std::string& jsonResponse, uint64_t frequencyHz = 14000000);
    
    /**
     * @brief Determine band name from frequency
     * @param frequencyHz Frequency in Hz
     * @return Band name string (e.g. "20m", "40m")
     */
    static std::string getBandFromFrequency(uint64_t frequencyHz);
    
    /**
     * @brief Set last error message
     * @param error Error message
     */
    void setError(const std::string& error);
    
    /**
     * @brief Clean up HTTP client
     */
    void cleanup();
    
    /**
     * @brief Check if cached data is valid for given frequency
     * @param frequencyHz Frequency to check
     * @return true if cache is valid
     */
    bool isCacheValid(uint64_t frequencyHz) const;
    
    /**
     * @brief Update cache with new antenna data
     * @param antennas Antenna information
     * @param currentAntenna Current active antenna
     * @param frequencyHz Frequency for this cache entry
     */
    void updateCache(const std::vector<AntennaInfo>& antennas, const AntennaInfo& currentAntenna, uint64_t frequencyHz);
    
    /**
     * @brief Check network connectivity with caching
     * @return true if network is ready
     */
    bool isNetworkReadyCached();
    
    /**
     * @brief Configure HTTP client for optimal performance
     */
    void configureHttpClientOptimal();
    
    /**
     * @brief Worker task for async operations
     * @param pvParameters Task parameters
     */
    static void workerTaskFunction(void* pvParameters);
    
    /**
     * @brief Initialize async worker task
     * @return true if initialization successful
     */
    bool initializeAsyncWorker();
    
    /**
     * @brief Shutdown async worker task
     */
    void shutdownAsyncWorker();
    
    /**
     * @brief Initialize WebSocket client (without connecting)
     * @return true if initialization successful
     */
    bool initializeWebSocketClient();
    
    /**
     * @brief Ensure WebSocket connection is established
     * @return true if connected or connection successful
     */
    bool ensureWebSocketConnected();
    
    /**
     * @brief Handle WebSocket response messages
     * @param message WebSocket message
     */
    void handleWebSocketResponse(const WSMessage& message);
    
    /**
     * @brief Handle WebSocket event messages
     * @param message WebSocket event message
     */
    void handleWebSocketEvent(const WSMessage& message);
    
    /**
     * @brief Handle WebSocket error messages
     * @param message WebSocket error message
     */
    void handleWebSocketError(const WSMessage& message);
    
    /**
     * @brief Try WebSocket operation, fallback to HTTP if needed
     * @param wsOperation WebSocket operation function
     * @param httpFallback HTTP fallback function
     * @return true if operation successful
     */
    bool tryWebSocketWithHttpFallback(std::function<bool()> wsOperation, std::function<bool()> httpFallback);
    
    /**
     * @brief Get current antenna via WebSocket
     * @return Current antenna info, or empty if unavailable
     */
    AntennaInfo getCurrentAntennaWS();
    
    /**
     * @brief Get compatible antennas via WebSocket
     * @param frequencyHz Frequency in Hz
     * @return Vector of compatible antennas
     */
    std::vector<AntennaInfo> getCompatibleAntennasWS(uint64_t frequencyHz);
    
    /**
     * @brief Switch to next antenna via WebSocket
     * @param frequencyHz Current frequency
     * @return true if successful
     */
    bool switchToNextAntennaWS(uint64_t frequencyHz);
};

} // namespace antenna
