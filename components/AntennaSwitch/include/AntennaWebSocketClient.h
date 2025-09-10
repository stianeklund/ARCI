#pragma once

#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include <chrono>
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "cJSON.h"

namespace antenna {

/**
 * @brief WebSocket message types according to API specification
 */
enum class WSMessageType {
    Request,
    Response, 
    Event,
    Error
};

/**
 * @brief WebSocket message structure for JSON communication
 */
struct WSMessage {
    std::string id;
    WSMessageType type;
    std::string action;
    std::string data;
    std::string event;
    
    WSMessage() = default;
    WSMessage(const std::string& msgId, WSMessageType msgType, const std::string& msgAction)
        : id(msgId), type(msgType), action(msgAction) {}
};

/**
 * @brief WebSocket event callback function type
 */
using WSEventCallback = std::function<void(const WSMessage&)>;

/**
 * @brief Configuration for WebSocket antenna switch client
 */
struct WSAntennaConfig {
    std::string wsUrl = "ws://192.168.1.100/ws";
    int reconnectTimeoutMs = 5000;
    int keepaliveIntervalMs = 30000;
    int responseTimeoutMs = 3000;
    bool autoReconnect = true;
    int maxReconnectAttempts = 5;
    int messageQueueSize = 20;
};

/**
 * @brief High-performance WebSocket client for antenna switching API
 * 
 * This class implements the WebSocket client protocol as specified in the
 * antenna switch API documentation, providing real-time bidirectional
 * communication with sub-10ms latency.
 */
class AntennaWebSocketClient {
public:
    /**
     * @brief Constructor
     */
    AntennaWebSocketClient();
    
    /**
     * @brief Destructor
     */
    ~AntennaWebSocketClient();
    
    // Non-copyable, non-movable for safety
    AntennaWebSocketClient(const AntennaWebSocketClient&) = delete;
    AntennaWebSocketClient& operator=(const AntennaWebSocketClient&) = delete;
    AntennaWebSocketClient(AntennaWebSocketClient&&) = delete;
    AntennaWebSocketClient& operator=(AntennaWebSocketClient&&) = delete;
    
    /**
     * @brief Initialize the WebSocket client
     * @param config WebSocket configuration
     * @return true if initialization successful
     */
    bool initialize(const WSAntennaConfig& config);
    
    /**
     * @brief Connect to WebSocket server
     * @return true if connection successful
     */
    bool connect();
    
    /**
     * @brief Disconnect from WebSocket server
     */
    void disconnect();
    
    /**
     * @brief Check if connected to server
     * @return true if connected
     */
    bool isConnected() const { return connected_.load(); }
    
    /**
     * @brief Send status query request
     * @param requestId Unique request ID for correlation
     * @return true if message sent successfully
     */
    bool sendStatusQuery(const std::string& requestId = "");
    
    /**
     * @brief Send relay control request
     * @param relay Relay number (1-16)
     * @param state Relay state (true = ON, false = OFF)
     * @param requestId Unique request ID for correlation
     * @return true if message sent successfully
     */
    bool sendRelayControl(int relay, bool state, const std::string& requestId = "");
    
    /**
     * @brief Send antenna switch request
     * @param radio Radio identifier ("A" or "B")
     * @param action Switch action ("next" or "previous")
     * @param requestId Unique request ID for correlation
     * @return true if message sent successfully
     */
    bool sendAntennaSwitch(const std::string& radio, const std::string& action, const std::string& requestId = "");
    
    /**
     * @brief Send configuration query request
     * @param requestId Unique request ID for correlation
     * @return true if message sent successfully
     */
    bool sendConfigQuery(const std::string& requestId = "");
    
    /**
     * @brief Send relay names query request
     * @param requestId Unique request ID for correlation
     * @return true if message sent successfully
     */
    bool sendRelayNamesQuery(const std::string& requestId = "");
    
    /**
     * @brief Subscribe to events
     * @param events List of event types to subscribe to
     * @param requestId Unique request ID for correlation
     * @return true if message sent successfully
     */
    bool subscribeToEvents(const std::vector<std::string>& events, const std::string& requestId = "");
    
    /**
     * @brief Unsubscribe from events
     * @param events List of event types to unsubscribe from
     * @param requestId Unique request ID for correlation
     * @return true if message sent successfully
     */
    bool unsubscribeFromEvents(const std::vector<std::string>& events, const std::string& requestId = "");
    
    /**
     * @brief Set response callback for handling server responses
     * @param callback Function to call when response received
     */
    void setResponseCallback(WSEventCallback callback) { responseCallback_ = std::move(callback); }
    
    /**
     * @brief Set event callback for handling server events
     * @param callback Function to call when event received
     */
    void setEventCallback(WSEventCallback callback) { eventCallback_ = std::move(callback); }
    
    /**
     * @brief Set error callback for handling errors
     * @param callback Function to call when error received
     */
    void setErrorCallback(WSEventCallback callback) { errorCallback_ = std::move(callback); }
    
    /**
     * @brief Get last error message
     * @return Last error message string
     */
    std::string getLastError() const { return lastError_; }
    
    /**
     * @brief Get connection statistics
     * @return Connection statistics structure
     */
    struct Stats {
        uint32_t messagesSent = 0;
        uint32_t messagesReceived = 0;
        uint32_t reconnectCount = 0;
        std::chrono::steady_clock::time_point lastConnectTime;
        std::chrono::steady_clock::time_point lastMessageTime;
    };
    
    Stats getStats() const { return stats_; }

private:
    static const char* TAG;
    
    WSAntennaConfig config_;
    esp_websocket_client_handle_t wsClient_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> reconnecting_{false};
    std::string lastError_;
    
    // Callbacks
    WSEventCallback responseCallback_;
    WSEventCallback eventCallback_; 
    WSEventCallback errorCallback_;
    
    // Message processing
    QueueHandle_t messageQueue_;
    TaskHandle_t messageProcessorTask_;
    std::atomic<bool> stopMessageProcessor_{false};
    
    // Connection management
    TaskHandle_t reconnectTask_;
    std::atomic<bool> stopReconnect_{false};
    SemaphoreHandle_t connectSemaphore_;
    
    // Statistics
    mutable Stats stats_;
    std::atomic<uint32_t> requestCounter_{1};
    
    // Receive buffer for message assembly
    std::string receiveBuffer_;
    SemaphoreHandle_t bufferMutex_;
    
    /**
     * @brief WebSocket event handler callback
     */
    static void websocketEventHandler(void* handler_args, esp_event_base_t base, 
                                      int32_t event_id, void* event_data);
    
    /**
     * @brief Handle WebSocket events
     */
    void handleWebSocketEvent(esp_websocket_event_id_t event_id, esp_websocket_event_data_t* event_data);
    
    /**
     * @brief Send JSON message to server
     * @param message JSON message object
     * @return true if sent successfully
     */
    bool sendMessage(const cJSON* message);
    
    /**
     * @brief Send raw JSON string to server
     * @param jsonStr JSON string to send
     * @return true if sent successfully
     */
    bool sendRawMessage(const std::string& jsonStr);
    
    /**
     * @brief Process received WebSocket message
     * @param data Message data
     * @param length Message length
     */
    void processReceivedMessage(const char* data, int length);
    
    /**
     * @brief Parse JSON message into WSMessage structure
     * @param jsonStr JSON string to parse
     * @return Parsed message or empty message if parsing failed
     */
    WSMessage parseMessage(const std::string& jsonStr);
    
    /**
     * @brief Generate unique request ID
     * @return Unique request ID string
     */
    std::string generateRequestId();
    
    /**
     * @brief Create JSON request message
     * @param action Action name
     * @param data Data object (can be nullptr)
     * @param requestId Request ID (optional)
     * @return JSON object or nullptr if creation failed
     */
    cJSON* createRequestMessage(const std::string& action, cJSON* data, const std::string& requestId = "");
    
    /**
     * @brief Set last error message
     * @param error Error message
     */
    void setError(const std::string& error);
    
    /**
     * @brief Message processor task function
     * @param pvParameters Task parameters
     */
    static void messageProcessorTaskFunction(void* pvParameters);
    
    /**
     * @brief Reconnect task function  
     * @param pvParameters Task parameters
     */
    static void reconnectTaskFunction(void* pvParameters);
    
    /**
     * @brief Start message processor task
     * @return true if started successfully
     */
    bool startMessageProcessor();
    
    /**
     * @brief Stop message processor task
     */
    void stopMessageProcessor();
    
    /**
     * @brief Start reconnect task
     * @return true if started successfully
     */
    bool startReconnectTask();
    
    /**
     * @brief Stop reconnect task
     */
    void stopReconnectTask();
    
    /**
     * @brief Attempt to reconnect to server
     * @return true if reconnection successful
     */
    bool attemptReconnect();
    
    /**
     * @brief Initialize WebSocket client handle
     * @return true if initialization successful
     */
    bool initializeWebSocketHandle();
    
    /**
     * @brief Cleanup WebSocket client resources
     */
    void cleanup();
};

} // namespace antenna