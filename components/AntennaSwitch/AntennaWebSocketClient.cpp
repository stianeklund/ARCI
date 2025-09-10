#include "AntennaWebSocketClient.h"
#include <sstream>
#include <iomanip>
#include "esp_timer.h"

namespace antenna {

const char* AntennaWebSocketClient::TAG = "AntennaWSClient";

AntennaWebSocketClient::AntennaWebSocketClient() 
    : wsClient_(nullptr), messageQueue_(nullptr), messageProcessorTask_(nullptr), 
      reconnectTask_(nullptr), connectSemaphore_(nullptr), bufferMutex_(nullptr) {
    ESP_LOGD(TAG, "AntennaWebSocketClient created");
    stats_.lastConnectTime = std::chrono::steady_clock::time_point{};
    stats_.lastMessageTime = std::chrono::steady_clock::time_point{};
}

AntennaWebSocketClient::~AntennaWebSocketClient() {
    cleanup();
}

bool AntennaWebSocketClient::initialize(const WSAntennaConfig& config) {
    config_ = config;
    
    ESP_LOGI(TAG, "Initializing WebSocket client for URL: %s", config_.wsUrl.c_str());
    
    // Create synchronization objects
    connectSemaphore_ = xSemaphoreCreateBinary();
    if (!connectSemaphore_) {
        setError("Failed to create connect semaphore");
        return false;
    }
    
    bufferMutex_ = xSemaphoreCreateMutex();
    if (!bufferMutex_) {
        setError("Failed to create buffer mutex");
        return false;
    }
    
    // Create message queue
    messageQueue_ = xQueueCreate(config_.messageQueueSize, sizeof(WSMessage*));
    if (!messageQueue_) {
        setError("Failed to create message queue");
        return false;
    }
    
    // Start message processor task
    if (!startMessageProcessor()) {
        setError("Failed to start message processor");
        return false;
    }
    
    // Initialize WebSocket client handle
    if (!initializeWebSocketHandle()) {
        setError("Failed to initialize WebSocket handle");
        return false;
    }
    
    initialized_.store(true);
    ESP_LOGI(TAG, "WebSocket client initialized successfully");
    return true;
}

bool AntennaWebSocketClient::connect() {
    if (!initialized_.load()) {
        setError("WebSocket client not initialized");
        return false;
    }
    
    if (connected_.load()) {
        ESP_LOGD(TAG, "Already connected");
        return true;
    }
    
    ESP_LOGI(TAG, "Connecting to WebSocket server: %s", config_.wsUrl.c_str());
    
    esp_err_t err = esp_websocket_client_start(wsClient_);
    if (err != ESP_OK) {
        setError("Failed to start WebSocket client: " + std::string(esp_err_to_name(err)));
        return false;
    }
    
    // Wait for connection with timeout
    if (xSemaphoreTake(connectSemaphore_, pdMS_TO_TICKS(config_.responseTimeoutMs)) == pdTRUE) {
        if (connected_.load()) {
            stats_.lastConnectTime = std::chrono::steady_clock::now();
            ESP_LOGI(TAG, "Connected to WebSocket server successfully");
            return true;
        }
    }
    
    setError("Connection timeout");
    return false;
}

void AntennaWebSocketClient::disconnect() {
    if (wsClient_ && connected_.load()) {
        ESP_LOGI(TAG, "Disconnecting from WebSocket server");
        esp_websocket_client_stop(wsClient_);
        connected_.store(false);
    }
}

bool AntennaWebSocketClient::sendStatusQuery(const std::string& requestId) {
    std::string reqId = requestId.empty() ? generateRequestId() : requestId;
    cJSON* request = createRequestMessage("status", nullptr, reqId);
    if (!request) {
        return false;
    }
    
    bool result = sendMessage(request);
    cJSON_Delete(request);
    return result;
}

bool AntennaWebSocketClient::sendRelayControl(int relay, bool state, const std::string& requestId) {
    if (relay < 1 || relay > 16) {
        setError("Invalid relay number: " + std::to_string(relay));
        return false;
    }
    
    std::string reqId = requestId.empty() ? generateRequestId() : requestId;
    
    cJSON* data = cJSON_CreateObject();
    if (!data) {
        setError("Failed to create data object");
        return false;
    }
    
    cJSON_AddNumberToObject(data, "relay", relay);
    cJSON_AddBoolToObject(data, "state", state);
    
    cJSON* request = createRequestMessage("relay_control", data, reqId);
    if (!request) {
        cJSON_Delete(data);
        return false;
    }
    
    bool result = sendMessage(request);
    cJSON_Delete(request);
    return result;
}

bool AntennaWebSocketClient::sendAntennaSwitch(const std::string& radio, const std::string& action, const std::string& requestId) {
    if (radio != "A" && radio != "B") {
        setError("Invalid radio identifier: " + radio);
        return false;
    }
    
    if (action != "next" && action != "previous") {
        setError("Invalid action: " + action);
        return false;
    }
    
    std::string reqId = requestId.empty() ? generateRequestId() : requestId;
    
    cJSON* data = cJSON_CreateObject();
    if (!data) {
        setError("Failed to create data object");
        return false;
    }
    
    cJSON_AddStringToObject(data, "radio", radio.c_str());
    cJSON_AddStringToObject(data, "action", action.c_str());
    
    cJSON* request = createRequestMessage("antenna_switch", data, reqId);
    if (!request) {
        cJSON_Delete(data);
        return false;
    }
    
    bool result = sendMessage(request);
    cJSON_Delete(request);
    return result;
}

bool AntennaWebSocketClient::sendConfigQuery(const std::string& requestId) {
    std::string reqId = requestId.empty() ? generateRequestId() : requestId;
    cJSON* request = createRequestMessage("config_basic", nullptr, reqId);
    if (!request) {
        return false;
    }
    
    bool result = sendMessage(request);
    cJSON_Delete(request);
    return result;
}

bool AntennaWebSocketClient::sendRelayNamesQuery(const std::string& requestId) {
    std::string reqId = requestId.empty() ? generateRequestId() : requestId;
    cJSON* request = createRequestMessage("relay_names", nullptr, reqId);
    if (!request) {
        return false;
    }
    
    bool result = sendMessage(request);
    cJSON_Delete(request);
    return result;
}

bool AntennaWebSocketClient::subscribeToEvents(const std::vector<std::string>& events, const std::string& requestId) {
    std::string reqId = requestId.empty() ? generateRequestId() : requestId;
    
    cJSON* data = cJSON_CreateObject();
    cJSON* eventArray = cJSON_CreateArray();
    
    if (!data || !eventArray) {
        setError("Failed to create JSON objects");
        if (data) cJSON_Delete(data);
        if (eventArray) cJSON_Delete(eventArray);
        return false;
    }
    
    for (const auto& event : events) {
        cJSON* eventStr = cJSON_CreateString(event.c_str());
        if (eventStr) {
            cJSON_AddItemToArray(eventArray, eventStr);
        }
    }
    
    cJSON_AddItemToObject(data, "events", eventArray);
    
    cJSON* request = createRequestMessage("subscribe", data, reqId);
    if (!request) {
        cJSON_Delete(data);
        return false;
    }
    
    bool result = sendMessage(request);
    cJSON_Delete(request);
    return result;
}

bool AntennaWebSocketClient::unsubscribeFromEvents(const std::vector<std::string>& events, const std::string& requestId) {
    std::string reqId = requestId.empty() ? generateRequestId() : requestId;
    
    cJSON* data = cJSON_CreateObject();
    cJSON* eventArray = cJSON_CreateArray();
    
    if (!data || !eventArray) {
        setError("Failed to create JSON objects");
        if (data) cJSON_Delete(data);
        if (eventArray) cJSON_Delete(eventArray);
        return false;
    }
    
    for (const auto& event : events) {
        cJSON* eventStr = cJSON_CreateString(event.c_str());
        if (eventStr) {
            cJSON_AddItemToArray(eventArray, eventStr);
        }
    }
    
    cJSON_AddItemToObject(data, "events", eventArray);
    
    cJSON* request = createRequestMessage("unsubscribe", data, reqId);
    if (!request) {
        cJSON_Delete(data);
        return false;
    }
    
    bool result = sendMessage(request);
    cJSON_Delete(request);
    return result;
}

void AntennaWebSocketClient::websocketEventHandler(void* handler_args, esp_event_base_t base, 
                                                   int32_t event_id, void* event_data) {
    AntennaWebSocketClient* client = static_cast<AntennaWebSocketClient*>(handler_args);
    esp_websocket_event_data_t* data = static_cast<esp_websocket_event_data_t*>(event_data);
    client->handleWebSocketEvent(static_cast<esp_websocket_event_id_t>(event_id), data);
}

void AntennaWebSocketClient::handleWebSocketEvent(esp_websocket_event_id_t event_id, esp_websocket_event_data_t* event_data) {
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket connected");
            connected_.store(true);
            reconnecting_.store(false);
            xSemaphoreGive(connectSemaphore_);
            break;
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket disconnected");
            connected_.store(false);
            if (config_.autoReconnect && !reconnecting_.load()) {
                ESP_LOGI(TAG, "Starting reconnection task");
                reconnecting_.store(true);
                if (!startReconnectTask()) {
                    ESP_LOGW(TAG, "Failed to start reconnection task");
                    reconnecting_.store(false);
                }
            }
            break;
            
        case WEBSOCKET_EVENT_DATA:
            if (event_data->op_code == 0x01) { // Text frame
                processReceivedMessage((const char*)event_data->data_ptr, event_data->data_len);
                stats_.messagesReceived++;
                stats_.lastMessageTime = std::chrono::steady_clock::now();
            }
            break;
            
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGW(TAG, "WebSocket error occurred");
            setError("WebSocket connection error");
            connected_.store(false);
            break;
            
        default:
            break;
    }
}

bool AntennaWebSocketClient::sendMessage(const cJSON* message) {
    if (!message || !connected_.load()) {
        return false;
    }
    
    char* jsonString = cJSON_Print(message);
    if (!jsonString) {
        setError("Failed to serialize JSON message");
        return false;
    }
    
    bool result = sendRawMessage(jsonString);
    free(jsonString);
    return result;
}

bool AntennaWebSocketClient::sendRawMessage(const std::string& jsonStr) {
    if (!connected_.load() || !wsClient_) {
        setError("WebSocket not connected");
        return false;
    }
    
    ESP_LOGD(TAG, "Sending: %s", jsonStr.c_str());
    
    esp_err_t err = esp_websocket_client_send_text(wsClient_, jsonStr.c_str(), jsonStr.length(), 
                                                   pdMS_TO_TICKS(config_.responseTimeoutMs));
    if (err != ESP_OK) {
        setError("Failed to send WebSocket message: " + std::string(esp_err_to_name(err)));
        return false;
    }
    
    stats_.messagesSent++;
    return true;
}

void AntennaWebSocketClient::processReceivedMessage(const char* data, int length) {
    if (!data || length <= 0) {
        return;
    }
    
    // Thread-safe buffer access
    if (xSemaphoreTake(bufferMutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire buffer mutex");
        return;
    }
    
    receiveBuffer_.append(data, length);
    
    // Check if we have a complete message (simple approach - assume one message per frame)
    std::string completeMessage = receiveBuffer_;
    receiveBuffer_.clear();
    
    xSemaphoreGive(bufferMutex_);
    
    ESP_LOGD(TAG, "Received: %s", completeMessage.c_str());
    
    // Parse and queue message for processing
    WSMessage parsedMessage = parseMessage(completeMessage);
    if (!parsedMessage.action.empty() || parsedMessage.type == WSMessageType::Event) {
        WSMessage* msgPtr = new WSMessage(std::move(parsedMessage));
        if (xQueueSend(messageQueue_, &msgPtr, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Message queue full, dropping message");
            delete msgPtr;
        }
    }
}

WSMessage AntennaWebSocketClient::parseMessage(const std::string& jsonStr) {
    WSMessage message;
    
    cJSON* json = cJSON_Parse(jsonStr.c_str());
    if (!json) {
        ESP_LOGW(TAG, "Failed to parse JSON message");
        return message;
    }
    
    // Parse ID
    cJSON* id = cJSON_GetObjectItem(json, "id");
    if (cJSON_IsString(id)) {
        message.id = id->valuestring;
    }
    
    // Parse type
    cJSON* type = cJSON_GetObjectItem(json, "type");
    if (cJSON_IsString(type)) {
        std::string typeStr = type->valuestring;
        if (typeStr == "response") {
            message.type = WSMessageType::Response;
        } else if (typeStr == "event") {
            message.type = WSMessageType::Event;
        } else if (typeStr == "error") {
            message.type = WSMessageType::Error;
        } else {
            message.type = WSMessageType::Request;
        }
    }
    
    // Parse action
    cJSON* action = cJSON_GetObjectItem(json, "action");
    if (cJSON_IsString(action)) {
        message.action = action->valuestring;
    }
    
    // Parse event
    cJSON* event = cJSON_GetObjectItem(json, "event");
    if (cJSON_IsString(event)) {
        message.event = event->valuestring;
    }
    
    // Parse data (serialize back to string for simplicity)
    cJSON* data = cJSON_GetObjectItem(json, "data");
    if (data) {
        char* dataStr = cJSON_Print(data);
        if (dataStr) {
            message.data = dataStr;
            free(dataStr);
        }
    }
    
    cJSON_Delete(json);
    return message;
}

std::string AntennaWebSocketClient::generateRequestId() {
    uint32_t counter = requestCounter_.fetch_add(1);
    uint64_t timestamp = esp_timer_get_time();
    
    std::ostringstream oss;
    oss << "req-" << std::hex << timestamp << "-" << counter;
    return oss.str();
}

cJSON* AntennaWebSocketClient::createRequestMessage(const std::string& action, cJSON* data, const std::string& requestId) {
    cJSON* request = cJSON_CreateObject();
    if (!request) {
        setError("Failed to create JSON object");
        return nullptr;
    }
    
    std::string reqId = requestId.empty() ? generateRequestId() : requestId;
    
    cJSON_AddStringToObject(request, "id", reqId.c_str());
    cJSON_AddStringToObject(request, "type", "request");
    cJSON_AddStringToObject(request, "action", action.c_str());
    
    if (data) {
        cJSON_AddItemToObject(request, "data", data);
    }
    
    return request;
}

void AntennaWebSocketClient::setError(const std::string& error) {
    lastError_ = error;
    ESP_LOGW(TAG, "Error: %s", error.c_str());
}

void AntennaWebSocketClient::messageProcessorTaskFunction(void* pvParameters) {
    AntennaWebSocketClient* client = static_cast<AntennaWebSocketClient*>(pvParameters);
    ESP_LOGI(client->TAG, "Message processor task started");
    
    while (!client->stopMessageProcessor_.load()) {
        WSMessage* msgPtr = nullptr;
        
        if (xQueueReceive(client->messageQueue_, &msgPtr, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (msgPtr) {
                // Dispatch message to appropriate callback
                switch (msgPtr->type) {
                    case WSMessageType::Response:
                        if (client->responseCallback_) {
                            client->responseCallback_(*msgPtr);
                        }
                        break;
                    case WSMessageType::Event:
                        if (client->eventCallback_) {
                            client->eventCallback_(*msgPtr);
                        }
                        break;
                    case WSMessageType::Error:
                        if (client->errorCallback_) {
                            client->errorCallback_(*msgPtr);
                        }
                        break;
                    default:
                        ESP_LOGD(client->TAG, "Unhandled message type");
                        break;
                }
                
                delete msgPtr;
            }
        }
    }
    
    ESP_LOGI(client->TAG, "Message processor task stopped");
    vTaskDelete(nullptr);
}

void AntennaWebSocketClient::reconnectTaskFunction(void* pvParameters) {
    AntennaWebSocketClient* client = static_cast<AntennaWebSocketClient*>(pvParameters);
    ESP_LOGI(client->TAG, "Reconnect task started");
    
    int attempts = 0;
    while (!client->stopReconnect_.load() && attempts < client->config_.maxReconnectAttempts) {
        vTaskDelay(pdMS_TO_TICKS(client->config_.reconnectTimeoutMs));
        
        if (client->connected_.load()) {
            ESP_LOGI(client->TAG, "Already reconnected, stopping reconnect task");
            break;
        }
        
        attempts++;
        ESP_LOGI(client->TAG, "Reconnection attempt %d/%d", attempts, client->config_.maxReconnectAttempts);
        
        if (client->attemptReconnect()) {
            ESP_LOGI(client->TAG, "Reconnection successful");
            client->stats_.reconnectCount++;
            break;
        }
    }
    
    client->reconnecting_.store(false);
    ESP_LOGI(client->TAG, "Reconnect task stopped");
    vTaskDelete(nullptr);
}

bool AntennaWebSocketClient::startMessageProcessor() {
    BaseType_t result = xTaskCreate(
        messageProcessorTaskFunction,
        "ws_msg_proc",
        4096,
        this,
        5,
        &messageProcessorTask_
    );
    
    return result == pdPASS;
}

void AntennaWebSocketClient::stopMessageProcessor() {
    if (messageProcessorTask_) {
        stopMessageProcessor_.store(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        messageProcessorTask_ = nullptr;
        stopMessageProcessor_.store(false);
    }
}

bool AntennaWebSocketClient::startReconnectTask() {
    if (reconnectTask_) {
        return true; // Already running
    }
    
    BaseType_t result = xTaskCreate(
        reconnectTaskFunction,
        "ws_reconnect",
        4096,
        this,
        4,
        &reconnectTask_
    );
    
    return result == pdPASS;
}

void AntennaWebSocketClient::stopReconnectTask() {
    if (reconnectTask_) {
        stopReconnect_.store(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        reconnectTask_ = nullptr;
        stopReconnect_.store(false);
    }
}

bool AntennaWebSocketClient::attemptReconnect() {
    ESP_LOGI(TAG, "Attempting to reconnect...");
    
    if (wsClient_) {
        esp_websocket_client_stop(wsClient_);
        esp_websocket_client_destroy(wsClient_);
        wsClient_ = nullptr;
    }
    
    if (!initializeWebSocketHandle()) {
        return false;
    }
    
    return connect();
}

bool AntennaWebSocketClient::initializeWebSocketHandle() {
    esp_websocket_client_config_t wsConfig = {};
    wsConfig.uri = config_.wsUrl.c_str();
    wsConfig.task_stack = 4096;
    wsConfig.task_prio = 5;
    wsConfig.keep_alive_idle = config_.keepaliveIntervalMs / 1000;
    wsConfig.keep_alive_interval = 10;
    wsConfig.keep_alive_count = 3;
    wsConfig.reconnect_timeout_ms = config_.reconnectTimeoutMs;
    wsConfig.network_timeout_ms = config_.responseTimeoutMs;
    
    wsClient_ = esp_websocket_client_init(&wsConfig);
    if (!wsClient_) {
        setError("Failed to initialize WebSocket client handle");
        return false;
    }
    
    esp_err_t err = esp_websocket_register_events(wsClient_, WEBSOCKET_EVENT_ANY, 
                                                  websocketEventHandler, this);
    if (err != ESP_OK) {
        setError("Failed to register WebSocket event handler");
        esp_websocket_client_destroy(wsClient_);
        wsClient_ = nullptr;
        return false;
    }
    
    return true;
}

void AntennaWebSocketClient::cleanup() {
    stopReconnectTask();
    stopMessageProcessor();
    
    if (wsClient_) {
        if (connected_.load()) {
            esp_websocket_client_stop(wsClient_);
        }
        esp_websocket_client_destroy(wsClient_);
        wsClient_ = nullptr;
    }
    
    if (messageQueue_) {
        // Clear any remaining messages
        WSMessage* msgPtr = nullptr;
        while (xQueueReceive(messageQueue_, &msgPtr, 0) == pdTRUE) {
            delete msgPtr;
        }
        vQueueDelete(messageQueue_);
        messageQueue_ = nullptr;
    }
    
    if (connectSemaphore_) {
        vSemaphoreDelete(connectSemaphore_);
        connectSemaphore_ = nullptr;
    }
    
    if (bufferMutex_) {
        vSemaphoreDelete(bufferMutex_);
        bufferMutex_ = nullptr;
    }
    
    connected_.store(false);
    initialized_.store(false);
    
    ESP_LOGD(TAG, "WebSocket client cleanup complete");
}

} // namespace antenna