#include "AntennaSwitch.h"
#include <cstring>
#include <sstream>
#include <cstdlib>
#include <climits>
#include "esp_netif.h"
#include "esp_wifi.h"
#include "cJSON.h"

namespace antenna {

const char* AntennaSwitch::TAG = "AntennaSwitch";

AntennaSwitch::AntennaSwitch() : httpClient_(nullptr), wsClient_(nullptr), requestQueue_(nullptr), workerTask_(nullptr) {
    ESP_LOGD(TAG, "AntennaSwitch created");
    
    // Initialize cache
    cache_.timestamp = std::chrono::steady_clock::time_point{};
    cache_.frequencyHz = 0;
    
    // Initialize network check timestamp
    lastNetworkCheck_ = std::chrono::steady_clock::time_point{};
}

AntennaSwitch::~AntennaSwitch() {
    shutdownAsyncWorker();
    cleanup();
}

bool AntennaSwitch::initialize(const AntennaConfig& config) {
    config_ = config;
    
    if (!config_.enabled) {
        ESP_LOGI(TAG, "AntennaSwitch disabled in configuration");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing AntennaSwitch with HTTP: %s, WebSocket: %s", 
             config_.baseUrl.c_str(), config_.wsUrl.c_str());
    
    // Clean up any existing clients
    cleanup();
    
    bool success = true;
    
    // Initialize HTTP client (always available as fallback)
    esp_http_client_config_t httpConfig = createHttpConfig();
    httpClient_ = esp_http_client_init(&httpConfig);
    
    if (!httpClient_) {
        setError("Failed to initialize HTTP client");
        return false;
    }
    
    configureHttpClientOptimal();
    
    // Initialize WebSocket client if requested - but defer connection until network ready
    if (config_.transportType == TransportType::WebSocket || config_.transportType == TransportType::Auto) {
        if (!initializeWebSocketClient()) {
            ESP_LOGW(TAG, "Failed to initialize WebSocket client, will use HTTP only");
            if (config_.transportType == TransportType::WebSocket) {
                success = false; // WebSocket required but failed
            }
        } else {
            ESP_LOGI(TAG, "WebSocket client initialized (connection deferred until network ready)");
        }
    }
    
    // Set initial transport preference
    if (wsClient_ && wsAvailable_.load()) {
        activeTransport_.store(TransportType::WebSocket);
        ESP_LOGI(TAG, "WebSocket transport active");
    } else {
        activeTransport_.store(TransportType::Http);
        ESP_LOGI(TAG, "HTTP transport active");
    }
    
    // Initialize async worker if enabled
    if (config_.useAsyncMode && !initializeAsyncWorker()) {
        ESP_LOGW(TAG, "Failed to initialize async worker, continuing in sync mode");
        config_.useAsyncMode = false;
    }
    
    initialized_.store(true);
    ESP_LOGI(TAG, "AntennaSwitch initialized successfully (async: %s, transport: %s)", 
             config_.useAsyncMode ? "enabled" : "disabled",
             activeTransport_.load() == TransportType::WebSocket ? "WebSocket" : "HTTP");
    return success;
}

bool AntennaSwitch::switchToNextAntenna(uint64_t frequencyHz) {
    if (!config_.enabled || !initialized_.load()) {
        ESP_LOGD(TAG, "AntennaSwitch not enabled or initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Switching to next antenna for frequency %.3f MHz", frequencyHz / 1e6);
    
    // Try WebSocket first if available, fallback to HTTP
    if (activeTransport_.load() == TransportType::WebSocket && wsClient_) {
        return tryWebSocketWithHttpFallback(
            [this, frequencyHz]() { return switchToNextAntennaWS(frequencyHz); },
            [this]() { return switchToAntenna(0); }
        );
    }
    
    // Use HTTP transport
    return switchToAntenna(0);
}

bool AntennaSwitch::switchToAntenna(int antennaId) {
    if (!config_.enabled || !initialized_.load()) {
        ESP_LOGD(TAG, "AntennaSwitch not enabled or initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Switching to next antenna (API only supports cycling, not direct selection to ID %d)", antennaId);
    
    // Create JSON payload for antenna switch request - API only supports "next" or "previous"
    std::ostringstream jsonPayload;
    jsonPayload << "{\"radio\":\"A\",\"action\":\"next\"}";
    
    ESP_LOGI(TAG, "POST %s%s with payload: %s", config_.baseUrl.c_str(), config_.controlEndpoint.c_str(), jsonPayload.str().c_str());
    
    std::string response;
    bool success = httpPost(config_.controlEndpoint, jsonPayload.str(), response);
    
    if (!success) {
        setError("Failed to send antenna switch request");
        return false;
    }
    
    ESP_LOGI(TAG, "Antenna switch request sent successfully");
    return true;
}

std::vector<AntennaInfo> AntennaSwitch::getCompatibleAntennas(uint64_t frequencyHz) {
    std::vector<AntennaInfo> antennas;
    
    if (!config_.enabled || !initialized_.load()) {
        ESP_LOGD(TAG, "AntennaSwitch not enabled or initialized");
        return antennas;
    }
    
    // Check cache first for significant performance improvement
    if (isCacheValid(frequencyHz)) {
        ESP_LOGD(TAG, "Returning cached antenna data for frequency %.3f MHz", frequencyHz / 1e6);
        return cache_.antennas;
    }
    
    // Try WebSocket first if available, fallback to HTTP
    if (activeTransport_.load() == TransportType::WebSocket && wsClient_) {
        antennas = getCompatibleAntennasWS(frequencyHz);
        if (!antennas.empty()) {
            return antennas;
        }
        ESP_LOGW(TAG, "WebSocket request failed, falling back to HTTP");
    }
    
    // HTTP fallback
    std::string response;
    if (!httpGet(config_.statusEndpoint, response)) {
        setError("Failed to get antenna status");
        return antennas;
    }
    
    // Use fast JSON parsing for better performance
    antennas = parseAntennaStatusFast(response, frequencyHz);
    
    // Update cache
    AntennaInfo currentAntenna;
    for (const auto& ant : antennas) {
        if (ant.isActive) {
            currentAntenna = ant;
            break;
        }
    }
    updateCache(antennas, currentAntenna, frequencyHz);
    
    std::string currentBand = getBandFromFrequency(frequencyHz);
    ESP_LOGI(TAG, "Found %d compatible antennas for band %s (cached for %ds)", antennas.size(), currentBand.c_str(), config_.cacheExpirationMs/1000);
    
    return antennas;
}

AntennaInfo AntennaSwitch::getCurrentAntenna() {
    AntennaInfo currentAntenna;
    
    if (!config_.enabled || !initialized_.load()) {
        ESP_LOGD(TAG, "AntennaSwitch not enabled or initialized");
        return currentAntenna;
    }
    
    // Use cache if available and fresh
    if (cache_.timestamp != std::chrono::steady_clock::time_point{}) {
        auto now = std::chrono::steady_clock::now();
        auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - cache_.timestamp).count();
        if (age < config_.cacheExpirationMs && cache_.currentAntenna.id != -1) {
        ESP_LOGD(TAG, "Returning cached current antenna: %s", cache_.currentAntenna.name.c_str());
        return cache_.currentAntenna;
        }
    }
    
    // Cache miss - fetch from API
    std::string response;
    if (!httpGet(config_.statusEndpoint, response)) {
        setError("Failed to get current antenna status");
        return currentAntenna;
    }
    
    auto antennas = parseAntennaStatusFast(response, 14000000); // Use 20m as default freq
    
    // Find the active antenna
    for (const auto& antenna : antennas) {
        if (antenna.isActive) {
            currentAntenna = antenna;
            break;
        }
    }
    
    // Update cache
    updateCache(antennas, currentAntenna, 14000000);
    
    return currentAntenna;
}

bool AntennaSwitch::isNetworkReady() {
    return isNetworkReadyCached();
}

bool AntennaSwitch::isNetworkReadyCached() {
    auto now = std::chrono::steady_clock::now();
    
    // Cache network status for 1 second to avoid repeated system calls
    if (lastNetworkCheck_ != std::chrono::steady_clock::time_point{}) {
        auto timeSinceCheck = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastNetworkCheck_).count();
        if (timeSinceCheck < 1000 && networkReady_.load()) {
            return true;
        }
    }
    
    // Check if network interface is up and has IP
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == nullptr) {
        ESP_LOGD(TAG, "WiFi STA interface not found");
        networkReady_.store(false);
        lastNetworkCheck_ = now;
        return false;
    }
    
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        ESP_LOGD(TAG, "Failed to get IP info");
        networkReady_.store(false);
        lastNetworkCheck_ = now;
        return false;
    }
    
    bool hasIp = (ip_info.ip.addr != 0);
    networkReady_.store(hasIp);
    lastNetworkCheck_ = now;
    
    if (hasIp) {
        ESP_LOGD(TAG, "Network ready with IP: " IPSTR, IP2STR(&ip_info.ip));
    } else {
        ESP_LOGD(TAG, "No IP address assigned");
    }
    
    return hasIp;
}

bool AntennaSwitch::isApiAvailable() {
    // Non-blocking check: rely on background worker to update apiAvailable_
    if (!config_.enabled) return false;
    if (!isNetworkReadyCached()) return false;
    return apiAvailable_.load();
}

void AntennaSwitch::updateConfig(const AntennaConfig& config) {
    bool wasEnabled = config_.enabled;
    config_ = config;
    
    if (config_.enabled && !wasEnabled) {
        // Re-initialize if enabling
        initialize(config_);
    } else if (!config_.enabled && wasEnabled) {
        // Clean up if disabling
        cleanup();
        initialized_.store(false);
    }
}

esp_http_client_config_t AntennaSwitch::createHttpConfig() {
    esp_http_client_config_t config = {};
    config.url = config_.baseUrl.c_str();
    config.event_handler = httpEventHandler;
    config.user_data = this;
    config.timeout_ms = config_.timeoutMs;
    config.buffer_size = MAX_RESPONSE_SIZE;
    config.buffer_size_tx = 1024;
    
    // Enable hostname resolution via DNS
    config.skip_cert_common_name_check = true;  // Allow hostname resolution
    config.transport_type = HTTP_TRANSPORT_OVER_TCP;
    
    // Performance optimizations
    config.keep_alive_enable = true;           // Keep connection alive
    config.keep_alive_idle = 60;               // 60s idle before keep-alive pings
    config.keep_alive_interval = 10;           // ping interval
    config.keep_alive_count = 3;              // Keep alive retry count
    config.disable_auto_redirect = true;      // Disable redirects for speed
    
    ESP_LOGI(TAG, "HTTP client configured for URL: %s (timeout: %d ms, keep-alive: enabled)", 
             config_.baseUrl.c_str(), config_.timeoutMs);
    
    return config;
}

esp_err_t AntennaSwitch::httpEventHandler(esp_http_client_event_t *evt) {
    AntennaSwitch* instance = static_cast<AntennaSwitch*>(evt->user_data);
    
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGW(instance->TAG, "HTTP_EVENT_ERROR - possible DNS resolution or connection failure");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(instance->TAG, "HTTP_EVENT_ON_CONNECTED - successfully resolved hostname and connected");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(instance->TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(instance->TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", 
                    evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(instance->TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                if (evt->data_len > 0 && instance->responseBuffer_.size() + evt->data_len < MAX_RESPONSE_SIZE) {
                    instance->responseBuffer_.append(static_cast<char*>(evt->data), evt->data_len);
                }
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(instance->TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(instance->TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
    return ESP_OK;
}

bool AntennaSwitch::httpGet(const std::string& endpoint, std::string& response) {
    if (!httpClient_) {
        setError("HTTP client not initialized");
        return false;
    }
    
    // Clear response buffer
    responseBuffer_.clear();
    
    // Build full URL
    std::string fullUrl = config_.baseUrl + endpoint;
    
    ESP_LOGD(TAG, "HTTP GET: %s", fullUrl.c_str());
    
    esp_err_t err = esp_http_client_set_url(httpClient_, fullUrl.c_str());
    if (err != ESP_OK) {
        setError("Failed to set HTTP URL");
        return false;
    }
    
    esp_http_client_set_method(httpClient_, HTTP_METHOD_GET);
    
    err = esp_http_client_perform(httpClient_);
    if (err != ESP_OK) {
        std::string errorMsg = "HTTP GET request failed: " + std::string(esp_err_to_name(err));
        // Common network/DNS related error codes
        if (err == ESP_FAIL || err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND) {
            errorMsg += " (DNS resolution or connection failed - check hostname and network)";
        }
        setError(errorMsg);
        ESP_LOGW(TAG, "HTTP GET failed to %s: %s", fullUrl.c_str(), errorMsg.c_str());
        return false;
    }
    
    int status_code = esp_http_client_get_status_code(httpClient_);
    if (status_code >= 200 && status_code < 300) {
        response = responseBuffer_;
        apiAvailable_.store(true);
        return true;
    } else {
        setError("HTTP GET failed with status: " + std::to_string(status_code));
        apiAvailable_.store(false);
        return false;
    }
}

bool AntennaSwitch::httpPost(const std::string& endpoint, const std::string& jsonPayload, std::string& response) {
    if (!httpClient_) {
        setError("HTTP client not initialized");
        return false;
    }
    
    // Clear response buffer
    responseBuffer_.clear();
    
    // Build full URL
    std::string fullUrl = config_.baseUrl + endpoint;
    
    ESP_LOGD(TAG, "HTTP POST: %s", fullUrl.c_str());
    ESP_LOGD(TAG, "Payload: %s", jsonPayload.c_str());
    
    esp_err_t err = esp_http_client_set_url(httpClient_, fullUrl.c_str());
    if (err != ESP_OK) {
        setError("Failed to set HTTP URL");
        return false;
    }
    
    esp_http_client_set_method(httpClient_, HTTP_METHOD_POST);
    esp_http_client_set_header(httpClient_, "Content-Type", "application/json");
    esp_http_client_set_post_field(httpClient_, jsonPayload.c_str(), jsonPayload.length());
    
    err = esp_http_client_perform(httpClient_);
    if (err != ESP_OK) {
        std::string errorMsg = "HTTP POST request failed: " + std::string(esp_err_to_name(err));
        // Common network/DNS related error codes
        if (err == ESP_FAIL || err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND) {
            errorMsg += " (DNS resolution or connection failed - check hostname and network)";
        }
        setError(errorMsg);
        ESP_LOGW(TAG, "HTTP POST failed to %s: %s", fullUrl.c_str(), errorMsg.c_str());
        return false;
    }
    
    int status_code = esp_http_client_get_status_code(httpClient_);
    if (status_code >= 200 && status_code < 300) {
        response = responseBuffer_;
        apiAvailable_.store(true);
        return true;
    } else {
        setError("HTTP POST failed with status: " + std::to_string(status_code));
        apiAvailable_.store(false);
        return false;
    }
}

std::vector<AntennaInfo> AntennaSwitch::parseAntennaStatus(const std::string& jsonResponse, uint64_t frequencyHz) {
    std::vector<AntennaInfo> antennas;
    
    ESP_LOGD(TAG, "Parsing antenna status from: %s", jsonResponse.c_str());
    
    if (jsonResponse.empty()) {
        ESP_LOGW(TAG, "Empty JSON response");
        return antennas;
    }
    
    // Parse current antenna name (e.g., "antenna": "Antenna 1")
    std::string currentAntennaName;
    size_t antennaPos = jsonResponse.find("\"antenna\":");
    if (antennaPos != std::string::npos) {
        size_t startQuote = jsonResponse.find("\"", antennaPos + 10);
        size_t endQuote = jsonResponse.find("\"", startQuote + 1);
        if (startQuote != std::string::npos && endQuote != std::string::npos) {
            currentAntennaName = jsonResponse.substr(startQuote + 1, endQuote - startQuote - 1);
            ESP_LOGD(TAG, "Current antenna: %s", currentAntennaName.c_str());
        }
    }
    
    // Parse available_antennas array (e.g., "available_antennas": [1, 5])
    size_t availablePos = jsonResponse.find("\"available_antennas\":");
    if (availablePos != std::string::npos) {
        size_t arrayStart = jsonResponse.find("[", availablePos);
        size_t arrayEnd = jsonResponse.find("]", arrayStart);
        if (arrayStart != std::string::npos && arrayEnd != std::string::npos) {
            std::string arrayContent = jsonResponse.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
            ESP_LOGD(TAG, "Available antennas array: [%s]", arrayContent.c_str());
            
            // Parse individual antenna IDs
            std::istringstream stream(arrayContent);
            std::string item;
            
            while (std::getline(stream, item, ',')) {
                // Remove whitespace
                item.erase(0, item.find_first_not_of(" \t"));
                item.erase(item.find_last_not_of(" \t") + 1);
                
                if (!item.empty()) {
                    // Convert string to integer without exceptions
                    char* endPtr = nullptr;
                    long antennaId = strtol(item.c_str(), &endPtr, 10);
                    
                    // Check if conversion was successful
                    if (endPtr != item.c_str() && *endPtr == '\0' && antennaId > 0 && antennaId <= INT_MAX) {
                        AntennaInfo antenna;
                        antenna.id = static_cast<int>(antennaId);
                        antenna.name = "Antenna " + std::to_string(antenna.id);
                        antenna.isActive = (antenna.name == currentAntennaName);
                        
                        // All available antennas are compatible with current frequency/band
                        std::string currentBand = getBandFromFrequency(frequencyHz);
                        antenna.compatibleBands = {currentBand};
                        
                        antennas.push_back(antenna);
                        ESP_LOGD(TAG, "Added antenna ID %d (%s) - active: %s", 
                                antenna.id, antenna.name.c_str(), antenna.isActive ? "yes" : "no");
                    } else {
                        ESP_LOGW(TAG, "Failed to parse antenna ID: %s", item.c_str());
                    }
                }
            }
        }
    }
    
    ESP_LOGI(TAG, "Parsed %d available antennas", antennas.size());
    return antennas;
}

std::string AntennaSwitch::getBandFromFrequency(const uint64_t frequencyHz) {
    // Convert Hz to MHz for easier comparison
    const double freqMHz = frequencyHz / 1e6;
    
    if (freqMHz >= 1.8 && freqMHz <= 2.0) return "160m";
    if (freqMHz >= 3.5 && freqMHz <= 4.0) return "80m";
    if (freqMHz >= 7.0 && freqMHz <= 7.3) return "40m";
    if (freqMHz >= 10.1 && freqMHz <= 10.15) return "30m";
    if (freqMHz >= 14.0 && freqMHz <= 14.35) return "20m";
    if (freqMHz >= 18.068 && freqMHz <= 18.168) return "17m";
    if (freqMHz >= 21.0 && freqMHz <= 21.45) return "15m";
    if (freqMHz >= 24.89 && freqMHz <= 24.99) return "12m";
    if (freqMHz >= 28.0 && freqMHz <= 29.7) return "10m";
    if (freqMHz >= 50.0 && freqMHz <= 54.0) return "6m";
    
    return "unknown";
}

void AntennaSwitch::setError(const std::string& error) {
    lastError_ = error;
    ESP_LOGW(TAG, "Error: %s", error.c_str());
}

void AntennaSwitch::cleanup() {
    shutdownAsyncWorker();
    
    if (wsClient_) {
        wsClient_->disconnect();
        wsClient_.reset();
    }
    
    if (httpClient_) {
        esp_http_client_cleanup(httpClient_);
        httpClient_ = nullptr;
    }
    responseBuffer_.clear();
    
    // Clear cache
    cache_.antennas.clear();
    cache_.currentAntenna = AntennaInfo{};
    cache_.timestamp = std::chrono::steady_clock::time_point{};
    
    networkReady_.store(false);
    lastNetworkCheck_ = std::chrono::steady_clock::time_point{};
    wsAvailable_.store(false);
    
    initialized_.store(false);
}

std::vector<AntennaInfo> AntennaSwitch::parseAntennaStatusFast(const std::string& jsonResponse, uint64_t frequencyHz) {
    std::vector<AntennaInfo> antennas;
    
    if (jsonResponse.empty()) {
        ESP_LOGW(TAG, "Empty JSON response");
        return antennas;
    }
    
    // Use cJSON for efficient parsing
    cJSON* json = cJSON_Parse(jsonResponse.c_str());
    if (!json) {
        ESP_LOGW(TAG, "Failed to parse JSON response");
        // Fallback to manual parsing
        return parseAntennaStatus(jsonResponse, frequencyHz);
    }
    
    // Parse current antenna name
    std::string currentAntennaName;
    cJSON* antennaItem = cJSON_GetObjectItem(json, "antenna");
    if (cJSON_IsString(antennaItem)) {
        currentAntennaName = antennaItem->valuestring;
        ESP_LOGD(TAG, "Current antenna (fast): %s", currentAntennaName.c_str());
    }
    
    // Parse available_antennas array
    cJSON* availableArray = cJSON_GetObjectItem(json, "available_antennas");
    if (cJSON_IsArray(availableArray)) {
        int arraySize = cJSON_GetArraySize(availableArray);
        antennas.reserve(arraySize);  // Pre-allocate for performance
        
        for (int i = 0; i < arraySize; i++) {
            cJSON* item = cJSON_GetArrayItem(availableArray, i);
            if (cJSON_IsNumber(item)) {
                AntennaInfo antenna;
                antenna.id = item->valueint;
                antenna.name = "Antenna " + std::to_string(antenna.id);
                antenna.isActive = (antenna.name == currentAntennaName);
                
                // All available antennas are compatible with current frequency/band
                std::string currentBand = getBandFromFrequency(frequencyHz);
                antenna.compatibleBands = {currentBand};
                
                antennas.push_back(std::move(antenna));  // Move for efficiency
                ESP_LOGD(TAG, "Added antenna ID %d (fast) - active: %s", 
                        antenna.id, antenna.isActive ? "yes" : "no");
            }
        }
    }
    
    cJSON_Delete(json);
    ESP_LOGI(TAG, "Fast parsed %d available antennas", antennas.size());
    return antennas;
}

bool AntennaSwitch::isCacheValid(uint64_t frequencyHz) const {
    if (cache_.timestamp == std::chrono::steady_clock::time_point{}) {
        return false;  // Never cached
    }
    // Expiration by config
    auto now = std::chrono::steady_clock::now();
    auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - cache_.timestamp).count();
    if (age >= config_.cacheExpirationMs) return false;
    
    // Check if frequency matches (same band)
    std::string requestedBand = getBandFromFrequency(frequencyHz);
    std::string cachedBand = getBandFromFrequency(cache_.frequencyHz);
    
    return requestedBand == cachedBand;
}

void AntennaSwitch::updateCache(const std::vector<AntennaInfo>& antennas, const AntennaInfo& currentAntenna, uint64_t frequencyHz) {
    cache_.antennas = antennas;
    cache_.currentAntenna = currentAntenna;
    cache_.frequencyHz = frequencyHz;
    cache_.timestamp = std::chrono::steady_clock::now();
    
    ESP_LOGD(TAG, "Cache updated for frequency %.3f MHz (%d antennas)", 
             frequencyHz / 1e6, antennas.size());
}

void AntennaSwitch::configureHttpClientOptimal() {
    if (!httpClient_) {
        return;
    }
    
    // Set optimal buffer sizes for ESP32
    esp_http_client_set_header(httpClient_, "Connection", "keep-alive");
    esp_http_client_set_header(httpClient_, "Cache-Control", "no-cache");
    
    // No compression for tiny responses
    
    ESP_LOGD(TAG, "HTTP client configured for optimal performance");
}

bool AntennaSwitch::initializeAsyncWorker() {
    if (!config_.useAsyncMode) {
        return false;
    }
    
    // Create request queue
    requestQueue_ = xQueueCreate(10, sizeof(void*));
    if (!requestQueue_) {
        ESP_LOGE(TAG, "Failed to create request queue");
        return false;
    }
    
    // Create worker task
    BaseType_t result = xTaskCreate(
        workerTaskFunction,
        "antenna_worker",
        4096,  // Stack size
        this,  // Task parameter
        5,     // Priority
        &workerTask_
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create worker task");
        vQueueDelete(requestQueue_);
        requestQueue_ = nullptr;
        return false;
    }
    
    ESP_LOGI(TAG, "Async worker initialized successfully");
    return true;
}

void AntennaSwitch::shutdownAsyncWorker() {
    if (workerTask_) {
        stopWorker_.store(true);
        
        // Send shutdown signal
        if (requestQueue_) {
            void* shutdownSignal = nullptr;
            xQueueSend(requestQueue_, &shutdownSignal, pdMS_TO_TICKS(100));
        }
        
        // Wait for task to finish
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Clean up
        if (requestQueue_) {
            vQueueDelete(requestQueue_);
            requestQueue_ = nullptr;
        }
        
        workerTask_ = nullptr;
        stopWorker_.store(false);
        ESP_LOGD(TAG, "Async worker shutdown complete");
    }
}

void AntennaSwitch::workerTaskFunction(void* pvParameters) {
    AntennaSwitch* instance = static_cast<AntennaSwitch*>(pvParameters);
    
    ESP_LOGI(instance->TAG, "Antenna async worker task started");
    
    uint32_t tick = 0;
    while (!instance->stopWorker_.load()) {
        void* request = nullptr;
        
        if (xQueueReceive(instance->requestQueue_, &request, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (request == nullptr) {
                // Shutdown signal
                break;
            }
            
            // Process async request here
            // For now, this is a placeholder for future async operations
        }
        
        // Perform periodic tasks: refresh status/cache every cacheExpirationMs (default 60s)
        vTaskDelay(pdMS_TO_TICKS(200));
        tick += 200;
        if (tick >= (uint32_t)instance->config_.cacheExpirationMs) {
            tick = 0;
            if (instance->isNetworkReadyCached() && instance->httpClient_) {
                std::string resp;
                if (instance->httpGet(instance->config_.statusEndpoint, resp)) {
                    // Parse and update cache without blocking hot path; keep last known freq/band
                    auto antennas = instance->parseAntennaStatusFast(resp, instance->cache_.frequencyHz ? instance->cache_.frequencyHz : 14000000);
                    AntennaInfo current;
                    for (const auto& a : antennas) {
                        if (a.isActive) { current = a; break; }
                    }
                    if (!antennas.empty()) {
                        instance->updateCache(antennas, current, instance->cache_.frequencyHz);
                    }
                }
            }
        }
    }
    
    ESP_LOGI(instance->TAG, "Antenna async worker task stopped");
    vTaskDelete(nullptr);
}

bool AntennaSwitch::initializeWebSocketClient() {
    ESP_LOGI(TAG, "Initializing WebSocket client (connection deferred)");
    
    wsClient_ = std::make_unique<AntennaWebSocketClient>();
    if (!wsClient_) {
        setError("Failed to create WebSocket client");
        return false;
    }
    
    WSAntennaConfig wsConfig;
    wsConfig.wsUrl = config_.wsUrl;
    wsConfig.reconnectTimeoutMs = config_.wsReconnectTimeoutMs;
    wsConfig.keepaliveIntervalMs = config_.wsKeepAliveMs;
    wsConfig.responseTimeoutMs = config_.timeoutMs;
    wsConfig.autoReconnect = true;
    wsConfig.maxReconnectAttempts = config_.maxRetries + 1;
    
    if (!wsClient_->initialize(wsConfig)) {
        setError("Failed to initialize WebSocket client: " + wsClient_->getLastError());
        wsClient_.reset();
        return false;
    }
    
    // Set up callbacks
    wsClient_->setResponseCallback([this](const WSMessage& msg) {
        handleWebSocketResponse(msg);
    });
    
    wsClient_->setEventCallback([this](const WSMessage& msg) {
        handleWebSocketEvent(msg);
    });
    
    wsClient_->setErrorCallback([this](const WSMessage& msg) {
        handleWebSocketError(msg);
    });
    
    // Don't connect yet - wait until network is ready and first use
    wsAvailable_.store(false);
    ESP_LOGI(TAG, "WebSocket client initialized, connection will be established on first use");
    
    return true;
}

bool AntennaSwitch::ensureWebSocketConnected() {
    if (!wsClient_) {
        return false;
    }
    
    if (wsClient_->isConnected()) {
        return true;
    }
    
    // Check if network is ready before attempting connection
    if (!isNetworkReadyCached()) {
        ESP_LOGD(TAG, "Network not ready for WebSocket connection");
        return false;
    }
    
    ESP_LOGI(TAG, "Attempting WebSocket connection to %s", config_.wsUrl.c_str());
    
    // Try to connect
    if (wsClient_->connect()) {
        wsAvailable_.store(true);
        ESP_LOGI(TAG, "WebSocket client connected successfully");
        
        // Subscribe to relevant events if enabled
        if (config_.enableWebSocketEvents) {
            std::vector<std::string> events = {
                "status_updates", "relay_state_changes", "frequency_changes"
            };
            wsClient_->subscribeToEvents(events);
        }
        return true;
    } else {
        ESP_LOGW(TAG, "Failed to connect WebSocket client: %s", wsClient_->getLastError().c_str());
        wsAvailable_.store(false);
        return false;
    }
}

void AntennaSwitch::handleWebSocketResponse(const WSMessage& message) {
    ESP_LOGD(TAG, "WebSocket response: action=%s, id=%s", message.action.c_str(), message.id.c_str());
    
    if (message.action == "status" && !message.data.empty()) {
        // Parse status response and update cache
        auto antennas = parseAntennaStatusFast(message.data, cache_.frequencyHz ? cache_.frequencyHz : 14000000);
        if (!antennas.empty()) {
            AntennaInfo currentAntenna;
            for (const auto& ant : antennas) {
                if (ant.isActive) {
                    currentAntenna = ant;
                    break;
                }
            }
            updateCache(antennas, currentAntenna, cache_.frequencyHz);
        }
    }
}

void AntennaSwitch::handleWebSocketEvent(const WSMessage& message) {
    ESP_LOGD(TAG, "WebSocket event: %s", message.event.c_str());
    
    if (message.event == "status_update" && !message.data.empty()) {
        // Real-time status update - refresh cache
        auto antennas = parseAntennaStatusFast(message.data, cache_.frequencyHz ? cache_.frequencyHz : 14000000);
        if (!antennas.empty()) {
            AntennaInfo currentAntenna;
            for (const auto& ant : antennas) {
                if (ant.isActive) {
                    currentAntenna = ant;
                    break;
                }
            }
            updateCache(antennas, currentAntenna, cache_.frequencyHz);
            ESP_LOGD(TAG, "Cache updated from WebSocket event");
        }
    }
}

void AntennaSwitch::handleWebSocketError(const WSMessage& message) {
    ESP_LOGW(TAG, "WebSocket error: %s", message.data.c_str());
    setError("WebSocket error: " + message.data);
    
    // Switch to HTTP transport on persistent errors
    if (activeTransport_.load() == TransportType::WebSocket) {
        ESP_LOGI(TAG, "Switching to HTTP transport due to WebSocket errors");
        activeTransport_.store(TransportType::Http);
    }
}

bool AntennaSwitch::tryWebSocketWithHttpFallback(std::function<bool()> wsOperation, std::function<bool()> httpFallback) {
    // Try to ensure WebSocket connection first
    if (!ensureWebSocketConnected()) {
        ESP_LOGD(TAG, "WebSocket not available, using HTTP");
        activeTransport_.store(TransportType::Http);
        return httpFallback();
    }
    
    // Try WebSocket operation
    if (wsOperation()) {
        return true;
    }
    
    // WebSocket failed, try HTTP fallback
    ESP_LOGW(TAG, "WebSocket operation failed, falling back to HTTP");
    activeTransport_.store(TransportType::Http);
    return httpFallback();
}

AntennaInfo AntennaSwitch::getCurrentAntennaWS() {
    AntennaInfo currentAntenna;
    
    if (!ensureWebSocketConnected()) {
        return currentAntenna;
    }
    
    // For now, WebSocket status queries are async - use cache
    // In the future, we could implement synchronous request/response
    if (cache_.timestamp != std::chrono::steady_clock::time_point{} && 
        cache_.currentAntenna.id != -1) {
        ESP_LOGD(TAG, "Returning cached current antenna from WebSocket: %s", cache_.currentAntenna.name.c_str());
        return cache_.currentAntenna;
    }
    
    // Trigger async status update
    wsClient_->sendStatusQuery();
    return currentAntenna;
}

std::vector<AntennaInfo> AntennaSwitch::getCompatibleAntennasWS(uint64_t frequencyHz) {
    std::vector<AntennaInfo> antennas;
    
    if (!ensureWebSocketConnected()) {
        return antennas;
    }
    
    // For async WebSocket, use cache if available
    if (isCacheValid(frequencyHz)) {
        ESP_LOGD(TAG, "Returning cached WebSocket antenna data for frequency %.3f MHz", frequencyHz / 1e6);
        return cache_.antennas;
    }
    
    // Trigger async status update
    wsClient_->sendStatusQuery();
    
    // Return empty for now - data will be available in cache after async response
    return antennas;
}

bool AntennaSwitch::switchToNextAntennaWS(uint64_t frequencyHz) {
    if (!ensureWebSocketConnected()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Switching to next antenna via WebSocket for frequency %.3f MHz", frequencyHz / 1e6);
    
    bool result = wsClient_->sendAntennaSwitch("A", "next");
    if (result) {
        ESP_LOGI(TAG, "Successfully sent WebSocket antenna switch command");
    }
    
    return result;
}

} // namespace antenna
