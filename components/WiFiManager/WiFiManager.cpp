#include "WiFiManager.h"
#include <cstring>
#include "esp_mac.h"
#include "esp_netif_ip_addr.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

namespace wifi {

const char* WiFiManager::TAG = "WiFiManager";

WiFiManager::WiFiManager()
    : wifiEventGroup_(nullptr)
    , netifSta_(nullptr)
    , status_(WiFiStatus::DISCONNECTED)
    , initialized_(false)
    , retryCount_(0) {
    ESP_LOGD(TAG, "WiFiManager created");
}

WiFiManager::~WiFiManager() {
    cleanup();
}

bool WiFiManager::initialize() {
    if (initialized_) {
        ESP_LOGW(TAG, "WiFi already initialized");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing WiFi subsystem");
    
    // Initialize NVS if needed
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create event group for WiFi events
    wifiEventGroup_ = xEventGroupCreate();
    if (!wifiEventGroup_) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return false;
    }
    
    // Initialize TCP/IP adapter
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create WiFi station netif
    netifSta_ = esp_netif_create_default_wifi_sta();
    if (!netifSta_) {
        ESP_LOGE(TAG, "Failed to create WiFi station netif");
        cleanup();
        return false;
    }

    // Set custom hostname if configured
#ifdef CONFIG_WIFI_HOSTNAME
    if (strlen(CONFIG_WIFI_HOSTNAME) > 0) {
        esp_err_t hostnameErr = esp_netif_set_hostname(netifSta_, CONFIG_WIFI_HOSTNAME);
        if (hostnameErr == ESP_OK) {
            ESP_LOGI(TAG, "Hostname set to: %s", CONFIG_WIFI_HOSTNAME);
        } else {
            ESP_LOGW(TAG, "Failed to set hostname: %s", esp_err_to_name(hostnameErr));
        }
    }
#endif

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                               &WiFiManager::wifiEventHandler, this));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                                               &WiFiManager::ipEventHandler, this));
    
    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    initialized_ = true;
    ESP_LOGI(TAG, "WiFi subsystem initialized successfully");
    return true;
}

bool WiFiManager::connect() {
    if (!initialized_) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return false;
    }
    
    // Check if SSID is configured
#ifndef CONFIG_WIFI_SSID
    ESP_LOGE(TAG, "WiFi SSID not configured - check KConfig settings");
    return false;
#endif
    
    if (strlen(CONFIG_WIFI_SSID) == 0) {
        ESP_LOGE(TAG, "WiFi SSID is empty - configure in KConfig");
        return false;
    }
    
    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", CONFIG_WIFI_SSID);
    
    // Configure WiFi
    wifi_config_t wifiConfig = {};
    strncpy((char*)wifiConfig.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifiConfig.sta.ssid) - 1);
    
#ifdef CONFIG_WIFI_PASSWORD
    strncpy((char*)wifiConfig.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifiConfig.sta.password) - 1);
#endif
    
    // Set threshold authmode - use WPA_WPA2_PSK for broader compatibility
    // This allows connection to both WPA and WPA2 networks
    wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifiConfig.sta.pmf_cfg.capable = true;
    wifiConfig.sta.pmf_cfg.required = false;

    // Enable scan for specific SSID (faster connection)
    wifiConfig.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifiConfig.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));
    
    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Connect to AP
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed: %s", esp_err_to_name(err));
        return false;
    }
    
    status_.store(WiFiStatus::CONNECTING);
    return true;
}

void WiFiManager::disconnect() {
    if (!initialized_) {
        return;
    }
    
    ESP_LOGI(TAG, "Disconnecting from WiFi");
    esp_wifi_disconnect();
    status_.store(WiFiStatus::DISCONNECTED);
}

WiFiStatus WiFiManager::getStatus() const {
    return status_.load();
}

bool WiFiManager::isConnected() const {
    return status_.load() == WiFiStatus::CONNECTED;
}

std::string WiFiManager::getIpAddress() const {
    if (!netifSta_ || !isConnected()) {
        return "";
    }
    
    esp_netif_ip_info_t ipInfo;
    if (esp_netif_get_ip_info(netifSta_, &ipInfo) != ESP_OK) {
        return "";
    }
    
    char ipStr[16];
    snprintf(ipStr, sizeof(ipStr), IPSTR, IP2STR(&ipInfo.ip));
    return std::string(ipStr);
}

bool WiFiManager::waitForConnection(uint32_t timeoutMs) {
    if (!wifiEventGroup_) {
        return false;
    }

    ESP_LOGI(TAG, "Waiting for WiFi connection (timeout: %lu ms)", timeoutMs);

    EventBits_t bits = xEventGroupWaitBits(wifiEventGroup_,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(timeoutMs));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi successfully");
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        return false;
    } else {
        ESP_LOGW(TAG, "WiFi connection timeout");
        return false;
    }
}

int WiFiManager::scanNetworks() {
    if (!initialized_) {
        ESP_LOGE(TAG, "WiFi not initialized - call initialize() first");
        return -1;
    }

    ESP_LOGI(TAG, "Starting WiFi scan...");

    // Need to start WiFi before scanning
    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "Failed to start WiFi for scan: %s", esp_err_to_name(err));
        return -1;
    }

    // Configure scan - all channels, active scan
    wifi_scan_config_t scanConfig = {};
    scanConfig.ssid = nullptr;        // Scan all SSIDs
    scanConfig.bssid = nullptr;       // Scan all BSSIDs
    scanConfig.channel = 0;           // Scan all channels
    scanConfig.show_hidden = true;    // Include hidden networks
    scanConfig.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scanConfig.scan_time.active.min = 100;
    scanConfig.scan_time.active.max = 300;

    // Start blocking scan
    err = esp_wifi_scan_start(&scanConfig, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi scan failed: %s", esp_err_to_name(err));
        return -1;
    }

    // Get scan results
    uint16_t apCount = 0;
    esp_wifi_scan_get_ap_num(&apCount);

    if (apCount == 0) {
        ESP_LOGW(TAG, "No WiFi networks found - check antenna connection");
        return 0;
    }

    ESP_LOGI(TAG, "Found %d WiFi networks:", apCount);

    // Allocate memory for AP records
    wifi_ap_record_t* apRecords = new (std::nothrow) wifi_ap_record_t[apCount];
    if (!apRecords) {
        ESP_LOGE(TAG, "Failed to allocate memory for scan results");
        return -1;
    }

    err = esp_wifi_scan_get_ap_records(&apCount, apRecords);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get AP records: %s", esp_err_to_name(err));
        delete[] apRecords;
        return -1;
    }

    // Log each network with signal strength interpretation
    for (uint16_t i = 0; i < apCount; i++) {
        const char* signalQuality;
        if (apRecords[i].rssi >= -50) {
            signalQuality = "Excellent";
        } else if (apRecords[i].rssi >= -60) {
            signalQuality = "Good";
        } else if (apRecords[i].rssi >= -70) {
            signalQuality = "Fair";
        } else if (apRecords[i].rssi >= -80) {
            signalQuality = "Weak";
        } else {
            signalQuality = "Very Weak";
        }

        const char* authMode;
        switch (apRecords[i].authmode) {
            case WIFI_AUTH_OPEN:            authMode = "Open"; break;
            case WIFI_AUTH_WEP:             authMode = "WEP"; break;
            case WIFI_AUTH_WPA_PSK:         authMode = "WPA"; break;
            case WIFI_AUTH_WPA2_PSK:        authMode = "WPA2"; break;
            case WIFI_AUTH_WPA_WPA2_PSK:    authMode = "WPA/WPA2"; break;
            case WIFI_AUTH_WPA3_PSK:        authMode = "WPA3"; break;
            case WIFI_AUTH_WPA2_WPA3_PSK:   authMode = "WPA2/WPA3"; break;
            default:                        authMode = "Other"; break;
        }

        ESP_LOGI(TAG, "  [%2d] %-32s  CH:%2d  RSSI:%4d dBm (%s)  Auth:%s",
                 i + 1,
                 (const char*)apRecords[i].ssid,
                 apRecords[i].primary,
                 apRecords[i].rssi,
                 signalQuality,
                 authMode);
    }

    // Check if target SSID is in list
#ifdef CONFIG_WIFI_SSID
    bool targetFound = false;
    for (uint16_t i = 0; i < apCount; i++) {
        if (strcmp((const char*)apRecords[i].ssid, CONFIG_WIFI_SSID) == 0) {
            targetFound = true;
            ESP_LOGI(TAG, "Target SSID '%s' found with RSSI %d dBm",
                     CONFIG_WIFI_SSID, apRecords[i].rssi);
            break;
        }
    }
    if (!targetFound) {
        ESP_LOGW(TAG, "Target SSID '%s' NOT found in scan results!", CONFIG_WIFI_SSID);
        ESP_LOGW(TAG, "Check: 1) SSID spelling  2) 2.4GHz vs 5GHz  3) Router distance");
    }
#endif

    delete[] apRecords;
    return static_cast<int>(apCount);
}

void WiFiManager::wifiEventHandler(void* arg, esp_event_base_t eventBase,
                                   int32_t eventId, void* eventData) {
    WiFiManager* manager = static_cast<WiFiManager*>(arg);
    manager->handleWifiEvent(eventBase, eventId, eventData);
}

void WiFiManager::ipEventHandler(void* arg, esp_event_base_t eventBase,
                                 int32_t eventId, void* eventData) {
    WiFiManager* manager = static_cast<WiFiManager*>(arg);
    manager->handleWifiEvent(eventBase, eventId, eventData);
}

void WiFiManager::handleWifiEvent(esp_event_base_t eventBase, int32_t eventId, void* eventData) {
    if (eventBase == WIFI_EVENT) {
        switch (eventId) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi station started");
                break;
                
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WiFi station connected");
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* disconnected =
                    static_cast<wifi_event_sta_disconnected_t*>(eventData);
                ESP_LOGW(TAG, "WiFi disconnected (reason: %d)", disconnected->reason);
                status_.store(WiFiStatus::DISCONNECTED);

#ifdef CONFIG_WIFI_MAXIMUM_RETRY
                // Retry logic using configured max retry count
                if (retryCount_ < CONFIG_WIFI_MAXIMUM_RETRY) {
                    retryCount_++;
                    ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d)...",
                             retryCount_, CONFIG_WIFI_MAXIMUM_RETRY);
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(TAG, "Max retries (%d) exhausted - giving up",
                             CONFIG_WIFI_MAXIMUM_RETRY);
                    xEventGroupSetBits(wifiEventGroup_, WIFI_FAIL_BIT);
                }
#else
                xEventGroupSetBits(wifiEventGroup_, WIFI_FAIL_BIT);
#endif
                break;
            }
                
            default:
                break;
        }
    } else if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = static_cast<ip_event_got_ip_t*>(eventData);
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        retryCount_ = 0;  // Reset retry counter on successful connection
        status_.store(WiFiStatus::CONNECTED);
        xEventGroupSetBits(wifiEventGroup_, WIFI_CONNECTED_BIT);

        // Initialize SNTP for UTC time sync (only once)
        if (!sntpInitialized_) {
            ESP_LOGI(TAG, "Initializing SNTP time sync");
            esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "pool.ntp.org");
            esp_sntp_init();
            sntpInitialized_ = true;
        }
    }
}

void WiFiManager::cleanup() {
    if (initialized_) {
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &WiFiManager::ipEventHandler);
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &WiFiManager::wifiEventHandler);
        
        esp_wifi_stop();
        esp_wifi_deinit();
        initialized_ = false;
    }
    
    if (netifSta_) {
        esp_netif_destroy_default_wifi(netifSta_);
        netifSta_ = nullptr;
    }
    
    if (wifiEventGroup_) {
        vEventGroupDelete(wifiEventGroup_);
        wifiEventGroup_ = nullptr;
    }
    
    status_.store(WiFiStatus::DISCONNECTED);
}

} // namespace wifi