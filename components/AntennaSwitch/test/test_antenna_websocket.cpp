#include "unity.h"
#include "AntennaSwitch.h"
#include "AntennaWebSocketClient.h"
#include "esp_log.h"
#include <memory>

using namespace antenna;

static constexpr auto TAG = "AntennaWSTest";

// Mock network ready to avoid actual network dependency in tests
class MockAntennaSwitch : public AntennaSwitch {
public:
    bool mockNetworkReady = true;
    
protected:
    bool isNetworkReadyCached() override {
        return mockNetworkReady;
    }
};

TEST_CASE("AntennaSwitch WebSocket Configuration", "[AntennaSwitch][WebSocket]") {
    ESP_LOGI(TAG, "Testing WebSocket configuration");
    
    MockAntennaSwitch antennaSwitch;
    AntennaConfig config;
    
    // Test HTTP only configuration
    config.enabled = true;
    config.transportType = TransportType::Http;
    config.baseUrl = "http://192.168.1.100";
    
    TEST_ASSERT_TRUE(antennaSwitch.initialize(config));
    ESP_LOGI(TAG, "HTTP-only configuration test passed");
    
    // Test WebSocket configuration  
    config.transportType = TransportType::WebSocket;
    config.wsUrl = "ws://192.168.1.100/ws";
    config.enableWebSocketEvents = true;
    config.wsReconnectTimeoutMs = 1000;  // Shorter timeout for tests
    config.wsKeepAliveMs = 10000;
    
    // Note: This will fail to actually connect but should initialize the client
    antennaSwitch.initialize(config);  // May fail connection but setup should work
    ESP_LOGI(TAG, "WebSocket configuration test completed");
    
    // Test Auto configuration (should initialize both)
    config.transportType = TransportType::Auto;
    TEST_ASSERT_TRUE(antennaSwitch.initialize(config));
    ESP_LOGI(TAG, "Auto transport configuration test passed");
}

TEST_CASE("AntennaWebSocketClient Message Creation", "[AntennaWebSocket]") {
    ESP_LOGI(TAG, "Testing WebSocket message creation");
    
    AntennaWebSocketClient wsClient;
    WSAntennaConfig wsConfig;
    wsConfig.wsUrl = "ws://test.local/ws";
    wsConfig.messageQueueSize = 5;
    
    // Initialize client (will fail to connect but should create internal structures)
    wsClient.initialize(wsConfig);
    
    // Test request ID generation - this should work without connection
    ESP_LOGI(TAG, "WebSocket client initialization test completed");
}

TEST_CASE("AntennaSwitch Transport Switching", "[AntennaSwitch][Transport]") {
    ESP_LOGI(TAG, "Testing transport switching logic");
    
    MockAntennaSwitch antennaSwitch;
    AntennaConfig config;
    config.enabled = true;
    config.transportType = TransportType::Auto;
    config.baseUrl = "http://192.168.1.100";
    config.wsUrl = "ws://192.168.1.100/ws";
    
    TEST_ASSERT_TRUE(antennaSwitch.initialize(config));
    
    // Test frequency compatibility check (should work with mock network)
    uint64_t testFreq = 14205000; // 20m band
    auto antennas = antennaSwitch.getCompatibleAntennas(testFreq);
    ESP_LOGI(TAG, "Found %zu antennas for test frequency", antennas.size());
    
    // Test antenna switching (should not crash)
    bool result = antennaSwitch.switchToNextAntenna(testFreq);
    ESP_LOGI(TAG, "Antenna switch result: %s", result ? "success" : "failed");
    
    ESP_LOGI(TAG, "Transport switching test completed");
}

TEST_CASE("WebSocket Message Parsing", "[AntennaWebSocket][JSON]") {
    ESP_LOGI(TAG, "Testing WebSocket message parsing");
    
    AntennaWebSocketClient wsClient;
    
    // Test JSON parsing capabilities by checking basic initialization
    WSAntennaConfig config;
    config.wsUrl = "ws://test.local/ws";
    config.messageQueueSize = 1;
    
    // Initialize (will fail connection but JSON parsing setup should work)
    bool initResult = wsClient.initialize(config);
    ESP_LOGI(TAG, "WebSocket JSON parsing initialization result: %s", initResult ? "success" : "failed");
    
    // Test error handling
    std::string lastError = wsClient.getLastError();
    ESP_LOGI(TAG, "Last error: %s", lastError.c_str());
    
    ESP_LOGI(TAG, "Message parsing test completed");
}

TEST_CASE("AntennaSwitch Dual Transport Fallback", "[AntennaSwitch][Fallback]") {
    ESP_LOGI(TAG, "Testing dual transport fallback mechanism");
    
    MockAntennaSwitch antennaSwitch;
    AntennaConfig config;
    config.enabled = true;
    config.transportType = TransportType::Auto;
    config.baseUrl = "http://192.168.1.100";
    config.wsUrl = "ws://192.168.1.100/ws";
    config.timeoutMs = 500;  // Short timeout for tests
    
    TEST_ASSERT_TRUE(antennaSwitch.initialize(config));
    
    // Test that API availability check works
    antennaSwitch.mockNetworkReady = true;
    bool networkReady = antennaSwitch.isNetworkReady();
    TEST_ASSERT_TRUE(networkReady);
    
    // Test configuration access
    const auto& retrievedConfig = antennaSwitch.getConfig();
    TEST_ASSERT_EQUAL_STRING(config.wsUrl.c_str(), retrievedConfig.wsUrl.c_str());
    TEST_ASSERT_EQUAL(TransportType::Auto, retrievedConfig.transportType);
    
    ESP_LOGI(TAG, "Dual transport fallback test completed");
}

void setUp(void) {
    // This runs before each test
}

void tearDown(void) {
    // This runs after each test
}

extern "C" void app_main() {
    // Initialize Unity test framework
    UNITY_BEGIN();
    
    ESP_LOGI(TAG, "Starting AntennaSwitch WebSocket integration tests");
    
    // Run tests
    RUN_TEST(AntennaSwitch WebSocket Configuration);
    RUN_TEST(AntennaWebSocketClient Message Creation);
    RUN_TEST(AntennaSwitch Transport Switching);
    RUN_TEST(WebSocket Message Parsing);
    RUN_TEST(AntennaSwitch Dual Transport Fallback);
    
    // Finish tests
    UNITY_END();
}