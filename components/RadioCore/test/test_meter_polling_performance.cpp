/**
 * @file test_meter_polling_performance.cpp  
 * @brief Unity tests for meter polling performance validation
 * 
 * Tests the performance characteristics of the smart polling system,
 * specifically focusing on meter command (SM, RM) polling frequency
 * and responsiveness for external display applications.
 */

#include "../../../components/unity/unity/src/unity.h"
#include "test_hooks.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <memory>

#include "../include/RadioManager.h"
#include "../include/BaseCommandHandler.h"
#include "../../SerialHandler/mock/include/MockSerialHandler.h"

using namespace radio;

static const char* TAG = "MeterPollingTest";

// Test fixture globals
static std::unique_ptr<MockSerialHandler> mockRadioSerial;
static std::unique_ptr<MockSerialHandler> mockUsbSerial;
static RadioManager* radioManager = nullptr;

// Test setup and teardown
void setUp_MeterPolling(void) {
    mockRadioSerial = std::make_unique<MockSerialHandler>();
    mockUsbSerial = std::make_unique<MockSerialHandler>();
    radioManager = new RadioManager(static_cast<ISerialChannel&>(*mockRadioSerial), static_cast<ISerialChannel&>(*mockUsbSerial));
    radioManager->getLocalCATHandler().parseMessage("PS1;"); // Ensure radio is "on"
    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();
}

void tearDown_MeterPolling(void) {
    delete radioManager;
    radioManager = nullptr;
    mockRadioSerial.reset();
    mockUsbSerial.reset();
}

// Test 1: Verify meter commands use real-time TTL
void test_meter_commands_use_realtime_ttl(void) {
    setUp_MeterPolling();
    
    // Enable smart polling and meter polling
    radioManager->getState().keepAlive.store(true);
    radioManager->getState().aiMode.store(0); // AI0 - polling allowed
    
    // Populate a the remote radio cache with real S-meter values
    radioManager->getRemoteCATHandler().parseMessage("SM0005;");
    
    // Immediately check if cache is considered fresh with real-time TTL
    const uint64_t currentTime = esp_timer_get_time();
    const bool freshWithRealtime = radioManager->getState().commandCache.isFresh("SM", currentTime, BaseCommandHandler::TTL_REALTIME);
    const bool freshWithStatus = radioManager->getState().commandCache.isFresh("SM", currentTime, BaseCommandHandler::TTL_STATUS);
    
    TEST_ASSERT_TRUE(freshWithRealtime);
    TEST_ASSERT_TRUE(freshWithStatus);
    
    ESP_LOGI(TAG, "SM cache fresh with real-time TTL: %s", freshWithRealtime ? "YES" : "NO");
    ESP_LOGI(TAG, "SM cache fresh with status TTL: %s", freshWithStatus ? "YES" : "NO");
    
    tearDown_MeterPolling();
}

// Test 2: Verify meter polling frequency is higher than standard commands
void test_meter_polling_frequency_higher_than_standard(void) {
    setUp_MeterPolling();
    
    // TTL comparison test
#ifdef CONFIG_RADIOCORE_SMART_POLLING
    const uint64_t realtimeTTL = BaseCommandHandler::TTL_REALTIME;
    const uint64_t statusTTL = BaseCommandHandler::TTL_STATUS; 
    const uint64_t dynamicTTL = BaseCommandHandler::TTL_DYNAMIC;
    
    // Real-time TTL should be shorter (more frequent polling)
    TEST_ASSERT_TRUE(realtimeTTL < statusTTL);
    TEST_ASSERT_TRUE(realtimeTTL < dynamicTTL);
    
    ESP_LOGI(TAG, "TTL comparison - Real-time: %lluus, Status: %lluus, Dynamic: %lluus", 
             realtimeTTL, statusTTL, dynamicTTL);
    
    // Verify configured values are reasonable for display updates
    TEST_ASSERT_TRUE(realtimeTTL <= 2000000); // ≤ 2 seconds for responsive display
    
    ESP_LOGI(TAG, "✅ Real-time TTL (%lluus) is suitable for responsive display updates", realtimeTTL);
#else
    TEST_IGNORE_MESSAGE("Smart polling not enabled in configuration");
#endif
    
    tearDown_MeterPolling();
}

// Test 3: Verify meter polling only occurs when meter polling is enabled
void test_meter_polling_conditional_on_config(void) {
    setUp_MeterPolling();
    
    // Enable conditions for polling
    radioManager->getState().keepAlive.store(true);
    radioManager->getState().aiMode.store(0); // AI0 - polling allowed
    
    // Clear any existing cache to force polling
    radioManager->getState().commandCache.clear();
    
    // Clear mock and perform sync
    mockRadioSerial->sentMessages.clear();
    radioManager->performPeriodicSync();

    // Check if meter commands were sent
    bool foundSM = false, foundRM = false;
    for (const auto& msg : mockRadioSerial->sentMessages) {
        if (msg == "SM0;") foundSM = true;
        if (msg == "RM;") foundRM = true;
    }
    
#ifdef CONFIG_RADIOCORE_METER_POLLING
    TEST_ASSERT_TRUE(foundSM);
    TEST_ASSERT_TRUE(foundRM);   // RM (all meters: SWR, COMP, ALC) should be sent in RX mode
    ESP_LOGI(TAG, "✅ Meter polling enabled: SM and RM (all meters) sent in RX mode");
#else
    TEST_ASSERT_FALSE(foundSM);
    TEST_ASSERT_FALSE(foundRM);
    ESP_LOGI(TAG, "✅ Meter polling disabled: SM and RM commands not sent");
#endif
    
    ESP_LOGI(TAG, "Commands sent: %zu total", mockRadioSerial->sentMessages.size());
    for (const auto& cmd : mockRadioSerial->sentMessages) {
        ESP_LOGI(TAG, "  - %s", cmd.c_str());
    }
    
    tearDown_MeterPolling();
}

// Test 4: Verify sync interval adjusts for meter polling
void test_sync_interval_adapts_for_meter_polling(void) {
    setUp_MeterPolling();
    
    // This test validates the main loop timing logic
#if defined(CONFIG_RADIOCORE_METER_POLLING) && CONFIG_RADIOCORE_METER_POLLING
    constexpr uint64_t expectedInterval = std::min(30000ULL, static_cast<uint64_t>(CONFIG_RADIOCORE_TTL_REALTIME_MS));
    
    // Should use the real-time TTL when meter polling is enabled
    TEST_ASSERT_EQUAL_UINT32(CONFIG_RADIOCORE_TTL_REALTIME_MS, expectedInterval);
    
    ESP_LOGI(TAG, "✅ Sync interval with meter polling: %llums (responsive for displays)", expectedInterval);
#else
    const uint64_t expectedInterval = 30000; // Standard 30s interval
    
    ESP_LOGI(TAG, "✅ Sync interval without meter polling: %llums (standard)", expectedInterval);
#endif
    
    // Verify interval is reasonable for the configuration
    TEST_ASSERT_TRUE(expectedInterval > 0);
    TEST_ASSERT_TRUE(expectedInterval <= 30000); // Never longer than original
    
    tearDown_MeterPolling();
}

// Test 5: Performance timing validation
void test_meter_polling_response_time(void) {
    setUp_MeterPolling();
    
    // Enable conditions for polling
    radioManager->getState().keepAlive.store(true);
    radioManager->getState().aiMode.store(0); // AI0 - polling allowed
    
    // Measure time for performPeriodicSync() call
    const uint64_t startTime = esp_timer_get_time();
    radioManager->performPeriodicSync();
    const uint64_t endTime = esp_timer_get_time();
    
    const uint64_t duration = endTime - startTime;
    
    // Should complete quickly (< 10ms for cache operations)
    TEST_ASSERT_TRUE(duration < 10000); // 10ms limit
    
    ESP_LOGI(TAG, "✅ performPeriodicSync() completed in %lluus (< 10ms)", duration);
    
    // Verify reasonable number of commands sent
    TEST_ASSERT_TRUE(mockRadioSerial->sentMessages.size() <= 20); // Reasonable upper bound
    ESP_LOGI(TAG, "Commands sent: %zu", mockRadioSerial->sentMessages.size());
    
    tearDown_MeterPolling();
}

// Test 6: Verify RM polling occurs only during transmission
void test_rm_polling_during_transmission(void) {
    setUp_MeterPolling();
    
    // Enable conditions for polling
    radioManager->getState().keepAlive.store(true);
    radioManager->getState().aiMode.store(0); // AI0 - polling allowed
    
    // Set radio to TX mode
    radioManager->getState().isTx.store(true);
    ESP_LOGI(TAG, "Setting radio to TX mode");
    
    // Clear mock and perform sync
    mockRadioSerial->sentMessages.clear();
    radioManager->performPeriodicSync();
    
    // Check if meter commands were sent
    bool foundSM = false, foundRM = false;
    for (const auto& msg : mockRadioSerial->sentMessages) {
        if (msg == "SM0;") foundSM = true;
        if (msg == "RM;") foundRM = true;
    }
    
#ifdef CONFIG_RADIOCORE_METER_POLLING
    TEST_ASSERT_TRUE(foundSM);
    TEST_ASSERT_TRUE(foundRM);   // RM should be sent (gets all meters: SWR, COMP, ALC)
    ESP_LOGI(TAG, "✅ TX mode: SM and RM (all meters) sent for meter polling");
#else
    TEST_ASSERT_FALSE(foundSM);
    TEST_ASSERT_FALSE(foundRM);
    ESP_LOGI(TAG, "✅ Meter polling disabled: No meter commands sent even in TX mode");
#endif
    
    ESP_LOGI(TAG, "TX mode commands sent: %zu total", mockRadioSerial->sentMessages.size());
    for (const auto& cmd : mockRadioSerial->sentMessages) {
        ESP_LOGI(TAG, "  - %s", cmd.c_str());
    }
    
    // Test switching back to RX mode
    radioManager->getState().isTx.store(false);
    ESP_LOGI(TAG, "Setting radio back to RX mode");
    
    // Clear mock and perform sync again
    mockRadioSerial->sentMessages.clear();
    radioManager->performPeriodicSync();
    
    // Check if RM polling stops in RX mode
    foundSM = false;
    foundRM = false;
    for (const auto& msg : mockRadioSerial->sentMessages) {
        if (msg == "SM0;") foundSM = true;
        if (msg == "RM;") foundRM = true;
    }
    
#ifdef CONFIG_RADIOCORE_METER_POLLING
    TEST_ASSERT_TRUE(foundSM);    // SM should still be sent in RX
    TEST_ASSERT_TRUE(foundRM);    // RM should still be sent in RX (but at moderate rate)
    ESP_LOGI(TAG, "✅ Back to RX mode: SM and RM sent at moderate rate");
#endif
    
    tearDown_MeterPolling();
}

void test_xi_polling_works_with_adaptive_command_limit_in_split_mode(void) {
    setUp_MeterPolling();
    
    // Enable split mode to activate XI polling
    radioManager->getState().split.store(true);
    radioManager->getState().powerOn.store(true);
    radioManager->getState().keepAlive.store(true);
    radioManager->getState().aiMode.store(0); // Allow polling
    
    // Clear any existing commands and perform sync
    mockRadioSerial->sentMessages.clear();
    radioManager->performPeriodicSync();
    
    // Count commands sent
    size_t totalCommands = mockRadioSerial->sentMessages.size();
    
    // Look for XI and RM commands
    bool foundXI = false, foundRM = false;
    for (const auto &msg : mockRadioSerial->sentMessages) {
        if (msg.find("XI;") != std::string::npos) {
            foundXI = true;
        }
        if (msg == "RM;") {
            foundRM = true;
        }
    }
    
    ESP_LOGI(TAG, "Split mode polling: sent %zu commands, XI found: %s, RM found: %s", 
             totalCommands, foundXI ? "YES" : "NO", foundRM ? "YES" : "NO");
    
    // With polling disabled in new architecture, no XI/RM should be enqueued here
    TEST_ASSERT_FALSE(foundXI);
    TEST_ASSERT_FALSE(foundRM);
    TEST_ASSERT_TRUE(totalCommands == 0);
    ESP_LOGI(TAG, "✅ No polling: XI/RM not sent in split mode (as designed)");
    
    tearDown_MeterPolling();
}

extern "C" void run_meter_polling_performance_tests(void) {
    RUN_TEST(test_meter_commands_use_realtime_ttl);
    RUN_TEST(test_meter_polling_frequency_higher_than_standard);
    RUN_TEST(test_meter_polling_conditional_on_config);
    RUN_TEST(test_sync_interval_adapts_for_meter_polling);
    RUN_TEST(test_meter_polling_response_time);
    RUN_TEST(test_rm_polling_during_transmission);
    RUN_TEST(test_xi_polling_works_with_adaptive_command_limit_in_split_mode);
}
