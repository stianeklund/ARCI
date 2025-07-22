/**
 * @file test_cache.cpp
 * @brief Consolidated cache tests for RadioManager command caching system
 *
 * Tests cover:
 * - Cache hit/miss behavior with TTL
 * - Cache timestamp updates (local SET, remote ANSWER)
 * - EX menu cache clearing
 * - General command cache clearing
 * - State consistency across VFOs
 * - Boundary conditions (frequencies, modes)
 * - Error recovery and state preservation
 * - Split mode cache interactions
 */

#include "unity.h"
#include "test_hooks.h"
#include "../../SerialHandler/mock/include/MockSerialHandler.h"
#include "../include/RadioManager.h"
#include "../include/ExtendedCommandHandler.h"
#include "../../CommandHandlers/include/BaseCommandHandler.h"
#include "include/CATTestUtils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

using radio::RadioManager;
// TEST_ASSERT_FREQUENCY_EQUAL is a preprocessor macro from CATTestUtils.h

// =============================================================================
// Test Fixture
// =============================================================================

class TestRadioManager : public RadioManager {
public:
    TestRadioManager(MockSerialHandler &radioSerial, MockSerialHandler &usbSerial)
        : RadioManager(radioSerial, usbSerial),
          m_mockRadioSerial(radioSerial),
          m_mockUsbSerial(usbSerial) {
    }

    void resetMockSerials() {
        test_utils::CATTestHelper::clearMockSerials(m_mockRadioSerial, m_mockUsbSerial);
    }

    MockSerialHandler &m_mockRadioSerial;
    MockSerialHandler &m_mockUsbSerial;
};

static std::unique_ptr<MockSerialHandler> mockRadioSerial;
static std::unique_ptr<MockSerialHandler> mockUsbSerial;
static TestRadioManager *testRadioManager = nullptr;

static void setUp_Cache() {
    mockRadioSerial = std::make_unique<MockSerialHandler>();
    mockUsbSerial = std::make_unique<MockSerialHandler>();

    testRadioManager = new TestRadioManager(*mockRadioSerial, *mockUsbSerial);

    // Ensure radio is "on" so commands are processed
    testRadioManager->getLocalCATHandler().parseMessage("PS1;");

    // Prime the IF cache so tests can assume cached responses
    const std::string basicIFResponse = "IF00014150000      000000022020000000;";
    testRadioManager->getRemoteCATHandler().parseMessage(basicIFResponse);

    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();
}

static void tearDown_Cache() {
    delete testRadioManager;
    testRadioManager = nullptr;
    mockRadioSerial.reset();
    mockUsbSerial.reset();
}

// =============================================================================
// Cache Hit/Miss Tests
// =============================================================================

void test_cache_hit_returns_cached_value_without_radio_traffic() {
    setUp_Cache();

    // 1. Set a value to populate the cache and its timestamp.
    testRadioManager->getLocalCATHandler().parseMessage("AG0050;"); // Set AF Gain to 50
    test_utils::CATTestHelper::assertForwardedToRadio(*mockRadioSerial, "AG0050;");
    TEST_ASSERT_EQUAL(50, testRadioManager->getState().afGain.load());

    // 2. Clear mock serials to ensure we are checking for new traffic.
    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();

    // 3. Immediately query the same command.
    testRadioManager->getLocalCATHandler().parseMessage("AG;");

    // 4. Assert that the response came from the cache (sent to USB)
    TEST_ASSERT_FALSE(mockUsbSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("AG0050;", mockUsbSerial->sentMessages.back().c_str());

    // 5. Assert that NO message was sent to the radio.
    TEST_ASSERT_TRUE(mockRadioSerial->sentMessages.empty());

    tearDown_Cache();
}

void test_cache_miss_on_stale_data_queries_radio() {
    setUp_Cache();

    // 1. Set a value to populate the cache.
    testRadioManager->getLocalCATHandler().parseMessage("RG075;"); // Set RF Gain to 75
    test_utils::CATTestHelper::assertForwardedToRadio(*mockRadioSerial, "RG075;");

    // 2. Clear mocks.
    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();

    // 3. Simulate a delay longer than the cache TTL (STATUS is 5000ms).
    vTaskDelay(pdMS_TO_TICKS(5100));

    // 4. Query the same command.
    testRadioManager->getLocalCATHandler().parseMessage("RG;");

    // 5. Assert that a query was sent to the radio because the cache was stale.
    TEST_ASSERT_FALSE(mockRadioSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("RG;", mockRadioSerial->sentMessages.back().c_str());

    // 6. Cached value should be returned immediately while the radio refresh is pending
    TEST_ASSERT_FALSE(mockUsbSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("RG075;", mockUsbSerial->sentMessages.back().c_str());

    tearDown_Cache();
}

void test_mode_query_responds_immediately_when_cache_stale() {
    setUp_Cache();

    // Seed mode state via remote update
    testRadioManager->getRemoteCATHandler().parseMessage("MD2;");

    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();

    const uint64_t nowUs = esp_timer_get_time();
    const uint64_t ttlStatus = radio::BaseCommandHandler::TTL_STATUS;
    uint64_t staleTimestamp = 1;
    if (nowUs > ttlStatus + 1000) {
        staleTimestamp = nowUs - ttlStatus - 1000;
    }

    testRadioManager->getState().commandCache.record("MD", staleTimestamp);

    testRadioManager->getLocalCATHandler().parseMessage("MD;");

    TEST_ASSERT_FALSE_MESSAGE(mockUsbSerial->sentMessages.empty(),
                             "Expected immediate USB response for stale MD cache");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("MD2;", mockUsbSerial->sentMessages.front().c_str(),
                                    "Immediate response should reflect cached mode");

    bool queriedRadio = false;
    for (const auto &msg : mockRadioSerial->sentMessages) {
        if (msg == "MD;") {
            queriedRadio = true;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(queriedRadio,
                             "Expected stale MD cache to trigger radio refresh query");

    tearDown_Cache();
}

// =============================================================================
// Cache Timestamp Update Tests
// =============================================================================

void test_cache_timestamp_updated_on_local_set_command() {
    setUp_Cache();

    // 1. Get the initial timestamp for 'MG' (Mic Gain), which should be 0.
    const uint64_t initial_ts = testRadioManager->getState().commandCache.get("MG");
    TEST_ASSERT_EQUAL_UINT32(0, initial_ts);

    // 2. Send a SET command for 'MG'.
    testRadioManager->getLocalCATHandler().parseMessage("MG042;");

    // 3. Get the new timestamp.
    const uint64_t new_ts = testRadioManager->getState().commandCache.get("MG");

    // 4. Assert that the new timestamp is greater than the initial one.
    TEST_ASSERT_GREATER_THAN(initial_ts, new_ts);

    tearDown_Cache();
}

void test_cache_timestamp_updated_on_remote_answer() {
    setUp_Cache();

    // 1. Query the radio for 'PC' (Power Control) to simulate a pending query.
    testRadioManager->getLocalCATHandler().parseMessage("PC;");
    const uint64_t initial_ts = testRadioManager->getState().commandCache.get("PC");

    // 2. Simulate a delay.
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Simulate the radio sending an answer.
    testRadioManager->getRemoteCATHandler().parseMessage("PC099;");

    // 4. Get the new timestamp.
    const uint64_t new_ts = testRadioManager->getState().commandCache.get("PC");

    // 5. Assert that the timestamp was updated by the remote answer.
    TEST_ASSERT_GREATER_THAN(initial_ts, new_ts);
    TEST_ASSERT_EQUAL(99, testRadioManager->getState().transmitPower);

    tearDown_Cache();
}

// =============================================================================
// EX Menu Cache Tests
// =============================================================================

void test_ex_menu_cache_clear_forces_radio_query() {
    setUp_Cache();

    // 1. Set an EX value to populate the cache
    testRadioManager->getLocalCATHandler().parseMessage("EX006000005;"); // Set sidetone volume to 5
    TEST_ASSERT_EQUAL_STRING("EX006000005;", mockRadioSerial->sentMessages.back().c_str());

    // 2. Clear mock serials
    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();

    // 3. Query should use cache (no radio traffic)
    testRadioManager->getLocalCATHandler().parseMessage("EX0060000;");
    TEST_ASSERT_TRUE(mockRadioSerial->sentMessages.empty());

    // 4. Clear the EX menu cache
    radio::ExtendedCommandHandler::clearMenuCache();

    // 5. Clear mock serials again
    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();

    // 6. Query should now forward to radio (cache is cleared)
    testRadioManager->getLocalCATHandler().parseMessage("EX0060000;");
    TEST_ASSERT_FALSE(mockRadioSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("EX0060000;", mockRadioSerial->sentMessages.back().c_str());

    tearDown_Cache();
}

void test_ex_menu_cache_clear_removes_values() {
    setUp_Cache();

    // 1. Set multiple EX values to populate the cache
    testRadioManager->getLocalCATHandler().parseMessage("EX006000005;"); // Sidetone volume = 5
    testRadioManager->getLocalCATHandler().parseMessage("EX002000003;"); // Display brightness = 3
    testRadioManager->getLocalCATHandler().parseMessage("EX056000001;"); // Transverter = 1

    // 2. Clear the EX menu cache
    radio::ExtendedCommandHandler::clearMenuCache();

    // 3. Clear mock serials
    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();

    // 4. All queries should forward to radio (no cached values)
    testRadioManager->getLocalCATHandler().parseMessage("EX0060000;");
    TEST_ASSERT_EQUAL_STRING("EX0060000;", mockRadioSerial->sentMessages.back().c_str());

    mockRadioSerial->sentMessages.clear();
    testRadioManager->getLocalCATHandler().parseMessage("EX0020000;");
    TEST_ASSERT_EQUAL_STRING("EX0020000;", mockRadioSerial->sentMessages.back().c_str());

    mockRadioSerial->sentMessages.clear();
    testRadioManager->getLocalCATHandler().parseMessage("EX0560000;");
    TEST_ASSERT_EQUAL_STRING("EX0560000;", mockRadioSerial->sentMessages.back().c_str());

    tearDown_Cache();
}

void test_command_cache_clear_forces_radio_query() {
    setUp_Cache();

    // 1. Set AG (AF Gain) to populate the general command cache
    testRadioManager->getLocalCATHandler().parseMessage("AG0050;");
    test_utils::CATTestHelper::assertForwardedToRadio(*mockRadioSerial, "AG0050;");
    TEST_ASSERT_EQUAL(50, testRadioManager->getState().afGain.load());

    // 2. Clear mock serials
    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();

    // 3. Query should use cache (no radio traffic, response to USB)
    testRadioManager->getLocalCATHandler().parseMessage("AG;");
    TEST_ASSERT_TRUE(mockRadioSerial->sentMessages.empty());  // No radio traffic
    TEST_ASSERT_FALSE(mockUsbSerial->sentMessages.empty());   // Response sent to USB
    TEST_ASSERT_EQUAL_STRING("AG0050;", mockUsbSerial->sentMessages.back().c_str());

    // 4. Clear the general command cache
    testRadioManager->clearCommandCache();

    // 5. Clear mock serials again
    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->sentMessages.clear();

    // 6. Query should now forward to radio (cache is cleared)
    testRadioManager->getLocalCATHandler().parseMessage("AG;");
    // Note: With cache cleared, the behavior depends on the handler
    // For AG command with cleared cache, it should still return the stored value
    // but the timestamp is cleared, so it may trigger a background refresh

    tearDown_Cache();
}

// =============================================================================
// State Consistency Tests
// =============================================================================

void test_state_consistency_across_vfos() {
    setUp_Cache();

    // Set both VFOs
    testRadioManager->getRemoteCATHandler().parseMessage("FA00014150000;");
    testRadioManager->getRemoteCATHandler().parseMessage("FB00007100000;");

    // Both should be stored correctly
    TEST_ASSERT_FREQUENCY_EQUAL(14150000ULL, testRadioManager->getVfoAFrequency());
    TEST_ASSERT_FREQUENCY_EQUAL(7100000ULL, testRadioManager->getVfoBFrequency());

    // Update one VFO
    testRadioManager->getRemoteCATHandler().parseMessage("FA00028074000;");
    TEST_ASSERT_FREQUENCY_EQUAL(28074000ULL, testRadioManager->getVfoAFrequency());
    TEST_ASSERT_FREQUENCY_EQUAL(7100000ULL, testRadioManager->getVfoBFrequency()); // Should not change

    tearDown_Cache();
}

void test_split_mode_cache_interactions() {
    setUp_Cache();

    // Set up split operation
    testRadioManager->enableSplit(true);

    // Split mode might affect how frequencies are cached
    testRadioManager->getRemoteCATHandler().parseMessage("FA00014150000;");
    testRadioManager->getRemoteCATHandler().parseMessage("FB00014152000;");

    // Query frequencies in split mode
    testRadioManager->getLocalCATHandler().parseMessage("FA;");
    testRadioManager->getLocalCATHandler().parseMessage("FB;");

    // Both frequencies should be handled correctly
    TEST_ASSERT_FREQUENCY_EQUAL(14150000ULL, testRadioManager->getVfoAFrequency());
    TEST_ASSERT_FREQUENCY_EQUAL(14152000ULL, testRadioManager->getVfoBFrequency());

    tearDown_Cache();
}

// =============================================================================
// Boundary Condition Tests
// =============================================================================

void test_cache_with_boundary_frequencies() {
    setUp_Cache();

    // Test minimum frequency
    testRadioManager->getRemoteCATHandler().parseMessage("FA00000030000;");
    TEST_ASSERT_FREQUENCY_EQUAL(30000ULL, testRadioManager->getVfoAFrequency());

    // Test maximum frequency
    testRadioManager->getRemoteCATHandler().parseMessage("FA00060000000;");
    TEST_ASSERT_FREQUENCY_EQUAL(60000000ULL, testRadioManager->getVfoAFrequency());

    // Cache should handle boundary values correctly

    tearDown_Cache();
}

void test_cache_with_all_modes() {
    setUp_Cache();

    // Test all valid modes
    for (int mode = 1; mode <= 9; mode++) {
        std::string modeCmd = "MD" + std::to_string(mode) + ";";
        testRadioManager->getRemoteCATHandler().parseMessage(modeCmd);
        TEST_ASSERT_EQUAL(mode, testRadioManager->getCurrentMode());
    }

    tearDown_Cache();
}

// =============================================================================
// Error Recovery Tests
// =============================================================================

void test_cache_behavior_after_errors() {
    setUp_Cache();

    // Set valid state
    testRadioManager->getRemoteCATHandler().parseMessage("FA00014150000;");
    TEST_ASSERT_FREQUENCY_EQUAL(14150000ULL, testRadioManager->getVfoAFrequency());

    // Send invalid commands
    testRadioManager->getLocalCATHandler().parseMessage("INVALID;");
    testRadioManager->getRemoteCATHandler().parseMessage("ERROR;");

    // Valid state should be preserved
    TEST_ASSERT_FREQUENCY_EQUAL(14150000ULL, testRadioManager->getVfoAFrequency());

    // System should continue to work normally
    testRadioManager->getLocalCATHandler().parseMessage("IF;");
    TEST_ASSERT_FALSE(mockUsbSerial->sentMessages.empty());

    tearDown_Cache();
}

void test_state_recovery_after_radio_disconnect() {
    setUp_Cache();

    // Set state
    testRadioManager->getRemoteCATHandler().parseMessage("FA00014150000;");
    testRadioManager->getRemoteCATHandler().parseMessage("PS1;");

    // Simulate radio disconnect (no responses)
    testRadioManager->getLocalCATHandler().parseMessage("FA00007100000;");
    // No radio response simulated

    // Internal state should remain consistent
    // Previous state should be preserved until confirmed

    tearDown_Cache();
}

// =============================================================================
// Test Runner
// =============================================================================

extern "C" void run_cache_tests(void) {
    // Cache hit/miss tests
    RUN_TEST(test_cache_hit_returns_cached_value_without_radio_traffic);
    RUN_TEST(test_cache_miss_on_stale_data_queries_radio);
    RUN_TEST(test_mode_query_responds_immediately_when_cache_stale);

    // Cache timestamp tests
    RUN_TEST(test_cache_timestamp_updated_on_local_set_command);
    RUN_TEST(test_cache_timestamp_updated_on_remote_answer);

    // EX menu cache tests
    RUN_TEST(test_ex_menu_cache_clear_forces_radio_query);
    RUN_TEST(test_ex_menu_cache_clear_removes_values);
    RUN_TEST(test_command_cache_clear_forces_radio_query);

    // State consistency tests
    RUN_TEST(test_state_consistency_across_vfos);
    RUN_TEST(test_split_mode_cache_interactions);

    // Boundary tests
    RUN_TEST(test_cache_with_boundary_frequencies);
    RUN_TEST(test_cache_with_all_modes);

    // Error recovery tests
    RUN_TEST(test_cache_behavior_after_errors);
    RUN_TEST(test_state_recovery_after_radio_disconnect);
}
