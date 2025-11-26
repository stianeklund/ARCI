#include "tests_main.h"
#ifdef CONFIG_RUN_UNIT_TESTS
#include "../../../components/unity/unity/src/unity.h"
#include "test_hooks.h"

#include "esp_log.h"
#include <vector>
#include <string>
#include <utility>
#include <cstring>

static auto TAG = "TESTS";

// Storage for suite-specific failure counts
static std::vector<std::pair<std::string, uint32_t>> suiteFailures;

// Storage for individual failing test names
static std::vector<std::string> individualFailingTests;

extern "C" void register_failing_test(const char* test_name) {
    if (test_name && *test_name) {
        individualFailingTests.emplace_back(test_name);
    }
}

// Helper to run test suites and track failures per suite
static void runTestSuiteWithFailureTracking(const char* suiteName, void (*testFunc)()) {
    ESP_LOGI(TAG, "Running %s tests...", suiteName);

    const uint32_t failuresBeforeSuite = Unity.TestFailures;
    testFunc();

    // If this suite had failures, record it
    if (const uint32_t failuresAfterSuite = Unity.TestFailures; failuresAfterSuite > failuresBeforeSuite) {
        uint32_t suiteFailureCount = failuresAfterSuite - failuresBeforeSuite;
        suiteFailures.emplace_back(suiteName, suiteFailureCount);
        ESP_LOGI(TAG, "❌ %s suite had %lu failure(s)", suiteName, suiteFailureCount);
    } else {
        ESP_LOGI(TAG, "✅ %s suite passed all tests", suiteName);
    }
}


extern "C" void run_button_handler_tests(void);
extern "C" void run_radiocat_handler_tests(void);
extern "C" void run_cat_parser_utils_tests(void);
extern "C" void run_radiomanager_cat_tests(void);
extern "C" void run_encoder_handler_tests(void);
extern "C" void run_radio_macro_manager_tests(void);
extern "C" void run_cat_parser_tests(void);
extern "C" void run_cache_tests(void); // Consolidated cache tests (replaces cache_behavior + cache_system)
extern "C" void run_meter_polling_performance_tests(void);
extern "C" void run_transverter_tests(void);
extern "C" void run_consolidated_command_handlers_tests(void);
// extern "C" void run_serial_handler_queue_tests(void); // Temporarily disabled

extern "C" void run_all_tests(void) {
    ESP_LOGI(TAG, "Starting unit tests...");
    
    // Clear previous failure tracking
    suiteFailures.clear();
    individualFailingTests.clear();
    
    UNITY_BEGIN();
    
    // Store initial test stats
    uint32_t initial_tests_run = Unity.NumberOfTests;
    uint32_t initial_failures = Unity.TestFailures;
    
    runTestSuiteWithFailureTracking("ButtonHandler", run_button_handler_tests);
    runTestSuiteWithFailureTracking("CAT Parser Utils", run_cat_parser_utils_tests);
    runTestSuiteWithFailureTracking("Radio CAT Handler", run_radiocat_handler_tests);
    runTestSuiteWithFailureTracking("RadioManager CAT", run_radiomanager_cat_tests);
    runTestSuiteWithFailureTracking("CAT Parser", run_cat_parser_tests);
    runTestSuiteWithFailureTracking("Cache", run_cache_tests); // Consolidated cache tests
    runTestSuiteWithFailureTracking("Meter Polling Performance", run_meter_polling_performance_tests);
    runTestSuiteWithFailureTracking("Transverter", run_transverter_tests);
    runTestSuiteWithFailureTracking("EncoderHandler", run_encoder_handler_tests);
    runTestSuiteWithFailureTracking("RadioMacroManager", run_radio_macro_manager_tests);
    runTestSuiteWithFailureTracking("Consolidated Command Handlers", run_consolidated_command_handlers_tests);
    // runTestSuiteWithFailureTracking("SerialHandler Queue", run_serial_handler_queue_tests); // Temporarily disabled
    
    UNITY_END();
    
    // Calculate final results
    uint32_t total_tests = Unity.NumberOfTests - initial_tests_run;
    uint32_t total_failures = Unity.TestFailures - initial_failures;
    uint32_t total_passed = total_tests - total_failures;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "================== TEST SUMMARY ==================");
    ESP_LOGI(TAG, "Total Tests:  %lu", total_tests);
    ESP_LOGI(TAG, "Passed:       %lu", total_passed);
    ESP_LOGI(TAG, "Failed:       %lu", total_failures);
    ESP_LOGI(TAG, "Success Rate: %.1f%%", total_tests > 0 ? (100.0 * total_passed / total_tests) : 100.0);
    ESP_LOGI(TAG, "==================================================");
    
    if (total_failures > 0) {
        ESP_LOGE(TAG, "❌ %lu test(s) failed in the following test suites:", total_failures);
        ESP_LOGE(TAG, "");
        
        // Show which suites had failures
        if (!suiteFailures.empty()) {
            ESP_LOGE(TAG, "FAILING TEST SUITES:");
            for (const auto& [suiteName, failureCount] : suiteFailures) {
                ESP_LOGE(TAG, "   - %s: %lu failure(s)", suiteName.c_str(), failureCount);
            }
        }
        
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "🔍 FAILING TEST DETAILS:");
        ESP_LOGE(TAG, "   Scroll up to find lines containing ':FAIL:' in format:");
        ESP_LOGE(TAG, "   test_function_name:FAIL: Expected 116000000 Was 0");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "📋 FAILING TESTS SUMMARY:");
        
        // Display individual failing test names
        if (!individualFailingTests.empty()) {
            ESP_LOGE(TAG, "   Individual failing tests:");
            for (const auto& testName : individualFailingTests) {
                ESP_LOGE(TAG, "   - %s:FAIL", testName.c_str());
            }
        } else {
            ESP_LOGE(TAG, "   Run the specific failing test suites listed above to see individual test names.");
            ESP_LOGE(TAG, "   Each failing test shows as: 'test_name:FAIL: reason'");
        }
        
        ESP_LOGE(TAG, "");
    } else {
        ESP_LOGI(TAG, "✅ All tests passed!");
    }
}
#endif
