#include "unity.h"
#include "test_hooks.h"
#include "../../RadioCore/test/include/CATTestUtils.h"
#include "FrequencyVfoCommandHandler.h"
#include "ExtendedCommandHandler.h"
#include "AntennaCommandHandler.h"
#include "RadioMacroManager.h"
#include "RadioManager.h"
#include "RadioCommand.h"
#include "CatParser.h"
#include "../../SerialHandler/mock/include/MockSerialHandler.h"
#include <memory>
#include <algorithm>

using namespace radio;

static std::unique_ptr<RadioManager> testRadioManager;
static std::unique_ptr<FrequencyVfoCommandHandler> freqHandler;
static std::unique_ptr<ExtendedCommandHandler> exHandler;
static std::unique_ptr<AntennaCommandHandler> antHandler;
static std::unique_ptr<RadioMacroManager> macroManager;
static std::unique_ptr<MockSerialHandler> radioSerial;
static std::unique_ptr<MockSerialHandler> usbSerial;

// Helper function to parse commands
RadioCommand parseCommand(const std::string& cmd, const CommandType type, const CommandSource source) {
    RadioCommand command;
    command.command = cmd.substr(0, 2); // Extract command prefix
    command.type = type;
    command.source = source;
    command.originalMessage = cmd;
    
    // Extract parameters (everything between command prefix and semicolon)
    if (cmd.length() > 3) { // Must have at least "XX;" format
        if (const size_t semicolonPos = cmd.find(';'); semicolonPos != std::string::npos && semicolonPos > 2) {
            if (const std::string paramStr = cmd.substr(2, semicolonPos - 2); !paramStr.empty()) {
                // For EX commands and others, store the full parameter string
                command.params.emplace_back(paramStr);
            }
        }
    }
    
    return command;
}

void setUp_transverter_tests() {
    radioSerial = std::make_unique<MockSerialHandler>();
    usbSerial = std::make_unique<MockSerialHandler>();

    testRadioManager = std::make_unique<RadioManager>(static_cast<ISerialChannel&>(*radioSerial), static_cast<ISerialChannel&>(*usbSerial));

    // RadioManager constructor already registers all handlers, so we don't need to register them again
    // Create separate handler instances ONLY for tests that call handleCommand() directly
    freqHandler = std::make_unique<FrequencyVfoCommandHandler>();
    exHandler = std::make_unique<ExtendedCommandHandler>();
    antHandler = std::make_unique<AntennaCommandHandler>();

    macroManager = std::make_unique<RadioMacroManager>(*testRadioManager);

    radioSerial->sentMessages.clear();
    usbSerial->sentMessages.clear();

    // Initialize transverter state
    testRadioManager->getState().transverter = false;
    testRadioManager->getState().transverterOffsetHz = 0;
    testRadioManager->getState().transverterOffsetPlus = true;
    testRadioManager->getState().drvConnectorMode = 0;
    testRadioManager->getState().hfLinearAmpControl = 0;
    testRadioManager->getState().vhfLinearAmpControl = 0;
}

void tearDown_transverter_tests() {
    testRadioManager.reset();
    freqHandler.reset();
    exHandler.reset();
    antHandler.reset();
    macroManager.reset();
    radioSerial.reset();
    usbSerial.reset();
}

// =============================================================================
// Test XO Command Parsing and State Updates
// =============================================================================

void test_xo_command_parsing() {
    setUp_transverter_tests();
    // Test XO command parsing for transverter offset
    // XO000116000000; = plus direction, 116 MHz offset
    testRadioManager->getLocalCATHandler().parseMessage("XO000116000000;");
    // Check that state was updated correctly
    TEST_ASSERT_TRUE(testRadioManager->getState().transverterOffsetPlus);
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(116000000ULL), static_cast<uint32_t>(testRadioManager->getState().transverterOffsetHz));
    tearDown_transverter_tests();
}

void test_xo_command_minus_direction() {
    setUp_transverter_tests();
    // Test XO with minus direction
    // XO100144000000; = minus direction, 144 MHz offset
    testRadioManager->getLocalCATHandler().parseMessage("XO100144000000;");

    // Check that state was updated correctly
    TEST_ASSERT_FALSE(testRadioManager->getState().transverterOffsetPlus);
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(144000000ULL), static_cast<uint32_t>(testRadioManager->getState().transverterOffsetHz));
    tearDown_transverter_tests();
}

void test_xo_answer_parsing() {
    setUp_transverter_tests();
    // Test XO answer from radio updates state
    testRadioManager->getRemoteCATHandler().parseMessage("XO000116000000;");
    TEST_ASSERT_TRUE(testRadioManager->getState().transverterOffsetPlus);
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(116000000ULL), static_cast<uint32_t>(testRadioManager->getState().transverterOffsetHz));
    tearDown_transverter_tests();
}

// =============================================================================
// Test Frequency Translation in FA/FB Commands
// =============================================================================

void test_fa_query_with_transverter_enabled() {
    setUp_transverter_tests();
    // Setup: Enable transverter with 116 MHz offset
    testRadioManager->getState().transverter = true;
    testRadioManager->getState().transverterOffsetEnabled = true;
    testRadioManager->getState().transverterOffsetPlus = true;
    testRadioManager->getState().transverterOffsetHz = 116000000ULL; // 116 MHz
    testRadioManager->updateVfoAFrequency(28360000ULL); // 28.360 MHz radio frequency

    // Mark cache as fresh
    testRadioManager->getState().commandCache.update("FA", esp_timer_get_time());

    // Query FA from USB
    const RadioCommand cmd = parseCommand("FA;", CommandType::Read, CommandSource::UsbCdc0);

    const bool result = freqHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should return transverter frequency (28.360 + 116 = 144.360 MHz)
    const std::string response = usbSerial->sentMessages.empty() ? "" : usbSerial->sentMessages.back();
    TEST_ASSERT_EQUAL_STRING("FA00144360000;", response.c_str());
    tearDown_transverter_tests();
}

void test_fa_query_with_transverter_disabled() {
    setUp_transverter_tests();
    // Setup: Disable transverter
    testRadioManager->getState().transverter = false;
    testRadioManager->updateVfoAFrequency(28360000ULL); // 28.360 MHz radio frequency

    // Mark cache as fresh
    testRadioManager->getState().commandCache.update("FA", esp_timer_get_time());

    // Query FA from USB
    const RadioCommand cmd = parseCommand("FA;", CommandType::Read, CommandSource::UsbCdc0);

    const bool result = freqHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should return actual radio frequency (no offset)
    const std::string response = usbSerial->sentMessages.empty() ? "" : usbSerial->sentMessages.back();
    TEST_ASSERT_EQUAL_STRING("FA00028360000;", response.c_str());
    tearDown_transverter_tests();
}

void test_fa_set_with_transverter_conversion() {
    setUp_transverter_tests();
    // Setup: Enable transverter with 116 MHz offset
    testRadioManager->getState().transverter = true;
    testRadioManager->getState().transverterOffsetEnabled = true;
    testRadioManager->getState().transverterOffsetPlus = true;
    testRadioManager->getState().transverterOffsetHz = 116000000ULL; // 116 MHz

    // Set FA to 144.360 MHz from USB (transverter frequency)
    const RadioCommand cmd = parseCommand("FA00144360000;", CommandType::Set, CommandSource::UsbCdc0);

    const bool result = freqHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should send converted frequency to radio (144.360 - 116 = 28.360 MHz)
    const std::string radioCmd = radioSerial->sentMessages.empty() ? "" : radioSerial->sentMessages.back();
    TEST_ASSERT_EQUAL_STRING("FA00028360000;", radioCmd.c_str());

    // State should be updated with radio frequency
    TEST_ASSERT_EQUAL_UINT32(28360000ULL, testRadioManager->getVfoAFrequency());
    tearDown_transverter_tests();
}

void test_fa_answer_forwarding_with_transverter() {
    setUp_transverter_tests();
    // Setup: Enable transverter and record a query
    testRadioManager->getState().transverter = true;
    testRadioManager->getState().transverterOffsetEnabled = true;
    testRadioManager->getState().transverterOffsetPlus = true;
    testRadioManager->getState().transverterOffsetHz = 116000000ULL; // 116 MHz
    const uint64_t nowUs = esp_timer_get_time();
    testRadioManager->getState().queryTracker.recordQuery("FA", nowUs);
    testRadioManager->noteQueryOrigin("FA", CommandSource::UsbCdc0, nowUs);

    // Radio responds with FA00028360000; (28.360 MHz)
    const RadioCommand cmd = parseCommand("FA00028360000;", CommandType::Answer, CommandSource::Remote);

    const bool result = freqHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should forward transverter frequency to USB (28.360 + 116 = 144.360 MHz)
    const std::string response = usbSerial->sentMessages.empty() ? "" : usbSerial->sentMessages.back();
    TEST_ASSERT_EQUAL_STRING("FA00144360000;", response.c_str());
    tearDown_transverter_tests();
}

void test_fb_transverter_functionality() {
    setUp_transverter_tests();
    // Test FB command with same transverter logic as FA
    testRadioManager->getState().transverter = true;
    testRadioManager->getState().transverterOffsetEnabled = true;
    testRadioManager->getState().transverterOffsetPlus = true;
    testRadioManager->getState().transverterOffsetHz = 116000000ULL;
    testRadioManager->updateVfoBFrequency(28360000ULL);

    // Mark cache as fresh
    testRadioManager->getState().commandCache.update("FB", esp_timer_get_time());

    // Query FB from USB
    const RadioCommand cmd = parseCommand("FB;", CommandType::Read, CommandSource::UsbCdc0);

    const bool result = freqHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should return transverter frequency
    const std::string response = usbSerial->sentMessages.empty() ? "" : usbSerial->sentMessages.back();
    TEST_ASSERT_EQUAL_STRING("FB00144360000;", response.c_str());
    tearDown_transverter_tests();
}

void test_transverter_minus_offset() {
    setUp_transverter_tests();
    // Test with minus offset direction
    testRadioManager->getState().transverter = true;
    testRadioManager->getState().transverterOffsetEnabled = true;
    testRadioManager->getState().transverterOffsetPlus = false; // MINUS direction
    testRadioManager->getState().transverterOffsetHz = 28000000ULL; // 28 MHz offset
    testRadioManager->updateVfoAFrequency(144360000ULL); // Radio at 144.360 MHz

    // Mark cache as fresh
    testRadioManager->getState().commandCache.update("FA", esp_timer_get_time());

    // Query FA from USB
    const RadioCommand cmd = parseCommand("FA;", CommandType::Read, CommandSource::UsbCdc0);

    const bool result = freqHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Display = Radio - Offset = 144.360 - 28 = 116.360 MHz
    const std::string response = usbSerial->sentMessages.empty() ? "" : usbSerial->sentMessages.back();
    TEST_ASSERT_EQUAL_STRING("FA00116360000;", response.c_str());
    tearDown_transverter_tests();
}

// =============================================================================
// Test Extended Menu Command Parsing
// =============================================================================

void test_ex085_drv_connector_parsing() {
    setUp_transverter_tests();
    // Test EX085 parsing for DRV connector mode - value "0" = DRO mode
    const RadioCommand cmd = parseCommand("EX08500000;", CommandType::Set, CommandSource::UsbCdc0);

    const bool result = exHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should set DRV connector to DRO mode (0)
    TEST_ASSERT_EQUAL_INT(0, testRadioManager->getState().drvConnectorMode);
    tearDown_transverter_tests();
}

void test_ex085_ant_connector_parsing() {
    setUp_transverter_tests();
    // Test EX085 parsing for ANT mode - value "1" = ANT mode
    const RadioCommand cmd = parseCommand("EX08500001;", CommandType::Set, CommandSource::UsbCdc0);

    const bool result = exHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should set DRV connector to ANT mode (1)
    TEST_ASSERT_EQUAL_INT(1, testRadioManager->getState().drvConnectorMode);
    tearDown_transverter_tests();
}

void test_ex059_hf_linear_amp_parsing() {
    setUp_transverter_tests();
    // Test EX059 parsing for HF linear amplifier control
    const RadioCommand cmd = parseCommand("EX05900003;", CommandType::Set, CommandSource::UsbCdc0);

    const bool result = exHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should set HF linear amp control to 3
    TEST_ASSERT_EQUAL_INT(3, testRadioManager->getState().hfLinearAmpControl);
    tearDown_transverter_tests();
}

void test_ex060_vhf_linear_amp_parsing() {
    setUp_transverter_tests();
    // Test EX060 parsing for VHF linear amplifier control
    const RadioCommand cmd = parseCommand("EX06000003;", CommandType::Set, CommandSource::UsbCdc0);

    const bool result = exHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should set VHF linear amp control to 3
    TEST_ASSERT_EQUAL_INT(3, testRadioManager->getState().vhfLinearAmpControl);
    tearDown_transverter_tests();
}

// =============================================================================
// Test Transverter Macro Operation
// =============================================================================



// =============================================================================
// Test UIXD Command (Transverter Display Offset Toggle)
// =============================================================================

void test_uixd_enable_display_offset() {
    setUp_transverter_tests();
    // UIXD1; should enable transverter display offset
    testRadioManager->getState().transverterOffsetEnabled = false;

    testRadioManager->getLocalCATHandler().parseMessage("UIXD1;");

    TEST_ASSERT_TRUE(testRadioManager->getState().transverterOffsetEnabled);
    tearDown_transverter_tests();
}

void test_uixd_disable_display_offset() {
    setUp_transverter_tests();
    // UIXD0; should disable transverter display offset
    testRadioManager->getState().transverterOffsetEnabled = true;

    testRadioManager->getLocalCATHandler().parseMessage("UIXD0;");

    TEST_ASSERT_FALSE(testRadioManager->getState().transverterOffsetEnabled);
    tearDown_transverter_tests();
}

void test_uixd_toggle_cycle() {
    setUp_transverter_tests();
    // Test toggling UIXD on and off in sequence
    testRadioManager->getState().transverterOffsetEnabled = false;

    // Enable
    testRadioManager->getLocalCATHandler().parseMessage("UIXD1;");
    TEST_ASSERT_TRUE(testRadioManager->getState().transverterOffsetEnabled);

    // Disable
    testRadioManager->getLocalCATHandler().parseMessage("UIXD0;");
    TEST_ASSERT_FALSE(testRadioManager->getState().transverterOffsetEnabled);

    // Enable again
    testRadioManager->getLocalCATHandler().parseMessage("UIXD1;");
    TEST_ASSERT_TRUE(testRadioManager->getState().transverterOffsetEnabled);

    tearDown_transverter_tests();
}

// =============================================================================
// Test Edge Cases and Error Handling
// =============================================================================

void test_frequency_underflow_protection() {
    setUp_transverter_tests();
    // Test that frequency subtraction doesn't cause underflow
    testRadioManager->getState().transverter = true;
    testRadioManager->getState().transverterOffsetEnabled = true;
    testRadioManager->getState().transverterOffsetPlus = false; // MINUS direction
    testRadioManager->getState().transverterOffsetHz = 50000000ULL; // 50 MHz offset
    testRadioManager->updateVfoAFrequency(14000000ULL); // 14 MHz (smaller than offset)

    // Mark cache as fresh
    testRadioManager->getState().commandCache.update("FA", esp_timer_get_time());

    // Query FA from USB
    const RadioCommand cmd = parseCommand("FA;", CommandType::Read, CommandSource::UsbCdc0);

    const bool result = freqHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should return 0 Hz (underflow protection)
    const std::string response = usbSerial->sentMessages.empty() ? "" : usbSerial->sentMessages.back();
    TEST_ASSERT_EQUAL_STRING("FA00000000000;", response.c_str());
    tearDown_transverter_tests();
}

void test_transverter_with_zero_offset() {
    setUp_transverter_tests();
    // Test transverter with zero offset (should pass through unchanged)
    testRadioManager->getState().transverter = true;
    testRadioManager->getState().transverterOffsetHz = 0; // Zero offset
    testRadioManager->updateVfoAFrequency(14235000ULL); // 14.235 MHz

    // Mark cache as fresh
    testRadioManager->getState().commandCache.update("FA", esp_timer_get_time());

    // Query FA from USB
    const RadioCommand cmd = parseCommand("FA;", CommandType::Read, CommandSource::UsbCdc0);

    const bool result = freqHandler->handleCommand(cmd, *radioSerial, *usbSerial, *testRadioManager);
    TEST_ASSERT_TRUE(result);

    // Should return original frequency (no offset applied)
    const std::string response = usbSerial->sentMessages.empty() ? "" : usbSerial->sentMessages.back();
    TEST_ASSERT_EQUAL_STRING("FA00014235000;", response.c_str());
    tearDown_transverter_tests();
}

// =============================================================================
// Test Runner
// =============================================================================

extern "C" void run_transverter_tests() {
    printf("Running Transverter Feature Tests...\n");
    
    // XO Command Tests
    RUN_TEST(test_xo_command_parsing);
    RUN_TEST(test_xo_command_minus_direction);
    RUN_TEST(test_xo_answer_parsing);
    
    // Frequency Translation Tests
    RUN_TEST(test_fa_query_with_transverter_enabled);
    RUN_TEST(test_fa_query_with_transverter_disabled);
    RUN_TEST(test_fa_set_with_transverter_conversion);
    RUN_TEST(test_fa_answer_forwarding_with_transverter);
    RUN_TEST(test_fb_transverter_functionality);
    RUN_TEST(test_transverter_minus_offset);
    
    // Extended Menu Tests
    RUN_TEST(test_ex085_drv_connector_parsing);
    RUN_TEST(test_ex085_ant_connector_parsing);
    RUN_TEST(test_ex059_hf_linear_amp_parsing);
    RUN_TEST(test_ex060_vhf_linear_amp_parsing);

    // UIXD Command Tests
    RUN_TEST(test_uixd_enable_display_offset);
    RUN_TEST(test_uixd_disable_display_offset);
    RUN_TEST(test_uixd_toggle_cycle);

    // Edge Case Tests
    RUN_TEST(test_frequency_underflow_protection);
    RUN_TEST(test_transverter_with_zero_offset);
}
