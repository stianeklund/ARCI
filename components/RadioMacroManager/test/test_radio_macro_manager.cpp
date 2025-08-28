#include "unity.h"
#include "test_hooks.h"
#include "../include/RadioMacroManager.h"
#include "../../SerialHandler/mock/include/MockSerialHandler.h"
#include "../../RadioCore/include/CATHandler.h"
#include "../../RadioCore/include/RadioManager.h"
#include "../../RadioCore/include/CommandDispatcher.h"
#include "../../CommandHandlers/include/MenuConfigCommandHandler.h"
#include "../../CommandHandlers/include/AntennaCommandHandler.h"
#include "../../CommandHandlers/include/FrequencyVfoCommandHandler.h"
#include <cstring>

using radio::RadioMacroManager;
using radio::CATHandler;
using radio::RadioManager;
using radio::CommandDispatcher;
using radio::MenuConfigCommandHandler;
using radio::AntennaCommandHandler;
using radio::FrequencyVfoCommandHandler;

static std::unique_ptr<MockSerialHandler> mockRadioSerial;
static std::unique_ptr<MockSerialHandler> mockUsbSerial;
static std::unique_ptr<RadioManager> radioManager;
static std::unique_ptr<CommandDispatcher> commandDispatcher;
static std::unique_ptr<CATHandler> catHandler;
static std::unique_ptr<RadioMacroManager> macroManager;

using radio::cat::ParserUtils;

void radioMacroManagerSetUp(void) {
    // Create fresh mock serial handlers for each test
    mockRadioSerial = std::make_unique<MockSerialHandler>();
    mockUsbSerial = std::make_unique<MockSerialHandler>();

    radioManager = std::make_unique<RadioManager>(*mockRadioSerial, *mockUsbSerial);
    commandDispatcher = std::make_unique<CommandDispatcher>();
    
    // Register handlers needed for transverter macro commands
    auto menuConfigHandler = std::make_unique<MenuConfigCommandHandler>();
    commandDispatcher->registerHandler(std::move(menuConfigHandler));
    
    auto antennaHandler = std::make_unique<AntennaCommandHandler>();
    commandDispatcher->registerHandler(std::move(antennaHandler));
    
    auto frequencyHandler = std::make_unique<FrequencyVfoCommandHandler>();
    commandDispatcher->registerHandler(std::move(frequencyHandler));
    
    catHandler = std::make_unique<CATHandler>(*commandDispatcher, *radioManager, *mockRadioSerial, *mockUsbSerial, radio::CommandSource::UsbCdc0);
    macroManager = std::make_unique<RadioMacroManager>(*radioManager);
}

void radioMacroManagerTearDown(void) {
    macroManager.reset();
    catHandler.reset();
    commandDispatcher.reset();
    radioManager.reset();
    // Give FreeRTOS IDLE task time to clean up deleted tasks
    // Need to wait longer than the keepalive task interval (1000ms)
    vTaskDelay(pdMS_TO_TICKS(1100));
    mockRadioSerial.reset();
    mockUsbSerial.reset();
}

void test_transverter_macro_enable() {
    radioMacroManagerSetUp();
    // Test comprehensive transverter macro enable
    const bool result = macroManager->executeTransverterMacro(true);
    TEST_ASSERT_TRUE(result);

    // Check that the macro sent all required commands
    const std::vector<std::string>& commands = mockRadioSerial->sentMessages;

    // Should include menu queries
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "EX0850000;"));  // Read DRV connector
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "EX0590000;"));  // Read HF linear amp
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "EX0600000;"));  // Read VHF linear amp

    // Should include configuration commands (only those that need to change)
    // Note: With read-compare-set, only commands that differ from current state are sent
    // Since we start with default state (all zeros), all enable commands should be sent
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "EX05600001;")); // Enable transverter (value 1 = ON)
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "EX08500000;")); // Set DRV to DRO
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "EX05900003;")); // Set HF amp to 3
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "EX06000003;")); // Set VHF amp to 3
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "AN911;"));       // Enable RX ANT and DRV
    radioMacroManagerTearDown();
}

void test_transverter_macro_disable() {

    radioMacroManagerSetUp();
    // Test transverter macro disable
    const bool result = macroManager->executeTransverterMacro(false);
    TEST_ASSERT_TRUE(result);

    // Check that disable commands were sent
    const std::vector<std::string>& commands = mockRadioSerial->sentMessages;
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "EX05600000;")); // Disable transverter
    TEST_ASSERT_TRUE(ParserUtils::vectorContains(commands, "AN900;"));       // RX ANT OFF, DRV OUT OFF (P1=9)
    
    // Verify that HF/VHF amplifier settings are NOT changed on disable (preserved for PTT detection)
    TEST_ASSERT_FALSE(ParserUtils::vectorContains(commands, "EX05900000;")); // Should NOT reset HF amp
    TEST_ASSERT_FALSE(ParserUtils::vectorContains(commands, "EX06000000;")); // Should NOT reset VHF amp
    radioMacroManagerTearDown();
}

void test_can_execute_macro_validation(void) {
    radioMacroManagerSetUp();
    
    // Test known macros
    TEST_ASSERT_TRUE(macroManager->canExecuteMacro("transverter"));
    
    // Test unimplemented macros
    TEST_ASSERT_FALSE(macroManager->canExecuteMacro("band_change"));
    TEST_ASSERT_FALSE(macroManager->canExecuteMacro("contest_mode"));
    
    // Test unknown macro
    TEST_ASSERT_FALSE(macroManager->canExecuteMacro("unknown_macro"));
    
    radioMacroManagerTearDown();
}

void test_unimplemented_macros_return_false(void) {
    radioMacroManagerSetUp();
    
    // Band change macro should return false (not implemented)
    bool result = macroManager->executeBandChangeMacro(5);
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_TRUE(strlen(macroManager->getLastError().c_str()) > 0);
    
    // Contest mode macro should return false (not implemented)
    result = macroManager->executeContestModeMacro();
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_TRUE(strlen(macroManager->getLastError().c_str()) > 0);
    
    radioMacroManagerTearDown();
}

void test_error_handling(void) {
    radioMacroManagerSetUp();
    
    // Test that error messages are properly stored and retrieved
    const std::string initialError = macroManager->getLastError();
    TEST_ASSERT_EQUAL_STRING("", initialError.c_str());
    
    // Execute an unimplemented macro to trigger error
    macroManager->executeBandChangeMacro(1);
    const std::string error = macroManager->getLastError();
    TEST_ASSERT_TRUE(strlen(error.c_str()) > 0);
    TEST_ASSERT_TRUE(error.find("not implemented") != std::string::npos);
    
    radioMacroManagerTearDown();
}

extern "C" void run_radio_macro_manager_tests(void) {
    RUN_TEST(test_transverter_macro_enable);
    RUN_TEST(test_transverter_macro_disable);
    RUN_TEST(test_can_execute_macro_validation);
    RUN_TEST(test_unimplemented_macros_return_false);
    RUN_TEST(test_error_handling);
}
