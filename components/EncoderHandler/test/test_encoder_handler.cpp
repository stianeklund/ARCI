// test_encoder_handler.cpp

#include "../include/EncoderHandler.h"
#include "../../RadioCore/include/RadioManager.h"
#include "freertos/FreeRTOS.h"
#include <memory>
#include <stdio.h>
#include <string.h>

#include "../../SerialHandler/mock/include/MockSerialHandler.h"
#include "../../../components/unity/unity/src/unity.h"
#include "test_hooks.h"

// **************************************************************
// Hardware Stubs and Global Variables for Testing
// **************************************************************

// Test variables - simplified since we're not simulating GPIO interrupts

#define UART_OUTPUT_BUFFER_SIZE 256
static char uart_output_buffer[UART_OUTPUT_BUFFER_SIZE];
static size_t uart_output_index = 0;

extern "C" {
    // Helper functions for test output - these don't conflict with ESP-IDF
    void reset_uart_buffer(void) {
        uart_output_index = 0;
        uart_output_buffer[0] = '\0';
    }

    const char *get_uart_output(void) {
        return uart_output_buffer;
    }
}

// **************************************************************
// TestEncoderHandler Class (Derived from EncoderHandler)
// **************************************************************

class TestEncoderHandler : public EncoderHandler {
public:
    static std::unique_ptr<MockSerialHandler> mockRadioSerial;
    static std::unique_ptr<MockSerialHandler> mockUsbSerial;
    static std::unique_ptr<radio::RadioManager> mockRadioManager;

    TestEncoderHandler(const gpio_num_t pinA, const gpio_num_t pinB)
        : EncoderHandler(pinA, pinB, mockRadioManager.get()) {
    }

    // Test the public interface only - no private access needed
    bool testIsTuning() const {
        return isTuning();
    }
};

// **************************************************************
// Static member definitions
// **************************************************************
std::unique_ptr<MockSerialHandler> TestEncoderHandler::mockRadioSerial;
std::unique_ptr<MockSerialHandler> TestEncoderHandler::mockUsbSerial;
std::unique_ptr<radio::RadioManager> TestEncoderHandler::mockRadioManager;

// **************************************************************
// Global Objects and Setup/Teardown Functions
// **************************************************************

static std::unique_ptr<TestEncoderHandler> encoderHandler;
static bool gpio_isr_service_installed = false;

void encoderHandlerSetUp(void) {
    // Install GPIO ISR service only once for all tests
    if (!gpio_isr_service_installed) {
        esp_err_t install_err = gpio_install_isr_service(0);
        if (install_err == ESP_OK || install_err == ESP_ERR_INVALID_STATE) {
            gpio_isr_service_installed = true;
        } else {
            ESP_LOGE("TEST", "Failed to install ISR service: %s", esp_err_to_name(install_err));
        }
    }

    // Create fresh mock serial handlers
    TestEncoderHandler::mockRadioSerial = std::make_unique<MockSerialHandler>();
    TestEncoderHandler::mockUsbSerial = std::make_unique<MockSerialHandler>();

    // Initialize mock RadioManager object
    TestEncoderHandler::mockRadioManager = std::make_unique<radio::RadioManager>(
        *TestEncoderHandler::mockRadioSerial,
        *TestEncoderHandler::mockUsbSerial
    );

    // Create an instance of TestEncoderHandler
    encoderHandler = std::make_unique<TestEncoderHandler>(GPIO_NUM_0, GPIO_NUM_1);
    encoderHandler->setup();
}

void encoderHandlerTearDown(void) {
    encoderHandler.reset();
    TestEncoderHandler::mockRadioManager.reset();
    // Give FreeRTOS IDLE task time to clean up deleted tasks
    vTaskDelay(pdMS_TO_TICKS(1100));
    TestEncoderHandler::mockRadioSerial.reset();
    TestEncoderHandler::mockUsbSerial.reset();
    reset_uart_buffer();
}

// **************************************************************
// Unit Test Cases
// **************************************************************

// Test 1: Basic setup and teardown test
void test_encoder_setup(void) {
    encoderHandlerSetUp();
    // Verify encoder was created and setup completed without crash
    TEST_ASSERT_NOT_NULL(encoderHandler.get());
    // Verify initial tuning state
    TEST_ASSERT_FALSE(encoderHandler->testIsTuning());
    encoderHandlerTearDown();
}

// Test 2: Test task execution with no position changes
void test_encoder_task_no_change(void) {
    encoderHandlerSetUp();
    // Call task when no encoder movement has occurred
    encoderHandler->task(false);
    // Should complete without crash and tuning should be false
    TEST_ASSERT_FALSE(encoderHandler->testIsTuning());
    encoderHandlerTearDown();
}

// Test 3: Test multiple task calls
void test_encoder_multiple_tasks(void) {
    encoderHandlerSetUp();
    // Call task multiple times to verify stability
    encoderHandler->task(false);
    encoderHandler->task(false);
    encoderHandler->task(false);
    // Should complete without crash
    TEST_ASSERT_FALSE(encoderHandler->testIsTuning());
    encoderHandlerTearDown();
}

// Test 4: Test repeated task calls for stability
void test_encoder_repeated_calls(void) {
    encoderHandlerSetUp();
    
    // Call task repeatedly to test stability
    for (int i = 0; i < 10; i++) {
        encoderHandler->task(false);
    }
    
    // Should complete without crash
    TEST_ASSERT_FALSE(encoderHandler->testIsTuning());
    encoderHandlerTearDown();
}

// Test 5: Test tuning state
void test_encoder_tuning_state(void) {
    encoderHandlerSetUp();
    // Initially should not be tuning
    TEST_ASSERT_FALSE(encoderHandler->testIsTuning());
    
    // Call task - should still not be tuning when no movement
    encoderHandler->task(false);
    TEST_ASSERT_FALSE(encoderHandler->testIsTuning());
    
    encoderHandlerTearDown();
}

// **************************************************************
// Run EncoderHandler Tests
// **************************************************************

extern "C" void run_encoder_handler_tests(void) {
    RUN_TEST(test_encoder_setup);
    RUN_TEST(test_encoder_task_no_change);
    RUN_TEST(test_encoder_multiple_tasks);
    RUN_TEST(test_encoder_repeated_calls);
    RUN_TEST(test_encoder_tuning_state);
}
