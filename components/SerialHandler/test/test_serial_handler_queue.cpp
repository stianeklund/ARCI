#include "unity.h"
#include "test_hooks.h"
#include "../include/SerialHandler.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <vector>
#include <string>

static const char* TAG = "test_serial_handler_queue";

extern "C" {

// Test SerialHandler queue capacity and timeout behavior
static SerialHandler* testHandler = nullptr;

void setUp_SerialHandlerQueue() {
    testHandler = new SerialHandler(UART_NUM_0);  // Use dummy UART port for testing
}

void tearDown_SerialHandlerQueue() {
    delete testHandler;
    testHandler = nullptr;
}

// Test that queue capacity is reduced from 64 to 16
void test_queue_capacity_reduced(void) {
    TEST_ASSERT_NOT_NULL(testHandler);
    
    // Fill queue with messages by simulating received data
    std::vector<std::string> testMessages;
    for (int i = 0; i < 20; i++) {  // Try to add more than capacity
        std::string msg = "FA" + std::to_string(i) + ";";
        testMessages.push_back(msg);
        testHandler->processReceivedData(
            reinterpret_cast<const uint8_t*>(msg.c_str()), 
            msg.length()
        );
    }
    
    // Count how many messages are actually queued
    int messageCount = 0;
    while (testHandler->hasMessage()) {
        auto [status, message] = testHandler->getMessage();
        if (status == ESP_OK) {
            messageCount++;
        }
    }
    
    // Should be limited to QUEUE_CAPACITY (16)
    TEST_ASSERT_EQUAL(16, messageCount);
    ESP_LOGI(TAG, "Queue capacity correctly limited to %d messages", messageCount);
}

// Test that messages expire after timeout period
void test_message_timeout_expiry(void) {
    TEST_ASSERT_NOT_NULL(testHandler);
    
    // Add a message to the queue
    std::string testMsg = "FB14074000;";
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(testMsg.c_str()), 
        testMsg.length()
    );
    
    // Verify message is queued
    TEST_ASSERT_TRUE(testHandler->hasMessage());
    
    // Wait for timeout period (1.5 seconds + margin)
    vTaskDelay(pdMS_TO_TICKS(1600));
    
    // Try to get message - should trigger clearExpiredMessages()
    auto [status, message] = testHandler->getMessage();
    
    // Message should have expired and been cleared
    TEST_ASSERT_EQUAL(ESP_FAIL, status);
    TEST_ASSERT_FALSE(testHandler->hasMessage());
    ESP_LOGI(TAG, "Message correctly expired after timeout");
}

// Test that fresh messages are not cleared
void test_fresh_messages_not_cleared(void) {
    TEST_ASSERT_NOT_NULL(testHandler);
    
    // Add a fresh message
    std::string testMsg = "IF00014074000000000000000000000030000000;";
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(testMsg.c_str()), 
        testMsg.length()
    );
    
    // Small delay (much less than timeout)
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Message should still be available
    TEST_ASSERT_TRUE(testHandler->hasMessage());
    
    auto [status, message] = testHandler->getMessage();
    TEST_ASSERT_EQUAL(ESP_OK, status);
    TEST_ASSERT_EQUAL_STRING("IF00014074000000000000000000000030000000;", message.c_str());
    ESP_LOGI(TAG, "Fresh message correctly preserved: %s", message.c_str());
}

// Test mixed age messages - only old ones are cleared
void test_mixed_age_message_clearing(void) {
    TEST_ASSERT_NOT_NULL(testHandler);
    
    // Add an old message
    std::string oldMsg = "RM1;";
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(oldMsg.c_str()), 
        oldMsg.length()
    );
    
    // Wait to make it stale
    vTaskDelay(pdMS_TO_TICKS(1600));
    
    // Add a fresh message
    std::string freshMsg = "SM0050;";
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(freshMsg.c_str()), 
        freshMsg.length()
    );
    
    // Try to get messages - should clear old but keep fresh
    auto [status1, message1] = testHandler->getMessage();
    
    // Should get the fresh message (old one cleared automatically)
    TEST_ASSERT_EQUAL(ESP_OK, status1);
    TEST_ASSERT_EQUAL_STRING("SM0050;", message1.c_str());
    
    // No more messages should be available
    TEST_ASSERT_FALSE(testHandler->hasMessage());
    ESP_LOGI(TAG, "Old message cleared, fresh message preserved: %s", message1.c_str());
}

// Test that queue doesn't accumulate stale commands during high traffic
void test_high_traffic_queue_behavior(void) {
    TEST_ASSERT_NOT_NULL(testHandler);
    
    // Simulate rapid CAT command traffic
    std::vector<std::string> commands = {
        "FA14074000;", "FB14074000;", "SM0040;", "RM1;", "IF00014074000000000000000000000030000000;",
        "FA14075000;", "FB14075000;", "SM0045;", "RM2;", "IF00014075000000000000000000000030000000;",
        "FA14076000;", "FB14076000;", "SM0050;", "RM1;", "IF00014076000000000000000000000030000000;",
        "FA14077000;", "FB14077000;", "SM0055;", "RM2;", "IF00014077000000000000000000000030000000;"
    };
    
    // Rapidly add commands
    for (const auto& cmd : commands) {
        testHandler->processReceivedData(
            reinterpret_cast<const uint8_t*>(cmd.c_str()), 
            cmd.length()
        );
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms between commands
    }
    
    // Wait for some to expire
    vTaskDelay(pdMS_TO_TICKS(1600));
    
    // Add one more fresh command
    std::string finalCmd = "PS1;";
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(finalCmd.c_str()), 
        finalCmd.length()
    );
    
    // Count remaining messages
    int remainingCount = 0;
    std::vector<std::string> remainingMessages;
    while (testHandler->hasMessage()) {
        auto [status, message] = testHandler->getMessage();
        if (status == ESP_OK) {
            remainingMessages.push_back(message);
            remainingCount++;
        }
    }
    
    // Should have fewer messages than we sent (old ones cleared)
    TEST_ASSERT_LESS_THAN(commands.size(), remainingCount);
    TEST_ASSERT_GREATER_THAN(0, remainingCount);
    
    // Last message should be the fresh one we added
    bool foundFreshMessage = false;
    for (const auto& msg : remainingMessages) {
        if (msg == finalCmd) {
            foundFreshMessage = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(foundFreshMessage);
    
    ESP_LOGI(TAG, "High traffic test: %zu commands sent, %d remain after cleanup", 
             commands.size(), remainingCount);
}

void run_serial_handler_queue_tests(void) {
    RUN_TEST(test_queue_capacity_reduced);
    RUN_TEST(test_message_timeout_expiry);
    RUN_TEST(test_fresh_messages_not_cleared);
    RUN_TEST(test_mixed_age_message_clearing);
    RUN_TEST(test_high_traffic_queue_behavior);
}

} // extern "C"