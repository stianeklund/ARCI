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

// Queue is a fixed-capacity ring; SerialHandler::QUEUE_CAPACITY is 64 (private, so
// the literal is used here). Enqueue more than capacity and confirm the ring caps
// at 64 by dropping the oldest frames.
static constexpr int QUEUE_CAPACITY = 64;

void test_queue_capacity_caps_at_64(void) {
    TEST_ASSERT_NOT_NULL(testHandler);

    // Enqueue more than capacity in a tight loop (all fresh, none expire).
    const int sent = QUEUE_CAPACITY + 20;
    for (int i = 0; i < sent; i++) {
        std::string msg = "FA" + std::to_string(i) + ";";
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

    // Ring keeps at most QUEUE_CAPACITY (64), dropping the oldest on overflow.
    TEST_ASSERT_EQUAL(QUEUE_CAPACITY, messageCount);
    ESP_LOGI(TAG, "Queue capacity correctly capped at %d messages", messageCount);
}

// Overflow drops the OLDEST frame and is counted in QueueStats.overflowDrops.
void test_overflow_drops_counted_in_stats(void) {
    TEST_ASSERT_NOT_NULL(testHandler);

    // Clear any counters accumulated during setup.
    testHandler->resetQueueStats();

    const int overflow = 12;
    const int sent = QUEUE_CAPACITY + overflow;
    for (int i = 0; i < sent; i++) {
        std::string msg = "FA" + std::to_string(i) + ";";
        testHandler->processReceivedData(
            reinterpret_cast<const uint8_t*>(msg.c_str()),
            msg.length()
        );
    }

    const auto stats = testHandler->resetQueueStats();
    // Exactly `overflow` frames were dropped to make room; depth is capped at 64.
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(overflow), stats.overflowDrops);
    TEST_ASSERT_EQUAL(static_cast<size_t>(QUEUE_CAPACITY), stats.currentDepth);
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(QUEUE_CAPACITY), stats.highWatermark);
    ESP_LOGI(TAG, "Overflow drops correctly counted: %lu", (unsigned long)stats.overflowDrops);
}

// H1 regression: getMessageView() copies the frame into a per-instance scratch
// buffer, so the returned view must stay valid even after the producer overwrites
// the ring slot it originally occupied. The view is only invalidated by the NEXT
// getMessageView() call, not by further enqueues.
void test_view_survives_slot_overwrite(void) {
    TEST_ASSERT_NOT_NULL(testHandler);

    const std::string first = "FA14074000;";
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(first.c_str()),
        first.length()
    );

    auto [status, view] = testHandler->getMessageView();
    TEST_ASSERT_EQUAL(ESP_OK, status);
    TEST_ASSERT_EQUAL(first.length(), view.length());
    TEST_ASSERT_EQUAL_STRING(first.c_str(), std::string(view).c_str());

    // Flood the ring with more than a full lap of fresh frames so the slot that
    // originally held `first` is definitely overwritten by the producer.
    for (int i = 0; i < QUEUE_CAPACITY + 20; i++) {
        std::string msg = "FB" + std::to_string(i) + ";";
        testHandler->processReceivedData(
            reinterpret_cast<const uint8_t*>(msg.c_str()),
            msg.length()
        );
    }

    // The held view must still reflect the original frame (copied into scratch).
    TEST_ASSERT_EQUAL(first.length(), view.length());
    TEST_ASSERT_EQUAL_STRING(first.c_str(), std::string(view).c_str());
    ESP_LOGI(TAG, "Held view survived slot overwrite: %s", std::string(view).c_str());
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

// Fix 12 regression: a frame that overflows the 64-byte accumulator without a
// terminator in one chunk sets m_discardUntilTerminator, so every byte of the
// next chunk (the garbage remainder of that oversized frame) is skipped until
// the next ';' resyncs - it must never be parsed as a fresh, valid command.
void test_oversized_frame_suffix_not_parsed_as_command(void) {
    TEST_ASSERT_NOT_NULL(testHandler);

    // Overflow the accumulator: 64 bytes, no terminator in this chunk.
    const std::string overflow(SerialHandler::MAX_MESSAGE_LENGTH, 'A');
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(overflow.c_str()),
        overflow.length()
    );
    TEST_ASSERT_FALSE(testHandler->hasMessage());

    // The garbage suffix of the discarded frame must not be parsed as TX1.
    const std::string suffix = "TX1;";
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(suffix.c_str()),
        suffix.length()
    );
    TEST_ASSERT_FALSE(testHandler->hasMessage());

    // A real frame arriving after the discarding ';' must parse normally.
    const std::string realFrame = "FA00014074000;";
    testHandler->processReceivedData(
        reinterpret_cast<const uint8_t*>(realFrame.c_str()),
        realFrame.length()
    );
    TEST_ASSERT_TRUE(testHandler->hasMessage());
    auto [status, message] = testHandler->getMessage();
    TEST_ASSERT_EQUAL(ESP_OK, status);
    TEST_ASSERT_EQUAL_STRING(realFrame.c_str(), message.c_str());
    TEST_ASSERT_FALSE(testHandler->hasMessage());
    ESP_LOGI(TAG, "Oversized frame suffix correctly discarded, real frame preserved: %s", message.c_str());
}

// Unity's global setUp()/tearDown() are no-ops here, so each test explicitly
// brackets itself with the queue fixture to get a fresh handler + empty queue.
#define RUN_QUEUE_TEST(fn) \
    do { setUp_SerialHandlerQueue(); RUN_TEST(fn); tearDown_SerialHandlerQueue(); } while (0)

void run_serial_handler_queue_tests(void) {
    RUN_QUEUE_TEST(test_queue_capacity_caps_at_64);
    RUN_QUEUE_TEST(test_overflow_drops_counted_in_stats);
    RUN_QUEUE_TEST(test_view_survives_slot_overwrite);
    RUN_QUEUE_TEST(test_message_timeout_expiry);
    RUN_QUEUE_TEST(test_fresh_messages_not_cleared);
    RUN_QUEUE_TEST(test_mixed_age_message_clearing);
    RUN_QUEUE_TEST(test_high_traffic_queue_behavior);
    RUN_QUEUE_TEST(test_oversized_frame_suffix_not_parsed_as_command);
}

} // extern "C"