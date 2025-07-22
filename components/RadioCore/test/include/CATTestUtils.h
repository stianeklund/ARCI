#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "../../SerialHandler/mock/include/MockSerialHandler.h"
#include "unity.h"

namespace test_utils {

/**
 * @brief Convert a 12-character array to uint64_t
 * @param arr The character array containing numeric string
 * @return The converted value or 0 if conversion fails
 */
uint64_t arrayToUint64(const std::array<char, 12>& arr);

/**
 * @brief Convert a 12-character array to uint32_t
 * @param arr The character array containing numeric string
 * @return The converted value or 0 if conversion fails
 */
uint32_t arrayToUint32(const std::array<char, 12>& arr);

/**
 * @brief Convert a 9-character array to uint8_t
 * @param arr The character array containing numeric string
 * @return The converted value or 0 if conversion fails
 */
uint8_t arrayToUint8(const std::array<char, 9>& arr);

// ============== NEW TEST UTILITIES ==============

/**
 * @brief Test result structure for handler tests
 */
struct HandlerTestResult {
    bool forwardedToRadio = false;
    bool respondedFromCache = false;
    bool updatedState = false;
    std::string response;
    std::string radioMessage;
};

/**
 * @brief Helper class for CAT command testing
 */
class CATTestHelper {
public:
    /**
     * @brief Assert that a command was forwarded to the radio
     */
    static void assertForwardedToRadio(const MockSerialHandler& radioSerial, std::string_view expectedCmd) {
        TEST_ASSERT_FALSE_MESSAGE(radioSerial.sentMessages.empty(), "Expected message to radio but none sent");
        bool found = false;
        for (const auto& msg : radioSerial.sentMessages) {
            if (msg == expectedCmd) { found = true; break; }
        }
        if (!found) {
            std::string err = std::string("Radio message mismatch. Expected '") + std::string(expectedCmd) +
                              "' in sequence, but last was '" + radioSerial.sentMessages.back() + "'";
            TEST_FAIL_MESSAGE(err.c_str());
        }
    }
    
    /**
     * @brief Assert that a response was sent to USB
     */
    static void assertSentToUSB(const MockSerialHandler& usbSerial, std::string_view expectedResp) {
        TEST_ASSERT_FALSE_MESSAGE(usbSerial.sentMessages.empty(), "Expected USB response but none sent");
        TEST_ASSERT_EQUAL_STRING_MESSAGE(expectedResp.data(), usbSerial.sentMessages.back().c_str(),
                                          "USB response mismatch");
    }
    
    /**
     * @brief Assert that no message was sent to radio
     */
    static void assertNotForwardedToRadio(const MockSerialHandler& radioSerial) {
        TEST_ASSERT_TRUE_MESSAGE(radioSerial.sentMessages.empty(), "Expected no radio message but found one");
    }
    
    /**
     * @brief Assert that a cache response was used (no radio query)
     */
    static void assertCacheUsed(const MockSerialHandler& radioSerial, const MockSerialHandler& usbSerial) {
        TEST_ASSERT_TRUE_MESSAGE(radioSerial.sentMessages.empty(), "Cache should be used, but radio was queried");
        TEST_ASSERT_FALSE_MESSAGE(usbSerial.sentMessages.empty(), "Cache response should be sent to USB");
    }
    
    /**
     * @brief Clear both mock serial handlers
     */
    static void clearMockSerials(MockSerialHandler& radioSerial, MockSerialHandler& usbSerial) {
        radioSerial.sentMessages.clear();
        radioSerial.clearReceivedMessages();
        usbSerial.sentMessages.clear();
        usbSerial.clearReceivedMessages();
    }
    
    /**
     * @brief Get the count of messages sent to radio
     */
    static size_t getRadioMessageCount(const MockSerialHandler& radioSerial) {
        return radioSerial.sentMessages.size();
    }
    
    /**
     * @brief Get the count of messages sent to USB
     */
    static size_t getUSBMessageCount(const MockSerialHandler& usbSerial) {
        return usbSerial.sentMessages.size();
    }
};

// ============== TEST MACROS ==============

/**
 * @brief Macro to test a complete SET command flow
 * Tests: Local SET -> Forward to Radio -> Remote Response -> State Update
 */
#define TEST_CAT_SET_FLOW(mgr, cmd, params, mockRadio, mockUsb) \
    do { \
        size_t initialRadioCount = test_utils::CATTestHelper::getRadioMessageCount(mockRadio); \
        mgr.getLocalCATHandler().parseMessage(cmd + params + ";"); \
        test_utils::CATTestHelper::assertForwardedToRadio(mockRadio, cmd + params + ";"); \
        mgr.getRemoteCATHandler().parseMessage(cmd + params + ";"); \
    } while(0)

/**
 * @brief Macro to test a READ command with cache
 */
#define TEST_CAT_QUERY_CACHED(mgr, cmd, mockRadio, mockUsb) \
    do { \
        mgr.getLocalCATHandler().parseMessage(cmd + ";"); \
        test_utils::CATTestHelper::assertCacheUsed(mockRadio, mockUsb); \
    } while(0)

/**
 * @brief Macro to test a READ command that queries radio
 */
#define TEST_CAT_QUERY_RADIO(mgr, cmd, mockRadio) \
    do { \
        mgr.getLocalCATHandler().parseMessage(cmd + ";"); \
        test_utils::CATTestHelper::assertForwardedToRadio(mockRadio, cmd + ";"); \
    } while(0)

/**
 * @brief Macro to verify frequency value (handles 64-bit Unity issue)
 */
#define TEST_ASSERT_FREQUENCY_EQUAL(expected, actual) \
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(expected), static_cast<uint32_t>(actual))

} // namespace test_utils
