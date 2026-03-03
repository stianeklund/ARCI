#include <string>
#include <vector>
#include <array>
#include <charconv>
#include "../../SerialHandler/include/SerialHandler.h"
#include "../include/RadioManager.h"
#include "esp_log.h"
#include "unity.h"
#include "test_hooks.h"
#include "../../SerialHandler/mock/include/MockSerialHandler.h"
#include "include/CATTestUtils.h"
#include <cstring>

[[maybe_unused]] static auto TAG = "RADIOMANAGER_CAT_TEST";

namespace radio {
    class TestRadioManager : public RadioManager {
    public:
        TestRadioManager(MockSerialHandler &radioSerial, MockSerialHandler &usbSerial)
            : RadioManager(radioSerial, usbSerial),
              m_mockRadioSerial(radioSerial),
              m_mockUsbSerial(usbSerial) {
        }

        void resetMockSerials() const {
            m_mockRadioSerial.sentMessages.clear();
            m_mockRadioSerial.clearReceivedMessages();
            m_mockUsbSerial.sentMessages.clear();
            m_mockUsbSerial.clearReceivedMessages();
        }

        MockSerialHandler &m_mockRadioSerial;
        MockSerialHandler &m_mockUsbSerial;
    };
}

using radio::TestRadioManager;

namespace {
    MockSerialHandler mockRadioSerial;
    MockSerialHandler mockUsbSerial;
    TestRadioManager *testRadioManager = nullptr;
    bool isInitialized = false;

    void initializeSharedRadioManager() {
        if (!isInitialized) {
            // Note: mockRadioSerial and mockUsbSerial are already constructed as static objects
            // Clear any previous state instead of reassigning
            mockRadioSerial.clearSentMessages();
            mockRadioSerial.clearReceivedMessages();
            mockUsbSerial.clearSentMessages();
            mockUsbSerial.clearReceivedMessages();
            testRadioManager = new TestRadioManager(mockRadioSerial, mockUsbSerial);

            // Start RadioManager tasks (required after constructor refactor)
            esp_err_t ret = testRadioManager->startTasks();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start RadioManager tasks");
            }

            isInitialized = true;
            ESP_LOGI(TAG, "Shared RadioManager initialized and tasks started");
        }
    }

    void setUpTestRadioManager() {
        initializeSharedRadioManager();
        // Reset state between tests without recreating objects
        testRadioManager->resetMockSerials();
        testRadioManager->clearCommandCache();
    }

    void tearDownTestRadioManager() {
        // Don't delete - just reset for next test
        if (testRadioManager) {
            testRadioManager->resetMockSerials();
        }
    }

    void cleanupSharedRadioManager() {
        if (testRadioManager) {
            delete testRadioManager;
            testRadioManager = nullptr;
            isInitialized = false;
        }
    }

    [[maybe_unused]] void printDumpQueue() {
        ESP_LOGI("MockSerial", "=== USB sentMessages ===");
        for (const auto &msg: mockUsbSerial.sentMessages) {
            ESP_LOGI("MockSerial", "USB: %s", msg.c_str());
        }

        ESP_LOGI("MockSerial", "=== Radio sentMessages ===");
        for (const auto &msg: mockRadioSerial.sentMessages) {
            ESP_LOGI("MockSerial", "Radio: %s", msg.c_str());
        }
    }

    void test_parseLocalRequest_PS() {
        setUpTestRadioManager();

        // Check initial power state (should be off by default)
        TEST_ASSERT_EQUAL(0, testRadioManager->getPowerState());

        // Simulate radio response PS1; (power on)
        testRadioManager->getRemoteCATHandler().parseMessage("PS1;");
        TEST_ASSERT_EQUAL(1, testRadioManager->getPowerState());

        // Clear the cache to ensure PS query goes to radio instead of using cached response
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();

        // Test local PS query - should send query to radio since cache is cleared
        testRadioManager->getLocalCATHandler().parseMessage("PS;");
        
        // Verify the query was sent to radio
        TEST_ASSERT_EQUAL(1, mockRadioSerial.sentMessages.size());
        TEST_ASSERT_EQUAL_STRING("PS;", mockRadioSerial.sentMessages.back().c_str());

        // Simulate radio response PS0; (power off)
        testRadioManager->getRemoteCATHandler().parseMessage("PS0;");
        TEST_ASSERT_EQUAL(0, testRadioManager->getPowerState());
        tearDownTestRadioManager();
    }

    void test_parseRemoteResponse_RX() {
        setUpTestRadioManager();
        testRadioManager->getRemoteCATHandler().parseMessage("RX;");
        // Verify the response was handled (exact behavior depends on implementation)
        tearDownTestRadioManager();
    }

    void test_parseRemoteResponse_TX() {
        setUpTestRadioManager();
        testRadioManager->getRemoteCATHandler().parseMessage("TX;");
        tearDownTestRadioManager();
        // Verify the response was handled (exact behavior depends on implementation)
    }

    void test_parseLocalRequest_FA() {
        setUpTestRadioManager();

        // Clear cache and serial messages for clean test state
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();

        // Test FA set command
        testRadioManager->getLocalCATHandler().parseMessage("FA00014150000;");

        // Should send to radio
        TEST_ASSERT_FALSE(mockRadioSerial.sentMessages.empty());
        
        // Check that the FA set command was sent (may not be the last message)
        bool foundFACommand = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg == "FA00014150000;") {
                foundFACommand = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundFACommand, "FA00014150000; command not found in sent messages");

        // Simulate radio response
        testRadioManager->getRemoteCATHandler().parseMessage("FA00014150000;");

        // Check frequency was updated in state (avoid Unity 64-bit issues)
        constexpr uint64_t expectedFreq = 14150000ULL;
        const uint64_t actualFreq = testRadioManager->getVfoAFrequency();
        TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(expectedFreq), static_cast<uint32_t>(actualFreq));
        tearDownTestRadioManager();
    }

    void test_parseLocalRequest_MD() {
        setUpTestRadioManager();

        // Test MD set command
        testRadioManager->getLocalCATHandler().parseMessage("MD2;");

        // Should send to radio
        TEST_ASSERT_FALSE(mockRadioSerial.sentMessages.empty());
        
        // Check that the MD set command was sent (may not be the last message)
        bool foundMDCommand = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg == "MD2;") {
                foundMDCommand = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundMDCommand, "MD2; command not found in sent messages");

        // Simulate radio response
        testRadioManager->getRemoteCATHandler().parseMessage("MD2;");

        // Check mode was updated
        TEST_ASSERT_EQUAL(2, testRadioManager->getCurrentMode());
        tearDownTestRadioManager();
    }

    void test_parseLocalRequest_AI() {
        setUpTestRadioManager();

        // Test AI query
        testRadioManager->getLocalCATHandler().parseMessage("AI;");

        // AI queries are handled locally and do NOT get forwarded to radio
        // This is correct behavior - AI mode is managed independently for USB/Display/Radio
        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.empty());
        
        // Should respond to USB instead
        TEST_ASSERT_FALSE(mockUsbSerial.sentMessages.empty());
        TEST_ASSERT_EQUAL_STRING("AI0;", mockUsbSerial.sentMessages.back().c_str());
        tearDownTestRadioManager();
    }

    void test_IF_command_parsing_and_building() {
        setUpTestRadioManager();
        
        // Prime the IF cache with a basic response
        const std::string basicIFResponse = "IF00014150000      000000022020000000;";
        testRadioManager->getRemoteCATHandler().parseMessage(basicIFResponse);
        mockRadioSerial.sentMessages.clear(); // Clear the setup messages

        // Test IF query - should respond from cache, NOT send to radio
        testRadioManager->getLocalCATHandler().parseMessage("IF;");

        // Should NOT send to radio (responds from cache)
        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.empty());

        // Should respond to USB with cached IF data
        TEST_ASSERT_FALSE(mockUsbSerial.sentMessages.empty());

        // Simulate complex IF response from radio
        const std::string ifResponse = "IF00028360000      000000014020010080;";

        // Clear cache and simulate USB query for IF to establish query tracking
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getLocalCATHandler().parseMessage("IF;");
        
        // Clear USB messages to isolate the radio response forwarding
        mockUsbSerial.sentMessages.clear();

        // Parse radio IF response - this should forward to USB
        testRadioManager->getRemoteCATHandler().parseMessage(ifResponse);

        // Verify the radio IF response was forwarded to USB
        TEST_ASSERT_FALSE(mockUsbSerial.sentMessages.empty());
        TEST_ASSERT_EQUAL_STRING(ifResponse.c_str(), mockUsbSerial.sentMessages.back().c_str());
        tearDownTestRadioManager();
    }

    void test_transverter_functions() {
        setUpTestRadioManager();

        // Test transverter state tracking
        testRadioManager->getRemoteCATHandler().parseMessage("EX05600001;");

        // Check transverter state was updated
        TEST_ASSERT_TRUE(testRadioManager->getState().transverter);

        // Test turning off transverter
        testRadioManager->getRemoteCATHandler().parseMessage("EX05600000;");
        TEST_ASSERT_FALSE(testRadioManager->getState().transverter);
        tearDownTestRadioManager();
    }

    void test_data_mode_functions() {
        setUpTestRadioManager();

        // Test data mode setting
        testRadioManager->setDataMode(1);
        TEST_ASSERT_EQUAL(1, testRadioManager->getDataMode());

        testRadioManager->setDataMode(0);
        TEST_ASSERT_EQUAL(0, testRadioManager->getDataMode());
        tearDownTestRadioManager();
    }

    void test_split_functions() {
        setUpTestRadioManager();

        // Test split enable
        testRadioManager->enableSplit(true);

        // Should send VV; FR0; FT1; commands
        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.size() >= 3);

        // Test split disable
        testRadioManager->disableSplit();

        // Should send additional commands
        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.size() >= 5);
        tearDownTestRadioManager();
    }

    void test_gain_and_shift_functions() {
        setUpTestRadioManager();

        // Test AF gain setting
        testRadioManager->setAfGain(50);
        TEST_ASSERT_FALSE(mockRadioSerial.sentMessages.empty());

        setUpTestRadioManager();

        // Test RF gain setting
        testRadioManager->setRfGain(75);
        TEST_ASSERT_FALSE(mockRadioSerial.sentMessages.empty());
        tearDownTestRadioManager();
    }

    void test_power_state_functions() {
        setUpTestRadioManager();

        // Simulate power on response
        testRadioManager->getRemoteCATHandler().parseMessage("PS1;");
        TEST_ASSERT_EQUAL(1, testRadioManager->getOnOffState());
        TEST_ASSERT_EQUAL(radio::PowerState::On, testRadioManager->getPowerState());

        // Simulate power off response
        testRadioManager->getRemoteCATHandler().parseMessage("PS0;");
        TEST_ASSERT_EQUAL(0, testRadioManager->getOnOffState());
        TEST_ASSERT_EQUAL(radio::PowerState::Off, testRadioManager->getPowerState());
        tearDownTestRadioManager();
    }

    void test_utility_functions() {
        // Test array conversion utilities
        constexpr std::array<char, 12> freqArray = {'1', '4', '1', '5', '0', '0', '0', '0', '\0'};
        const uint64_t freq = test_utils::arrayToUint64(freqArray);
        // Use compatible assertion for 64-bit values to avoid Unity 64-bit support issues
        TEST_ASSERT_EQUAL_UINT32(14150000UL, static_cast<uint32_t>(freq));

        constexpr std::array<char, 9> uint8Array3 = {'0', '5', '\0'};
        const uint8_t val3 = test_utils::arrayToUint8(uint8Array3);
        TEST_ASSERT_EQUAL(5, val3);
        tearDownTestRadioManager();
    }

    void test_forward_EX_query_response() {
        setUpTestRadioManager();

        // Send local EX query
        testRadioManager->getLocalCATHandler().parseMessage("EX0060000;");

        // Should forward to radio
        TEST_ASSERT_FALSE(mockRadioSerial.sentMessages.empty());
        TEST_ASSERT_EQUAL_STRING("EX0060000;", mockRadioSerial.sentMessages.back().c_str());

        // Simulate radio response
        testRadioManager->getRemoteCATHandler().parseMessage("EX006000005;");

        // Should forward to USB - search through all messages
        TEST_ASSERT_FALSE(mockUsbSerial.sentMessages.empty());
        bool foundEXResponse = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg == "EX006000005;") {
                foundEXResponse = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundEXResponse, "Expected EX006000005; in USB messages");
        tearDownTestRadioManager();
    }

    void test_transverter_state_transitions() {
        setUpTestRadioManager();

        // Test transverter enable
        testRadioManager->getRemoteCATHandler().parseMessage("EX05600001;");
        TEST_ASSERT_EQUAL(1, testRadioManager->getState().transverter ? 1 : 0);

        // Test transverter disable
        testRadioManager->getRemoteCATHandler().parseMessage("EX05600000;");
        TEST_ASSERT_EQUAL(0, testRadioManager->getState().transverter ? 1 : 0);
        tearDownTestRadioManager();
    }

    // ============== COMPREHENSIVE CAT COMMAND TESTS ==============

    void test_AC_antenna_tuner_commands() {
        setUpTestRadioManager();

        // Prime cache with a simulated radio response first
        testRadioManager->getRemoteCATHandler().parseMessage("AC111;");
        mockRadioSerial.sentMessages.clear(); // Clear setup messages
        
        // Test AC read query - should use cached data, not query radio
        testRadioManager->getLocalCATHandler().parseMessage("AC;");

        // Should NOT send query to radio (cache is fresh)
        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.empty());
        // Should respond with cached data to USB
        TEST_ASSERT_FALSE(mockUsbSerial.sentMessages.empty());

        // Test AC set command
        testRadioManager->getLocalCATHandler().parseMessage("AC111;");
        TEST_ASSERT_EQUAL_STRING("AC111;", mockRadioSerial.sentMessages.back().c_str());

        // Simulate radio response
        testRadioManager->getRemoteCATHandler().parseMessage("AC111;");
        TEST_ASSERT_FALSE(mockUsbSerial.sentMessages.empty());
        tearDownTestRadioManager();
    }

    void test_AG_AF_gain_commands() {
        setUpTestRadioManager();

        // Prime cache with a simulated radio response first
        testRadioManager->getRemoteCATHandler().parseMessage("AG0100;");
        mockRadioSerial.sentMessages.clear(); // Clear setup messages

        // Test AG read - should respond from cache, not send to radio
        testRadioManager->getLocalCATHandler().parseMessage("AG;");
        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.empty());
        // Should respond with cached data to USB
        TEST_ASSERT_FALSE(mockUsbSerial.sentMessages.empty());

        // Test AG set with various values
        testRadioManager->getLocalCATHandler().parseMessage("AG0050;");
        test_utils::CATTestHelper::assertForwardedToRadio(mockRadioSerial, "AG0050;");

        testRadioManager->getLocalCATHandler().parseMessage("AG0255;");
        test_utils::CATTestHelper::assertForwardedToRadio(mockRadioSerial, "AG0255;");
        tearDownTestRadioManager();
    }

    void test_AI_auto_information_commands() {
        setUpTestRadioManager();

        // Reset AI coordination state to prevent debouncing from previous tests
        testRadioManager->getState().lastAiCoordinationTime.store(0);
        testRadioManager->getState().usbCdc0AiMode.store(0);
        testRadioManager->getState().usbCdc1AiMode.store(0);
        testRadioManager->getState().displayAiMode.store(2);  // Default display mode

        // AI mode coordination: radio gets max(usbCdc0, usbCdc1, display) where display defaults to 2
        // So AI0 from USB results in coordinated mode = max(0, 0, 2) = 2 sent to radio
        testRadioManager->getLocalCATHandler().parseMessage("AI0;");
        test_utils::CATTestHelper::assertForwardedToRadio(mockRadioSerial, "AI2;"); // Coordinated mode

        mockRadioSerial.clearSentMessages();

        // Reset debounce timer to allow immediate second AI command (debounce interval is 200ms)
        testRadioManager->getState().lastAiCoordinationTime.store(0);

        testRadioManager->getLocalCATHandler().parseMessage("AI2;");
        test_utils::CATTestHelper::assertForwardedToRadio(mockRadioSerial, "AI2;");

        // Simulate radio response
        testRadioManager->getRemoteCATHandler().parseMessage("AI2;");
        tearDownTestRadioManager();
    }

    void test_BC_beat_cancel_commands() {
        setUpTestRadioManager();

        // Test BC read and set
        testRadioManager->getLocalCATHandler().parseMessage("BC;");
        testRadioManager->getLocalCATHandler().parseMessage("BC1;");
        test_utils::CATTestHelper::assertForwardedToRadio(mockRadioSerial, "BC1;");
        tearDownTestRadioManager();
    }

    void test_BD_band_commands() {
        setUpTestRadioManager();

        // Test band selection
        testRadioManager->getLocalCATHandler().parseMessage("BD03;"); // 40m
        TEST_ASSERT_EQUAL_STRING("BD03;", mockRadioSerial.sentMessages.back().c_str());

        testRadioManager->getLocalCATHandler().parseMessage("BD05;"); // 20m
        TEST_ASSERT_EQUAL_STRING("BD05;", mockRadioSerial.sentMessages.back().c_str());
        tearDownTestRadioManager();
    }

    void test_BU_band_up_down_commands() {
        setUpTestRadioManager();

        // Test BU (Band Up) with valid band parameters per TS-590SG specification
        testRadioManager->getLocalCATHandler().parseMessage("BU05;"); // Band up to 20m
        TEST_ASSERT_FALSE(mockRadioSerial.sentMessages.empty());
        TEST_ASSERT_EQUAL_STRING("BU05;", mockRadioSerial.sentMessages.back().c_str());

        testRadioManager->getLocalCATHandler().parseMessage("BU03;"); // Band up to 40m
        TEST_ASSERT_EQUAL_STRING("BU03;", mockRadioSerial.sentMessages.back().c_str());

        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.size() >= 2);
        tearDownTestRadioManager();
    }

    void test_BY_busy_commands() {
        setUpTestRadioManager();

        // Test busy status
        testRadioManager->getLocalCATHandler().parseMessage("BY;");

        // Simulate radio busy response
        testRadioManager->getRemoteCATHandler().parseMessage("BY1;");
        tearDownTestRadioManager();
    }

    void test_CA_CW_break_in_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("CA;");
        testRadioManager->getLocalCATHandler().parseMessage("CA1;");
        TEST_ASSERT_EQUAL_STRING("CA1;", mockRadioSerial.sentMessages.back().c_str());
        tearDownTestRadioManager();
    }

    void test_CG_carrier_gain_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("CG000;");
        testRadioManager->getLocalCATHandler().parseMessage("CG100;");
        TEST_ASSERT_EQUAL_STRING("CG100;", mockRadioSerial.sentMessages.back().c_str());
        tearDownTestRadioManager();
    }

    void test_CT_CTCSS_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("CT;");
        testRadioManager->getLocalCATHandler().parseMessage("CT1;");

        tearDownTestRadioManager();
    }

    void test_DA_data_mode_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("DA;");
        testRadioManager->getLocalCATHandler().parseMessage("DA1;");

        // Simulate response
        testRadioManager->getRemoteCATHandler().parseMessage("DA1;");
        TEST_ASSERT_EQUAL(1, testRadioManager->getDataMode());

        tearDownTestRadioManager();
    }

    void test_DN_microphone_down_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("DN;");
        testRadioManager->getLocalCATHandler().parseMessage("DN12;");
        tearDownTestRadioManager();
    }

    void test_FB_VFO_B_frequency_commands() {
        setUpTestRadioManager();
        // Test FB read
        testRadioManager->getLocalCATHandler().parseMessage("FB;");

        // Test FB set
        testRadioManager->getLocalCATHandler().parseMessage("FB00007100000;");
        test_utils::CATTestHelper::assertForwardedToRadio(mockRadioSerial, "FB00007100000;");

        // Simulate radio response
        testRadioManager->getRemoteCATHandler().parseMessage("FB00007100000;");
        TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(7100000ULL),
                                 static_cast<uint32_t>(testRadioManager->getVfoBFrequency()));
        tearDownTestRadioManager();
    }

    void test_FL_IF_filter_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("FL;");
        testRadioManager->getLocalCATHandler().parseMessage("FL1;");
        testRadioManager->getLocalCATHandler().parseMessage("FL2;");
        tearDownTestRadioManager();
    }

    void test_FR_function_receive_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("FR;");
        testRadioManager->getLocalCATHandler().parseMessage("FR0;");
        testRadioManager->getLocalCATHandler().parseMessage("FR1;");
        tearDownTestRadioManager();
    }

    void test_FS_fine_step_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("FS;");
        testRadioManager->getLocalCATHandler().parseMessage("FS1;");
        tearDownTestRadioManager();
    }

    void test_FT_function_transmit_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("FT;");
        testRadioManager->getLocalCATHandler().parseMessage("FT0;");
        testRadioManager->getLocalCATHandler().parseMessage("FT1;");
        tearDownTestRadioManager();
    }

    void test_FW_DSP_filter_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("FW;");
        testRadioManager->getLocalCATHandler().parseMessage("FW1500;");
        tearDownTestRadioManager();
    }

    void test_GT_AGC_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("GT;");
        testRadioManager->getLocalCATHandler().parseMessage("GT003;");
        tearDownTestRadioManager();
    }

    void test_ID_radio_identification_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("ID;");

        // Simulate radio response
        testRadioManager->getRemoteCATHandler().parseMessage("ID020;");
        tearDownTestRadioManager();
    }

    void test_IS_DSP_filter_shift_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("IS;");
        testRadioManager->getLocalCATHandler().parseMessage("IS 0300;");
        testRadioManager->getLocalCATHandler().parseMessage("IS 1000;");
        tearDownTestRadioManager();
    }

    void test_KS_keying_speed_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("KS;");
        testRadioManager->getLocalCATHandler().parseMessage("KS020;");
    }

    void test_KY_keying_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("KY TEST;");
        testRadioManager->getLocalCATHandler().parseMessage("KY1;");
        tearDownTestRadioManager();
    }

    void test_LK_lock_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("LK;");
        testRadioManager->getLocalCATHandler().parseMessage("LK1;");
        tearDownTestRadioManager();
    }

    void test_LM_load_message_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("LM;");
        testRadioManager->getLocalCATHandler().parseMessage("LM12;");
        tearDownTestRadioManager();
    }

    void test_MC_memory_channel_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("MC;");
        testRadioManager->getLocalCATHandler().parseMessage("MC001;");
        tearDownTestRadioManager();
    }

    void test_MG_microphone_gain_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("MG;");
        testRadioManager->getLocalCATHandler().parseMessage("MG050;");
        tearDownTestRadioManager();
    }

    void test_ML_monitor_level_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("ML;");
        testRadioManager->getLocalCATHandler().parseMessage("ML005;");
        tearDownTestRadioManager();
    }

    void test_MR_memory_read_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("MR001;");

        // Simulate complex memory response
        testRadioManager->getRemoteCATHandler().parseMessage("MR00100014150000000000002000000020000000;");
        tearDownTestRadioManager();
    }

    void test_MW_memory_write_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("MW00100014150000000000002000000020000000;");
        tearDownTestRadioManager();
    }

    void test_NB_noise_blanker_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("NB;");
        testRadioManager->getLocalCATHandler().parseMessage("NB1;");
        testRadioManager->getLocalCATHandler().parseMessage("NB2;");
        testRadioManager->getLocalCATHandler().parseMessage("NB3;");
        tearDownTestRadioManager();
    }

    void test_NL_noise_blanker_level_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("NL;");
        testRadioManager->getLocalCATHandler().parseMessage("NL010;");
        tearDownTestRadioManager();
    }

    void test_NR_noise_reduction_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("NR;");
        testRadioManager->getLocalCATHandler().parseMessage("NR1;");
        testRadioManager->getLocalCATHandler().parseMessage("NR2;");
        tearDownTestRadioManager();
    }

    void test_NT_notch_commands() {
        setUpTestRadioManager();

        // Test query
        testRadioManager->getLocalCATHandler().parseMessage("NT;");

        // Test SET with mode only (P1) - Auto notch
        testRadioManager->getLocalCATHandler().parseMessage("NT10;");
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, testRadioManager->getState().notchFilterMode,
            "NT10; should set mode to 1 (Auto)");
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, testRadioManager->getState().notchFilterBandwidth,
            "NT10; should set bandwidth to 0");

        // Test SET Manual Normal (mode=2, bw=0)
        testRadioManager->getLocalCATHandler().parseMessage("NT20;");
        TEST_ASSERT_EQUAL_INT_MESSAGE(2, testRadioManager->getState().notchFilterMode,
            "NT20; should set mode to 2 (Manual)");
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, testRadioManager->getState().notchFilterBandwidth,
            "NT20; should set bandwidth to 0 (Normal)");

        // CRITICAL: Test SET Manual Wide (mode=2, bw=1) - this was the bug!
        testRadioManager->getLocalCATHandler().parseMessage("NT21;");
        TEST_ASSERT_EQUAL_INT_MESSAGE(2, testRadioManager->getState().notchFilterMode,
            "NT21; should set mode to 2 (Manual)");
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, testRadioManager->getState().notchFilterBandwidth,
            "NT21; should set bandwidth to 1 (Wide) - P2 must be parsed as separate param!");

        // Test SET OFF
        testRadioManager->getLocalCATHandler().parseMessage("NT00;");
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, testRadioManager->getState().notchFilterMode,
            "NT00; should set mode to 0 (OFF)");
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, testRadioManager->getState().notchFilterBandwidth,
            "NT00; should set bandwidth to 0");

        tearDownTestRadioManager();
    }

    void test_PA_preamp_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("PA;");
        testRadioManager->getLocalCATHandler().parseMessage("PA0;");
        testRadioManager->getLocalCATHandler().parseMessage("PA1;");
        tearDownTestRadioManager();
    }

    void test_PB_passband_tuning_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("PB;");
        testRadioManager->getLocalCATHandler().parseMessage("PB1;");
        testRadioManager->getLocalCATHandler().parseMessage("PB2;");
        testRadioManager->getLocalCATHandler().parseMessage("PB3;");
        testRadioManager->getLocalCATHandler().parseMessage("PB4;");
        testRadioManager->getLocalCATHandler().parseMessage("PB5;");
        tearDownTestRadioManager();
    }

    void test_PC_power_control_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("PC;");
        testRadioManager->getLocalCATHandler().parseMessage("PC050;");
        testRadioManager->getLocalCATHandler().parseMessage("PC100;");
        tearDownTestRadioManager();
    }

    void test_PL_speech_processor_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("PL;");
        testRadioManager->getLocalCATHandler().parseMessage("PL010050;"); // input 10, output 50
        tearDownTestRadioManager();
    }

    void test_PR_speech_processor_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("PR;");
        testRadioManager->getLocalCATHandler().parseMessage("PR1;");
        tearDownTestRadioManager();
    }

    void test_QI_QMB_inquiry_commands() {
        setUpTestRadioManager();
        testRadioManager->getLocalCATHandler().parseMessage("QI;");
        tearDownTestRadioManager();
    }

    void test_QR_quick_memory_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("QR;");
        testRadioManager->getLocalCATHandler().parseMessage("QR12;");
        tearDownTestRadioManager();
    }

    void test_RA_attenuator_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("RA;");
        testRadioManager->getLocalCATHandler().parseMessage("RA01;");
        tearDownTestRadioManager();
    }

    void test_RC_RIT_clear_commands() {
        setUpTestRadioManager();
        testRadioManager->getLocalCATHandler().parseMessage("RC;");
        tearDownTestRadioManager();
    }

    void test_RD_RIT_down_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("RD;");
        testRadioManager->getLocalCATHandler().parseMessage("RD0010;");
        tearDownTestRadioManager();
    }

    void test_RG_RF_gain_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("RG;");
        testRadioManager->getLocalCATHandler().parseMessage("RG100;");
        testRadioManager->getLocalCATHandler().parseMessage("RG255;");
        tearDownTestRadioManager();
    }

    void test_RL_noise_reduction_level_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("RL;");
        testRadioManager->getLocalCATHandler().parseMessage("RL05;");
        tearDownTestRadioManager();
    }

    void test_RM_read_meter_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("RM;");
        testRadioManager->getLocalCATHandler().parseMessage("RM0;"); // No selection
        testRadioManager->getLocalCATHandler().parseMessage("RM1;"); // SWR
        testRadioManager->getLocalCATHandler().parseMessage("RM2;"); //COMP
        testRadioManager->getLocalCATHandler().parseMessage("RM3;"); // ALC
        tearDownTestRadioManager();
    }

    void test_RT_RIT_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("RT;");
        testRadioManager->getLocalCATHandler().parseMessage("RT1;");
        tearDownTestRadioManager();
    }

    void test_RU_RIT_up_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("RU;");
        testRadioManager->getLocalCATHandler().parseMessage("RU0010;");
        tearDownTestRadioManager();
    }

    void test_SC_scan_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("SC;");
        testRadioManager->getLocalCATHandler().parseMessage("SC1;");
        tearDownTestRadioManager();
    }

    void test_SD_CW_break_in_delay_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("SD;");
        testRadioManager->getLocalCATHandler().parseMessage("SD0250;");
        tearDownTestRadioManager();
    }

    void test_SH_slope_high_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("SH;");
        testRadioManager->getLocalCATHandler().parseMessage("SH12;");
        tearDownTestRadioManager();
    }

    void test_SL_slope_low_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("SL;");
        testRadioManager->getLocalCATHandler().parseMessage("SL03;");
        tearDownTestRadioManager();
    }

    void test_SM_S_meter_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("SM0;");

        // Simulate S-meter response
        testRadioManager->getRemoteCATHandler().parseMessage("SM0050;");
        tearDownTestRadioManager();
    }

    void test_SQ_squelch_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("SQ;");
        testRadioManager->getLocalCATHandler().parseMessage("SQ050;");
        testRadioManager->getLocalCATHandler().parseMessage("SQ255;");
        tearDownTestRadioManager();
    }

    void test_TC_tone_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("TC;");
        testRadioManager->getLocalCATHandler().parseMessage("TC1;");
        tearDownTestRadioManager();
    }

    void test_TN_tone_number_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("TN;");
        testRadioManager->getLocalCATHandler().parseMessage("TN08;");
        tearDownTestRadioManager();
    }

    void test_TO_tone_offset_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("TO;");
        testRadioManager->getLocalCATHandler().parseMessage("TO1;");
        tearDownTestRadioManager();
    }

    void test_TS_TF_set_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("TS;");
        testRadioManager->getLocalCATHandler().parseMessage("TS2;");
        tearDownTestRadioManager();
    }

    void test_TX_transmit_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("TX;");
        testRadioManager->getLocalCATHandler().parseMessage("TX1;");

        // Test should not keep radio in TX mode
        testRadioManager->getLocalCATHandler().parseMessage("RX;");
        tearDownTestRadioManager();
    }

    void test_UP_microphone_up_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("UP;");
        testRadioManager->getLocalCATHandler().parseMessage("UP12;");
        tearDownTestRadioManager();
    }

    void test_VD_VOX_delay_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("VD;");
        testRadioManager->getLocalCATHandler().parseMessage("VD0500;");
        tearDownTestRadioManager();
    }

    void test_VG_VOX_gain_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("VG;");
        testRadioManager->getLocalCATHandler().parseMessage("VG050;");
        tearDownTestRadioManager();
    }

    void test_VV_VFO_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("VV;");
        tearDownTestRadioManager();
    }

    void test_VX_VOX_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("VX;");
        testRadioManager->getLocalCATHandler().parseMessage("VX1;");
        tearDownTestRadioManager();
    }

    void test_XT_XIT_commands() {
        setUpTestRadioManager();

        testRadioManager->getLocalCATHandler().parseMessage("XT;");
        testRadioManager->getLocalCATHandler().parseMessage("XT1;");
        tearDownTestRadioManager();
    }

    void test_UIDE_display_communication_enable() {
        setUpTestRadioManager();

        // Setup display serial
        MockSerialHandler mockDisplaySerial;
        testRadioManager->setDisplaySerial(&mockDisplaySerial);

        // Initially, display communication should be enabled (default state)
        TEST_ASSERT_TRUE(testRadioManager->getState().displayCommunicationEnabled.load());

        // Disable display communication via UIDE0
        testRadioManager->getLocalCATHandler().parseMessage("UIDE0;");
        TEST_ASSERT_FALSE(testRadioManager->getState().displayCommunicationEnabled.load());
        // UIDE command itself should still go through (uses sendToDisplayOnly)
        TEST_ASSERT_EQUAL_STRING("UIDE0;", mockDisplaySerial.sentMessages.back().c_str());

        // Clear display messages to test blocking
        mockDisplaySerial.clearSentMessages();

        // Try to send a message to display - should be blocked
        testRadioManager->sendToDisplay("FA00014150000;");
        // Display should NOT receive FA response (communication disabled)
        TEST_ASSERT_TRUE(mockDisplaySerial.sentMessages.empty());

        // Query UIDE - should respond even when disabled
        mockDisplaySerial.clearSentMessages();
        testRadioManager->getLocalCATHandler().parseMessage("UIDE;");
        TEST_ASSERT_EQUAL_STRING("UIDE0;", mockDisplaySerial.sentMessages.back().c_str());

        // Re-enable display communication via UIDE1
        mockDisplaySerial.clearSentMessages();
        testRadioManager->getLocalCATHandler().parseMessage("UIDE1;");
        TEST_ASSERT_TRUE(testRadioManager->getState().displayCommunicationEnabled.load());
        TEST_ASSERT_EQUAL_STRING("UIDE1;", mockDisplaySerial.sentMessages.back().c_str());

        // Clear and test that normal communication works again
        mockDisplaySerial.clearSentMessages();
        // Manually forward a response to display (simulating main task forwarding logic)
        testRadioManager->sendToDisplay("FB00007100000;");
        // Display SHOULD receive FB response now (communication enabled)
        TEST_ASSERT_FALSE(mockDisplaySerial.sentMessages.empty());

        // Clear display serial pointer to prevent dangling pointer in next test
        testRadioManager->setDisplaySerial(nullptr);

        tearDownTestRadioManager();
    }

    void test_command_error_handling() {
        setUpTestRadioManager();

        // Test invalid commands
        testRadioManager->getLocalCATHandler().parseMessage("XX;");
        testRadioManager->getLocalCATHandler().parseMessage("FA999999999999;"); // Invalid frequency
        testRadioManager->getLocalCATHandler().parseMessage("MD9;"); // Invalid mode

        // Radio should still respond to valid commands after errors
        testRadioManager->getLocalCATHandler().parseMessage("ID;");
        TEST_ASSERT_FALSE(mockRadioSerial.sentMessages.empty());
    }

    void test_command_sequencing() {
        setUpTestRadioManager();

        // Test rapid command sequence
        // testRadioManager->clearCommandCache(); // might need to clear cache first?
        testRadioManager->getLocalCATHandler().parseMessage("FA00014150000;");
        testRadioManager->getLocalCATHandler().parseMessage("MD2;");
        testRadioManager->getLocalCATHandler().parseMessage("AG100;");
        testRadioManager->getLocalCATHandler().parseMessage("RG200;");

        // All commands should be sent to radio
        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.size() >= 4);
        tearDownTestRadioManager();
    }

    void test_parameter_validation() {
        setUpTestRadioManager();

        // Test parameter boundary conditions
        testRadioManager->getLocalCATHandler().parseMessage("AG0000;"); // Min AF gain
        testRadioManager->getLocalCATHandler().parseMessage("AG0255;"); // Max AF gain
        testRadioManager->getLocalCATHandler().parseMessage("RG000;"); // Min RF gain
        testRadioManager->getLocalCATHandler().parseMessage("RG255;"); // Max RF gain
        testRadioManager->getLocalCATHandler().parseMessage("PC005;"); // Min power
        testRadioManager->getLocalCATHandler().parseMessage("PC100;"); // Max power

        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.size() >= 6);
        tearDownTestRadioManager();
    }

    void test_memory_operations() {
        setUpTestRadioManager();

        // Test memory channel operations
        testRadioManager->getLocalCATHandler().parseMessage("MC001;"); // Select memory 1
        testRadioManager->getLocalCATHandler().parseMessage("MR001;"); // Read memory 1

        // Simulate memory data response
        testRadioManager->getRemoteCATHandler().parseMessage("MR00100014150000000000002000000020000000;");

        // Test memory write
        testRadioManager->getLocalCATHandler().parseMessage("MW00200007100000000000001000000010000000;");

        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.size() >= 3);
        tearDownTestRadioManager();
    }

    void test_frequency_operations() {
        setUpTestRadioManager();

        // Test various frequency operations
        testRadioManager->getLocalCATHandler().parseMessage("FA00003500000;"); // 80m
        testRadioManager->getLocalCATHandler().parseMessage("FB00007100000;"); // 40m VFO B
        testRadioManager->getLocalCATHandler().parseMessage("FR1;"); // Select VFO B
        testRadioManager->getLocalCATHandler().parseMessage("FT1;"); // Set TX VFO to B

        // Simulate responses
        testRadioManager->getRemoteCATHandler().parseMessage("FA00003500000;");
        testRadioManager->getRemoteCATHandler().parseMessage("FB00007100000;");

        // Check frequencies (avoid Unity 64-bit issues)
        TEST_ASSERT_EQUAL_UINT32(3500000UL, static_cast<uint32_t>(testRadioManager->getVfoAFrequency()));
        TEST_ASSERT_EQUAL_UINT32(7100000UL, static_cast<uint32_t>(testRadioManager->getVfoBFrequency()));
        tearDownTestRadioManager();
    }

    void test_mode_operations() {
        setUpTestRadioManager();

        // Test all supported modes
        const std::vector<std::string> modes = {"MD1;", "MD2;", "MD3;", "MD4;", "MD5;", "MD6;", "MD7;", "MD8;", "MD9;"};

        for (const auto &mode: modes) {
            testRadioManager->getLocalCATHandler().parseMessage(mode);
            // Simulate radio response
            testRadioManager->getRemoteCATHandler().parseMessage(mode);
        }

        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.size() >= modes.size());
        tearDownTestRadioManager();
    }

    void test_filter_operations() {
        setUpTestRadioManager();

        // Test filter width operations
        testRadioManager->getLocalCATHandler().parseMessage("FW2400;"); // Wide filter
        testRadioManager->getLocalCATHandler().parseMessage("FW0500;"); // Narrow filter
        testRadioManager->getLocalCATHandler().parseMessage("SH12;"); // Slope high
        testRadioManager->getLocalCATHandler().parseMessage("SL03;"); // Slope low

        TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.size() >= 4);
        tearDownTestRadioManager();
    }

    // ============== PROGRAMMER TOOL COMPATIBILITY TESTS ==============
    // Tests for fixes enabling TS-590G Programmer and similar tools to work
    // through ARCI. See: TC format fix, AI0 query forwarding, unhandled ?; response.

    // Test: TC command uses correct "TC P1;" format (space before parameter)
    void test_TC_format_has_space() {
        setUpTestRadioManager();
        mockRadioSerial.sentMessages.clear();

        // Local SET: TC 1; should send "TC 1;" to radio (with space)
        testRadioManager->getLocalCATHandler().parseMessage("TC 1;");

        bool foundTC = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg.find("TC") != std::string::npos) {
                // Must contain a space between TC and digit
                TEST_ASSERT_EQUAL_STRING("TC 1;", msg.c_str());
                foundTC = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundTC, "Expected TC command to be sent to radio");

        // TC query should respond locally with space format
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getLocalCATHandler().parseMessage("TC;");
        bool foundTCResponse = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg.find("TC ") != std::string::npos) {
                foundTCResponse = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundTCResponse, "Expected TC response with space format");
        tearDownTestRadioManager();
    }

    // Test: AI0 mode still forwards responses to explicit queries
    void test_AI0_forwards_query_responses() {
        setUpTestRadioManager();

        // CDC0 AI mode defaults to 0 (which is what programmers set)
        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());

        // Clear cache so SC query goes to radio
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();

        // Programmer sends SC; query (source: UsbCdc0, AI mode 0)
        testRadioManager->getLocalCATHandler().parseMessage("SC;");

        // Should have been forwarded to radio
        bool sentToRadio = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg == "SC;") {
                sentToRadio = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(sentToRadio, "SC; should be forwarded to radio");

        // Simulate radio response
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getRemoteCATHandler().parseMessage("SC0;");

        // Response MUST be forwarded to USB even though AI mode is 0
        bool foundSCResponse = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg == "SC0;") {
                foundSCResponse = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundSCResponse,
            "SC0; response must be forwarded to CDC0 in AI0 mode when recently queried");
        tearDownTestRadioManager();
    }

    // Test: AI0 mode suppresses unsolicited responses (not queried)
    void test_AI0_suppresses_unsolicited_responses() {
        setUpTestRadioManager();

        // CDC0 AI mode defaults to 0
        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());

        mockUsbSerial.sentMessages.clear();

        // Radio sends unsolicited FA update (no prior query from CDC0)
        testRadioManager->getRemoteCATHandler().parseMessage("FA00014074000;");

        // Should NOT be forwarded to USB in AI0 mode (unsolicited)
        bool foundFA = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg.find("FA") != std::string::npos) {
                foundFA = true;
                break;
            }
        }
        TEST_ASSERT_FALSE_MESSAGE(foundFA,
            "Unsolicited FA should NOT be forwarded to CDC0 in AI0 mode");
        tearDownTestRadioManager();
    }

    // Test: Unhandled commands get ?; response back to originating interface
    void test_unhandled_command_returns_error() {
        setUpTestRadioManager();
        mockUsbSerial.sentMessages.clear();

        // Send a command with no handler (XX is not a real CAT command)
        testRadioManager->getLocalCATHandler().parseMessage("XX;");

        // Should receive ?; back on USB (like a real radio would)
        bool foundError = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg == "?;") {
                foundError = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundError,
            "Unhandled command should return ?; to originating interface");
        tearDownTestRadioManager();
    }

    // Test: Simulates a TS-590G Programmer startup sequence
    void test_programmer_startup_sequence() {
        setUpTestRadioManager();

        // CDC0 AI mode defaults to 0 (programmer behavior)
        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());

        // Step 1: Programmer sends TC 1; (terminal control on)
        testRadioManager->getLocalCATHandler().parseMessage("TC 1;");

        // Step 2: Programmer sends ID; (identification)
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getLocalCATHandler().parseMessage("ID;");
        bool foundID = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg.find("ID023") != std::string::npos) {
                foundID = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundID, "ID response should be returned locally");

        // Step 3: Programmer sends AI0; (disable auto-information)
        testRadioManager->getLocalCATHandler().parseMessage("AI0;");

        // Step 4: Programmer queries SC; — must get a response even in AI0
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getLocalCATHandler().parseMessage("SC;");

        // Simulate radio response
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getRemoteCATHandler().parseMessage("SC0;");
        bool foundSC = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg == "SC0;") {
                foundSC = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundSC,
            "Programmer must receive SC0; response after AI0 + SC; query");

        // Step 5: Programmer queries EX menu item — must get a response
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getLocalCATHandler().parseMessage("EX0060000;");

        // Simulate radio response
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getRemoteCATHandler().parseMessage("EX006000005;");
        bool foundEX = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg == "EX006000005;") {
                foundEX = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundEX,
            "Programmer must receive EX response after AI0 + EX query");

        tearDownTestRadioManager();
    }

    // Test: MR0000; is synthesized locally (not forwarded to radio)
    void test_MR_AI0_forwards_query_response() {
        setUpTestRadioManager();

        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();

        // Programmer sends MR0000; (memory read channel 0)
        testRadioManager->getLocalCATHandler().parseMessage("MR0000;");

        // MR should NOT be forwarded to radio (synthesized locally)
        bool sentToRadio = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg.find("MR0") != std::string::npos) {
                sentToRadio = true;
                break;
            }
        }
        TEST_ASSERT_FALSE_MESSAGE(sentToRadio, "MR0000; should be synthesized locally, not sent to radio");

        // Synthesized response MUST be sent to CDC0 immediately
        bool foundMRResponse = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg.find("MR0000") != std::string::npos) {
                foundMRResponse = true;
                // Verify response has correct spec format (42 chars for empty channel)
                TEST_ASSERT_TRUE_MESSAGE(msg.length() >= 42,
                    "MR response must have full spec format (>= 42 chars)");
                // Verify P1=0 (simplex), P2P3=000 (channel 0)
                TEST_ASSERT_EQUAL_MESSAGE('0', msg[2], "P1 should be '0' (simplex)");
                TEST_ASSERT_EQUAL_MESSAGE('0', msg[3], "P2 should be '0' (hundreds)");
                TEST_ASSERT_EQUAL_MESSAGE('0', msg[4], "P3[0] should be '0' (tens)");
                TEST_ASSERT_EQUAL_MESSAGE('0', msg[5], "P3[1] should be '0' (ones)");
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundMRResponse,
            "Synthesized MR response must be sent to CDC0 immediately");
        tearDownTestRadioManager();
    }

    // Test: EX0060000; (read query classified as Set) gets forwarded in AI0 mode
    void test_EX_parameterized_read_AI0() {
        setUpTestRadioManager();

        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();

        // Programmer sends EX0060000; (read menu 006, sidetone volume)
        testRadioManager->getLocalCATHandler().parseMessage("EX0060000;");

        // Should have been forwarded to radio
        bool sentToRadio = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg == "EX0060000;") {
                sentToRadio = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(sentToRadio, "EX0060000; must be forwarded to radio");

        // Simulate radio response
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getRemoteCATHandler().parseMessage("EX006000005;");

        // Response MUST be forwarded in AI0 mode
        bool foundEXResponse = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg == "EX006000005;") {
                foundEXResponse = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundEXResponse,
            "EX response must be forwarded to CDC0 in AI0 mode for parameterized read");
        tearDownTestRadioManager();
    }

    // Test: AG0; (parameterized read) gets query-tracked in AI0 mode
    void test_AG_parameterized_read_AI0() {
        setUpTestRadioManager();

        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();

        // Programmer sends AG0; (read AF gain — P1=0)
        testRadioManager->getLocalCATHandler().parseMessage("AG0;");

        // Should have been forwarded to radio
        bool sentToRadio = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg.find("AG") != std::string::npos) {
                sentToRadio = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(sentToRadio, "AG0; should be forwarded to radio");

        // Simulate radio response
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getRemoteCATHandler().parseMessage("AG0128;");

        bool foundAGResponse = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg.find("AG0") != std::string::npos) {
                foundAGResponse = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundAGResponse,
            "AG response must be forwarded to CDC0 in AI0 mode for parameterized read");
        tearDownTestRadioManager();
    }

    // Test: SU0; (scan group read) gets query-tracked in AI0 mode
    void test_SU_parameterized_read_AI0() {
        setUpTestRadioManager();

        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();

        // Programmer sends SU0; (read scan group 0)
        testRadioManager->getLocalCATHandler().parseMessage("SU0;");

        // Should have been forwarded to radio
        bool sentToRadio = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg.find("SU") != std::string::npos) {
                sentToRadio = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(sentToRadio, "SU0; should be forwarded to radio");

        // Simulate radio response (SU answer has many params)
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getRemoteCATHandler().parseMessage("SU000014000000000140740001000;");

        bool foundSUResponse = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg.find("SU0") != std::string::npos) {
                foundSUResponse = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundSUResponse,
            "SU response must be forwarded to CDC0 in AI0 mode for parameterized read");
        tearDownTestRadioManager();
    }

    // Test: SS00; (program slow scan read) gets query-tracked in AI0 mode
    void test_SS_parameterized_read_AI0() {
        setUpTestRadioManager();

        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();

        // Programmer sends SS00; (read program slow scan channel 0, lower freq)
        testRadioManager->getLocalCATHandler().parseMessage("SS00;");

        // Should have been forwarded to radio
        bool sentToRadio = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg.find("SS") != std::string::npos) {
                sentToRadio = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(sentToRadio, "SS00; should be forwarded to radio");

        // Simulate radio response
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getRemoteCATHandler().parseMessage("SS0000014000000;");

        bool foundSSResponse = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg.find("SS00") != std::string::npos) {
                foundSSResponse = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundSSResponse,
            "SS response must be forwarded to CDC0 in AI0 mode for parameterized read");
        tearDownTestRadioManager();
    }

    // Test: Radio ?; response to SC query is forwarded to CDC0
    void test_MR_error_response_forwarded_to_CDC0() {
        setUpTestRadioManager();

        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());
        testRadioManager->clearCommandCache();
        mockRadioSerial.sentMessages.clear();
        mockUsbSerial.sentMessages.clear();

        // Programmer sends SC; (scan query — forwarded to radio)
        testRadioManager->getLocalCATHandler().parseMessage("SC;");

        // Verify it was forwarded to radio
        bool sentToRadio = false;
        for (const auto& msg : mockRadioSerial.sentMessages) {
            if (msg.find("SC") != std::string::npos) {
                sentToRadio = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(sentToRadio, "SC; should be forwarded to radio");

        // Radio responds with ?; (e.g., unsupported in current state)
        mockUsbSerial.sentMessages.clear();
        testRadioManager->getRemoteCATHandler().parseMessage("?;");

        // ?; MUST be forwarded to CDC0 because SC was recently queried
        bool foundError = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg == "?;") {
                foundError = true;
                break;
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(foundError,
            "Radio ?; response must be forwarded to CDC0 when there is a pending query (SC)");
        tearDownTestRadioManager();
    }

    // Test: Unsolicited ?; from radio is NOT forwarded to CDC0 (no pending query)
    void test_unsolicited_error_suppressed() {
        setUpTestRadioManager();

        TEST_ASSERT_EQUAL(0, testRadioManager->getState().usbCdc0AiMode.load());
        testRadioManager->clearCommandCache();
        // Clear the query tracker so prior tests don't leave stale pending queries
        testRadioManager->getState().queryTracker.clear();
        mockUsbSerial.sentMessages.clear();

        // No query sent — just a stray ?; from the radio
        testRadioManager->getRemoteCATHandler().parseMessage("?;");

        // ?; should NOT appear on CDC0 (no pending query context)
        bool foundError = false;
        for (const auto& msg : mockUsbSerial.sentMessages) {
            if (msg == "?;") {
                foundError = true;
                break;
            }
        }
        TEST_ASSERT_FALSE_MESSAGE(foundError,
            "Unsolicited ?; should NOT be forwarded to CDC0 when no query is pending");
        tearDownTestRadioManager();
    }

    // Test runner function
    extern "C" void run_radiomanager_cat_tests(void) {
        // Original basic tests
        RUN_TEST(test_parseLocalRequest_PS);
        RUN_TEST(test_parseRemoteResponse_RX);
        RUN_TEST(test_parseRemoteResponse_TX);
        RUN_TEST(test_parseLocalRequest_FA);
        RUN_TEST(test_parseLocalRequest_MD);
        RUN_TEST(test_parseLocalRequest_AI);
        RUN_TEST(test_IF_command_parsing_and_building);
        RUN_TEST(test_transverter_functions);
        RUN_TEST(test_data_mode_functions);
        RUN_TEST(test_split_functions);
        RUN_TEST(test_gain_and_shift_functions);
        RUN_TEST(test_power_state_functions);
        RUN_TEST(test_utility_functions);
        RUN_TEST(test_forward_EX_query_response);
        RUN_TEST(test_transverter_state_transitions);

        RUN_TEST(test_AC_antenna_tuner_commands);
        RUN_TEST(test_AG_AF_gain_commands);
        RUN_TEST(test_AI_auto_information_commands);
        RUN_TEST(test_BC_beat_cancel_commands);
        RUN_TEST(test_BD_band_commands);
        RUN_TEST(test_BU_band_up_down_commands);
        RUN_TEST(test_BY_busy_commands);
        RUN_TEST(test_CA_CW_break_in_commands);
        RUN_TEST(test_CG_carrier_gain_commands);
        RUN_TEST(test_CT_CTCSS_commands);
        RUN_TEST(test_DA_data_mode_commands);
        RUN_TEST(test_DN_microphone_down_commands);
        RUN_TEST(test_FB_VFO_B_frequency_commands);
        RUN_TEST(test_FL_IF_filter_commands);
        RUN_TEST(test_FR_function_receive_commands);
        RUN_TEST(test_FS_fine_step_commands);
        RUN_TEST(test_FT_function_transmit_commands);
        RUN_TEST(test_FW_DSP_filter_commands);
        RUN_TEST(test_GT_AGC_commands);
        RUN_TEST(test_ID_radio_identification_commands);
        RUN_TEST(test_IS_DSP_filter_shift_commands);
        RUN_TEST(test_KS_keying_speed_commands);
        RUN_TEST(test_KY_keying_commands);
        RUN_TEST(test_LK_lock_commands);
        RUN_TEST(test_LM_load_message_commands);
        RUN_TEST(test_MC_memory_channel_commands);
        RUN_TEST(test_MG_microphone_gain_commands);
        RUN_TEST(test_ML_monitor_level_commands);
        RUN_TEST(test_MR_memory_read_commands);
        RUN_TEST(test_MW_memory_write_commands);
        RUN_TEST(test_NB_noise_blanker_commands);
        RUN_TEST(test_NL_noise_blanker_level_commands);
        RUN_TEST(test_NR_noise_reduction_commands);
        RUN_TEST(test_NT_notch_commands);
        RUN_TEST(test_PA_preamp_commands);
        RUN_TEST(test_PB_passband_tuning_commands);
        RUN_TEST(test_PC_power_control_commands);
        RUN_TEST(test_PL_speech_processor_commands);
        RUN_TEST(test_PR_speech_processor_commands);
        RUN_TEST(test_QI_QMB_inquiry_commands);
        RUN_TEST(test_QR_quick_memory_commands);
        RUN_TEST(test_RA_attenuator_commands);
        RUN_TEST(test_RC_RIT_clear_commands);
        RUN_TEST(test_RD_RIT_down_commands);
        RUN_TEST(test_RG_RF_gain_commands);
        RUN_TEST(test_RL_noise_reduction_level_commands);
        RUN_TEST(test_RM_read_meter_commands);
        RUN_TEST(test_RT_RIT_commands);
        RUN_TEST(test_RU_RIT_up_commands);
        RUN_TEST(test_SC_scan_commands);
        RUN_TEST(test_SD_CW_break_in_delay_commands);
        RUN_TEST(test_SH_slope_high_commands);
        RUN_TEST(test_SL_slope_low_commands);
        RUN_TEST(test_SM_S_meter_commands);
        RUN_TEST(test_SQ_squelch_commands);
        RUN_TEST(test_TC_tone_commands);
        RUN_TEST(test_TN_tone_number_commands);
        RUN_TEST(test_TO_tone_offset_commands);
        RUN_TEST(test_TS_TF_set_commands);
        RUN_TEST(test_TX_transmit_commands);
        RUN_TEST(test_UP_microphone_up_commands);
        RUN_TEST(test_VD_VOX_delay_commands);
        RUN_TEST(test_VG_VOX_gain_commands);
        RUN_TEST(test_VV_VFO_commands);
        RUN_TEST(test_VX_VOX_commands);
        RUN_TEST(test_XT_XIT_commands);
        RUN_TEST(test_UIDE_display_communication_enable);

        // Advanced test scenarios
        RUN_TEST(test_command_error_handling);
        RUN_TEST(test_command_sequencing);
        RUN_TEST(test_parameter_validation);
        RUN_TEST(test_memory_operations);
        RUN_TEST(test_frequency_operations);
        RUN_TEST(test_mode_operations);
        RUN_TEST(test_filter_operations);

        // Programmer tool compatibility tests
        RUN_TEST(test_TC_format_has_space);
        RUN_TEST(test_AI0_forwards_query_responses);
        RUN_TEST(test_AI0_suppresses_unsolicited_responses);
        RUN_TEST(test_unhandled_command_returns_error);
        RUN_TEST(test_programmer_startup_sequence);

        // Parameterized-read AI0 query tracking tests
        RUN_TEST(test_MR_AI0_forwards_query_response);
        RUN_TEST(test_EX_parameterized_read_AI0);
        RUN_TEST(test_AG_parameterized_read_AI0);
        RUN_TEST(test_SU_parameterized_read_AI0);
        RUN_TEST(test_SS_parameterized_read_AI0);

        // Radio ?; forwarding tests
        RUN_TEST(test_MR_error_response_forwarded_to_CDC0);
        RUN_TEST(test_unsolicited_error_suppressed);

        cleanupSharedRadioManager();
    }
}
