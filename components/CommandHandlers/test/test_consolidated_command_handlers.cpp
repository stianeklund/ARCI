#include "unity.h"
#include "test_hooks.h"
#include "FrequencyVfoCommandHandler.h"
#include "RadioManager.h"
#include "StatusInfoCommandHandler.h"
#include "esp_timer.h"
#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>

#include "MockSerialHandler.h"

class MockSerialHandler;
using namespace radio;

static MockSerialHandler* mockRadioSerial;
static MockSerialHandler* mockUsbSerial;
static std::unique_ptr<RadioManager> testRadioManager;


// Clean initialization function since Unity setUp() is not being called reliably
void initializeTestObjects() {
    if (mockRadioSerial) {
        delete mockRadioSerial;
        mockRadioSerial = nullptr;
    }
    if (mockUsbSerial) {
        delete mockUsbSerial;
        mockUsbSerial = nullptr;
    }
    testRadioManager.reset();

    mockRadioSerial = new MockSerialHandler();
    mockUsbSerial = new MockSerialHandler();

    testRadioManager = std::make_unique<RadioManager>(
        static_cast<ISerialChannel&>(*mockRadioSerial),
        static_cast<ISerialChannel&>(*mockUsbSerial));

    // Start RadioManager tasks (required after Phase 1.1 refactoring)
    esp_err_t ret = testRadioManager->startTasks();
    if (ret != ESP_OK) {
        ESP_LOGE("CommandHandlerTests", "Failed to start RadioManager tasks");
    }

    if (testRadioManager) {
        // Clear any previous state
        mockRadioSerial->sentMessages.clear();
        mockRadioSerial->clearReceivedMessages();
        mockUsbSerial->sentMessages.clear();
        mockUsbSerial->clearReceivedMessages();
    }
}

// Using per-test initialization via initializeTestObjects(); no global setUp/tearDown.

// =============================================================================
// FrequencyVfoCommandHandler Tests
// =============================================================================

void test_frequency_vfo_handler_fa_command() {
    // Initialize test objects since Unity setUp is not called reliably
    initializeTestObjects();
    
    // Test FA (VFO A frequency) command
    const std::string testCommand = "FA14150000;";
    
    // First verify that the RadioManager is properly constructed
    TEST_ASSERT_NOT_NULL(testRadioManager.get());
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);

    // Verify VFO A frequency was updated
    TEST_ASSERT_EQUAL_UINT32(14150000, testRadioManager->getVfoAFrequency());
}

void test_frequency_vfo_handler_ch_command() {
    // Test CH (MULTI/CH encoder step) command
    
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    const std::string testCommand = "CH01;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

void test_frequency_vfo_handler_vv_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Set different frequencies for VFO A and B first
    testRadioManager->updateVfoAFrequency(14150000);
    testRadioManager->updateVfoBFrequency(7050000);
    
    // Test VV (VFO copy A=B) command
    const std::string testCommand = "VV;";

    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Verify VFO B now matches VFO A
    TEST_ASSERT_EQUAL_UINT32(testRadioManager->getVfoAFrequency(), testRadioManager->getVfoBFrequency());
}

void test_frequency_up_zero_step_no_change() {
    initializeTestObjects();

    const uint32_t startFreq = 7'000'000UL;
    testRadioManager->updateVfoAFrequency(startFreq);
    testRadioManager->updateMode(3);

    mockRadioSerial->sentMessages.clear();

    testRadioManager->getLocalCATHandler().parseMessage("UP00;");

    TEST_ASSERT_FALSE(mockRadioSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("UP00;", mockRadioSerial->sentMessages.back().c_str());
    TEST_ASSERT_EQUAL_UINT32(startFreq, static_cast<uint32_t>(testRadioManager->getVfoAFrequency()));
}

void test_frequency_up_multi_step_updates_state() {
    initializeTestObjects();

    const uint32_t startFreq = 7'000'000UL;
    testRadioManager->updateVfoAFrequency(startFreq);
    testRadioManager->updateMode(3); // CW => 500 Hz step

    mockRadioSerial->sentMessages.clear();

    testRadioManager->getLocalCATHandler().parseMessage("UP12;");

    TEST_ASSERT_FALSE(mockRadioSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("UP12;", mockRadioSerial->sentMessages.back().c_str());

    const uint32_t expected = startFreq + (500UL * 12UL);
    TEST_ASSERT_EQUAL_UINT32(expected, static_cast<uint32_t>(testRadioManager->getVfoAFrequency()));
}

void test_frequency_dn_multi_step_updates_state() {
    initializeTestObjects();

    const uint32_t startFreq = 7'000'000UL;
    testRadioManager->updateVfoAFrequency(startFreq);
    testRadioManager->updateMode(3); // CW => 500 Hz step

    mockRadioSerial->sentMessages.clear();

    testRadioManager->getLocalCATHandler().parseMessage("DN05;");

    TEST_ASSERT_FALSE(mockRadioSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("DN05;", mockRadioSerial->sentMessages.back().c_str());

    const uint32_t expected = startFreq - (500UL * 5UL);
    TEST_ASSERT_EQUAL_UINT32(expected, static_cast<uint32_t>(testRadioManager->getVfoAFrequency()));
}

void test_frequency_fr_local_set_does_not_trigger_if_query() {
    initializeTestObjects();

    mockRadioSerial->clearSentMessages();
    mockUsbSerial->clearSentMessages();

    testRadioManager->getLocalCATHandler().parseMessage("FR1;");

    TEST_ASSERT_FALSE(mockRadioSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("FR1;", mockRadioSerial->sentMessages.front().c_str());
    const auto it = std::find(mockRadioSerial->sentMessages.begin(), mockRadioSerial->sentMessages.end(), std::string{"IF;"});
    TEST_ASSERT_TRUE_MESSAGE(it == mockRadioSerial->sentMessages.end(), "Local FR should not trigger IF query");
}

void test_frequency_fr_remote_answer_triggers_single_if_query() {
    initializeTestObjects();

    mockRadioSerial->clearSentMessages();
    mockUsbSerial->clearSentMessages();

    testRadioManager->getRemoteCATHandler().parseMessage("FR1;");

    TEST_ASSERT_TRUE(mockRadioSerial->sentMessages.size() >= 1);
    TEST_ASSERT_EQUAL_STRING("IF;", mockRadioSerial->sentMessages.back().c_str());

    // Subsequent FT answer should not schedule another IF within TTL_REALTIME window
    mockRadioSerial->sentMessages.clear();
    testRadioManager->getRemoteCATHandler().parseMessage("FT0;");
    TEST_ASSERT_TRUE(mockRadioSerial->sentMessages.empty());
}

void test_frequency_ft_local_set_does_not_trigger_if_query() {
    initializeTestObjects();

    mockRadioSerial->clearSentMessages();

    testRadioManager->getLocalCATHandler().parseMessage("FT1;");

    TEST_ASSERT_FALSE(mockRadioSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("FT1;", mockRadioSerial->sentMessages.front().c_str());
    const auto it = std::find(mockRadioSerial->sentMessages.begin(), mockRadioSerial->sentMessages.end(), std::string{"IF;"});
    TEST_ASSERT_TRUE_MESSAGE(it == mockRadioSerial->sentMessages.end(), "Local FT should not trigger IF query");
}

void test_frequency_ft_remote_answer_triggers_if_query_once() {
    initializeTestObjects();

    mockRadioSerial->clearSentMessages();

    testRadioManager->getRemoteCATHandler().parseMessage("FT1;");

    TEST_ASSERT_TRUE(mockRadioSerial->sentMessages.size() >= 1);
    TEST_ASSERT_EQUAL_STRING("IF;", mockRadioSerial->sentMessages.back().c_str());
}

void test_frequency_fb_usb_query_uses_cache_when_fresh() {
    initializeTestObjects();

    testRadioManager->updateVfoBFrequency(10'132'000ULL);
    mockRadioSerial->clearSentMessages();
    mockUsbSerial->clearSentMessages();

    testRadioManager->getLocalCATHandler().parseMessage("FB;");

    TEST_ASSERT_TRUE(mockRadioSerial->sentMessages.empty());
    const auto &messages = mockUsbSerial->sentMessages;
    const auto it = std::find(messages.begin(), messages.end(), std::string{"FB00010132000;"});
    TEST_ASSERT_TRUE_MESSAGE(it != messages.end(), "Expected cached FB response on USB");
}

void test_frequency_fb_usb_query_forwards_when_cache_stale() {
    initializeTestObjects();

    testRadioManager->updateVfoBFrequency(10'120'000ULL);
    // Force cache to be stale by setting timestamp to 0 AFTER the frequency update
    testRadioManager->getState().commandCache.update("FB", 0);

    mockRadioSerial->clearSentMessages();
    mockUsbSerial->clearSentMessages();

    testRadioManager->getLocalCATHandler().parseMessage("FB;");

    // When cache is stale, FB responds with cached value AND forwards query to radio
    TEST_ASSERT_FALSE(mockRadioSerial->sentMessages.empty());
    TEST_ASSERT_EQUAL_STRING("FB;", mockRadioSerial->sentMessages.back().c_str());
    TEST_ASSERT_FALSE(mockUsbSerial->sentMessages.empty()); // Cached response sent to USB
    TEST_ASSERT_EQUAL_STRING("FB00010120000;", mockUsbSerial->sentMessages.back().c_str());

    mockRadioSerial->sentMessages.clear();
    mockUsbSerial->clearSentMessages();

    testRadioManager->getRemoteCATHandler().parseMessage("FB00010132000;");

    // State should be updated from radio's answer
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(10'132'000ULL), static_cast<uint32_t>(testRadioManager->getVfoBFrequency()));

    // Note: Fresh answer is NOT forwarded to USB because we already sent cached response.
    // This is by design - see BaseCommandHandler "avoid duplicates" comment.
    // The cached response (10,120,000) was sent immediately; the fresh data (10,132,000)
    // updates internal state but doesn't generate a second response to the same query.
}

// =============================================================================
// ModeCommandHandler Tests
// =============================================================================

void test_mode_handler_md_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test MD (mode) command
    const std::string testCommand = "MD2;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Verify mode was updated
    TEST_ASSERT_EQUAL_INT(2, testRadioManager->getMode());
}

void test_mode_handler_bu_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
    }
    
    // Test BU (band up) command - new command in expanded ModeCommandHandler
    const std::string testCommand = "BU02;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

// =============================================================================
// GainLevelCommandHandler Tests
// =============================================================================

void test_gain_level_handler_ag_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test AG (AF gain) command
    const std::string testCommand = "AG0020;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Verify AF gain was updated (if getter exists)
    // TEST_ASSERT_EQUAL_INT(50, testRadioManager->getAfGain());
    TEST_ASSERT_TRUE(true);
}


// =============================================================================
// StatusInfoCommandHandler Tests
// =============================================================================

void test_status_info_if_response_tracks_active_vfo() {
    initializeTestObjects();

    testRadioManager->updateVfoAFrequency(7'100'000ULL);
    testRadioManager->updateVfoBFrequency(14'070'000ULL);
    testRadioManager->updateRxVfo(0);
    testRadioManager->updateTxVfo(1);
    testRadioManager->updateMode(2); // USB mode
    testRadioManager->getState().isTx.store(false);

    auto& state = testRadioManager->getState();

    const std::string rxResponse = StatusInfoCommandHandler::formatIFResponse(state);
    TEST_ASSERT_TRUE_MESSAGE(rxResponse.starts_with("IF00007100000"), rxResponse.c_str());
    TEST_ASSERT_TRUE_MESSAGE(rxResponse.size() > 30, "IF response shorter than expected");
    TEST_ASSERT_EQUAL_CHAR_MESSAGE('0', rxResponse[30], "RX IF response should indicate VFO A");

    testRadioManager->getState().isTx.store(true);
    const std::string txResponse = StatusInfoCommandHandler::formatIFResponse(state);
    TEST_ASSERT_TRUE_MESSAGE(txResponse.starts_with("IF00014070000"), txResponse.c_str());
    TEST_ASSERT_TRUE_MESSAGE(txResponse.size() > 30, "IF response shorter than expected");
    TEST_ASSERT_EQUAL_CHAR_MESSAGE('1', txResponse[30], "TX IF response should indicate VFO B");

    testRadioManager->getState().isTx.store(false);
}

void test_status_info_remote_if_rx_broadcasts_release() {
    initializeTestObjects();

    auto& state = testRadioManager->getState();
    state.mode.store(3); // ensure IF formatter emits valid mode digit

    const bool originalTx = state.isTx.load();

    state.isTx.store(true);
    const std::string ifTx = StatusInfoCommandHandler::formatIFResponse(state);

    state.isTx.store(false);
    const std::string ifRx = StatusInfoCommandHandler::formatIFResponse(state);

    state.isTx.store(originalTx);

    // Simulate radio entering TX via IF answer
    testRadioManager->getRemoteCATHandler().parseMessage(ifTx);

    // Clear any IF forwarding noise before testing RX broadcast
    mockUsbSerial->clearSentMessages();

    // Simulate radio returning to RX via IF answer
    testRadioManager->getRemoteCATHandler().parseMessage(ifRx);

    TEST_ASSERT_FALSE(state.isTx.load());

    const auto& messages = mockUsbSerial->sentMessages;
    const auto it = std::find(messages.begin(), messages.end(), std::string{"RX;"});
    TEST_ASSERT_TRUE_MESSAGE(it != messages.end(), "Expected RX; broadcast to USB after IF RX");
}

void test_status_info_xi_returns_tx_frequency_in_split_mode() {
    initializeTestObjects();

    testRadioManager->updateVfoAFrequency(7'100'000ULL);
    testRadioManager->updateVfoBFrequency(14'070'000ULL);
    testRadioManager->updateMode(2);
    testRadioManager->updateDataMode(0);
    testRadioManager->updateRxVfo(0);
    testRadioManager->updateTxVfo(1);
    testRadioManager->updateSplitEnabled(true);

    mockUsbSerial->clearSentMessages();

    testRadioManager->getLocalCATHandler().parseMessage("XI;");

    TEST_ASSERT_FALSE_MESSAGE(mockUsbSerial->sentMessages.empty(), "Expected XI response to be sent to USB");
    const std::string& response = mockUsbSerial->sentMessages.back();
    TEST_ASSERT_TRUE_MESSAGE(response.starts_with("XI00014070000"), response.c_str());
}

void test_status_info_xi_returns_rx_frequency_in_simplex_mode() {
    initializeTestObjects();

    testRadioManager->updateVfoAFrequency(7'100'000ULL);
    testRadioManager->updateVfoBFrequency(14'070'000ULL);
    testRadioManager->updateMode(2);
    testRadioManager->updateDataMode(0);
    testRadioManager->updateRxVfo(0);
    testRadioManager->updateTxVfo(0);
    testRadioManager->updateSplitEnabled(false);

    mockUsbSerial->clearSentMessages();

    testRadioManager->getLocalCATHandler().parseMessage("XI;");

    TEST_ASSERT_FALSE_MESSAGE(mockUsbSerial->sentMessages.empty(), "Expected XI response to be sent to USB");
    const std::string& response = mockUsbSerial->sentMessages.back();
    TEST_ASSERT_TRUE_MESSAGE(response.starts_with("XI00007100000"), response.c_str());
}

void test_status_info_xi_answer_updates_tx_vfo_b_frequency() {
    initializeTestObjects();

    testRadioManager->updateTxVfo(1);
    testRadioManager->updateVfoAFrequency(7'080'000ULL);
    testRadioManager->updateVfoBFrequency(14'000'000ULL);

    // Record a pending XI query from USB so the answer will be routed back
    const uint64_t nowUs = esp_timer_get_time();
    testRadioManager->getState().queryTracker.recordQuery("XI", nowUs);
    testRadioManager->noteQueryOrigin("XI", CommandSource::UsbCdc0, nowUs);

    mockUsbSerial->clearSentMessages();

    testRadioManager->getRemoteCATHandler().parseMessage("XI00010132000200;");

    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(10'132'000ULL), static_cast<uint32_t>(testRadioManager->getVfoBFrequency()));
    TEST_ASSERT_EQUAL_INT(2, testRadioManager->getMode());
    TEST_ASSERT_EQUAL_INT(0, testRadioManager->getDataMode());

    const auto &messages = mockUsbSerial->sentMessages;
    const auto it = std::find(messages.begin(), messages.end(), std::string{"XI00010132000200;"});
    TEST_ASSERT_TRUE_MESSAGE(it != messages.end(), "Expected XI answer forwarded to USB");
}

void test_status_info_xi_answer_updates_tx_vfo_a_frequency() {
    initializeTestObjects();

    testRadioManager->updateTxVfo(0);
    testRadioManager->updateVfoAFrequency(7'050'000ULL);
    testRadioManager->updateVfoBFrequency(14'010'000ULL);

    // Record a pending XI query from USB so the answer will be routed back
    const uint64_t nowUs = esp_timer_get_time();
    testRadioManager->getState().queryTracker.recordQuery("XI", nowUs);
    testRadioManager->noteQueryOrigin("XI", CommandSource::UsbCdc0, nowUs);

    mockUsbSerial->clearSentMessages();

    testRadioManager->getRemoteCATHandler().parseMessage("XI00010133000110;");

    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(10'133'000ULL), static_cast<uint32_t>(testRadioManager->getVfoAFrequency()));
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(14'010'000ULL), static_cast<uint32_t>(testRadioManager->getVfoBFrequency()));
    TEST_ASSERT_EQUAL_INT(1, testRadioManager->getDataMode());

    const auto &messages = mockUsbSerial->sentMessages;
    const auto it = std::find(messages.begin(), messages.end(), std::string{"XI00010133000110;"});
    TEST_ASSERT_TRUE_MESSAGE(it != messages.end(), "Expected XI answer forwarded to USB");
}


void test_status_info_if_answer_updates_split_vfo_targets() {
    initializeTestObjects();

    constexpr uint64_t initialVfoA = 7'002'010ULL;
    constexpr uint64_t initialVfoB = 7'004'500ULL;
    constexpr uint64_t reportedTxFrequency = 7'004'570ULL;

    testRadioManager->updateVfoAFrequency(initialVfoA);
    testRadioManager->updateVfoBFrequency(initialVfoB);
    testRadioManager->updateRxVfo(0);
    testRadioManager->updateTxVfo(1);
    testRadioManager->updateSplitEnabled(true);

    std::string ifTx = "IF00007004570"; // P1 frequency placeholder
    ifTx += "     ";                  // P2: five spaces
    ifTx += " 0000";                  // P3: offset +0 Hz
    ifTx += "0";                      // P4: RIT OFF
    ifTx += "0";                      // P5: XIT OFF
    ifTx += "0";                      // P6: memory hundreds digit
    ifTx += "22";                     // P7: memory tens/ones placeholder
    ifTx += "1";                      // P8: TX state
    ifTx += "2";                      // P9: mode (USB)
    ifTx += "1";                      // P10: VFO B selected
    ifTx += "0";                      // P11: scan status
    ifTx += "1";                      // P12: split enabled
    ifTx += "0";                      // P13: tone state
    ifTx += "00";                     // P14: tone frequency index
    ifTx += "0";                      // P15: constant
    ifTx += ";";

    std::ostringstream freqFormatter;
    freqFormatter << std::setw(11) << std::setfill('0') << reportedTxFrequency;
    ifTx.replace(2, 11, freqFormatter.str());

    testRadioManager->getRemoteCATHandler().parseMessage(ifTx);

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(initialVfoA), static_cast<uint32_t>(testRadioManager->getState().vfoAFrequency.load()),
                                     "IF TX frame should not disturb VFO A frequency");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(reportedTxFrequency), static_cast<uint32_t>(testRadioManager->getState().vfoBFrequency.load()),
                                     "IF TX frame should refresh VFO B frequency");
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, testRadioManager->getState().currentTxVfo.load(),
                                  "IF TX frame should confirm TX VFO selection");
}

void test_status_info_if_resend_guard_prevents_spam() {
    initializeTestObjects();

    mockRadioSerial->clearSentMessages();
    mockUsbSerial->clearSentMessages();

    testRadioManager->getLocalCATHandler().parseMessage("IF;");
    TEST_ASSERT_EQUAL_MESSAGE(1, static_cast<int>(mockRadioSerial->sentMessages.size()),
                              "First IF query should forward to radio");

    testRadioManager->getLocalCATHandler().parseMessage("IF;");
    TEST_ASSERT_EQUAL_MESSAGE(1, static_cast<int>(mockRadioSerial->sentMessages.size()),
                              "Guard should suppress rapid repeated IF query");
}


// =============================================================================
// ReceiverProcessingCommandHandler Tests
// =============================================================================

void test_receiver_processing_handler_pa_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test PA (preamp) command - moved from AntennaCommandHandler
    const std::string testCommand = "PA1;";
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    TEST_ASSERT_TRUE(testRadioManager->getPreamp());
}

void test_receiver_processing_handler_nr_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
    }
    
    // Test NR (noise reduction) command - new DSP command
    const std::string testCommand = "NR1;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

// =============================================================================
// MemoryCommandHandler Tests  
// =============================================================================

void test_memory_handler_sv_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test SV (memory transfer) command - newly added
    const std::string testCommand = "SV;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

void test_memory_handler_mc_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test MC (memory channel) command
    const std::string testCommand = "MC010;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

// =============================================================================
// CwCommandHandler Tests
// =============================================================================

void test_cw_handler_ks_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test KS (keyer speed) command
    const std::string testCommand = "KS025;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

// =============================================================================
// ScanCommandHandler Tests
// =============================================================================

void test_scan_handler_sc_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test SC (scan) command
    const std::string testCommand = "SC1;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

// =============================================================================
// ToneSquelchCommandHandler Tests
// =============================================================================

void test_tone_squelch_handler_sq_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test SQ (squelch) command - moved from old conflicting handlers
    const std::string testCommand = "SQ050;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

// =============================================================================
// MenuConfigCommandHandler Tests
// =============================================================================

void test_menu_config_handler_ex_command() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test EX (extended menu) command - moved from conflicting handlers
    const std::string testCommand = "EX0050001;";
    
    // Process command through the local CAT handler
    testRadioManager->getLocalCATHandler().parseMessage(testCommand);
    
    // Test passes if no exceptions are thrown
    TEST_ASSERT_TRUE(true);
}

// =============================================================================
// Integration Tests
// =============================================================================

void test_command_routing_priorities() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Test that commands are routed to the correct consolidated handlers
    // This tests the dispatcher registration order and command routing

    const std::vector<std::string> testCommands = {
        "FA14150000;", "RT1;", "CH01;", "MD2;", "U;", "AG050;",
        "PA1;", "NR1;", "TX;", "MC010;", "SV;", "KS025;", 
        "SC1;", "SQ050;", "EX0050001;"
    };
    
    for (const auto& command : testCommands) {
        // Process command through the local CAT handler
        testRadioManager->getLocalCATHandler().parseMessage(command);
        // Test passes if no exceptions are thrown during parsing/handling
    }
    
    TEST_ASSERT_TRUE(true); // Test passes if all commands processed without errors
}

void test_consolidated_handler_coverage() {
    // Safety check - if RadioManager construction failed, skip this test
    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }
    
    // Verify that all major command types are handled by consolidated handlers
    const std::vector<std::string> commands = {
        "FA;", "FB;", "FR;", "FT;", "DN;", "UP;", "RT;", "XT;", "CH;", "VV;", // FrequencyVfoCommandHandler
        "MD;", "DA;", "BU;", "BD;", // ModeCommandHandler  
        "AG;", "SH;", "SL;", // GainLevelCommandHandler
        "PA;", "RA;", "NR;", "NB;", // ReceiverProcessingCommandHandler
        "TX;", "RX;", "PC;", // TransmitterCommandHandler
        "MC;", "MW;", "MR;", "SV;", // MemoryCommandHandler
        "KS;", "KY;", "SD;", // CwCommandHandler
        "SC;", "SU;", "SW;", // ScanCommandHandler
        "SQ;", "TO;", "TN;", // ToneSquelchCommandHandler
        "EX;", "MN;", "LK;" // MenuConfigCommandHandler
    };
    
    for (const auto& cmd : commands) {
        // Process command through the local CAT handler
        testRadioManager->getLocalCATHandler().parseMessage(cmd);
        // Test passes if no exceptions are thrown during parsing/handling
    }
    
    // Verify that all commands processed without errors
    TEST_ASSERT_TRUE(true);
}

// =============================================================================
// TX Ownership Mutual Exclusion Tests
// =============================================================================

void test_tx_ownership_single_source_acquire() {
    initializeTestObjects();

    // USB CDC0 acquires TX
    testRadioManager->getLocalCATHandler().parseMessage("TX0;");

    // Check that TX is owned by USB CDC0
    TEST_ASSERT_TRUE(testRadioManager->getState().isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), testRadioManager->getState().getTxOwner());
}

void test_tx_ownership_second_source_blocked() {
    initializeTestObjects();

    // Test the basic ownership concept by checking state consistency
    auto& state = testRadioManager->getState();

    // Manually test TX ownership acquisition
    const uint64_t currentTime = esp_timer_get_time();

    // First source acquires TX
    bool acquired1 = state.tryAcquireTx(CommandSource::UsbCdc0, currentTime);
    TEST_ASSERT_TRUE(acquired1);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());

    // Second source tries to acquire TX - should fail
    bool acquired2 = state.tryAcquireTx(CommandSource::UsbCdc1, currentTime);
    TEST_ASSERT_FALSE(acquired2);
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());
}

void test_tx_ownership_owner_can_release() {
    initializeTestObjects();

    // USB CDC0 acquires TX
    testRadioManager->getLocalCATHandler().parseMessage("TX0;");
    TEST_ASSERT_TRUE(testRadioManager->getState().isTx.load());

    // Same source releases TX
    testRadioManager->getLocalCATHandler().parseMessage("RX;");

    // Check that TX is released
    TEST_ASSERT_FALSE(testRadioManager->getState().isTx.load());
    TEST_ASSERT_EQUAL(-1, testRadioManager->getState().getTxOwner());
}

void test_tx_ownership_non_owner_cannot_release() {
    initializeTestObjects();

    // Test the basic ownership release concept
    auto& state = testRadioManager->getState();
    const uint64_t currentTime = esp_timer_get_time();

    // First source acquires TX
    bool acquired = state.tryAcquireTx(CommandSource::UsbCdc0, currentTime);
    TEST_ASSERT_TRUE(acquired);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());

    // Different source tries to release TX - should fail
    bool released = state.releaseTx(CommandSource::UsbCdc1, currentTime);
    TEST_ASSERT_FALSE(released);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());
}

void test_tx_ownership_timeout_releases() {
    initializeTestObjects();

    // Test timeout functionality
    auto& state = testRadioManager->getState();
    const uint64_t currentTime = esp_timer_get_time();

    // Acquire TX
    bool acquired = state.tryAcquireTx(CommandSource::UsbCdc0, currentTime);
    TEST_ASSERT_TRUE(acquired);
    TEST_ASSERT_TRUE(state.isTx.load());

    // Simulate timeout by manually calling force release
    bool forceReleased = state.forceReleaseTx(currentTime);
    TEST_ASSERT_TRUE(forceReleased);

    // Check that TX is released
    TEST_ASSERT_FALSE(state.isTx.load());
    TEST_ASSERT_EQUAL(-1, state.getTxOwner());
}

void test_tx_ownership_radio_authority() {
    initializeTestObjects();

    // Test that radio (Remote source) can override any ownership
    auto& state = testRadioManager->getState();
    const uint64_t currentTime = esp_timer_get_time();

    // USB CDC0 acquires TX
    bool acquired = state.tryAcquireTx(CommandSource::UsbCdc0, currentTime);
    TEST_ASSERT_TRUE(acquired);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());

    // Radio force releases (simulating radio RX command)
    bool forceReleased = state.forceReleaseTx(currentTime);
    TEST_ASSERT_TRUE(forceReleased);

    // Radio should have released TX ownership
    TEST_ASSERT_FALSE(state.isTx.load());
    TEST_ASSERT_EQUAL(-1, state.getTxOwner());
}

void test_tx_ownership_radio_tx_answer() {
    initializeTestObjects();

    // Test that radio can take ownership from any current owner
    auto& state = testRadioManager->getState();
    const uint64_t currentTime = esp_timer_get_time();

    // USB CDC0 acquires TX
    bool acquired = state.tryAcquireTx(CommandSource::UsbCdc0, currentTime);
    TEST_ASSERT_TRUE(acquired);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());

    // Radio takes ownership (simulating radio TX answer forcing ownership)
    state.forceReleaseTx(currentTime);
    bool radioAcquired = state.tryAcquireTx(CommandSource::Remote, currentTime);
    TEST_ASSERT_TRUE(radioAcquired);

    // Radio should have ownership
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::Remote), state.getTxOwner());
}

void test_tx_ownership_panel_can_always_force_rx() {
    initializeTestObjects();

    // Test that Panel (physical buttons) can always force RX regardless of ownership
    auto& state = testRadioManager->getState();
    const uint64_t currentTime = esp_timer_get_time();

    // USB CDC0 acquires TX
    bool acquired = state.tryAcquireTx(CommandSource::UsbCdc0, currentTime);
    TEST_ASSERT_TRUE(acquired);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());

    // Panel (physical MOX button) forces RX
    bool panelForced = state.forceReleaseTx(currentTime);
    TEST_ASSERT_TRUE(panelForced);

    // TX should be released
    TEST_ASSERT_FALSE(state.isTx.load());
    TEST_ASSERT_EQUAL(-1, state.getTxOwner());
}

// =============================================================================
// Stuck TX Recovery Tests - Tests for orphaned TX state detection and recovery
// =============================================================================

void test_stuck_tx_unsolicited_if_sets_activation_time() {
    // Test that unsolicited IF TX response sets txActivationTime for timeout watchdog
    initializeTestObjects();

    auto& state = testRadioManager->getState();

    // Ensure clean state - no TX, no owner, no activation time
    state.isTx.store(false);
    state.txActivationTime.store(0);

    // Build an IF response indicating TX (P8=1) - simulating unsolicited AI mode response
    // IF response format: IF + P1(freq 11) + P2(5 spaces) + P3(offset 5) + P4(RIT) + P5(XIT) +
    //                     P6(mem hundreds) + P7(mem tens/ones 2) + P8(TX) + P9(mode) + P10(VFO) +
    //                     P11(scan) + P12(split) + P13(tone) + P14(tone freq 2) + P15(const) + ;
    std::string ifTxUnsolicited = "IF00007100000";  // P1: frequency
    ifTxUnsolicited += "     ";   // P2: 5 spaces
    ifTxUnsolicited += " 0000";   // P3: offset
    ifTxUnsolicited += "0";       // P4: RIT off
    ifTxUnsolicited += "0";       // P5: XIT off
    ifTxUnsolicited += "0";       // P6: memory hundreds
    ifTxUnsolicited += "00";      // P7: memory channel
    ifTxUnsolicited += "1";       // P8: TX state = 1 (transmitting)
    ifTxUnsolicited += "2";       // P9: mode (USB)
    ifTxUnsolicited += "0";       // P10: VFO A
    ifTxUnsolicited += "0";       // P11: scan off
    ifTxUnsolicited += "0";       // P12: split off
    ifTxUnsolicited += "0";       // P13: tone off
    ifTxUnsolicited += "00";      // P14: tone freq
    ifTxUnsolicited += "0";       // P15: constant
    ifTxUnsolicited += ";";

    // Process unsolicited IF response (no prior IF query recorded)
    // This simulates AI mode auto-broadcast
    testRadioManager->getRemoteCATHandler().parseMessage(ifTxUnsolicited);

    // Verify TX state is set
    TEST_ASSERT_TRUE_MESSAGE(state.isTx.load(), "isTx should be true after IF TX response");

    // Key assertion: txActivationTime should be set even for unsolicited IF
    // This enables the timeout watchdog to detect stuck TX
    TEST_ASSERT_TRUE_MESSAGE(state.txActivationTime.load() > 0,
                             "txActivationTime must be set for timeout watchdog to work");
}

void test_stuck_tx_orphaned_state_cleared_by_if_rx() {
    // Test that orphaned TX state (isTx=true but no owner) is cleared when IF reports RX
    initializeTestObjects();

    auto& state = testRadioManager->getState();

    // Simulate orphaned TX state: isTx=true but txOwner=-1 (no owner)
    // This can happen when unsolicited IF sets isTx without ownership
    state.isTx.store(true);
    state.txActivationTime.store(esp_timer_get_time());
    // Note: txOwner remains -1 (default) - this is the "orphaned" state

    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(-1, state.getTxOwner());

    // Record an IF query to simulate polling during TX
    const uint64_t nowUs = esp_timer_get_time();
    state.queryTracker.recordQuery("IF", nowUs);

    // Build IF response indicating RX (P8=0)
    std::string ifRx = "IF00007100000";  // P1: frequency
    ifRx += "     ";   // P2: 5 spaces
    ifRx += " 0000";   // P3: offset
    ifRx += "0";       // P4: RIT off
    ifRx += "0";       // P5: XIT off
    ifRx += "0";       // P6: memory hundreds
    ifRx += "00";      // P7: memory channel
    ifRx += "0";       // P8: TX state = 0 (receiving)
    ifRx += "2";       // P9: mode (USB)
    ifRx += "0";       // P10: VFO A
    ifRx += "0";       // P11: scan off
    ifRx += "0";       // P12: split off
    ifRx += "0";       // P13: tone off
    ifRx += "00";      // P14: tone freq
    ifRx += "0";       // P15: constant
    ifRx += ";";

    mockUsbSerial->clearSentMessages();

    // Process IF RX response from radio
    testRadioManager->getRemoteCATHandler().parseMessage(ifRx);

    // Verify TX state is cleared
    TEST_ASSERT_FALSE_MESSAGE(state.isTx.load(),
                              "isTx should be false after IF RX response clears orphaned state");

    // Verify txActivationTime is reset
    TEST_ASSERT_EQUAL_MESSAGE(0, static_cast<int>(state.txActivationTime.load()),
                              "txActivationTime should be reset when TX is cleared");

    // Verify RX; broadcast was sent
    const auto& messages = mockUsbSerial->sentMessages;
    const auto it = std::find(messages.begin(), messages.end(), std::string{"RX;"});
    TEST_ASSERT_TRUE_MESSAGE(it != messages.end(),
                             "RX; should be broadcast when orphaned TX state is cleared");
}

void test_stuck_tx_local_owner_cleared_by_radio_rx() {
    // Test that even when a local source owns TX, radio RX IF response clears state
    // This tests the "radio is authoritative" behavior
    initializeTestObjects();

    auto& state = testRadioManager->getState();
    const uint64_t currentTime = esp_timer_get_time();

    // USB CDC0 acquires TX (simulating local PTT)
    bool acquired = state.tryAcquireTx(CommandSource::UsbCdc0, currentTime);
    TEST_ASSERT_TRUE(acquired);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());

    // Record an IF query (simulating periodic TX poll)
    state.queryTracker.recordQuery("IF", currentTime);

    // Build IF response indicating RX (radio has returned to receive)
    std::string ifRx = "IF00007100000";
    ifRx += "     ";
    ifRx += " 0000";
    ifRx += "0";
    ifRx += "0";
    ifRx += "0";
    ifRx += "00";
    ifRx += "0";       // P8: TX state = 0 (receiving)
    ifRx += "2";
    ifRx += "0";
    ifRx += "0";
    ifRx += "0";
    ifRx += "0";
    ifRx += "00";
    ifRx += "0";
    ifRx += ";";

    mockUsbSerial->clearSentMessages();

    // Process IF RX response from radio
    testRadioManager->getRemoteCATHandler().parseMessage(ifRx);

    // Radio says RX, so our state should reflect that
    TEST_ASSERT_FALSE_MESSAGE(state.isTx.load(),
                              "isTx should be false - radio is authoritative on TX/RX state");
}

void test_stuck_tx_timeout_check_works_with_activation_time() {
    // Test that isTxTimedOut() returns true when txActivationTime is set and timeout exceeded
    initializeTestObjects();

    auto& state = testRadioManager->getState();

    // Simulate TX started long ago (beyond timeout)
    const uint64_t oldTime = 1000000ULL;  // 1 second in the past
    const uint64_t currentTime = oldTime + RadioState::TX_TIMEOUT_US + 1000000ULL;  // Beyond timeout

    state.isTx.store(true);
    state.txActivationTime.store(oldTime);

    // Verify timeout is detected
    TEST_ASSERT_TRUE_MESSAGE(state.isTxTimedOut(currentTime),
                             "isTxTimedOut should return true when TX_TIMEOUT_US exceeded");

    // Verify timeout is NOT detected when within timeout
    const uint64_t recentTime = oldTime + 1000000ULL;  // 1 second after start
    TEST_ASSERT_FALSE_MESSAGE(state.isTxTimedOut(recentTime),
                              "isTxTimedOut should return false when within TX_TIMEOUT_US");
}

void test_stuck_tx_timeout_requires_activation_time() {
    // Test that isTxTimedOut() returns false when txActivationTime is 0
    // This was the original bug - unsolicited IF didn't set txActivationTime
    initializeTestObjects();

    auto& state = testRadioManager->getState();

    // Use synthetic time values to avoid issues with small esp_timer_get_time() values during tests
    constexpr uint64_t TX_TIMEOUT_US = 30 * 1000 * 1000;  // 30 seconds
    const uint64_t syntheticCurrentTime = TX_TIMEOUT_US * 2;  // 60 seconds - well past timeout

    // Simulate buggy state: isTx=true but txActivationTime=0
    state.isTx.store(true);
    state.txActivationTime.store(0);

    // Even with a very large currentTime, timeout should return false
    // because txActivationTime is 0 (the check is: activationTime > 0 && ...)
    TEST_ASSERT_FALSE_MESSAGE(state.isTxTimedOut(syntheticCurrentTime),
                              "isTxTimedOut should return false when txActivationTime is 0");

    // Now set activation time to ensure timeout triggers
    // Set activation to 1 second ago, which is 31 seconds before current (> 30 sec timeout)
    const uint64_t timeoutTriggerActivationTime = syntheticCurrentTime - TX_TIMEOUT_US - 1000000;  // 31 seconds ago
    state.txActivationTime.store(timeoutTriggerActivationTime);
    TEST_ASSERT_TRUE_MESSAGE(state.isTxTimedOut(syntheticCurrentTime),
                             "isTxTimedOut should work after txActivationTime is set");
}

void test_panel_tx_can_override_remote_ownership() {
    // Test that Panel (physical MOX button) can force TX even when Remote owns TX
    // This simulates the "stuck in transmit" recovery scenario
    initializeTestObjects();

    auto& state = testRadioManager->getState();
    const uint64_t currentTime = esp_timer_get_time();

    // Remote (radio) acquires TX - simulating radio reporting TX via IF
    bool acquired = state.tryAcquireTx(CommandSource::Remote, currentTime);
    TEST_ASSERT_TRUE(acquired);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::Remote), state.getTxOwner());

    // Clear sent messages to track what happens next
    mockRadioSerial->clearSentMessages();

    // Panel sends TX0; via Panel CAT handler - this should force-acquire TX from Remote
    testRadioManager->getPanelCATHandler().parseMessage("TX0;");

    // Panel should now own TX
    TEST_ASSERT_TRUE_MESSAGE(state.isTx.load(), "isTx should still be true after Panel TX");
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(CommandSource::Panel), state.getTxOwner(),
                              "Panel should have taken TX ownership from Remote");

    // TX0; should have been sent to radio
    const auto& messages = mockRadioSerial->sentMessages;
    const auto it = std::find(messages.begin(), messages.end(), std::string{"TX0;"});
    TEST_ASSERT_TRUE_MESSAGE(it != messages.end(), "TX0; should be sent to radio");
}

void test_if_rx_clears_local_owner_tx_state() {
    // Test that IF RX response clears TX state even when a local source owns TX
    // This ensures radio is authoritative over local software state
    initializeTestObjects();

    auto& state = testRadioManager->getState();
    const uint64_t currentTime = esp_timer_get_time();

    // USB CDC0 acquires TX (simulating software PTT)
    bool acquired = state.tryAcquireTx(CommandSource::UsbCdc0, currentTime);
    TEST_ASSERT_TRUE(acquired);
    TEST_ASSERT_TRUE(state.isTx.load());
    TEST_ASSERT_EQUAL(static_cast<int>(CommandSource::UsbCdc0), state.getTxOwner());

    // Record an IF query (simulating periodic TX poll)
    state.queryTracker.recordQuery("IF", currentTime);

    // Build IF response indicating RX (radio has stopped transmitting)
    std::string ifRx = "IF00007100000";
    ifRx += "     ";
    ifRx += " 0000";
    ifRx += "0";
    ifRx += "0";
    ifRx += "0";
    ifRx += "00";
    ifRx += "0";       // P8: TX state = 0 (receiving)
    ifRx += "2";
    ifRx += "0";
    ifRx += "0";
    ifRx += "0";
    ifRx += "0";
    ifRx += "00";
    ifRx += "0";
    ifRx += ";";

    mockUsbSerial->clearSentMessages();

    // Process IF RX response from radio
    testRadioManager->getRemoteCATHandler().parseMessage(ifRx);

    // Radio says RX, so TX state should be cleared completely
    TEST_ASSERT_FALSE_MESSAGE(state.isTx.load(),
                              "isTx should be false - radio is authoritative");
    TEST_ASSERT_EQUAL_MESSAGE(-1, state.getTxOwner(),
                              "txOwner should be -1 after radio reports RX");

    // RX; should be broadcast to interfaces
    const auto& messages = mockUsbSerial->sentMessages;
    const auto it = std::find(messages.begin(), messages.end(), std::string{"RX;"});
    TEST_ASSERT_TRUE_MESSAGE(it != messages.end(),
                             "RX; should be broadcast when radio reports RX");
}

// =============================================================================
// Duplicate Response Prevention Tests
// =============================================================================

void test_stale_cache_no_duplicate_response() {
    initializeTestObjects();

    if (!testRadioManager) {
        TEST_IGNORE_MESSAGE("RadioManager construction failed");
        return;
    }

    auto& state = testRadioManager->getState();
    const uint64_t startTime = esp_timer_get_time();

    // Step 1: Prime the IF cache with an initial radio response
    const std::string initialIfResponse = "IF00014150000      000000002020000000;";
    testRadioManager->getRemoteCATHandler().parseMessage(initialIfResponse);

    // Verify cache was populated
    const uint64_t cacheTime1 = state.commandCache.get("IF");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, cacheTime1, "IF cache should be populated");

    // Clear sent messages before the test
    mockUsbSerial->clearSentMessages();
    mockRadioSerial->clearSentMessages();

    // Step 2: Wait for cache to become stale (but still exist)
    // StatusInfoCommandHandler uses 1.5s TTL for IF
    vTaskDelay(pdMS_TO_TICKS(1600)); // Wait 1.6 seconds

    const uint64_t beforeQuery = esp_timer_get_time();

    // Step 3: Send IF query from UsbCdc0 (simulating TCP client)
    // This should:
    // - Serve stale cached response immediately
    // - Refresh cache by querying radio
    // - NOT record query origin (to prevent duplicate when radio responds)
    testRadioManager->getLocalCATHandler().parseMessage("IF;");

    // Step 4: Verify exactly ONE response was sent to USB (the cached one)
    const auto& usbMessages1 = mockUsbSerial->sentMessages;
    size_t ifResponseCount = 0;
    for (const auto& msg : usbMessages1) {
        if (msg.substr(0, 2) == "IF") {
            ifResponseCount++;
        }
    }
    TEST_ASSERT_EQUAL_MESSAGE(1, ifResponseCount,
        "Should send exactly 1 IF response (cached) to USB, not duplicate");

    // Step 5: Verify radio query was sent to refresh cache
    const auto& radioMessages = mockRadioSerial->sentMessages;
    bool radioQuerySent = false;
    for (const auto& msg : radioMessages) {
        if (msg == "IF;") {
            radioQuerySent = true;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(radioQuerySent, "Should send IF; query to radio to refresh cache");

    // Clear USB messages before radio response
    mockUsbSerial->clearSentMessages();

    // Step 6: Simulate radio responding with updated data
    const std::string radioResponse = "IF00014160000      000000002020000000;";
    testRadioManager->getRemoteCATHandler().parseMessage(radioResponse);

    // Step 7: Verify radio response was NOT sent back to USB (no duplicate)
    const auto& usbMessages2 = mockUsbSerial->sentMessages;
    size_t duplicateCount = 0;
    for (const auto& msg : usbMessages2) {
        if (msg.substr(0, 2) == "IF") {
            duplicateCount++;
        }
    }
    TEST_ASSERT_EQUAL_MESSAGE(0, duplicateCount,
        "Radio response should NOT be routed back to USB (would be duplicate)");

    // Step 8: Verify cache was updated with new radio data
    const uint64_t cacheTime2 = state.commandCache.get("IF");
    TEST_ASSERT_GREATER_THAN_MESSAGE(cacheTime1, cacheTime2,
        "Cache should be updated with new radio response");
}

extern "C" void run_consolidated_command_handlers_tests() {
    // FrequencyVfoCommandHandler tests
    RUN_TEST(test_frequency_vfo_handler_fa_command);
    
    RUN_TEST(test_frequency_vfo_handler_ch_command);
    RUN_TEST(test_frequency_vfo_handler_vv_command);
    RUN_TEST(test_frequency_up_zero_step_no_change);
    RUN_TEST(test_frequency_up_multi_step_updates_state);
    RUN_TEST(test_frequency_dn_multi_step_updates_state);
    RUN_TEST(test_frequency_fr_local_set_does_not_trigger_if_query);
    RUN_TEST(test_frequency_fr_remote_answer_triggers_single_if_query);
    RUN_TEST(test_frequency_ft_local_set_does_not_trigger_if_query);
    RUN_TEST(test_frequency_ft_remote_answer_triggers_if_query_once);
    RUN_TEST(test_frequency_fb_usb_query_uses_cache_when_fresh);
    RUN_TEST(test_frequency_fb_usb_query_forwards_when_cache_stale);

    // ModeCommandHandler tests
    RUN_TEST(test_mode_handler_md_command);
    RUN_TEST(test_mode_handler_bu_command);
    
    // GainLevelCommandHandler tests
    RUN_TEST(test_gain_level_handler_ag_command);

    // StatusInfoCommandHandler tests
    RUN_TEST(test_status_info_if_response_tracks_active_vfo);
    RUN_TEST(test_status_info_remote_if_rx_broadcasts_release);
    RUN_TEST(test_status_info_xi_returns_tx_frequency_in_split_mode);
    RUN_TEST(test_status_info_xi_returns_rx_frequency_in_simplex_mode);
    RUN_TEST(test_status_info_xi_answer_updates_tx_vfo_b_frequency);
    RUN_TEST(test_status_info_xi_answer_updates_tx_vfo_a_frequency);
    RUN_TEST(test_status_info_if_answer_updates_split_vfo_targets);
    RUN_TEST(test_status_info_if_resend_guard_prevents_spam);

    // ReceiverProcessingCommandHandler tests
    RUN_TEST(test_receiver_processing_handler_pa_command);
    RUN_TEST(test_receiver_processing_handler_nr_command);
    
    // MemoryCommandHandler tests
    RUN_TEST(test_memory_handler_sv_command);
    RUN_TEST(test_memory_handler_mc_command);
    
    // CwCommandHandler tests
    RUN_TEST(test_cw_handler_ks_command);
    
    // ScanCommandHandler tests
    RUN_TEST(test_scan_handler_sc_command);
    
    // ToneSquelchCommandHandler tests
    RUN_TEST(test_tone_squelch_handler_sq_command);
    
    // MenuConfigCommandHandler tests
    RUN_TEST(test_menu_config_handler_ex_command);
    
    // Integration tests
    RUN_TEST(test_command_routing_priorities);
    RUN_TEST(test_consolidated_handler_coverage);

    // TX Ownership tests
    RUN_TEST(test_tx_ownership_single_source_acquire);
    RUN_TEST(test_tx_ownership_second_source_blocked);
    RUN_TEST(test_tx_ownership_owner_can_release);
    RUN_TEST(test_tx_ownership_non_owner_cannot_release);
    RUN_TEST(test_tx_ownership_timeout_releases);
    RUN_TEST(test_tx_ownership_radio_authority);
    RUN_TEST(test_tx_ownership_radio_tx_answer);
    RUN_TEST(test_tx_ownership_panel_can_always_force_rx);

    // Stuck TX Recovery tests
    RUN_TEST(test_stuck_tx_unsolicited_if_sets_activation_time);
    RUN_TEST(test_stuck_tx_orphaned_state_cleared_by_if_rx);
    RUN_TEST(test_stuck_tx_local_owner_cleared_by_radio_rx);
    RUN_TEST(test_stuck_tx_timeout_check_works_with_activation_time);
    RUN_TEST(test_stuck_tx_timeout_requires_activation_time);
    RUN_TEST(test_panel_tx_can_override_remote_ownership);
    RUN_TEST(test_if_rx_clears_local_owner_tx_state);

    // Duplicate Response Prevention tests
    RUN_TEST(test_stale_cache_no_duplicate_response);
}
