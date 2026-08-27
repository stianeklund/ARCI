// Direct unit tests for CatParser frame parsing (parseFrame / determineCommandType /
// parseParameters), exercised through the only public entry point parseMessage().
//
// These complement test_cat_parser_utils.cpp (which covers the stateless ParserUtils
// helpers). CatParser is pure logic with no UART/hardware dependency, so every case
// below is deterministic. Command formats and the Set/Read/Answer inference rules are
// validated against spec/ts590sg_cat_commands_v3.json.

#include "unity.h"
#include "test_hooks.h"
#include "CatParser.h"
#include <string>
#include <variant>

using radio::CatParser;
using radio::RadioCommand;
using radio::CommandSource;
using radio::CommandType;

// Parse a single message frame and return the resulting RadioCommand.
// commandsOut receives the number of commands the parser emitted (0 when a frame is
// rejected / guarded), so callers can distinguish "no command" from a default command.
static RadioCommand parseSingle(const std::string& frame, CommandSource source, int* commandsOut = nullptr) {
    CatParser parser;
    RadioCommand result;
    int count = 0;
    parser.parseMessage(frame, source, [&](const RadioCommand& cmd) {
        result = cmd;
        ++count;
    });
    if (commandsOut) {
        *commandsOut = count;
    }
    return result;
}

// ============== determineCommandType: Set / Read / Answer inference ==============

// Bare "FA;" from a client is a READ query (empty params + local source).
void test_frame_FA_bare_is_read() {
    int n = 0;
    const RadioCommand cmd = parseSingle("FA;", CommandSource::UsbCdc0, &n);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_TRUE(cmd.command == "FA");
    TEST_ASSERT_EQUAL(CommandType::Read, cmd.type);
    TEST_ASSERT_FALSE(cmd.hasParams());
}

// "FA00014074000;" from a client (USB/TCP/Display/Panel/Macro) is a SET.
void test_frame_FA_value_from_client_is_set() {
    const RadioCommand usb = parseSingle("FA00014074000;", CommandSource::UsbCdc0);
    TEST_ASSERT_EQUAL(CommandType::Set, usb.type);
    TEST_ASSERT_TRUE(usb.command == "FA");
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(usb.getParamCount()));

    // Same frame from a TCP client is also a SET (all local sources behave alike).
    const RadioCommand tcp = parseSingle("FA00014074000;", CommandSource::Tcp0);
    TEST_ASSERT_EQUAL(CommandType::Set, tcp.type);
}

// The identical frame from the radio (Remote) is an ANSWER (source-dependent inference).
void test_frame_FA_value_from_radio_is_answer() {
    const RadioCommand cmd = parseSingle("FA00014074000;", CommandSource::Remote);
    TEST_ASSERT_EQUAL(CommandType::Answer, cmd.type);
    TEST_ASSERT_TRUE(cmd.command == "FA");
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(cmd.getParamCount()));
}

// A bare prefix from the radio with no params is an ANSWER (unsolicited status).
void test_frame_bare_from_radio_is_answer() {
    const RadioCommand cmd = parseSingle("FA;", CommandSource::Remote);
    TEST_ASSERT_EQUAL(CommandType::Answer, cmd.type);
}

// Set-only / action prefixes are SET even with no params when locally sourced.
// (spec: BD/BU/CH/DN/EM/MK/MW/QD/QI/RC/SR/SV/UP/VV are set-only; RX/TX are actions.)
void test_frame_action_prefix_no_params_is_set() {
    TEST_ASSERT_EQUAL(CommandType::Set, parseSingle("TX;", CommandSource::UsbCdc0).type);
    TEST_ASSERT_EQUAL(CommandType::Set, parseSingle("RX;", CommandSource::UsbCdc0).type);
    TEST_ASSERT_EQUAL(CommandType::Set, parseSingle("QD;", CommandSource::UsbCdc0).type);
    // A non-action prefix with no params from a client is still a READ.
    TEST_ASSERT_EQUAL(CommandType::Read, parseSingle("AG;", CommandSource::UsbCdc0).type);
}

// EX 7-char params (menu 3 + "0000" padding) is a READ regardless of source.
// spec/ts590sg_cat_commands_v3.json EX parsing_notes:
//   "Total parameter length 7 chars (menu + '0000') = READ command".
// Example read frame from the spec: "EX0060000;" -> params "0060000" (7 chars).
void test_frame_EX_7char_is_read() {
    TEST_ASSERT_EQUAL(CommandType::Read, parseSingle("EX0060000;", CommandSource::UsbCdc0).type);
    TEST_ASSERT_EQUAL(CommandType::Read, parseSingle("EX0060000;", CommandSource::Remote).type);
}

// EX 8+ char params carry a value: SET from a local source, ANSWER from the radio.
// spec: "Total parameter length 8+ chars = SET (local source) or ANSWER (remote source)".
// Example: "EX006000005;" -> params "006000005" (9 chars: menu 006 + 0000 + value 05).
void test_frame_EX_with_value_is_source_dependent() {
    const RadioCommand localSet = parseSingle("EX006000005;", CommandSource::UsbCdc0);
    TEST_ASSERT_EQUAL(CommandType::Set, localSet.type);
    TEST_ASSERT_TRUE(localSet.command == "EX");

    const RadioCommand remoteAnswer = parseSingle("EX006000005;", CommandSource::Remote);
    TEST_ASSERT_EQUAL(CommandType::Answer, remoteAnswer.type);
}

// PS exception: "PS;" is a READ from BOTH sources. The parser special-cases the radio's
// bare "PS;" as a query (RRC-1258 power poll) instead of the default Answer.
void test_frame_PS_bare_is_read_from_both_sources() {
    TEST_ASSERT_EQUAL(CommandType::Read, parseSingle("PS;", CommandSource::UsbCdc0).type);
    TEST_ASSERT_EQUAL(CommandType::Read, parseSingle("PS;", CommandSource::Remote).type);
    // With a value it follows the normal rules again: client=Set, radio=Answer.
    TEST_ASSERT_EQUAL(CommandType::Set, parseSingle("PS1;", CommandSource::UsbCdc0).type);
    TEST_ASSERT_EQUAL(CommandType::Answer, parseSingle("PS1;", CommandSource::Remote).type);
}

// UI meta commands use a 4-character prefix; params start after 4 chars.
void test_frame_UI_4char_prefix() {
    // UIPC050; -> prefix "UIPC", params "050", SET from a client. "UIPC" has no dedicated
    // split branch, so parseParameters falls back to the default numeric parse: "050" -> 50.
    const RadioCommand set = parseSingle("UIPC050;", CommandSource::UsbCdc0);
    TEST_ASSERT_TRUE(set.command == "UIPC");
    TEST_ASSERT_EQUAL(CommandType::Set, set.type);
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(set.getParamCount()));
    const radio::ParamValue* p0 = set.getParamFast(0);
    TEST_ASSERT_NOT_NULL(p0);
    TEST_ASSERT_TRUE(p0->isInt());
    TEST_ASSERT_EQUAL_INT(50, p0->asInt());

    // UIXD; (length 5) -> prefix "UIXD" with no params -> READ query.
    const RadioCommand read = parseSingle("UIXD;", CommandSource::UsbCdc0);
    TEST_ASSERT_TRUE(read.command == "UIXD");
    TEST_ASSERT_EQUAL(CommandType::Read, read.type);
    TEST_ASSERT_FALSE(read.hasParams());
}

// Minimal-selector queries: CD<x>; (1-char param) and SM0; are READs, not Sets.
void test_frame_selector_queries_are_read() {
    TEST_ASSERT_EQUAL(CommandType::Read, parseSingle("CD0;", CommandSource::UsbCdc0).type);
    TEST_ASSERT_EQUAL(CommandType::Read, parseSingle("SM0;", CommandSource::UsbCdc0).type);
    // AG with a single-digit param selects a sub-function (read); multi-digit is a Set.
    TEST_ASSERT_EQUAL(CommandType::Read, parseSingle("AG0;", CommandSource::UsbCdc0).type);
    TEST_ASSERT_EQUAL(CommandType::Set, parseSingle("AG0255;", CommandSource::UsbCdc0).type);
}

// ============== parseParameters: per-prefix field splitting ==============

// AS SET/ANSWER form splits into 5 fields: P1(1) P2(2) P3(11) P4(1) P5(1) = 16 chars.
// spec AS format: "ASP1P2P2P3P3P3P3P3P3P3P3P3P3P3P4P5;".
// Frame: AS 0 05 00014074000 2 0 ; -> "AS0050001407400020;".
void test_frame_AS_field_split() {
    const RadioCommand cmd = parseSingle("AS0050001407400020;", CommandSource::UsbCdc0);
    TEST_ASSERT_EQUAL(CommandType::Set, cmd.type);
    TEST_ASSERT_EQUAL_INT(5, static_cast<int>(cmd.getParamCount()));

    const radio::ParamValue* p1 = cmd.getParamFast(0);
    const radio::ParamValue* p2 = cmd.getParamFast(1);
    const radio::ParamValue* p3 = cmd.getParamFast(2);
    const radio::ParamValue* p4 = cmd.getParamFast(3);
    TEST_ASSERT_NOT_NULL(p1);
    TEST_ASSERT_NOT_NULL(p2);
    TEST_ASSERT_NOT_NULL(p3);
    TEST_ASSERT_NOT_NULL(p4);
    TEST_ASSERT_TRUE(p1->asString() == "0");            // P1: const
    TEST_ASSERT_TRUE(p2->asString() == "05");           // P2: channel
    TEST_ASSERT_TRUE(p3->asString() == "00014074000");  // P3: frequency (11)
    TEST_ASSERT_TRUE(p4->asString() == "2");            // P4: mode
    // P5 spills into the heap overflow vector (inline capacity is 4).
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(cmd.overflowParams.size()));
    TEST_ASSERT_TRUE(std::holds_alternative<std::string>(cmd.overflowParams[0]));
    TEST_ASSERT_TRUE(std::get<std::string>(cmd.overflowParams[0]) == "0"); // P5: data flag
}

// PL splits into two 3-wide fields: P1P1P1 (input) + P2P2P2 (output).
// spec PL format: "PLP1P1P1P2P2P2;". Frame "PL100200;" -> "100" + "200".
void test_frame_PL_field_split() {
    const RadioCommand cmd = parseSingle("PL100200;", CommandSource::UsbCdc0);
    TEST_ASSERT_EQUAL(CommandType::Set, cmd.type);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(cmd.getParamCount()));
    TEST_ASSERT_TRUE(cmd.getParamFast(0)->asString() == "100");
    TEST_ASSERT_TRUE(cmd.getParamFast(1)->asString() == "200");
}

// PA ANSWER form splits into two 1-wide fields P1P2 (e.g. radio reply "PA10;").
void test_frame_PA_field_split() {
    const RadioCommand cmd = parseSingle("PA10;", CommandSource::Remote);
    TEST_ASSERT_EQUAL(CommandType::Answer, cmd.type);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(cmd.getParamCount()));
    TEST_ASSERT_TRUE(cmd.getParamFast(0)->asString() == "1");
    TEST_ASSERT_TRUE(cmd.getParamFast(1)->asString() == "0");
}

// CD compact form splits into selector + value (CDx<value>).
void test_frame_CD_selector_value_split() {
    const RadioCommand cmd = parseSingle("CD00;", CommandSource::UsbCdc0);
    TEST_ASSERT_EQUAL(CommandType::Set, cmd.type);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(cmd.getParamCount()));
    TEST_ASSERT_TRUE(cmd.getParamFast(0)->asString() == "0"); // selector
    TEST_ASSERT_TRUE(cmd.getParamFast(1)->asString() == "0"); // value
}

// SM and RM answers are P1(1) + P2(4), not one five-digit integer.
void test_frame_meter_answer_field_split() {
    const RadioCommand sm = parseSingle("SM00005;", CommandSource::Remote);
    TEST_ASSERT_EQUAL(CommandType::Answer, sm.type);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(sm.getParamCount()));
    TEST_ASSERT_TRUE(sm.getParamFast(0)->asString() == "0");
    TEST_ASSERT_TRUE(sm.getParamFast(1)->asString() == "0005");

    const RadioCommand rm = parseSingle("RM10001;", CommandSource::Remote);
    TEST_ASSERT_EQUAL(CommandType::Answer, rm.type);
    TEST_ASSERT_EQUAL_INT(2, static_cast<int>(rm.getParamCount()));
    TEST_ASSERT_TRUE(rm.getParamFast(0)->asString() == "1");
    TEST_ASSERT_TRUE(rm.getParamFast(1)->asString() == "0001");
}

// ============== Malformed / edge input ==============

// An empty message is guarded in parseMessage and emits no command.
void test_frame_empty_message_no_command() {
    int n = -1;
    parseSingle("", CommandSource::UsbCdc0, &n);
    TEST_ASSERT_EQUAL_INT(0, n);
}

// A lone ";" is the wakeup/terminator command: emitted with command == ";".
void test_frame_semicolon_only_is_wakeup() {
    int n = 0;
    const RadioCommand cmd = parseSingle(";", CommandSource::UsbCdc0, &n);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_TRUE(cmd.command == ";");
}

// A frame with no terminator fails CAT validation and produces no command.
void test_frame_no_terminator_rejected() {
    int n = -1;
    parseSingle("FA", CommandSource::UsbCdc0, &n);
    TEST_ASSERT_EQUAL_INT(0, n);
}

// An unknown 2-letter prefix is still structurally valid: the parser does not check the
// prefix against the spec, so "ZZ;" parses as a READ query with prefix "ZZ".
void test_frame_unknown_prefix_still_parses() {
    int n = 0;
    const RadioCommand cmd = parseSingle("ZZ;", CommandSource::UsbCdc0, &n);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_TRUE(cmd.command == "ZZ");
    TEST_ASSERT_EQUAL(CommandType::Read, cmd.type);
}

// The parser does NOT normalise case: a lowercase prefix is preserved verbatim
// ("fa;" -> command "fa"), because getPrefix() is a plain substr and isValidCAT()
// accepts lowercase letters (std::isalpha). Any case-folding happens upstream, not here.
void test_frame_lowercase_prefix_not_aliased() {
    int n = 0;
    const RadioCommand cmd = parseSingle("fa;", CommandSource::UsbCdc0, &n);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_TRUE(cmd.command == "fa");
    TEST_ASSERT_FALSE(cmd.command == "FA");
}

// A parameter value >= ParamValue::SSO_CAPACITY (16) chars is truncated to 15 chars and
// flagged via wasTruncated(). KY (CW keyer text) can carry long payloads; it falls into
// the default parse branch and is stored as a single string param.
void test_frame_param_truncation_flag() {
    // params = "0ABCDEFGHIJKLMNOP" (17 chars, exceeds SSO_CAPACITY).
    const RadioCommand cmd = parseSingle("KY0ABCDEFGHIJKLMNOP;", CommandSource::UsbCdc0);
    TEST_ASSERT_EQUAL(CommandType::Set, cmd.type);
    TEST_ASSERT_EQUAL_INT(1, static_cast<int>(cmd.getParamCount()));
    const radio::ParamValue* p0 = cmd.getParamFast(0);
    TEST_ASSERT_NOT_NULL(p0);
    TEST_ASSERT_TRUE(p0->isString());
    TEST_ASSERT_TRUE(p0->wasTruncated());
    TEST_ASSERT_EQUAL_INT(radio::ParamValue::SSO_CAPACITY - 1,
                          static_cast<int>(p0->asStringView().length()));

    // A value that fits (< 16 chars) is stored intact and not flagged.
    const RadioCommand fits = parseSingle("KY0SHORT;", CommandSource::UsbCdc0);
    const radio::ParamValue* q0 = fits.getParamFast(0);
    TEST_ASSERT_NOT_NULL(q0);
    TEST_ASSERT_FALSE(q0->wasTruncated());
    TEST_ASSERT_TRUE(q0->asString() == "0SHORT");
}

extern "C" void run_cat_parser_frame_tests(void) {
    // determineCommandType matrix
    RUN_TEST(test_frame_FA_bare_is_read);
    RUN_TEST(test_frame_FA_value_from_client_is_set);
    RUN_TEST(test_frame_FA_value_from_radio_is_answer);
    RUN_TEST(test_frame_bare_from_radio_is_answer);
    RUN_TEST(test_frame_action_prefix_no_params_is_set);
    RUN_TEST(test_frame_EX_7char_is_read);
    RUN_TEST(test_frame_EX_with_value_is_source_dependent);
    RUN_TEST(test_frame_PS_bare_is_read_from_both_sources);
    RUN_TEST(test_frame_UI_4char_prefix);
    RUN_TEST(test_frame_selector_queries_are_read);

    // parseParameters field splitting
    RUN_TEST(test_frame_AS_field_split);
    RUN_TEST(test_frame_PL_field_split);
    RUN_TEST(test_frame_PA_field_split);
    RUN_TEST(test_frame_CD_selector_value_split);
    RUN_TEST(test_frame_meter_answer_field_split);

    // Malformed / edge input
    RUN_TEST(test_frame_empty_message_no_command);
    RUN_TEST(test_frame_semicolon_only_is_wakeup);
    RUN_TEST(test_frame_no_terminator_rejected);
    RUN_TEST(test_frame_unknown_prefix_still_parses);
    RUN_TEST(test_frame_lowercase_prefix_not_aliased);
    RUN_TEST(test_frame_param_truncation_flag);
}
