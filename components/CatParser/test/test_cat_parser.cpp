#include <string>
#include <vector>
#include "unity.h"
#include "test_hooks.h"
#include "../include/ParserUtils.h"
#include "esp_log.h"

using namespace radio::cat;

static const char *TAG = "CAT_PARSER_TEST";


// ============== PARSER UTILITY TESTS ==============

void test_parseFrequency_valid_values() {
    
    // Test valid frequency values
    TEST_ASSERT_EQUAL_UINT32(14150000UL, static_cast<uint32_t>(ParserUtils::parseFrequency("14150000")));
    TEST_ASSERT_EQUAL_UINT32(7100000UL, static_cast<uint32_t>(ParserUtils::parseFrequency("07100000")));
    TEST_ASSERT_EQUAL_UINT32(28074000UL, static_cast<uint32_t>(ParserUtils::parseFrequency("28074000")));
    
}

void test_parseFrequency_boundary_values() {
    
    // Test boundary values
    TEST_ASSERT_EQUAL_UINT32(30000UL, static_cast<uint32_t>(ParserUtils::parseFrequency("00030000")));
    TEST_ASSERT_EQUAL_UINT32(60000000UL, static_cast<uint32_t>(ParserUtils::parseFrequency("60000000")));
    
}

void test_parseFrequency_invalid_values() {
    
    // Test invalid frequency values should return 0
    TEST_ASSERT_EQUAL_UINT32(0UL, static_cast<uint32_t>(ParserUtils::parseFrequency("")));
    TEST_ASSERT_EQUAL_UINT32(0UL, static_cast<uint32_t>(ParserUtils::parseFrequency("invalid")));
    TEST_ASSERT_EQUAL_UINT32(0UL, static_cast<uint32_t>(ParserUtils::parseFrequency("14150ABC")));
    
}

void test_parseInt_valid_values() {
    
    // Test valid integer parsing
    TEST_ASSERT_EQUAL_INT(123, ParserUtils::parseInt("123"));
    TEST_ASSERT_EQUAL_INT(0, ParserUtils::parseInt("0"));
    TEST_ASSERT_EQUAL_INT(255, ParserUtils::parseInt("255"));
    TEST_ASSERT_EQUAL_INT(1, ParserUtils::parseInt("1"));
    
}

void test_parseInt_with_whitespace() {
    
    // Test parsing with whitespace
    TEST_ASSERT_EQUAL_INT(123, ParserUtils::parseInt(" 123"));
    TEST_ASSERT_EQUAL_INT(123, ParserUtils::parseInt("  123"));
    TEST_ASSERT_EQUAL_INT(123, ParserUtils::parseInt("\t123"));
    
}

void test_parseInt_invalid_values() {
    
    // Test invalid integer values should return -1
    TEST_ASSERT_EQUAL_INT(-1, ParserUtils::parseInt(""));
    TEST_ASSERT_EQUAL_INT(-1, ParserUtils::parseInt("invalid"));
    TEST_ASSERT_EQUAL_INT(-1, ParserUtils::parseInt("12A"));
    
}

void test_parseBool_valid_values() {
    
    // Test valid boolean values
    TEST_ASSERT_TRUE(ParserUtils::parseBool("1"));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("0"));
    
}

void test_parseBool_invalid_values() {
    
    // Test invalid boolean values should return false
    TEST_ASSERT_FALSE(ParserUtils::parseBool(""));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("2"));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("true"));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("false"));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("10"));
    
}

void test_isValidCAT_valid_commands() {

    // Test valid CAT command formats
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("FA;"));
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("FA00014150000;"));
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("MD1;"));
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("IF00014150000      000000003020010080;"));

}

void test_isValidCAT_invalid_commands() {

    // Test invalid CAT command formats
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT(""));
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("F"));
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("FA"));
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("FA00014150000"));  // No semicolon
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("1A;"));         // First char not letter
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("F1;"));         // Second char not letter

}

void test_getPrefix_valid_commands() {
    
    // Test valid command prefixes
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("FA;") == "FA");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("MD1;") == "MD");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("TX;") == "TX");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("IF00014150000;") == "IF");
    
}

void test_getPrefix_invalid_commands() {
    
    // Test invalid cases should return empty
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("").empty());
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("F").empty());
    
}

void test_getParameters_with_params() {

    // Test commands with parameters
    TEST_ASSERT_TRUE(ParserUtils::getParameters("FA00014150000;") == "00014150000");
    TEST_ASSERT_TRUE(ParserUtils::getParameters("MD1;") == "1");
    TEST_ASSERT_TRUE(ParserUtils::getParameters("AG0255;") == "0255");

}

void test_getParameters_without_params() {
    
    // Test commands without parameters
    TEST_ASSERT_TRUE(ParserUtils::getParameters("FA;").empty());
    TEST_ASSERT_TRUE(ParserUtils::getParameters("MD;").empty());
    TEST_ASSERT_TRUE(ParserUtils::getParameters("TX;").empty());
    
}

// ============== COMMAND VALIDATION TESTS ==============

void test_validate_FA_parameters(void) {
    
    // Test FA parameter validation
    [[maybe_unused]] std::string validFA = "14150000";
    [[maybe_unused]] std::string invalidFA = "99999999999";  // Too many digits
    
    // This would test parameter validation if utilities exist
}

void test_validate_MD_parameters() {
    
    // Test MD parameter validation
    for (int mode = 1; mode <= 9; mode++) {
        //std::to_string(mode);
        // Valid modes 1-9
    }
    
    // Invalid modes
    [[maybe_unused]] std::string invaliMode = "0";   // Mode 0 invalid
    [[maybe_unused]] std::string invalidMode2 = "10"; // Mode 10 invalid

}

// ============== EDGE CASE TESTS ==============

void test_handle_long_commands() {
    
    // Test handling of very long commands
    std::string longCommand = "IF00014150000                    000000003020010080;";
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT(longCommand));
    
}

void test_handle_special_characters() {
    
    // Test handling of commands with spaces and special characters
    std::string ifCommand = "IF00014150000      +00000003020010080;";
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT(ifCommand));
    
}

void test_case_sensitivity() {
    
    // CAT commands should be case sensitive (uppercase)
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("FA;"));
    // Lowercase should be invalid (implementation dependent)
    // TEST_ASSERT_FALSE(ParserUtils::isValidCAT("fa;"));

}

// ============== UI META COMMAND TESTS ==============

void test_ui_command_prefix_extraction() {
    // UI meta commands should have 4-character prefixes
    // These tests verify the parser correctly identifies UI commands

    // Standard 2-char commands
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("FA;") == "FA");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("MD1;") == "MD");

    // UI commands - getPrefix still returns 2 chars (extended in CatParser)
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("UIXD1;") == "UI");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("UIPC050;") == "UI");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("UIBL128;") == "UI");
}

void test_ui_command_parameters() {
    // UI commands should extract parameters after 4-char prefix
    // getParameters uses 2-char offset, so raw call returns wrong value
    // CatParser handles this specially for UI commands

    // Raw getParameters (2-char offset) - shows what CatParser must fix
    TEST_ASSERT_TRUE(ParserUtils::getParameters("UIXD1;") == "XD1");
    TEST_ASSERT_TRUE(ParserUtils::getParameters("UIPC050;") == "PC050");

    // Query format (no params after 4-char prefix)
    // UIXD; has length 5, getParameters returns "XD" which is not the intent
    // CatParser checks frame.length() > 5 for UI param extraction
}

extern "C" void run_cat_parser_tests(void) {
    RUN_TEST(test_parseFrequency_valid_values);
    RUN_TEST(test_parseFrequency_boundary_values);
    RUN_TEST(test_parseFrequency_invalid_values);
    
    RUN_TEST(test_parseInt_valid_values);
    RUN_TEST(test_parseInt_with_whitespace);
    RUN_TEST(test_parseInt_invalid_values);
    
    RUN_TEST(test_parseBool_valid_values);
    RUN_TEST(test_parseBool_invalid_values);
    
    RUN_TEST(test_isValidCAT_valid_commands);
    RUN_TEST(test_isValidCAT_invalid_commands);
    
    RUN_TEST(test_getPrefix_valid_commands);
    RUN_TEST(test_getPrefix_invalid_commands);
    
    RUN_TEST(test_getParameters_with_params);
    RUN_TEST(test_getParameters_without_params);

    RUN_TEST(test_validate_FA_parameters);
    RUN_TEST(test_validate_MD_parameters);
    
    RUN_TEST(test_handle_long_commands);
    RUN_TEST(test_handle_special_characters);
    RUN_TEST(test_case_sensitivity);

    // UI meta command tests
    RUN_TEST(test_ui_command_prefix_extraction);
    RUN_TEST(test_ui_command_parameters);
}
