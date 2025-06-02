#include "unity.h"
#include "test_hooks.h"
#include "ParserUtils.h"
#include <string_view>

using namespace radio::cat;

// Note: Using global Unity setUp/tearDown from another suite.

// Test ParserUtils::parseFrequency
void test_parseFrequency() {
    // Valid frequency values
    TEST_ASSERT_EQUAL_UINT32(14074000, (uint32_t)ParserUtils::parseFrequency("14074000"));
    TEST_ASSERT_EQUAL_UINT32(7074000, (uint32_t)ParserUtils::parseFrequency("07074000"));
    TEST_ASSERT_EQUAL_UINT32(144000000, (uint32_t)ParserUtils::parseFrequency("144000000"));
    TEST_ASSERT_EQUAL_UINT32(50000000, (uint32_t)ParserUtils::parseFrequency("050000000"));
    
    // Edge cases
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)ParserUtils::parseFrequency("000000000"));
    TEST_ASSERT_EQUAL_UINT32(999999999, (uint32_t)ParserUtils::parseFrequency("999999999"));
    
    // Invalid cases should return 0
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)ParserUtils::parseFrequency(""));
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)ParserUtils::parseFrequency("invalid"));
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)ParserUtils::parseFrequency("14074ABC"));
}

// Test ParserUtils::parseInt
void test_parseInt() {
    // Valid integer values
    TEST_ASSERT_EQUAL_INT(123, ParserUtils::parseInt("123"));
    TEST_ASSERT_EQUAL_INT(0, ParserUtils::parseInt("0"));
    TEST_ASSERT_EQUAL_INT(255, ParserUtils::parseInt("255"));
    TEST_ASSERT_EQUAL_INT(1, ParserUtils::parseInt("1"));
    
    // With whitespace
    TEST_ASSERT_EQUAL_INT(123, ParserUtils::parseInt(" 123"));
    TEST_ASSERT_EQUAL_INT(123, ParserUtils::parseInt("  123"));
    TEST_ASSERT_EQUAL_INT(123, ParserUtils::parseInt("\t123"));
    
    // Invalid cases should return -1
    TEST_ASSERT_EQUAL_INT(-1, ParserUtils::parseInt(""));
    TEST_ASSERT_EQUAL_INT(-1, ParserUtils::parseInt("invalid"));
    TEST_ASSERT_EQUAL_INT(-1, ParserUtils::parseInt("12A"));
}

// Test ParserUtils::parseBool
void test_parseBool() {
    // Valid boolean values
    TEST_ASSERT_TRUE(ParserUtils::parseBool("1"));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("0"));
    
    // Invalid cases should return false
    TEST_ASSERT_FALSE(ParserUtils::parseBool(""));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("2"));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("true"));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("false"));
    TEST_ASSERT_FALSE(ParserUtils::parseBool("10"));
}

// Test ParserUtils::isValidCAT
void test_isValidCAT() {
    // Valid CAT commands
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("FA;"));
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("FA14074000;"));
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("MD1;"));
    TEST_ASSERT_TRUE(ParserUtils::isValidCAT("IF00014074000     +0000000000003000000;"));
    
    // Invalid cases
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT(""));
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("F"));
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("FA"));
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("FA14074000"));  // No semicolon
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("1A;"));         // First char not letter
    TEST_ASSERT_FALSE(ParserUtils::isValidCAT("F1;"));         // Second char not letter
}

// Test ParserUtils::getPrefix
void test_getPrefix() {
    // Valid prefixes
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("FA;") == "FA");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("MD1;") == "MD");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("TX;") == "TX");
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("IF00014074000;") == "IF");
    
    // Invalid cases should return empty
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("").empty());
    TEST_ASSERT_TRUE(ParserUtils::getPrefix("F").empty());
}

// Test ParserUtils::getParameters
void test_getParameters() {
    // Commands with parameters
    TEST_ASSERT_TRUE(ParserUtils::getParameters("FA14074000;") == "14074000");
    TEST_ASSERT_TRUE(ParserUtils::getParameters("MD1;") == "1");
    TEST_ASSERT_TRUE(ParserUtils::getParameters("AG0123;") == "0123");
    
    // Query commands (no parameters)
    TEST_ASSERT_TRUE(ParserUtils::getParameters("FA;").empty());
    TEST_ASSERT_TRUE(ParserUtils::getParameters("MD;").empty());
    
    // Invalid cases
    TEST_ASSERT_TRUE(ParserUtils::getParameters("").empty());
    TEST_ASSERT_TRUE(ParserUtils::getParameters("FA").empty());  // No semicolon
    TEST_ASSERT_TRUE(ParserUtils::getParameters("F;").empty());  // Too short
}

// Test ParserUtils::isQuery
void test_isQuery() {
    // Valid queries
    TEST_ASSERT_TRUE(ParserUtils::isQuery("FA;"));
    TEST_ASSERT_TRUE(ParserUtils::isQuery("MD;"));
    TEST_ASSERT_TRUE(ParserUtils::isQuery("TX;"));
    TEST_ASSERT_TRUE(ParserUtils::isQuery("EX006;"));
    TEST_ASSERT_TRUE(ParserUtils::isQuery("CD0;"));
    TEST_ASSERT_TRUE(ParserUtils::isQuery("CD1;"));
    
    // Not queries (have parameters)
    TEST_ASSERT_FALSE(ParserUtils::isQuery("FA14074000;"));
    TEST_ASSERT_FALSE(ParserUtils::isQuery("MD1;"));
    TEST_ASSERT_FALSE(ParserUtils::isQuery("AG0123;"));
    
    // Invalid cases
    TEST_ASSERT_FALSE(ParserUtils::isQuery(""));
    TEST_ASSERT_FALSE(ParserUtils::isQuery("FA"));
    TEST_ASSERT_FALSE(ParserUtils::isQuery("F;"));
}

// Test ParserUtils::trimTrailing
void test_trimTrailing() {
    // With trailing whitespace
    TEST_ASSERT_TRUE(ParserUtils::trimTrailing("test\n") == "test");
    TEST_ASSERT_TRUE(ParserUtils::trimTrailing("test\r\n") == "test");
    TEST_ASSERT_TRUE(ParserUtils::trimTrailing("test ") == "test");
    TEST_ASSERT_TRUE(ParserUtils::trimTrailing("test \n \r") == "test");
    
    // Without trailing whitespace
    TEST_ASSERT_TRUE(ParserUtils::trimTrailing("test") == "test");
    TEST_ASSERT_TRUE(ParserUtils::trimTrailing("") == "");
    
    // Only whitespace
    TEST_ASSERT_TRUE(ParserUtils::trimTrailing("\n\r ").empty());
}


// Test ParserUtils::sanitizeFrame - leading quirk
void test_sanitizeFrame_leading_quirk() {
    // Test stripping leading "?;" quirk
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("?;FA14070000;") == "FA14070000;");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("?;MD1;") == "MD1;");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("?;TX;") == "TX;");

    // Don't strip if "?;" is standalone (no command after)
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("?;") == "?;");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("?;;") == "?;;");
}

// Test ParserUtils::sanitizeFrame - trailing quirk
void test_sanitizeFrame_trailing_quirk() {
    // Test stripping trailing "?;" quirk
    // Note: Results in frames without terminator, which will fail validation
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("FA14070000?;") == "FA14070000");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("MD1?;") == "MD1");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("TX0?;") == "TX0");
}

// Test ParserUtils::sanitizeFrame - no quirk
void test_sanitizeFrame_no_quirk() {
    // Test frames without quirks remain unchanged
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("FA14070000;") == "FA14070000;");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("MD1;") == "MD1;");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("TX;") == "TX;");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("IF00014070000;") == "IF00014070000;");
}

// Test ParserUtils::sanitizeFrame - both quirks
void test_sanitizeFrame_both_quirks() {
    // Test frame with both leading and trailing quirks
    // Leading quirk removed first, then trailing quirk
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("?;FA14070000?;") == "FA14070000");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("?;MD1?;") == "MD1");
}

// Test ParserUtils::sanitizeFrame - edge cases
void test_sanitizeFrame_edge_cases() {
    // Test edge cases
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("").empty());
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame(";") == ";");
    TEST_ASSERT_TRUE(ParserUtils::sanitizeFrame("FA") == "FA");  // No terminator
}

// Register test functions
extern "C" void run_cat_parser_utils_tests(void) {
    RUN_TEST(test_parseFrequency);
    RUN_TEST(test_parseInt);
    RUN_TEST(test_parseBool);
    RUN_TEST(test_isValidCAT);
    RUN_TEST(test_getPrefix);
    RUN_TEST(test_getParameters);
    RUN_TEST(test_isQuery);
    RUN_TEST(test_trimTrailing);
    RUN_TEST(test_sanitizeFrame_leading_quirk);
    RUN_TEST(test_sanitizeFrame_trailing_quirk);
    RUN_TEST(test_sanitizeFrame_no_quirk);
    RUN_TEST(test_sanitizeFrame_both_quirks);
    RUN_TEST(test_sanitizeFrame_edge_cases);
}
