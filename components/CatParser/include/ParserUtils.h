#pragma once

#include <string_view>
#include <string>
#include <charconv>
#include <cstdint>
#include <cctype>
#include <vector>

#include "esp_log.h"

namespace radio::cat {
    /**
     * @brief Shared CAT parsing utilities for ESP32S3 optimized performance
     *
     * This header-only library provides common parsing functions used by both
     * LocalCATHandler and RadioCATHandler. All functions are inline for optimal
     * performance and minimal memory overhead.
     */
    class ParserUtils {
    public:
        /**
         * @brief Parse frequency value from string
         * @param freqStr String view containing frequency digits
         * @return Parsed frequency in Hz, or 0 on error
         */
        static inline uint64_t parseFrequency(std::string_view freqStr) {
            uint64_t freq = 0;
            auto result = std::from_chars(freqStr.data(), freqStr.data() + freqStr.size(), freq);
            // Check that parsing succeeded AND consumed the entire string
            return (result.ec == std::errc{} && result.ptr == freqStr.data() + freqStr.size()) ? freq : 0;
        }

        /**
         * @brief Parse integer value from string with optional whitespace handling
         * @param valStr String view containing integer
         * @return Parsed integer, or -1 on error
         */
        static inline int parseInt(std::string_view valStr) {
            // Trim leading whitespace as std::from_chars is strict
            if (const size_t first_char = valStr.find_first_not_of(" \t"); first_char != std::string_view::npos) {
                valStr = valStr.substr(first_char);
            }

            int value = 0;
            auto result = std::from_chars(valStr.data(), valStr.data() + valStr.size(), value);
            // Check that parsing succeeded AND consumed the entire string
            return (result.ec == std::errc{} && result.ptr == valStr.data() + valStr.size()) ? value : -1;
        }

        /**
         * @brief Parse boolean value from single character
         * @param valStr String view containing '0' or '1'
         * @return true if '1', false otherwise
         */
        static inline bool parseBool(std::string_view valStr) {
            return valStr.length() == 1 && valStr[0] == '1';
        }

        /**
         * @brief Validate basic CAT command/response format
         * @param cmd Command string to validate
         * @return true if valid format (2+ chars, ends with ';', starts with letters)
         */
        static inline bool isValidCAT(std::string_view cmd) {
            if (cmd == ";") {
                return true;
            }
            // Handle error responses (these are valid responses from radio)
            if (cmd == "?;" || cmd == "E;" || cmd == "O;") {
                return true;
            }

            // Standard CAT command validation (minimum 3 chars: 2 letters + semicolon)
            if (cmd.length() < 3) {
                return false;
            }

            if (!cmd.ends_with(";")) {
                return false;
            }

            // Check that first two characters are letters for standard CAT commands
            return std::isalpha(static_cast<unsigned char>(cmd[0])) &&
                   std::isalpha(static_cast<unsigned char>(cmd[1]));
        }

        /**
         * @brief Sanitize CAT frame by removing radio firmware quirks
         *
         * The TS-590SG sometimes emits malformed error markers like "?;COMMAND;"
         * or "RESPONSE?;" which are protocol-level quirks that should be handled
         * at the parser layer, not the transport layer.
         *
         * Leading "?;" before a valid command is stripped to recover the command.
         * Trailing "?;" is stripped entirely (both '?' and ';'), producing an
         * unterminated frame that will fail CAT validation and be dropped.
         * This is intentional: trailing "?;" indicates a radio error condition.
         *
         * Examples:
         *   "?;FA14070000;" -> "FA14070000;" (leading error marker stripped)
         *   "FA14070000?;" -> "FA14070000"   (error frame, will fail validation)
         *   "FA14070000;" -> "FA14070000;"   (unchanged)
         *
         * @param frame CAT frame to sanitize
         * @return Sanitized frame view (may point into original frame)
         */
        static inline std::string_view sanitizeFrame(std::string_view frame) {
            // Strip leading "?;" if it appears before another command
            // Example: "?;FA14070000;" -> "FA14070000;"
            if (frame.length() > 3 && frame.starts_with("?;") && frame[2] != ';') {
                frame.remove_prefix(2);
            }

            // Strip trailing "?;" quirk from radio responses
            // Removes both '?' and ';', producing an unterminated frame that will
            // fail CAT validation. This effectively drops error-tagged responses.
            if (frame.length() > 2 && frame.ends_with("?;")) {
                frame.remove_suffix(2);
            }

            return frame;
        }

        /**
         * @brief Extract command prefix (first 2 characters)
         * @param cmd Command string
         * @return Prefix as string_view, or empty if invalid
         */
        static inline std::string_view getPrefix(std::string_view cmd) {
            return (cmd.length() >= 2) ? cmd.substr(0, 2) : std::string_view{};
        }

        /**
         * @brief Extract parameters between command prefix and semicolon
         * @param cmd Command string
         * @return Parameters as string_view, or empty if invalid
         */
        static inline std::string_view getParameters(std::string_view cmd) {
            if (cmd.length() < 3 || !cmd.ends_with(";")) {
                return std::string_view{};
            }
            return cmd.substr(2, cmd.length() - 3);
        }

        /**
         * @brief Check if command is a query (no parameters)
         * @param cmd Command string
         * @return true if query format (e.g., "FA;")
         */
        static inline bool isQuery(std::string_view cmd) {
            if (cmd.length() < 3 || !cmd.ends_with(";")) {
                return false;
            }

            // Most queries are 3 characters long (e.g., "FA;")
            if (cmd.length() == 3 && cmd.ends_with(';')) {
                return true;
            }

            // Some queries are longer (e.g., "EX006;")
            if (cmd.starts_with("EX") && cmd.length() == 6) {
                return true;
            }
            if (cmd.starts_with("CD0;") || cmd.starts_with("CD1;")) {
                return true;
            }

            return false;
        }

        /**
         * @brief Remove trailing whitespace from string_view
         * @param str String to trim
         * @return Trimmed string_view
         */
        static inline std::string_view trimTrailing(std::string_view str) {
            while (!str.empty() && (str.back() == '\n' || str.back() == '\r' || str.back() == ' ' || str.back() ==
                                    '\t')) {
                str.remove_suffix(1);
            }
            return str;
        }

        /**
         * @brief Split CAT message into individual commands/responses
         * @param message The complete message containing one or more CAT commands
         * @param callback Function to call for each parsed command
         * @return Number of commands found
         */
        template<typename Callback>
        static size_t splitCATMessage(std::string_view message, Callback &&callback) {
            size_t start = 0;
            size_t commandCount = 0;

            while (start < message.length()) {
                // Skip any leading whitespace/control bytes (treat anything <= ' ' as skippable),
                // but do not skip a leading ';' which is a valid wakeup command.
                while (start < message.length()) {
                    unsigned char c = static_cast<unsigned char>(message[start]);
                    if (c == ';') break;
                    if (c <= ' ') {
                        start++;
                        continue;
                    }
                    break;
                }

                if (start >= message.length()) break;

                // Find the end of current command (look for ';')
                size_t end = message.find(';', start);
                if (end == std::string_view::npos) {
                    // No semicolon found, treat rest as single command
                    end = message.length();
                } else {
                    end++; // Include the semicolon
                }

                // Extract command
                std::string_view command = message.substr(start, end - start);

                // Skip empty commands and whitespace-only, but allow ";" (wakeup command)
                if (!command.empty() && command != "\n" && command != "\r\n") {
                    // Trim trailing whitespace before processing
                    command = trimTrailing(command);
                    // Process non-empty commands, including ";" wakeup command
                    if (!command.empty()) {
                        callback(command);
                        commandCount++;
                    }
                }

                start = end;
            }

            return commandCount;
        }

        /**
         * @brief Validate and parse a simple integer parameter command
         * @param cmd Command string
         * @param prefix Expected command prefix (e.g., "AG")
         * @param minVal Minimum valid value
         * @param maxVal Maximum valid value
         * @param value Output parameter for parsed value
         * @return true if valid and parsed successfully
         */
        static inline bool parseSimpleIntCommand(std::string_view cmd, std::string_view prefix,
                                                 int minVal, int maxVal, int &value) {
            if (!cmd.starts_with(prefix) || cmd.length() < prefix.length() + 1) {
                return false;
            }

            // Handle query format (e.g., "AG;")
            if (cmd.length() == prefix.length() + 1 && cmd.ends_with(";")) {
                value = -1; // Indicate query
                return true;
            }

            // Parse the parameter
            std::string_view param = getParameters(cmd);
            if (param.empty()) {
                return false;
            }

            value = parseInt(param);
            return value >= minVal && value <= maxVal;
        }

        /**
         * @brief Parse frequency command (FA/FB format)
         * @param cmd Command string
         * @param isVfoA true for FA, false for FB
         * @param frequency Output parameter for parsed frequency
         * @return 0=invalid, 1=query, 2=set command
         */
        static inline int parseFrequencyCommand(std::string_view cmd, bool isVfoA, uint64_t &frequency) {
            const std::string_view prefix = isVfoA ? "FA" : "FB";

            if (!cmd.starts_with(prefix)) {
                return 0; // Invalid
            }

            // Query format: "FA;" or "FB;" (exactly 3 chars)
            if (cmd.length() == 3 && cmd.back() == ';') {
                return 1; // Query
            }

            if (cmd.length() >= 14) {
                // FAxxxxxxxxxx; format
                std::string_view param = getParameters(cmd);
                frequency = parseFrequency(param);
                if (frequency > 0) {
                    return 2; // Set command
                }
            }

            return 0; // Invalid
        }

        /**
         * @brief Build IF response string for testing
         * @param vfoAFrequency VFO A frequency in Hz
         * @param ritXitOffset RIT/XIT offset (-9999 to 9999)
         * @param ritOn RIT enabled state
         * @param xitOn XIT enabled state
         * @param memoryChannel Memory channel number
         * @param isTx Transmit state
         * @param mode Operating mode
         * @param split Split operation state
         * @param toneEnabled Tone enabled state
         * @param toneNumber Tone number
         * @return IF response string
         */
        static inline std::string buildIFResponse(
            uint64_t vfoAFrequency = 14074000,
            int32_t ritXitOffset = 0,
            bool ritOn = false,
            bool xitOn = false,
            int memoryChannel = 22,
            bool isTx = false,
            int mode = 3,
            bool split = false,
            int toneEnabled = 0,
            int toneNumber = 0
        ) {
            std::string ifResponse;
            ifResponse.reserve(40);
            
            ifResponse = "IF";
            
            char freqBuf[12];
            std::snprintf(freqBuf, sizeof(freqBuf), "%011llu", vfoAFrequency);
            ifResponse += freqBuf;
            
            ifResponse += "     ";
            
            char offsetBuf[6];
            if (ritXitOffset >= 0) {
                int32_t clampedOffset = std::min(ritXitOffset, static_cast<int32_t>(9999));
                std::snprintf(offsetBuf, sizeof(offsetBuf), " %04d", clampedOffset);
            } else {
                int32_t clampedOffset = std::max(ritXitOffset, static_cast<int32_t>(-9999));
                std::snprintf(offsetBuf, sizeof(offsetBuf), "-%04d", -clampedOffset);
            }
            ifResponse += offsetBuf;
            
            ifResponse += ritOn ? '1' : '0';
            ifResponse += xitOn ? '1' : '0';
            
            char memBuf[4];
            if (memoryChannel < 0) memoryChannel = 22;
            std::snprintf(memBuf, sizeof(memBuf), "%03d", memoryChannel);
            ifResponse += memBuf;
            
            ifResponse += isTx ? '1' : '0';
            
            char modeBuf[4];
            std::snprintf(modeBuf, sizeof(modeBuf), "%d", mode);
            ifResponse += modeBuf;
            
            ifResponse += '0';
            ifResponse += '0';
            ifResponse += split ? '1' : '0';
            
            char toneEnabledBuf[4];
            std::snprintf(toneEnabledBuf, sizeof(toneEnabledBuf), "%d", toneEnabled);
            ifResponse += toneEnabledBuf;
            
            char toneNumBuf[3];
            std::snprintf(toneNumBuf, sizeof(toneNumBuf), "%02d", toneNumber);
            ifResponse += toneNumBuf;
            
            ifResponse += '0';
            ifResponse += ';';
            
            return ifResponse;
        }

        static bool vectorContains(const std::vector<std::string> & vector, const char * str);
    };
} // namespace radio::cat
