#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @file FrequencyFormatter.h
 * @brief Shared CAT frequency formatting utilities
 *
 * Header-only component providing zero-allocation, high-performance frequency
 * formatting for CAT protocol commands. Replaces scattered snprintf() calls
 * with a unified, optimized implementation.
 *
 * Design rationale:
 * - Header-only: No compilation overhead, enables inlining
 * - Zero-allocation: Uses internal buffer, no heap pressure
 * - Fast: Custom reverse decimal conversion avoids snprintf overhead
 * - Thread-safe: When used as thread_local (recommended pattern)
 *
 * Usage:
 * @code
 * static thread_local FrequencyFormatter formatter;
 * const char* cmd = formatter.formatCommand("FA", 14074000);
 * // Result: "FA00014074000;"
 * @endcode
 *
 * CAT frequency format:
 * - 11 digits, zero-padded (e.g., 00014074000 for 14.074 MHz)
 * - Transmitted as Hz (14074000 Hz = 14.074 MHz)
 * - Commands: FA (VFO A), FB (VFO B)
 */
class FrequencyFormatter {
    static constexpr size_t BUFFER_SIZE = 16; // "FA" + 11 digits + ";" + null
    mutable char buffer_[BUFFER_SIZE]{};

public:
    /**
     * @brief Format frequency as 11-digit zero-padded string
     * @param freq Frequency in Hz
     * @return Pointer to null-terminated string (valid until next call)
     * @note Result: "00014074000" for 14.074 MHz
     */
    [[nodiscard]] const char* format(uint64_t freq) const {
        char* p = buffer_ + BUFFER_SIZE - 1;
        *p = '\0';

        // Fast reverse decimal conversion (11 digits, zero-padded)
        for (int i = 0; i < 11; ++i) {
            *--p = '0' + (freq % 10);
            freq /= 10;
        }
        return p;
    }

    /**
     * @brief Format complete CAT frequency command
     * @param prefix Command prefix (typically "FA" or "FB")
     * @param freq Frequency in Hz
     * @return Pointer to null-terminated CAT command (valid until next call)
     * @note Result: "FA00014074000;" for VFO A at 14.074 MHz
     */
    [[nodiscard]] const char* formatCommand(const char* prefix, uint64_t freq) const {
        char* p = buffer_ + BUFFER_SIZE - 1;
        *p = '\0';
        *--p = ';';

        // Fast reverse decimal conversion for frequency (11 digits)
        for (int i = 0; i < 11; ++i) {
            *--p = '0' + (freq % 10);
            freq /= 10;
        }

        // Prepend command prefix (typically 2 chars: "FA" or "FB")
        *--p = prefix[1];
        *--p = prefix[0];
        return p;
    }

    /**
     * @brief Format VFO A frequency command
     * @param freq Frequency in Hz
     * @return Pointer to "FA..." CAT command (valid until next call)
     */
    [[nodiscard]] const char* formatFA(uint64_t freq) const {
        return formatCommand("FA", freq);
    }

    /**
     * @brief Format VFO B frequency command
     * @param freq Frequency in Hz
     * @return Pointer to "FB..." CAT command (valid until next call)
     */
    [[nodiscard]] const char* formatFB(uint64_t freq) const {
        return formatCommand("FB", freq);
    }
};
