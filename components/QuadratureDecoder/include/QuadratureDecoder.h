#pragma once

#include <cstdint>

/**
 * @file QuadratureDecoder.h
 * @brief Universal quadrature encoder state transition decoder
 *
 * Provides a shared implementation of 2-bit Gray code quadrature decoding
 * for all standard rotary encoders (KY-040, EC11E, Bourns, etc.).
 *
 * The quadrature state transition table represents the electrical protocol,
 * not hardware-specific behavior. All 2-bit quadrature encoders use the same
 * Gray code state machine, regardless of manufacturer or physical design.
 *
 * Hardware-specific features (glitch filtering, debouncing, acceleration)
 * remain in the respective encoder handlers.
 */

namespace quadrature {

/**
 * @brief Universal 2-bit Gray code quadrature state transition table
 *
 * Index format: (oldState << 2) | newState
 * Returns: +1 (CW rotation), -1 (CCW rotation), 0 (no change or invalid)
 *
 * State encoding:
 *   00 (0): Both pins LOW
 *   01 (1): A=LOW, B=HIGH
 *   10 (2): A=HIGH, B=LOW
 *   11 (3): Both pins HIGH
 *
 * Valid transitions in Gray code:
 *   CW:  00 -> 10 -> 11 -> 01 -> 00
 *   CCW: 00 -> 01 -> 11 -> 10 -> 00
 *
 * This table works for all standard quadrature encoders because the
 * electrical signal pattern is standardized, not hardware-specific.
 */
static constexpr int8_t TRANSITION_TABLE[16] = {
    // From state 00 (both LOW)
    0,  // 00 -> 00: no change
    -1, // 00 -> 01: CCW step (A stays LOW, B rises)
    +1, // 00 -> 10: CW step (A rises, B stays LOW)
    0,  // 00 -> 11: invalid (both rise simultaneously)

    // From state 01 (A=LOW, B=HIGH)
    +1, // 01 -> 00: CW step (B falls while A stays LOW)
    0,  // 01 -> 01: no change
    0,  // 01 -> 10: invalid (both change)
    -1, // 01 -> 11: CCW step (A rises while B stays HIGH)

    // From state 10 (A=HIGH, B=LOW)
    -1, // 10 -> 00: CCW step (A falls while B stays LOW)
    0,  // 10 -> 01: invalid (both change)
    0,  // 10 -> 10: no change
    +1, // 10 -> 11: CW step (B rises while A stays HIGH)

    // From state 11 (both HIGH)
    0,  // 11 -> 00: invalid (both fall simultaneously)
    +1, // 11 -> 01: CW step (A falls while B stays HIGH)
    -1, // 11 -> 10: CCW step (B falls while A stays HIGH)
    0   // 11 -> 11: no change
};

/**
 * @brief Decode quadrature state transition
 *
 * Performs O(1) lookup to determine rotation direction from state change.
 * Thread-safe (read-only access to constexpr table).
 *
 * @param oldState Previous 2-bit encoder state (0-3)
 * @param newState Current 2-bit encoder state (0-3)
 * @return Direction: +1 (clockwise), -1 (counter-clockwise), 0 (no change)
 *
 * Example usage:
 * @code
 * uint8_t oldState = (oldA << 1) | oldB;  // Previous state: 0b00 to 0b11
 * uint8_t newState = (newA << 1) | newB;  // Current state: 0b00 to 0b11
 * int8_t delta = quadrature::decode(oldState, newState);
 * if (delta != 0) {
 *     position += delta;  // Update position counter
 * }
 * @endcode
 */
constexpr inline int8_t decode(const uint8_t oldState, const uint8_t newState) {
    // Validate state range (0-3)
    if (oldState > 3 || newState > 3) {
        return 0;  // Invalid state, no movement
    }

    // Lookup table index: combine old state (bits 3-2) with new state (bits 1-0)
    const uint8_t index = (oldState << 2) | newState;
    return TRANSITION_TABLE[index];
}

/**
 * @brief Get human-readable state name for debugging
 *
 * @param state Encoder state (0-3)
 * @return State description string
 */
constexpr inline const char* stateName(const uint8_t state) {
    switch (state) {
        case 0: return "00_LOW_LOW";
        case 1: return "01_LOW_HIGH";
        case 2: return "10_HIGH_LOW";
        case 3: return "11_HIGH_HIGH";
        default: return "INVALID";
    }
}

} // namespace quadrature
