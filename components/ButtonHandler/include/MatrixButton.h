#pragma once

#include "Button.h"
#include "TCA8418Handler.h"

/**
 * @brief Wrapper class that provides Button-like interface for TCA8418 matrix keys
 * 
 * This class adapts the TCA8418 raw key events to use the existing Button class
 * logic for debouncing, long press detection, and state management.
 */

class MatrixButton {
public:
    explicit MatrixButton(TCA8418Handler::MatrixKey key, uint32_t debounceTime = 50, uint32_t longPressTime = 500);

    /**
     * @brief Update the button state based on TCA8418 key event
     * @param pressed The raw pressed state from TCA8418 (false = pressed, true = released)
     */
    void updateState(bool pressed);
    
    // Delegate to Button class methods
    void update();             // Update timing logic (for long press detection)
    bool wasPressed();
    bool wasReleased(); 
    [[nodiscard]] bool isPressed() const;
    [[nodiscard]] bool isLongPressed() const;
    [[nodiscard]] bool hasStateChanged() const;
    [[nodiscard]] int64_t pressedTime() const;
    
    // New long press handling methods
    bool wasLongPressed();     // Returns true once when long press is first detected
    bool wasShortPressed();    // Returns true once when short press is completed (on release)
    bool wasShortReleased();   // Returns true once when released after short press
    bool wasLongReleased();    // Returns true once when released after long press

    [[nodiscard]] TCA8418Handler::MatrixKey getKey() const { return m_key; }
    
    // Allow access to TCA state for transition detection
    [[nodiscard]] bool getLastTcaState() const { return m_lastTcaState; }

#ifdef CONFIG_RUN_UNIT_TESTS
    // Test methods for unit testing
    void simulatePress();
    void simulateRelease();
#endif

private:
    Button m_button;
    TCA8418Handler::MatrixKey m_key;
    bool m_lastTcaState;  // Track last TCA8418 state to detect changes
};