#pragma once

#include "driver/gpio.h"
#include "esp_system.h"

class Button {
public:
    // Constructor with default debounce and long press times  
    // NOTE: pin parameter kept for compatibility but not used for TCA8418 buttons
    explicit Button(gpio_num_t pin, uint32_t debounceTime = 50, uint32_t longPressTime = 1000);
    
    // Event-driven state updates (called by MatrixButton or hardware interrupt)
    void update();  // Still needed for long press timing logic
    void handlePress();   // Called when physical button press detected
    void handleRelease(); // Called when physical button release detected

    // Accessor methods for button state
    [[nodiscard]] bool isPressed() const;
    [[nodiscard]] bool hasStateChanged() const;
    bool wasPressed();
    bool wasReleased();
    [[nodiscard]] bool isLongPressed() const;
    [[nodiscard]] int64_t pressedTime() const;
    
    // New methods for proper long press handling
    bool wasLongPressed();     // Returns true once when long press is first detected
    bool wasShortPressed();    // Returns true once when short press is completed (on release)
    bool wasShortReleased();   // Returns true once when released after short press
    bool wasLongReleased();    // Returns true once when released after long press


    void updateState(const Button &other) {
        // Update mutable state only:
        m_state = other.m_state;
        m_lastState = other.m_lastState;
        m_stateChanged = other.m_stateChanged;
        m_lastDebounceTime = other.m_lastDebounceTime;
        m_pressedTime = other.m_pressedTime;
        m_longPressed = other.m_longPressed;
    }
    void simulateRelease();
    void simulatePress();

private:
    // Hardware configuration
    const gpio_num_t m_pin;
    const uint32_t m_debounceTime;
    const uint32_t m_longPressTime;

    // State tracking
    bool m_state;             // Current stable state
    bool m_lastState;         // Last raw reading state
    bool m_stateChanged;      // Flag to indicate a change in state (debounced)
    int64_t m_lastDebounceTime; // Last time the state was checked (for debounce)
    int64_t m_pressedTime;      // Timestamp when button was pressed
    bool m_longPressed;         // Flag to indicate if long press occurred
    bool m_longPressReported;   // Flag to prevent multiple long press reports
    bool m_shortPressReported;  // Flag to prevent short press if long press occurred
    bool m_wasReleased;         // Track if button was just released
    bool m_releasedReported;    // Track if release event was consumed
    bool m_shortReleasedReported;  // Track if short release event was consumed
    bool m_longReleasedReported;   // Track if long release event was consumed
};
