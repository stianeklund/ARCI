#include "Button.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "Button";

// Helper: Convert ESP timer value to milliseconds
inline int64_t getMillis() {
    return esp_timer_get_time() / 1000;
}

Button::Button(gpio_num_t pin, uint32_t debounceTime, uint32_t longPressTime)
    : m_pin(pin),
      m_debounceTime(debounceTime),
      m_longPressTime(longPressTime),
      m_state(false),
      m_lastState(false),
      m_stateChanged(false),
      m_lastDebounceTime(0),
      m_pressedTime(0),
      m_longPressed(false),
      m_longPressReported(false),
      m_shortPressReported(false),
      m_wasReleased(false),
      m_releasedReported(false),
      m_shortReleasedReported(false),
      m_longReleasedReported(false) {
}

bool Button::hasStateChanged() const { return m_stateChanged;}

void Button::update() {
    // Only handle long press timing logic now - no GPIO reading
    const int64_t now = getMillis();
    
    // Check for a long press event if the button remains pressed
    if (m_state && !m_longPressed && (now - m_pressedTime >= m_longPressTime)) {
        m_longPressed = true;
        m_longPressReported = false; // Reset report flag when long press is detected
    }
}

void Button::handlePress() {
    const int64_t now = getMillis();
    
    // Only update if not already pressed (debouncing handled by TCA8418)
    if (!m_state) {
        m_state = true;
        m_stateChanged = true;
        m_pressedTime = now;
        m_longPressed = false;
        m_longPressReported = false;
        m_shortPressReported = false;
        m_wasReleased = false;
        m_releasedReported = false;
        m_shortReleasedReported = false;
        m_longReleasedReported = false;
    }
}

void Button::handleRelease() {
    // Only update if currently pressed (debouncing handled by TCA8418)
    if (m_state) {
        m_state = false;
        m_stateChanged = true;
        m_wasReleased = true;
        m_releasedReported = false;
        // Don't reset m_longPressed yet - we need it for wasShortPressed logic
    }
}

bool Button::isPressed() const {
    return m_state;
}

bool Button::wasPressed() {
    if (m_stateChanged && m_state) {
        m_stateChanged = false;
        return true;
    }
    return false;
}

bool Button::wasReleased() {
    if (m_stateChanged && !m_state) {
        m_stateChanged = false;
        return true;
    }
    return false;
}

bool Button::isLongPressed() const {
    return m_longPressed;
}

int64_t Button::pressedTime() const {
    return m_pressedTime;
}

void Button::simulatePress() {
    // Only set state change if we're not already pressed
    if (!m_state) {
        m_state = true;
        m_stateChanged = true;
        m_pressedTime = getMillis(); // Capture the press time for long press logic
        m_longPressed = false; // Reset long press flag on new press
        m_longPressReported = false;
        m_shortPressReported = false;
        m_wasReleased = false;
        m_releasedReported = false;
        m_shortReleasedReported = false;
        m_longReleasedReported = false;
    }
}

bool Button::wasLongPressed() {
    // Return true once when long press is first detected
    if (m_longPressed && !m_longPressReported) {
        m_longPressReported = true;
        m_shortPressReported = true; // Prevent short press from triggering
        return true;
    }
    return false;
}

bool Button::wasShortPressed() {
    // Return true once when button is released after a short press (not long press)
    if (m_wasReleased && !m_releasedReported && !m_shortPressReported) {
        m_releasedReported = true;
        m_shortPressReported = true;
        m_wasReleased = false;
        // Reset long press state now that we've handled the release
        m_longPressed = false;
        m_longPressReported = false;
        return true;
    }
    return false;
}

void Button::simulateRelease() {
    // Only set state change if we're currently pressed
    if (m_state) {
        m_state = false;
        m_stateChanged = true;
        m_wasReleased = true;
        m_releasedReported = false;
        // Don't reset m_longPressed yet - we need it for release logic
    }
}

bool Button::wasShortReleased() {
    // Return true once when button is released after a short press (not long press)
    if (m_wasReleased && !m_longPressed && !m_shortReleasedReported) {
        m_shortReleasedReported = true;
        m_wasReleased = false;
        m_releasedReported = true;
        // Reset all press flags now that we've handled the short release
        m_shortPressReported = true;
        m_longPressReported = false;
        return true;
    }
    return false;
}

bool Button::wasLongReleased() {
    // Return true once when button is released after a long press
    if (m_wasReleased && m_longPressed && !m_longReleasedReported) {
        m_longReleasedReported = true;
        m_wasReleased = false;
        m_releasedReported = true;
        // Reset long press state now that we've handled the long release
        m_longPressed = false;
        m_longPressReported = false;
        m_shortPressReported = false;
        return true;
    }
    return false;
}