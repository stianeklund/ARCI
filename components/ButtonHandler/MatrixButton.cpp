#include "include/MatrixButton.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "soc/gpio_num.h"

static const char* TAG = "MatrixButton";

class TCA8418Handler; // Forward declaration

MatrixButton::MatrixButton(const TCA8418Handler::MatrixKey key, const uint32_t debounceTime, const uint32_t longPressTime)
    : m_button(GPIO_NUM_NC, debounceTime, longPressTime)  // Use dummy GPIO pin
    , m_key(key)
    , m_lastTcaState(true)  // TCA8418 starts with released state (true)
{
}

void MatrixButton::updateState(const bool pressed) {
    // TCA8418 logic: pressed=false means button down, pressed=true means button up
    // Button class logic: true means pressed, false means released
    // So we need to invert the TCA8418 state

    const bool buttonPressed = !pressed;
    
    // Only update if state actually changed
    if (pressed != m_lastTcaState) {
        m_lastTcaState = pressed;
        
        ESP_LOGD(TAG, "MatrixButton state change: TCA pressed=%s -> buttonPressed=%s", 
                 pressed ? "true" : "false", buttonPressed ? "true" : "false");
        
        // Use Button's new event-driven methods
        if (buttonPressed) {
            m_button.handlePress();
            ESP_LOGD(TAG, "Called handlePress()");
        } else {
            m_button.handleRelease();
            ESP_LOGD(TAG, "Called handleRelease()");
        }
    }
    
    // Always call update to handle timing logic (for long press detection)
    m_button.update();
}

void MatrixButton::update() {
    m_button.update();
}

bool MatrixButton::wasPressed() {
    return m_button.wasPressed();
}

bool MatrixButton::wasReleased() {
    return m_button.wasReleased();
}

bool MatrixButton::isPressed() const {
    return m_button.isPressed();
}

bool MatrixButton::isLongPressed() const {
    return m_button.isLongPressed();
}

bool MatrixButton::hasStateChanged() const {
    return m_button.hasStateChanged();
}

int64_t MatrixButton::pressedTime() const {
    return m_button.pressedTime();
}

bool MatrixButton::wasLongPressed() {
    return m_button.wasLongPressed();
}

bool MatrixButton::wasShortPressed() {
    return m_button.wasShortPressed();
}

bool MatrixButton::wasShortReleased() {
    return m_button.wasShortReleased();
}

bool MatrixButton::wasLongReleased() {
    return m_button.wasLongReleased();
}

#ifdef CONFIG_RUN_UNIT_TESTS
void MatrixButton::simulatePress() {
    // Delegate to the internal button for state management
    m_button.simulatePress();
}

void MatrixButton::simulateRelease() {
    // Delegate to the internal button for state management
    m_button.simulateRelease();
}
#endif

inline int64_t getMillis() {
    return esp_timer_get_time() / 1000;
}