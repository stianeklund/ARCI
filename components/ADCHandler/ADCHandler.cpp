#include "ADCHandler.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "RadioManager.h"
#include "../../include/pin_definitions.h"
#include <sstream>
#include <iomanip>
#include <cmath>

static const char* TAG = "ADCHandler";

ADCHandler::ADCHandler(radio::RadioManager *radioManager)
    : m_radioManager(*radioManager),
      m_currentValues(),
      m_previousValues(),
      m_hasChanged(false),
      m_adc1_handle(nullptr),
      adcTaskHandle(nullptr) {

    ESP_LOGI(TAG, "ADCHandler initialized (AF/RF gain, IF shift, RIT/XIT now controlled by encoders)");

    adc_oneshot_unit_init_cfg_t init_config1 = {};
    init_config1.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &m_adc1_handle));

    // Configuration for future analog inputs if needed
    // Currently no ADC channels configured as encoders replaced potentiometers
}

ADCHandler::~ADCHandler() {
    if (m_adc1_handle) {
        adc_oneshot_del_unit(m_adc1_handle);
    }
    if (adcTaskHandle) {
        vTaskDelete(adcTaskHandle);
    }
    // Queue no longer used
}

void ADCHandler::start() {
    xTaskCreate(adcTaskFunction, "ADC_Task", 4096, this, 5, &adcTaskHandle); // Priority 5 for reliable sampling
}

bool ADCHandler::hasChanged() const {
    return m_hasChanged;
}

ADCHandler::ADCValues ADCHandler::getValues() const {
    return m_currentValues;
}

void ADCHandler::adcTaskFunction(void* parameter) {
    ADCHandler* handler = static_cast<ADCHandler*>(parameter);
    while (true) {
        handler->update();
        if (handler->hasChanged()) {
            handler->handleChange();
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Faster sampling for smoother response
    }
}

adc_channel_t ADCHandler::gpioToAdcChannel(const gpio_num_t gpio) {
    // Map GPIO pins to their correct ADC1 channels based on ESP32 variant
#if defined(CONFIG_IDF_TARGET_ESP32)
    // Original ESP32 mappings
    switch (gpio) {
        case GPIO_NUM_33: return ADC_CHANNEL_5;  // PIN_AF_GAIN
        case GPIO_NUM_32: return ADC_CHANNEL_4;  // PIN_RF_GAIN
        case GPIO_NUM_35: return ADC_CHANNEL_7;  // PIN_IF_SHIFT
        case GPIO_NUM_34: return ADC_CHANNEL_6;  // PIN_RIT_XIT
        default: return static_cast<adc_channel_t>(0);
    }
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3 mappings (GPIO_n maps to ADC1_CH_n for GPIO 1-10)
    // NOTE: GPIO6-9 freed for encoders (RF/AF gain, RIT/XIT, IF_SHIFT now encoder-controlled)
    switch (gpio) {
        case GPIO_NUM_10: return ADC_CHANNEL_9;  // PIN_ANALOG_OUT (only remaining ADC)
        // GPIO4, 5 also available if needed in future
        case GPIO_NUM_4:  return ADC_CHANNEL_3;  // PIN_TRANSVERTER_MACRO (if used)
        case GPIO_NUM_5:  return ADC_CHANNEL_4;  // PIN_TF_SET (if used)
        default: return static_cast<adc_channel_t>(0);
    }
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
    // ESP32-P4 mappings - adjust channels to match your hardware
    switch (gpio) {
        case GPIO_NUM_10: return ADC_CHANNEL_0;  // PIN_ANALOG_OUT
        default: return static_cast<adc_channel_t>(0);
    }
#else
#error "Unsupported ESP32 variant for ADC channel mapping"
#endif
}

int ADCHandler::readAndMap(const gpio_num_t pin, const int minVal, const int maxVal) const {
    int adcRaw;
    ESP_ERROR_CHECK(adc_oneshot_read(m_adc1_handle, gpioToAdcChannel(pin), &adcRaw));
    return map(adcRaw, 0, 4095, minVal, maxVal); // 12-bit ADC, so max value is 4095
}


int ADCHandler::map(const int x, const int in_min, const int in_max, const int out_min, const int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void ADCHandler::update() {
    m_previousValues = m_currentValues;

    // No analog inputs currently configured
    // AF/RF gain, IF shift, and RIT/XIT now controlled by encoders
    m_hasChanged = false;
}

void ADCHandler::handleChange() {
    // No analog inputs currently configured
    // AF/RF gain, IF shift, and RIT/XIT now controlled by encoders
    // Reserved for future analog input handling if needed
}
