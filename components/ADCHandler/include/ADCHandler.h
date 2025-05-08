#pragma once

#include "soc/gpio_num.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

namespace radio {
    class RadioManager;
}

class ADCHandler {
public:
    ADCHandler(radio::RadioManager *radioManager);
    ~ADCHandler();

    struct ADCValues {
        // Note: AF/RF gain, IF shift, and RIT/XIT now controlled by encoders
        // Kept for future analog inputs if needed
    };

    void start();
    bool hasChanged() const;
    ADCValues getValues() const;


private:
    radio::RadioManager &m_radioManager;
    ADCValues m_currentValues;
    ADCValues m_previousValues;
    bool m_hasChanged;
    adc_oneshot_unit_handle_t m_adc1_handle;
    TaskHandle_t adcTaskHandle;

    static void adcTaskFunction(void* parameter);
    static adc_channel_t gpioToAdcChannel(gpio_num_t gpio);
    int readAndMap(gpio_num_t pin, int minVal, int maxVal) const;
    static int map(int x, int in_min, int in_max, int out_min, int out_max);
    void update();
    void handleChange();
};
