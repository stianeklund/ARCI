#pragma once

#include "sdkconfig.h"
#include "esp_err.h"
#include "RadioManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class NvsManager {
public:
    NvsManager(radio::RadioManager& radioManager);
    esp_err_t init();
    esp_err_t saveRadioState();
    esp_err_t loadRadioState();
    void setupPowerStateCallback();

    // Startup sequence: load -> sync with radio -> save
    esp_err_t loadAndSyncOnStartup();

    /**
     * @brief Load button mode memory from NVS
     * @param modeMemory Array to populate with mode data (size must be 11)
     * @return ESP_OK if loaded successfully, ESP_ERR_NOT_FOUND if no data, ESP_FAIL on error
     */
    esp_err_t loadButtonModeMemory(uint8_t* modeMemory, size_t size);

    /**
     * @brief Save button mode memory to NVS
     * @param modeMemory Array containing mode data (size must be 11)
     * @return ESP_OK on success, ESP_FAIL on error
     */
    esp_err_t saveButtonModeMemory(const uint8_t* modeMemory, size_t size);

private:
    static void onPowerStateChange(bool powerOn, bool oldState);

    radio::RadioManager& m_radioManager;
    static NvsManager* s_instance; // For static callback access

    static constexpr const char* TAG = "NvsManager";
    static constexpr const char* STORAGE_NAMESPACE = "radio_state";
    static constexpr const char* BUTTON_MODE_MEMORY_KEY = "band_modes";
};
