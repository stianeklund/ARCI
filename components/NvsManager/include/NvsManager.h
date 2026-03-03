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
     * @brief Compact NVS blob for EX menu values (801 bytes)
     */
    struct ExNvsData {
        uint8_t version = 1;
        char values[100][8]; // null-terminated string per menu item
    };
    static_assert(sizeof(ExNvsData) == 801, "ExNvsData must be 801 bytes");

    /**
     * @brief Save EX menu blob to NVS
     * @param data Packed EX menu data
     * @return ESP_OK on success
     */
    esp_err_t saveExMenuBlob(const ExNvsData& data);

    /**
     * @brief Load EX menu blob from NVS
     * @param data Output: populated from NVS
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND on first boot
     */
    esp_err_t loadExMenuBlob(ExNvsData& data);

    /**
     * @brief High-level: save ExtendedMenuState to NVS
     * Packs the singleton ExtendedMenuState into a blob and writes it.
     */
    esp_err_t saveExtendedMenu();

    /**
     * @brief High-level: load ExtendedMenuState from NVS
     * Reads the blob and populates the singleton ExtendedMenuState.
     */
    esp_err_t loadExtendedMenu();

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
    static constexpr const char* EX_MENU_NAMESPACE = "ex_menu";
    static constexpr const char* EX_MENU_KEY = "values";
    static constexpr size_t EX_MENU_COUNT = 100;
};
