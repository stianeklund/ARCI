#include "NvsManager.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <cstring>

// Static instance for callback access
NvsManager* NvsManager::s_instance = nullptr;

NvsManager::NvsManager(radio::RadioManager& radioManager)
    : m_radioManager(radioManager) {
    s_instance = this;
}

esp_err_t NvsManager::init() {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing and re-initializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
#else
    ESP_LOGI(TAG, "NVS persistence disabled by configuration");
    return ESP_OK;
#endif
}

void NvsManager::setupPowerStateCallback() {
    ESP_LOGI(TAG, "Setting up power state change callback for event-driven saves");
    m_radioManager.setPowerStateChangeCallback(&NvsManager::onPowerStateChange);
}

esp_err_t NvsManager::loadAndSyncOnStartup() {
    ESP_LOGI(TAG, "Starting load and sync sequence");
    
    // Load previously saved state
    esp_err_t ret = loadRadioState();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load radio state: %s", esp_err_to_name(ret));
        // Continue with defaults
    }
    
    // If interface is powered on and radio is connected, sync and save
    if (m_radioManager.getOnOffState() == 1 && m_radioManager.isRadioConnected()) {
        ESP_LOGI(TAG, "Interface powered on - syncing transverter settings and saving updated state");
        m_radioManager.syncTransverterMenuSettings();
        
        // Small delay to allow sync to complete
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Save the synchronized state
        ret = saveRadioState();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Radio state saved after startup sync");
        } else {
            ESP_LOGW(TAG, "Failed to save radio state after sync: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGI(TAG, "Interface off or radio disconnected - skipping sync");
    }
    
    return ESP_OK;
}

void NvsManager::onPowerStateChange(bool powerOn, bool oldState) {
    if (!s_instance) {
        ESP_LOGE("NvsManager", "Static instance not available for callback");
        return;
    }
    
    ESP_LOGI(s_instance->TAG, "Power state callback: %s -> %s", 
             oldState ? "ON" : "OFF", powerOn ? "ON" : "OFF");
    
    if (!powerOn && oldState) {
        // Power turning OFF - save current state
        ESP_LOGI(s_instance->TAG, "Power turning OFF - saving radio state to NVS");
        esp_err_t ret = s_instance->saveRadioState();
        if (ret == ESP_OK) {
            ESP_LOGI(s_instance->TAG, "Radio state saved successfully on power-off");
        } else {
            ESP_LOGE(s_instance->TAG, "Failed to save radio state on power-off: %s", esp_err_to_name(ret));
        }
        // Save EX menu if dirty
        ret = s_instance->saveExtendedMenu();
        if (ret == ESP_OK) {
            ESP_LOGI(s_instance->TAG, "EX menu saved on power-off");
        } else {
            ESP_LOGW(s_instance->TAG, "Failed to save EX menu on power-off: %s", esp_err_to_name(ret));
        }
    } else if (powerOn && !oldState) {
        // Power turning ON - will be handled by separate sync mechanism
        ESP_LOGI(s_instance->TAG, "Power turning ON - sync and save will be handled separately");
    }
}

esp_err_t NvsManager::saveRadioState() {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    const auto& state = m_radioManager.getState();

    err = nvs_set_u64(my_handle, "vfoA", state.vfoAFrequency.load());
    if (err != ESP_OK) return err;

    err = nvs_set_u64(my_handle, "vfoB", state.vfoBFrequency.load());
    if (err != ESP_OK) return err;

    err = nvs_set_i8(my_handle, "mode", state.mode.load());
    if (err != ESP_OK) return err;

    // Save transverter configuration
    err = nvs_set_u8(my_handle, "transverter", state.transverter.load(std::memory_order_relaxed) ? 1 : 0);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(my_handle, "tvr_offset_plus", state.transverterOffsetPlus.load(std::memory_order_relaxed) ? 1 : 0);
    if (err != ESP_OK) return err;

    err = nvs_set_u64(my_handle, "tvr_offset_hz", state.transverterOffsetHz.load(std::memory_order_relaxed));
    if (err != ESP_OK) return err;

    // Save transverter-related menu settings
    err = nvs_set_i32(my_handle, "drv_connector", state.drvConnectorMode);
    if (err != ESP_OK) return err;

    err = nvs_set_i32(my_handle, "hf_linear_amp", state.hfLinearAmpControl.load(std::memory_order_relaxed));
    if (err != ESP_OK) return err;

    err = nvs_set_i32(my_handle, "vhf_linear_amp", state.vhfLinearAmpControl.load(std::memory_order_relaxed));
    if (err != ESP_OK) return err;

    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    nvs_close(my_handle);
    return ESP_OK;
#else
    ESP_LOGD(TAG, "NVS save requested but persistence is disabled");
    return ESP_OK;
#endif
}

esp_err_t NvsManager::loadRadioState() {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace '%s' not found. This is normal on first boot. Using default state.", STORAGE_NAMESPACE);
        return ESP_OK; // Not an error, just no state to load.
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err; // A real error occurred.
    }

    auto& state = m_radioManager.getState();

    uint64_t vfoA = 0;
    err = nvs_get_u64(my_handle, "vfoA", &vfoA);
    if (err == ESP_OK) {
        state.vfoAFrequency.store(vfoA);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }

    uint64_t vfoB = 0;
    err = nvs_get_u64(my_handle, "vfoB", &vfoB);
    if (err == ESP_OK) {
        state.vfoBFrequency.store(vfoB);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }

    int8_t mode = 0;
    err = nvs_get_i8(my_handle, "mode", &mode);
    if (err == ESP_OK) {
        state.mode.store(mode);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }

    nvs_close(my_handle);
    return ESP_OK;
#else
    ESP_LOGI(TAG, "NVS load requested but persistence is disabled, using default state");
    return ESP_OK;
#endif
}

esp_err_t NvsManager::loadButtonModeMemory(uint8_t* modeMemory, size_t size) {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    if (modeMemory == nullptr || size != 11) {
        ESP_LOGE(TAG, "Invalid parameters for loadButtonModeMemory (modeMemory=%p, size=%zu)", modeMemory, size);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "NVS namespace not found, button mode memory will use defaults");
        return ESP_ERR_NOT_FOUND;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for button mode memory!", esp_err_to_name(err));
        return err;
    }

    size_t requiredSize = size;
    err = nvs_get_blob(my_handle, BUTTON_MODE_MEMORY_KEY, modeMemory, &requiredSize);

    nvs_close(my_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "No button mode memory found in NVS");
        return ESP_ERR_NOT_FOUND;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading button mode memory: %s", esp_err_to_name(err));
        return err;
    }

    if (requiredSize != size) {
        ESP_LOGW(TAG, "Button mode memory size mismatch (expected %zu, got %zu)", size, requiredSize);
        return ESP_FAIL;
    }

    // Validate loaded data - ensure all modes are valid (1-9, excluding 0 and 8)
    for (size_t i = 0; i < size; i++) {
        const uint8_t mode = modeMemory[i];
        if (mode < 1 || mode > 9 || mode == 8) {
            ESP_LOGW(TAG, "Invalid mode %d for band %zu in NVS data", mode, i);
            return ESP_ERR_INVALID_STATE;
        }
    }

    ESP_LOGI(TAG, "Loaded button mode memory from NVS");
    return ESP_OK;
#else
    ESP_LOGD(TAG, "NVS persistence disabled, button mode memory not loaded");
    return ESP_ERR_NOT_FOUND;
#endif
}

esp_err_t NvsManager::saveButtonModeMemory(const uint8_t* modeMemory, size_t size) {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    if (modeMemory == nullptr || size != 11) {
        ESP_LOGE(TAG, "Invalid parameters for saveButtonModeMemory (modeMemory=%p, size=%zu)", modeMemory, size);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate data before saving - ensure all modes are valid (1-9, excluding 0 and 8)
    for (size_t i = 0; i < size; i++) {
        const uint8_t mode = modeMemory[i];
        if (mode < 1 || mode > 9 || mode == 8) {
            ESP_LOGE(TAG, "Invalid mode %d for band %zu, cannot save to NVS", mode, i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for button mode memory save: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(my_handle, BUTTON_MODE_MEMORY_KEY, modeMemory, size);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write button mode memory to NVS: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }

    err = nvs_commit(my_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit button mode memory to NVS: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }

    nvs_close(my_handle);
    ESP_LOGD(TAG, "Button mode memory saved to NVS");
    return ESP_OK;
#else
    ESP_LOGD(TAG, "NVS persistence disabled, button mode memory not saved");
    return ESP_OK;
#endif
}

esp_err_t NvsManager::saveExMenuBlob(const ExNvsData& data) {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    nvs_handle_t handle;
    esp_err_t err = nvs_open(EX_MENU_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", EX_MENU_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(handle, EX_MENU_KEY, &data, sizeof(data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write EX menu blob: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit EX menu: %s", esp_err_to_name(err));
    }
    return err;
#else
    ESP_LOGD(TAG, "NVS persistence disabled, EX menu not saved");
    return ESP_OK;
#endif
}

esp_err_t NvsManager::loadExMenuBlob(ExNvsData& data) {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    nvs_handle_t handle;
    esp_err_t err = nvs_open(EX_MENU_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No EX menu data in NVS (first boot)");
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", EX_MENU_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    size_t blobSize = sizeof(data);
    err = nvs_get_blob(handle, EX_MENU_KEY, &data, &blobSize);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No EX menu blob found in NVS");
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read EX menu blob: %s", esp_err_to_name(err));
    }
    return err;
#else
    ESP_LOGD(TAG, "NVS persistence disabled, EX menu not loaded");
    return ESP_ERR_NOT_FOUND;
#endif
}

esp_err_t NvsManager::saveExtendedMenu() {
    // Delegate to packAndSaveExMenu() implemented in RadioCore
    // (avoids NvsManager depending on CommandHandlers component)
    extern esp_err_t packAndSaveExMenu(NvsManager& nvs);
    return packAndSaveExMenu(*this);
}

esp_err_t NvsManager::loadExtendedMenu() {
    // Delegate to loadAndUnpackExMenu() implemented in RadioCore
    // (avoids NvsManager depending on CommandHandlers component)
    extern esp_err_t loadAndUnpackExMenu(NvsManager& nvs);
    return loadAndUnpackExMenu(*this);
}
