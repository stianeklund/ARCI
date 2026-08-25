#include "NvsManager.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <cstring>

namespace {
// RAII guard so early returns between nvs_open and nvs_commit cannot leak the handle
// (audit finding M5). Closes the handle on scope exit once marked open.
struct NvsHandleGuard {
    nvs_handle_t handle = 0;
    bool open = false;
    ~NvsHandleGuard() { if (open) nvs_close(handle); }
};
} // namespace

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
    // Start the deferred-save worker before registering the callback so a very early
    // power-off event always has a task to signal.
    if (m_saveTaskHandle == nullptr) {
        if (xTaskCreate(&NvsManager::saveWorkerTask, "nvs_save", 4096, this, 4, &m_saveTaskHandle) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create deferred NVS save task; saves will run inline");
            m_saveTaskHandle = nullptr;
        }
    }
    ESP_LOGI(TAG, "Setting up power state change callback for event-driven saves");
    m_radioManager.setPowerStateChangeCallback(&NvsManager::onPowerStateChange);
}

void NvsManager::saveWorkerTask(void* arg) {
    auto* self = static_cast<NvsManager*>(arg);
    while (true) {
        // Block until a power-off event requests a persistent save. Performing the
        // flash commits here (not in the dispatch-context callback) keeps them off
        // dispatchMutex_. Wakes promptly on notification (well within ~1 s).
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "Deferred save: writing radio state + EX menu to NVS");
        esp_err_t ret = self->saveRadioState();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Radio state saved successfully (deferred power-off)");
        } else {
            ESP_LOGE(TAG, "Deferred radio state save failed: %s", esp_err_to_name(ret));
        }

        ret = self->saveExtendedMenu();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "EX menu saved (deferred power-off)");
        } else {
            ESP_LOGW(TAG, "Deferred EX menu save failed: %s", esp_err_to_name(ret));
        }
    }
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
        // Power turning OFF - defer the flash commits to the save worker. This callback
        // runs inside CAT dispatch (under dispatchMutex_); doing the writes here would
        // stall every other interface for the duration of the flash-cache-blocking commit.
        if (s_instance->m_saveTaskHandle != nullptr) {
            ESP_LOGI(s_instance->TAG, "Power turning OFF - signaling deferred NVS save");
            xTaskNotifyGive(s_instance->m_saveTaskHandle);
        } else {
            // Worker unavailable (creation failed): fall back to inline save.
            ESP_LOGW(s_instance->TAG, "Save worker unavailable - saving radio state inline");
            s_instance->saveRadioState();
            s_instance->saveExtendedMenu();
        }
    } else if (powerOn && !oldState) {
        // Power turning ON - will be handled by separate sync mechanism
        ESP_LOGI(s_instance->TAG, "Power turning ON - sync and save will be handled separately");
    }
}

esp_err_t NvsManager::saveRadioState() {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    // RAII guard closes the handle on any early return below (audit M5).
    NvsHandleGuard guard;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &guard.handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    guard.open = true;

    const auto& state = m_radioManager.getState();

    err = nvs_set_u64(guard.handle, "vfoA", state.vfoAFrequency.load());
    if (err != ESP_OK) return err;

    err = nvs_set_u64(guard.handle, "vfoB", state.vfoBFrequency.load());
    if (err != ESP_OK) return err;

    err = nvs_set_i8(guard.handle, "mode", state.mode.load());
    if (err != ESP_OK) return err;

    // Save transverter configuration
    err = nvs_set_u8(guard.handle, "transverter", state.transverter.load(std::memory_order_relaxed) ? 1 : 0);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(guard.handle, "tvr_offset_plus", state.transverterOffsetPlus.load(std::memory_order_relaxed) ? 1 : 0);
    if (err != ESP_OK) return err;

    err = nvs_set_u64(guard.handle, "tvr_offset_hz", state.transverterOffsetHz.load(std::memory_order_relaxed));
    if (err != ESP_OK) return err;

    // Save transverter-related menu settings
    err = nvs_set_i32(guard.handle, "drv_connector", state.drvConnectorMode);
    if (err != ESP_OK) return err;

    err = nvs_set_i32(guard.handle, "hf_linear_amp", state.hfLinearAmpControl.load(std::memory_order_relaxed));
    if (err != ESP_OK) return err;

    err = nvs_set_i32(guard.handle, "vhf_linear_amp", state.vhfLinearAmpControl.load(std::memory_order_relaxed));
    if (err != ESP_OK) return err;

    err = nvs_set_u16(guard.handle, "atu_bands", state.tunerConfiguredBands.load(std::memory_order_relaxed));
    if (err != ESP_OK) return err;

    err = nvs_set_u16(guard.handle, "atu_enabled", state.tunerEnabledBands.load(std::memory_order_relaxed));
    if (err != ESP_OK) return err;

    err = nvs_commit(guard.handle);
    if (err != ESP_OK) return err;

    return ESP_OK; // guard closes the handle
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
        nvs_close(my_handle);
        return err;
    }

    uint16_t tunerConfiguredBands = 0;
    err = nvs_get_u16(my_handle, "atu_bands", &tunerConfiguredBands);
    if (err == ESP_OK) {
        state.tunerConfiguredBands.store(tunerConfiguredBands, std::memory_order_relaxed);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(my_handle);
        return err;
    }

    uint16_t tunerEnabledBands = 0;
    err = nvs_get_u16(my_handle, "atu_enabled", &tunerEnabledBands);
    if (err == ESP_OK) {
        state.tunerEnabledBands.store(tunerEnabledBands, std::memory_order_relaxed);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(my_handle);
        return err;
    }

    nvs_close(my_handle);
    // The cached AC state is unknown after reboot, so this deliberately sends the
    // configured policy even when the default in-memory state happens to match it.
    m_radioManager.applyCurrentBandTunerSetting();
    return ESP_OK;
#else
    ESP_LOGI(TAG, "NVS load requested but persistence is disabled, using default state");
    return ESP_OK;
#endif
}

esp_err_t NvsManager::saveTunerBandSettings(const uint16_t configuredBands, const uint16_t enabledBands) {
#ifdef CONFIG_ENABLE_NVS_PERSISTENCE
    NvsHandleGuard guard;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &guard.handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for ATU band settings: %s", esp_err_to_name(err));
        return err;
    }
    guard.open = true;

    err = nvs_set_u16(guard.handle, "atu_bands", configuredBands);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(guard.handle, "atu_enabled", enabledBands);
    if (err != ESP_OK) return err;
    return nvs_commit(guard.handle);
#else
    (void)configuredBands;
    (void)enabledBands;
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
