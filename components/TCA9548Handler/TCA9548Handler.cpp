#include "include/TCA9548Handler.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "TCA9548Handler";

TCA9548Handler::TCA9548Handler(uint8_t i2cAddr)
    : m_i2cAddr(i2cAddr)
    , m_i2cBusHandle(nullptr)
    , m_i2cDevHandle(nullptr)
    , m_currentChannel(0xFF)
    , m_initialized(false)
    , m_mutex(nullptr)
{
    // Create mutex for thread-safe channel switching
    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create TCA9548 mutex");
    }
}

TCA9548Handler::~TCA9548Handler()
{
    if (m_i2cDevHandle != nullptr) {
        i2c_master_bus_rm_device(m_i2cDevHandle);
        m_i2cDevHandle = nullptr;
    }

    if (m_mutex != nullptr) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

esp_err_t TCA9548Handler::initialize(i2c_master_bus_handle_t i2cBusHandle)
{
    if (i2cBusHandle == nullptr) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return ESP_ERR_INVALID_ARG;
    }

    m_i2cBusHandle = i2cBusHandle;

    // Configure TCA9548 device on I2C bus
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = m_i2cAddr;
    dev_cfg.scl_speed_hz = 400000; // 400 kHz

    esp_err_t ret = i2c_master_bus_add_device(m_i2cBusHandle, &dev_cfg, &m_i2cDevHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TCA9548 to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Test device presence by trying to read the channel register
    uint8_t channelMask = 0;
    esp_err_t test_ret = i2c_master_receive(m_i2cDevHandle, &channelMask, 1, 1000);
    if (test_ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9548 not responding at 0x%02X - check RST pin", m_i2cAddr);
        return ESP_ERR_NOT_FOUND;
    }

    // Disable all channels initially
    uint8_t disableMask = 0x00;
    ret = writeChannelRegister(disableMask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable all channels: %s", esp_err_to_name(ret));
        return ret;
    }
    m_currentChannel = 0xFF;

    m_initialized = true;
    ESP_LOGI(TAG, "TCA9548 initialized at 0x%02X", m_i2cAddr);
    return ESP_OK;
}

esp_err_t TCA9548Handler::selectChannel(uint8_t channel)
{
    if (!m_initialized) {
        ESP_LOGE(TAG, "TCA9548 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (m_mutex == nullptr) {
        ESP_LOGE(TAG, "TCA9548 mutex not created");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel != 0xFF && channel >= MAX_CHANNELS) {
        ESP_LOGE(TAG, "Invalid channel %d (must be 0-7 or 0xFF)", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // Acquire mutex for thread-safe channel switching
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire TCA9548 mutex within %dms", MUTEX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

    // Calculate channel mask (0x00 = all disabled, 0x01-0x80 = single channel)
    uint8_t channelMask = (channel == 0xFF) ? 0x00 : (1 << channel);

    esp_err_t ret = writeChannelRegister(channelMask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select channel %d", channel);
        xSemaphoreGive(m_mutex);  // Release mutex on error
        return ret;
    }

    m_currentChannel = channel;
    ESP_LOGD(TAG, "Selected channel %d (mask 0x%02X)", channel, channelMask);

    // Release mutex
    xSemaphoreGive(m_mutex);
    return ESP_OK;
}

esp_err_t TCA9548Handler::disableAllChannels()
{
    return selectChannel(0xFF);
}

bool TCA9548Handler::isDevicePresent()
{
    if (m_i2cDevHandle == nullptr) {
        return false;
    }

    // Try to read current channel register
    uint8_t channelMask = 0;
    esp_err_t ret = readChannelRegister(channelMask);
    return (ret == ESP_OK);
}

esp_err_t TCA9548Handler::writeChannelRegister(uint8_t channelMask)
{
    // TCA9548 has no register address - write directly sends channel mask
    esp_err_t ret = i2c_master_transmit(m_i2cDevHandle, &channelMask, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t TCA9548Handler::readChannelRegister(uint8_t& channelMask)
{
    // TCA9548 has no register address - read directly returns channel mask
    esp_err_t ret = i2c_master_receive(m_i2cDevHandle, &channelMask, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}