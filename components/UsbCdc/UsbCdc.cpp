#include "UsbCdc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"
#include "tusb.h"
#include "../../include/pin_definitions.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include <cstdint>
#include <algorithm>
#include <cstring>

// Using ESP-IDF complete default descriptors for dual CDC
// All descriptor strings are managed through sdkconfig configuration


static const char *TAG = "UsbCdc";

bool UsbCdc::m_initialized = false;
bool UsbCdc::m_control_pins_initialized = false;
bool UsbCdc::m_dtr_state = false;
bool UsbCdc::m_rts_state = false;
uint64_t UsbCdc::m_writeBlockUntilUs[2] = {0, 0};
uint8_t UsbCdc::m_backpressureLevel[2] = {0, 0};
bool UsbCdc::m_last_cdc_connected = false;
bool UsbCdc::m_last_usb_mounted = false;
UsbCdc::RxCallback UsbCdc::m_rx_callback = nullptr;
UsbCdc::TxCallback UsbCdc::m_tx_callback = nullptr;
uint8_t UsbCdc::m_tx_buf[2][UsbCdc::TX_BUFFER_SIZE] = {};
size_t UsbCdc::m_tx_head[2] = {0, 0};
size_t UsbCdc::m_tx_tail[2] = {0, 0};
SemaphoreHandle_t UsbCdc::m_tx_mutex[2] = {nullptr, nullptr};
bool UsbCdc::m_require_dtr = false; // default: be lenient for TS-590SG apps

// ESP-IDF CDC-ACM callback for RX events
static void cdc_rx_callback(int itf, cdcacm_event_t *event) {
    switch (event->type) {
        case CDC_EVENT_RX_WANTED_CHAR:
            // Frame boundary (';') detected: wake the reader to drain frames
            ESP_LOGD("UsbCdc", "CDC%d wanted-char received; signaling RX", itf);
            UsbCdc::notifyRxAvailable(static_cast<uint8_t>(itf));
            break;
        case CDC_EVENT_RX:
            // Wake on generic RX as well; accumulator will split on ';'
            ESP_LOGV("UsbCdc", "CDC%d RX event; signaling RX", itf);
            UsbCdc::notifyRxAvailable(static_cast<uint8_t>(itf));
            break;
        default:
            break;
    }
}

// ESP-IDF CDC-ACM callback for line state changes
static void cdc_line_state_callback(int itf, cdcacm_event_t *event) {
    if (event->type == CDC_EVENT_LINE_STATE_CHANGED) {
        bool dtr = event->line_state_changed_data.dtr;
        bool rts = event->line_state_changed_data.rts;
        ESP_LOGD("UsbCdc", "CDC%d line state: DTR=%d, RTS=%d", itf, dtr, rts);

        // Mirror only DTR to a physical pin; RTS/CTS handling removed
        UsbCdc::setDTR(dtr);

        // If DTR is set, the host is ready to communicate
        // This is a good signal that CAT clients have opened the port
        if (dtr) {
            ESP_LOGD("UsbCdc", "Host ready for communication (DTR asserted)");
            // Flush any buffered data when host opens the port
            UsbCdc::flush(static_cast<uint8_t>(itf));
        }
    }
}

// ESP-IDF CDC-ACM callback for line coding changes
static void cdc_line_coding_callback(int itf, cdcacm_event_t *event) {
    if (event->type == CDC_EVENT_LINE_CODING_CHANGED) {
        uint32_t baud_rate = event->line_coding_changed_data.p_line_coding->bit_rate;
        ESP_LOGD("UsbCdc", "CDC%d line coding: baud=%ul", itf, baud_rate);
    }
}

// TinyUSB 2.x device event handler (replaces tud_mount_cb/tud_umount_cb)
static void tinyusb_device_event_handler(tinyusb_event_t *event, void *arg) {
    switch (event->id) {
        case TINYUSB_EVENT_ATTACHED:
            ESP_LOGI("UsbCdc", "USB device mounted");
            UsbCdc::setDTR(true);
            break;
        case TINYUSB_EVENT_DETACHED:
            ESP_LOGI("UsbCdc", "USB device unmounted");
            UsbCdc::setDTR(false);
            break;
        default:
            break;
    }
}

esp_err_t UsbCdc::init() {
    if (m_initialized) {
        ESP_LOGW(TAG, "USB CDC already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing USB CDC");

    // Use ESP-IDF 2.x default configuration with required task settings
    tinyusb_config_t tusb_cfg = {};
    tusb_cfg.event_cb = tinyusb_device_event_handler;
    tusb_cfg.task.size = TINYUSB_DEFAULT_TASK_SIZE;
    tusb_cfg.task.priority = TINYUSB_DEFAULT_TASK_PRIO;
    tusb_cfg.task.xCoreID = TINYUSB_DEFAULT_TASK_AFFINITY;

    // Install TinyUSB driver
    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TinyUSB driver: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize first CDC instance
    tinyusb_config_cdcacm_t acm_cfg_0 = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = nullptr,  // Will set via register_callback
        .callback_rx_wanted_char = nullptr,
        .callback_line_state_changed = nullptr,
        .callback_line_coding_changed = nullptr
    };
    err = tinyusb_cdcacm_init(&acm_cfg_0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CDC ACM 0: %s", esp_err_to_name(err));
        return err;
    }
    // Wake when ';' (CAT frame terminator) is received on CDC0
    // Use TinyUSB wanted-char per interface; events still delivered via CDC-ACM callback
    tud_cdc_n_set_wanted_char(0, ';');

    // Initialize second CDC instance
    tinyusb_config_cdcacm_t acm_cfg_1 = {
        .cdc_port = TINYUSB_CDC_ACM_1,
        .callback_rx = nullptr,  // Will set via register_callback
        .callback_rx_wanted_char = nullptr,
        .callback_line_state_changed = nullptr,
        .callback_line_coding_changed = nullptr
    };
    err = tinyusb_cdcacm_init(&acm_cfg_1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CDC ACM 1: %s", esp_err_to_name(err));
        return err;
    }
    // Wake when ';' (CAT frame terminator) is received on CDC1
    tud_cdc_n_set_wanted_char(1, ';');

    // Register callbacks for ';' wanted-char and generic RX (per interface)
    err = tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_RX_WANTED_CHAR, cdc_rx_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CDC 0 wanted-char callback: %s", esp_err_to_name(err));
        return err;
    }
    err = tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_RX, cdc_rx_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CDC 0 RX callback: %s", esp_err_to_name(err));
        return err;
    }

    err = tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_1, CDC_EVENT_RX_WANTED_CHAR, cdc_rx_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CDC 1 wanted-char callback: %s", esp_err_to_name(err));
        return err;
    }
    err = tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_1, CDC_EVENT_RX, cdc_rx_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CDC 1 RX callback: %s", esp_err_to_name(err));
        return err;
    }

    // Register line state callbacks for both CDC instances
    err = tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_LINE_STATE_CHANGED, cdc_line_state_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CDC 0 line state callback: %s", esp_err_to_name(err));
        return err;
    }

    err = tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_1, CDC_EVENT_LINE_STATE_CHANGED, cdc_line_state_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CDC 1 line state callback: %s", esp_err_to_name(err));
        return err;
    }

    // Register line coding callbacks for both CDC instances
    err = tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_LINE_CODING_CHANGED, cdc_line_coding_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CDC 0 line coding callback: %s", esp_err_to_name(err));
        return err;
    }

    err = tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_1, CDC_EVENT_LINE_CODING_CHANGED, cdc_line_coding_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CDC 1 line coding callback: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Registered all callbacks for both CDC instances");

    // Initialize control pins (DTR, RTS, CTS)
    err = initControlPins();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize control pins: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize TX mutexes
    for (auto &m : m_tx_mutex) {
        if (m == nullptr) {
            m = xSemaphoreCreateMutex();
        }
    }

    m_initialized = true;
    ESP_LOGI(TAG, "USB CDC initialized successfully");
    return ESP_OK;
}

esp_err_t UsbCdc::enableUsbLogs() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "USB CDC not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Redirecting logs to USB CDC");
    esp_err_t err = tinyusb_console_init(TINYUSB_CDC_ACM_0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to redirect logs to USB: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t UsbCdc::disableUsbLogs() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "USB CDC not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Redirecting logs back to UART");
    esp_err_t err = tinyusb_console_deinit(TINYUSB_CDC_ACM_0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to redirect logs back to UART: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t UsbCdc::writeData(const uint8_t* data, size_t length, uint8_t instance) {
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!data || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Validate instance
    if (instance > 1) {
        ESP_LOGE(TAG, "Invalid CDC instance: %d", instance);
        return ESP_ERR_INVALID_ARG;
    }

    // Check if USB is mounted and CDC is connected
    const bool mounted = tud_mounted();
    const bool connected = tud_cdc_n_connected(instance);
    const bool suspended = tud_suspended();
    
    // Only log when connection status changes for instance 0 (avoid log spam)
    if (instance == 0 && (connected != m_last_cdc_connected || mounted != m_last_usb_mounted)) {
        ESP_LOGI(TAG, "CDC%d connection changed: mounted=%d, connected=%d, suspended=%d", 
                 instance, mounted, connected, suspended);
        
        // If we just disconnected, clear the TX buffer to prevent overflow buildup
        if (!connected && mounted && m_last_cdc_connected) {
            ESP_LOGI(TAG, "CDC%d client disconnected, clearing TX buffer to prevent overflow", instance);
            if (xSemaphoreTake(m_tx_mutex[instance], pdMS_TO_TICKS(10)) == pdTRUE) {
                m_tx_head[instance] = 0;
                m_tx_tail[instance] = 0;
                xSemaphoreGive(m_tx_mutex[instance]);
            }
            
            // Send a remote wakeup if suspended to notify host we have data
            if (suspended && tud_remote_wakeup()) {
                ESP_LOGI(TAG, "Sent USB remote wakeup to host");
            }
        }
        
        // Update state after checking for disconnection
        m_last_cdc_connected = connected;
        m_last_usb_mounted = mounted;
    }
    
    // Require USB mounted
    if (!mounted) {
        ESP_LOGV(TAG, "USB not mounted, dropping %zu bytes", length);
        return ESP_OK;  // Return success to avoid error cascade
    }

    // Optionally require DTR; if not required, accept and buffer data even if not connected
    if (!connected && m_require_dtr) {
        ESP_LOGV(TAG, "CDC%d not connected (DTR=0), dropping %zu bytes (require_dtr)", instance, length);
        return ESP_OK;
    }
    
    // If suspended, try to wake up the host
    if (suspended) {
        ESP_LOGV(TAG, "USB suspended, attempting remote wakeup");
        if (tud_remote_wakeup()) {
            ESP_LOGD(TAG, "Sent USB remote wakeup to host");
        }
    }
    
    ESP_LOGV(TAG, "CDC%d USB status: mounted=%d, connected=%d, suspended=%d", instance, mounted, connected, suspended);

    // First, try to flush any buffered data from previous short writes
    (void)txDequeueToUsb(instance);

    // Write data to CDC queue
    ESP_LOGV(TAG, "Writing %zu bytes to CDC%d", length, instance);
    const tinyusb_cdcacm_itf_t cdc_itf = (instance == 0) ? TINYUSB_CDC_ACM_0 : TINYUSB_CDC_ACM_1;
    const size_t written = tinyusb_cdcacm_write_queue(cdc_itf, data, length);
    if (written != length) {
        // Enqueue remaining bytes into ring buffer for later retry
        const size_t remaining = length - written;
        if (remaining > 0) {
            if (m_tx_mutex[instance] && xSemaphoreTake(m_tx_mutex[instance], portMAX_DELAY) == pdTRUE) {
                txEnqueue(instance, data + written, remaining);
                xSemaphoreGive(m_tx_mutex[instance]);
            }
        }
        // If not connected (DTR=0) but we don't require DTR, it's normal to queue
        if (!tud_cdc_n_connected(instance) && !m_require_dtr) {
            ESP_LOGV(TAG, "CDC%d not connected; queued %zu bytes for later (lenient mode)", instance, remaining);
            // Don't attempt flush; will be flushed when host opens
            return ESP_OK;
        }
        // Else: some data queued due to full USB queue
        ESP_LOGD(TAG, "CDC%d write incomplete: %zu/%zu bytes (queued %zu for retry)", instance, written, length, remaining);
    }

    // Flush the data with no timeout (non-blocking)
    esp_err_t flush_result = tinyusb_cdcacm_write_flush(cdc_itf, 0);
    if (flush_result != ESP_OK && flush_result != ESP_ERR_TIMEOUT) {
        // If not connected and we don't require DTR, we'll flush later
        if (!tud_cdc_n_connected(instance) && !m_require_dtr) {
            ESP_LOGV(TAG, "CDC%d flush deferred (DTR=0, lenient mode)", instance);
            return ESP_OK;
        }
        ESP_LOGV(TAG, "CDC%d flush warning: %s", instance, esp_err_to_name(flush_result));
        // Don't return error for flush issues - data was queued successfully
    }
    // Try to push any buffered data after flush as space may be free now
    (void)txDequeueToUsb(instance);
    return ESP_OK;
}

esp_err_t UsbCdc::writeString(const char* str, uint8_t instance) {
    if (!str) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t length = strlen(str);
    return writeData(reinterpret_cast<const uint8_t*>(str), length, instance);
}

void UsbCdc::flush(uint8_t instance) {
    const tinyusb_cdcacm_itf_t cdc_itf = (instance == 0) ? TINYUSB_CDC_ACM_0 : TINYUSB_CDC_ACM_1;
    (void)tinyusb_cdcacm_write_flush(cdc_itf, 0);
    // Attempt to push buffered data as well
    (void)txDequeueToUsb(instance);
}

size_t UsbCdc::txFreeSpace(uint8_t instance) {
    const size_t head = m_tx_head[instance];
    const size_t tail = m_tx_tail[instance];
    if (head >= tail) {
        return (TX_BUFFER_SIZE - (head - tail) - 1);
    } else {
        return (tail - head - 1);
    }
}

size_t UsbCdc::txBuffered(uint8_t instance) {
    const size_t head = m_tx_head[instance];
    const size_t tail = m_tx_tail[instance];
    if (head >= tail) return head - tail;
    return TX_BUFFER_SIZE - (tail - head);
}

size_t UsbCdc::txContigAvailable(uint8_t instance) {
    const size_t head = m_tx_head[instance];
    const size_t tail = m_tx_tail[instance];
    if (tail <= head) {
        return TX_BUFFER_SIZE - head;
    } else {
        return tail - head - 1;
    }
}

void UsbCdc::txEnqueue(uint8_t instance, const uint8_t* data, size_t len) {
    // Check if we should buffer data when client is disconnected
    if (!tud_cdc_n_connected(instance)) {
        // If not connected and buffer is getting full, start dropping data to prevent overflow
        size_t free = txFreeSpace(instance);
        if (free < TX_BUFFER_SIZE / 4) { // Drop data when buffer is >75% full and client disconnected
            ESP_LOGD(TAG, "CDC%d disconnected with low buffer space (%zu free), dropping %zu bytes", 
                     instance, free, len);
            return;
        }
    }
    
    size_t free = txFreeSpace(instance);
    if (len > free) {
        // More detailed logging for buffer overflow
        bool connected = tud_cdc_n_connected(instance);
        if (connected) {
            ESP_LOGW(TAG, "CDC%d TX buffer overflow: dropping %zu bytes (free %zu, client connected)", 
                     instance, len - free, free);
        } else {
            ESP_LOGD(TAG, "CDC%d TX buffer overflow: dropping %zu bytes (free %zu, client disconnected)", 
                     instance, len - free, free);
        }
        len = free; // drop excess
    }
    while (len) {
        size_t contig = txContigAvailable(instance);
        size_t chunk = (len < contig) ? len : contig;
        if (chunk == 0) break;
        memcpy(&m_tx_buf[instance][m_tx_head[instance]], data, chunk);
        m_tx_head[instance] = (m_tx_head[instance] + chunk) % TX_BUFFER_SIZE;
        data += chunk;
        len -= chunk;
    }
}

size_t UsbCdc::txDequeueToUsb(uint8_t instance) {
    if (!m_tx_mutex[instance]) return 0;
    if (!tud_mounted()) return 0;
    if (!tud_cdc_n_connected(instance)) return 0;
    size_t moved = 0;
    if (xSemaphoreTake(m_tx_mutex[instance], portMAX_DELAY) != pdTRUE) return 0;
    const tinyusb_cdcacm_itf_t cdc_itf = (instance == 0) ? TINYUSB_CDC_ACM_0 : TINYUSB_CDC_ACM_1;
    while (m_tx_tail[instance] != m_tx_head[instance]) {
        size_t tail = m_tx_tail[instance];
        size_t head = m_tx_head[instance];
        size_t contig = (head >= tail) ? (head - tail) : (TX_BUFFER_SIZE - tail);
        if (contig == 0) break;
        size_t written = tinyusb_cdcacm_write_queue(cdc_itf, &m_tx_buf[instance][tail], contig);
        if (written == 0) {
            break; // USB queue full
        }
        moved += written;
        m_tx_tail[instance] = (tail + written) % TX_BUFFER_SIZE;
    }
    // Attempt non-blocking flush to move data downstream
    (void)tinyusb_cdcacm_write_flush(cdc_itf, 0);
    xSemaphoreGive(m_tx_mutex[instance]);
    return moved;
}

esp_err_t UsbCdc::initControlPins() {
    if (m_control_pins_initialized) {
        ESP_LOGW(TAG, "Control pins already initialized");
        return ESP_OK;
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGI(TAG, "Initializing USB CDC control pin (DTR: %d)", PIN_USB_DTR);

    // Configure DTR as output (Data Terminal Ready)
    gpio_config_t dtr_config = {
        .pin_bit_mask = 1ULL << PIN_USB_DTR,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&dtr_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DTR pin: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize DTR to inactive state (low)
    gpio_set_level(static_cast<gpio_num_t>(PIN_USB_DTR), 0);
    m_dtr_state = false;

    m_control_pins_initialized = true;
    ESP_LOGI(TAG, "USB CDC control pins initialized successfully");
#else
    ESP_LOGW(TAG, "Control pins only supported on ESP32-S3");
#endif

    return ESP_OK;
}

esp_err_t UsbCdc::setDTR(bool state) {
    if (!m_control_pins_initialized) {
        ESP_LOGE(TAG, "Control pins not initialized");
        return ESP_ERR_INVALID_STATE;
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGD(TAG, "Setting DTR to %s", state ? "HIGH" : "LOW");
    gpio_set_level(static_cast<gpio_num_t>(PIN_USB_DTR), state ? 1 : 0);
    m_dtr_state = state;
    return ESP_OK;
#else
    ESP_LOGW(TAG, "DTR control only supported on ESP32-S3");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t UsbCdc::setRTS(bool state) {
    if (!m_control_pins_initialized) {
        ESP_LOGE(TAG, "Control pins not initialized");
        return ESP_ERR_INVALID_STATE;
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGD(TAG, "Setting RTS to %s", state ? "HIGH" : "LOW");
    gpio_set_level(static_cast<gpio_num_t>(PIN_USB_RTS), state ? 1 : 0);
    m_rts_state = state;
    return ESP_OK;
#else
    ESP_LOGW(TAG, "RTS control only supported on ESP32-S3");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

bool UsbCdc::getCTS() {
    if (!m_control_pins_initialized) {
        ESP_LOGE(TAG, "Control pins not initialized");
        return false;
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    int level = gpio_get_level(static_cast<gpio_num_t>(PIN_USB_CTS));
    ESP_LOGD(TAG, "CTS level: %d", level);
    return level == 1;
#else
    ESP_LOGW(TAG, "CTS reading only supported on ESP32-S3");
    return false;
#endif
}

bool UsbCdc::getDTR() {
    return m_dtr_state;
}

bool UsbCdc::getRTS() {
    return m_rts_state;
}

bool UsbCdc::isConnected() { return tud_mounted() && tud_cdc_n_connected(0); }
bool UsbCdc::isConnected(uint8_t instance) { return tud_mounted() && tud_cdc_n_connected(instance); }

esp_err_t UsbCdc::attemptReconnect() {
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // If already connected, nothing to do
    if (isConnected()) {
        return ESP_OK;
    }
    
    const bool mounted = tud_mounted();
    const bool suspended = tud_suspended();
    
    ESP_LOGD(TAG, "Attempting reconnect: mounted=%d, suspended=%d", mounted, suspended);
    
    // If suspended, try remote wakeup
    if (mounted && suspended) {
        if (tud_remote_wakeup()) {
            ESP_LOGI(TAG, "Sent remote wakeup signal to host");
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "Remote wakeup failed - host may not support it");
        }
    }
    
    // Toggle DTR to signal presence (some hosts detect this)
    if (m_control_pins_initialized) {
        // Quick toggle to signal device presence
        setDTR(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        setDTR(true);
        ESP_LOGD(TAG, "Toggled DTR to signal presence");
    }
    
    return ESP_OK;
}

size_t UsbCdc::availableForWrite(uint8_t instance) {
    if (!m_initialized || instance > 1) {
        return 0;
    }

    size_t available = 0;

    if (tud_mounted()) {
        available += tud_cdc_n_write_available(instance);
    }

    if (m_tx_mutex[instance] != nullptr) {
        if (xSemaphoreTake(m_tx_mutex[instance], 0) == pdTRUE) {
            available += txFreeSpace(instance);
            xSemaphoreGive(m_tx_mutex[instance]);
        }
    } else {
        available += txFreeSpace(instance);
    }

    return available;
}

void UsbCdc::setRxCallback(RxCallback cb) {
    m_rx_callback = cb;
}

UsbCdc::TxCallback UsbCdc::setTxCallback(TxCallback cb) {
    TxCallback previous = m_tx_callback;
    m_tx_callback = cb;
    return previous;
}

void UsbCdc::notifyRxAvailable(uint8_t instance) {
    if (m_rx_callback) {
        m_rx_callback(instance);
    }
}

void UsbCdc::notifyTxMirrored(uint8_t instance, const uint8_t* data, size_t length) {
    if (m_tx_callback && data != nullptr && length > 0) {
        m_tx_callback(instance, data, length);
    }
}

namespace {
constexpr uint64_t BACKPRESSURE_BASE_COOLDOWN_US = 200000; // 200 ms
constexpr uint8_t BACKPRESSURE_MAX_LEVEL = 4;
} // namespace

bool UsbCdc::shouldThrottleWrite(uint8_t instance) {
    if (instance > 1 || !m_initialized) {
        return false;
    }
    const uint64_t now = esp_timer_get_time();
    return now < m_writeBlockUntilUs[instance];
}

void UsbCdc::notifyWriteSuccess(uint8_t instance) {
    if (instance > 1) {
        return;
    }
    m_writeBlockUntilUs[instance] = 0;
    m_backpressureLevel[instance] = 0;
}

void UsbCdc::notifyWriteBackpressure(uint8_t instance) {
    if (instance > 1) {
        return;
    }
    const uint64_t now = esp_timer_get_time();
    const uint8_t level = std::min<uint8_t>(m_backpressureLevel[instance], BACKPRESSURE_MAX_LEVEL);
    const uint64_t cooldown = BACKPRESSURE_BASE_COOLDOWN_US << level;
    m_writeBlockUntilUs[instance] = now + cooldown;
    if (m_backpressureLevel[instance] < BACKPRESSURE_MAX_LEVEL) {
        ++m_backpressureLevel[instance];
    }
}

