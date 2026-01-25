#include "../components/CommandHandlers/include/InterfaceSystemCommandHandler.h"
#include "../include/pin_definitions.h"
#include "ADCHandler.h"
#include "ButtonHandler.h"
#include "CATHandler.h"
#include "CdcSerialHandler.h"
#include "Diagnostics.h"
#include "DisplayLatencyProfiler.h"
#include "EncoderHandler.h"
#include "MultiEncoderHandler.h"
#include "NvsManager.h"
#include "PCF8575Handler.h"
#include "RadioMacroManager.h"
#include "RadioManager.h"
#include "SerialHandler.h"
#include "TCA8418Handler.h"
#include "TCA9548Handler.h"
#include "UsbCdc.h"
#include "WiFiManager.h"
#include "TcpCatBridge.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <algorithm>
#include <string>
#include <string_view>
#include "../components/CommandHandlers/include/BaseCommandHandler.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "led_strip.h"

static constexpr auto TAG = "RRC_Interface";

// Button Test Mode - Set to true to enable keycode diagnostic mode
// In this mode, only button key codes are logged without any other functionality
static constexpr bool BUTTON_TEST_MODE = false;

// WS2812 RGB LED configuration
static constexpr size_t LED_STRIP_LED_COUNT = 1; // Only 1 LED on DevKit
led_strip_handle_t led_strip_handle = nullptr;
static constexpr bool DEBUG = false;

// --- Threading and Synchronization ---
// Semaphores for event-driven wakeups for each task
static SemaphoreHandle_t g_usbMessageReadySem = nullptr;
static SemaphoreHandle_t g_usb2MessageReadySem = nullptr; // Second CDC instance
static SemaphoreHandle_t g_radioMessageReadySem = nullptr;
static SemaphoreHandle_t g_displayMessageReadySem = nullptr;

// Task handles
static TaskHandle_t g_usbTaskHandle = nullptr;
static TaskHandle_t g_usb2TaskHandle = nullptr; // Second CDC instance
static TaskHandle_t g_radioTaskHandle = nullptr;
static TaskHandle_t g_displayTaskHandle = nullptr;
static TaskHandle_t g_mainTaskHandle = nullptr;


// =============================================================================
// Global Instances (organized by functional area)
// =============================================================================
namespace
{
    // --- Serial Communication ---
    CdcSerialHandler usbSerial;                   // Primary USB CDC
    CdcSerialHandler usb2Serial(1);               // Secondary USB CDC
    SerialHandler radioSerial(UART_NUM_1);        // Radio UART
    SerialHandler displaySerial(UART_NUM_2);      // Display UART

    // --- Radio Core ---
    radio::RadioManager radioManager(radioSerial, usbSerial);
    std::unique_ptr<radio::CATHandler> displayCatHandler;  // Display CAT (lazy init)
    std::unique_ptr<radio::CATHandler> usb2CatHandler;     // USB2 CAT (lazy init)
    radio::RadioMacroManager radioMacroManager(radioManager);
    radioManager.setMacroManager(&radioMacroManager);

    // --- System Services (NvsManager created early for ButtonHandler dependency) ---
    NvsManager nvsManager(radioManager);

    // --- Input Handlers ---
    EncoderHandler encoderHandler(PIN_ENCODER_A, PIN_ENCODER_B, &radioManager);
    std::unique_ptr<MultiEncoderHandler> multiEncoderHandler;  // Multi-encoder (lazy init)
    ButtonHandler buttonHandler(&radioManager, &radioMacroManager, &nvsManager);
    ADCHandler adcHandler(&radioManager);

    // --- I2C Bus & Expanders ---
    TCA9548Handler tca9548Handler(TCA9548_I2C_ADDR);
    PCF8575Handler pcf8575Handler(PCF8575_I2C_ADDR, PIN_PCF8575_INT);
    TCA8418Handler tca8418Handler(PIN_TCA8418_INT, 5, 10);    // Main button matrix
    TCA8418Handler tca8418Handler2(PIN_TCA8418_INT_2, 5, 3);  // F1-F6 macros

    // --- System Services (continued) ---
    Diagnostics diagnostics(radioManager);
    wifi::WiFiManager wifiManager;

#ifdef CONFIG_TCP_CAT_BRIDGE_ENABLE
    std::unique_ptr<tcp_cat_bridge::TcpCatBridge> tcpCatBridge0;
    std::unique_ptr<radio::CATHandler> tcp0CatHandler;
    #ifdef CONFIG_TCP_CAT_BRIDGE_ENABLE_DUAL
    std::unique_ptr<tcp_cat_bridge::TcpCatBridge> tcpCatBridge1;
    std::unique_ptr<radio::CATHandler> tcp1CatHandler;
    #endif
#endif

    // --- Debug State ---
    std::string lastRadioCommand;  // For loop prevention debugging
} // namespace

extern "C" {
void app_main(void);
void setup(void);
#ifdef CONFIG_RUN_UNIT_TESTS
void run_all_tests(void);
#endif
}

#ifdef DEBUG
void debugPrintMsg(std::string_view localMsg, std::string_view remoteMsg)
{
    if constexpr (!DEBUG)
        return;

    if (localMsg.length() > 3)
    {
        usbSerial.sendMessage("COM0 Data Received: " + std::string(localMsg));
    }

    if (localMsg.length() <= 3 && localMsg[2] == ';')
    {
        usbSerial.sendMessage("COM0 Request: " + std::string(localMsg) + " len: " + std::to_string(localMsg.length()));
    }

    if (remoteMsg.length() > 3)
    {
        usbSerial.sendMessage("COM1 Data Received: " + std::string(remoteMsg));
    }

    if (remoteMsg.length() <= 3 && remoteMsg[2] == ';')
    {
        usbSerial.sendMessage("COM1 Request: " + std::string(remoteMsg) +
                              " len: " + std::to_string(remoteMsg.length()));
    }
}
#endif

void initializeLED()
{
    ESP_LOGI(TAG, "Initializing WS2812 RGB LED on GPIO %d", PIN_LED);

    // LED strip configuration
    led_strip_config_t strip_config = {.strip_gpio_num = PIN_LED,
                                       .max_leds = LED_STRIP_LED_COUNT,
                                       .led_model = LED_MODEL_WS2812,
                                       .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
                                       .flags = {
                                           .invert_out = false,
                                       }};

    // RMT configuration
    led_strip_rmt_config_t rmt_config = {.clk_src = RMT_CLK_SRC_DEFAULT,
                                         .resolution_hz = 10 * 1000 * 1000, // 10MHz
                                         .mem_block_symbols = 64,
                                         .flags = {
                                             .with_dma = false,
                                         }};

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(ret));
        return;
    }

    // Clear LED (turn off)
    ret = led_strip_clear(led_strip_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "WS2812 RGB LED initialized successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to clear LED strip: %s", esp_err_to_name(ret));
    }
}

void setLEDColor(const uint8_t red, const uint8_t green, const uint8_t blue, const uint8_t brightness_percent)
{
    if (led_strip_handle == nullptr)
    {
        ESP_LOGW(TAG, "LED strip not initialized");
        return;
    }

    // Apply brightness scaling
    const uint8_t scaled_red = red * brightness_percent / 100;
    const uint8_t scaled_green = green * brightness_percent / 100;
    const uint8_t scaled_blue = blue * brightness_percent / 100;

    if (esp_err_t ret = led_strip_set_pixel(led_strip_handle, 0, scaled_red, scaled_green, scaled_blue); ret == ESP_OK)
    {
        ret = led_strip_refresh(led_strip_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "❌ Failed to refresh LED strip: %s", esp_err_to_name(ret));
        }
    }
    else
    {
        ESP_LOGE(TAG, "❌ Failed to set LED pixel: %s", esp_err_to_name(ret));
    }
}

void updateLEDStatus()
{
    const uint8_t currentState = radioManager.getOnOffState();
    if (currentState == 1)
    {
        setLEDColor(0, 255, 0, 5); // Green at 5% brightness
    }
    else
    {
        setLEDColor(255, 0, 0, 2); // Red at 2% brightness
    }
}


void initializeNVS()
{
    ESP_LOGI(TAG, "Initializing NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void configureGPIO()
{
    // Configure encoder GPIOs (TCA8418 INT configured by its handler)
    constexpr gpio_config_t io_conf = {.pin_bit_mask =
                                       (1ULL << PIN_ENCODER_A) | (1ULL << PIN_ENCODER_B),
                                   .mode = GPIO_MODE_INPUT,
                                   .pull_up_en = GPIO_PULLUP_ENABLE,
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .intr_type = GPIO_INTR_DISABLE};
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure GPIO pins: %s", esp_err_to_name(ret));
    }
}

i2c_master_bus_handle_t i2c_bus_handle = nullptr;

void initializeI2C()
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags =
            {
                .enable_internal_pullup = true,
                .allow_pd = false,
            },
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return;
    }

    // Diagnostic scan (DEBUG level)
    Diagnostics::scanI2CBus(i2c_bus_handle);
}

void initializeTCA8418()
{
    // Initialize TCA9548 I2C multiplexer
    esp_err_t ret = tca9548Handler.initialize(i2c_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize TCA9548, I2C multiplexing will not work");
        return;
    }

    // Diagnostic channel scan (DEBUG level)
    Diagnostics::scanTCA9548Channels(i2c_bus_handle, tca9548Handler);

    // Initialize TCA8418 #1 (main buttons) on channel 0
    tca8418Handler.setMuxChannel(&tca9548Handler, TCA9548_CHANNEL_TCA8418_1);
    if (!tca8418Handler.initialize(i2c_bus_handle))
    {
        ESP_LOGE(TAG, "Failed to initialize TCA8418 #1, matrix keys will not work");
    }
    else
    {
        buttonHandler.setupTCA8418KeyMappings(&tca8418Handler);
    }

    // Initialize TCA8418 #2 (F1-F6 macros) on channel 1
    tca8418Handler2.setMuxChannel(&tca9548Handler, TCA9548_CHANNEL_TCA8418_2);
    if (!tca8418Handler2.initialize(i2c_bus_handle))
    {
        ESP_LOGE(TAG, "Failed to initialize TCA8418 #2, F1-F6 macro keys will not work");
    }
    else
    {
        buttonHandler.setupF1F6KeyMappings(&tca8418Handler2);
    }

    // Initialize PCF8575 (I2C encoders) on channel 2
    pcf8575Handler.setMuxChannel(&tca9548Handler, TCA9548_CHANNEL_PCF8575);
    ret = tca9548Handler.selectChannel(TCA9548_CHANNEL_PCF8575);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to select TCA9548 channel %d for PCF8575", TCA9548_CHANNEL_PCF8575);
        return;
    }
    ret = pcf8575Handler.initialize(i2c_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize PCF8575, I2C encoders will not work");
    }
}

void initializeMultiEncoderHandler()
{
    multiEncoderHandler = std::make_unique<MultiEncoderHandler>(&radioManager);

    // Configure callbacks before init (SimpleQuadratureEncoder auto-starts)
    multiEncoderHandler->configureDefaultCallbacks();

    esp_err_t ret = multiEncoderHandler->initializeDirectEncoders();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize direct GPIO encoders");
        return;
    }

    ret = multiEncoderHandler->initializeI2CEncoders(&pcf8575Handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C encoders");
        return;
    }

    multiEncoderHandler->startAll();

    // Prime AF/RF gain state from radio
    radioManager.dispatchMessage(radioManager.getPanelCATHandler(), "AG0;");
    radioManager.dispatchMessage(radioManager.getPanelCATHandler(), "RG;");
}

void initializeSystemComponents()
{
    encoderHandler.setup();
    ESP_ERROR_CHECK(radio::RadioManager::init());
    radioManager.setMacroManager(&radioMacroManager);
    ESP_ERROR_CHECK(radioSerial.setupUart(57600, PIN_UART1_TX, PIN_UART1_RX));
    ESP_ERROR_CHECK(displaySerial.setupUart(57600, PIN_DISPLAY_TX, PIN_DISPLAY_RX));
    vTaskDelay(pdMS_TO_TICKS(100));
}

void initializeUsbCdc()
{
#if (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_RUN_UNIT_TESTS))
    ESP_ERROR_CHECK(UsbCdc::init());
#else
    ESP_LOGW(TAG, "USB CDC not supported on this target");
#endif
}

// --- TASKS ---

[[noreturn]] static void usb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "USB task started");
    static uint32_t messageCount = 0;
    constexpr uint32_t STACK_CHECK_INTERVAL = 1000; // Check every 1000 messages
    constexpr UBaseType_t STACK_LOW_THRESHOLD = 1024; // Warn if < 1KB free
    constexpr TickType_t WDT_TIMEOUT_TICKS = pdMS_TO_TICKS(4000); // 4s < 5s WDT timeout

    // Register with task watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (true)
    {
        // Reset watchdog at start of each iteration
        esp_task_wdt_reset();

        // Wait for a message from USB with timeout for WDT
        if (xSemaphoreTake(g_usbMessageReadySem, WDT_TIMEOUT_TICKS) == pdTRUE)
        {
            // Event-driven: callback fired, drain all available data
            while (true)
            {
                const auto messageResult = usbSerial.getMessageView();
                if (messageResult.first != ESP_OK || messageResult.second.empty())
                {
                    break; // no more data in CDC ring
                }
                ESP_LOGV(TAG, "USB message received: %.*s", static_cast<int>(messageResult.second.length()),
                         messageResult.second.data());
                // dispatchMessage internally uses dispatchMutex_ for serialization - no global mutex needed
                (void)radioManager.dispatchMessage(radioManager.getLocalCATHandler(), messageResult.second);

                // Periodic stack watermark monitoring
                if (++messageCount % STACK_CHECK_INTERVAL == 0)
                {
                    const UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
                    if (watermark < STACK_LOW_THRESHOLD)
                    {
                        ESP_LOGW(TAG, "⚠️ usb_task stack low: %u bytes free (consider increasing stack size)", watermark);
                    }
                    ESP_LOGD(TAG, "usb_task stack watermark: %u bytes free", watermark);
                }
            }
        }
    }
}

[[noreturn]] static void usb2_task(void *pvParameters)
{
    ESP_LOGI(TAG, "USB2 task started");
    static uint32_t messageCount = 0;
    constexpr uint32_t STACK_CHECK_INTERVAL = 1000; // Check every 1000 messages
    constexpr UBaseType_t STACK_LOW_THRESHOLD = 1024; // Warn if < 1KB free
    constexpr TickType_t WDT_TIMEOUT_TICKS = pdMS_TO_TICKS(4000); // 4s < 5s WDT timeout

    // Register with task watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (true)
    {
        // Reset watchdog at start of each iteration
        esp_task_wdt_reset();

        // Wait for a message from USB CDC instance 1 with timeout for WDT
        if (xSemaphoreTake(g_usb2MessageReadySem, WDT_TIMEOUT_TICKS) == pdTRUE)
        {
            // Event-driven: callback fired, drain all available data (quiet logs)
            while (true)
            {
                const auto messageResult = usb2Serial.getMessageView();
                if (messageResult.first != ESP_OK || messageResult.second.empty())
                {
                    break; // no more data in CDC ring
                }
                ESP_LOGV(TAG, "USB2 message received: %.*s", static_cast<int>(messageResult.second.length()),
                         messageResult.second.data());
                // dispatchMessage internally uses dispatchMutex_ for serialization - no global mutex needed
                if (usb2CatHandler)
                {
                    (void)radioManager.dispatchMessage(*usb2CatHandler, messageResult.second);
                }

                // Periodic stack watermark monitoring
                if (++messageCount % STACK_CHECK_INTERVAL == 0)
                {
                    const UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
                    if (watermark < STACK_LOW_THRESHOLD)
                    {
                        ESP_LOGW(TAG, "⚠️ usb2_task stack low: %u bytes free (consider increasing stack size)", watermark);
                    }
                    ESP_LOGD(TAG, "usb2_task stack watermark: %u bytes free", watermark);
                }
            }
        }
    }
}

[[noreturn]] static void radio_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Radio task started");
    constexpr TickType_t WDT_TIMEOUT_TICKS = pdMS_TO_TICKS(4000); // 4s < 5s WDT timeout

    // Register with task watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (true)
    {
        // Reset watchdog at start of each iteration
        esp_task_wdt_reset();

        if (xSemaphoreTake(g_radioMessageReadySem, WDT_TIMEOUT_TICKS) == pdTRUE)
        {
            while (radioSerial.hasMessage())
            {
                const auto messageResult = radioSerial.getMessageView();
                if (messageResult.first != ESP_OK || messageResult.second.empty())
                {
                    continue;
                }

                ESP_LOGV(TAG, "📻 Radio: '%.*s'", static_cast<int>(messageResult.second.length()),
                         messageResult.second.data());

                // Dispatch to command handlers
                const bool wasHandled =
                    radioManager.dispatchMessage(radioManager.getRemoteCATHandler(), messageResult.second);

                // Forward to USB if not handled by a command handler
                if (!wasHandled && radioManager.shouldForwardToUSB(messageResult.second))
                {
                    usbSerial.sendMessage(messageResult.second);
                }

                // Forward to Display (tuning suppression handled by ForwardingPolicy)
                if (!wasHandled && radioManager.shouldForwardToDisplay(messageResult.second))
                {
                    // Block bare query commands (3 chars like "FA;") except RX/TX status notifications
                    const bool isStatusNotification =
                        (messageResult.second[0] == 'R' && messageResult.second[1] == 'X') ||
                        (messageResult.second[0] == 'T' && messageResult.second[1] == 'X');
                    const bool isBareQuery = messageResult.second.length() == 3 &&
                                              messageResult.second[2] == ';' &&
                                              !isStatusNotification;

                    if (!isBareQuery)
                    {
                        const esp_err_t sendResult = displaySerial.sendMessage(messageResult.second);
                        if (sendResult != ESP_OK)
                        {
                            ESP_LOGW(TAG, "Display forward failed (%s)", esp_err_to_name(sendResult));
                        }
                    }
                }
            }
        }
    }
}

[[noreturn]] static void display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Display task started");
    constexpr TickType_t WDT_TIMEOUT_TICKS = pdMS_TO_TICKS(4000); // 4s < 5s WDT timeout

    // Register with task watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (true)
    {
        // Reset watchdog at start of each iteration
        esp_task_wdt_reset();

        // Wait for a message from the display with timeout for WDT
        if (xSemaphoreTake(g_displayMessageReadySem, WDT_TIMEOUT_TICKS) == pdTRUE)
        {
            while (displaySerial.hasMessage())
            {
                const auto [fst, snd] = displaySerial.getMessageView();
                if (fst != ESP_OK || snd.empty())
                {
                    continue;
                }

                ESP_LOGD(TAG, "📺 Display message received: '%.*s'", static_cast<int>(snd.length()),
                         snd.data());

                // Handle display status responses (UIPS/UIPT) before standard CAT dispatch
                if (snd.length() >= 5 && snd[0] == 'U' && snd[1] == 'I')
                {
                    if (snd[2] == 'P' && snd[3] == 'S' && snd.length() >= 6)
                    {
                        // UIPS<n>; - Display awake state (0=asleep, 1=awake)
                        const bool awake = (snd[4] == '1');
                        radioManager.setDisplayAwake(awake);
                        continue; // Don't forward to CAT dispatcher
                    }
                    else if (snd[2] == 'P' && snd[3] == 'T' && snd.length() >= 6)
                    {
                        // UIPT<n>; - Display screensaver timeout in minutes
                        // Parse number (could be 0, 5, 10, 15, 30)
                        int timeout = 0;
                        for (size_t i = 4; i < snd.length() && snd[i] != ';'; ++i)
                        {
                            if (snd[i] >= '0' && snd[i] <= '9')
                                timeout = timeout * 10 + (snd[i] - '0');
                        }
                        radioManager.setDisplayScreensaverTimeout(static_cast<uint8_t>(timeout));
                        continue; // Don't forward to CAT dispatcher
                    }
                }

                // dispatchMessage internally uses dispatchMutex_ for serialization - no global mutex needed
                if (displayCatHandler)
                {
                    (void)radioManager.dispatchMessage(*displayCatHandler, snd);
                }
            }
        }
    }
}

[[noreturn]] static void main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Main task started");
    uint8_t lastOnOffState = 255;
    uint64_t lastAiEnforceUs = 0; // throttle AI enforcement
    // Periodic PS0; emulation when radio is OFF and CDC connected (TS-590SG behavior)
    uint64_t lastPs0Usb0Us = 0;
    uint64_t lastPs0Usb1Us = 0;
    uint64_t lastProfilerLogUs = 0; // Display latency profiler logging

    // Register with task watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (true)
    {
        // Reset watchdog at start of each iteration
        esp_task_wdt_reset();
        auto &state = radioManager.getState();
        const uint8_t currentOnOffState = radioManager.getOnOffState();
        const bool keepAliveState = state.keepAlive.load();

        if (currentOnOffState == 1 && keepAliveState)
        {
            // Periodic polling disabled. Keep lightweight maintenance only.
            if (radioManager.getVfoAFrequency() > 0)
            {
                radioManager.updateBandFromVfoA();
            }

            // Enforce radio AI mode only if any client still wants AI updates
            const uint64_t nowUs = esp_timer_get_time();
            if (nowUs - lastAiEnforceUs >= radio::BaseCommandHandler::TTL_STATUS)
            { // align with STATUS TTL
                const uint8_t desiredAi =
                    std::max({state.usbCdc0AiMode.load(), state.usbCdc1AiMode.load(), state.displayAiMode.load()});
                const uint8_t reportedAi = state.aiMode.load();

                if (desiredAi > 0 && reportedAi != desiredAi)
                {
                    ESP_LOGI(TAG, "AI enforcement: desired=%u, reported=%u — sending AI%u; to radio", desiredAi,
                             reportedAi, desiredAi);
                    std::string aiCmd = "AI" + std::to_string(desiredAi) + ";";
                    radioManager.sendRawRadioCommand(aiCmd);
                    // Query to confirm: SET commands don't receive responses per TS-590SG protocol
                    radioManager.sendRawRadioCommand("AI;");
                }
                lastAiEnforceUs = nowUs;
            }
        }
        else
        {
            // Radio OFF: emulate TS-590SG periodic PS0; when CDC is connected
            const uint64_t nowUs = esp_timer_get_time();
            // Only emit if we have high confidence radio is actually off:
            // - Interface powerOn is false
            // - And either we consider radio not connected, or we have no fresh PS from radio recently
            const bool radioConnected = radioManager.isRadioConnected();
            const bool psFresh = state.commandCache.isFresh("PS", nowUs, 2000000ULL); // 2s
            const bool definitelyOff = (currentOnOffState == 0) && (!radioConnected || !psFresh);
#if (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3))
            if (UsbCdc::isConnected(0) && definitelyOff)
            {
                if (nowUs - lastPs0Usb0Us >= 1000000ULL)
                { // 1 second
                    radioManager.sendToSource(radio::CommandSource::UsbCdc0, "PS0;");
                    lastPs0Usb0Us = nowUs;
                }
            }
            else
            {
                lastPs0Usb0Us = 0; // reset timer when disconnected
            }
            if (UsbCdc::isConnected(1) && definitelyOff)
            {
                if (nowUs - lastPs0Usb1Us >= 1000000ULL)
                { // 1 second
                    radioManager.sendToSource(radio::CommandSource::UsbCdc1, "PS0;");
                    lastPs0Usb1Us = nowUs;
                }
            }
            else
            {
                lastPs0Usb1Us = 0;
            }
#endif
        }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (currentOnOffState != lastOnOffState)
        {
            updateLEDStatus();
            lastOnOffState = currentOnOffState;
        }
#endif

        // Display latency profiler - log summary every 10 seconds
        {
            const uint64_t nowUs = esp_timer_get_time();
            constexpr uint64_t PROFILER_INTERVAL_US = 10000000; // 10 seconds
            if (nowUs - lastProfilerLogUs >= PROFILER_INTERVAL_US)
            {
                radio::DisplayLatencyProfiler::instance().logAndReset();
                lastProfilerLogUs = nowUs;
            }
        }

        // Other periodic, non-critical actions can go here
        vTaskDelay(pdMS_TO_TICKS(100)); // Main loop delay - reduced from 10ms to reduce CPU overhead
    }
}

void setup()
{
    ESP_LOGI(TAG, "Starting setup");

    // Reduce TinyUSB CDC ACM flush warnings to VERBOSE level
    esp_log_level_set("tusb_cdc_acm", ESP_LOG_ERROR);

    ESP_ERROR_CHECK(nvsManager.init());
    nvsManager.setupPowerStateCallback();
    configureGPIO();

    ESP_LOGI(TAG, "Installing GPIO ISR service");
    esp_err_t install_err = gpio_install_isr_service(0);
    if (install_err != ESP_OK && install_err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(install_err));
    }

    initializeI2C();
    initializeTCA8418();

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    initializeLED();
    setLEDColor(255, 0, 0, 2); // Initial state: Red
#endif

#if (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_RUN_UNIT_TESTS))
    initializeUsbCdc();
#endif
    initializeSystemComponents();

    // Initialize WiFi for antenna switch functionality
    ESP_LOGI(TAG, "Initializing WiFi");
    if (wifiManager.initialize())
    {
        ESP_LOGI(TAG, "Starting WiFi connection");
        wifiManager.connect();

        // Wait for WiFi connection before starting TCP bridge
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
        if (wifiManager.waitForConnection(10000)) {
            ESP_LOGI(TAG, "WiFi connected! IP: %s", wifiManager.getIpAddress().c_str());

#ifdef CONFIG_TCP_CAT_BRIDGE_ENABLE
            // Initialize TCP-CAT bridge for TCP port 0
            ESP_LOGI(TAG, "Initializing TCP-CAT bridge for port 0");
            tcpCatBridge0 = std::make_unique<tcp_cat_bridge::TcpCatBridge>(
                CONFIG_TCP_CAT_BRIDGE_PORT_0,  // port
                0                              // bridgeId
            );

            // Create TCP0 CAT handler
            tcp0CatHandler = std::make_unique<radio::CATHandler>(
                radioManager.getCommandDispatcher(), radioManager, radioSerial, usbSerial, radio::CommandSource::Tcp0
            );

            // Wire TCP0 frame callback to dispatch to TCP0 CAT handler
            if (tcpCatBridge0 && tcp0CatHandler)
            {
                tcpCatBridge0->setIncomingFrameCallback(
                    [&](const std::string_view frame)
                    {
                        if (frame.empty() || !tcp0CatHandler)
                        {
                            return;
                        }
                        radioManager.dispatchMessage(*tcp0CatHandler, frame);
                    });
            }

            // Register TCP0 bridge with RadioManager
            radioManager.setTcp0Bridge(tcpCatBridge0.get());

            esp_err_t bridge0_err = tcpCatBridge0->start();
            if (bridge0_err == ESP_OK) {
                ESP_LOGI(TAG, "TCP-CAT bridge started for port 0 on port %d", CONFIG_TCP_CAT_BRIDGE_PORT_0);
                ESP_LOGI(TAG, "  Connect via: nc %s %d",
                         wifiManager.getIpAddress().c_str(), CONFIG_TCP_CAT_BRIDGE_PORT_0);
            } else {
                ESP_LOGW(TAG, "Failed to start TCP-CAT bridge for port 0: %s", esp_err_to_name(bridge0_err));
            }

            #ifdef CONFIG_TCP_CAT_BRIDGE_ENABLE_DUAL
            // Initialize TCP-CAT bridge for TCP port 1
            ESP_LOGI(TAG, "Initializing TCP-CAT bridge for port 1");
            tcpCatBridge1 = std::make_unique<tcp_cat_bridge::TcpCatBridge>(
                CONFIG_TCP_CAT_BRIDGE_PORT_1,  // port
                1                              // bridgeId
            );

            // Create TCP1 CAT handler
            tcp1CatHandler = std::make_unique<radio::CATHandler>(
                radioManager.getCommandDispatcher(), radioManager, radioSerial, usbSerial, radio::CommandSource::Tcp1
            );

            // Wire TCP1 frame callback to dispatch to TCP1 CAT handler
            if (tcpCatBridge1 && tcp1CatHandler)
            {
                tcpCatBridge1->setIncomingFrameCallback(
                    [&](const std::string_view frame)
                    {
                        if (frame.empty() || !tcp1CatHandler)
                        {
                            return;
                        }
                        radioManager.dispatchMessage(*tcp1CatHandler, frame);
                    });
            }

            // Register TCP1 bridge with RadioManager
            radioManager.setTcp1Bridge(tcpCatBridge1.get());

            esp_err_t bridge1_err = tcpCatBridge1->start();
            if (bridge1_err == ESP_OK) {
                ESP_LOGI(TAG, "TCP-CAT bridge started for port 1 on port %d", CONFIG_TCP_CAT_BRIDGE_PORT_1);
                ESP_LOGI(TAG, "  Connect via: nc %s %d",
                         wifiManager.getIpAddress().c_str(), CONFIG_TCP_CAT_BRIDGE_PORT_1);
            } else {
                ESP_LOGW(TAG, "Failed to start TCP-CAT bridge for port 1: %s", esp_err_to_name(bridge1_err));
            }
            #endif
#endif
        } else {
            ESP_LOGW(TAG, "WiFi connection timeout - TCP bridge not started");
        }
    }
    else
    {
        ESP_LOGW(TAG, "WiFi initialization failed - antenna switch will use radio fallback");
    }

    if (!displayCatHandler)
    {
        displayCatHandler =
            std::make_unique<radio::CATHandler>(radioManager.getCommandDispatcher(), radioManager, radioSerial,
                                                displaySerial, radio::CommandSource::Display);
        ESP_LOGI(TAG, "Display CAT handler initialized");
    }

    // Initialize second USB CDC CAT handler (same pattern as display)
    if (!usb2CatHandler)
    {
        usb2CatHandler = std::make_unique<radio::CATHandler>(radioManager.getCommandDispatcher(), radioManager,
                                                             radioSerial, usb2Serial,
                                                             radio::CommandSource::UsbCdc1);
        ESP_LOGI(TAG, "USB2 CDC CAT handler initialized");
    }

    radioManager.setDisplaySerial(&displaySerial);

    // Query display state on startup (responses handled in display_task)
    displaySerial.sendMessage("UIPS;");  // Query awake/asleep state
    displaySerial.sendMessage("UIPT;");  // Query screensaver timeout

    // Register CDC1 with RadioManager so answers can be targeted to CDC1
    radioManager.setUsbCdc1Serial(&usb2Serial);

    // Create mutex and semaphores
    g_usbMessageReadySem = xSemaphoreCreateBinary();
    g_usb2MessageReadySem = xSemaphoreCreateBinary(); // Second CDC instance
    g_radioMessageReadySem = xSemaphoreCreateBinary();
    g_displayMessageReadySem = xSemaphoreCreateBinary();

    // Register callbacks to signal the correct semaphores
    radioSerial.setOnFrameCallback([]() { xSemaphoreGive(g_radioMessageReadySem); });
    displaySerial.setOnFrameCallback([]() { xSemaphoreGive(g_displayMessageReadySem); });
#if (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_RUN_UNIT_TESTS))
    UsbCdc::setRxCallback(
        [](const uint8_t instance)
        {
            if (instance == 0)
            {
                xSemaphoreGive(g_usbMessageReadySem);
            }
            else if (instance == 1)
            {
                xSemaphoreGive(g_usb2MessageReadySem);
            }
        });
#endif
    radioManager.setImmediatePollingCallback([]() { xSemaphoreGive(g_radioMessageReadySem); });

    // Start RadioManager tasks (must be after peripheral setup, before using radio)
    ESP_LOGI(TAG, "Starting RadioManager tasks");
    ESP_ERROR_CHECK(radioManager.startTasks());

    // Load ButtonHandler mode memory from NVS (must be after nvs_flash_init())
    ESP_LOGI(TAG, "Loading button mode memory from NVS");
    buttonHandler.loadModeMemoryFromNvs();

    adcHandler.start();
    buttonHandler.start();
    encoderHandler.start();

    // Initialize multi-encoder handler (must be after I2C and system components)
    // Now using SimpleQuadratureEncoder (GPIO ISR, no PCNT) - won't interfere with RMS250
    initializeMultiEncoderHandler();

    ESP_ERROR_CHECK(nvsManager.loadAndSyncOnStartup());

#if (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_RUN_UNIT_TESTS))
    ESP_LOGI(TAG, "Waiting for CDC connection...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    usbSerial.sendMessage("USB CDC Interface Ready");
#endif

    ESP_LOGI(TAG, "Setup complete, creating tasks");
}

void app_main()
{
#ifdef CONFIG_RUN_UNIT_TESTS
    run_all_tests();
#else
    // Check if button test mode is enabled
    if constexpr (BUTTON_TEST_MODE) {
        ESP_LOGI(TAG, "BUTTON_TEST_MODE enabled - entering diagnostic mode");
        Diagnostics::runButtonTestMode(
            PIN_I2C_SDA, PIN_I2C_SCL,
            tca9548Handler, tca8418Handler, tca8418Handler2,
            TCA9548_CHANNEL_TCA8418_1, TCA9548_CHANNEL_TCA8418_2);
        // Never returns - infinite loop in test mode
    }

    setup();
    diagnostics.start();

    // Create tasks
    xTaskCreate(usb_task, "usb_task", 8192, NULL, 10, &g_usbTaskHandle);
    xTaskCreate(usb2_task, "usb2_task", 8192, NULL, 10, &g_usb2TaskHandle);
    xTaskCreate(radio_task, "radio_task", 8192, NULL, 8, &g_radioTaskHandle);
    xTaskCreate(display_task, "display_task", 4096, NULL, 8, &g_displayTaskHandle);
    xTaskCreate(main_task, "main_task", 4096, NULL, 5, &g_mainTaskHandle);

    // Configure diagnostics to monitor task stacks
    diagnostics.setTaskHandles(g_usbTaskHandle, g_usb2TaskHandle, g_radioTaskHandle, g_displayTaskHandle,
                               g_mainTaskHandle);

    // Query radio power state on startup to establish authoritative state
    // This ensures we respond correctly to RRC-1258 keepalive queries
    ESP_LOGI(TAG, "Querying radio power state on startup...");
    radioSerial.sendMessage("PS;");

    ESP_LOGI(TAG, "All tasks created. Main task is now complete.");
#endif
}

#ifdef CONFIG_RUN_UNIT_TESTS
// Forward declaration of the proper test runner from unit_test component
extern "C" void run_all_tests(void);
#endif
