#include "Diagnostics.h"
#include "TCA9548Handler.h"
#include "TCA8418Handler.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <cstdlib>
#include <cstring>

Diagnostics::Diagnostics(radio::RadioManager& radioManager)
    : m_radioManager(radioManager), m_taskHandle(nullptr) {}

void Diagnostics::setTaskHandles(TaskHandle_t usbTask, TaskHandle_t usb2Task,
                                TaskHandle_t radioTask, TaskHandle_t displayTask,
                                TaskHandle_t mainTask) {
    m_monitoredTasks.clear();
    m_monitoredTasks.reserve(5);
    
    // Stack sizes must match the xTaskCreate() calls in main.cpp, otherwise
    // stackUsedBytes = stackSize - highWaterMark underflows (UBaseType_t is unsigned).
    if (usbTask) {
        m_monitoredTasks.push_back({usbTask, "usb_task", 8192});
    }
    if (usb2Task) {
        m_monitoredTasks.push_back({usb2Task, "usb2_task", 8192});
    }
    if (radioTask) {
        m_monitoredTasks.push_back({radioTask, "radio_task", 8192});
    }
    if (displayTask) {
        m_monitoredTasks.push_back({displayTask, "display_task", 6144});
    }
    if (mainTask) {
        m_monitoredTasks.push_back({mainTask, "main_task", 4096});
    }
}

void Diagnostics::printMemoryStatus() const {
    // Internal SRAM monitoring
    const size_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t internalLargest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    const size_t internalTotal = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    const size_t internalUsed = internalTotal - internalFree;
    const float internalUsedPercent = internalTotal > 0 ? (float(internalUsed) / float(internalTotal)) * 100.0f : 0.0f;
    
    ESP_LOGD(TAG, "Memory: SRAM free=%zu/%zu largest=%zu min=%lu used=%.1f%%",
             internalFree, internalTotal, internalLargest,
             static_cast<unsigned long>(esp_get_minimum_free_heap_size()), internalUsedPercent);
    
    // SPIRAM monitoring (if available)
    const size_t spiramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (spiramFree > 0) {
        const size_t spiramLargest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        const size_t spiramTotal = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        const size_t spiramUsed = spiramTotal - spiramFree;
        const float spiramUsedPercent = spiramTotal > 0 ? (float(spiramUsed) / float(spiramTotal)) * 100.0f : 0.0f;
        
        ESP_LOGD(TAG, "Memory: SPIRAM free=%zu/%zu largest=%zu used=%.1f%%",
                 spiramFree, spiramTotal, spiramLargest, spiramUsedPercent);
    }
}

void Diagnostics::printStackStatus() const {
    if (m_monitoredTasks.empty()) {
        ESP_LOGW(TAG, "No tasks configured for stack monitoring");
        return;
    }
    
    ESP_LOGV(TAG, "Task stacks: name used/total usage free");
    for (const auto& taskInfo : m_monitoredTasks) {
        if (taskInfo.handle) {
            // Get stack high water mark (in words, so multiply by 4 for bytes on 32-bit system)
            const UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(taskInfo.handle);
            const UBaseType_t stackRemainingBytes = stackRemaining * sizeof(StackType_t);
            const UBaseType_t stackUsedBytes = taskInfo.stackSize - stackRemainingBytes;
            const float stackUsagePercent = (float(stackUsedBytes) / float(taskInfo.stackSize)) * 100.0f;
            
            ESP_LOGV(TAG, "Task stack: %s %lu/%lu used=%.1f%% free=%lu",
                     taskInfo.name, 
                     stackUsedBytes,
                     taskInfo.stackSize,
                     stackUsagePercent,
                     stackRemainingBytes);
            
            // Warn about high stack usage
            if (stackUsagePercent > 80.0f) {
                ESP_LOGW(TAG, "⚠️  HIGH STACK USAGE for task [%s]: %.1f%% used", 
                         taskInfo.name, stackUsagePercent);
            }
        } else {
            ESP_LOGW(TAG, "Task [%s]: Handle is null", taskInfo.name);
        }
    }
}

void Diagnostics::printCpuStatus() {
    // Collect per-task runtime data via the STRUCT API (uxTaskGetSystemState),
    // which returns a TaskStatus_t[] plus a total-runtime out-param. No text
    // table, no strtok/sscanf parsing, and no dependence on FreeRTOS column layout.
    const UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    auto taskStatusArray = static_cast<TaskStatus_t*>(
        heap_caps_malloc(taskCount * sizeof(TaskStatus_t), MALLOC_CAP_8BIT));
    if (!taskStatusArray) {
        ESP_LOGE(TAG, "Failed to allocate buffer for CPU monitoring");
        return;
    }

    uint32_t totalRuntime = 0;
    const UBaseType_t actualTaskCount = uxTaskGetSystemState(taskStatusArray, taskCount, &totalRuntime);

    // Extract idle-task runtimes directly from the struct array
    uint32_t idle0Runtime = 0, idle1Runtime = 0;
    for (UBaseType_t i = 0; i < actualTaskCount; i++) {
        const char* taskName = taskStatusArray[i].pcTaskName;
        if (strcmp(taskName, "IDLE0") == 0) {
            idle0Runtime = taskStatusArray[i].ulRunTimeCounter;
        } else if (strcmp(taskName, "IDLE1") == 0) {
            idle1Runtime = taskStatusArray[i].ulRunTimeCounter;
        }
    }

    // Calculate deltas (handles wraparound automatically)
    const uint32_t deltaIdle0 = idle0Runtime - m_lastIdle0Runtime;
    const uint32_t deltaIdle1 = idle1Runtime - m_lastIdle1Runtime;
    const uint32_t deltaTotal = totalRuntime - m_lastTotalTime;

    // totalRuntime is the COMBINED run-time across all cores, so per-core wall time
    // over the interval is deltaTotal / core count. Normalize each single-core idle
    // delta against that (not the combined total) or a fully idle core reads ~50%
    // busy on a 2-core chip. Clamp to [0,100] to absorb counter rounding.
    if (deltaTotal > 0 && m_lastTotalTime > 0) {
        const float perCoreDelta = static_cast<float>(deltaTotal) / CONFIG_FREERTOS_NUMBER_OF_CORES;

        auto clampPercent = [](float p) { return p < 0.0f ? 0.0f : (p > 100.0f ? 100.0f : p); };
        const float idle0Percent = clampPercent(perCoreDelta > 0.0f ? static_cast<float>(deltaIdle0) / perCoreDelta * 100.0f : 0.0f);
        const float idle1Percent = clampPercent(perCoreDelta > 0.0f ? static_cast<float>(deltaIdle1) / perCoreDelta * 100.0f : 0.0f);

        const float core0Usage = 100.0f - idle0Percent;
        const float core1Usage = 100.0f - idle1Percent;

        ESP_LOGD(TAG, "CPU: C0=%.1f%% C1=%.1f%% total=%.1f%% idle(C0=%.1f%% C1=%.1f%%)",
                 core0Usage, core1Usage, (core0Usage + core1Usage) / 2.0f,
                 idle0Percent, idle1Percent);

        // Print top active tasks
        ESP_LOGV(TAG, "CPU tasks: name core cpu priority stackFree");

        for (UBaseType_t i = 0; i < actualTaskCount; i++) {
            const char* name = taskStatusArray[i].pcTaskName;
            const uint32_t runtime = taskStatusArray[i].ulRunTimeCounter;
            const float percentFloat = totalRuntime > 0
                ? (static_cast<float>(runtime) * 100.0f) / static_cast<float>(totalRuntime)
                : 0.0f;

            // Skip IDLE tasks and show tasks with measurable activity
            if (strncmp(name, "IDLE", 4) != 0 && (percentFloat > 0.0f || runtime > 1000)) {
                const char* coreStr = "?";
                const BaseType_t coreAffinity = xTaskGetCoreID(taskStatusArray[i].xHandle);
                if (coreAffinity == 0) coreStr = "0";
                else if (coreAffinity == 1) coreStr = "1";
                else coreStr = "B";

                const UBaseType_t priority = taskStatusArray[i].uxCurrentPriority;
                const uint32_t stackFree = taskStatusArray[i].usStackHighWaterMark * sizeof(StackType_t);

                ESP_LOGV(TAG, "CPU task: %-16s core=%s cpu=%.1f%% prio=%lu stackFree=%lu",
                         name, coreStr, percentFloat, priority, stackFree);
            }
        }
    }

    // Update baseline for next iteration
    m_lastIdle0Runtime = idle0Runtime;
    m_lastIdle1Runtime = idle1Runtime;
    m_lastTotalTime = totalRuntime;

    heap_caps_free(taskStatusArray);
}

void Diagnostics::start() {
    // Low priority background task. Boot-time creation failure is unrecoverable.
    if (xTaskCreate(taskWrapper, "DiagnosticsTask", 4096, this, 3, &m_taskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Boot failure: DiagnosticsTask creation failed (out of heap) - aborting");
        abort();
    }
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void Diagnostics::task() {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(PRINT_INTERVAL_MS));

#ifndef CONFIG_RUN_UNIT_TESTS
        // CPU utilization monitoring
        printCpuStatus();

        // Memory monitoring
        printMemoryStatus();

        // Per-task stack high-water monitoring
        printStackStatus();

        auto [totalCommandsProcessed, localCommandsProcessed, remoteCommandsProcessed] =
            m_radioManager.getStatistics();
        auto dispatcherStats = m_radioManager.getCommandDispatcher().getStatistics();

        // CAT parser statistics (summed across all per-source parsers)
        const auto sumParser = [this]() {
            radio::UnifiedCATStatistics s;
            for (const radio::CATHandler* h : {
                     &m_radioManager.getLocalCATHandler(), &m_radioManager.getRemoteCATHandler(),
                     &m_radioManager.getPanelCATHandler(), &m_radioManager.getMacroCATHandler()}) {
                const auto& p = h->getParserStatistics();
                s.totalMessagesParsed += p.totalMessagesParsed;
                s.setCommands += p.setCommands;
                s.readCommands += p.readCommands;
                s.answerCommands += p.answerCommands;
                s.parseErrors += p.parseErrors;
                s.unknownCommands += p.unknownCommands;
            }
            return s;
        }();
        ESP_LOGD(TAG,
                 "Commands: radio(total=%lu local=%lu remote=%lu) "
                 "parser(msg=%zu set=%zu read=%zu ans=%zu parseErr=%zu unknown=%zu) "
                 "dispatch(total=%lu handled=%lu unhandled=%lu handlerErr=%lu)",
                 static_cast<unsigned long>(totalCommandsProcessed),
                 static_cast<unsigned long>(localCommandsProcessed),
                 static_cast<unsigned long>(remoteCommandsProcessed),
                 sumParser.totalMessagesParsed, sumParser.setCommands, sumParser.readCommands,
                 sumParser.answerCommands, sumParser.parseErrors, sumParser.unknownCommands,
                 static_cast<unsigned long>(dispatcherStats.totalCommandsDispatched.load()),
                 static_cast<unsigned long>(dispatcherStats.commandsHandled.load()),
                 static_cast<unsigned long>(dispatcherStats.commandsUnhandled.load()),
                 static_cast<unsigned long>(dispatcherStats.handlerErrors.load()));

        // Error response diagnostics
        if (dispatcherStats.totalErrorResponses > 0) {
            const uint64_t currentTime = esp_timer_get_time();
            const char* source = dispatcherStats.lastCommandSource[0] == '\0'
                ? "unknown" : dispatcherStats.lastCommandSource;
            const uint64_t lastErrorAgeMs = dispatcherStats.lastErrorTime > 0 && currentTime >= dispatcherStats.lastErrorTime
                ? (currentTime - dispatcherStats.lastErrorTime) / 1000 : 0;
            const size_t recentErrors = dispatcherStats.recentErrorCount(currentTime);
            float lifetimeRate = 0.0f;
            if (const uint64_t runtimeMs = currentTime / 1000; runtimeMs > 0) {
                lifetimeRate = static_cast<float>(dispatcherStats.totalErrorResponses) / (runtimeMs / 60000.0f);
            }

            ESP_LOGD(TAG,
                     "Radio errors: total=%zu (?=%zu E=%zu O=%zu) bursts=%zu recent5s=%zu "
                     "meanInterval=%llums lifetimeRate=%.2f/min lastAge=%llums",
                     dispatcherStats.totalErrorResponses, dispatcherStats.questionMarkErrors,
                     dispatcherStats.eErrors, dispatcherStats.oErrors, dispatcherStats.errorBursts,
                     recentErrors, dispatcherStats.averageErrorInterval / 1000, lifetimeRate,
                     static_cast<unsigned long long>(lastErrorAgeMs));

            if (dispatcherStats.lastCommandBeforeErrorTime > 0) {
                const uint64_t commandAge = (currentTime - dispatcherStats.lastCommandBeforeErrorTime) / 1000;
                ESP_LOGV(TAG, "Last command before error: %s source=%s age=%llums",
                         dispatcherStats.lastCommandBeforeError, source, commandAge);
            } else if (dispatcherStats.lastCommandBeforeError[0] == '\0') {
                ESP_LOGV(TAG, "Last command before error: no recent ARCI->radio command; attribution unavailable");
            } else {
                ESP_LOGV(TAG, "Last command before error: %s source=%s age=unknown",
                         dispatcherStats.lastCommandBeforeError, source);
            }
        } else {
            ESP_LOGD(TAG, "Radio errors: none");
        }
#endif
    }
}

void Diagnostics::taskWrapper(void* arg) {
    static_cast<Diagnostics*>(arg)->task();
}

int Diagnostics::scanI2CBus(void* busHandle) {
    auto i2cBus = static_cast<i2c_master_bus_handle_t>(busHandle);

    ESP_LOGD(TAG, "=== I2C Bus Scan Start ===");
    int devicesFound = 0;

    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        i2c_device_config_t probe_cfg = {};
        probe_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        probe_cfg.device_address = addr;
        probe_cfg.scl_speed_hz = 100000;

        i2c_master_dev_handle_t probe_handle;
        esp_err_t ret = i2c_master_bus_add_device(i2cBus, &probe_cfg, &probe_handle);
        if (ret == ESP_OK) {
            ret = i2c_master_probe(i2cBus, addr, 100);
            if (ret == ESP_OK) {
                ESP_LOGD(TAG, "  Found I2C device at 0x%02X", addr);
                devicesFound++;
            }
            i2c_master_bus_rm_device(probe_handle);
        }
    }

    if (devicesFound == 0) {
        ESP_LOGW(TAG, "No I2C devices found - check pull-ups/power/wiring");
    } else {
        ESP_LOGD(TAG, "=== I2C Bus Scan Complete: %d device(s) found ===", devicesFound);
    }

    return devicesFound;
}

void Diagnostics::scanTCA9548Channels(void* busHandle, TCA9548Handler& tca9548Handler) {
    auto i2cBus = static_cast<i2c_master_bus_handle_t>(busHandle);

    ESP_LOGD(TAG, "=== Scanning TCA9548 Channels ===");

    for (uint8_t channel = 0; channel < 8; channel++) {
        esp_err_t ret = tca9548Handler.selectChannel(channel);
        if (ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));

            for (uint8_t addr = 0x03; addr < 0x78; addr++) {
                i2c_device_config_t probe_cfg = {};
                probe_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
                probe_cfg.device_address = addr;
                probe_cfg.scl_speed_hz = 100000;

                i2c_master_dev_handle_t probe_handle;
                ret = i2c_master_bus_add_device(i2cBus, &probe_cfg, &probe_handle);
                if (ret == ESP_OK) {
                    ret = i2c_master_probe(i2cBus, addr, 100);
                    if (ret == ESP_OK) {
                        ESP_LOGD(TAG, "  CH%d: device at 0x%02X", channel, addr);
                    }
                    i2c_master_bus_rm_device(probe_handle);
                }
            }
        }
    }

    tca9548Handler.selectChannel(0xFF);
    ESP_LOGD(TAG, "=== Channel Scan Complete ===");
}

void Diagnostics::runButtonTestMode(
    gpio_num_t i2cSda,
    gpio_num_t i2cScl,
    TCA9548Handler& tca9548Handler,
    TCA8418Handler& tca8418Handler1,
    TCA8418Handler& tca8418Handler2,
    uint8_t tca9548Channel1,
    uint8_t tca9548Channel2) {

    ESP_LOGI(TAG, "=================================================================");
    ESP_LOGI(TAG, "           BUTTON TEST MODE - KEYCODE DIAGNOSTIC                 ");
    ESP_LOGI(TAG, "=================================================================");
    ESP_LOGI(TAG, "This mode will log ONLY button key codes without any other");
    ESP_LOGI(TAG, "functionality. Press each button and note the key code displayed.");
    ESP_LOGI(TAG, "=================================================================");

    // Initialize NVS (required for some components)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C bus
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.sda_io_num = i2cSda;
    i2c_mst_config.scl_io_num = i2cScl;
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.intr_priority = 0;
    i2c_mst_config.trans_queue_depth = 0;
    i2c_mst_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized");

    // Initialize TCA9548 multiplexer
    ESP_ERROR_CHECK(tca9548Handler.initialize(i2c_bus_handle));
    ESP_LOGI(TAG, "TCA9548 multiplexer initialized");

    // Initialize TCA8418 #1 (main button matrix)
    ESP_LOGI(TAG, "Initializing TCA8418 #1 on channel %d", tca9548Channel1);
    tca8418Handler1.setMuxChannel(&tca9548Handler, tca9548Channel1);

    if (!tca8418Handler1.initialize(i2c_bus_handle)) {
        ESP_LOGE(TAG, "Failed to initialize TCA8418 #1 - TEST ABORTED");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    ESP_LOGI(TAG, "TCA8418 #1 initialized successfully");

    // Initialize TCA8418 #2 (F1-F6 macros)
    ESP_LOGI(TAG, "Initializing TCA8418 #2 on channel %d", tca9548Channel2);
    tca8418Handler2.setMuxChannel(&tca9548Handler, tca9548Channel2);

    if (!tca8418Handler2.initialize(i2c_bus_handle)) {
        ESP_LOGE(TAG, "Failed to initialize TCA8418 #2 - continuing with TCA8418 #1 only");
    } else {
        ESP_LOGI(TAG, "TCA8418 #2 initialized successfully");
    }

    // Set up simple callbacks that only log key codes
    ESP_LOGI(TAG, "=================================================================");
    ESP_LOGI(TAG, "Setting up test callbacks for ALL possible key codes (0x01-0x50)");
    ESP_LOGI(TAG, "=================================================================");

    // Lambda to log key events for TCA8418 #1
    auto logKey1 = [](TCA8418Handler::MatrixKey key, bool pressed) {
        if (pressed) {
            ESP_LOGI("KEY_TEST_1", "🔘 TCA8418 #1: Key 0x%02X (%d) PRESSED",
                     static_cast<uint8_t>(key), static_cast<uint8_t>(key));
        } else {
            ESP_LOGD("KEY_TEST_1", "   TCA8418 #1: Key 0x%02X (%d) released",
                     static_cast<uint8_t>(key), static_cast<uint8_t>(key));
        }
    };

    // Lambda to log key events for TCA8418 #2
    auto logKey2 = [](TCA8418Handler::MatrixKey key, bool pressed) {
        if (pressed) {
            ESP_LOGI("KEY_TEST_2", "🔘 TCA8418 #2: Key 0x%02X (%d) PRESSED",
                     static_cast<uint8_t>(key), static_cast<uint8_t>(key));
        } else {
            ESP_LOGD("KEY_TEST_2", "   TCA8418 #2: Key 0x%02X (%d) released",
                     static_cast<uint8_t>(key), static_cast<uint8_t>(key));
        }
    };

    // Register callbacks for all possible key codes (0x01 to 0x50 = 80 keys max)
    for (uint8_t keyCode = 0x01; keyCode <= 0x50; keyCode++) {
        tca8418Handler1.setKeyCallback(static_cast<TCA8418Handler::MatrixKey>(keyCode), logKey1);
        tca8418Handler2.setKeyCallback(static_cast<TCA8418Handler::MatrixKey>(keyCode), logKey2);
    }

    ESP_LOGI(TAG, "=================================================================");
    ESP_LOGI(TAG, "TEST MODE READY - Press buttons to see their key codes");
    ESP_LOGI(TAG, "Format: Key 0x[HEX] ([DECIMAL]) PRESSED");
    ESP_LOGI(TAG, "=================================================================");

    // Infinite loop - just let the interrupt-driven key handlers do their work
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
