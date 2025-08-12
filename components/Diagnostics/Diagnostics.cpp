#include "Diagnostics.h"
#include "TCA9548Handler.h"
#include "TCA8418Handler.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <cstring>

Diagnostics::Diagnostics(radio::RadioManager& radioManager)
    : m_radioManager(radioManager), m_taskHandle(nullptr) {}

void Diagnostics::setTaskHandles(TaskHandle_t usbTask, TaskHandle_t usb2Task,
                                TaskHandle_t radioTask, TaskHandle_t displayTask,
                                TaskHandle_t mainTask) {
    m_monitoredTasks.clear();
    m_monitoredTasks.reserve(5);
    
    if (usbTask) {
        m_monitoredTasks.push_back({usbTask, "usb_task", 4096});
    }
    if (usb2Task) {
        m_monitoredTasks.push_back({usb2Task, "usb2_task", 4096});
    }
    if (radioTask) {
        m_monitoredTasks.push_back({radioTask, "radio_task", 4096});
    }
    if (displayTask) {
        m_monitoredTasks.push_back({displayTask, "display_task", 4096});
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
    
    ESP_LOGI(TAG, "[Periodic monitor] Internal SRAM: %zu/%zu bytes free (largest: %zu), %.1f%% used",
             internalFree, internalTotal, internalLargest, internalUsedPercent);
    
    // SPIRAM monitoring (if available)
    const size_t spiramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (spiramFree > 0) {
        const size_t spiramLargest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        const size_t spiramTotal = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        const size_t spiramUsed = spiramTotal - spiramFree;
        const float spiramUsedPercent = spiramTotal > 0 ? (float(spiramUsed) / float(spiramTotal)) * 100.0f : 0.0f;
        
        ESP_LOGI(TAG, "[Periodic monitor] SPIRAM: %zu/%zu bytes free (largest: %zu), %.1f%% used",
                 spiramFree, spiramTotal, spiramLargest, spiramUsedPercent);
    }
}

void Diagnostics::printStackStatus() const {
    if (m_monitoredTasks.empty()) {
        ESP_LOGW(TAG, "No tasks configured for stack monitoring");
        return;
    }
    
    ESP_LOGI(TAG, "--- Task Stack Status ---");
    for (const auto& taskInfo : m_monitoredTasks) {
        if (taskInfo.handle) {
            // Get stack high water mark (in words, so multiply by 4 for bytes on 32-bit system)
            const UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(taskInfo.handle);
            const UBaseType_t stackRemainingBytes = stackRemaining * sizeof(StackType_t);
            const UBaseType_t stackUsedBytes = taskInfo.stackSize - stackRemainingBytes;
            const float stackUsagePercent = (float(stackUsedBytes) / float(taskInfo.stackSize)) * 100.0f;
            
            ESP_LOGI(TAG, "Task [%s]: %lu/%lu bytes used (%.1f%%), %lu bytes free",
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
    constexpr size_t bufferSize = 2048;
    char* runTimeStatsBuffer = static_cast<char*>(heap_caps_malloc(bufferSize, MALLOC_CAP_8BIT));
    if (!runTimeStatsBuffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer for CPU monitoring");
        return;
    }

    // Get runtime statistics
    vTaskGetRunTimeStats(runTimeStatsBuffer);

    // Parse the runtime stats to extract per-core information
    uint32_t idle0Runtime = 0, idle1Runtime = 0, totalRuntime = 0;

    // Make a copy for parsing since strtok modifies the buffer
    char* parseBuffer = static_cast<char*>(heap_caps_malloc(bufferSize, MALLOC_CAP_8BIT));
    if (!parseBuffer) {
        heap_caps_free(runTimeStatsBuffer);
        return;
    }
    strcpy(parseBuffer, runTimeStatsBuffer);

    // Parse the buffer to find IDLE0 and IDLE1 runtimes
    const char* line = strtok(parseBuffer, "\n\r");
    while (line != nullptr) {
        char taskName[configMAX_TASK_NAME_LEN];
        uint32_t runtime, percent;

        if (sscanf(line, "%s %lu %lu", taskName, &runtime, &percent) == 3 ||
            sscanf(line, "%s %lu %lu%%", taskName, &runtime, &percent) == 3) {
            if (strcmp(taskName, "IDLE0") == 0) {
                idle0Runtime = runtime;
            } else if (strcmp(taskName, "IDLE1") == 0) {
                idle1Runtime = runtime;
            }
            totalRuntime += runtime;
        }
        line = strtok(nullptr, "\n\r");
    }

    heap_caps_free(parseBuffer);

    // Calculate deltas (handles wraparound automatically)
    const uint32_t deltaIdle0 = idle0Runtime - m_lastIdle0Runtime;
    const uint32_t deltaIdle1 = idle1Runtime - m_lastIdle1Runtime;
    const uint32_t deltaTotal = totalRuntime - m_lastTotalTime;

    // Calculate per-core usage percentages from deltas
    if (deltaTotal > 0 && m_lastTotalTime > 0) {
        const float idle0Percent = static_cast<float>(deltaIdle0) / deltaTotal * 100.0f;
        const float idle1Percent = static_cast<float>(deltaIdle1) / deltaTotal * 100.0f;

        const float core0Usage = 100.0f - idle0Percent;
        const float core1Usage = 100.0f - idle1Percent;

        ESP_LOGI(TAG, "========== CPU UTILIZATION ==========");
        ESP_LOGI(TAG, "  Core 0: %4.1f%% busy (%4.1f%% idle)", core0Usage, idle0Percent);
        ESP_LOGI(TAG, "  Core 1: %4.1f%% busy (%4.1f%% idle)", core1Usage, idle1Percent);
        ESP_LOGI(TAG, "  Total:  %4.1f%% system load", (core0Usage + core1Usage) / 2.0f);

        // Print top active tasks
        ESP_LOGI(TAG, "Top Active Tasks:");
        ESP_LOGI(TAG, "Task Name        | Core | CPU%% | Priority | Stack Free");
        ESP_LOGI(TAG, "-----------------|------|------|----------|----------");

        const UBaseType_t taskCount = uxTaskGetNumberOfTasks();
        auto taskStatusArray = static_cast<TaskStatus_t*>(
            heap_caps_malloc(taskCount * sizeof(TaskStatus_t), MALLOC_CAP_8BIT));

        if (taskStatusArray) {
            const UBaseType_t actualTaskCount = uxTaskGetSystemState(taskStatusArray, taskCount, nullptr);

            // Re-get runtime stats for task parsing
            vTaskGetRunTimeStats(runTimeStatsBuffer);
            parseBuffer = static_cast<char*>(heap_caps_malloc(bufferSize, MALLOC_CAP_8BIT));
            if (parseBuffer) {
                strcpy(parseBuffer, runTimeStatsBuffer);
                line = strtok(parseBuffer, "\n\r");

                while (line != nullptr) {
                    char taskName[configMAX_TASK_NAME_LEN];
                    uint32_t runtime, percent;

                    int parsedFields = sscanf(line, "%s %lu %lu", taskName, &runtime, &percent);
                    if (parsedFields == 2) {
                        percent = totalRuntime > 0 ? (runtime * 100) / totalRuntime : 0;
                        parsedFields = 3;
                    } else if (parsedFields != 3) {
                        parsedFields = sscanf(line, "%s %lu %lu%%", taskName, &runtime, &percent);
                    }

                    if (parsedFields >= 2) {
                        const float percentFloat = totalRuntime > 0
                            ? (static_cast<float>(runtime) * 100.0f) / static_cast<float>(totalRuntime)
                            : 0.0f;

                        // Skip IDLE tasks and show tasks with measurable activity
                        if (strncmp(taskName, "IDLE", 4) != 0 && (percentFloat > 0.0f || runtime > 1000)) {
                            const char* coreStr = "?";
                            uint32_t stackFree = 0;
                            UBaseType_t priority = 0;

                            // Find task in system state
                            for (UBaseType_t i = 0; i < actualTaskCount; i++) {
                                if (strcmp(taskStatusArray[i].pcTaskName, taskName) == 0) {
                                    const TaskHandle_t taskHandle = taskStatusArray[i].xHandle;
                                    priority = taskStatusArray[i].uxCurrentPriority;
                                    stackFree = taskStatusArray[i].usStackHighWaterMark * sizeof(StackType_t);

                                    const BaseType_t coreAffinity = xTaskGetCoreID(taskHandle);
                                    if (coreAffinity == 0) coreStr = "0";
                                    else if (coreAffinity == 1) coreStr = "1";
                                    else coreStr = "B";
                                    break;
                                }
                            }

                            ESP_LOGI(TAG, "%-16s | %-4s | %3.1f%% | %8lu | %8lu",
                                     taskName, coreStr, percentFloat, priority, stackFree);
                        }
                    }
                    line = strtok(nullptr, "\n\r");
                }
                heap_caps_free(parseBuffer);
            }
            heap_caps_free(taskStatusArray);
        }
    }

    // Update baseline for next iteration
    m_lastIdle0Runtime = idle0Runtime;
    m_lastIdle1Runtime = idle1Runtime;
    m_lastTotalTime = totalRuntime;

    ESP_LOGI(TAG, "Memory: Free=%luKB, Min=%luKB",
             esp_get_free_heap_size() / 1024, esp_get_minimum_free_heap_size() / 1024);
    ESP_LOGI(TAG, "==========================================");

    heap_caps_free(runTimeStatsBuffer);
}

void Diagnostics::start() {
    xTaskCreate(taskWrapper, "DiagnosticsTask", 4096, this, 3, &m_taskHandle); // Low priority background task
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

        auto [totalCommandsProcessed, localCommandsProcessed, remoteCommandsProcessed, readCommandsProcessed,
            setCommandsProcessed, feedbackLoopsPrevented] = m_radioManager.getStatistics();
        auto dispatcherStats = m_radioManager.getCommandDispatcher().getStatistics();

        ESP_LOGI(TAG, "--- RadioManager Stats ---");
        ESP_LOGI(TAG, "Total Commands Processed: %zu", totalCommandsProcessed);
        ESP_LOGI(TAG, "Local Commands: %zu", localCommandsProcessed);
        ESP_LOGI(TAG, "Remote Commands: %zu", remoteCommandsProcessed);
        ESP_LOGI(TAG, "Read Commands: %zu", readCommandsProcessed);
        ESP_LOGI(TAG, "Set Commands: %zu", setCommandsProcessed);
        ESP_LOGI(TAG, "Feedback Loops Prevented: %zu", feedbackLoopsPrevented);

        ESP_LOGI(TAG, "--- CommandDispatcher Stats ---");
        ESP_LOGI(TAG, "Commands Dispatched: %zu", dispatcherStats.totalCommandsDispatched);
        ESP_LOGI(TAG, "Commands Handled: %zu", dispatcherStats.commandsHandled);
        ESP_LOGI(TAG, "Commands Unhandled: %zu", dispatcherStats.commandsUnhandled);
        ESP_LOGI(TAG, "Handler Errors: %zu", dispatcherStats.handlerErrors);

        // Error response diagnostics
        if (dispatcherStats.totalErrorResponses > 0) {
            ESP_LOGI(TAG, "--- ERROR RESPONSE ANALYSIS ---");
            ESP_LOGI(TAG, "Total Error Responses: %zu", dispatcherStats.totalErrorResponses);
            ESP_LOGI(TAG, "Question Mark Errors (?;): %zu", dispatcherStats.questionMarkErrors);
            ESP_LOGI(TAG, "E Errors (E;): %zu", dispatcherStats.eErrors);
            ESP_LOGI(TAG, "O Errors (O;): %zu", dispatcherStats.oErrors);
            ESP_LOGI(TAG, "Error Bursts Detected: %zu", dispatcherStats.errorBursts);
            ESP_LOGI(TAG, "Average Error Interval: %llu ms", dispatcherStats.averageErrorInterval / 1000);

            const uint64_t currentTime = esp_timer_get_time();
            if (dispatcherStats.lastCommandBeforeErrorTime > 0) {
                const uint64_t commandAge = (currentTime - dispatcherStats.lastCommandBeforeErrorTime) / 1000;
                ESP_LOGI(TAG, "Last Command Before Error: %s (sent %llu ms ago)",
                         dispatcherStats.lastCommandBeforeError.c_str(), commandAge);
            } else {
                ESP_LOGI(TAG, "Last Command Before Error: %s (timestamp unknown)",
                         dispatcherStats.lastCommandBeforeError.c_str());
            }

            if (const uint64_t runtimeMs = esp_timer_get_time() / 1000; runtimeMs > 60000) {
                const float errorRate = static_cast<float>(dispatcherStats.totalErrorResponses) / (runtimeMs / 60000.0f);
                ESP_LOGI(TAG, "Error Rate: %.2f errors/minute", errorRate);
            }

            if (dispatcherStats.averageErrorInterval > 0 && dispatcherStats.averageErrorInterval < 5000000) {
                ESP_LOGW(TAG, "⚠️  HIGH ERROR RATE: Errors occurring every %llu ms (< 5s)",
                         dispatcherStats.averageErrorInterval / 1000);
            }
        } else {
            ESP_LOGI(TAG, "✅ No error responses detected");
        }

        ESP_LOGI(TAG, "--------------------------------");
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
        i2c_device_config_t probe_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = false,
            },
        };

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
                i2c_device_config_t probe_cfg = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = addr,
                    .scl_speed_hz = 100000,
                    .scl_wait_us = 0,
                    .flags = {
                        .disable_ack_check = false,
                    },
                };

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
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = i2cSda,
        .scl_io_num = i2cScl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

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
