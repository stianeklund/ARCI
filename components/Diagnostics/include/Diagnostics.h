#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "RadioManager.h"
#include <vector>

struct TaskStackInfo {
    TaskHandle_t handle;
    const char* name;
    UBaseType_t stackSize; // in bytes
};

// Forward declarations for button test mode
class TCA9548Handler;
class TCA8418Handler;

class Diagnostics {
public:
    explicit Diagnostics(radio::RadioManager& radioManager);
    void start();

    // Set task handles for stack monitoring
    void setTaskHandles(TaskHandle_t usbTask, TaskHandle_t usb2Task,
                       TaskHandle_t radioTask, TaskHandle_t displayTask,
                       TaskHandle_t mainTask);

    /**
     * @brief Run button test mode - logs all button key codes for debugging
     *
     * This mode only logs button key codes without any other functionality.
     * It initializes I2C, TCA9548, and TCA8418 handlers, then registers
     * callbacks for all possible key codes.
     *
     * @param i2cSda I2C SDA GPIO pin
     * @param i2cScl I2C SCL GPIO pin
     * @param tca9548Handler TCA9548 I2C multiplexer handler
     * @param tca8418Handler1 First TCA8418 keypad controller handler
     * @param tca8418Handler2 Second TCA8418 keypad controller handler
     * @param tca9548Channel1 TCA9548 channel for first TCA8418
     * @param tca9548Channel2 TCA9548 channel for second TCA8418
     */
    [[noreturn]] static void runButtonTestMode(
        gpio_num_t i2cSda,
        gpio_num_t i2cScl,
        TCA9548Handler& tca9548Handler,
        TCA8418Handler& tca8418Handler1,
        TCA8418Handler& tca8418Handler2,
        uint8_t tca9548Channel1,
        uint8_t tca9548Channel2);

    /**
     * @brief Scan I2C bus for devices
     *
     * Scans the I2C bus and logs all found devices with their addresses.
     * Identifies known devices (TCA9548A, TCA8418, PCF8575).
     *
     * @param busHandle I2C master bus handle
     * @return Number of devices found
     */
    static int scanI2CBus(void* busHandle);

    /**
     * @brief Scan all channels of a TCA9548 multiplexer for devices
     *
     * @param busHandle I2C master bus handle
     * @param tca9548Handler TCA9548 multiplexer handler
     */
    static void scanTCA9548Channels(void* busHandle, TCA9548Handler& tca9548Handler);

private:
    void task();
    void printMemoryStatus() const;
    void printStackStatus() const;
    void printCpuStatus();
    static void taskWrapper(void* arg);

    radio::RadioManager& m_radioManager;
    TaskHandle_t m_taskHandle;

    // Task monitoring
    std::vector<TaskStackInfo> m_monitoredTasks;

    // CPU monitoring state (for delta calculations)
    uint32_t m_lastIdle0Runtime{0};
    uint32_t m_lastIdle1Runtime{0};
    uint32_t m_lastTotalTime{0};

    static constexpr const char* TAG = "DIAGNOSTICS";
    static constexpr int PRINT_INTERVAL_MS = 60000; // 60 seconds
};
