#include "include/AudioHandler.h"
#include "usb/usb_host.h"
// main.c
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


static const char *TAG = "AUDIOHANDLER";

// Audio parameters
#define SAMPLE_RATE     48000        // Audio sample rate (Hz)
#define AMPLITUDE       3000.0f       // Sine amplitude (16-bit samples)
static double phase = 0.0; // Current phase of sine wave


/**
 * @brief Fill a buffer with audio data.
 *
 * This function is intended to be called from the USB Audio TX callback.
 * It generates a sine wave tone if a note is active; otherwise, it outputs silence.
 *
 * @param buffer The output audio buffer.
 * @param length The length of the buffer in bytes.
 *
 * Note: The buffer is assumed to hold interleaved stereo 16-bit samples.
 */
void usb_audio_output_callback(uint8_t *buffer, size_t length) {
    // For stereo 16-bit: each sample is 4 bytes (2 bytes per channel)
    size_t sample_count = length / 4;

    // TODO
    /*
    // Write the same sample to both left and right channels.
    buffer[4 * i + 0] = sample & 0xFF;
    buffer[4 * i + 1] = (sample >> 8) & 0xFF;
    buffer[4 * i + 2] = sample & 0xFF;
    buffer[4 * i + 3] = (sample >> 8) & 0xFF;
    */
}

/**
 * @brief Stub for USB Audio Input callback.
 *
 * In a full bi‑directional implementation, this function would handle incoming audio data.
 * For this demo, the incoming data is simply ignored.
 */
void usb_audio_input_callback(const uint8_t *buffer, const size_t length) {
    // Discard or process input audio data as needed.
    (void) buffer;
    (void) length;
}

/**
 * @brief Task to run the TinyUSB device stack.
 */
void usb_device_task(void *param) {
    while (1) {
        //tud_task(); // TinyUSB device task processing
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Task to simulate periodic USB Audio streaming.
 *
 * In a production implementation the TinyUSB Audio driver would invoke the
 * appropriate callbacks when the host requests data.
 */
void usb_audio_task(void *param) {
    // Example audio buffer (adjust size as needed for your endpoint configuration)
    uint8_t audio_buffer[256];
    while (1) {
        // Fill the buffer with audio samples (sine tone or silence)
        usb_audio_output_callback(audio_buffer, sizeof(audio_buffer));

        // Here you would normally hand the buffer over to the USB Audio TX endpoint.
        // For this demo, we simply wait to simulate periodic audio streaming.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Note: To enable USB Audio composite support, you must properly configure
 * TinyUSB (and if needed, set up a composite device) via menuconfig or additional descriptor files.
 */
void test_run(void) {
    ESP_LOGI(TAG, "Starting USB Audio");

    // Initialize TinyUSB device stack.
    //tusb_init();

    // Create tasks for USB device processing and audio streaming.
    xTaskCreate(usb_device_task, "usb_device_task", 4096, nullptr, 5, nullptr);
    xTaskCreate(usb_audio_task, "usb_audio_task", 4096, nullptr, 5, nullptr);

    // Main loop – tasks handle USB and audio processing.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}