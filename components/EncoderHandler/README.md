## EncoderHandler Class Documentation

This class provides functionality for reading and interpreting signals from a rotary encoder. It's designed to work with
ESP32 microcontrollers and integrates with the RadioManager for frequency tuning control.

I spent a LOT of time trying to find a solid "solution" for this considering we're working with latency.
There are some specific "workarounds" here where we modify the frequency and pass it directly to consumers like the display.
This allows the display to get a fast update while the radio is still processing the frequency change, but it also can
cause oddities where the display doesn't always match the radio's frequency.

**Purpose:** To read the position of a rotary encoder, process movement with acceleration and debouncing, and send 
frequency commands to the radio through the CAT interface.

### Class Overview

The `EncoderHandler` class manages the hardware interaction with a rotary encoder (two GPIO pins) and processes its
signals to determine direction and magnitude. It then sends frequency commands to the radio through the RadioManager
based on the encoder's movement. It includes sophisticated acceleration, debouncing, and sensitivity controls.

My goal is to try to make this class even more simple and flexible than it is currently, but the current implementation
works well or my use case.

### Configuration Options (Kconfig)

All options live under `idf.py menuconfig → Encoder Handler Options`.

#### Acceleration Model
- 3‑rate ladder with hysteresis and instant precision on reversals:
  - 1 Hz (slow) → 10 Hz (moderate) → 100 Hz (fast)
  - Gentle 1 kHz snapping only when at 100 Hz

#### Core Controls
- `ENCODER_MIN_STEPS_THRESHOLD` (1–10, default 1)
  - Minimum raw encoder edges required before a live update is considered. Raise for very noisy hardware.
- `ENCODER_LIVE_UPDATE_INTERVAL_MS` (10–200, default 20)
  - Static live update gate used by the simplified model (and as a fallback). Lower = more responsive, higher = less CAT traffic.
- `ENCODER_FINAL_UPDATE_DELAY_MS` (20–500, default 40)
  - Debounce before sending the final “stopped” update after motion ends.
- `TUNING_EMA_ALPHA` (x100, 5–50, default 25)
  - Velocity EMA smoothing for both models. Higher values smooth more but react slower. 25–35 is a good range.
- `TUNING_DIRECTION_RETENTION` (%, 20–90, default 60)
  - How much velocity is retained across a direction change. Lower favors instant precision; higher feels smoother.

#### Tuning Acceleration
- `VFO_TUNING_ACCELERATION` (bool, default on): enables 1/10/100 Hz selection by velocity.
- `TUNING_SLOW_THRESHOLD` (edges/sec): 1→10 Hz threshold (suggested ~180 eps for optical encoders).
- `TUNING_FAST_THRESHOLD` (edges/sec): 10→100 Hz threshold (suggested ~1200 eps).
- `TUNING_KHZ_SNAP_WINDOW` (Hz): 1 kHz snap window used only at 100 Hz step (100–120 Hz recommended).

#### Mode Interaction
- FS=1 (Fine Tuning) caps max step at 10 Hz.
- Mode floors (internal behavior): FM ≥ 1 kHz, AM ≥ 100 Hz, CW/SSB ≥ 1 Hz.

#### Recommended Defaults (RMS20‑250‑1)
- EMA alpha 30–35; Slow threshold ~180 eps; Fast threshold ~1200 eps.
- Live update 12–20 ms; Final delay 40–80 ms; 1 kHz snap window 100–120 Hz.

#### Tuning Tips
- Too eager 1→10 Hz: increase `TUNING_SLOW_THRESHOLD` slightly (e.g., +20–40).
- 100 Hz engages too soon: increase `TUNING_FAST_THRESHOLD` (e.g., +200–300).
- Fine feels laggy: increase `TUNING_EMA_ALPHA` to 35; decrease to 25 if too twitchy.
- Boundary pogo: reduce `TUNING_KHZ_SNAP_WINDOW` to ~100.

### Constructor:
`EncoderHandler(gpio_num_t pinA, gpio_num_t pinB, radio::RadioManager* radioManager)`

* **`pinA`**: The GPIO pin connected to the 'A' signal of the rotary encoder.
* **`pinB`**: The GPIO pin connected to the 'B' signal of the rotary encoder.
* **`radioManager`**: A pointer to the RadioManager instance for sending frequency commands.

The constructor initializes member variables and prepares the encoder for use. It's crucial that you pass a valid 
`RadioManager` instance; otherwise, the class won't be able to send frequency updates.

### Public Methods:

* **`setup()`**:
    * Configures the specified GPIO pins (`pinA`, `pinB`) as inputs with pull-up resistors.
    * Installs interrupt service routines (ISRs) for both pins, triggered on any edge change.
    * Initializes the encoder state machine for quadrature decoding.

* **`start()`**:
    * Creates and starts the encoder processing task with high priority (10).
    * The task runs continuously to process encoder movements and send frequency updates.

* **`stop()`**:
    * Stops the encoder processing task gracefully.
    * Cleans up resources and resets the task handle.

* **`task()`** (called internally by the FreeRTOS task):
    * Core processing function that handles encoder position changes.
    * Implements the configurable step threshold, timing intervals, and acceleration.
    * Calculates frequency changes based on encoder movement and current VFO.
    * Sends `FA` or `FB` commands to the radio through RadioManager when thresholds are met.
    * Manages tuning state flags to coordinate with other components (display suppression, etc.).

### Key Features:

* **Event-driven:** Uses GPIO interrupts + semaphore for immediate response to encoder changes
* **Zero polling overhead:** Task sleeps until encoder movement occurs (no CPU waste)
* **Quadrature decoding:** Proper A/B signal decoding for direction and magnitude
* **Configurable sensitivity:** Adjustable step threshold prevents accidental changes
* **Smart acceleration:** Speed-aware step sizing for efficient frequency tuning  
* **VFO awareness:** Automatically targets the correct VFO (A/B) based on radio state
* **Transverter support:** Handles frequency offsets for transverter operation
* **Task-based:** Runs in dedicated FreeRTOS task for non-blocking operation

### Usage Example:

```c++
// Assuming you have RadioManager initialized...
radio::RadioManager radioManager;

EncoderHandler encoder(GPIO_NUM_2, GPIO_NUM_4, &radioManager);

void setup() {
    encoder.setup();  // Configure GPIO pins and interrupts
    encoder.start();  // Start the encoder processing task
}

// The encoder runs autonomously in its own FreeRTOS task
// No manual polling required - frequency updates are sent automatically
```

### Important Notes:

* **RadioManager Dependency:** The `EncoderHandler` *requires* a valid `RadioManager` instance to function correctly.
  Make sure you initialize it before creating an `EncoderHandler`.
* **Pin Connections:** Ensure that the encoder's 'A' and 'B' signals are connected to the specified GPIO pins (`pinA`,
  `pinB`). The class automatically enables pull-up resistors on these pins.
* **Configuration:** Use `idf.py menuconfig` to adjust encoder sensitivity, timing intervals, and polling rates to 
  match your hardware and preferences.
* **VFO Integration:** The encoder automatically detects the active VFO (A or B) and sends the appropriate `FA` or `FB`
  commands. It tunes the RX VFO when receiving and the TX VFO when transmitting, ensuring the encoder always controls the VFO that's actually in use.
* **Acceleration:** Built-in TuningAccelerator provides smart step sizing based on turning speed and frequency bands.
* **Coordination:** The encoder sets tuning state flags that other components (like display handlers) use to suppress
  conflicting updates during active tuning.

This documentation provides guidance for using the `EncoderHandler` class in your ESP32 radio control projects. 
Configure the Kconfig options to match your specific encoder hardware and tuning preferences.
