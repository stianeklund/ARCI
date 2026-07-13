# ARCI Codebase Audit — 2026-07-13

## Scope and method

This is a read-only source audit of the ESP32-S3 firmware's CAT command, state,
UART, USB CDC, TCP bridge, task, and diagnostic paths. It uses the current
working tree, which already contained unrelated modifications when the audit
started. No source changes were made as part of the audit.

The previously fixed FA/FB cache-freshness defect is treated as the baseline;
it is deliberately not reported below as a new finding. The issues below are
independent transport, concurrency, correctness, performance, architecture,
and security risks.

Runtime/hardware reproduction and an ESP-IDF build were not performed in this
environment. The highest-priority issues are proven directly from control flow
and locking semantics in the source.

## Executive summary

The architecture has a strong command/handler split, but it lacks clear
ownership at its most important concurrency boundaries:

- The TCP bridge can deadlock on an ordinary client disconnect.
- UART output is not owned by one serializer, allowing concurrent writers to
  mix CAT frames.
- UART receive hands out a view into a queue slot after releasing that slot for
  reuse.
- A public mutable `RadioState` contains non-atomic fields that are accessed by
  button, encoder, and dispatch tasks concurrently.
- A single recursive dispatcher lock serializes every ingress path; some
  handlers sleep while holding it and timeout callers silently discard work.

The recommended first milestone is to establish explicit ownership of all
transport TX and RX buffers, then remove cross-task mutation of the state bag.

## Findings

| Priority | Finding | Consequence |
| --- | --- | --- |
| Critical | TCP send-error self-deadlock | A fatal TCP send error can freeze the bridge and potentially the global CAT dispatcher. |
| High | UART TX bypasses its intended lock | Concurrent maintenance and CAT sends can concatenate/interleave frames. |
| High | UART RX returns an unleased queue view | Backlog can overwrite a frame while CAT processing still reads it. |
| High | Shared `RadioState` contains data races | Input handlers can use stale or undefined state to choose radio actions. |
| High | Global dispatch lock causes head-of-line blocking | A slow command can starve radio, display, USB, TCP, button, and encoder traffic; timeout drops are silent. |
| Medium | CDC overflow corrupts its accumulator | Long/bursty input can inject stale bytes and lose valid bytes. |
| Medium | UART diagnostic snapshot races | Error logging reads a `std::string` concurrently with writers. |
| Medium | Cache/query timestamp pairs are incoherent | Concurrent access can produce a tag from one command and a timestamp from another. |
| Medium | Observability is unreliable | Some stats are raced and the printed `RadioManager` counters are never incremented. |
| Medium | TCP CAT is unauthenticated | Any host that can reach the listening port can control CAT operations. |
| Low | Task startup and tests are failure-blind | Several task creations are unchecked; the serial queue suite is stale and disabled. |

### 1. Critical — TCP send error deadlocks on `clientsMutex_`

`TcpCatBridge::sendToActiveClient()` takes `clientsMutex_`. On a fatal `send()`
failure it calls `closeClient()`, which attempts to take the same mutex again.
`RtosMutex` is explicitly non-recursive, so this task blocks permanently.

This is particularly damaging when the bridge is sending a cached response from
within `RadioManager::dispatchMessage()`: the dispatch lock remains held and
all ingress paths subsequently wait or time out.

Relevant code:

- `components/TcpCatBridge/TcpCatBridge.cpp:326` — takes the lock and invokes
  `closeClient(i)` on the error path.
- `components/TcpCatBridge/TcpCatBridge.cpp:388` — `closeClient()` takes that
  same lock.
- `components/CommonConstants/include/rtos_mutex.h:14` — `RtosMutex` is
  non-recursive.

Recommendation: call `closeClientLocked(i)` while the lock is held. Add a
disconnect/error-path test that asserts a following send and inbound command
still complete.

### 2. High — UART frame integrity has no single owner

`RadioManager::sendRadioCommand()` takes `radioTxMutex_`, but many callers
write the same `radioSerial_` directly. Examples include dispatcher
auto-queries, the TX timeout/keepalive task, startup/periodic queries, and
individual command handlers. `SerialHandler::sendMessage()` writes a frame body
and, when needed, its semicolon terminator in two separate `uart_write_bytes()`
calls.

Therefore two tasks can produce a stream such as `FA...IF...;`, then a later
semicolon, even if the UART driver serializes individual calls. The radio sees
malformed CAT input and may emit `?;`/`E;`, or apply an unintended command.
The display UART has the same generic `SerialHandler` limitation and also has
multiple senders.

Relevant code:

- `components/RadioCore/RadioManager.cpp:845` — the intended radio TX mutex.
- `components/RadioCore/RadioManager.cpp:1926` and `:1952` — direct timeout
  task writes that bypass it.
- `main/main.cpp:786` — periodic EX refresh bypasses it.
- `components/RadioCore/CommandDispatcher.cpp:446` — auto-query bypasses it.
- `components/SerialHandler/SerialHandler.cpp:238` and `:306` — no internal TX
  lock and separate body/terminator writes.

Recommendation: make the radio and display TX implementations private to an
outbound transport owner. All callers should enqueue complete frames; only that
owner should call `uart_write_bytes()`. Decide deliberately which frames may be
dropped, coalesced, or retried under backpressure.

### 3. High — `getMessageView()` releases a slot before its view is consumed

`SerialHandler::getMessageView()` constructs a `std::string_view` into the ring
buffer, advances the head, decrements the queue count, and releases the
spinlock. The UART event task is then free to refill and eventually wrap around
to the same slot while `radio_task` or `display_task` still parses, logs, or
forwards the view.

The global dispatcher can wait for up to two seconds, which makes this much
more likely precisely during contention. The result is a corrupted, changed,
or unrelated CAT frame reaching the parser.

Relevant code:

- `components/SerialHandler/SerialHandler.cpp:174` — returns the view after
  releasing the queue slot.
- `components/SerialHandler/SerialHandler.cpp:631` — producer reuses queue
  slots as frames arrive.

Recommendation: use `getMessage()` at this boundary, return a fixed-size owned
frame object, or introduce an explicit queue-slot lease whose release occurs
only after dispatch/forwarding is done. Add a producer/consumer saturation test
that forces a full wrap while the consumer holds a frame.

### 4. High — the public state bag has cross-task data races

`RadioState` describes the marked region as shared by button, encoder, macro,
and dispatch tasks and says it “MUST be atomic.” It nevertheless includes many
plain mutable fields in that region, including `mainAntenna`, `attenuator`,
tuner state, `fineTune`, and filter selection. `getState()` exposes mutable
state directly, so the dispatcher mutex cannot protect all accesses.

Concrete examples:

- `EncoderHandler` reads `state.fineTune` while the frequency handler writes it
  on local/remote CAT frames.
- `ButtonHandler` reads `attenuator`, `mainAntenna`, and `txAtIn` before it
  dispatches a toggle command, while remote answers can change those fields.

This is C++ undefined behavior, not just eventually-consistent state. It can
also choose the wrong next state for a physical button press.

Relevant code:

- `components/StateManager/include/RadioState.h:340` — shared region with
  non-atomic fields at `:345-362` and `:393-404`.
- `components/EncoderHandler/EncoderHandler.cpp:573` — cross-task `fineTune`
  read.
- `components/CommandHandlers/frequency/FrequencyVfoCommandHandler.cpp:883` —
  write.
- `components/ButtonHandler/ButtonHandler.cpp:2198` — direct button-side read.

Recommendation: make state private. For independent scalar state use atomics;
for related settings use a small mutex-protected snapshot or a single
state-owner task. Expose intent methods such as `toggleAttenuator()` rather
than exposing fields to input tasks.

### 5. High — a single global dispatch lock creates loss and latency cascades

Every USB, radio, display, panel, macro, and TCP frame enters the same
recursive `dispatchMutex_`. It has a two-second acquisition timeout. At least
one command handler calls `vTaskDelay(50 ms)` twice while owning that lock to
repeat `PS0;`. Callers such as the USB and TCP ingress loops discard a `false`
result, so a lock timeout becomes a silent lost CAT command; the radio ingress
can instead forward a frame without applying its normal state handler.

This is a fundamental head-of-line blocking point: a low-priority or slow path
can delay high-priority encoder activity and consume the UART queue until it
overflows or expires.

Relevant code:

- `components/RadioCore/RadioManager.cpp:777` — global timed lock.
- `components/CommandHandlers/interface/InterfaceSystemCommandHandler.cpp:415`
  — delay under dispatch.
- `main/main.cpp:413` and `:462` — ignored dispatch result for USB ingress.
- `main/main.cpp:507-518` — different radio-ingress behavior after `false`.

Recommendation: use a command actor/ordered work queue for state mutation and
an independent paced TX sequencer. No handler should sleep, perform network
I/O, or wait for a radio response while it owns global command ordering. Return
a defined rejection/error response or enqueue retry rather than silently
discarding a local command.

### 6. Medium — CDC accumulation loses valid bytes and can retain invalid bytes

When the CDC accumulator is full, `getMessageView()` first copies the bytes
that fit, then computes an overflow and shifts the accumulator by only that
overflow. It does not retain the correct suffix of `old + new`: it retains a
region beyond the former logical length and copies only the tail of the new
input. The next parser pass may see stale bytes while the beginning of the new
input, including a delimiter, was lost.

Relevant code:

- `components/UsbCdc/include/CdcSerialHandler.h:31` — 512-byte accumulator and
  256-byte frame buffer.
- `components/UsbCdc/CdcSerialHandler.cpp:171` and `:205` — overflow handling.

Recommendation: either reject/reset a malformed overlong frame through the
next delimiter, or form a correctly bounded suffix of the concatenated data.
Never return a truncated frame as if it were complete. Add tests for a full
accumulator followed by a split frame and for a frame longer than 256 bytes.

### 7. Medium — diagnostic and cache synchronization is incomplete

`SerialHandler::lastSentFrame_` is a `std::string` written from TX tasks and
read from the UART RX event task on error logging, with no lock. That can race
on string storage during an error response.

The timestamp tracker has a different coherence problem: `record()` publishes
the tag and timestamp as separate relaxed operations, and `get()` reads them as
separate relaxed operations. During a collision/replacement, a reader can match
a new tag and use an old timestamp, causing incorrect cache freshness or query
ownership decisions.

Relevant code:

- `components/SerialHandler/SerialHandler.cpp:335` — string write.
- `components/SerialHandler/SerialHandler.cpp:476` and `:522` — concurrent
  reads.
- `components/StateManager/include/RadioState.h:86-126` — separately published
  cache tag and timestamp.

Recommendation: replace the diagnostic string with a fixed-size protected
snapshot. For the cache use a packed atomic record or a generation/version
protocol with release/acquire publication.

### 8. Medium — current metrics cannot reliably diagnose production behavior

`CommandDispatcher::getStatistics()` locks `statsMutex_`, but most counter
writes happen without that mutex. The lock therefore does not synchronize the
copy with writers. Separately, `RadioManager::Statistics` is printed by
`Diagnostics` but its counters have no writes in the repository, so that report
is permanently zero-valued.

Relevant code:

- `components/RadioCore/CommandDispatcher.cpp:219` — unlocked counter write.
- `components/RadioCore/CommandDispatcher.cpp:538` — locked reader.
- `components/RadioCore/include/RadioManager.h:91-102` — inert statistics
  structure/accessors.
- `components/Diagnostics/Diagnostics.cpp:251` — reporting path.

Recommendation: use atomics for independent counters and a mutex only for the
string/timing fields, or protect every read and write consistently. Remove or
wire up the `RadioManager` counters. Preserve the existing queue-age telemetry;
it is valuable for distinguishing stale state from a congested display path.

### 9. Medium — TCP CAT is an unrestricted LAN control surface

The TCP bridge binds to `INADDR_ANY`, accepts a client, and applies received
CAT commands without authentication, authorization, or transport encryption.
Because CAT includes power and transmit controls, any reachable host can operate
the radio.

Relevant code:

- `components/TcpCatBridge/TcpCatBridge.cpp:121-127` — all-interface bind.
- `components/TcpCatBridge/TcpCatBridge.cpp:241` — input is passed to the CAT
  callback without an authentication gate.

Recommendation: keep the feature opt-in, add an allow-list and authentication,
or bind only to a deliberately isolated management network. Treat PTT/power
commands as a higher-risk authorization class.

### 10. Low — lifecycle checks and test coverage hide regressions

The main ingress tasks, diagnostics, button task, encoder task, and ADC task
have unchecked `xTaskCreate()` results. On memory pressure the system can boot
with a missing transport/input task and still report success. Semaphore
creation in `main` is also unchecked.

The `SerialHandler` queue suite is stale: it expects a capacity of 16 while
production uses 64. The test suite is commented out of the aggregate runner,
so it neither catches the stale expectation nor exercises queue overflow and
lifetime behavior.

Relevant code:

- `main/main.cpp:963-966` and `:1034-1038` — unchecked semaphore/task creation.
- `components/EncoderHandler/EncoderHandler.cpp:267`,
  `components/ButtonHandler/ButtonHandler.cpp:65`, and
  `components/ADCHandler/ADCHandler.cpp:42` — unchecked task creation.
- `components/SerialHandler/include/SerialHandler.h:43` — queue capacity 64.
- `components/SerialHandler/test/test_serial_handler_queue.cpp:27-52` — stale
  expectation of 16.
- `components/unit_test/tests_main.cpp:54` and `:79` — suite disabled.

Recommendation: fail startup or degrade explicitly when a required task or
semaphore cannot be created. Re-enable the serial suite after updating its
capacity expectation, then add deterministic stress tests for all P0 defects.

## Recommended remediation order

### P0 — correctness and freeze prevention

1. Fix the TCP `closeClient()` self-deadlock.
2. Establish a single serialized TX owner per UART and route every radio/display
   send through it.
3. Replace UART `string_view` dequeueing with owned/leased frames.
4. Add targeted tests for TCP send failures, interleaved TX attempts, and ring
   wrap while a consumer processes a frame.

### P1 — state ownership and predictable latency

1. Privatize `RadioState`; add atomic or snapshot-based APIs for cross-task
   fields and intent-based operations for buttons/encoders.
2. Replace the global timed dispatch mutex with ordered command processing that
   does not hold the state-ordering boundary across delays or I/O.
3. Move radio pacing, retries, and backpressure policy to the TX owner.
4. Fix CDC overflow and ensure overlong frames are rejected explicitly.

### P2 — operability and security

1. Make all metrics coherent and activate the serial queue tests.
2. Check all task/semaphore creation and report a clear degraded state.
3. Secure TCP CAT before enabling it outside a trusted isolated LAN.
4. Keep queue depth, age, drop, and suppression telemetry enabled and include
   lock-wait/dispatch-drop counters in the same periodic report.

## Verification strategy after changes

- Run the target Unity suite in the real ESP-IDF environment, including the
  re-enabled serial queue tests.
- Add host/target tests for TCP send error handling and timeout recovery.
- Stress UART TX with concurrent producer tasks and verify exact byte framing.
- Stress RX with more than one ring capacity while delaying the consumer.
- Use ThreadSanitizer only where an appropriate host-compatible abstraction is
  available; on target, use deterministic barriers and repeated stress loops.
- Verify on hardware with AI-mode traffic, encoder activity, USB CDC polling,
  display forwarding, and an intentionally disconnected TCP client.

