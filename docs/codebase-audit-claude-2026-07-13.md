# ARCI Codebase Audit (Claude) — 2026-07-13

## Method

Five independent subsystem audits (RadioCore concurrency, transports, state
management, input/peripheral handlers, lifecycle/perf/tests) were run without
access to `docs/codebase-audit-2026-07-13.md` (the GPT audit), then merged and
arbitrated. Conflicting claims between the two audits were resolved by direct
code reads. Section "Convergence with the GPT audit" at the end maps every GPT
finding to a verdict.

Severity legend: consequence × likelihood, on-target. `NEW` = not in the GPT
audit.

## Remediation status (updated 2026-07-13, same day)

Three implementation waves were completed after this audit. Status per finding:

| Status | Findings |
| --- | --- |
| ✅ Fixed (P0 wave) | C1 TCP self-deadlock; C2 mux atomicity (RAII `ChannelGuard`); H1 `getMessageView` copy-out; H2 `sendLocal` dispatch bypass (+ dead conveniences deleted); H3 `lastSentFrame_` fixed buffer; H5 TX mutex inside `SerialHandler::sendMessage` (`radioTxMutex_` deleted); `rtos_mutex.h` missing `<mutex>` include |
| ✅ Fixed (P1 wave) | H4 66 shared fields atomicized; M2 fused tag+timestamp tracker (release/acquire); H6 macros → `macro_async` worker, PS0/transverter sync → one-shot tasks, no `vTaskDelay` under `dispatchMutex_` anywhere; M1 `DispatchOutcome` + `?;` on timeout + retry-once in radio/display tasks; H7 button event queue (single ownership); H8 encoder wake ISR armed + peek-then-commit pulse accounting; M5 NVS handle RAII; M6 deferred power-off saves |
| ✅ Fixed (P2 wave) | H9 WiFi backoff reconnect; M4 TCP pending-TX buffer; M9 opt-in auth token + accept rate-limit + bind addr; M10 `stop()` join handshake; M11 all metrics coherent/wired (fictional `feedbackLoopsPrevented` deleted); M14 WDT panic + drain feeds; M15 stuck-INT watchdog, stack-bytes constant, PCF8575 bounded wait, init-order callback races (TCA8418 via main.cpp reorder, PCF8575 via release/acquire `m_callbackReady`); M16 fail-fast creation; L7 fd revalidation; serial-queue test suite revived (+H1 regression test); dead code deleted (main.cpp, RadioCore, ButtonHandler, `readMultipleRegisters`, two-part `sendMessage` chain, CDC `onFrameCallback_`); L2 stack monitor wired with real sizes |
| ✅ Fixed (P3 wave) | M7 CDC TX serialized (drain+enqueue under one lock, residue-ordering guard, atomic backpressure); M8 UART 64-byte overflow discard + fragment resync-through-terminator; M12 MacroStorage cache mutex (NVS off-lock); M13 `ParamValue::wasTruncated()`; `bandDownSlotIndex` bug fixed via shared `bandNumber` (F-button-3 steps N→N-1); `sendCommandSequence` propagates dispatch failures; VOX end-to-end (button reads `voxEnabled`; `handleVX` now writes it on set/answer); encoder gesture-end idle nuance + `tuningStopTime`; direct `CatParser::parseFrame`/`determineCommandType` tests added (Set/Read/Answer, EX 7v8, PS); vacuous CatParser tests given real assertions; stray `nul`/`README.md~` deleted |
| ⬜ Open (Medium) | M3 TOCTOU toggles (no longer UB after H4, but lost-update button toggles remain — needs semantic toggle-in-handler) |
| ⬜ Open (Low/hygiene) | Boot latency (10 s WiFi block + 3 s CDC sleep); Diagnostics runtime-stats text parsing; ADCHandler idle 50 Hz no-op task; `SerialHandler` destructor task-delete; CDC >256 B / TCP >128 B frame truncation without terminator |
| ⬜ Open (Tests) | Vacuous `TEST_ASSERT_TRUE(true)` / `TEST_IGNORE` guards in CommandHandlers suite (~12+16); zero coverage for NvsManager/WiFiManager/Diagnostics; inert per-component `test/CMakeLists.txt` files (vestigial — `unit_test` is the real runner) |

---

## Critical

### C1. TCP send-error self-deadlock (GPT #1 — confirmed)

`TcpCatBridge::sendToActiveClient()` holds `clientsMutex_`
(`TcpCatBridge.cpp:331`); the fatal-send-error path calls `closeClient(i)`
(`:350`), which re-takes the same non-recursive `RtosMutex` (`:389`) with
`portMAX_DELAY` → permanent block. The blocked task is whichever task ran
`dispatchMessage` (it still holds `dispatchMutex_`), so all CAT ingress
degrades to 2-s timeouts, and the WDT (advisory-only, see M14) never recovers
it. Remotely inducible by any client that disconnects abruptly mid-response.

**Fix:** call `closeClientLocked(i)` — the correct variant already exists at
`:368` and is used everywhere else. One line.

### C2. TCA9548 mux channel-select and device transaction are not atomic (NEW)

`TCA9548Handler::selectChannel` locks only the channel-register write
(`TCA9548Handler.cpp:96-116`); every consumer then performs its I2C
transaction *outside* the lock (`TCA8418Handler.cpp:349-392`,
`PCF8575Handler.cpp:238-301`). Three equal-priority (12) tasks share the mux,
and both TCA8418 keypads sit at the same address 0x34 on channels 0/1
(`main.cpp:298,309`). A preemption between select and transaction makes task
A's transaction *succeed against the wrong keypad* — silently draining the
other matrix's event FIFO (lost/cross-fired buttons) or corrupting its config
during recovery writes.

**Fix:** RAII bus-scope lock held across select + transaction (e.g.
`lockChannel(ch)` returned by the mux handler).

---

## High

### H1. `getMessageView()` returns a view into an already-released ring slot (GPT #3 — confirmed)

`SerialHandler.cpp:184-194` builds the `string_view` on `m_queue[m_head]`,
then advances head / decrements count *before* returning. The producer
(`uartEventTask`, prio 12) outranks the consumers (`radio_task`/`display_task`,
prio 8). Worst case: when the queue is full, the *very next* received frame is
written into exactly the slot being parsed (`:642-645`) — no 64-slot wrap
needed. Consumers hold the view across `dispatchMessage()` (up to 2 s on lock
timeout) plus USB/display forwarding (`main.cpp:497-535`, `:560-598`), so the
window is real precisely under the AI2/AI4 load the 64-slot queue was sized
for. Also violates the `ISerialChannel.h:59` lifetime contract.

**Fix:** copy the frame into a per-channel scratch buffer under the spinlock
(the pattern `CdcSerialHandler::m_frameBuffer` already implements correctly)
and return a view of that.

### H2. `sendLocal(string_view)` bypasses the dispatch mutex (NEW)

`RadioManager.h:522` calls `localHandler_->parseMessage(frames)` directly — no
`dispatchMutex_`. Reachable from ButtonTask: split button → `toggleSplit()`
(`ButtonHandler.cpp:333,2258`) → `disableSplit()` → `sendLocal("FT0;")`
(`RadioManager.cpp:1410,1414`). If pressed while a USB/TCP client is
mid-dispatch, two tasks run the *same* stateful `CatParser`/handlers
concurrently — every invariant the dispatch lock protects is void in that
window. The sibling `initializer_list` overload (`RadioManager.cpp:1360`)
routes correctly, so this is clearly an oversight. The unlocked header
conveniences `copyVfoAToB()/setRxOnA()/…` (`RadioManager.h:612-632`) share
the defect but currently have no callers.

**Fix:** `sendLocal(string_view)` → `dispatchMessage(*localHandler_, frames)`.
One line. Delete or fix the dead conveniences.

### H3. `lastSentFrame_` (std::string) data race — heap-corruption class (GPT #7a — confirmed, severity raised)

Written on every UART1 TX by whichever task called `sendMessage`
(`SerialHandler.cpp:335-339`), read via `.c_str()` from the prio-12 UART event
task on `?;`/`E;`/`O;` (`:476,522-526`), no lock. Frames >15 chars (e.g. `KY`
text) leave SSO; concurrent `assign()` against a reader, or two writers, is
use-after-free / double-free territory, not just torn diagnostics.

**Fix:** fixed-size `char` buffer guarded by the existing spinlock (or a small
TX mutex per H5).

### H4. Shared `RadioState` fields documented "MUST be atomic" are plain, and are read/written cross-task (GPT #4 — confirmed + extended)

`RadioState.h:340-343` demands atomicity for the shared region; the fields
are plain: `mainAntenna`, `attenuator`, `rxAtIn/txAtIn/atTuning`, `fineTune`,
`preAmplifier`, `processor`, `agcMode`, `noiseBlanker`, `nr1Level/nr2Speed`,
`transmitPower`, `keyingSpeed`, `carrierLevel`, `speechProcessorIn/OutLevel`,
`manualNotchFrequency`, band slot indices, and more (`:345-362, :377-443`).

Beyond the reads GPT found, the **panel encoder task writes these fields
outside the mutex**: `adjustUIValue` write-backs at `RadioManager.cpp:2200,
2210, 2217, 2230, 2237, 2266, 2301, 2308`. Example: the panel builds
`PL%03d%03d` from an unlocked read of `speechProcessorOutLevel` (`:2231`)
while a CAT client's `PL` set is dispatching — the panel frame reverts the
client's change. On Xtensa aligned ≤4-byte accesses don't tear, so the
practical failure is lost/stale updates plus compiler-reordering license (and
formal UB); `int64_t` fields do tear (see H7).

**Fix:** atomicize the listed fields (mechanical; all ≤4 bytes) and funnel
`adjustUIValue` write-backs through dispatch. Longer term, narrow the mutable
`getState()` surface (F9 below).

### H5. Radio-UART TX has three serialization domains (GPT #2 — confirmed with corrections)

Writer inventory for UART1:

| Serialized by | Sites |
| --- | --- |
| `radioTxMutex_` | `sendRadioCommand` (`RadioManager.cpp:881,927`) |
| `dispatchMutex_` only | `BaseCommandHandler::sendToRadio` (`BaseCommandHandler.cpp:59`), auto-query (`CommandDispatcher.cpp:446,498`), antenna/memory handlers |
| Nothing | `txTimeoutTask` (`RadioManager.cpp:1926,1952`), `keepaliveTask` (`:1995,2011`), main_task EX refresh (`main.cpp:786`), startup `PS;` (`main.cpp:1047`) |

**Correction to the GPT audit:** the byte-splicing scenario ("body from task
A, then task B's frame, then A's terminator") is *not currently live*. The
split body/terminator write in `sendMessage` (`SerialHandler.cpp:308` + `:321`)
only executes when the caller passes an unterminated frame (`needs_terminator`,
`:239`), and every current radio-UART caller passes complete `;`-terminated
frames (verified: literals, `setToReadMap` values, `originalMessage` retains
its `;` per `RadioCommand.h:92` / `isValidCAT`). Single `uart_write_bytes`
calls are serialized by the IDF driver. What remains real: the H3 string race
lives on this path; the free-space check-then-write (`:288-304`) is TOCTOU
across domains (drops, not corruption); direct writers never call
`recordCommandSentToRadio`, so `?;` error diagnostics misattribute the failing
command; and the two-arg spliceable `sendRadioCommand(part1, part2)` is loaded
but currently caller-less. The architecture point stands: no single TX owner.

**Fix:** move TX serialization *into* `SerialHandler::sendMessage` (covering
free-space check + writes + `lastSentFrame_`), delete `radioTxMutex_`.

### H6. CAT-initiated macros sleep while holding the dispatch lock (extends GPT #5)

`MXE<slot>;` from any CDC/TCP client runs inside `dispatchCommand` (lock held)
→ `executeUserMacro` (`RadioMacroManager.cpp:247-265`) → NVS reads +
`vTaskDelay(50 ms)` *per macro command*, each re-entering the recursive
mutex. A 20-command macro holds the lock ≥1 s; 40+ commands exceed the 2-s
`try_lock_for` (`RadioManager.cpp:783`) and every other interface's commands
are **silently dropped** (results void-cast, `main.cpp:413,462,870,909` — the
client never even gets `?;`). GPT's example (PS0's 2×50 ms,
`InterfaceSystemCommandHandler.cpp:408-417`) is the mild version; further
instances: `syncTransverterMenuSettings` 5×10 ms + 6 sends under lock
(`RadioManager.cpp:1084-1096`), and NVS *flash commits* under lock via the
power-state callback (M6) and `MXW/MXD` (`MXCommandHandler.cpp:322`).

**Fix:** `handleMXE` enqueues to a worker (the F-button path already runs
macros outside the lock); move NVS/delays out of handler context —
`bootSequenceTask` (`RadioManager.cpp:999-1052`) demonstrates the right
pattern.

### H7. Button state mutated concurrently from two tasks (NEW)

`buttonTask` (prio 5) runs `button.update()`/long-press checks every 50 ms
(`ButtonHandler.cpp:100-158`) while the prio-12 TCA8418 `keyTask` mutates the
same `MatrixButton` objects via callbacks (`:1070-1339`). `Button` state is
plain fields including `int64_t m_pressedTime` (`Button.h:51-62`) — which
**does tear on 32-bit Xtensa**. A torn read can spuriously satisfy the
long-press comparison: on the POWER button that sends `PS0;` (radio off,
`ButtonHandler.cpp:247-253`); the check-then-set consumption flags can also
double-fire MOX (`TX2;`, `:2499-2528`).

**Fix:** single ownership — keyTask enqueues raw key events to ButtonTask.

### H8. Encoder wake ISR never fires; fixing it exposes a pulse-drop bug (NEW)

(a) PCNT branch configures pins `GPIO_INTR_DISABLE` (`EncoderHandler.cpp:133`)
then registers `wakeIsrHandler` (`:239-240`) without ever setting an edge type
— the ISR is dead, tuning wakes only on the 50-ms semaphore timeout (`:314`).
Up to 50 ms latency at gesture start; `m_wakeIsrCount` always 0.
(b) Interlocked: `:415-426` irreversibly consumes pulses from
`m_edgeRemainder` into `delta`; if no update is due (`:470,:492`) the function
exits and `delta` evaporates. Today masked because the 50-ms wake exceeds the
15-ms live-update gate — enable the ISR without fixing (b) and the VFO knob
under-counts ("slippery"). Fix both together.

### H9. WiFi gives up permanently after 5 retries; `WIFI_AUTO_RECONNECT` is dead Kconfig (NEW)

`WiFiManager.cpp:346-360`: 5 immediate retries (no backoff), then
`WIFI_FAIL_BIT` forever; `retryCount_` only resets on GOT_IP. Kconfig symbols
`WIFI_AUTO_RECONNECT` / `WIFI_CONNECTION_TIMEOUT_MS` are referenced nowhere.
AP reboots for 60 s → TCP CAT (a primary control path) is dead until
device power-cycle.

**Fix:** periodic backoff reconnect honoring the existing Kconfig.

---

## Medium

### M1. Dispatch-lock timeout paths silently lose or half-process work (GPT #5b — confirmed + extended)

USB/TCP callers void-cast `dispatchMessage` results (`main.cpp:413,462,870,909`)
— a 2-s timeout is a silently vanished client command. Worse, `radio_task`
(`main.cpp:507-536`) treats timeout as `wasHandled=false` and **raw-forwards
the radio answer to USB/display without updating state/caches** — clients see
an answer ARCI's own cache doesn't reflect; subsequent cached query replies
are stale. **Fix:** distinguish lock-timeout from handler-false in the return;
reply `?;` on drop; don't forward unprocessed answers as if handled.

### M2. Cache tag+timestamp published as two independent relaxed stores (GPT #7b/#8-adjacent — confirmed + extended)

`CommandTimestampTracker::record()` (`RadioState.h:86-127`) stores tag then
timestamp, both relaxed; readers check tag then load timestamp. Cross-pairing
(new tag, old timestamp) is possible during slot replacement. New concrete
scenario: **lost invalidation** — ButtonTask's band-change
`commandCache.invalidate("FA")` (`ButtonHandler.cpp:451,559,…` →
`RadioManager.cpp:468-470`) racing a concurrent `record()` of a
pre-band-change FA answer can leave the *old frequency marked fresh*, and
cached-query replies serve it for the TTL window. Same shape for value+
timestamp pairs: `updateVfoAFrequency` (`RadioManager.cpp:507-526`) from
EncoderTask vs a stale in-flight FA answer under the mutex → "old value, new
timestamp", frequency visibly reverts mid-tuning. **Fix:** fuse tag+timestamp
into one 64-bit atomic (16-bit tag + 48-bit µs) or per-slot seqlock;
release/acquire publication; route encoder cache write-backs through dispatch.

### M3. TOCTOU toggle pattern in every ButtonTask toggle (GPT #4-adjacent — confirmed)

Read cached state (no lock) → invert → dispatch: attenuator
(`ButtonHandler.cpp:2198`), preamp (`:2222`), processor (`:358`), AGC (`:398`),
NB (`:414,460`), antenna (`:2601`), TX-ATU (`:2643`), split
(`RadioManager.cpp:1427`). A concurrent CAT client toggle makes the button
press a visible no-op or the wrong direction. **Fix:** semantic toggle
resolved inside the handler under the dispatch lock.

### M4. TCP partial-send silently drops the frame tail → permanent client desync (NEW)

Sockets are non-blocking (`TcpCatBridge.cpp:185-186`); on EAGAIN after a
*partial* send the remaining bytes are dropped (`:352-356`). The client
receives half a frame and every subsequent frame concatenates garbage —
unrecoverable desync, unlike dropping whole frames. **Fix:** per-client
pending-TX buffer, or whole-frame drop / disconnect on would-block.

### M5. NVS handle leak on every `saveRadioState` error path (NEW)

Nine early `return err;` between `nvs_open` (`NvsManager.cpp:101`) and
`nvs_close` (`:141`). Repeated transient failures exhaust the NVS handle pool
→ all NVS opens (macros, EX menu, button memory) start failing. **Fix:** RAII
handle wrapper.

### M6. Blocking flash commits inside the power-state callback on the dispatch path (NEW)

`updatePowerState` → `powerStateChangeCallback_` (`RadioManager.cpp:742-744`)
→ `saveRadioState()` + `saveExtendedMenu()` (`NvsManager.cpp:76-91`): two
`nvs_commit` flash writes (tens–hundreds of ms, flash cache stalled) in
whichever task processed the `PS` frame, under the dispatch lock (see H6).
Queued radio frames hit the 1-s expiry / drop-oldest path meanwhile.
**Fix:** defer saves to a low-priority worker.

### M7. CDC TX reordering under backpressure + torn backpressure state (NEW)

`UsbCdc::writeData` drains the retry ring, then queues new data
(`UsbCdc.cpp:338-376`); if the drain exits with residue (`:471-473`) and the
host frees FIFO space before the new-data queue call, new bytes overtake old
ring residue → garbled CDC stream. Multiple unserialized writer tasks
(radio_task, dispatch contexts, txTimeoutTask); `m_writeBlockUntilUs` is a
plain `uint64_t` on a 32-bit CPU — torn read/write (`UsbCdc.cpp:27,667-702`).
**Fix:** one TX mutex per CDC instance across check+drain+queue; atomic
backpressure state.

### M8. UART accumulator: exactly-64-byte frame is queued corrupted, without terminator (NEW)

Append is guarded (`SerialHandler.cpp:550`) but the END_MARKER branch runs
regardless (`:561`): 64 non-`;` bytes then `;` → the 64 junk bytes are queued
as a frame (backward scan `:578-597` blesses any two letters). The related
fragment-shift heuristic (`:600-615`) can splice line noise into
plausible-looking frames. **Fix:** treat full-accumulator-at-`;` as overflow
and discard; on unmatched `;` drop through the last `;`.

### M9. TCP bridge is an unauthenticated LAN transmitter-control surface (GPT #9 — confirmed + extended)

`INADDR_ANY` bind (`TcpCatBridge.cpp:124`), no auth, no idle timeout; plus
`acceptClient` (`:161-167`) silently evicts the current client on any new
connection — a reconnect loop is a trivial operator-DoS. PTT/power are
reachable. **Fix:** Kconfig-gated shared-secret handshake, optional bind
address, rate-limit connects.

### M10. `TcpCatBridge::stop()` teardown races (NEW)

`stop()` force-`vTaskDelete`s after 100 ms (`TcpCatBridge.cpp:88-97`) — the
task may hold `clientsMutex_` (subsequent `closeAllClients()` in `stop()`
deadlocks); the task's self-exit path (`:478-482`) races the handle null-check.
Shutdown/WiFi-loss only. **Fix:** signal via `running_`, wait for confirmed
exit, clean up in one place.

### M11. Statistics are raced or inert (GPT #8 — confirmed)

`CommandDispatcher::stats_` counters are written without `statsMutex_`
(`CommandDispatcher.cpp:219,306,403,426,…`) while `getStatistics()` copies
under it (`:538`); `CATHandler::stats_` relies on the (H2-broken) dispatch
convention. `RadioManager::Statistics` (`RadioManager.h:91-102`) has **zero
writers** — Diagnostics prints permanent zeros (`Diagnostics.cpp:251-261`);
`CatParser` stats are maintained but never read. CDC `sendFailures_` is never
incremented despite real drop paths (`CdcSerialHandler.cpp:79-84,129-134`).
**Fix:** atomics for counters; wire up or delete the inert structs.

### M12. MacroStorage cache accessed cross-task without a lock (NEW)

`cache_` written via CAT dispatch (MXW/MXD/slot ops), read from ButtonTask
(`RadioMacroManager.cpp:225,290`); ~100-byte `MacroDefinition` copies can
tear. **Fix:** small mutex; writes are rare.

### M13. `ParamValue` silently truncates parameters ≥16 chars (NEW)

`RadioCommand.h:52-63` truncates to 15 chars, no flag, no log; the default
`parseParameters` branch routes unlisted prefixes here (e.g. `KY` 24-char
text). Mitigated today because handlers mostly parse `originalMessage`.
**Fix:** log/count truncation or overflow to heap params.

### M14. Watchdog is advisory-only; inner drain loops don't feed it (NEW)

`CONFIG_ESP_TASK_WDT_PANIC` not set (`sdkconfig:1999`) — a hung task logs and
lives (see C1). The per-task inner `while(hasMessage())` drain loops
(`main.cpp:403-425,450-475,495-537,558-599`) don't reset the WDT under
sustained flood. **Fix:** enable panic; reset (or cap batch) inside drains.

### M15. Input-side lifecycle issues (NEW)

- **Init-order callback races:** TCA8418 keyTask + interrupts start before
  `m_keyCallbacks` (a `std::map`) is populated (`TCA8418Handler.cpp:113-177`
  vs `main.cpp:299-316`; init self-gives the semaphore at `:176`) — concurrent
  map read/insert is UB, boot-reproducible with a held key. Same shape for
  PCF8575 → `MultiEncoderHandler` (`PCF8575Handler.cpp:127` vs `main.cpp:999`).
- **PCF8575 task waits `portMAX_DELAY`** (`PCF8575Handler.cpp:344`) — one
  missed edge freezes AF/RF-gain + filter encoders until reboot (TCA8418's
  500-ms fallback pattern at `TCA8418Handler.cpp:450` is the fix).
- **keyTask stack is 6 KB, not the documented 24 KB** — `KEY_TASK_STACK_WORDS
  = 6144; // ~24KB` (`TCA8418Handler.h:155`) but `xTaskCreate` takes bytes;
  Encoder/Button tasks run the same deep CAT chain on 4 KB.
- **TCA8418 stuck-INT watchdog is dead code:** `m_lastIntTime` is refreshed
  immediately before the check (`TCA8418Handler.cpp:483-486` vs `:832`).

### M16. Unchecked task/semaphore creation at startup (GPT #10a — confirmed)

`main.cpp:963-966` (4 semaphores), `:1034-1038` (5 tasks),
`Diagnostics.cpp:236`, `ButtonHandler.cpp:65`, `EncoderHandler.cpp:267`,
`ADCHandler.cpp:42`. Heap pressure at boot → missing transport/input task,
system reports success. **Fix:** fail fast with a clear log.

---

## Functional bugs (found incidentally)

- **`bandDownSlotIndex` is never written** — only init + read
  (`RadioState.h:345`, `ButtonHandler.cpp:430`): Function-button-3 always
  computes 0→10 and sends `BD10;` (jump to GENE) on every press.
  `bandUpSlotIndex` is never synced with CAT-initiated BU/BD either.
- **`RadioMacroManager::sendCommandSequence` cannot fail** — dispatch result
  discarded, unconditional `return true` (`RadioMacroManager.cpp:127,140`);
  the downstream error branches and the transverter test assertion are
  unreachable.
- **Local VOX state desyncs** — function-`static bool voxState`
  (`ButtonHandler.cpp:2560,2584`) ignores CAT-initiated VOX changes.
- **`Diagnostics::printStackStatus` is never called**, and if enabled would
  misreport: stack sizes hardcoded to 4096 vs actual 8192/8192/8192/6144/4096
  → unsigned underflow (`Diagnostics.cpp:61-92,21-33` vs `main.cpp:1034-1038`).

---

## Low (selected)

- CDC frame >256 B truncated without terminator (`CdcSerialHandler.cpp:176,229`)
  — theoretical for CAT. (This is the real kernel of GPT #6; see convergence.)
- TCP stale-index/fd reuse between select snapshot and `handleClientData`
  (`TcpCatBridge.cpp:429-471`) — benign today, fragile after the C1 fix;
  re-validate the fd.
- `SerialHandler` destructor deletes a task that may hold the spinlock; leaks
  `dtmp` (`SerialHandler.cpp:19-27`) — shutdown-only.
- `rtos_mutex.h:133,138` uses `std::adopt_lock_t`/`std::defer_lock_t` without
  `#include <mutex>` — transitive-include landmine.
- Dead code with latent traps: `sendVfoUpdates` (mutating static, no callers),
  `changeBand()` (sleeps, no callers), two-arg `sendRadioCommand` (spliceable,
  no callers), `togglePowerState()` (`ButtonHandler.cpp:186-216`), split
  numeric-entry path (flag never set), `initializeNVS()`/`debugPrintMsg`/
  `lastRadioCommand` in main.cpp, `readMultipleRegisters` (would read the
  wrong mux channel if revived), CDC `getMessage()` (bypasses accumulator) and
  `onFrameCallback_` (stored, never invoked). Project rule: delete.
- Boot latency: up to 10 s WiFi block + fixed 3 s CDC sleep before CAT tasks
  exist (`main.cpp:844,1008`).
- ADCHandler spins a 50 Hz no-op task with zero channels configured.
- Diagnostics runtime-stats parsing uses unbounded `vTaskGetRunTimeStats` into
  2048 B + unbounded `%s` sscanf (`Diagnostics.cpp:103,122,167`).
- Stray `nul` file at repo root; `README.md~` backup file.

## Test health (GPT #10b — confirmed + extended)

- SerialHandler queue suite disabled (`tests_main.cpp:54,79`) *and* stale
  (expects capacity 16 vs current 64) — the only coverage for the trickiest
  concurrency in the system (H1's ring buffer).
- Vacuous tests inflate the pass count: `test_validate_FA/MD_parameters` have
  zero assertions; 12× `TEST_ASSERT_TRUE(true)` in the consolidated handler
  suite ("passes if no exceptions" in a `-fno-exceptions` build); ~17 soft
  `TEST_IGNORE` guards.
- `CatParser::parseFrame`/`determineCommandType` (the Set/Read/Answer
  inference matrix, EX 7-vs-8 rule, PS exception) has no direct unit tests;
  `test_cat_parser.cpp` duplicates `test_cat_parser_utils.cpp`.
- No tests at all: NvsManager, WiFiManager, MacroStorage, Diagnostics,
  RadioMacroManager user-macro path.
- Per-component `test/CMakeLists.txt` files are inert (only
  `components/unit_test` builds); the AntennaSwitch "test" doesn't compile.
- Zero coverage for any concurrency finding above (by nature of Unity
  on-target tests, but deterministic stress tests are feasible — see GPT
  audit's verification strategy, which is sound).

## Verified sound (checked, no issue)

Lock ordering overall (dispatch → stats / dispatch → radioTx are the only
nestings, no cycles); the recursive dispatch mutex + adopt pattern including
its timeout path; priority inheritance everywhere (no unbounded inversion);
TX-ownership and control-lease state machines (`RadioState.cpp`) — all
transitions under their mutexes, release correctly owner-scoped; origin-routing
tables' release/acquire publication (`RadioManager.cpp:1828-1885`); the
SerialHandler RX enqueue path and its drop/expiry telemetry; ISR discipline in
every input component (FromISR APIs only, IRAM_ATTR present, no logging/alloc);
QuadratureDecoder table; PCNT overflow/tick math; CDC RX accumulator overflow
arithmetic (traced byte-by-byte — correct drop-oldest suffix); CDC
`getMessageView` copy-out lifetime (the model H1 should copy); TCP RX
extract-under-lock-dispatch-outside pattern; boot-sequence task pacing;
`compactHash` bounds; WDT registration/timeout budget vs the 2-s lock wait;
CatParser malformed-input handling and `from_chars` usage; NvsManager blob
validation; WiFi event-group/PMF/SNTP setup (modulo H9).

---

## Convergence with the GPT audit (`codebase-audit-2026-07-13.md`)

| GPT finding | Verdict |
| --- | --- |
| 1. TCP send-error self-deadlock (Critical) | **Confirmed**, same lines. Agreed top fix. |
| 2. UART TX no single owner (High) | **Confirmed architecturally; mechanism corrected.** Byte-splicing via split body/terminator writes is not live — all current callers pass `;`-terminated frames (single driver-serialized write). Real content: three serialization domains, H3 string race, TOCTOU space check, misattributed error diagnostics, latent two-arg splice. Same fix direction (single TX owner). |
| 3. RX `string_view` unleased slot (High) | **Confirmed**; worse than stated — full-queue case corrupts on the *next* frame, no 64-wrap needed; producer outranks consumers. |
| 4. `RadioState` data races (High) | **Confirmed + extended** — panel `adjustUIValue` *writes* outside the mutex; and the bigger hole is H2 (`sendLocal` bypasses the dispatch mutex entirely), which GPT missed. |
| 5. Global dispatch lock HOL + silent drops (High) | **Confirmed + extended** — worst case is not PS0's 100 ms but CAT-initiated user macros: 50 ms × N under the lock, >2 s for large macros, guaranteeing the silent-drop path; plus NVS flash commits under the lock. Also: radio_task's timeout path forwards answers without state update (M1). |
| 6. CDC overflow corrupts accumulator (Medium) | **Refuted.** Traced byte-by-byte: the overflow path yields exactly the last 512 bytes of old+new (correct drop-oldest, loss logged). The adjacent real defect is >256 B frame truncation without terminator — Low, theoretical for CAT. |
| 7. `lastSentFrame_` race + tracker incoherence (Medium) | **Confirmed; string race upgraded to High** (heap-corruption class, not just diagnostics). Tracker: added the lost-invalidation and old-value/new-timestamp scenarios (M2). |
| 8. Unreliable metrics (Medium) | **Confirmed**, same lines; added inert CDC `sendFailures_` and unread CatParser stats. |
| 9. TCP unauthenticated (Medium) | **Confirmed**; added eviction-DoS and partial-send desync (M4). |
| 10. Unchecked creation + stale tests (Low) | **Confirmed + substantially extended** (vacuous tests, inert test CMake, parser coverage gap). |

**Overall:** 9 of 10 GPT findings confirmed (several verbatim to the line);
one refuted (#6); one materially corrected in mechanism but not in conclusion
(#2). The GPT audit's transport/dispatch analysis is accurate and its
remediation order is sensible. Its blind spots were the I2C/input layer
(C2, H7, H8, M15 — including the second Critical in the codebase) and
lifecycle items (H9 WiFi, M5 NVS leak, M6 flash-under-lock), plus the
highest-leverage one-line fix (H2 `sendLocal`).

## Recommended remediation order

**P0 — freezes, corruption, wrong-device I/O (small, surgical):**
1. C1 `closeClientLocked` (one line).
2. H2 `sendLocal` → `dispatchMessage` (one line); delete dead conveniences.
3. C2 mux RAII lock across select+transaction.
4. H3 `lastSentFrame_` → fixed buffer under lock, folded into
5. H5 TX mutex inside `SerialHandler::sendMessage`; delete `radioTxMutex_`.
6. H1 copy-out in `getMessageView` (adopt the CDC pattern).

**P1 — state ownership and latency:**
7. H4 atomicize shared fields; route `adjustUIValue` write-backs via dispatch.
8. H6 macros/NVS/sleeps off the dispatch lock (worker task).
9. M1 distinguish lock-timeout from unhandled; `?;` on drop.
10. M2 fused tag+timestamp atomic; encoder cache write-backs via dispatch.
11. H7 single-ownership button events; H8 encoder ISR + pulse-drop together.

**P2 — robustness and operability:**
12. H9 WiFi reconnect; M5 NVS RAII; M6 deferred saves; M14 WDT panic + drain
    feeds; M16 fail-fast creation; M15 init-order + PCF8575 timeout + stack
    sizes; M4/M9/M10 TCP hardening.
13. M11 metrics coherence; re-enable + fix the serial queue suite; delete dead
    code; deterministic stress tests for H1/H5/C1 per the GPT audit's
    verification strategy.
