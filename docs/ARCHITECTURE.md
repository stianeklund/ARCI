# Quick Memory — TS‑590SG Remote CAT (Command‑Based)

Distilled overview of the command-based CAT architecture for the Kenwood TS-590SG on ESP32-S3.

## 1) Purpose (ESP32‑S3 + TS‑590SG)

- Emulate a TS‑590SG over USB‑CDC so PC CAT apps (N1MM, Hamlib, HRD, fldigi, etc.) see a local radio.
- Forward CAT frames to the real TS‑590SG over UART/WAN and return radio answers back to USB.
- Use a unified, source‑aware, command system for parsing, routing, and hardware control.

## 2) High‑Level Flow (Command-Based Architecture)

USB, TCP network, Display UART2, or Radio UART (bytes)
  ↓
main.cpp (routes based on source)
  ↓
RadioManager.getLocalCATHandler() (USB/TCP/Display) or RadioManager.getRemoteCATHandler() (Radio)
  ↓
CATHandler (with source awareness) → CatParser (splits multi‑frame strings like "FT1;FR0;")
  ↓
RadioCommand { command, type=Set|Read|Answer, source, params }
  ↓
CommandDispatcher → SpecializedCommandHandler (one of 16 categories)
  ↓
RadioManager (state management & hardware I/O)
  ↓
USB, TCP network, Display UART2, and Radio serial (routing based on source)

- Local (USB/TCP/Display): Commands processed immediately via CommandDispatcher, use cache when available
- Remote (Radio): Responses update state directly via command handlers and forwarded to interfaces
- Source‑aware routing prevents feedback loops between interfaces
- All interfaces receive both cached query responses and real-time radio updates

## 3) Core Components (what to look for)

**Primary Architecture (Post-Refactoring):**
- **RadioManager**: `components/RadioCore/RadioManager.{h,cpp}` — **Central component**: owns CommandDispatcher/CATHandlers, manages state, provides handler accessors (getLocalCATHandler/getRemoteCATHandler), exposes convenience methods and high-level helpers (e.g., enableSplit/disableSplit) to keep UI CAT‑agnostic
- **CATHandler**: `components/RadioCore/CATHandler.{h,cpp}` — Source-aware message parser wrapper around CatParser, calls CommandDispatcher directly
- **CommandDispatcher**: `components/RadioCore/CommandDispatcher.{h,cpp}` — O(1) registry, routes commands to handlers
- **TcpCatBridge**: `components/TcpCatBridge/TcpCatBridge.{h,cpp}` — TCP server for network-based CAT control, routes to dedicated CATHandler instances
- **RadioMacroManager**: `components/RadioMacroManager/RadioMacroManager.{h,cpp}` — Unified macro execution (semantic + user-defined)
- **CatParser**: `components/RadioCore/CatParser.{h,cpp}` — One parser for all sources
- **BaseCommandHandler**: `components/RadioCore/BaseCommandHandler.{h,cpp}` — Common utilities, zero‑heap style
- **Specialized Handlers (16)**: `components/RadioCore/*CommandHandler*.{h,cpp}` — Updates RadioState directly

The 16 Handler Categories (category and related CAT commands):

1. Frequency/VFO: FA, FB, DN, UP, FR, FT, FS, TS, SP, RC RT, XT, RD, RU, VV, XO, EM, RI, XI
2. Mode/Band: MD, DA, MK, AS, BU, BD
3. Gain/Levels: AG, RG, MG, CG, ML
4. RX DSP: PA, RA, FL, FW, SH, SL, IS, GC, GT, NB, NR, RL NT, BP, BC
5. Transmitter: TX, RX, PC, TP, PR, PL, VX, VG, VD
6. Antenna/Tuner: AC, AN
7. CW Ops: KS, SD, KY, CA, CD0, CD1, CD2
8. Memory: MC, MW, MR, QI, QD, QR, SV
9. Scan: SC, SS, SU
10. Tone/Squelch: SQ, TO, TN, CT, CN
11. Audio/EQ: EQ, UR, UT
12. Status/Info: SM, RM, IF, ID, FV, TY, TC, RS, BY
13. Voice Msg: LM, PB, VR
14. Visual Scan: VS0, VS1, VS2,VS3, VS4
15. Menu/Config: MF, EX, ES
16. Interface/System: AI, PS, SR, LK, TC

Each handler is the single owner of its commands; no overlap. UI code should not construct CAT frames directly.

Notes on Split (TS‑590SG):
- Enabling/disabling split is achieved via VFO selection: `FR` (RX VFO) and `FT` (TX VFO) & parsing of the `IF` command.
- `SP` is for split frequency/setting, not the enable/disable operation.

## Radio Macro System

**RadioMacroManager** is the unified macro execution engine for all macro types:

### Semantic Macros (hardcoded complex operations)
- **Transverter Macro**: EX056 menu control + AN911 RX Ant + AN991 DRV Out
- **Split Macro**: VFO copy + FR/FT configuration
- **Band Change Macro**: Future implementation for complex band switching
- **Contest Mode Macro**: Future implementation for contest-specific radio setup

### User-Defined Macros (stored in MacroStorage)
- Up to 50 macros stored in NVS
- F-button slot assignments (F1-F6 short/long press = 12 slots)
- Pipe-separated CAT command sequences (e.g., `FA00014074000|MD2|DA1`)
- MX protocol for CRUD operations (MXW, MXR, MXA, MXE, MXD)

**Architecture Pattern:**
- Single execution engine for all macro types (no separate MacroExecutor)
- Commands dispatched through CATHandler for proper state tracking
- Inter-command delays (50ms user macros, 20ms semantic) for radio processing
- Error handling without exceptions (ESP-IDF compatible)
- Used by ButtonHandler (F-buttons), MXCommandHandler (MXE protocol)

**Example Usage:**
```cpp
// Semantic macros
bool result = macroManager.executeTransverterMacro(true);

// User-defined macros
esp_err_t err = macroManager.executeUserMacro(5);   // By ID
esp_err_t err = macroManager.executeSlot(0);        // F1 short press
```

For small, common multi‑frame operations, prefer RadioManager helpers:

```cpp
// Simple split enable via helpers (keeps UI CAT‑agnostic)
radioManager.enableSplit(/*copyVfoBeforeEnable=*/true);  // VV; FR0; FT1;
// Or use direct local handler
radioManager.getLocalCATHandler().parseMessage("VV;FR0;FT1;");
// Disable split
radioManager.disableSplit();  // FT0; FR0;
```

Guideline: macros for longer, multi‑step sequences or user‑configurable flows; RadioManager helpers or direct handler access for short, ubiquitous sequences.

## 4) RadioCommand (type‑safe, source‑aware, zero‑copy)

- Fields: `command` (e.g. "FA"), `type` (Set|Read|Answer), `source` (enum), `params` (typed), `originalMessage` (preserved for zero-copy forwarding).
- **CommandSource enum**: `UsbCdc0`, `UsbCdc1`, `Tcp0`, `Tcp1`, `Display`, `Panel`, `Remote`, `Macro`
- Methods: `shouldSendToRadio()`, `isUsb()`, `isTcp()`, `isLocal()` implement routing policy.
- **Zero-copy pass-through**: Original message is preserved to avoid string reconstruction when forwarding.

Examples

- "FA00014150000;" from USB → Set → forward original to radio using `originalMessage`, update cache timestamp
- "FA;" query from USB → Read → check cache freshness, forward original to radio if stale, respond from cache if fresh
- "FA00014150000;" from Radio → Answer → update RadioState and cache timestamp

## 5) TTL‑Based Query Cache System

**Purpose**: Optimize query performance by using cached data when fresh, reducing unnecessary radio communication.

**Implementation**:
- `CommandTimestamps` class in RadioState tracks last update time per CAT command
- Cache updates occur on: 1) SET commands from USB, 2) ANSWER commands from radio
- Query logic: Check cache age → Use cache if fresh (< 3s default) → Query radio if stale
- Per-command TTL tracking allows independent freshness validation
- All command handlers now implement consistent TTL cache pattern (RM, SD, TO, TP, TS, MF, MG, MK, ML, etc.)
- Local queries check `isCacheFresh()` and return cached data when appropriate

**Benefits**:
- Reduces radio load when data is fresh
- Works consistently regardless of AI mode (AI0/AI2/AI4)
- Zero additional latency for cached responses
- Prevents stale data by enforcing TTL

**Testing**:
- The `test_cache_system.cpp` file contains tests that specifically validate the cache's TTL logic, including cache hits, misses on stale data, and timestamp updates.

## 6) State Sync (No Internal Polling)

Internal polling has been removed to avoid conflicts with the radio’s CAT/AI interface and the RRC tunnel.

- AI modes: Prefer the radio’s Auto Information (AI) updates when enabled; otherwise respond to host queries and push local state changes.
- Cache: TTLs remain for answering local USB queries from cache vs. fetching fresh state via targeted commands initiated by handlers or client requests.
- Display: Receives updates pushed from handlers and forwarded radio answers; the display should not poll the radio.

## 7) CAT Command Spec (SSoT)

- `spec/ts590sg_cat_commands_v3.json` = authoritative spec (set/read/answer formats, enums, widths, min/max).
- Parser/handlers validate against JSON.

## 7) Testing (Unity)

Key tests (names may vary by repo):

- `test_cat_commands.cpp` — Main test file covering CAT commands, state changes, and forwarding logic.
- `test_consolidated_command_handlers.cpp` — Covers all 16 handlers in a more isolated way.
- `test_unified_cat_system.cpp` — parser + dispatcher integration.
- `test_cat_parser_utils.cpp` — parsing/validation utilities.
- `test_cache.cpp` — Verifies the TTL-based query cache system.

Highlights:

- Routing is verified (no command overlap).
- Local vs Remote source behavior covered.
- Cache hits and misses (staleness) are tested.

## 8) Build & Tooling

- ESP‑IDF components + CMake.
- Add CatParser, CommandDispatcher, and all 16 handlers to `CMakeLists.txt`.
- Pre‑commit: clang‑format, cpplint, JSON validation.
- Use `idf.py test` for Unity.

## 9) FAQ‑style gotchas (quick recall)

**Command Flow:**
- Direct flow: main.cpp → RadioManager.getLocal/RemoteCATHandler() → CatParser → CommandDispatcher → Handlers → RadioManager (state updates)
- No event queues or processing loops; commands execute immediately
- CommandDispatcher owned by RadioManager (central component)
- Message source determined once in main.cpp, not re-evaluated downstream

**Architecture Principles:**
- One parser for both directions; intent comes from source + content
- Handlers own business logic; RadioManager coordinates state management + command dispatch
- Registry pattern: adding commands = register handlers; no core changes
- Source‑aware forwarding prevents USB↔Radio feedback loops
- Memory: zero‑heap in hot paths; prefer string_view parsing

**Performance Optimizations:**
- Zero-copy pass-through: Original CAT messages preserved in RadioCommand to avoid reconstruction
- TTL-based caching: Queries use cached data when fresh (< 3s), reducing radio communication
- Per-command cache timestamps: Independent TTL tracking for each CAT command
- Direct forwarding: Pass-through commands skip string rebuilding, using `originalMessage`
- O(1) command dispatch: Hash table lookup in CommandDispatcher for constant-time routing
- Jump table optimization: Handler methods use constexpr lookup tables instead of cascading if-else chains
- Branch prediction hints: [[likely]]/[[unlikely]] attributes for common/uncommon code paths
- Optimized string formatting: Constexpr digit lookup tables for faster CAT response generation
- Reduced memory allocations: Pre-allocated containers and string reserves to minimize heap usage
- Cache-friendly data structures: Optimized hash table load factors and container sizing

## 10) RadioManager Helpers & Direct Access (Keep UI CAT‑agnostic)

UI/feature components (ButtonHandler, EncoderHandler, etc.) should not embed CAT literals. Use:

**High-level helpers (preferred):**

- `RadioManager::enableSplit(copyVfo=true)`, `RadioManager::disableSplit()`, `RadioManager::toggleSplit(copyVfo=true)`
- `RadioManager::copyVfoAToB()`, `setRxOnA/B()`, `setTxOnA/B()`
- `RadioManager::getLocalCATHandler().parseMessage()` for direct local command injection
- `RadioManager::getRemoteCATHandler().parseMessage()` for processing radio responses

**Direct handler access (when needed):**
- `radioManager.getLocalCATHandler().parseMessage("FA00014150000;")` for local commands
- `radioManager.getRemoteCATHandler().parseMessage("FA00014150000;")` for radio responses

**State access:**
- `radioManager.getState()` for direct state access
- `radioManager.getVfoAFrequency()`, `radioManager.getDataMode()`, etc. for convenience getters

Rationale: keeps protocol details centralized, enables future protocol support without touching UI code.

## 11) Towards Multi‑Protocol CAT Support

Goal: allow swapping Kenwood TS‑590SG CAT for other CAT dialects while preserving the domain core.

Recommended layering and contracts:

- Protocol Adapter interface (concept):
  - `parseMessage(std::string_view, source, callback<RadioCommand>)`
  - `formatResponse(...)` helpers (as needed)
  - Current `CatParser` is the Kenwood adapter implementation.

- Stable domain model: `RadioCommand`, `CommandDispatcher`, `ICommandHandler` set, and `RadioManager` remain unchanged.

- CATHandler owns a single adapter instance (injected). To support another protocol, provide a new adapter and relevant handlers while reusing shared domain concepts.

- Command spec (JSON) remains the SSoT per protocol. Select spec/adapter at build time (Kconfig) or runtime if needed.

Pragmatic steps (incremental):
- Step 1: Define a minimal `IProtocolAdapter` interface in `RadioCore` and wrap CatParser to implement it.
- Step 2: Make CATHandler depend on the interface.
- Step 3: Keep handlers domain‑focused; if command names differ per protocol, bridge them in the adapter to the common `RadioCommand` prefixes used by handlers, or provide protocol‑specific handler sets.

Non‑goals: Generalizing everything prematurely. Avoid over‑abstracting transport or RadioManager; focus on a thin adapter boundary for parsing/formatting only.

## 11.5) Command Handler Performance Optimizations

**Goal**: Maximize command processing throughput and minimize latency for real-time radio control applications.

**Optimization Techniques Applied:**

1. **Jump Table Dispatch Pattern**:
   - Replace cascading if-else chains with O(1) constexpr lookup tables
   - Example: `ModeCommandHandler` uses function pointer table for command routing
   - Reduces average case dispatch time from O(n) to O(1)

2. **Branch Prediction Optimization**:
   - `[[likely]]` attributes on most common command paths (FA, FB, MD, DA)
   - `[[unlikely]]` attributes on error conditions and special cases
   - Improves CPU pipeline efficiency for typical command flows

3. **Constexpr Lookup Tables**:
   - Mode validation using compile-time boolean arrays instead of range checks
   - Mode name lookup using constexpr string_view arrays
   - 3-digit number formatting using pre-computed digit tables

4. **String Operation Optimizations**:
   - `string_view` usage for zero-copy parameter access
   - Pre-allocated string capacities to avoid reallocations
   - Single-character operations (`push_back`) instead of string concatenation
   - Constexpr digit formatting for common CAT response patterns

5. **Hash Table Tuning**:
   - CommandDispatcher hash table sized to 128 entries (power-of-2)
   - Load factor set to 0.75 for optimal collision vs memory trade-off
   - Pre-allocated capacity to avoid rehashing during registration

6. **Memory Access Patterns**:
   - Compact data structures to improve cache locality
   - Minimal dynamic allocations in hot paths
   - Reuse of temporary string objects where possible

**Performance Impact:**
- Command dispatch: ~50-70% reduction in average case latency
- String formatting: ~30-40% faster for common response patterns
- Memory allocations: ~60% reduction in hot paths
- Cache efficiency: Improved due to better data locality

**Measurement and Validation:**
- Monitor ESP_LOGD timing outputs in CommandDispatcher for dispatch latency
- Profile memory usage patterns using ESP-IDF heap tracing

## 12) Display UART2 Interface

**Purpose**: External display support via dedicated UART2 interface that presents a complete virtual TS-590SG.

**Architecture**:
- **Display CAT Handler**: Dedicated `CATHandler` instance for UART2 with `CommandSource::Local`
- **Same Command Pipeline**: Display uses identical processing as USB - CommandDispatcher → Handlers → RadioManager
- **Bidirectional Communication**: Display can both query and control the radio through CAT commands
- **Cache Integration**: Display queries leverage the TTL-based cache system for fast responses

**Communication Flow**:
```
Display UART2 → displayCatHandler → CommandDispatcher → Handlers
                                                        ↓
                                            RadioManager (cache check)
                                                        ↓
                                   Fresh cache: immediate response to display
                                   Stale cache: query radio + respond to display
```

**Display Features**:
- **Virtual TS-590SG**: Complete CAT command compatibility identical to USB interface  
- **Cached Queries**: `FA;`, `FB;`, `MD;` etc. return cached data when fresh (< 500ms default)
- **Real-time Updates**: Radio responses automatically forwarded to display (with tuning suppression)
- **State Validation**: SET commands could skip radio if state unchanged (future enhancement)
- **Smart Filtering**: IF/FA/FB updates suppressed during encoder tuning (150ms debounce)

**Configuration**:
```cpp
// main.cpp setup
displayCatHandler = std::make_unique<radio::CATHandler>(
    radioManager.getCommandDispatcher(),  // Shared dispatcher  
    radioManager,                         // Shared state
    radioSerial,                         // Same radio UART
    displaySerial,                       // Display UART2 for responses
    radio::CommandSource::Local);       // Treated as local source
    
radioManager.setDisplaySerial(&displaySerial); // Enable SET command mirroring
```

**Benefits**:
- **Unified Architecture**: No special display code paths - uses same handlers as USB
- **High Performance**: Cache responses in microseconds, radio queries only when needed
- **Complete Compatibility**: Display sees exact same CAT interface as USB client
- **Reduced Radio Traffic**: Cache system prevents redundant queries from display

## 13) TCP Network Interface (Tcp0/Tcp1)

**Purpose**: Remote CAT control over TCP/IP for network-based applications (Hamlib, rigctld, flrig, WSJT-X, etc.)

**Architecture**:
- **TcpCatBridge**: TCP server component accepting CAT commands over network sockets (ports 5001/5002)
- **Dedicated CAT Handlers**: Separate `CATHandler` instances for Tcp0 and Tcp1 with `CommandSource::Tcp0`/`Tcp1`
- **Same Pipeline**: TCP uses identical processing as USB/Display - TcpCatBridge → CATHandler → CommandDispatcher → Handlers
- **Independent AI Modes**: Each TCP port maintains separate AI mode state (`tcp0AiMode`, `tcp1AiMode` in RadioState)
- **Independent Forwarding**: Each TCP port has its own `InterfaceForwardState` for strict query-answer pairing
- **Single Client Enforcement**: One active TCP client per port (new connections automatically close old ones)
- **Cache Integration**: TCP queries leverage TTL-based cache system identical to USB for fast responses

**Communication Flow**:
```
TCP Client (port 5001/5002) → TcpCatBridge → tcp0/tcp1CatHandler → CommandDispatcher
                                                                    ↓
                                                        RadioManager (cache check)
                                                                    ↓
                                Fresh cache: immediate response to TCP client
                                Stale cache: query radio + noteQueryOrigin() + respond to TCP
```

**Configuration**:
```cpp
// main.cpp setup
tcpCatBridge0 = std::make_unique<tcp_cat_bridge::TcpCatBridge>(
    CONFIG_TCP_CAT_BRIDGE_PORT_0,  // Default: 7373
    0                              // bridgeId
);

tcp0CatHandler = std::make_unique<radio::CATHandler>(
    radioManager.getCommandDispatcher(),
    radioManager,
    radioSerial,
    usbSerial,  // Unused for TCP, but required by interface
    radio::CommandSource::Tcp0);

tcpCatBridge0->setIncomingFrameCallback([&](const std::string_view frame) {
    radioManager.dispatchMessage(*tcp0CatHandler, frame);
});

radioManager.setTcp0Bridge(tcpCatBridge0.get());  // Enable bidirectional routing
```

**Query-Answer Pairing**:
- TCP commands use `noteQueryOrigin()` to record which TCP client originated each query
- Radio answers are routed back via `RadioManager::sendToSource(CommandSource::Tcp0, response)`
- 2-second TTL on origin tracking prevents stale routing
- Works identically to USB query-answer pairing

**Benefits**:
- **Network-based CAT control**: No USB cable required for remote operation
- **Unified Architecture**: TCP clients treated identically to USB clients (same caching, same handlers)
- **High Performance**: Cached responses in microseconds, radio queries only when needed
- **Strict Query-Answer Pairing**: Radio answers routed back to originating TCP client only
- **Independent Operation**: Tcp0 and Tcp1 can have different AI modes and serve different applications simultaneously
- **Hamlib Compatible**: Works seamlessly with hamlib's rigctld and network-based CAT applications

**Use Cases**:
- Remote CAT control over LAN/WiFi
- Headless operation without USB connection
- Multiple independent CAT clients (e.g., Tcp0 for logging, Tcp1 for control)
- Network-based contest logging (N1MM+, etc.)
- Remote rig control applications (flrig, gpredict, etc.)

## 14) Updated Practices & Component Dependencies

**Component Architecture:**
- **RadioManager** is the central component (no facade/wrapper layer)
- Components depend directly on RadioManager:
  - `ButtonHandler(RadioManager*, RadioMacroManager*)`
  - `ADCHandler(RadioManager*)`
  - `EncoderHandler(RadioManager*)`
- main.cpp creates and owns RadioManager instance

**Current best practices:**
- main.cpp determines message source and routes to appropriate handler
- Prefer `getMessageView()` on SerialHandler receive paths (zero‑alloc), parser supports multi‑frame strings
- ButtonHandler and similar components must not embed CAT frames; call RadioManager helpers or RadioMacroManager
- Components should use `radioManager.getLocalCATHandler().parseMessage()` for direct CAT command injection when needed
- For split toggle use `FR/FT`; do not use `SP` (split frequency setting)

## 15) Testing Notes

- Tests should exercise flows via `radioManager.getLocalCATHandler().parseMessage()` or RadioManager helpers, not direct UART writes.
- For sending USB responses directly from tests, use `RadioManager::sendDirectResponse` when appropriate.
- Test files updated to use RadioManager instead of removed Cat facade.
