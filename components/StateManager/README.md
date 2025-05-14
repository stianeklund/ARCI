# StateManager

Thread-safe radio state data model providing the single source of truth for all radio parameters.

## Overview

The StateManager component provides `RadioState`, a cache-line-optimized struct containing all radio state information. It uses `std::atomic` for thread-safe access from multiple FreeRTOS tasks without explicit locking on hot paths.

## Key Features

- **Cache-line aligned** - Hot-path fields (VFO frequencies, mode, TX state) grouped in first 64 bytes
- **Lock-free reads** - Most fields use `std::atomic` with relaxed memory ordering
- **Command timestamp tracking** - O(1) lookup for TTL-based caching via `CommandTimestampTracker`
- **TX ownership management** - Mutual exclusion with timeout protection
- **UI state tracking** - Panel-display overlay interactions (power slider, etc.)

## State Categories

| Category | Fields | Access Pattern |
|----------|--------|----------------|
| Hot path | VFO frequencies, mode, TX state | Every CAT command |
| Warm path | RIT/XIT, AI modes, split | Frequent updates |
| Cold path | Band, antenna, audio settings | Rare changes |

## Thread Safety

```cpp
// Safe from any task:
uint64_t freq = state.vfoAFrequency.load(std::memory_order_relaxed);

// TX ownership requires mutex:
bool acquired = state.tryAcquireTx(source, currentTime);
```

## Dependencies

None (foundation component)
