# Test Structure Overview

## Current Test Architecture

The test system is now organized by component with dedicated test suites for each major functionality area. Tests use Unity framework and are executed via `idf.py test`.

## Test Components

### 1. CatParser Tests (`components/CatParser/test/`)
**Files**: `test_cat_parser.cpp`, `test_cat_parser_utils.cpp`
**Purpose**: Validate CAT command parsing logic without system complexity

**Tests**:
- Command parsing (extracting command and parameters)
- Parameter extraction and validation (frequencies, modes, etc.)
- Error handling for malformed commands
- Parser utility functions
- Boundary conditions validation

**Key Characteristic**: Pure parsing logic testing, no RadioManager dependencies

### 2. CommandHandlers Tests (`components/CommandHandlers/test/`)
**Files**: `test_consolidated_command_handlers.cpp`, `test_transverter_features.cpp`
**Purpose**: Validate individual command handler behavior

**Tests**:
- Command handler implementations
- Parameter validation within handlers
- Transverter-specific functionality
- Handler routing and dispatch logic

**Key Characteristic**: Tests command handlers in isolation with mock dependencies

### 3. RadioCore Tests (`components/RadioCore/test/`)
**Files**: 
- `test_cache_behavior.cpp` - Cache-specific behaviors
- `test_cache_system.cpp` - Cache system functionality
- `test_radiocat_handler.cpp` - CAT handler behavior
- `test_radiomanager_cat_commands.cpp` - RadioManager CAT integration
- `test_unified_architecture.cpp` - Architecture validation
- `test_unified_cat_system.cpp` - Complete CAT system testing
- `test_meter_polling_performance.cpp` - Performance testing

**Purpose**: Validate core radio management and CAT system integration

**Tests**:
- Complete CAT command flows
- Cache behavior and TTL management
- RadioManager integration
- System performance characteristics
- Architecture compliance

**Key Characteristic**: End-to-end system testing with full RadioManager

### 4. Additional Component Tests
- **ButtonHandler** (`test_button_handler.cpp`) - Button input handling
- **EncoderHandler** (`test_encoder_handler.cpp`) - Rotary encoder functionality  
- **RadioMacroManager** (`test_radio_macro_manager.cpp`) - Macro system

## Test Utilities (`CATTestUtils.h`)

### Helper Functions
- `assertForwardedToRadio()` - Verify command sent to radio
- `assertSentToUSB()` - Verify response sent to USB
- `assertNotForwardedToRadio()` - Verify no radio communication
- `assertCacheUsed()` - Verify cache was used instead of radio query
- `clearMockSerials()` - Reset mock serial handlers

### Test Macros
- `TEST_CAT_SET_FLOW()` - Test complete SET command flow
- `TEST_CAT_QUERY_CACHED()` - Test cached query response
- `TEST_CAT_QUERY_RADIO()` - Test query that goes to radio
- `TEST_ASSERT_FREQUENCY_EQUAL()` - Handle 64-bit frequency comparisons

## Understanding Local vs Remote Handlers

### Local CAT Handler (`getLocalCATHandler()`)
- Processes commands from USB/PC software
- For queries: Responds from cache if fresh, otherwise queries radio
- For SET commands: Forwards to radio
- Source: `CommandSource::Local`

### Remote CAT Handler (`getRemoteCATHandler()`)
- Processes responses from physical radio
- Updates RadioManager's internal state
- For special commands (like IF): Forwards to USB
- Never sends back to radio (prevents loops)
- Source: `CommandSource::Remote`

## Test Patterns

### Testing SET Commands
```cpp
// 1. Send SET command via local handler
testRadioManager->getLocalCATHandler().parseMessage("FA00014150000;");

// 2. Verify forwarded to radio
TEST_ASSERT_EQUAL_STRING("FA00014150000;", mockRadioSerial.lastSent());

// 3. Simulate radio acknowledgment
testRadioManager->getRemoteCATHandler().parseMessage("FA00014150000;");

// 4. Verify state updated
TEST_ASSERT_EQUAL(14150000ULL, testRadioManager->getVfoAFrequency());
```

### Testing Cached Queries
```cpp
// 1. Populate cache via remote response
testRadioManager->getRemoteCATHandler().parseMessage("FA00014150000;");

// 2. Query via local handler
testRadioManager->getLocalCATHandler().parseMessage("FA;");

// 3. Verify cache used (no radio query)
TEST_ASSERT_TRUE(mockRadioSerial.sentMessages.empty());

// 4. Verify USB response
TEST_ASSERT_EQUAL_STRING("FA00014150000;", mockUsbSerial.lastSent());
```

### Testing Cache Expiration
```cpp
// 1. Populate cache
testRadioManager->getRemoteCATHandler().parseMessage("FA00014150000;");

// 2. Advance time beyond TTL
cacheManager.advanceTime(5000);

// 3. Query should now go to radio
testRadioManager->getLocalCATHandler().parseMessage("FA;");
TEST_ASSERT_FALSE(mockRadioSerial.sentMessages.empty());
```

## Benefits of This Structure

1. **Clear Intent**: Each test file has a specific, well-defined purpose
2. **Isolated Testing**: Can test parsing without RadioManager complexity
3. **Better Coverage**: Explicit tests for cache behavior, error handling
4. **Maintainable**: Changes to routing logic don't break parser tests
5. **Debuggable**: Test failures pinpoint exact layer with issues
6. **Scalable**: Easy to add new test categories or expand existing ones

## Running Tests

To run all tests:
```bash
idf.py test
```

To run specific component tests:
```bash
idf.py test -T unit_test  # All tests via the unit_test component
```

Individual test filtering can be done via Unity's built-in test filtering mechanisms.

## Current Test Coverage

### Component Coverage:
- [x] **CatParser**: Parser logic and utilities
- [x] **CommandHandlers**: Handler implementations and transverter features
- [x] **RadioCore**: Complete CAT system, cache behavior, performance
- [x] **ButtonHandler**: Input handling
- [x] **EncoderHandler**: Rotary encoder functionality
- [x] **RadioMacroManager**: Macro system
- [ ] **SerialHandler**: Has mocks but no dedicated tests
- [ ] **StateManager**: No dedicated tests (tested via integration)
- [ ] **Audio, NvsManager, UsbCdc**: No test coverage

### Test Types:
- **Unit Tests**: Individual component functionality
- **Integration Tests**: Cross-component interactions  
- **Performance Tests**: Cache and polling performance
- **Mock-based Tests**: Isolated component testing

## Adding New Tests

When adding new functionality:

1. **For new CAT commands**: Add parser tests in CatParser, handler tests in CommandHandlers, integration tests in RadioCore
2. **For new components**: Create dedicated test directory with CMakeLists.txt
3. **For performance concerns**: Add specific performance tests in RadioCore
4. **Always**: Update unit_test/CMakeLists.txt to include new test files

## Architecture Benefits

1. **Component Isolation**: Each component has focused, independent tests
2. **Clear Separation**: Parser logic tested separately from system integration
3. **Mock Dependencies**: Components tested in isolation using MockSerialHandler
4. **Performance Validation**: Dedicated performance tests for critical paths
5. **Maintainable**: Component-based organization makes test maintenance easier

## Common Pitfalls to Avoid

1. **Component Boundaries**: Keep component tests focused on their specific responsibilities
2. **Mock Usage**: Use MockSerialHandler appropriately for isolation testing
3. **Integration Testing**: Ensure end-to-end flows are tested in RadioCore integration tests
4. **Test Dependencies**: Update unit_test/CMakeLists.txt when adding new test files
5. **Performance Awareness**: Consider performance implications, especially for cache and polling operations