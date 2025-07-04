# CommandHandlers

Modular CAT command handlers implementing radio functionality for Kenwood TS-590SG.
I wanted to try to group commands into logical "categories", this is what gave birth to the CommandHandlers basically.
Is it great? Maybe not, but I wanted to support _all_ of the TS-590SG CAT commands and this was a good way to do so
without huge classes.

## Purpose

Provides specialized handlers for different CAT command categories. Each handler processes specific command prefixes and manages corresponding radio functionality.

## Architecture

All handlers implement `ICommandHandler` interface:
- `canHandle()`: Check if handler supports command
- `handleCommand()`: Process the command
- `getPrefixes()`: Return supported 2-char CAT prefixes

Base class `BaseCommandHandler` provides common functionality for CAT response generation.

## Handler Categories

### Frequency & VFO
**FrequencyVfoCommandHandler**: FA, FB, FR, FT, VV, VS, FN, FC, VX
- VFO A/B frequency control
- RX/TX VFO selection
- Visual scan, sub-RX, transverter offset

### Mode & Filters
**ModeCommandHandler**: MD, DA, FW, SL, SH, FI, IS
- Operating mode (SSB, CW, AM, FM, FSK)
- Data mode, filter width, IF shift

### Transmitter
**TransmitterCommandHandler**: TX, RX, MX, KS, PC
- PTT control, MOX, keyer speed, power level

### Receiver
**ReceiverProcessingCommandHandler**: AG, RG, MG, SQ, NB, NR, BC, NL
- AF/RF/Mic gain, squelch, noise reduction, beat cancel

### Audio & CW
**AudioEqualizerCommandHandler**: EQ
**CwCommandHandler**: KY, BK, BT, RA, RB, RC, RM, RT, TB

### Memory & Scan
**MemoryCommandHandler**: MC, MR, MW, MN
**ScanCommandHandler**: SC, CT, TS
**VisualScanCommandHandler**: VS, DD

### Interface & Status
**InterfaceSystemCommandHandler**: AI, PS, ID, IF, MD
**StatusInfoCommandHandler**: SM, RM, AC, TY, TN, RA, ID

### Configuration
**MenuConfigCommandHandler**: EX, MF
**ExtendedCommandHandler**: EX (extended menu items)

### Specialized
**AntennaCommandHandler**: AN
**ToneSquelchCommandHandler**: TN, CT, TO
**GainLevelCommandHandler**: AG, RG, MG, VG, SQ
**UICommandHandler**: UI elements and display

## Usage

```cpp
// Handlers registered with CommandDispatcher
auto dispatcher = std::make_unique<radio::CommandDispatcher>(radioManager);
dispatcher->registerHandler(std::make_unique<FrequencyVfoCommandHandler>());
dispatcher->registerHandler(std::make_unique<ModeCommandHandler>());
// ... etc

// Dispatcher routes commands to appropriate handler
dispatcher->dispatch(command, radioSerial, usbSerial);
```

## Testing

Comprehensive test coverage in `test/`:
- `test_consolidated_command_handlers.cpp`: Main handler tests
- `test_transverter_features.cpp`: Transverter-specific tests

## Dependencies

- RadioCommand, RadioState, RadioManager
- ISerialChannel for serial I/O