# CatParser

Unified CAT (Computer Aided Transceiver) protocol parser for Kenwood TS-590SG commands.

## Purpose

Parses CAT commands from both local (USB) and remote (radio) sources into structured `RadioCommand` objects. Replaces separate local/remote parsers with single unified implementation.

## Features

- Dual/Triple-source parsing (Local: USB & TCP, Remote from radio / RemoteRig)
- Command type inference (SET, READ, ANSWER)
- Multi-command message support (split on `;`)
- Parse error tracking with statistics
- Zero-copy string parsing with `string_view`

## Usage

```cpp
radio::CatParser parser;
parser.setErrorCallback([](std::string_view err) {
    ESP_LOGE(TAG, "Parse error: %.*s", err.size(), err.data());
});

parser.parseMessage("FA14150000;MD2;", radio::CommandSource::Local,
    [](const radio::RadioCommand& cmd) {
        // Process each parsed command
    });

// Statistics
auto stats = parser.getStatistics();
ESP_LOGI(TAG, "Parsed: %zu, Errors: %zu", stats.totalMessagesParsed, stats.parseErrors);
```

## Command Format

- `FAxxxxxxxxxx;` - SET frequency
- `FA;` - READ frequency query
- `FA14150000;` from radio - ANSWER

## Dependencies

- `RadioCommand.h`: Command structure definition
- `ParserUtils.h`: String parsing utilities

## Testing

See `test/test_cat_parser.cpp` for unit tests.