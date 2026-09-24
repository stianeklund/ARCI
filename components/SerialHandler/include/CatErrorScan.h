#pragma once

#include <cstddef>
#include <cstdint>

// Kenwood CAT error replies seen in a UART RX chunk.
struct CatErrorScan {
    bool eoError{false};        // bare "E;" or "O;" frame
    bool questionError{false};  // bare "?;" frame
};

// Kenwood error replies are whole frames ("E;", "O;", "?;"), so the error char
// only counts when it starts a frame: preceded by ';', CR/LF/whitespace, or the
// start of the stream. This keeps legitimate frames ending in E/O (e.g. "TO;")
// from being flagged.
//
// prevByte is the last byte of the previous RX chunk, so a frame split across
// chunks ("T" + "O;") is judged correctly; pass ';' for the start of the stream.
// Limitation: an error frame split between its char and ';' ("E" + ";") is missed.
constexpr bool isCatFrameBoundary(uint8_t b) {
    return b == ';' || b == '\r' || b == '\n' || b == ' ' || b == '\t';
}

constexpr CatErrorScan scanCatErrorFrames(const uint8_t* data, size_t len, uint8_t prevByte) {
    CatErrorScan result{};
    for (size_t i = 0; i + 1 < len; i++) {
        if (data[i + 1] != ';') continue;
        const uint8_t before = (i == 0) ? prevByte : data[i - 1];
        if (!isCatFrameBoundary(before)) continue;
        if (data[i] == 'E' || data[i] == 'O') result.eoError = true;
        if (data[i] == '?') result.questionError = true;
    }
    return result;
}
