// Filters Unity output so PASS lines are hidden while preserving FAIL/summary

#include <cctype>
#include <cstdio>
#include <string>

extern "C" {
    // Unity will call this for every character it prints when UNITY_OUTPUT_CHAR is set
    int capture_unity_char(int c);
}

// Buffer a single line of Unity output until newline
static std::string g_unity_line_buf;

static void flush_line_filtered(const std::string& line) {
    // Hide per-test PASS lines: they contain ":PASS" in Unity's default format
    if (line.find(":PASS") != std::string::npos) {
        return; // suppress
    }
    // Otherwise, forward the line to stdout unchanged
    for (char ch : line) {
        std::putchar(static_cast<unsigned char>(ch));
    }
    std::putchar('\n');
    std::fflush(stdout);
}

extern "C" int capture_unity_char(int c) {
    // Normalize CR to LF handling; treat both as end-of-line
    if (c == '\n' || c == '\r') {
        if (!g_unity_line_buf.empty()) {
            flush_line_filtered(g_unity_line_buf);
            g_unity_line_buf.clear();
        }
        return c;
    }

    // Accumulate characters until end of line
    g_unity_line_buf.push_back(static_cast<char>(c));
    return c;
}

