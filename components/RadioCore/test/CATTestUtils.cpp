#include "include/CATTestUtils.h"
#include <cstdlib>
#include <algorithm>
#include <cstring>

namespace test_utils {

uint64_t arrayToUint64(const std::array<char, 12>& arr) {
    const auto end_it = std::ranges::find(arr, '\0');
    const size_t length = std::distance(arr.begin(), end_it);
    if (length == 0) return 0;

    // Create null-terminated string
    char buffer[13];
    std::memcpy(buffer, arr.data(), length);
    buffer[length] = '\0';
    
    return static_cast<uint64_t>(std::atoll(buffer));
}

uint32_t arrayToUint32(const std::array<char, 12>& arr) {
    const auto end = std::ranges::find(arr, '\0');
    size_t length = std::distance(arr.begin(), end);
    if (length == 0) return 0;

    // Create null-terminated string
    char buffer[13];
    std::memcpy(buffer, arr.data(), length);
    buffer[length] = '\0';
    
    return static_cast<uint32_t>(std::atol(buffer));
}

uint8_t arrayToUint8(const std::array<char, 9>& arr) {
    const auto end_it = std::ranges::find(arr, '\0');
    const size_t length = std::distance(arr.begin(), end_it);
    if (length == 0) return 0;

    // Create null-terminated string
    char buffer[10];
    std::memcpy(buffer, arr.data(), length);
    buffer[length] = '\0';
    
    return static_cast<uint8_t>(std::atoi(buffer));
}

} // namespace test_utils