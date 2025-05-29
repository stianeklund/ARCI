#include "ParserUtils.h"
#include <algorithm>
#include <string_view>

namespace radio::cat {

    bool ParserUtils::vectorContains(const std::vector<std::string>& vec, const char* str) {
        // Use string_view comparison to avoid temporary std::string allocation
        const std::string_view target{str};
        return std::any_of(vec.begin(), vec.end(),
            [&target](const std::string& s) { return s == target; });
    }

} // namespace radio::cat