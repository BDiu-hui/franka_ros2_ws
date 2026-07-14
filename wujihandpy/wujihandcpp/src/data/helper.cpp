
#include <cctype>
#include <cstddef>

#include <spdlog/fmt/fmt.h>

#include <wujihandcpp/data/helper.hpp>

#include "wujihandcpp/utility/api.hpp"

namespace wujihandcpp::data {

WUJIHANDCPP_API size_t FirmwareVersionData::string_length() const {
    if (pre == '~')
        return fmt::formatted_size("{}.{}.{}", major, minor, patch);
    else if (auto upper = std::toupper(static_cast<unsigned char>(pre));
             'A' <= upper && upper <= 'Z')
        return fmt::formatted_size("{}.{}.{}-rc{}", major, minor, patch, int(upper - 'A'));
    else
        return fmt::formatted_size("{}.{}.{}-{}", major, minor, patch, int(pre));
}

WUJIHANDCPP_API void FirmwareVersionData::write_to_string(char* dst) const {
    if (pre == '~')
        fmt::format_to(dst, "{}.{}.{}", major, minor, patch);
    else if (auto upper = std::toupper(static_cast<unsigned char>(pre));
             'A' <= upper && upper <= 'Z')
        fmt::format_to(dst, "{}.{}.{}-rc{}", major, minor, patch, int(upper - 'A'));
    else
        fmt::format_to(dst, "{}.{}.{}-{}", major, minor, patch, int(pre));
}

} // namespace wujihandcpp::data
