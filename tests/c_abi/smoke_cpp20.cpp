#include "entasis.h"

#include <cstdint>
#include <string_view>
#include <cstring>

int main()
{
    if (entasis_abi_version() != ENTASIS_ABI_VERSION)
        return 1;

    const entasis_version_t version = entasis_version_current();
    if (version.major != ENTASIS_VERSION_MAJOR ||
        version.minor != ENTASIS_VERSION_MINOR || version.patch != ENTASIS_VERSION_PATCH ||
        version.prerelease.count != std::strlen(ENTASIS_VERSION_PRERELEASE) ||
        (version.prerelease.count != 0u &&
         std::memcmp(version.prerelease.data, ENTASIS_VERSION_PRERELEASE,
                     static_cast<std::size_t>(version.prerelease.count)) != 0))
        return 2;

    const entasis_string_view_t status = entasis_status_text(ENTASIS_STATUS_INVALID_DESCRIPTION);
    const std::string_view text{status.data, static_cast<std::size_t>(status.count)};
    if (text != "invalid description")
        return 3;

    const entasis_body_handle_t invalid = entasis_body_handle_invalid();
    if (invalid.value != -1 || entasis_body_handle_is_valid(invalid))
        return 4;
    if (!entasis_body_handle_is_valid(entasis_body_handle_t{0}))
        return 5;

    entasis_diagnostic_t diagnostic{};
    entasis_diagnostic_record(
        &diagnostic,
        ENTASIS_STATUS_DISPOSED,
        ENTASIS_DIAGNOSTIC_OPERATION_WORLD_INITIALIZE,
        11);
    if (diagnostic.status != ENTASIS_STATUS_DISPOSED || diagnostic.detail != 11)
        return 6;
    return 0;
}
