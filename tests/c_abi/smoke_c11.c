#include "entasis.h"

#include <stdint.h>
#include <string.h>

static int view_equals(entasis_string_view_t view, const char *expected)
{
    const size_t expected_count = strlen(expected);
    return view.count == (uint64_t)expected_count &&
           (expected_count == 0u || memcmp(view.data, expected, expected_count) == 0);
}

int main(void)
{
    if (entasis_abi_version() != ENTASIS_ABI_VERSION)
        return 1;

    const entasis_version_t version = entasis_version_current();
    if (version.major != ENTASIS_VERSION_MAJOR ||
        version.minor != ENTASIS_VERSION_MINOR ||
        version.patch != ENTASIS_VERSION_PATCH ||
        !view_equals(version.prerelease, ENTASIS_VERSION_PRERELEASE))
        return 2;

    if (!entasis_status_ok(ENTASIS_STATUS_OK))
        return 3;
    if (entasis_status_failed(ENTASIS_STATUS_OK))
        return 4;
    if (!entasis_status_failed(ENTASIS_STATUS_INVALID_ARGUMENT))
        return 5;
    if (!view_equals(entasis_status_text(ENTASIS_STATUS_CAPACITY_MISSING), "capacity missing"))
        return 6;
    if (!view_equals(entasis_status_text(UINT8_C(255)), "unknown status"))
        return 7;

    entasis_diagnostic_t diagnostic = {
        ENTASIS_STATUS_INVALID_ARGUMENT,
        ENTASIS_DIAGNOSTIC_OPERATION_ASSET_IMPORT,
        {UINT8_C(0), UINT8_C(0)},
        INT32_C(7)};
    entasis_diagnostic_clear(&diagnostic);
    if (diagnostic.status != ENTASIS_STATUS_OK || diagnostic.operation != 0u || diagnostic.detail != 0)
        return 8;
    entasis_diagnostic_record(
        &diagnostic,
        ENTASIS_STATUS_NOT_FOUND,
        ENTASIS_DIAGNOSTIC_OPERATION_SHAPE_REGISTER,
        INT32_C(93));
    if (diagnostic.status != ENTASIS_STATUS_NOT_FOUND ||
        diagnostic.operation != ENTASIS_DIAGNOSTIC_OPERATION_SHAPE_REGISTER ||
        diagnostic.detail != 93)
        return 9;
    entasis_diagnostic_clear(NULL);
    entasis_diagnostic_record(NULL, ENTASIS_STATUS_OK, ENTASIS_DIAGNOSTIC_OPERATION_NONE, 0);

    const entasis_body_handle_t body_invalid = entasis_body_handle_invalid();
    const entasis_static_handle_t static_invalid = entasis_static_handle_invalid();
    const entasis_constraint_handle_t constraint_invalid = entasis_constraint_handle_invalid();
    const entasis_shape_handle_t shape_invalid = entasis_shape_handle_invalid();
    if (body_invalid.value != -1 || entasis_body_handle_is_valid(body_invalid))
        return 10;
    if (static_invalid.value != -1 || entasis_static_handle_is_valid(static_invalid))
        return 11;
    if (constraint_invalid.value != -1 || entasis_constraint_handle_is_valid(constraint_invalid))
        return 12;
    if (shape_invalid.packed != 0u || entasis_shape_handle_is_valid(shape_invalid))
        return 13;
    if (!entasis_body_handle_is_valid((entasis_body_handle_t){INT32_C(0)}))
        return 14;
    if (!entasis_static_handle_is_valid((entasis_static_handle_t){INT32_C(0)}))
        return 15;
    if (!entasis_constraint_handle_is_valid((entasis_constraint_handle_t){INT32_C(0)}))
        return 16;
    if (!entasis_shape_handle_is_valid((entasis_shape_handle_t){UINT32_C(0x80000000)}))
        return 17;

    return 0;
}
