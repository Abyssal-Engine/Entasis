#include <entasis/entasis.h>
#include <entasis/cooking.h>
#include "generated_layout_asserts.h"

int entasis_include_cpp20_translation_unit()
{
    constexpr entasis_version_t version = ENTASIS_CURRENT_VERSION;
    return static_cast<int>(version.major);
}
