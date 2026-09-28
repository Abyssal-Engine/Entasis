#include <entasis/entasis.h>
#include <entasis/cooking.h>
#include "generated_layout_asserts.h"

int entasis_include_c11_translation_unit(void)
{
    const entasis_version_t version = ENTASIS_CURRENT_VERSION;
    return (int)version.major;
}
