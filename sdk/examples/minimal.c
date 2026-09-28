#include <entasis/entasis.h>
#include <entasis/cooking.h>

#include <string.h>
#include <stdio.h>

int main(void)
{
    if (entasis_abi_version() != ENTASIS_ABI_VERSION)
        return 1;
    if (entasis_cooking_abi_version() != ENTASIS_COOKING_ABI_VERSION)
        return 2;

    const entasis_version_t version = entasis_version_current();
    if (version.major != ENTASIS_VERSION_MAJOR || version.minor != ENTASIS_VERSION_MINOR ||
        version.patch != ENTASIS_VERSION_PATCH ||
        version.prerelease.count != strlen(ENTASIS_VERSION_PRERELEASE) ||
        (version.prerelease.count != 0u &&
         memcmp(version.prerelease.data, ENTASIS_VERSION_PRERELEASE,
                (size_t)version.prerelease.count) != 0))
    {
        fputs("Entasis header/library product version mismatch\n", stderr);
        return 5;
    }

    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    const entasis_world_description_t description = entasis_world_description_default();
    if (entasis_world_init(&world, &description, &diagnostic) != ENTASIS_STATUS_OK)
        return 3;
    return entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK ? 0 : 4;
}
