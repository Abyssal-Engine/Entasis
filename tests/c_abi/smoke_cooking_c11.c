#include "entasis_cooking.h"

int main(void)
{
    return entasis_cooking_abi_version() == ENTASIS_COOKING_ABI_VERSION ? 0 : 1;
}
