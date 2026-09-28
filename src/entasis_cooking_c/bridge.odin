package entasis_cooking_c

import entasis "entasis:entasis"

abi_cooking_version :: #force_inline proc "contextless" () -> u32
{
	return entasis.ABI_VERSION;
}
