#+vet !unused-imports

package entasis_c

import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	abi_custom_constraint_register_position_space :: proc "contextless" (
		world: ^Entasis_World,
		registration: ^Entasis_Custom_Constraint_Registration,
		position_space: Entasis_Constraint_Position_Space,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		return abi_custom_constraint_register(world, registration, diagnostic, u8(position_space));
	}
	
	abi_constraint_set_reaction_provider_position_space :: proc "contextless" (
		world: ^Entasis_World,
		type_id: Entasis_Constraint_Type_ID,
		provider: ^Entasis_Joint_Reaction_Provider,
		position_space: Entasis_Constraint_Position_Space,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		return abi_constraint_set_reaction_provider(world, type_id, provider, diagnostic, u8(position_space));
	}
}
