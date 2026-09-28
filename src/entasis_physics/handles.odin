// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

Body_Handle :: struct
{
	value: i32,
}

Static_Handle :: struct
{
	value: i32,
}

Constraint_Handle :: struct
{
	value: i32,
}

Body_Memory_Location :: struct
{
	set_index: i32,
	index:     i32,
}

Physics_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Invalid_Description,
	Not_Found,
	Capacity_Missing,
	Shape_In_Use,
	Disposed,
	No_Convergence,
}

Reference_State :: enum u8
{
	Missing,
	Present,
}

Body_Set_State :: enum u8
{
	Unallocated,
	Allocated,
}

Sleep_Candidate_State :: enum u8
{
	Not_Candidate,
	Candidate,
}

Body_Mobility :: enum u8
{
	Dynamic,
	Kinematic,
	Static,
}

body_handle_invalid :: proc "contextless" () -> Body_Handle
{
	return {-1};
}

static_handle_invalid :: proc "contextless" () -> Static_Handle
{
	return {-1};
}

constraint_handle_invalid :: proc "contextless" () -> Constraint_Handle
{
	return {-1};
}
#assert(size_of(Body_Handle) == 4);
#assert(size_of(Static_Handle) == 4);
#assert(size_of(Constraint_Handle) == 4);
#assert(size_of(Body_Memory_Location) == 8);
