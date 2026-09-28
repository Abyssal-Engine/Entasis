// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

TYPED_INDEX_EXISTS_MASK :: u32(1 << 31);
TYPED_INDEX_TYPE_MASK :: u32(0x7f00_0000);
TYPED_INDEX_INDEX_MASK :: u32(0x00ff_ffff);

Typed_Index :: struct
{
	packed: u32,
}

typed_index_create :: proc "contextless" (type_id, index: int) -> (Typed_Index, Physics_Status)
{
	if type_id < 0 || type_id >= 128 || index < 0 || index >= 1 << 24
	{
		return {}, .Invalid_Argument;
	}
	return {u32(type_id << 24 | index) | TYPED_INDEX_EXISTS_MASK}, .Ok;
}

typed_index_state :: proc "contextless" (typed_index: Typed_Index) -> Reference_State
{
	if typed_index.packed & TYPED_INDEX_EXISTS_MASK != 0
	{
		return .Present;
	}
	return .Missing;
}

typed_index_type :: proc "contextless" (typed_index: Typed_Index) -> i32
{
	return i32((typed_index.packed & TYPED_INDEX_TYPE_MASK) >> 24);
}

typed_index_index :: proc "contextless" (typed_index: Typed_Index) -> i32
{
	return i32(typed_index.packed & TYPED_INDEX_INDEX_MASK);
}

Continuous_Detection_Mode :: enum u8
{
	Discrete,
	Passive,
	Continuous,
}

Continuous_Detection :: struct
{
	mode:                        Continuous_Detection_Mode,
	_padding:                    [3]u8,
	minimum_sweep_timestep:      f32,
	sweep_convergence_threshold: f32,
}

Collidable_Description :: struct
{
	shape:                        Typed_Index,
	continuity:                   Continuous_Detection,
	minimum_speculative_margin:   f32,
	maximum_speculative_margin:   f32,
}

Collidable :: struct
{
	shape:                      Typed_Index,
	continuity:                 Continuous_Detection,
	minimum_speculative_margin: f32,
	maximum_speculative_margin: f32,
	speculative_margin:         f32,
	broad_phase_index:          i32,
}

Collidable_Reference :: struct
{
	packed: u32,
}

collidable_reference_create :: proc "contextless" (
	mobility: Body_Mobility, raw_handle_value: int,
) -> (Collidable_Reference, Physics_Status)
{
	if mobility > .Static || raw_handle_value < 0 || raw_handle_value >= 1 << 30
	{
		return {}, .Invalid_Argument;
	}
	return {u32(mobility) << 30 | u32(raw_handle_value)}, .Ok;
}

collidable_reference_body :: proc "contextless" (
	mobility: Body_Mobility, handle: Body_Handle,
) -> (Collidable_Reference, Physics_Status)
{
	if mobility != .Dynamic && mobility != .Kinematic
	{
		return {}, .Invalid_Argument;
	}
	return collidable_reference_create(mobility, int(handle.value));
}

collidable_reference_static :: proc "contextless" (handle: Static_Handle) -> (Collidable_Reference, Physics_Status)
{
	return collidable_reference_create(.Static, int(handle.value));
}

collidable_reference_mobility :: proc "contextless" (reference: Collidable_Reference) -> Body_Mobility
{
	return Body_Mobility(reference.packed >> 30);
}

collidable_reference_raw_handle :: proc "contextless" (reference: Collidable_Reference) -> i32
{
	return i32(reference.packed & 0x3fff_ffff);
}

collidable_reference_body_handle :: proc "contextless" (reference: Collidable_Reference) -> (Body_Handle, Physics_Status)
{
	mobility := collidable_reference_mobility(reference);
	if mobility != .Dynamic && mobility != .Kinematic
	{
		return body_handle_invalid(), .Invalid_Argument;
	}
	return {collidable_reference_raw_handle(reference)}, .Ok;
}

collidable_reference_static_handle :: proc "contextless" (reference: Collidable_Reference) -> (Static_Handle, Physics_Status)
{
	if collidable_reference_mobility(reference) != .Static
	{
		return static_handle_invalid(), .Invalid_Argument;
	}
	return {collidable_reference_raw_handle(reference)}, .Ok;
}

continuous_detection_discrete :: proc "contextless" () -> Continuous_Detection
{
	return {};
}
continuous_detection_passive :: proc "contextless" () -> Continuous_Detection
{
	return {mode=.Passive};
}
continuous_detection_continuous :: proc "contextless" (
	minimum_sweep_timestep: f32 = 1e-3, sweep_convergence_threshold: f32 = 1e-3,
) -> Continuous_Detection
{
	return {
		mode=.Continuous,
		minimum_sweep_timestep=minimum_sweep_timestep,
		sweep_convergence_threshold=sweep_convergence_threshold,
	};
}

#assert(size_of(Typed_Index) == 4);
#assert(size_of(Continuous_Detection) == 12);
#assert(size_of(Collidable_Description) == 24);
#assert(size_of(Collidable) == 32);
#assert(size_of(Collidable_Reference) == 4);
