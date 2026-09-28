package entasis

import physics "entasis:entasis_physics"

// Diagnostic_Operation identifies a cold facade operation without storing or
// formatting a string. more operation values may be added compatibly
Diagnostic_Operation :: enum u8
{
	None,
	World_Initialize,
	World_Ensure_Capacity,
	Callback_Configure,
	Shape_Register,
	Constraint_Register,
	Collision_Task_Register,
	Sweep_Task_Register,
	Dispatcher_Initialize,
	Asset_Import,
}

// Diagnostic is optional caller-owned cold-path failure context
//
// detail is operation specific. the facade owns no diagnostic memory and never
// writes a global last-error value
Diagnostic :: struct
{
	status:    Status,
	operation: Diagnostic_Operation,
	detail:    i32,
}

// status_ok reports whether a public operation completed successfully
status_ok :: #force_inline proc "contextless" (status: Status) -> bool
{
	return status == .Ok;
}

// status_failed reports whether a public operation failed
status_failed :: #force_inline proc "contextless" (status: Status) -> bool
{
	return status != .Ok;
}

// status_text returns stable, allocation-free text for every public status
status_text :: proc "contextless" (status: Status) -> string
{
	switch status
	{
		case .Ok:
		return "ok";
		case .Invalid_Argument:
		return "invalid argument";
		case .Invalid_Description:
		return "invalid description";
		case .Not_Found:
		return "not found";
		case .Capacity_Missing:
		return "capacity missing";
		case .Shape_In_Use:
		return "shape in use";
		case .Disposed:
		return "disposed";
		case .No_Convergence:
		return "no convergence";
	}
	return "unknown status";
}

// diagnostic_clear resets optional caller-owned diagnostic context
diagnostic_clear :: #force_inline proc "contextless" (diagnostic: ^Diagnostic)
{
	if diagnostic == nil
	{
		return;
	}
	diagnostic^ = {};
}

// diagnostic_record stores allocation-free cold-path failure context
diagnostic_record :: #force_inline proc "contextless" (
	diagnostic: ^Diagnostic,
	status: Status,
	operation: Diagnostic_Operation,
	detail: i32 = 0,
)
{
	if diagnostic == nil
	{
		return;
	}
	diagnostic^ = {
		status=status,
		operation=operation,
		detail=detail,
	};
}

// body_handle_invalid returns the public invalid body-handle sentinel
body_handle_invalid :: #force_inline proc "contextless" () -> Body_Handle
{
	return physics.body_handle_invalid();
}

// body_handle_is_valid checks only the sentinel representation. world operations
// remain responsible for detecting stale handles
body_handle_is_valid :: #force_inline proc "contextless" (handle: Body_Handle) -> bool
{
	return handle.value >= 0;
}

// static_handle_invalid returns the public invalid static-handle sentinel
static_handle_invalid :: #force_inline proc "contextless" () -> Static_Handle
{
	return physics.static_handle_invalid();
}

// static_handle_is_valid checks only the sentinel representation. world operations
// remain responsible for detecting stale handles
static_handle_is_valid :: #force_inline proc "contextless" (handle: Static_Handle) -> bool
{
	return handle.value >= 0;
}

// constraint_handle_invalid returns the public invalid constraint-handle sentinel
constraint_handle_invalid :: #force_inline proc "contextless" () -> Constraint_Handle
{
	return physics.constraint_handle_invalid();
}

// constraint_handle_is_valid checks only the sentinel representation. world
// operations remain responsible for detecting stale handles
constraint_handle_is_valid :: #force_inline proc "contextless" (
	handle: Constraint_Handle,
) -> bool
{
	return handle.value >= 0;
}

// shape_handle_invalid returns the public missing-shape sentinel
shape_handle_invalid :: #force_inline proc "contextless" () -> Shape_Handle
{
	return {};
}

// shape_handle_is_valid checks only the typed-index existence bit. world
// operations remain responsible for detecting stale handles
shape_handle_is_valid :: #force_inline proc "contextless" (handle: Shape_Handle) -> bool
{
	return physics.typed_index_state(handle) == .Present;
}
#assert(size_of(Status) == size_of(physics.Physics_Status));
#assert(int(Status.Ok) == int(physics.Physics_Status.Ok));
#assert(int(Status.Invalid_Argument) == int(physics.Physics_Status.Invalid_Argument));
#assert(int(Status.Invalid_Description) == int(physics.Physics_Status.Invalid_Description));
#assert(int(Status.Not_Found) == int(physics.Physics_Status.Not_Found));
#assert(int(Status.Capacity_Missing) == int(physics.Physics_Status.Capacity_Missing));
#assert(int(Status.Shape_In_Use) == int(physics.Physics_Status.Shape_In_Use));
#assert(int(Status.Disposed) == int(physics.Physics_Status.Disposed));
#assert(int(Status.No_Convergence) == 7);
