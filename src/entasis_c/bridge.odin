package entasis_c

import entasis "entasis:entasis"

ENTASIS_ABI_VERSION :: entasis.ABI_VERSION;
ENTASIS_TRUE        :: Entasis_Bool(1);
ENTASIS_FALSE       :: Entasis_Bool(0);

abi_bool :: #force_inline proc "contextless" (value: bool) -> Entasis_Bool
{
	return value ? ENTASIS_TRUE : ENTASIS_FALSE;
}

abi_string_view :: #force_inline proc "contextless" (value: string) -> Entasis_String_View
{
	return {
		data=cstring(raw_data(value)),
		count=u64(len(value)),
	};
}

abi_version :: #force_inline proc "contextless" () -> u32
{
	return ENTASIS_ABI_VERSION;
}

abi_version_current :: proc "contextless" () -> Entasis_Version
{
	version := entasis.version_current();
	return {
		major=version.major,
		minor=version.minor,
		patch=version.patch,
		prerelease=abi_string_view(version.prerelease),
	};
}

abi_status_ok :: #force_inline proc "contextless" (status: Entasis_Status) -> Entasis_Bool
{
	return abi_bool(status == Entasis_Status(entasis.Status.Ok));
}

abi_status_failed :: #force_inline proc "contextless" (status: Entasis_Status) -> Entasis_Bool
{
	return abi_bool(status != Entasis_Status(entasis.Status.Ok));
}

abi_status_text :: proc "contextless" (status: Entasis_Status) -> Entasis_String_View
{
	text := "unknown status";
	switch status
	{
		case Entasis_Status(entasis.Status.Ok):
			text = "ok";
		case Entasis_Status(entasis.Status.Invalid_Argument):
			text = "invalid argument";
		case Entasis_Status(entasis.Status.Invalid_Description):
			text = "invalid description";
		case Entasis_Status(entasis.Status.Not_Found):
			text = "not found";
		case Entasis_Status(entasis.Status.Capacity_Missing):
			text = "capacity missing";
		case Entasis_Status(entasis.Status.Shape_In_Use):
			text = "shape in use";
		case Entasis_Status(entasis.Status.Disposed):
			text = "disposed";
	}
	return abi_string_view(text);
}

abi_diagnostic_clear :: #force_inline proc "contextless" (diagnostic: ^Entasis_Diagnostic)
{
	if diagnostic == nil
	{
		return;
	}
	diagnostic^ = {};
}

abi_diagnostic_record :: #force_inline proc "contextless" (
	diagnostic: ^Entasis_Diagnostic,
	status: Entasis_Status,
	operation: Entasis_Diagnostic_Operation,
	detail: i32,
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

abi_body_handle_invalid :: #force_inline proc "contextless" () -> Entasis_Body_Handle
{
	handle := entasis.body_handle_invalid();
	return {value=handle.value};
}

abi_body_handle_is_valid :: #force_inline proc "contextless" (
	handle: Entasis_Body_Handle,
) -> Entasis_Bool
{
	return abi_bool(entasis.body_handle_is_valid({value=handle.value}));
}

abi_static_handle_invalid :: #force_inline proc "contextless" () -> Entasis_Static_Handle
{
	handle := entasis.static_handle_invalid();
	return {value=handle.value};
}

abi_static_handle_is_valid :: #force_inline proc "contextless" (
	handle: Entasis_Static_Handle,
) -> Entasis_Bool
{
	return abi_bool(entasis.static_handle_is_valid({value=handle.value}));
}

abi_constraint_handle_invalid :: #force_inline proc "contextless" () -> Entasis_Constraint_Handle
{
	handle := entasis.constraint_handle_invalid();
	return {value=handle.value};
}

abi_constraint_handle_is_valid :: #force_inline proc "contextless" (
	handle: Entasis_Constraint_Handle,
) -> Entasis_Bool
{
	return abi_bool(entasis.constraint_handle_is_valid({value=handle.value}));
}

abi_shape_handle_invalid :: #force_inline proc "contextless" () -> Entasis_Shape_Handle
{
	handle := entasis.shape_handle_invalid();
	return {packed=handle.packed};
}

abi_shape_handle_is_valid :: #force_inline proc "contextless" (
	handle: Entasis_Shape_Handle,
) -> Entasis_Bool
{
	return abi_bool(entasis.shape_handle_is_valid({packed=handle.packed}));
}

#assert(ENTASIS_ABI_VERSION == entasis.ABI_VERSION);
#assert(size_of(Entasis_Body_Handle) == size_of(entasis.Body_Handle));
#assert(size_of(Entasis_Static_Handle) == size_of(entasis.Static_Handle));
#assert(size_of(Entasis_Constraint_Handle) == size_of(entasis.Constraint_Handle));
#assert(size_of(Entasis_Shape_Handle) == size_of(entasis.Shape_Handle));
#assert(size_of(Entasis_Vector2) == size_of(entasis.Vector2));
#assert(size_of(Entasis_Vector3) == size_of(entasis.Vector3));
#assert(size_of(Entasis_Vector4) == size_of(entasis.Vector4));
#assert(size_of(Entasis_Quaternion) == size_of(entasis.Quaternion));
#assert(size_of(Entasis_Matrix3x3) == size_of(entasis.Matrix3x3));
#assert(size_of(Entasis_Symmetric3x3) == size_of(entasis.Symmetric3x3));
#assert(size_of(Entasis_Bounding_Box) == size_of(entasis.Bounding_Box));
#assert(size_of(Entasis_Rigid_Pose) == size_of(entasis.Rigid_Pose));
#assert(size_of(Entasis_Body_Velocity) == size_of(entasis.Body_Velocity));
#assert(size_of(Entasis_Body_Inertia) == size_of(entasis.Body_Inertia));
#assert(size_of(Entasis_Continuous_Detection) == size_of(entasis.Continuous_Detection));
#assert(size_of(Entasis_Collidable_Description) == size_of(entasis.Collidable_Description));
#assert(size_of(Entasis_Activity_Description) == size_of(entasis.Activity_Description));
#assert(size_of(Entasis_Body_Description) == size_of(entasis.Body_Description));
#assert(size_of(Entasis_Static_Description) == size_of(entasis.Static_Description));
#assert(size_of(Entasis_Collidable_Reference) == size_of(entasis.Collidable_Reference));
#assert(size_of(Entasis_Ray) == size_of(entasis.Ray));
#assert(size_of(Entasis_Diagnostic) == size_of(entasis.Diagnostic));
