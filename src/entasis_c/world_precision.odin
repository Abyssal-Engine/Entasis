package entasis_c

import entasis "entasis:entasis"

abi_global_position :: #force_inline proc "contextless" (x, y, z: f64) -> Entasis_Global_Position
{
	return {x=x, y=y, z=z};
}

abi_origin_frame_at :: proc "contextless" (
	origin: ^Entasis_Global_Position, result: ^Entasis_Origin_Frame,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if origin == nil || result == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	frame: entasis.Origin_Frame;
	status: entasis.Status;
	frame, status = entasis.origin_frame_at(cast(entasis.Global_Position)origin^);
	if status == .Ok
	{
		result^ = transmute(Entasis_Origin_Frame)frame;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_origin_frame_to_local :: proc "contextless" (
	frame: ^Entasis_Origin_Frame, position: ^Entasis_Global_Position,
	result: ^Entasis_Vector3, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if frame == nil || position == nil || result == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	local: entasis.Vector3;
	status: entasis.Status;
	local, status = entasis.origin_frame_to_local(
		transmute(entasis.Origin_Frame)frame^, cast(entasis.Global_Position)position^,
	);
	if status == .Ok
	{
		result^ = abi_vector3_from_core(local);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_origin_frame_to_global :: proc "contextless" (
	frame: ^Entasis_Origin_Frame, position: ^Entasis_Vector3,
	result: ^Entasis_Global_Position, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if frame == nil || position == nil || result == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	global: entasis.Global_Position;
	status: entasis.Status;
	global, status = entasis.origin_frame_to_global(
		transmute(entasis.Origin_Frame)frame^, abi_vector3_to_core(position^),
	);
	if status == .Ok
	{
		result^ = cast(Entasis_Global_Position)global;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_global_position_delta :: proc "contextless" (
	from, to: ^Entasis_Global_Position, result: ^Entasis_Vector3,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if from == nil || to == nil || result == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	delta: entasis.Vector3;
	status: entasis.Status;
	delta, status = entasis.global_position_delta(
		cast(entasis.Global_Position)from^, cast(entasis.Global_Position)to^,
	);
	if status == .Ok
	{
		result^ = abi_vector3_from_core(delta);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_origin_frame_translate :: proc "contextless" (
	frame: ^Entasis_Origin_Frame, delta: ^Entasis_Global_Position,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if frame == nil || delta == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	native: entasis.Origin_Frame = transmute(entasis.Origin_Frame)frame^;
	status: entasis.Status = entasis.origin_frame_translate(&native, cast(entasis.Global_Position)delta^);
	if status == .Ok
	{
		frame^ = transmute(Entasis_Origin_Frame)native;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_world_shift_origin :: proc "contextless" (
	world: ^Entasis_World, frame: ^Entasis_Origin_Frame,
	delta: ^Entasis_Vector3, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_world_resource_for_domain(world, diagnostic, .None, .Exclusive);
	if ready != abi_status(.Ok)
	{
		return ready;
	}
	defer abi_world_release(resource);
	if frame == nil || delta == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	native: entasis.Origin_Frame = transmute(entasis.Origin_Frame)frame^;
	status: entasis.Status = entasis.world_shift_origin(&resource.world, &native, abi_vector3_to_core(delta^));
	if status == .Ok
	{
		frame^ = transmute(Entasis_Origin_Frame)native;
	}
	return abi_status_finish(status, diagnostic, .None);
}
