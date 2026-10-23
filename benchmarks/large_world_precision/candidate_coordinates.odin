package large_world_precision

import entasis "entasis:entasis"

// select once at compile time. mixed64 authors and reads authoritative f64
// positions, while baseline/origin use float inputs
candidate_body_add :: proc(
	owner: ^Benchmark_Owner, description: entasis.Body_Description,
	position: entasis.Global_Position,
) -> (entasis.Body_Handle, entasis.Status)
{
	when PRECISION_NATIVE_GLOBAL
	{
		return entasis.body_add_global(&owner.world, description, position);
	}
	else
	{
		local: entasis.Vector3;
		status: entasis.Status;
		local, status = candidate_local_position(owner, position);
		if status != .Ok
		{
			return {}, status;
		}
		stored: entasis.Body_Description = description;
		stored.pose.position = local;
		return entasis.body_add(&owner.world, stored);
	}
}

candidate_static_add :: proc(
	owner: ^Benchmark_Owner, shape: entasis.Shape_Handle,
	position: entasis.Global_Position, awakening: entasis.Awakening_Policy,
) -> (entasis.Static_Handle, entasis.Status)
{
	when PRECISION_NATIVE_GLOBAL
	{
		return entasis.static_add_global(&owner.world, entasis.static_body(shape, entasis.pose()), position, awakening);
	}
	else
	{
		local: entasis.Vector3;
		status: entasis.Status;
		local, status = candidate_local_position(owner, position);
		if status != .Ok
		{
			return {}, status;
		}
		return entasis.static_add(&owner.world, entasis.static_body(shape, entasis.pose(local)), awakening);
	}
}

candidate_body_global_position :: #force_inline proc(
	owner: ^Benchmark_Owner, handle: entasis.Body_Handle,
) -> (entasis.Global_Position, entasis.Status)
{
	when PRECISION_NATIVE_GLOBAL
	{
		pose: entasis.Global_Pose;
		status: entasis.Status;
		pose, status = entasis.body_get_global_pose(&owner.world, handle);
		return pose.position, status;
	}
	else
	{
		body: entasis.Body_Description;
		status: entasis.Status;
		body, status = entasis.body_get(&owner.world, handle);
		if status != .Ok
		{
			return {}, status;
		}
		return candidate_global_position(owner, body.pose.position);
	}
}

when !PRECISION_NATIVE_GLOBAL
{
	candidate_local_position :: #force_inline proc(
		owner: ^Benchmark_Owner,
		global: entasis.Global_Position,
	) -> (entasis.Vector3, entasis.Status)
	{
		when PRECISION_CANDIDATE == "baseline"
		{
			return entasis.origin_frame_to_local({}, global);
		}
		else
		{
			return entasis.origin_frame_to_local(owner.frame, global);
		}
	}

	candidate_global_position :: #force_inline proc(
		owner: ^Benchmark_Owner,
		local: entasis.Vector3,
	) -> (entasis.Global_Position, entasis.Status)
	{
		when PRECISION_CANDIDATE == "baseline"
		{
			return entasis.global_position(f64(local.x), f64(local.y), f64(local.z)), .Ok;
		}
		else
		{
			return entasis.origin_frame_to_global(owner.frame, local);
		}
	}

}
