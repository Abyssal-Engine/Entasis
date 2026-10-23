package large_world_precision

@(require) import "core:fmt"
@(require) import "core:math"
@(require) import entasis "entasis:entasis"
@(require) import support "../benchmark_support"

when PRECISION_NATIVE_GLOBAL
{
	// untimed low-chaos oracle. two independent sphere/floor contacts and a
	// separate shapeless distance joint do not acquire a different constraint
	// graph when the broad-phase tree changes traversal order
	Precision_Control_World :: struct
	{
		world: entasis.World,
		policy: entasis.Default_Narrow_Policy,
		bodies: [4]entasis.Body_Handle,
		joint: entasis.Constraint_Handle,
		origin: entasis.Global_Position,
	}

	Precision_Control_Errors :: struct
	{
		position: f64,
		orientation: f64,
		linear: f64,
		angular: f64,
	}

	precision_control_create :: proc(
		owner: ^Precision_Control_World, pool: ^entasis.Buffer_Pool,
		origin: entasis.Global_Position, workers: int,
	) -> entasis.Status
	{
		owner.origin = origin;
		owner.policy = {material=entasis.contact_material(0.5, 2, entasis.spring_settings(30, 1))};
		description: entasis.World_Description = entasis.world_description_default();
		description.gravity = {0, -10, 0};
		description.damping = {};
		description.solve = {velocity_iterations=1, substeps=4, fallback_batch_threshold=64};
		description.threading = {worker_count=i32(workers), worker_pool_block_size=POOL_MINIMUM_BLOCK_SIZE};
		description.capacity = {
			bodies=4, statics=1, inactive_body_sets=1, shapes_per_type=2,
			constraints=32, initial_constraints_per_type_batch=8, minimum_constraints_per_body=8,
			broad_phase_candidates=32, pairs=32, collision_child_pairs=i32(workers),
			inactive_pairs=1, pending_pairs_per_worker=32,
		};
		description.narrow_callbacks = entasis.narrow_policy_default(&owner.policy);
		status: entasis.Status = entasis.world_init_with_pool(&owner.world, description, pool);
		if status != .Ok
		{
			return status;
		}
		floor: entasis.Shape_Handle;
		floor, status = entasis.shape_add(&owner.world, entasis.box(8, 1, 8));
		if status != .Ok
		{
			return status;
		}
		_, status = entasis.static_add_global(&owner.world,
			entasis.static_body(floor, entasis.pose()), {origin.x, origin.y - 0.5, origin.z});
		if status != .Ok
		{
			return status;
		}
		shape: entasis.Shape_Handle;
		shape, status = entasis.shape_add(&owner.world, entasis.sphere(0.5));
		if status != .Ok
		{
			return status;
		}
		inertia: entasis.Body_Inertia;
		inertia, status = entasis.shape_registered_inertia(&owner.world, shape, 1);
		if status != .Ok
		{
			return status;
		}
		positions: [4]entasis.Global_Position = {
			{-2, 0.501, 0.25}, {2, 2, -0.125}, {-1.001, 8, 2}, {1.001, 8, 2},
		};
		for position, index in positions
		{
			body: entasis.Body_Description = entasis.body_dynamic(
				shape, inertia, entasis.pose(), {}, entasis.body_activity(-1, 32));
			body.collidable.maximum_speculative_margin = f32(math.F32_MAX);
			if index >= 2
			{
				body = entasis.body_shapeless(inertia, entasis.pose(), {}, entasis.body_activity(-1, 32));
			}
			owner.bodies[index], status = entasis.body_add_global(&owner.world, body,
				{origin.x + position.x, origin.y + position.y, origin.z + position.z});
			if status != .Ok
			{
				return status;
			}
		}
		owner.joint, status = entasis.constraint_add_2(&owner.world, owner.bodies[2], owner.bodies[3],
			entasis.Center_Distance_Constraint{target_distance=2, spring_settings=entasis.spring_settings(30, 1)});
		return status;
	}

	// quaternion signs are equivalent. the chord formula avoids losing tiny
	// angles to a rounded acos argument and normalizes both quaternion lengths
	precision_orientation_error :: proc "contextless" (a, b: entasis.Quaternion) -> f64
	{
		av: [4]f64 = {f64(a.x), f64(a.y), f64(a.z), f64(a.w)};
		bv: [4]f64 = {f64(b.x), f64(b.y), f64(b.z), f64(b.w)};
		an: f64;
		bn: f64;
		for value, axis in av
		{
			an += value * value;
			bn += bv[axis] * bv[axis];
		}
		if !(an > 0 && bn > 0)
		{
			return math.F64_MAX;
		}
		an = math.sqrt(an);
		bn = math.sqrt(bn);
		difference: f64;
		sum: f64;
		for value, axis in av
		{
			d: f64 = value / an - bv[axis] / bn;
			s: f64 = value / an + bv[axis] / bn;
			difference += d * d;
			sum += s * s;
		}
		return 4 * math.atan2(math.sqrt(min(difference, sum)), math.sqrt(max(difference, sum)));
	}

	precision_control_compare :: proc(
		near, far: ^Precision_Control_World, errors: ^Precision_Control_Errors,
	) -> entasis.Status
	{
		for handle, index in far.bodies
		{
			a, a_status := entasis.body_get(&near.world, near.bodies[index]);
			b, b_status := entasis.body_get(&far.world, handle);
			pa, pa_status := entasis.body_get_global_pose(&near.world, near.bodies[index]);
			pb, pb_status := entasis.body_get_global_pose(&far.world, handle);
			if a_status != .Ok || b_status != .Ok || pa_status != .Ok || pb_status != .Ok
			{
				return .Invalid_Description;
			}
			local: entasis.Global_Position = {
				pb.position.x - far.origin.x, pb.position.y - far.origin.y, pb.position.z - far.origin.z,
			};
			angle: f64 = precision_orientation_error(pa.orientation, pb.orientation);
			if precision_global_close(pa.position, local, POSITION_TOLERANCE) != .Accepted ||
				precision_vector_close(a.velocity.linear, b.velocity.linear, 0.0001) != .Accepted ||
				precision_vector_close(a.velocity.angular, b.velocity.angular, 0.0001) != .Accepted ||
				precision_close(angle, 0, 0.0001) != .Accepted
			{
				fmt.eprintfln("precision_control_failed body=%d origin=%v near=%v far_local=%v near_velocity=%v far_velocity=%v orientation_error=%.17g",
					index, far.origin, pa.position, local, a.velocity, b.velocity, angle);
				return .Invalid_Description;
			}
			errors.position = max(errors.position, abs(local.x - pa.position.x), abs(local.y - pa.position.y), abs(local.z - pa.position.z));
			errors.orientation = max(errors.orientation, angle);
			errors.linear = max(errors.linear, f64(abs(a.velocity.linear.x - b.velocity.linear.x)),
				f64(abs(a.velocity.linear.y - b.velocity.linear.y)), f64(abs(a.velocity.linear.z - b.velocity.linear.z)));
			errors.angular = max(errors.angular, f64(abs(a.velocity.angular.x - b.velocity.angular.x)),
				f64(abs(a.velocity.angular.y - b.velocity.angular.y)), f64(abs(a.velocity.angular.z - b.velocity.angular.z)));
			if index < 2 && local.y < 0
			{
				return .Invalid_Description;
			}
		}
		return .Ok;
	}

	precision_control_run :: proc(
		pool: ^entasis.Buffer_Pool, origin: entasis.Global_Position, workers: int,
	) -> (result: support.Benchmark_Result)
	{
		worlds: [2]Precision_Control_World;
		defer
		{
			for &owner in worlds
			{
				status: entasis.Status = entasis.world_destroy(&owner.world);
				if status != .Ok
				{
					result.release_status = status;
					result.status = .Release_Failed;
				}
			}
		}
		for &owner, index in worlds
		{
			position: entasis.Global_Position;
			if index == 1
			{
				position = origin;
			}
			status: entasis.Status = precision_control_create(&owner, pool, position, workers);
			if status != .Ok
			{
				return {status=.Creation_Failed, physics_status=status};
			}
		}
		errors: Precision_Control_Errors;
		for step in 0 ..< 128
		{
			for &owner in worlds
			{
				status: entasis.Status = entasis.world_step(&owner.world, 1 / f32(60));
				if status != .Ok
				{
					return {status=.Step_Failed, physics_status=status, failed_step=step + 1};
				}
				stats, stats_status := entasis.world_stats(&owner.world);
				if stats_status != .Ok || stats.active_bodies != 4 || stats.statics != 1 ||
					stats.sleeping_bodies != 0 || stats.active_constraints < 2
				{
					return {status=.Validation_Failed, physics_status=stats_status, failed_step=step + 1};
				}
			}
			status: entasis.Status = precision_control_compare(&worlds[0], &worlds[1], &errors);
			if status != .Ok
			{
				return {status=.Validation_Failed, physics_status=status, failed_step=step + 1};
			}
			if step == 0
			{
				for &owner in worlds
				{
					a, _ := entasis.body_get(&owner.world, owner.bodies[2]);
					b, _ := entasis.body_get(&owner.world, owner.bodies[3]);
					if !(a.velocity.linear.x > 0 && b.velocity.linear.x < 0)
					{
						return {status=.Validation_Failed, failed_step=1};
					}
				}
			}
		}
		fmt.printfln("precision_control_passed origin=%v workers=%d steps=128 position_error=%.17g orientation_error=%.17g linear_error=%.17g angular_error=%.17g",
			origin, workers, errors.position, errors.orientation, errors.linear, errors.angular);
		return {status=.Ok};
	}
}
