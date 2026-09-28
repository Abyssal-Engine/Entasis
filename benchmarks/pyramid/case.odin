package pyramid

import "base:runtime"
import "core:math"
import entasis "entasis:entasis"
import support "../benchmark_support"

Case_Status :: support.Case_Status;
Benchmark_Result :: support.Benchmark_Result;
Benchmark_Sample :: support.Benchmark_Sample;
POOL_MINIMUM_BLOCK_SIZE :: 65_536;
Owner_State :: enum u8
{
	Empty,
	Ready,
}
Benchmark_Owner :: struct
{
	world: entasis.World,
	narrow_policy: entasis.Default_Narrow_Policy,
	bodies: []entasis.Body_Handle,
	state: Owner_State,
	options: support.Options,
	registered_shapes: int,
	shapes: support.Shape_Cache,
	final_steps: int,
}

activity :: proc(owner: ^Benchmark_Owner) -> entasis.Activity_Description
{
	threshold := f32(-1);
	if owner.options.sleep == .Enabled
	{
		threshold = 0.01;
	}
	return entasis.body_activity(threshold, 32);
}

benchmark_world_description :: proc(owner: ^Benchmark_Owner, worker_count: int) -> entasis.World_Description
{
	pairs := owner.options.body_count * 12;
	inactive_sets, inactive_pairs := 1, 1;
	if owner.options.sleep == .Enabled
	{
		inactive_sets = owner.options.body_count + 1;
		inactive_pairs = pairs;
	}
	description := entasis.world_description_default();
	description.gravity = {0, -10, 0};
	description.damping = {};
	description.capacity = {
		bodies=i32(owner.options.body_count),
		statics=i32(owner.options.static_count),
		inactive_body_sets=i32(inactive_sets),
		shapes_per_type=4,
		constraints=i32(pairs),
		initial_constraints_per_type_batch=64,
		minimum_constraints_per_body=8,
		broad_phase_candidates=i32(pairs),
		pairs=i32(pairs),
		collision_child_pairs=i32(worker_count),
		inactive_pairs=i32(inactive_pairs),
		pending_pairs_per_worker=4096,
	};
	description.solve = {velocity_iterations=i32(owner.options.velocity_iterations), substeps=i32(owner.options.substeps), fallback_batch_threshold=64};
	description.threading = {worker_count=i32(worker_count), worker_pool_block_size=65_536};
	description.narrow_callbacks = entasis.narrow_policy_default(&owner.narrow_policy);
	return description;
}

benchmark_build_fixture :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	o := owner.options;
	shape, inertia, status := support.register_dynamic(&owner.world, &owner.shapes, o.shape, o.shape_size, o.density);
	if status != .Ok
	{
		return status;
	}
	body_index := 0;
	for layer in 0 ..< o.rows
	{
		layer_side := o.rows - layer;
		for depth in 0 ..< layer_side
		{
			for column in 0 ..< layer_side
			{
				position := entasis.Vector3{
					(f32(column) - 0.5 * f32(layer_side - 1)) * (0.5*o.layout_scale),
					(o.shape_size[1]/2) + f32(layer) * o.shape_size[1],
					(f32(depth) - 0.5 * f32(layer_side - 1)) * (0.5*o.layout_scale),
				};
				owner.bodies[body_index], status = entasis.body_add(&owner.world,
				entasis.body_dynamic(shape, inertia, entasis.pose(position), {}, activity(owner)),
				);
				if status != .Ok
				{
					return status;
				}
				body_index += 1;
			}
		}
	}
	if o.projectile_count > 0
	{
		projectile_shape, projectile_inertia, projectile_status := support.register_dynamic(&owner.world, &owner.shapes, .Sphere, {2*o.projectile_radius, 2*o.projectile_radius, 2*o.projectile_radius}, o.projectile_density);
		if projectile_status != .Ok
		{
			return projectile_status;
		}
		for index in 0 ..< o.projectile_count
		{
			position := entasis.Vector3{
				o.projectile_center[0] + f32(index) * o.projectile_spacing[0],
				o.projectile_center[1] + f32(index) * o.projectile_spacing[1],
				o.projectile_center[2] + f32(index) * o.projectile_spacing[2],
			};
			owner.bodies[body_index], status = entasis.body_add(&owner.world,
			entasis.body_dynamic(projectile_shape, projectile_inertia, entasis.pose(position), {}, activity(owner)),
			);
			if status != .Ok
			{
				return status;
			}
			body_index += 1;
		}
	}
	floor_shape, floor_status := support.register_shape(&owner.world, &owner.shapes, o.static_shape, o.floor_size);
	if floor_status != .Ok
	{
		return floor_status;
	}
	_, status = entasis.static_add(&owner.world, entasis.static_body(floor_shape, entasis.pose({0, -o.floor_size[1]/2, 0})));
	return status;
}

benchmark_launch_projectiles :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	for handle in owner.bodies[owner.options.population_count:]
	{
		status := entasis.body_awaken(&owner.world, handle);
		if status != .Ok
		{
			return status;
		}
		body, body_status := entasis.body_get(&owner.world, handle);
		if body_status != .Ok
		{
			return body_status;
		}
		velocity := body.velocity;
		velocity.linear = entasis.Vector3{owner.options.projectile_velocity[0], owner.options.projectile_velocity[1], owner.options.projectile_velocity[2]};
		status = entasis.body_set_velocity(&owner.world, handle, velocity);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

benchmark_owner_create :: proc(
owner: ^Benchmark_Owner, pool: ^entasis.Buffer_Pool, options: support.Options,
) -> Benchmark_Result
{
	owner.options = options;
	owner.final_steps = options.steps + options.warmup_steps;
	allocation_error: runtime.Allocator_Error;
	owner.bodies, allocation_error = make([]entasis.Body_Handle, owner.options.body_count);
	if allocation_error != nil
	{
		return {status=.Allocation_Failed};
	}
	owner.narrow_policy = {material=entasis.contact_material(0.5, 2, entasis.spring_settings(30, 1))};
	status := entasis.world_init_with_pool(&owner.world, benchmark_world_description(owner, options.worker_count), pool);
	if status != .Ok
	{
		return {status=.Creation_Failed, physics_status=status};
	}
	owner.state = .Ready;
	status = benchmark_build_fixture(owner);
	if status != .Ok
	{
		return {status=.Fixture_Failed, physics_status=status};
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok
	{
		return {status=.Fixture_Failed, physics_status=stats_status};
	}
	owner.registered_shapes = owner.shapes.count;
	if stats.active_bodies != owner.options.body_count || stats.sleeping_bodies != 0 ||
	stats.statics != owner.options.static_count || stats.registered_shapes != owner.registered_shapes ||
	stats.active_constraints != 0
	{
		return {status=.Fixture_Failed};
	}
	return {status=.Ok};
}

benchmark_owner_destroy :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	status := entasis.Status.Ok;
	if owner.state == .Ready
	{
		status = entasis.world_destroy(&owner.world);
	}
	delete(owner.bodies);
	owner^ = {};
	return status;
}

benchmark_validate :: proc(owner: ^Benchmark_Owner, sample: ^Benchmark_Sample) -> Benchmark_Result
{
	stats, status := entasis.world_stats(&owner.world);
	if status != .Ok
	{
		return {status=.Validation_Failed, physics_status=status};
	}
	sample.stats = stats;
	if stats.step_index != u64(owner.final_steps) || stats.active_bodies + stats.sleeping_bodies != owner.options.body_count ||
	stats.statics != owner.options.static_count || stats.registered_shapes != owner.registered_shapes || (owner.options.sleep == .Disabled && stats.sleeping_bodies != 0)
	{
		return {status=.Validation_Failed};
	}

 o := owner.options;
 radius := math.sqrt(o.shape_size[0]*o.shape_size[0]+o.shape_size[1]*o.shape_size[1]+o.shape_size[2]*o.shape_size[2])/2;
 x_extent := max(o.floor_size[0]/2, f32(o.rows-1)*0.25*o.layout_scale)+radius;
 z_extent := max(o.floor_size[2]/2, f32(o.rows-1)*0.25*o.layout_scale)+radius;
 y_extent := max(o.floor_size[1], f32(o.rows)*o.shape_size[1])+radius;
 if o.projectile_count > 0
 {
  duration := f32(o.steps)/f32(o.timestep_hz);
  x_extent = max(x_extent, math.abs(o.projectile_center[0])+f32(o.projectile_count-1)*math.abs(o.projectile_spacing[0])+o.projectile_radius+duration*math.abs(o.projectile_velocity[0]));
  y_extent = max(y_extent, math.abs(o.projectile_center[1])+f32(o.projectile_count-1)*math.abs(o.projectile_spacing[1])+o.projectile_radius+duration*math.abs(o.projectile_velocity[1]));
  z_extent = max(z_extent, math.abs(o.projectile_center[2])+f32(o.projectile_count-1)*math.abs(o.projectile_spacing[2])+o.projectile_radius+duration*math.abs(o.projectile_velocity[2]));
 }
 // extent counters describe escape from the authored scene. finite poses and
 // population remain acceptance conditions for open islands and pyramid
	for handle in owner.bodies
	{
		body, body_status := entasis.body_get(&owner.world, handle);
		if body_status != .Ok
		{
			return {status=.Validation_Failed, physics_status=body_status};
		}
  position := body.pose.position;
  if math.abs(position.x)>x_extent || math.abs(position.z)>z_extent || position.y>2*y_extent
  {
   sample.out_of_bounds_count += 1;
  }
  if position.y < -o.floor_size[1]-radius
  {
   sample.below_floor_count += 1;
  }
		values := [7]f32{
			body.pose.position.x, body.pose.position.y, body.pose.position.z,
			body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z, body.pose.orientation.w,
		};
		for value in values
		{
			if transmute(u32)value & 0x7f80_0000 == 0x7f80_0000
			{
				sample.invalid_transform_count += 1;
				break;
			}
		}
	}
	if sample.invalid_transform_count != 0
	{
		return {status=.Validation_Failed};
	}
	return {status=.Ok};
}
