package container

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

HULL_PROFILE :: #config(ENTASIS_HULL_PROFILE, false);

benchmark_world_description :: proc(owner: ^Benchmark_Owner, worker_count: int) -> entasis.World_Description
{
	pairs := max(1, (owner.options.body_count * 96 * 1024 + 9999) / 10000);
	inactive_sets, inactive_pairs := 1, 1;
	if owner.options.sleep == .Enabled
	{
		inactive_sets = owner.options.body_count + 1;
		inactive_pairs = pairs;
	}
	description := entasis.world_description_default();
	description.gravity = {0, -10, 0};
	description.damping = {};
	description.profiling = HULL_PROFILE;
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
	t := o.layout_scale;
	floor_shape, status := support.register_shape(&owner.world, &owner.shapes, o.static_shape, {o.container_size[0], t, o.container_size[2]});
	if status != .Ok
	{
		return status;
	}
	side_shape, side_status := support.register_shape(&owner.world, &owner.shapes, o.static_shape, {t, o.container_size[1], o.container_size[2]});
	if side_status != .Ok
	{
		return side_status;
	}
	front_shape, front_status := support.register_shape(&owner.world, &owner.shapes, o.static_shape, {o.container_size[0], o.container_size[1], t});
	if front_status != .Ok
	{
		return front_status;
	}
	shape, inertia, shape_status := support.register_dynamic(&owner.world, &owner.shapes, o.shape, o.shape_size, o.density);
	if shape_status != .Ok
	{
		return shape_status;
	}
	shapes := [5]entasis.Shape_Handle{floor_shape, side_shape, side_shape, front_shape, front_shape};
	x, y, z := o.container_size[0]/2, (o.container_size[1]-t)/2, o.container_size[2]/2;
	positions := [5]entasis.Vector3{{0, -t/2, 0}, {-x, y, 0}, {x, y, 0}, {0, y, -z}, {0, y, z}};
	for position, index in positions
	{
		_, status = entasis.static_add(&owner.world, entasis.static_body(shapes[index], entasis.pose(position)));
		if status != .Ok
		{
			return status;
		}
	}
	description := entasis.body_dynamic(shape, inertia, entasis.pose(), {}, activity(owner));
	description.collidable.maximum_speculative_margin = f32(math.F32_MAX);
	origin_x := -0.5*f32(o.grid[0]-1)*o.spacing[0];
	origin_z := -0.5*f32(o.grid[2]-1)*o.spacing[2];
	index := 0;
	for y in 0 ..< o.grid[1]
	{
		for z in 0 ..< o.grid[2]
		{
			for x in 0 ..< o.grid[0]
			{
				description.pose.position = {origin_x+f32(x)*o.spacing[0], o.spawn_height+f32(y)*o.spacing[1], origin_z+f32(z)*o.spacing[2]};
				owner.bodies[index], status = entasis.body_add(&owner.world, description);
				if status != .Ok
				{
					return status;
				}
				index += 1;
			}
		}
	}
	return .Ok;
}

benchmark_owner_create :: proc(
owner: ^Benchmark_Owner, pool: ^entasis.Buffer_Pool, options: support.Options,
) -> Benchmark_Result
{
	owner.options = options;
	owner.final_steps = options.steps;
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
	for handle in owner.bodies
	{
		body, body_status := entasis.body_get(&owner.world, handle);
		if body_status != .Ok
		{
			return {status=.Validation_Failed, physics_status=body_status};
		}
		p := body.pose.position;
		o := owner.options;
		radius := math.sqrt(o.shape_size[0]*o.shape_size[0]+o.shape_size[1]*o.shape_size[1]+o.shape_size[2]*o.shape_size[2])/2;
		x_limit := max(o.container_size[0]/2+o.layout_scale/2, f32(o.grid[0]-1)*o.spacing[0]/2) + radius;
		z_limit := max(o.container_size[2]/2+o.layout_scale/2, f32(o.grid[2]-1)*o.spacing[2]/2) + radius;
		y_limit := 2*max(o.container_size[1], o.spawn_height+f32(o.grid[1]-1)*o.spacing[1])+radius;
		if p.y < min(f32(0), o.spawn_height-o.shape_size[1])
		{
			sample.below_floor_count += 1;
		}
		if math.abs(p.x)>x_limit || math.abs(p.z)>z_limit || p.y>y_limit
		{
			sample.out_of_bounds_count += 1;
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
	if sample.invalid_transform_count != 0 || sample.below_floor_count != 0
	{
		return {status=.Validation_Failed};
	}
	return {status=.Ok};
}
