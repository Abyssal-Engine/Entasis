package pyramid_wall

import "base:runtime"
import entasis "entasis:entasis"

ROWS :: 180
BODY_COUNT :: ROWS * (ROWS + 1) / 2

Options :: struct
{
	steps: int,
	worker_count: int,
	velocity_iterations: int,
	substeps: int,
	timestep_hz: int,
	sample_every: int,
	sleep_threshold: f32,
	friction: f32,
	hertz: f32,
	damping_ratio: f32,
	recovery: f32,
	output: string,
	snapshot: string,
}

Owner_State :: enum u8
{
	Empty,
	Ready,
}

Owner :: struct
{
	world: entasis.World,
	policy: entasis.Default_Narrow_Policy,
	bodies: []entasis.Body_Handle,
	state: Owner_State,
}

wall_position :: proc(index: int) -> entasis.Vector3
{
	remaining: int = index
	row: int
	width: int = ROWS
	for remaining >= width
	{
		remaining -= width
		width -= 1
		row += 1
	}
	return {(f32(row) + 1) * 0.5 + f32(remaining) - 0.5 * f32(ROWS), f32(row) + 0.5, 0}
}

owner_create :: proc(owner: ^Owner, pool: ^entasis.Buffer_Pool, options: Options) -> entasis.Status
{
	allocation_error: runtime.Allocator_Error
	owner.bodies, allocation_error = make([]entasis.Body_Handle, BODY_COUNT)
	if allocation_error != nil
	{
		return .Capacity_Missing
	}
	owner.policy.material = entasis.contact_material(options.friction, options.recovery, entasis.spring_settings(options.hertz, options.damping_ratio))
	pairs: i32 = BODY_COUNT * 12
	description: entasis.World_Description = entasis.world_description_default()
	description.gravity = {0, -10, 0}
	description.damping = {}
	description.capacity = {
		bodies=BODY_COUNT, statics=1, inactive_body_sets=BODY_COUNT+1, shapes_per_type=4,
		constraints=pairs, initial_constraints_per_type_batch=64, minimum_constraints_per_body=8,
		broad_phase_candidates=pairs, pairs=pairs, collision_child_pairs=i32(options.worker_count),
		inactive_pairs=pairs, pending_pairs_per_worker=4096,
	}
	description.solve = {velocity_iterations=i32(options.velocity_iterations), substeps=i32(options.substeps), fallback_batch_threshold=64}
	description.threading = {worker_count=i32(options.worker_count), worker_pool_block_size=65536}
	description.narrow_callbacks = entasis.narrow_policy_default(&owner.policy)
	status: entasis.Status = entasis.world_init_with_pool(&owner.world, description, pool)
	if status != .Ok
	{
		return status
	}
	owner.state = .Ready
	shape: entasis.Shape_Handle
	shape, status = entasis.shape_add(&owner.world, entasis.box(1, 1, 1))
	if status != .Ok
	{
		return status
	}
	inertia: entasis.Body_Inertia
	inertia, status = entasis.shape_registered_inertia(&owner.world, shape, 1)
	if status != .Ok
	{
		return status
	}
	for index: int = 0; index < BODY_COUNT; index += 1
	{
		body: entasis.Body_Description = entasis.body_dynamic(shape, inertia, entasis.pose(wall_position(index)), {}, entasis.body_activity(options.sleep_threshold, 32))
		body.collidable.continuity.mode = .Discrete
		owner.bodies[index], status = entasis.body_add(&owner.world, body)
		if status != .Ok
		{
			return status
		}
	}
	floor: entasis.Shape_Handle
	floor, status = entasis.shape_add(&owner.world, entasis.box(1000, 2, 1000))
	if status != .Ok
	{
		return status
	}
	floor_handle: entasis.Static_Handle
	floor_handle, status = entasis.static_add(&owner.world, entasis.static_body(floor, entasis.pose({0, -1, 0})))
	_ = floor_handle
	return status
}

owner_destroy :: proc(owner: ^Owner)
{
	if owner.state == .Ready
	{
		entasis.world_destroy(&owner.world)
	}
	delete(owner.bodies)
	owner^ = {}
}
