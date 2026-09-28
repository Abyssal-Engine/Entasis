package ragdoll_stair_tumble

import "base:runtime"
import "core:math"
import entasis "entasis:entasis"

// reference: Benchmark-Polygon config/cases.json (ragdoll fixture revision 4)
// and BepuRagdollStairTumbleCase.cs. linked-body collisions remain enabled
CASE_ID :: "ragdoll_stair_tumble";
RAGDOLL_GRID :: [2]int{32, 16};
PART_COUNT :: 15;
LINK_COUNT :: 14;
RAGDOLL_COUNT :: RAGDOLL_GRID[0] * RAGDOLL_GRID[1];
STAIR_COUNT :: 64;
DYNAMIC_BODY_COUNT :: RAGDOLL_COUNT * PART_COUNT;
STATIC_BODY_COUNT :: STAIR_COUNT + 4;
PROTOCOL_SHAPE_COUNT :: DYNAMIC_BODY_COUNT + STATIC_BODY_COUNT;
REGISTERED_SHAPE_COUNT :: 1 + 4 + PART_COUNT;
AUTHORED_CONSTRAINT_COUNT :: RAGDOLL_COUNT * LINK_COUNT;
WARMUP_STEP_COUNT :: 30;
MEASURED_STEP_COUNT :: 600;
FINAL_WORLD_STEP_COUNT :: MEASURED_STEP_COUNT;
TIMESTEP_DURATION :: f32(1.0 / 60.0);
POOL_MINIMUM_BLOCK_SIZE :: 65_536;
COLUMN_SPACING :: f32(2.8);
ROW_SPACING :: f32(0.75);
STAIR_RISE :: f32(1.5);
STAIR_DEPTH :: f32(0.75);
YAW_PATTERN :: [3]f32{-5, 0, 5};
PITCH_DEGREES :: f32(12);

// reserve contact storage separately from the 7,168 authored sockets.
// sixteen pair slots per body cover the initially overlapping ragdoll rows
// and their staircase/catch-basin contacts as initial engine capacity hints
PAIR_CAPACITY :: DYNAMIC_BODY_COUNT * 16;
CONSTRAINT_CAPACITY :: PAIR_CAPACITY + AUTHORED_CONSTRAINT_COUNT;

Part_Shape :: enum u8
{
	Box,
	Sphere,
}

Ragdoll_Part :: struct
{
	shape: Part_Shape,
	center: entasis.Vector3,
	half_extents: entasis.Vector3,
	radius: f32,
}

Ragdoll_Link :: struct
{
	parent: int,
	child: int,
	anchor: entasis.Vector3,
}

Static_Box :: struct
{
	center: entasis.Vector3,
	half_extents: entasis.Vector3,
}

PARTS :: [PART_COUNT]Ragdoll_Part{
	{shape=.Box, center={0.0, 1.28, 0.0}, half_extents={0.18, 0.14, 0.12}},
	{shape=.Box, center={0.0, 1.68, 0.0}, half_extents={0.24, 0.25, 0.13}},
	{shape=.Sphere, center={0.0, 2.1, 0.0}, radius=0.16},
	{shape=.Box, center={-0.48, 1.76, 0.0}, half_extents={0.22, 0.08, 0.08}},
	{shape=.Box, center={-0.92, 1.76, 0.0}, half_extents={0.2, 0.07, 0.07}},
	{shape=.Box, center={-1.23, 1.76, 0.0}, half_extents={0.1, 0.06, 0.12}},
	{shape=.Box, center={0.48, 1.76, 0.0}, half_extents={0.22, 0.08, 0.08}},
	{shape=.Box, center={0.92, 1.76, 0.0}, half_extents={0.2, 0.07, 0.07}},
	{shape=.Box, center={1.23, 1.76, 0.0}, half_extents={0.1, 0.06, 0.12}},
	{shape=.Box, center={-0.13, 0.85, 0.0}, half_extents={0.1, 0.27, 0.1}},
	{shape=.Box, center={-0.13, 0.34, 0.0}, half_extents={0.085, 0.23, 0.085}},
	{shape=.Box, center={-0.13, 0.03, 0.14}, half_extents={0.1, 0.06, 0.2}},
	{shape=.Box, center={0.13, 0.85, 0.0}, half_extents={0.1, 0.27, 0.1}},
	{shape=.Box, center={0.13, 0.34, 0.0}, half_extents={0.085, 0.23, 0.085}},
	{shape=.Box, center={0.13, 0.03, 0.14}, half_extents={0.1, 0.06, 0.2}},
};

LINKS :: [LINK_COUNT]Ragdoll_Link{
	{parent=0, child=1, anchor={0.0, 1.425, 0.0}},
	{parent=1, child=2, anchor={0.0, 1.935, 0.0}},
	{parent=1, child=3, anchor={-0.25, 1.76, 0.0}},
	{parent=3, child=4, anchor={-0.71, 1.76, 0.0}},
	{parent=4, child=5, anchor={-1.125, 1.76, 0.0}},
	{parent=1, child=6, anchor={0.25, 1.76, 0.0}},
	{parent=6, child=7, anchor={0.71, 1.76, 0.0}},
	{parent=7, child=8, anchor={1.125, 1.76, 0.0}},
	{parent=0, child=9, anchor={-0.13, 1.13, 0.0}},
	{parent=9, child=10, anchor={-0.13, 0.57, 0.0}},
	{parent=10, child=11, anchor={-0.13, 0.1, 0.04}},
	{parent=0, child=12, anchor={0.13, 1.13, 0.0}},
	{parent=12, child=13, anchor={0.13, 0.57, 0.0}},
	{parent=13, child=14, anchor={0.13, 0.1, 0.04}},
};

EXTRA_STATICS :: [4]Static_Box{
	{center={0.0, -0.5, 36.0}, half_extents={64.0, 0.5, 12.0}},
	{center={-64.5, 3.5, 36.25}, half_extents={0.5, 4.5, 12.25}},
	{center={64.5, 3.5, 36.25}, half_extents={0.5, 4.5, 12.25}},
	{center={0.0, 3.5, 48.5}, half_extents={65.5, 4.5, 1.0}},
};

Case_Status :: enum u8
{
	Ok,
	Allocation_Failed,
	Creation_Failed,
	Fixture_Failed,
	Step_Failed,
	Validation_Failed,
	Release_Failed,
	Output_Failed,
}

Step_Phase :: enum u8
{
	None,
	Warmup,
	Measured,
}

Benchmark_Result :: struct
{
	status: Case_Status,
	physics_status: entasis.Status,
	phase: Step_Phase,
	failed_step: int,
	release_status: entasis.Status,
}

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
	joints: []entasis.Constraint_Handle,
	state: Owner_State,
}

Benchmark_Sample :: struct
{
	stats: entasis.World_Stats,
	invalid_transform_count: int,
	elapsed_ms: f64,
}

benchmark_world_description :: proc(owner: ^Benchmark_Owner, worker_count: int) -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {0, -10, 0};
	description.damping = {};
	description.capacity = {
		bodies=i32(DYNAMIC_BODY_COUNT),
		statics=i32(STATIC_BODY_COUNT),
		inactive_body_sets=1,
		shapes_per_type=REGISTERED_SHAPE_COUNT,
		constraints=i32(CONSTRAINT_CAPACITY),
		initial_constraints_per_type_batch=64,
		minimum_constraints_per_body=8,
		broad_phase_candidates=i32(PAIR_CAPACITY),
		pairs=i32(PAIR_CAPACITY),
		collision_child_pairs=i32(worker_count),
		inactive_pairs=1,
		pending_pairs_per_worker=4096,
	};
	description.solve = {velocity_iterations=4, substeps=1, fallback_batch_threshold=64};
	description.threading = {worker_count=i32(worker_count), worker_pool_block_size=65_536};
	description.narrow_callbacks = entasis.narrow_policy_default(&owner.narrow_policy);
	return description;
}

ragdoll_rotation :: proc "contextless" (index: int) -> entasis.Quaternion
{
	yaw_pattern := YAW_PATTERN;
	yaw := yaw_pattern[index % len(yaw_pattern)] * f32(math.PI) / 180;
	pitch := PITCH_DEGREES * f32(math.PI) / 180;
	sy := math.sin(yaw * 0.5);
	cy := math.cos(yaw * 0.5);
	sp := math.sin(pitch * 0.5);
	cp := math.cos(pitch * 0.5);
	// Hamilton product: yaw around Y multiplied by pitch around X
	q := entasis.Quaternion{x=cy * sp, y=sy * cp, z=-sy * sp, w=cy * cp};
	inverse_length := 1 / math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	return {x=q.x * inverse_length, y=q.y * inverse_length, z=q.z * inverse_length, w=q.w * inverse_length};
}

rotate_part_center :: proc "contextless" (q: entasis.Quaternion, v: entasis.Vector3) -> entasis.Vector3
{
	t := entasis.Vector3{2 * (q.y * v.z - q.z * v.y), 2 * (q.z * v.x - q.x * v.z), 2 * (q.x * v.y - q.y * v.x)};
	return {
		v.x + q.w * t.x + q.y * t.z - q.z * t.y,
		v.y + q.w * t.y + q.z * t.x - q.x * t.z,
		v.z + q.w * t.z + q.x * t.y - q.y * t.x,
	};
}

benchmark_build_fixture :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	parts := PARTS;
	step_shape, status := entasis.shape_add(&owner.world, entasis.box_half_extents(64, 0.75, 0.375));
	if status != .Ok
	{
		return status;
	}
	for row in 0 ..< STAIR_COUNT
	{
		position := entasis.Vector3{
			0, f32(STAIR_COUNT - 1 - row) * STAIR_RISE - 0.75,
			(f32(row) - 0.5 * f32(STAIR_COUNT - 1)) * STAIR_DEPTH,
		};
		_, status = entasis.static_add(&owner.world, entasis.static_body(step_shape, entasis.pose(position)));
		if status != .Ok
		{
			return status;
		}
	}
	for box in EXTRA_STATICS
	{
		shape, shape_status := entasis.shape_add(&owner.world, entasis.box_half_extents(box.half_extents.x, box.half_extents.y, box.half_extents.z));
		if shape_status != .Ok
		{
			return shape_status;
		}
		_, status = entasis.static_add(&owner.world, entasis.static_body(shape, entasis.pose(box.center)));
		if status != .Ok
		{
			return status;
		}
	}
	part_shapes: [PART_COUNT]entasis.Shape_Handle;
	part_inertias: [PART_COUNT]entasis.Body_Inertia;
	for part, index in PARTS
	{
		switch part.shape
		{
			case .Sphere:
				part_shapes[index], status = entasis.shape_add(&owner.world, entasis.sphere(part.radius));
			case .Box:
				part_shapes[index], status = entasis.shape_add(&owner.world, entasis.box_half_extents(part.half_extents.x, part.half_extents.y, part.half_extents.z));
		}
		if status != .Ok
		{
			return status;
		}
		part_inertias[index], status = entasis.shape_registered_inertia(&owner.world, part_shapes[index], 1);
		if status != .Ok
		{
			return status;
		}
	}
	for row in 0 ..< RAGDOLL_GRID[0]
	{
		for column in 0 ..< RAGDOLL_GRID[1]
		{
			ragdoll_index := row * RAGDOLL_GRID[1] + column;
			rotation := ragdoll_rotation(ragdoll_index);
			translation := entasis.Vector3{
				(f32(column) - 0.5 * f32(RAGDOLL_GRID[1] - 1)) * COLUMN_SPACING,
				f32(STAIR_COUNT - 1 - row) * STAIR_RISE + 0.20,
				(f32(row) - 0.5 * f32(STAIR_COUNT - 1)) * ROW_SPACING,
			};
			for part, part_index in PARTS
			{
				center := rotate_part_center(rotation, part.center);
				pose := entasis.Rigid_Pose{
					position={translation.x + center.x, translation.y + center.y, translation.z + center.z},
					orientation=rotation,
				};
				velocity := entasis.velocity({0, 0, row == 0 ? 8 : 2});
				owner.bodies[ragdoll_index * PART_COUNT + part_index], status = entasis.body_add(&owner.world,
					entasis.body_dynamic(part_shapes[part_index], part_inertias[part_index], pose, velocity, entasis.body_activity(-1, 32)),
				);
				if status != .Ok
				{
					return status;
				}
			}
		}
	}
	for ragdoll_index in 0 ..< RAGDOLL_COUNT
	{
		for link, link_index in LINKS
		{
			parent := parts[link.parent].center;
			child := parts[link.child].center;
			joint := entasis.Ball_Socket{
				local_offset_a={link.anchor.x - parent.x, link.anchor.y - parent.y, link.anchor.z - parent.z},
				local_offset_b={link.anchor.x - child.x, link.anchor.y - child.y, link.anchor.z - child.z},
				spring_settings=entasis.spring_settings(30, 1),
			};
			owner.joints[ragdoll_index * LINK_COUNT + link_index], status = entasis.constraint_add_2(&owner.world,
				owner.bodies[ragdoll_index * PART_COUNT + link.parent], owner.bodies[ragdoll_index * PART_COUNT + link.child], joint,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

benchmark_owner_create :: proc(
	owner: ^Benchmark_Owner, pool: ^entasis.Buffer_Pool, worker_count: int,
) -> Benchmark_Result
{
	allocation_error: runtime.Allocator_Error;
	owner.bodies, allocation_error = make([]entasis.Body_Handle, DYNAMIC_BODY_COUNT);
	if allocation_error != nil
	{
		return {status=.Allocation_Failed};
	}
	owner.joints, allocation_error = make([]entasis.Constraint_Handle, AUTHORED_CONSTRAINT_COUNT);
	if allocation_error != nil
	{
		return {status=.Allocation_Failed};
	}
	owner.narrow_policy = {material=entasis.contact_material(0.5, 2, entasis.spring_settings(30, 1))};
	status := entasis.world_init_with_pool(&owner.world, benchmark_world_description(owner, worker_count), pool);
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
	if stats.active_bodies != DYNAMIC_BODY_COUNT || stats.sleeping_bodies != 0 ||
		stats.statics != STATIC_BODY_COUNT || stats.registered_shapes != REGISTERED_SHAPE_COUNT ||
		stats.active_constraints != AUTHORED_CONSTRAINT_COUNT
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
	delete(owner.joints);
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
	if stats.step_index != FINAL_WORLD_STEP_COUNT || stats.active_bodies + stats.sleeping_bodies != DYNAMIC_BODY_COUNT ||
		stats.statics != STATIC_BODY_COUNT || stats.registered_shapes != REGISTERED_SHAPE_COUNT || stats.sleeping_bodies != 0
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
	for handle in owner.joints
	{
		joint: entasis.Ball_Socket;
		joint_status := entasis.constraint_get(&owner.world, handle, &joint);
		if joint_status != .Ok
		{
			return {status=.Validation_Failed, physics_status=joint_status};
		}
	}
	if sample.invalid_transform_count != 0
	{
		return {status=.Validation_Failed};
	}
	return {status=.Ok};
}
