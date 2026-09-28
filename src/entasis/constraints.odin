package entasis

import "core:math"
import physics "entasis:entasis_physics"

// Constraint_Type_ID is the stable built-in constraint description identity.
// ordinary add/get/apply calls infer it at compile time from the description
Constraint_Type_ID :: distinct i32;

// CONSTRAINT_TYPE_INVALID invalid public constraint type sentinel
CONSTRAINT_TYPE_INVALID :: Constraint_Type_ID(-1);

// Constraint_State identifies whether a live constraint is stored in the active
// solver or in one sleeping island. Missing is the zero-value sentinel
Constraint_State :: enum u8
{
	Missing,
	Active,
	Inactive,
}

// Constraint_Info is a compact pointer-free inspection record. only the first
// body_count body handles are valid. unused entries contain invalid handles
Constraint_Info :: struct
{
	handle:     Constraint_Handle,
	type_id:    Constraint_Type_ID,
	bodies:     [4]Body_Handle,
	body_count: u8,
	state:      Constraint_State,
}

// Servo_Settings limits correction speed and force for position or orientation targets
Servo_Settings :: physics.Servo_Settings;
// Motor_Settings limits force and sets mass-scaled damping for velocity targets
Motor_Settings :: physics.Motor_Settings;

// Angular_Axis_Gear_Motor couples angular velocities along A's axis at a chosen ratio
Angular_Axis_Gear_Motor :: physics.Angular_Axis_Gear_Motor;
// Angular_Axis_Motor drives relative angular speed along A's local axis
Angular_Axis_Motor      :: physics.Angular_Axis_Motor;
// Angular_Hinge aligns two local axes while leaving rotation around them free
Angular_Hinge           :: physics.Angular_Hinge;
// Angular_Motor drives relative angular velocity expressed in A's local frame
Angular_Motor           :: physics.Angular_Motor;
// Angular_Servo drives a target relative orientation with speed and force limits
Angular_Servo           :: physics.Angular_Servo;
// Angular_Swivel_Hinge keeps the swivel and hinge axes perpendicular
Angular_Swivel_Hinge    :: physics.Angular_Swivel_Hinge;
// Area_Constraint preserves the scaled triangle area between three body centers
Area_Constraint         :: physics.Area_Constraint;
// Ball_Socket keeps two local anchor points together while allowing rotation
Ball_Socket             :: physics.Ball_Socket;
// Ball_Socket_Motor drives B's anchor velocity relative to A in A's local frame
Ball_Socket_Motor       :: physics.Ball_Socket_Motor;
// Ball_Socket_Servo brings two local anchors together with speed and force limits
Ball_Socket_Servo       :: physics.Ball_Socket_Servo;
// Center_Distance_Constraint holds a target distance between body centers
// without local anchor offsets
Center_Distance_Constraint :: physics.Center_Distance_Constraint;
// Center_Distance_Limit bounds the distance between body centers
Center_Distance_Limit      :: physics.Center_Distance_Limit;
// Distance_Limit bounds the distance between two local anchor points
Distance_Limit             :: physics.Distance_Limit;
// Distance_Servo drives anchor separation to a target distance
Distance_Servo             :: physics.Distance_Servo;
// Hinge joins local anchors and aligns axes while leaving hinge rotation free
Hinge                      :: physics.Hinge;
// Linear_Axis_Limit bounds anchor separation projected onto A's local axis
Linear_Axis_Limit          :: physics.Linear_Axis_Limit;
// Linear_Axis_Motor drives relative anchor speed along A's local axis
Linear_Axis_Motor          :: physics.Linear_Axis_Motor;
// Linear_Axis_Servo drives anchor separation along A's local plane normal
Linear_Axis_Servo          :: physics.Linear_Axis_Servo;
// One_Body_Angular_Motor drives one body's world-space angular velocity
One_Body_Angular_Motor     :: physics.One_Body_Angular_Motor;
// One_Body_Angular_Servo drives one body toward a world-space orientation
One_Body_Angular_Servo     :: physics.One_Body_Angular_Servo;
// One_Body_Linear_Motor drives a local anchor's world-space velocity
One_Body_Linear_Motor      :: physics.One_Body_Linear_Motor;
// One_Body_Linear_Servo drives a local anchor toward a world-space position
One_Body_Linear_Servo      :: physics.One_Body_Linear_Servo;
// Point_On_Line_Servo keeps B's anchor on a line fixed in A's local frame
Point_On_Line_Servo        :: physics.Point_On_Line_Servo;
// Swing_Limit bounds the angle between two body-local axes by their minimum dot
Swing_Limit                :: physics.Swing_Limit;
// Swivel_Hinge joins anchors while keeping swivel and hinge axes perpendicular
Swivel_Hinge               :: physics.Swivel_Hinge;
// Twist_Limit bounds relative twist between two local orientation bases
Twist_Limit                :: physics.Twist_Limit;
// Twist_Motor drives relative twist speed around the two local axes
Twist_Motor                :: physics.Twist_Motor;
// Twist_Servo drives a target twist angle between two local orientation bases
Twist_Servo                :: physics.Twist_Servo;
// Volume_Constraint preserves scaled signed tetrahedron volume across four bodies
Volume_Constraint          :: physics.Volume_Constraint;
// Weld holds the relative position and orientation of two bodies
Weld                       :: physics.Weld;

// servo_settings constructs the exact runtime servo settings
// allocation: none
servo_settings :: #force_inline proc "contextless" (
	maximum_speed, base_speed, maximum_force: f32,
) -> Servo_Settings
{
	return {
		maximum_speed=maximum_speed,
		base_speed=base_speed,
		maximum_force=maximum_force,
	};
}

// motor_settings constructs the existing high-stiffness motor settings.
// softness is zero for a rigid motor and increases as the motor becomes softer
// allocation: none
motor_settings :: #force_inline proc "contextless" (
	maximum_force, softness: f32,
) -> Motor_Settings
{
	damping := f32(math.F32_MAX);
	if softness > 0
	{
		damping = 1 / softness;
	}
	return {maximum_force=maximum_force, damping=damping};
}

// motor_settings_damping constructs motor settings from the raw mass-scaled
// damping constant stored by the runtime. prefer motor_settings for ordinary use
// allocation: none
motor_settings_damping :: #force_inline proc "contextless" (
	maximum_force, damping: f32,
) -> Motor_Settings
{
	return {maximum_force=maximum_force, damping=damping};
}

// constraint_type_id returns the built-in public constraint type represented by
// T. contact constraints and unsupported/custom descriptions return invalid
constraint_type_id :: #force_inline proc "contextless" ($T: typeid) -> Constraint_Type_ID
{
	type_id := physics.constraint_description_type_id(T);
	if (type_id >= physics.BALL_SOCKET_TYPE_ID && type_id <= physics.HINGE_TYPE_ID) ||
		(type_id >= physics.BALL_SOCKET_MOTOR_TYPE_ID &&
		type_id <= physics.CENTER_DISTANCE_LIMIT_TYPE_ID)
	{
		return Constraint_Type_ID(type_id);
	}
	return CONSTRAINT_TYPE_INVALID;
}

// constraint_body_count returns the built-in description body arity or zero for
// an unsupported description type
constraint_body_count :: #force_inline proc "contextless" ($T: typeid) -> int
{
	if constraint_type_id(T) == CONSTRAINT_TYPE_INVALID
	{
		return 0;
	}
	return int(physics.constraint_description_body_count(T));
}

@(private)
constraint_world_ready :: #force_inline proc "contextless" (
	world: ^World,
) -> (^world_data, ^physics.Simulation, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, nil, .Invalid_Argument;
	}
	return data, &data.simulation, .Ok;
}

@(private)
constraint_invalid_bodies :: #force_inline proc "contextless" () -> [4]Body_Handle
{
	invalid := body_handle_invalid();
	return {invalid, invalid, invalid, invalid};
}

@(private)
constraint_add_array :: #force_inline proc (
	world: ^World,
	body_handles: [4]Body_Handle,
	description: $T,
) -> (Constraint_Handle, Status)
{
	if constraint_type_id(T) == CONSTRAINT_TYPE_INVALID
	{
		return constraint_handle_invalid(), .Invalid_Argument;
	}
	data, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return constraint_handle_invalid(), ready_status;
	}
	world_invalidate_views_data(data);
	stored_body_handles := body_handles;
	stored_description := description;
	return physics.simulation_add_constraint(
		simulation, &stored_body_handles, &stored_description,
	);
}

// constraint_add adds one built-in constraint from an exact-length body slice
// allocation: world storage may grow. ownership: owner thread while world idle
constraint_add :: proc (
	world: ^World,
	bodies: []Body_Handle,
	description: $T,
) -> (Constraint_Handle, Status)
{
	body_count := constraint_body_count(T);
	if body_count == 0 || len(bodies) != body_count
	{
		return constraint_handle_invalid(), .Invalid_Argument;
	}
	body_handles := constraint_invalid_bodies();
	for index in 0 ..< body_count
	{
		body_handles[index] = bodies[index];
	}
	return constraint_add_array(world, body_handles, description);
}

// constraint_add_1 adds a one-body built-in constraint
constraint_add_1 :: #force_inline proc (
	world: ^World,
	a: Body_Handle,
	description: $T,
) -> (Constraint_Handle, Status)
{
	when T != One_Body_Angular_Servo && T != One_Body_Angular_Motor &&
		T != One_Body_Linear_Servo && T != One_Body_Linear_Motor
	{
		#assert(false, "constraint_add_1 requires a one-body constraint description");
	}
	body_handles := constraint_invalid_bodies();
	body_handles[0] = a;
	return constraint_add_array(world, body_handles, description);
}

// constraint_add_2 adds a two-body built-in constraint
constraint_add_2 :: #force_inline proc (
	world: ^World,
	a, b: Body_Handle,
	description: $T,
) -> (Constraint_Handle, Status)
{
	when T == One_Body_Angular_Servo || T == One_Body_Angular_Motor ||
		T == One_Body_Linear_Servo || T == One_Body_Linear_Motor ||
		T == Area_Constraint || T == Volume_Constraint
	{
		#assert(false, "constraint_add_2 requires a two-body constraint description");
	}
	body_handles := constraint_invalid_bodies();
	body_handles[0] = a;
	body_handles[1] = b;
	return constraint_add_array(world, body_handles, description);
}

// constraint_add_3 adds a three-body built-in constraint
constraint_add_3 :: #force_inline proc (
	world: ^World,
	a, b, c: Body_Handle,
	description: $T,
) -> (Constraint_Handle, Status)
{
	when T != Area_Constraint
	{
		#assert(false, "constraint_add_3 requires a three-body constraint description");
	}
	body_handles := constraint_invalid_bodies();
	body_handles[0] = a;
	body_handles[1] = b;
	body_handles[2] = c;
	return constraint_add_array(world, body_handles, description);
}

// constraint_add_4 adds a four-body built-in constraint
constraint_add_4 :: #force_inline proc (
	world: ^World,
	a, b, c, d: Body_Handle,
	description: $T,
) -> (Constraint_Handle, Status)
{
	when T != Volume_Constraint
	{
		#assert(false, "constraint_add_4 requires a four-body constraint description");
	}
	body_handles := constraint_invalid_bodies();
	body_handles[0] = a;
	body_handles[1] = b;
	body_handles[2] = c;
	body_handles[3] = d;
	return constraint_add_array(world, body_handles, description);
}

// constraint_get copies one active or sleeping built-in description into target
// allocation: none. ownership: owner thread while world idle
constraint_get :: #force_inline proc "contextless" (
	world: ^World,
	handle: Constraint_Handle,
	target: ^$T,
) -> Status
{
	if target == nil || constraint_type_id(T) == CONSTRAINT_TYPE_INVALID
	{
		return .Invalid_Argument;
	}
	_, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return ready_status;
	}
	return physics.simulation_get_constraint_description(
		simulation, handle, target,
	);
}

// constraint_apply replaces one built-in description. applying a sleeping
// constraint awakens its island through the low-level path
// ownership: owner thread while world idle
constraint_apply :: #force_inline proc (
	world: ^World,
	handle: Constraint_Handle,
	description: $T,
) -> Status
{
	if constraint_type_id(T) == CONSTRAINT_TYPE_INVALID
	{
		return .Invalid_Argument;
	}
	data, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return ready_status;
	}
	world_invalidate_views_data(data);
	stored_description := description;
	return physics.simulation_apply_constraint_description(
		simulation, handle, &stored_description,
	);
}

// constraint_remove removes one active or sleeping constraint. removing a
// sleeping constraint may awaken its island before removal
// ownership: owner thread while world idle
constraint_remove :: #force_inline proc (
	world: ^World,
	handle: Constraint_Handle,
) -> Status
{
	data, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return ready_status;
	}
	world_invalidate_views_data(data);
	return physics.simulation_remove_constraint(simulation, handle);
}

@(private)
constraint_info_empty :: #force_inline proc "contextless" () -> Constraint_Info
{
	return {
		handle=constraint_handle_invalid(),
		type_id=CONSTRAINT_TYPE_INVALID,
		bodies=constraint_invalid_bodies(),
		state=.Missing,
	};
}

@(private)
constraint_inspect_ready :: proc "contextless" (
	simulation: ^physics.Simulation,
	handle: Constraint_Handle,
) -> (Constraint_Info, Status)
{
	info := constraint_info_empty();
	if simulation == nil || simulation.state != .Ready
	{
		return info, .Disposed;
	}
	solver := &simulation.solver;
	if handle.value < 0 || int(handle.value) >= int(solver.handle_to_constraint.length)
	{
		return info, .Not_Found;
	}
	location := solver.handle_to_constraint.memory[handle.value];
	if location.set_index == 0
	{
		resolved, resolve_status := physics.solver_resolve(solver, handle);
		if resolve_status != .Ok
		{
			return info, resolve_status;
		}
		batch := &solver.active_set.batches.memory[resolved.batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[resolved.type_id]);
		if type_batch_index < 0 || type_batch_index >= int(batch.type_batch_count)
		{
			return info, .Not_Found;
		}
		type_batch := &batch.type_batches.memory[type_batch_index];
		reference, reference_status := physics.type_batch_read_reference(
			type_batch, solver.bodies, int(resolved.index_in_type_batch),
		);
		if reference_status != .Ok
		{
			return info, reference_status;
		}
		info.handle = handle;
		info.type_id = Constraint_Type_ID(resolved.type_id);
		info.body_count = u8(reference.body_count);
		info.state = .Active;
		for index in 0 ..< int(reference.body_count)
		{
			info.bodies[index] = reference.body_handles[index];
		}
		return info, .Ok;
	}
	if location.set_index > 0
	{
		record_index, resolve_status :=
			physics.island_sleeper_resolve_inactive_constraint_index(
			&simulation.sleeper, handle,
		);
		if resolve_status != .Ok
		{
			return info, resolve_status;
		}
		record := &simulation.sleeper.inactive_constraints.memory[record_index];
		info.handle = handle;
		info.type_id = Constraint_Type_ID(record.type_id);
		info.body_count = u8(record.body_count);
		info.state = .Inactive;
		for index in 0 ..< int(record.body_count)
		{
			info.bodies[index] = record.body_handles[index];
		}
		return info, .Ok;
	}
	return info, .Not_Found;
}

// constraint_inspect returns a pointer-free body/type/storage snapshot without
// exposing the solver's AoSoA layout. allocation: none
constraint_inspect :: proc "contextless" (
	world: ^World,
	handle: Constraint_Handle,
) -> (Constraint_Info, Status)
{
	_, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return constraint_info_empty(), ready_status;
	}
	return constraint_inspect_ready(simulation, handle);
}

// constraint_count returns the total number of active and sleeping constraints
constraint_count :: #force_inline proc "contextless" (
	world: ^World,
) -> (int, Status)
{
	_, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	return int(simulation.solver.active_set.constraint_count) +
		simulation.sleeper.inactive_constraint_count, .Ok;
}

// constraint_enumerate writes live constraints in ascending numeric handle
// order. if output is too small, it writes the prefix and returns total with
// Capacity_Missing. allocation: none
constraint_enumerate :: proc "contextless" (
	world: ^World,
	output: []Constraint_Info,
) -> (written, total: int, status: Status)
{
	_, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return 0, 0, ready_status;
	}
	total = int(simulation.solver.active_set.constraint_count) +
		simulation.sleeper.inactive_constraint_count;
	maximum_handle := min(
		int(simulation.solver.handle_pool.next_index),
		int(simulation.solver.handle_to_constraint.length),
	);
	for handle_value in 0 ..< maximum_handle
	{
		info, inspect_status := constraint_inspect_ready(
			simulation, {i32(handle_value)},
		);
		if inspect_status == .Not_Found
		{
			continue;
		}
		if inspect_status != .Ok
		{
			return written, total, inspect_status;
		}
		if written < len(output)
		{
			output[written] = info;
		}
		written += 1;
	}
	if written != total
	{
		return min(written, len(output)), total, .Invalid_Description;
	}
	if len(output) < total
	{
		return len(output), total, .Capacity_Missing;
	}
	return total, total, .Ok;
}

@(private)
constraint_batch_preflight :: proc (
	simulation: ^physics.Simulation,
	count: int,
) -> Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	if count < 0
	{
		return .Invalid_Argument;
	}
	if count == 0
	{
		return .Ok;
	}
	solver := &simulation.solver;
	available := solver.handle_pool.available_id_count;
	new_ids := max(count - available, 0);
	next_index := int(solver.handle_pool.next_index);
	if next_index > int(max(i32)) - new_ids
	{
		return .Capacity_Missing;
	}
	return physics.solver_ensure_handle_capacity(
		solver, max(next_index + new_ids, 1),
	);
}

@(private)
constraint_add_batch_prepare :: proc (
	world: ^World,
	description_count, handle_capacity: int,
) -> (^world_data, ^physics.Simulation, Status)
{
	if handle_capacity < description_count
	{
		return nil, nil, .Invalid_Argument;
	}
	data, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return nil, nil, ready_status;
	}
	if description_count == 0
	{
		return data, simulation, .Ok;
	}
	world_invalidate_views_data(data);
	preflight_status := constraint_batch_preflight(simulation, description_count);
	if preflight_status != .Ok
	{
		return nil, nil, preflight_status;
	}
	return data, simulation, .Ok;
}

// constraint_add_batch_1 adds homogeneous one-body constraints in input order
constraint_add_batch_1 :: proc (
	world: ^World,
	bodies: []Body_Handle,
	descriptions: []$T,
	handles: []Constraint_Handle,
) -> (written: int, status: Status)
{
	when T != One_Body_Angular_Servo && T != One_Body_Angular_Motor &&
		T != One_Body_Linear_Servo && T != One_Body_Linear_Motor
	{
		#assert(false, "constraint_add_batch_1 requires a one-body constraint description");
	}
	if constraint_type_id(T) == CONSTRAINT_TYPE_INVALID || len(bodies) != len(descriptions) ||
		len(handles) < len(descriptions)
	{
		return 0, .Invalid_Argument;
	}
	if len(descriptions) == 0
	{
		return 0, .Ok;
	}
	for index in 0 ..< len(descriptions)
	{
		handles[index] = constraint_handle_invalid();
	}
	_, simulation, prepare_status := constraint_add_batch_prepare(
		world, len(descriptions), len(handles),
	);
	if prepare_status != .Ok
	{
		return 0, prepare_status;
	}
	for index in 0 ..< len(descriptions)
	{
		body_handles := constraint_invalid_bodies();
		body_handles[0] = bodies[index];
		stored_description := descriptions[index];
		handle, add_status := physics.simulation_add_constraint(
			simulation, &body_handles, &stored_description,
		);
		if add_status != .Ok
		{
			return index, add_status;
		}
		handles[index] = handle;
	}
	return len(descriptions), .Ok;
}

// constraint_add_batch_2 adds homogeneous two-body constraints in input order
constraint_add_batch_2 :: proc (
	world: ^World,
	bodies: [][2]Body_Handle,
	descriptions: []$T,
	handles: []Constraint_Handle,
) -> (written: int, status: Status)
{
	when T == One_Body_Angular_Servo || T == One_Body_Angular_Motor ||
		T == One_Body_Linear_Servo || T == One_Body_Linear_Motor ||
		T == Area_Constraint || T == Volume_Constraint
	{
		#assert(false, "constraint_add_batch_2 requires a two-body constraint description");
	}
	if constraint_type_id(T) == CONSTRAINT_TYPE_INVALID || len(bodies) != len(descriptions) ||
		len(handles) < len(descriptions)
	{
		return 0, .Invalid_Argument;
	}
	if len(descriptions) == 0
	{
		return 0, .Ok;
	}
	for index in 0 ..< len(descriptions)
	{
		handles[index] = constraint_handle_invalid();
	}
	_, simulation, prepare_status := constraint_add_batch_prepare(
		world, len(descriptions), len(handles),
	);
	if prepare_status != .Ok
	{
		return 0, prepare_status;
	}
	for index in 0 ..< len(descriptions)
	{
		body_handles := constraint_invalid_bodies();
		body_handles[0] = bodies[index][0];
		body_handles[1] = bodies[index][1];
		stored_description := descriptions[index];
		handle, add_status := physics.simulation_add_constraint(
			simulation, &body_handles, &stored_description,
		);
		if add_status != .Ok
		{
			return index, add_status;
		}
		handles[index] = handle;
	}
	return len(descriptions), .Ok;
}

// constraint_add_batch_3 adds homogeneous three-body constraints in input order
constraint_add_batch_3 :: proc (
	world: ^World,
	bodies: [][3]Body_Handle,
	descriptions: []$T,
	handles: []Constraint_Handle,
) -> (written: int, status: Status)
{
	when T != Area_Constraint
	{
		#assert(false, "constraint_add_batch_3 requires a three-body constraint description");
	}
	if constraint_type_id(T) == CONSTRAINT_TYPE_INVALID || len(bodies) != len(descriptions) ||
		len(handles) < len(descriptions)
	{
		return 0, .Invalid_Argument;
	}
	if len(descriptions) == 0
	{
		return 0, .Ok;
	}
	for index in 0 ..< len(descriptions)
	{
		handles[index] = constraint_handle_invalid();
	}
	_, simulation, prepare_status := constraint_add_batch_prepare(
		world, len(descriptions), len(handles),
	);
	if prepare_status != .Ok
	{
		return 0, prepare_status;
	}
	for index in 0 ..< len(descriptions)
	{
		body_handles := constraint_invalid_bodies();
		for body_index in 0 ..< 3
		{
			body_handles[body_index] = bodies[index][body_index];
		}
		stored_description := descriptions[index];
		handle, add_status := physics.simulation_add_constraint(
			simulation, &body_handles, &stored_description,
		);
		if add_status != .Ok
		{
			return index, add_status;
		}
		handles[index] = handle;
	}
	return len(descriptions), .Ok;
}

// constraint_add_batch_4 adds homogeneous four-body constraints in input order
constraint_add_batch_4 :: proc (
	world: ^World,
	bodies: [][4]Body_Handle,
	descriptions: []$T,
	handles: []Constraint_Handle,
) -> (written: int, status: Status)
{
	when T != Volume_Constraint
	{
		#assert(false, "constraint_add_batch_4 requires a four-body constraint description");
	}
	if constraint_type_id(T) == CONSTRAINT_TYPE_INVALID || len(bodies) != len(descriptions) ||
		len(handles) < len(descriptions)
	{
		return 0, .Invalid_Argument;
	}
	if len(descriptions) == 0
	{
		return 0, .Ok;
	}
	for index in 0 ..< len(descriptions)
	{
		handles[index] = constraint_handle_invalid();
	}
	_, simulation, prepare_status := constraint_add_batch_prepare(
		world, len(descriptions), len(handles),
	);
	if prepare_status != .Ok
	{
		return 0, prepare_status;
	}
	for index in 0 ..< len(descriptions)
	{
		body_handles := bodies[index];
		stored_description := descriptions[index];
		handle, add_status := physics.simulation_add_constraint(
			simulation, &body_handles, &stored_description,
		);
		if add_status != .Ok
		{
			return index, add_status;
		}
		handles[index] = handle;
	}
	return len(descriptions), .Ok;
}

// constraint_apply_batch applies matching homogeneous descriptions in input
// order and returns the committed prefix
constraint_apply_batch :: proc (
	world: ^World,
	handles: []Constraint_Handle,
	descriptions: []$T,
) -> (applied: int, status: Status)
{
	if len(handles) != len(descriptions) || constraint_type_id(T) == CONSTRAINT_TYPE_INVALID
	{
		return 0, .Invalid_Argument;
	}
	if len(handles) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	for index in 0 ..< len(handles)
	{
		stored_description := descriptions[index];
		apply_status := physics.simulation_apply_constraint_description(
			simulation, handles[index], &stored_description,
		);
		if apply_status != .Ok
		{
			return index, apply_status;
		}
	}
	return len(handles), .Ok;
}

// constraint_remove_batch removes constraints in input order and returns the
// committed prefix
constraint_remove_batch :: proc (
	world: ^World,
	handles: []Constraint_Handle,
) -> (removed: int, status: Status)
{
	if len(handles) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := constraint_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	for index in 0 ..< len(handles)
	{
		remove_status := physics.simulation_remove_constraint(
			simulation, handles[index],
		);
		if remove_status != .Ok
		{
			return index, remove_status;
		}
	}
	return len(handles), .Ok;
}
