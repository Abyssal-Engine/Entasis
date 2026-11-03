package solver_contacts

import "core:os"
import "core:simd"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Origin :: enum u32
{
	Accepted,
	R02_Pairs,
}
Status :: enum
{
	Ok,
	Invalid_Input,
	Unsupported,
	Allocation_Failed,
	IO_Error,
}
Contact_Input :: struct
{
	handle: i32,
	bodies: [2]i32,
	offsets: [4]util.Vector3,
	depths: [4]f32,
	offset_b: util.Vector3,
	normal: util.Vector3,
	material: physics.Contact_Material_Properties,
	tangent_impulse: util.Vector2,
	normal_impulses: [4]f32,
	twist_impulse: f32,
}
Type_Input :: struct
{
	type_id: i32,
	contacts: []Contact_Input,
}
Batch_Input :: struct
{
	types: []Type_Input,
}
Input :: struct
{
	origin: Origin,
	step: i32,
	dt: f32,
	solve: physics.Solve_Description,
	fallback_batch_threshold: i32,
	fallback_batch_index: i32,
	pose_context: physics.Default_Pose_Integrator_Context,
	dynamics: []physics.Body_Dynamics,
	body_handles: []i32,
	batches: []Batch_Input,
}
Counts :: struct
{
	bodies, manifolds, points, batches, fallback_manifolds: i64,
	type_counts: [8]i64,
}

input_counts :: proc(input: ^Input) -> Counts
{
	result: Counts = {bodies=i64(len(input.dynamics)), batches=i64(len(input.batches))};
	for batch, batch_index in input.batches
	{
		for kind in batch.types
		{
			count: i64 = i64(len(kind.contacts));
			result.manifolds += count;
			result.points += count * i64(kind.type_id % 4 + 1);
			if kind.type_id <= 7
			{
				result.type_counts[kind.type_id] += count;
			}
			if batch_index == int(input.fallback_batch_index)
			{
				result.fallback_manifolds += count;
			}
		}
	}
	return result;
}

input_destroy :: proc(input: ^Input)
{
	for batch in input.batches
	{
		for kind in batch.types
		{
			delete(kind.contacts);
		}
		delete(batch.types);
	}
	delete(input.batches);
	delete(input.dynamics);
	delete(input.body_handles);
	input^ = {};
}

contact_read :: proc(type_batch: ^physics.Type_Batch, index: int) -> Contact_Input
{
	contact_count: int = int(type_batch.type_id % 4 + 1);
	body_count: int = int(type_batch.body_count);
	lane: int = index % util.PRODUCTION_LANE_COUNT;
	prestep: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_prestep_bundle(type_batch, index));
	impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(type_batch, index));
	refs: [^]util.I32x8 = physics.type_batch_body_bundle(type_batch, index);
	result: Contact_Input = {handle=type_batch.index_to_handle.memory[index], bodies={-1, -1}};
	for body in 0 ..< body_count
	{
		result.bodies[body] = simd.extract(refs[body], lane);
	}
	for point in 0 ..< contact_count
	{
		result.offsets[point] = {simd.extract(prestep[4*point], lane), simd.extract(prestep[4*point+1], lane), simd.extract(prestep[4*point+2], lane)};
		result.depths[point] = simd.extract(prestep[4*point+3], lane);
		result.normal_impulses[point] = simd.extract(impulses[2+point], lane);
	}
	cursor: int = contact_count * 4;
	if body_count == 2
	{
		result.offset_b = {simd.extract(prestep[cursor], lane), simd.extract(prestep[cursor+1], lane), simd.extract(prestep[cursor+2], lane)};
		cursor += 3;
	}
	result.normal = {simd.extract(prestep[cursor], lane), simd.extract(prestep[cursor+1], lane), simd.extract(prestep[cursor+2], lane)};
	result.material = {
		friction_coefficient=simd.extract(prestep[cursor+3], lane),
		spring_settings={angular_frequency=simd.extract(prestep[cursor+4], lane), twice_damping_ratio=simd.extract(prestep[cursor+5], lane)},
		maximum_recovery_velocity=simd.extract(prestep[cursor+6], lane),
	};
	result.tangent_impulse = {simd.extract(impulses[0], lane), simd.extract(impulses[1], lane)};
	result.twist_impulse = simd.extract(impulses[2+contact_count], lane);
	return result;
}

input_capture :: proc(simulation: ^physics.Simulation, input: ^Input, origin: Origin, step: i32) -> Status
{
	when util.PRECISION_MODE != .Float_Coordinates || util.PRODUCTION_LANE_COUNT != 8
	{
		return .Unsupported;
	}
	if simulation.restitution != nil || simulation.body_control != nil || simulation.solver.joint_breaks != nil
	{
		return .Unsupported;
	}
	callbacks: physics.Pose_Integrator_Callbacks = simulation.integrator.callbacks;
	if callbacks.initialize != physics.pose_integrator_default_initialize ||
	callbacks.prepare_for_integration != physics.pose_integrator_default_prepare ||
	callbacks.integrate_velocity != physics.pose_integrator_default_velocity ||
	callbacks.dispose != physics.pose_integrator_default_dispose ||
	callbacks.user_context != &simulation.default_pose_context ||
	callbacks.angular_mode != .Nonconserving || callbacks.allow_substeps_for_unconstrained != .Disabled || callbacks.integrate_kinematic_velocity != .Disabled
	{
		return .Unsupported;
	}
	for set in simulation.bodies.sets.memory[1:int(simulation.bodies.sets.length)]
	{
		if set.count != 0
		{
			return .Unsupported;
		}
	}
	active: ^physics.Body_Set = &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	input^ = {
		origin=origin, step=step, dt=1.0/60,
		solve=simulation.solve_description,
		fallback_batch_threshold=simulation.solver.fallback_batch_threshold,
		fallback_batch_index=simulation.solver.fallback_batch_index,
		pose_context=simulation.default_pose_context,
	};
	input.dynamics = make([]physics.Body_Dynamics, active.count);
	input.body_handles = make([]i32, active.count);
	input.batches = make([]Batch_Input, simulation.solver.active_set.batch_count);
	for index in 0 ..< active.count
	{
		input.dynamics[index] = active.dynamics_state.memory[index];
		input.body_handles[index] = active.index_to_handle.memory[index].value;
		if active.activity.memory[index].sleep_threshold >= 0
		{
			input_destroy(input);
			return .Unsupported;
		}
	}
	for batch_index in 0 ..< len(input.batches)
	{
		batch: ^physics.Constraint_Batch = &simulation.solver.active_set.batches.memory[batch_index];
		input.batches[batch_index].types = make([]Type_Input, batch.type_batch_count);
		for type_index in 0 ..< int(batch.type_batch_count)
		{
			source: ^physics.Type_Batch = &batch.type_batches.memory[type_index];
			if source.count > 0 && (source.type_id < 0 || source.type_id > 7)
			{
				input_destroy(input);
				return .Unsupported;
			}
			kind: ^Type_Input = &input.batches[batch_index].types[type_index];
			kind.type_id = source.type_id;
			kind.contacts = make([]Contact_Input, source.count);
			for index in 0 ..< int(source.count)
			{
				kind.contacts[index] = contact_read(source, index);
			}
		}
	}
	return .Ok;
}

Input_Reader :: struct
{
	data: []u8,
	offset: int,
	status: Status,
}
Input_Writer :: struct
{
	data: []u8,
	offset: int,
}

read_word :: proc(reader: ^Input_Reader) -> u32
{
	if reader.status != .Ok || reader.offset > len(reader.data)-4
	{
		reader.status = .Invalid_Input;
		return 0;
	}
	cursor: int = reader.offset;
	reader.offset += 4;
	return u32(reader.data[cursor]) | u32(reader.data[cursor+1]) << 8 | u32(reader.data[cursor+2]) << 16 | u32(reader.data[cursor+3]) << 24;
}
read_scalar :: proc(reader: ^Input_Reader) -> f32
{
	return transmute(f32)read_word(reader);
}
read_vector :: proc(reader: ^Input_Reader) -> util.Vector3
{
	return {read_scalar(reader), read_scalar(reader), read_scalar(reader)};
}
write_word :: proc(writer: ^Input_Writer, value: u32)
{
	for index in 0 ..< 4
	{
		writer.data[writer.offset+index] = u8(value >> (uint(index)*8));
	}
	writer.offset += 4;
}
write_scalar :: proc(writer: ^Input_Writer, value: f32)
{
	write_word(writer, transmute(u32)value);
}
write_vector :: proc(writer: ^Input_Writer, value: util.Vector3)
{
	write_scalar(writer, value.x);
	write_scalar(writer, value.y);
	write_scalar(writer, value.z);
}

input_body_validate :: proc(body: ^physics.Body_Dynamics) -> Status
{
	values: [27]f32 = {body.motion.pose.orientation.x, body.motion.pose.orientation.y, body.motion.pose.orientation.z, body.motion.pose.orientation.w,
		body.motion.pose.position.x, body.motion.pose.position.y, body.motion.pose.position.z,
		body.motion.velocity.linear.x, body.motion.velocity.linear.y, body.motion.velocity.linear.z,
		body.motion.velocity.angular.x, body.motion.velocity.angular.y, body.motion.velocity.angular.z,
		body.inertia.local.inverse_inertia_tensor.xx, body.inertia.local.inverse_inertia_tensor.yx, body.inertia.local.inverse_inertia_tensor.yy,
		body.inertia.local.inverse_inertia_tensor.zx, body.inertia.local.inverse_inertia_tensor.zy, body.inertia.local.inverse_inertia_tensor.zz, body.inertia.local.inverse_mass,
		body.inertia.world.inverse_inertia_tensor.xx, body.inertia.world.inverse_inertia_tensor.yx, body.inertia.world.inverse_inertia_tensor.yy,
		body.inertia.world.inverse_inertia_tensor.zx, body.inertia.world.inverse_inertia_tensor.zy, body.inertia.world.inverse_inertia_tensor.zz, body.inertia.world.inverse_mass};
	for value in values
	{
		if physics.constraint_finite(value) == .Missing
		{
			return .Invalid_Input;
		}
	}
	description: physics.Body_Description = {pose=body.motion.pose, local_inertia=body.inertia.local};
	if body.inertia.world.inverse_mass < 0 || physics.body_description_validate(&description) != .Ok
	{
		return .Invalid_Input;
	}
	return .Ok;
}

read_body :: proc(reader: ^Input_Reader) -> physics.Body_Dynamics
{
	body: physics.Body_Dynamics;
	body.motion.pose.orientation = {read_scalar(reader), read_scalar(reader), read_scalar(reader), read_scalar(reader)};
	body.motion.pose.position = read_vector(reader);
	body.motion.velocity.linear = read_vector(reader);
	body.motion.velocity.angular = read_vector(reader);
	inertias: [2]^physics.Body_Inertia = {&body.inertia.local, &body.inertia.world};
	for inertia in inertias
	{
		inertia.inverse_inertia_tensor = {read_scalar(reader), read_scalar(reader), read_scalar(reader), read_scalar(reader), read_scalar(reader), read_scalar(reader)};
		inertia.inverse_mass = read_scalar(reader);
	}
	if input_body_validate(&body) != .Ok
	{
		reader.status = .Invalid_Input;
	}
	return body;
}
write_body :: proc(writer: ^Input_Writer, body: physics.Body_Dynamics)
{
	write_scalar(writer, body.motion.pose.orientation.x);
	write_scalar(writer, body.motion.pose.orientation.y);
	write_scalar(writer, body.motion.pose.orientation.z);
	write_scalar(writer, body.motion.pose.orientation.w);
	write_vector(writer, body.motion.pose.position);
	write_vector(writer, body.motion.velocity.linear);
	write_vector(writer, body.motion.velocity.angular);
	inertias: [2]physics.Body_Inertia = {body.inertia.local, body.inertia.world};
	for inertia in inertias
	{
		write_scalar(writer, inertia.inverse_inertia_tensor.xx);
		write_scalar(writer, inertia.inverse_inertia_tensor.yx);
		write_scalar(writer, inertia.inverse_inertia_tensor.yy);
		write_scalar(writer, inertia.inverse_inertia_tensor.zx);
		write_scalar(writer, inertia.inverse_inertia_tensor.zy);
		write_scalar(writer, inertia.inverse_inertia_tensor.zz);
		write_scalar(writer, inertia.inverse_mass);
	}
}

input_contact_validate :: proc(contact: ^Contact_Input, body_count, contact_count, body_limit, handle_limit: int) -> Status
{
	if contact.handle < 0 || int(contact.handle) >= handle_limit
	{
		return .Invalid_Input;
	}
	for body in 0 ..< body_count
	{
		encoded: u32 = u32(contact.bodies[body]);
		if contact.bodies[body] < 0 || encoded & physics.BODY_REFERENCE_INDEX_MASK >= u32(body_limit) || encoded & ~(physics.BODY_REFERENCE_INDEX_MASK | physics.BODY_REFERENCE_KINEMATIC_MASK) != 0
		{
			return .Invalid_Input;
		}
	}
	values: [7]f32 = {contact.material.friction_coefficient, contact.material.spring_settings.angular_frequency,
		contact.material.spring_settings.twice_damping_ratio, contact.material.maximum_recovery_velocity,
		contact.tangent_impulse.x, contact.tangent_impulse.y, contact.twist_impulse};
	for value in values
	{
		if physics.constraint_finite(value) == .Missing
		{
			return .Invalid_Input;
		}
	}
	if contact.material.friction_coefficient < 0 || contact.material.spring_settings.angular_frequency <= 0 ||
		contact.material.spring_settings.twice_damping_ratio < 0 || contact.material.maximum_recovery_velocity < 0 ||
		physics.constraint_unit_vector3(contact.normal) == .Missing ||
		(body_count == 2 && physics.constraint_vector3_finite(contact.offset_b) == .Missing)
	{
		return .Invalid_Input;
	}
	for point in 0 ..< contact_count
	{
		if physics.constraint_vector3_finite(contact.offsets[point]) == .Missing ||
			physics.constraint_finite(contact.depths[point]) == .Missing || physics.constraint_finite(contact.normal_impulses[point]) == .Missing
		{
			return .Invalid_Input;
		}
	}
	return .Ok;
}

read_contact :: proc(reader: ^Input_Reader, body_count, contact_count, body_limit: int) -> Contact_Input
{
	contact: Contact_Input = {handle=i32(read_word(reader)), bodies={-1, -1}};
	for body in 0 ..< body_count
	{
		contact.bodies[body] = i32(read_word(reader));
	}
	for point in 0 ..< contact_count
	{
		contact.offsets[point] = read_vector(reader);
		contact.depths[point] = read_scalar(reader);
	}
	if body_count == 2
	{
		contact.offset_b = read_vector(reader);
	}
	contact.normal = read_vector(reader);
	contact.material = {
		friction_coefficient=read_scalar(reader),
		spring_settings={angular_frequency=read_scalar(reader), twice_damping_ratio=read_scalar(reader)},
		maximum_recovery_velocity=read_scalar(reader),
	};
	contact.tangent_impulse = {read_scalar(reader), read_scalar(reader)};
	for point in 0 ..< contact_count
	{
		contact.normal_impulses[point] = read_scalar(reader);
	}
	contact.twist_impulse = read_scalar(reader);
	if input_contact_validate(&contact, body_count, contact_count, body_limit, len(reader.data)/4) != .Ok
	{
		reader.status = .Invalid_Input;
	}
	return contact;
}
write_contact :: proc(writer: ^Input_Writer, contact: Contact_Input, body_count, contact_count: int)
{
	write_word(writer, u32(contact.handle));
	for body in 0 ..< body_count
	{
		write_word(writer, u32(contact.bodies[body]));
	}
	for point in 0 ..< contact_count
	{
		write_vector(writer, contact.offsets[point]);
		write_scalar(writer, contact.depths[point]);
	}
	if body_count == 2
	{
		write_vector(writer, contact.offset_b);
	}
	write_vector(writer, contact.normal);
	write_scalar(writer, contact.material.friction_coefficient);
	write_scalar(writer, contact.material.spring_settings.angular_frequency);
	write_scalar(writer, contact.material.spring_settings.twice_damping_ratio);
	write_scalar(writer, contact.material.maximum_recovery_velocity);
	write_scalar(writer, contact.tangent_impulse.x);
	write_scalar(writer, contact.tangent_impulse.y);
	for point in 0 ..< contact_count
	{
		write_scalar(writer, contact.normal_impulses[point]);
	}
	write_scalar(writer, contact.twist_impulse);
}

input_byte_count :: proc(input: ^Input) -> int
{
	words: int = 21 + 28*len(input.dynamics);
	for batch in input.batches
	{
		words += 1;
		for kind in batch.types
		{
			words += 2 + len(kind.contacts)*(8+4*(1+int(kind.type_id/4))+5*int(kind.type_id%4+1));
		}
	}
	return words*4;
}

// encoding consumes the input shape admitted before publication
input_encode :: proc(input: ^Input) -> []u8
{
	writer: Input_Writer = {data=make([]u8, input_byte_count(input))};
	write_word(&writer, 0x41544e45);
	write_word(&writer, 0x544e4f43);
	write_word(&writer, 32);
	write_word(&writer, 8);
	write_word(&writer, u32(input.origin));
	write_word(&writer, u32(input.step));
	write_scalar(&writer, input.dt);
	write_word(&writer, u32(input.solve.substep_count));
	write_word(&writer, u32(input.solve.velocity_iteration_count));
	write_word(&writer, u32(input.fallback_batch_threshold));
	write_word(&writer, u32(input.fallback_batch_index));
	write_word(&writer, u32(physics.Angular_Integration_Mode.Nonconserving));
	write_word(&writer, u32(physics.Velocity_Integration_State.Disabled));
	write_word(&writer, u32(physics.Velocity_Integration_State.Disabled));
	write_vector(&writer, input.pose_context.gravity);
	write_scalar(&writer, input.pose_context.linear_damping);
	write_scalar(&writer, input.pose_context.angular_damping);
	write_word(&writer, u32(len(input.dynamics)));
	write_word(&writer, u32(len(input.batches)));
	for body, index in input.dynamics
	{
		write_word(&writer, u32(input.body_handles[index]));
		write_body(&writer, body);
	}
	for batch in input.batches
	{
		write_word(&writer, u32(len(batch.types)));
		for kind in batch.types
		{
			write_word(&writer, u32(kind.type_id));
			write_word(&writer, u32(len(kind.contacts)));
			for contact in kind.contacts
			{
				write_contact(&writer, contact, 1+int(kind.type_id/4), int(kind.type_id%4+1));
			}
		}
	}
	return writer.data;
}

input_scan :: proc(data: []u8, output: ^Input) -> Status
{
	reader: Input_Reader = {data=data};
	if read_word(&reader) != 0x41544e45 || read_word(&reader) != 0x544e4f43
	{
		return .Invalid_Input;
	}
	if read_word(&reader) != 32 || read_word(&reader) != 8
	{
		return .Unsupported;
	}
	input: Input;
	input.origin = Origin(read_word(&reader));
	input.step = i32(read_word(&reader));
	input.dt = read_scalar(&reader);
	input.solve.substep_count = i32(read_word(&reader));
	input.solve.velocity_iteration_count = i32(read_word(&reader));
	input.fallback_batch_threshold = i32(read_word(&reader));
	input.fallback_batch_index = i32(read_word(&reader));
	input.solve.fallback_batch_threshold = input.fallback_batch_threshold;
	angular: u32 = read_word(&reader);
	unconstrained: u32 = read_word(&reader);
	kinematic: u32 = read_word(&reader);
	input.pose_context.gravity = read_vector(&reader);
	input.pose_context.linear_damping = read_scalar(&reader);
	input.pose_context.angular_damping = read_scalar(&reader);
	body_count: int = int(read_word(&reader));
	batch_count: int = int(read_word(&reader));
	if reader.status != .Ok || u32(input.origin) > 1 || input.step <= 0 || input.dt != f32(1.0/60) || input.solve.substep_count != 4 || input.solve.velocity_iteration_count != 1 || angular != 0 || unconstrained != 0 || kinematic != 0 || input.pose_context.gravity != (util.Vector3{0, -10, 0}) || input.pose_context.linear_damping != 0 || input.pose_context.angular_damping != 0
	{
		return .Unsupported;
	}
	if body_count <= 0 || body_count > (len(data)-reader.offset)/112 || batch_count <= 0 || batch_count > (len(data)-reader.offset)/4 || input.fallback_batch_threshold <= 0 || input.fallback_batch_threshold > 64 || input.fallback_batch_index != input.fallback_batch_threshold || batch_count > int(input.fallback_batch_index)+1
	{
		return .Invalid_Input;
	}
	if output != nil
	{
		output^ = input;
		output.dynamics = make([]physics.Body_Dynamics, body_count);
		output.body_handles = make([]i32, body_count);
		output.batches = make([]Batch_Input, batch_count);
	}
	for index in 0 ..< body_count
	{
		handle: i32 = i32(read_word(&reader));
		body: physics.Body_Dynamics = read_body(&reader);
		if handle < 0 || int(handle) >= len(data)/4
		{
			return .Invalid_Input;
		}
		if output != nil
		{
			output.body_handles[index] = handle;
			output.dynamics[index] = body;
		}
	}
	points: i64;
	for batch_index in 0 ..< batch_count
	{
		type_count: int = int(read_word(&reader));
		if reader.status != .Ok || type_count < 0 || type_count > 18 || type_count > (len(data)-reader.offset)/8
		{
			return .Invalid_Input;
		}
		if output != nil
		{
			output.batches[batch_index].types = make([]Type_Input, type_count);
		}
		type_mask: u32;
		for type_index in 0 ..< type_count
		{
			type_id: i32 = i32(read_word(&reader));
			count: int = int(read_word(&reader));
			if reader.status != .Ok || type_id < 0 || type_id > 17 || (type_id > 10 && type_id < 15) || (type_id > 7 && count > 0) || count < 0 || type_mask & (1 << uint(type_id)) != 0
			{
				return .Unsupported;
			}
			type_mask |= 1 << uint(type_id);
			body_fields: int = 1+int(type_id/4);
			point_fields: int = int(type_id%4+1);
			if count > (len(data)-reader.offset)/(4*(8+4*body_fields+5*point_fields))
			{
				return .Invalid_Input;
			}
			points += i64(count)*i64(point_fields);
			if output != nil
			{
				output.batches[batch_index].types[type_index] = {type_id=type_id, contacts=make([]Contact_Input, count)};
			}
			for index in 0 ..< count
			{
				contact: Contact_Input = read_contact(&reader, body_fields, point_fields, body_count);
				if reader.status != .Ok
				{
					return reader.status;
				}
				if output != nil
				{
					output.batches[batch_index].types[type_index].contacts[index] = contact;
				}
			}
		}
	}
	if reader.status != .Ok || reader.offset != len(data) || points <= 0
	{
		return .Invalid_Input;
	}
	return .Ok;
}

input_validate :: proc(input: ^Input) -> Status
{
	if len(input.dynamics) == 0 || len(input.dynamics) != len(input.body_handles) || len(input.batches) == 0 || input.solve.velocity_iteration_scheduler != nil
	{
		return .Unsupported;
	}
	if u32(input.origin) > 1 || input.step <= 0 || input.dt != f32(1.0/60) || input.solve.substep_count != 4 ||
		input.solve.velocity_iteration_count != 1 || input.pose_context.gravity != (util.Vector3{0, -10, 0}) ||
		input.pose_context.linear_damping != 0 || input.pose_context.angular_damping != 0
	{
		return .Unsupported;
	}
	if input.fallback_batch_threshold <= 0 || input.fallback_batch_threshold > 64 ||
		input.fallback_batch_index != input.fallback_batch_threshold || len(input.batches) > int(input.fallback_batch_index)+1
	{
		return .Invalid_Input;
	}
	for batch in input.batches
	{
		if len(batch.types) > 18
		{
			return .Invalid_Input;
		}
		type_mask: u32;
		for kind in batch.types
		{
			if kind.type_id < 0 || kind.type_id > 17 || (kind.type_id > 10 && kind.type_id < 15) ||
				(kind.type_id > 7 && len(kind.contacts) > 0) || type_mask & (1 << uint(kind.type_id)) != 0
			{
				return .Unsupported;
			}
			type_mask |= 1 << uint(kind.type_id);
		}
	}
	bytes: int = input_byte_count(input);
	for &body, index in input.dynamics
	{
		handle: i32 = input.body_handles[index];
		if handle < 0 || int(handle) >= bytes/4 || input_body_validate(&body) != .Ok
		{
			return .Invalid_Input;
		}
	}
	points: i64;
	for batch in input.batches
	{
		for kind in batch.types
		{
			body_count: int = 1+int(kind.type_id/4);
			contact_count: int = int(kind.type_id%4+1);
			points += i64(len(kind.contacts))*i64(contact_count);
			for &contact in kind.contacts
			{
				if input_contact_validate(&contact, body_count, contact_count, len(input.dynamics), bytes/4) != .Ok
				{
					return .Invalid_Input;
				}
			}
		}
	}
	if points <= 0
	{
		return .Invalid_Input;
	}
	return input_validate_graph(input, bytes/4);
}

input_validate_graph :: proc(input: ^Input, handle_limit: int) -> Status
{
	handles: []u8 = make([]u8, handle_limit);
	defer delete(handles);
	for handle in input.body_handles
	{
		if handles[handle] != 0
		{
			return .Invalid_Input;
		}
		handles[handle] = 1;
	}
	for index in 0 ..< len(handles)
	{
		handles[index] = 0;
	}
	for batch, batch_index in input.batches
	{
		seen: []u8 = make([]u8, len(input.dynamics));
		defer delete(seen);
		for kind in batch.types
		{
			for contact in kind.contacts
			{
				if handles[contact.handle] != 0
				{
					return .Invalid_Input;
				}
				handles[contact.handle] = 1;
				for body in 0 ..< 1+int(kind.type_id/4)
				{
					index: int = int(u32(contact.bodies[body]) & physics.BODY_REFERENCE_INDEX_MASK);
					mobility: u32 = u32(contact.bodies[body]) & physics.BODY_REFERENCE_KINEMATIC_MASK;
					if (input.dynamics[index].inertia.local.inverse_mass == 0) != (mobility != 0) || (mobility == 0 && batch_index < int(input.fallback_batch_index) && seen[index] != 0)
					{
						return .Invalid_Input;
					}
					seen[index] = 1;
				}
			}
		}
	}
	return .Ok;
}

input_decode :: proc(data: []u8, output: ^Input) -> Status
{
	when util.PRECISION_MODE != .Float_Coordinates || util.PRODUCTION_LANE_COUNT != 8
	{
		return .Unsupported;
	}
	status: Status = input_scan(data, nil);
	if status != .Ok
	{
		return status;
	}
	status = input_scan(data, output);
	if status == .Ok
	{
		status = input_validate_graph(output, len(data)/4);
	}
	if status != .Ok
	{
		input_destroy(output);
	}
	return status;
}

input_read :: proc(path: string, output: ^Input) -> Status
{
	data: []u8;
	error: os.Error;
	data, error = os.read_entire_file(path, context.allocator);
	if error != nil
	{
		return .IO_Error;
	}
	defer delete(data);
	return input_decode(data, output);
}
input_write :: proc(path: string, input: ^Input) -> Status
{
	status: Status = input_validate(input);
	if status != .Ok
	{
		return status;
	}
	data: []u8 = input_encode(input);
	defer delete(data);
	file: ^os.File;
	error: os.Error;
	file, error = os.open(path, {.Write, .Create, .Excl});
	if error != nil
	{
		return .IO_Error;
	}
	defer os.close(file);
	written: int;
	written, error = os.write(file, data);
	if error != nil || written != len(data)
	{
		return .IO_Error;
	}
	return .Ok;
}
