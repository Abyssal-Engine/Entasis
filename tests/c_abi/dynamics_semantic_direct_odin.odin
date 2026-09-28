package main

import "core:fmt"
import entasis "entasis:entasis"

check :: proc(status: entasis.Status, label: string)
{
	if status != .Ok
	{
		panic(label);
	}
}

float_bits :: #force_inline proc(value: f32) -> u32
{
	return transmute(u32)value;
}

unit_inertia :: proc() -> entasis.Body_Inertia
{
	return {
		inverse_inertia_tensor={xx=1, yy=1, zz=1},
		inverse_mass=1,
	};
}

body_at :: proc(x: f32) -> entasis.Body_Description
{
	return entasis.body_shapeless(
		unit_inertia(),
		entasis.pose({x, 0, 0}),
		entasis.velocity(),
		entasis.body_activity(0.01, 255),
	);
}

main :: proc()
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.threading.worker_count = 1;
	description.capacity.bodies = 32;
	description.capacity.constraints = 32;
	description.capacity.initial_constraints_per_type_batch = 8;
	description.capacity.minimum_constraints_per_body = 8;
	description.capacity.broad_phase_candidates = 64;
	description.capacity.pairs = 64;
	description.capacity.collision_child_pairs = 64;

	world: entasis.World;
	check(entasis.world_init(&world, description), "world_init");

	bodies: [2]entasis.Body_Handle;
	status: entasis.Status;
	bodies[0], status = entasis.body_add(&world, body_at(0));
	check(status, "body_a");
	bodies[1], status = entasis.body_add(&world, body_at(2));
	check(status, "body_b");

	ball_socket := entasis.Ball_Socket{
		local_offset_a={0.5, 0, 0},
		local_offset_b={-0.5, 0, 0},
		spring_settings=entasis.spring_settings(30, 1),
	};
	constraint, constraint_status := entasis.constraint_add_2(
		&world, bodies[0], bodies[1], ball_socket,
	);
	check(constraint_status, "constraint_add");
	ball_socket.local_offset_a.x = 0.25;
	ball_socket.local_offset_b.x = -0.25;
	check(entasis.constraint_apply(&world, constraint, ball_socket), "constraint_apply");

	check(entasis.body_apply_linear_impulse(&world, bodies[0], {2, 0.5, 0}), "linear_impulse");
	check(entasis.body_apply_angular_impulse(&world, bodies[1], {0, 0, 0.25}), "angular_impulse");
	check(entasis.body_apply_impulse(&world, bodies[0], {0, 0, 1}, {0, 1, 0}), "offset_impulse");

	stepper := entasis.fixed_stepper(1.0 / 60.0, 4);
	steps, alpha, step_status := entasis.fixed_stepper_update(&stepper, &world, 1.0 / 30.0);
	check(step_status, "fixed_step");

	state_a, state_a_status := entasis.body_get(&world, bodies[0]);
	check(state_a_status, "state_a");
	state_b, state_b_status := entasis.body_get(&world, bodies[1]);
	check(state_b_status, "state_b");
	info, info_status := entasis.constraint_inspect(&world, constraint);
	check(info_status, "constraint_info");
	impulses: [32]f32;
	impulse_written, impulse_required, impulse_status := entasis.constraint_accumulated_impulses(
		&world, constraint, impulses[:],
	);
	check(impulse_status, "constraint_impulses");
	magnitude, magnitude_status := entasis.constraint_accumulated_impulse_magnitude(&world, constraint);
	check(magnitude_status, "constraint_magnitude");

	fmt.printf(
		"solve steps=%d alpha=%d constraint=%d type=%d bodies=%d impulses=%d/%d magnitude=%d ax=%d ay=%d az=%d avx=%d avy=%d avz=%d bx=%d by=%d bz=%d bvx=%d bvy=%d bvz=%d\n",
		steps,
		float_bits(alpha),
		constraint.value,
		info.type_id,
		info.body_count,
		impulse_written,
		impulse_required,
		float_bits(magnitude),
		float_bits(state_a.pose.position.x),
		float_bits(state_a.pose.position.y),
		float_bits(state_a.pose.position.z),
		float_bits(state_a.velocity.linear.x),
		float_bits(state_a.velocity.linear.y),
		float_bits(state_a.velocity.linear.z),
		float_bits(state_b.pose.position.x),
		float_bits(state_b.pose.position.y),
		float_bits(state_b.pose.position.z),
		float_bits(state_b.velocity.linear.x),
		float_bits(state_b.velocity.linear.y),
		float_bits(state_b.velocity.linear.z),
	);

	applied_a := body_at(4);
	applied_a.velocity.linear = {3, 2, 1};
	added_c := body_at(-2);
	commands := [3]entasis.Command{
		entasis.command_body_apply(bodies[0], applied_a),
		entasis.command_body_add(added_c, 0),
		entasis.command_body_remove(bodies[1]),
	};
	statuses: [3]entasis.Status;
	body_results: [1]entasis.Body_Handle;
	buffer := entasis.command_buffer(commands[:], statuses[:], body_results[:]);
	processed, command_status := entasis.world_apply_commands(&world, &buffer);
	check(command_status, "commands");

	command_a, command_a_status := entasis.body_get(&world, bodies[0]);
	check(command_a_status, "command_a");
	command_c, command_c_status := entasis.body_get(&world, body_results[0]);
	check(command_c_status, "command_c");
	_, removed_status := entasis.body_get(&world, bodies[1]);
	stats, stats_status := entasis.world_stats(&world);
	check(stats_status, "stats");

	fmt.printf(
		"commands processed=%d statuses=%d,%d,%d result=%d removed=%d active=%d constraints=%d ax=%d avx=%d cx=%d cvx=%d\n",
		processed,
		statuses[0],
		statuses[1],
		statuses[2],
		body_results[0].value,
		removed_status,
		stats.active_bodies,
		stats.active_constraints,
		float_bits(command_a.pose.position.x),
		float_bits(command_a.velocity.linear.x),
		float_bits(command_c.pose.position.x),
		float_bits(command_c.velocity.linear.x),
	);

	if entasis.constraint_remove(&world, constraint) != .Not_Found
	{
		panic("constraint removed with body");
	}
	check(entasis.body_remove(&world, bodies[0]), "remove_a");
	check(entasis.body_remove(&world, body_results[0]), "remove_c");
	check(entasis.world_destroy(&world), "destroy");
}
