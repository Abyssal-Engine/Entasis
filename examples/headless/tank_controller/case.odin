package tank_controller

import entasis "entasis:entasis"

STEPS :: 420;
TIMESTEP :: f32(1.0 / 120.0);
Control :: enum
{
	Authored, Controlled,
}
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	properties: entasis.Collision_Property_Table,
	layer_rules: entasis.Layer_Matrix,
	materials: [1]entasis.Material,
	collision_policy: entasis.Layer_Material_Policy,
	pose_policy: entasis.Uniform_Gravity_Policy,
	bodies: [7]entasis.Body_Handle,
	motors: [6]entasis.Constraint_Handle,
	mounts: [6]entasis.Vector3,
	tick, constraint_count: int,
	position: entasis.Vector3,
	tracks: entasis.Vector2,
	control: Control,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = entasis.collision_property_init(&c.properties, 24, 4);
	if status != .Ok
	{
		return status;
	}
	ground_layer, vehicle_layer: entasis.Collision_Layer;
	ground_layer, _ = entasis.collision_layer(0);
	vehicle_layer, _ = entasis.collision_layer(1);
	c.layer_rules = entasis.layer_matrix_all();
	status = entasis.layer_matrix_deny(&c.layer_rules, vehicle_layer, vehicle_layer);
	if status != .Ok
	{
		return status;
	}
	c.materials[0] = entasis.material(1.6, 2, entasis.spring_settings(30, 1));
	material_id: entasis.Material_ID;
	material_id, _ = entasis.material_id(0);
	c.collision_policy = entasis.layer_material_policy(&c.properties, &c.layer_rules, entasis.material_table(c.materials[:]));
	c.pose_policy = entasis.uniform_gravity_policy({0, -9.81, 0}, 0.01, 0.01);
	description: entasis.World_Description = entasis.world_description_default();
	description.solve = entasis.solve_description_substeps(2, 6);
	status = entasis.world_description_set_callbacks(&description,
		entasis.narrow_policy_layers_materials(&c.collision_policy), entasis.pose_policy_uniform(&c.pose_policy));
	if status != .Ok
	{
		return status;
	}
	c.description = description;
	status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	floor_shape, chassis_shape, wheel_shape: entasis.Shape_Handle;
	floor_shape, status = entasis.shape_add(&c.world, entasis.box(80, 1, 40));
	if status != .Ok
	{
		return status;
	}
	floor: entasis.Static_Handle;
	floor, status = entasis.static_add(&c.world, entasis.static_body(floor_shape, entasis.pose({20, -0.5, 0})), .None);
	if status != .Ok
	{
		return status;
	}
	status = entasis.collision_property_set_static(&c.properties, floor,
		entasis.collision_properties(entasis.collision_filter(ground_layer, entasis.layer_mask_all()), material_id));
	if status != .Ok
	{
		return status;
	}
	chassis_shape, status = entasis.shape_add(&c.world, entasis.box(3.8, 0.9, 2.2));
	if status != .Ok
	{
		return status;
	}
	chassis_inertia, wheel_inertia: entasis.Body_Inertia;
	chassis_inertia, _ = entasis.shape_inertia(entasis.box(3.8, 0.9, 2.2), 12);
	c.bodies[0], status = entasis.body_add(&c.world, entasis.body_dynamic(chassis_shape, chassis_inertia,
		entasis.pose({0, 1.4, 0}), {}, entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	vehicle_properties: entasis.Collision_Properties = entasis.collision_properties(
		entasis.collision_filter(vehicle_layer, entasis.layer_mask_all()), material_id);
	status = entasis.collision_property_set_body(&c.properties, c.bodies[0], vehicle_properties);
	if status != .Ok
	{
		return status;
	}
	wheel_shape, status = entasis.shape_add(&c.world, entasis.sphere(0.48));
	if status != .Ok
	{
		return status;
	}
	wheel_inertia, _ = entasis.shape_inertia(entasis.sphere(0.48), 1);
	c.tracks = {1, 0.4};
	x_positions: [3]f32 = {-1.3, 0, 1.3};
	index: int;
	for side in -1 ..= 1
	{
		if side == 0
		{
			continue;
		}
		z: f32 = f32(side)*0.85;
		for x in x_positions
		{
			mount: entasis.Vector3 = {x, -0.35, z};
			c.mounts[index] = mount;
			wheel: entasis.Body_Handle;
			wheel, status = entasis.body_add(&c.world, entasis.body_dynamic(wheel_shape, wheel_inertia,
				entasis.pose({x, 0.55, z}), {}, entasis.body_activity(-1, 255)));
			if status != .Ok
			{
				return status;
			}
			c.bodies[index+1] = wheel;
			status = entasis.collision_property_set_body(&c.properties, wheel, vehicle_properties);
			if status != .Ok
			{
				return status;
			}
			_, status = entasis.constraint_add_2(&c.world, c.bodies[0], wheel,
				entasis.Linear_Axis_Servo{local_offset_a=mount, local_offset_b={}, local_plane_normal={0, -1, 0},
					target_offset=0.5, servo_settings=entasis.servo_settings(10, 0, 1600), spring_settings=entasis.spring_settings(8, 0.8)});
			if status != .Ok
			{
				return status;
			}
			_, status = entasis.constraint_add_2(&c.world, c.bodies[0], wheel,
				entasis.Point_On_Line_Servo{local_offset_a=mount, local_offset_b={}, local_direction={0, -1, 0},
					servo_settings=entasis.servo_settings(10, 0, 1600), spring_settings=entasis.spring_settings(30, 1)});
			if status != .Ok
			{
				return status;
			}
			_, status = entasis.constraint_add_2(&c.world, c.bodies[0], wheel,
				entasis.Angular_Hinge{local_hinge_axis_a={0, 0, 1}, local_hinge_axis_b={0, 0, 1}, spring_settings=entasis.spring_settings(30, 1)});
			if status != .Ok
			{
				return status;
			}
			target: f32 = -30 if side < 0 else -12;
			c.motors[index], status = entasis.constraint_add_2(&c.world, wheel, c.bodies[0],
				entasis.Angular_Axis_Motor{local_axis_a={0, 0, 1}, target_velocity=target, settings=entasis.motor_settings(6000, 1e-4)});
			if status != .Ok
			{
				return status;
			}
			index += 1;
		}
	}
	return .Ok;
}

case_control :: proc(c: ^Case, tracks: entasis.Vector2) -> entasis.Status
{
	if tracks == c.tracks
	{
		return .Ok;
	}
	for motor, index in c.motors
	{
		target: f32 = -30*(tracks.x if index < 3 else tracks.y);
		status: entasis.Status = entasis.constraint_apply(&c.world, motor,
			entasis.Angular_Axis_Motor{local_axis_a={0, 0, 1}, target_velocity=target, settings=entasis.motor_settings(6000, 1e-4)});
		if status != .Ok
		{
			return status;
		}
	}
	c.tracks, c.control = tracks, .Controlled;
	return .Ok;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = entasis.world_step(&c.world, TIMESTEP);
	if status == .Ok
	{
		c.tick += 1;
	}
	return status;
}

case_check :: proc(c: ^Case) -> entasis.Status
{
	state: entasis.Body_State;
	status: entasis.Status;
	state, status = entasis.body_get(&c.world, c.bodies[0]);
	if status != .Ok
	{
		return status;
	}
	c.position = state.pose.position;
	if c.control == .Authored && (c.position.x < 3 || abs(c.position.z) < 1 || c.position.y < 0.2)
	{
		return .Invalid_Argument;
	}
	c.constraint_count, status = entasis.body_constraint_count(&c.world, c.bodies[0]);
	return .Ok if status == .Ok && c.constraint_count >= 24 && c.tick == STEPS else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	entasis.collision_property_destroy(&c.properties);
	c^ = {};
}
