package materials_and_filtering

import entasis "entasis:entasis"

STEPS :: 365;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	properties: entasis.Collision_Property_Table,
	layer_rules: entasis.Layer_Matrix,
	materials: [1]entasis.Material,
	policy: entasis.Layer_Material_Policy,
	pose_policy: entasis.Uniform_Gravity_Policy,
	body: entasis.Body_Handle,
	cycle, tick: int,
	results: [4]f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = entasis.collision_property_init(&c.properties, 4, 4);
	if status != .Ok
	{
		return status;
	}
	layer: entasis.Collision_Layer;
	layer, _ = entasis.collision_layer(0);
	material_id: entasis.Material_ID;
	material_id, _ = entasis.material_id(0);
	friction: f32 = 0.05 if c.cycle == 0 else 1;
	recovery: f32 = 2;
	if c.cycle >= 2
	{
		friction = 0;
		recovery = 0.1 if c.cycle == 2 else 4;
	}
	c.materials[0] = entasis.material(friction, recovery, entasis.spring_settings(30, 1));
	c.layer_rules = entasis.layer_matrix_all();
	c.policy = entasis.layer_material_policy(&c.properties, &c.layer_rules, entasis.material_table(c.materials[:]));
	c.pose_policy = entasis.uniform_gravity_policy({0, -9.81, 0}, 0.01, 0.01);
	description: entasis.World_Description = entasis.world_description_default();
	if c.cycle >= 2
	{
		description.gravity = {};
		c.pose_policy = entasis.uniform_gravity_policy({});
	}
	status = entasis.world_description_set_callbacks(&description,
		entasis.narrow_policy_layers_materials(&c.policy), entasis.pose_policy_uniform(&c.pose_policy));
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
	floor_shape, shape: entasis.Shape_Handle;
	floor_shape, status = entasis.shape_add(&c.world, entasis.box(30, 1, 8) if c.cycle < 2 else entasis.box(8, 1, 8));
	if status != .Ok
	{
		return status;
	}
	floor: entasis.Static_Handle;
	floor, status = entasis.static_add(&c.world, entasis.static_body(floor_shape, entasis.pose({0, -0.5, 0})), .None);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	pose: entasis.Rigid_Pose = entasis.pose({-5, 0.51, 0});
	velocity: entasis.Body_Velocity = entasis.velocity({6, 0, 0});
	if c.cycle < 2
	{
		shape, status = entasis.shape_add(&c.world, entasis.box(1, 1, 1));
		inertia, _ = entasis.shape_inertia(entasis.box(1, 1, 1), 1);
	}
	else
	{
		shape, status = entasis.shape_add(&c.world, entasis.sphere(1));
		inertia, _ = entasis.shape_inertia(entasis.sphere(1), 1);
		pose, velocity = entasis.pose({0, 0.2, 0}), {};
	}
	if status != .Ok
	{
		return status;
	}
	c.body, status = entasis.body_add(&c.world, entasis.body_dynamic(shape, inertia, pose, velocity, entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	value: entasis.Collision_Properties = entasis.collision_properties(entasis.collision_filter(layer, entasis.layer_mask_all()), material_id);
	status = entasis.collision_property_set_static(&c.properties, floor, value);
	if status != .Ok
	{
		return status;
	}
	return entasis.collision_property_set_body(&c.properties, c.body, value);
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	limit: int = 180 if c.cycle < 2 else 1;
	if c.tick == limit
	{
		if c.cycle == 3
		{
			return .Invalid_Argument;
		}
		entasis.world_destroy(&c.world);
		entasis.collision_property_destroy(&c.properties);
		c.cycle += 1;
		c.tick = 0;
		return case_create(c);
	}
	status: entasis.Status = entasis.world_step(&c.world, TIMESTEP);
	if status != .Ok
	{
		return status;
	}
	c.tick += 1;
	if c.tick == limit
	{
		state: entasis.Body_State;
		state, status = entasis.body_get(&c.world, c.body);
		if status != .Ok
		{
			return status;
		}
		c.results[c.cycle] = state.pose.position.x if c.cycle < 2 else state.velocity.linear.y;
	}
	return .Ok;
}

case_check :: proc(c: ^Case) -> entasis.Status
{
	return .Ok if c.cycle == 3 && c.tick == 1 && c.results[0] > c.results[1]+1 && c.results[3] > c.results[2]+0.5 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	entasis.collision_property_destroy(&c.properties);
	c^ = {};
}
