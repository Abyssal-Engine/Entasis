package ragdoll

import "core:math"
import entasis "entasis:entasis"

STEPS :: 240;
TIMESTEP :: f32(1.0 / 60.0);
LINKS :: [5][2]int{{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}};
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	properties: entasis.Collision_Property_Table,
	layer_rules: entasis.Layer_Matrix,
	materials: [1]entasis.Material,
	policy: entasis.Layer_Material_Policy,
	pose_policy: entasis.Uniform_Gravity_Policy,
	bodies: [6]entasis.Body_Handle,
	offsets_a, offsets_b: [5]entasis.Vector3,
	tick, constraint_count: int,
	maximum_error: f32,
}

rotate_vector :: #force_inline proc "contextless" (
	orientation: entasis.Quaternion, value: entasis.Vector3,
) -> entasis.Vector3
{
	t := entasis.Vector3{
		2 * (orientation.y * value.z - orientation.z * value.y),
		2 * (orientation.z * value.x - orientation.x * value.z),
		2 * (orientation.x * value.y - orientation.y * value.x),
	};
	cross := entasis.Vector3{
		orientation.y * t.z - orientation.z * t.y,
		orientation.z * t.x - orientation.x * t.z,
		orientation.x * t.y - orientation.y * t.x,
	};
	return {
		value.x + orientation.w * t.x + cross.x,
		value.y + orientation.w * t.y + cross.y,
		value.z + orientation.w * t.z + cross.z,
	};
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = entasis.collision_property_init(&c.properties, 16, 4);
	if status != .Ok
	{
		return status;
	}
	ragdoll_layer, ground_layer: entasis.Collision_Layer;
	ragdoll_layer, _ = entasis.collision_layer(1);
	ground_layer, _ = entasis.collision_layer(2);
	c.layer_rules = entasis.layer_matrix_none();
	status = entasis.layer_matrix_allow(&c.layer_rules, ragdoll_layer, ground_layer);
	if status != .Ok
	{
		return status;
	}
	c.materials[0] = entasis.material_default();
	c.policy = entasis.layer_material_policy(&c.properties, &c.layer_rules, entasis.material_table(c.materials[:]));
	description: entasis.World_Description = entasis.world_description_default();
	description.solve = entasis.solve_description_substeps(2, 6);
	c.pose_policy = entasis.uniform_gravity_policy(description.gravity, description.damping.linear, description.damping.angular);
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
	body_shape, ground_shape: entasis.Shape_Handle;
	body_shape, status = entasis.shape_add(&c.world, entasis.sphere(0.25));
	if status != .Ok
	{
		return status;
	}
	ground_shape, status = entasis.shape_add(&c.world, entasis.box(20, 1, 20));
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_registered_inertia(&c.world, body_shape, 2);
	if status != .Ok
	{
		return status;
	}
	positions: [6]entasis.Vector3 = {
		{0, 5, 0}, {0, 5.8, 0}, {-0.8, 5, 0}, {0.8, 5, 0},
		{-0.35, 4, 0}, {0.35, 4, 0},
	};
	material_id: entasis.Material_ID;
	material_id, _ = entasis.material_id(0);
	ragdoll_filter: entasis.Collision_Filter = entasis.collision_filter(ragdoll_layer, entasis.layer_mask(ground_layer));
	for &body, index in c.bodies
	{
		body, status = entasis.body_add(&c.world, entasis.body_dynamic(body_shape, inertia,
			entasis.pose(positions[index]), {}, entasis.body_activity(-1, 255)));
		if status != .Ok
		{
			return status;
		}
		status = entasis.collision_property_set_body(&c.properties, body, entasis.collision_properties(ragdoll_filter, material_id));
		if status != .Ok
		{
			return status;
		}
	}
	ground: entasis.Static_Handle;
	ground, status = entasis.static_add(&c.world, entasis.static_body(ground_shape, entasis.pose({0, -0.5, 0})), .None);
	if status != .Ok
	{
		return status;
	}
	ground_filter: entasis.Collision_Filter = entasis.collision_filter(ground_layer, entasis.layer_mask(ragdoll_layer));
	status = entasis.collision_property_set_static(&c.properties, ground, entasis.collision_properties(ground_filter, material_id));
	if status != .Ok
	{
		return status;
	}
	for link, index in LINKS
	{
		a: entasis.Vector3 = positions[link[0]];
		b: entasis.Vector3 = positions[link[1]];
		delta: entasis.Vector3 = {b.x-a.x, b.y-a.y, b.z-a.z};
		length: f32 = math.sqrt(delta.x*delta.x+delta.y*delta.y+delta.z*delta.z);
		axis: entasis.Vector3 = {delta.x/length, delta.y/length, delta.z/length};
		c.offsets_a[index] = {delta.x*0.5, delta.y*0.5, delta.z*0.5};
		c.offsets_b[index] = {-delta.x*0.5, -delta.y*0.5, -delta.z*0.5};
		_, status = entasis.constraint_add_2(&c.world, c.bodies[link[0]], c.bodies[link[1]],
			entasis.Ball_Socket{local_offset_a=c.offsets_a[index], local_offset_b=c.offsets_b[index], spring_settings=entasis.spring_settings(35, 1)});
		if status != .Ok
		{
			return status;
		}
		_, status = entasis.constraint_add_2(&c.world, c.bodies[link[0]], c.bodies[link[1]],
			entasis.Swing_Limit{axis_local_a=axis, axis_local_b=axis, minimum_dot=0.2, spring_settings=entasis.spring_settings(25, 1)});
		if status != .Ok
		{
			return status;
		}
		_, status = entasis.constraint_add_2(&c.world, c.bodies[link[0]], c.bodies[link[1]],
			entasis.Twist_Limit{local_basis_a={w=1}, local_basis_b={w=1}, minimum_angle=-0.65, maximum_angle=0.65, spring_settings=entasis.spring_settings(25, 1)});
		if status != .Ok
		{
			return status;
		}
	}
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
	c.maximum_error = 0;
	for link, index in LINKS
	{
		a, b: entasis.Body_State;
		status: entasis.Status;
		a, status = entasis.body_get(&c.world, c.bodies[link[0]]);
		if status != .Ok
		{
			return status;
		}
		b, status = entasis.body_get(&c.world, c.bodies[link[1]]);
		if status != .Ok
		{
			return status;
		}
		world_offset_a: entasis.Vector3 = rotate_vector(a.pose.orientation, c.offsets_a[index]);
		world_offset_b: entasis.Vector3 = rotate_vector(b.pose.orientation, c.offsets_b[index]);
		dx: f32 = b.pose.position.x+world_offset_b.x-a.pose.position.x-world_offset_a.x;
		dy: f32 = b.pose.position.y+world_offset_b.y-a.pose.position.y-world_offset_a.y;
		dz: f32 = b.pose.position.z+world_offset_b.z-a.pose.position.z-world_offset_a.z;
		c.maximum_error = max(c.maximum_error, math.sqrt(dx*dx+dy*dy+dz*dz));
	}
	infos: [64]entasis.Constraint_Info;
	required: int;
	status: entasis.Status;
	c.constraint_count, required, status = entasis.constraint_enumerate(&c.world, infos[:]);
	if status != .Ok || c.constraint_count < len(LINKS)*3 || required < len(LINKS)*3 || c.maximum_error > 0.3
	{
		return .Invalid_Argument;
	}
	return .Ok if c.tick == STEPS else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	entasis.collision_property_destroy(&c.properties);
	c^ = {};
}
