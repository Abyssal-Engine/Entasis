package main

import "core:fmt"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

Payload :: struct
{
	child: e.Shape_Handle, radius: f32, tag: u32
}

State :: struct
{
	disposed: int
}

require :: proc(status: e.Status)
{
	if status != .Ok
	{
		panic(fmt.tprintf("custom shape semantic status: %v", status));
	}
}

bounds :: proc "contextless" (ctx, raw: rawptr, q: ^u.Quaternion, access: e.Shape_Access, out: ^e.Shape_Bounds) -> e.Status
{
	_ = ctx;
	value, status := e.shape_access_compute_bounds(access, (^Payload)(raw).child, q^);
	out^ = value;
	return status;
}

inertia :: proc "contextless" (ctx, raw: rawptr, mass: f32, access: e.Shape_Access, out: ^e.Body_Inertia) -> e.Status
{
	_ = ctx;
	value, status := e.shape_access_compute_inertia(access, (^Payload)(raw).child, mass);
	out^ = value;
	return status;
}

ray :: proc "contextless" (ctx, raw: rawptr, pose: ^e.Rigid_Pose, input: ^p.Tree_Ray, access: e.Shape_Access, out: ^e.Shape_Ray_Hit) -> e.Status
{
	_ = ctx;
	value, status := e.shape_access_ray_test(access, (^Payload)(raw).child, pose^, input^);
	out^ = value;
	return status;
}

support :: proc "contextless" (ctx, raw: rawptr, direction: ^u.Vector3, access: e.Shape_Access, out: ^u.Vector3) -> e.Status
{
	_ = ctx;
	value, status := e.shape_access_support(access, (^Payload)(raw).child, direction^);
	out^ = value;
	return status;
}

dispose :: proc "contextless" (ctx, raw: rawptr, access: e.Shape_Access, pool: ^u.Buffer_Pool) -> e.Status
{
	_, _, _ = raw, access, pool;
	(^State)(ctx).disposed += 1;
	return .Ok;
}

main :: proc()
{
	for w in 0 ..< 2
	{
		world: e.World;
		description := e.world_description_default();
		description.threading.worker_count = 1;
		description.gravity = {};
		description.capacity.bodies = 32;
		description.capacity.statics = 16;
		description.capacity.shapes_per_type = 4;
		description.capacity.constraints = 32;
		description.capacity.pairs = 128;
		description.capacity.broad_phase_candidates = 128;
		require(e.world_init(&world, description));
		state: State;
		registration := e.custom_shape_registration_contextual(Payload, .Convex, &state,
			bounds, inertia, ray, support, support, dispose);
		registration.alignment = 32;
		type_id, status := e.custom_shape_register_contextual(&world, registration);
		require(status);
		registration = {};
		input := Payload{radius=f32(w+1), tag=u32(123+w)};
		input.child, status = e.shape_add(&world, e.sphere(input.radius));
		require(status);
		shape, shape_status := e.custom_shape_add_raw(&world, type_id, &input, size_of(input));
		require(shape_status);
		data, data_status := e.custom_shape_data(&world, shape);
		require(data_status);
		output := (^Payload)(data.data)^;
		body_inertia, inertia_status := e.custom_shape_inertia(&world, shape, 2);
		require(inertia_status);
		_, status = e.static_add(&world, e.static_body(shape, e.pose()), .None);
		require(status);
		query: e.Query_Context;
		require(e.query_context_init(&query, &world));
		hit, hit_status := e.ray_cast_closest_with_context(&query, {origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=10});
		require(hit_status);
		fmt.printf("world=%d type=%d tag=%d bytes=%d mass=%d tensor=%d ray=%d normal=%d\n", w, i32(type_id), output.tag,
			data.size, transmute(u32)body_inertia.inverse_mass, transmute(u32)body_inertia.inverse_inertia_tensor.xx,
			transmute(u32)hit.t, transmute(u32)hit.normal.x);
		require(e.query_context_destroy(&query));
		require(e.world_destroy(&world));
		fmt.printf("disposed=%d\n", state.disposed);
	}
}
