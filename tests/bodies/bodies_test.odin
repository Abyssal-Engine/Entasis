package bodies_tests

import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 18
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 65536, power), util.Memory_Status.Ok);
	}
}

exhaust_i32_1024 :: proc(pool: ^util.Buffer_Pool, buffers: ^[32]util.Buffer(i32)) -> int
{
	count := 0;
	for count < len(buffers)
	{
		buffer, status := util.buffer_pool_take_at_least(pool, i32, 1024);
		if status != .Ok
		{
			break;
		}
		buffers[count] = buffer;
		count += 1;
	}
	return count;
}

return_i32_buffers :: proc(pool: ^util.Buffer_Pool, buffers: ^[32]util.Buffer(i32), count: int)
{
	for index in 0 ..< count
	{
		_ = util.buffer_pool_return(pool, &buffers[index]);
	}
}

dynamic_description :: proc(index: int, shape: physics.Typed_Index = {}) -> physics.Body_Description
{
	return {
		pose={
			orientation=util.quaternion_identity(),
			position={f32(index), f32(index * 2), f32(-index)},
		},
		velocity={
			linear={f32(index + 1), f32(index + 2), f32(index + 3)},
			angular={f32(index + 4), f32(index + 5), f32(index + 6)},
		},
		local_inertia={
			inverse_inertia_tensor={1, 0, 2, 0, 0, 3},
			inverse_mass=0.5,
		},
		collidable={
			shape=shape,
			continuity=physics.continuous_detection_discrete(),
			minimum_speculative_margin=0,
			maximum_speculative_margin=10,
		},
		activity={sleep_threshold=0.01, minimum_timestep_count_under_threshold=32},
	};
}

@(test)
typed_indices_preserve_source_packing_and_invalid_sentinel :: proc(t: ^testing.T)
{
	index, status := physics.typed_index_create(37, 0x00ab_cdef);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.typed_index_state(index), physics.Reference_State.Present);
	testing.expect_value(t, physics.typed_index_type(index), i32(37));
	testing.expect_value(t, physics.typed_index_index(index), i32(0x00ab_cdef));
	testing.expect_value(t, physics.typed_index_state({}), physics.Reference_State.Missing);
	_, invalid_type := physics.typed_index_create(128, 0);
	_, invalid_index := physics.typed_index_create(0, 1 << 24);
	testing.expect_value(t, invalid_type, physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, invalid_index, physics.Physics_Status.Invalid_Argument);
}

@(test)
body_lifecycle_preserves_dense_locations_set_moves_and_source_handle_reuse :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	testing.expect_value(t, physics.bodies_initialize(&bodies, 4, 4, 1, &pool), physics.Physics_Status.Ok);
	defer physics.bodies_dispose(&bodies);
	description0 := dynamic_description(0);
	description1 := dynamic_description(1);
	description2 := dynamic_description(2);
	handle0, add0 := physics.bodies_add(&bodies, &description0);
	handle1, add1 := physics.bodies_add(&bodies, &description1);
	handle2, add2 := physics.bodies_add(&bodies, &description2);
	testing.expect_value(t, add0, physics.Physics_Status.Ok);
	testing.expect_value(t, add1, physics.Physics_Status.Ok);
	testing.expect_value(t, add2, physics.Physics_Status.Ok);
	testing.expect_value(t, handle0.value, i32(0));
	testing.expect_value(t, handle1.value, i32(1));
	testing.expect_value(t, handle2.value, i32(2));
	testing.expect_value(t, physics.bodies_move(&bodies, handle1, 1), physics.Physics_Status.Ok);
	location1, resolve1 := physics.bodies_resolve(&bodies, handle1);
	location2, resolve2 := physics.bodies_resolve(&bodies, handle2);
	testing.expect_value(t, resolve1, physics.Physics_Status.Ok);
	testing.expect_value(t, resolve2, physics.Physics_Status.Ok);
	testing.expect_value(t, location1.set_index, i32(1));
	testing.expect_value(t, location2.index, i32(1));
	description_after_move, description_status := physics.bodies_get_description(&bodies, handle1);
	testing.expect_value(t, description_status, physics.Physics_Status.Ok);
	testing.expect_value(t, description_after_move.pose.position.x, f32(1));
	testing.expect_value(
		t,
		physics.bodies_move(&bodies, handle1, physics.BODIES_ACTIVE_SET_INDEX),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, physics.bodies_remove(&bodies, handle1), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_reference_state(&bodies, handle1), physics.Reference_State.Missing);
	testing.expect_value(t, physics.bodies_remove(&bodies, handle1), physics.Physics_Status.Not_Found);
	replacement_description := dynamic_description(9);
	replacement, replacement_status := physics.bodies_add(&bodies, &replacement_description);
	testing.expect_value(t, replacement_status, physics.Physics_Status.Ok);
	testing.expect_value(t, replacement.value, handle1.value);
	testing.expect_value(t, physics.bodies_reference_state(&bodies, replacement), physics.Reference_State.Present);
}

@(test)
static_lifecycle_preserves_swap_remove_maps_and_source_handle_reuse :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	registry: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&registry, 2, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&registry);
	sphere := physics.Sphere{1};
	shape, shape_status := physics.shape_registry_add(&registry, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	statics: physics.Statics;
	testing.expect_value(t, physics.statics_initialize(&statics, 2, &pool), physics.Physics_Status.Ok);
	defer physics.statics_dispose(&statics);
	testing.expect_value(t, physics.statics_bind_shape_registry(&statics, &registry), physics.Physics_Status.Ok);
	description0 := physics.Static_Description{pose=physics.rigid_pose_identity(), shape=shape};
	description1 := description0;
	description1.pose.position = {2, 3, 4};
	handle0, add0 := physics.statics_add(&statics, &description0);
	handle1, add1 := physics.statics_add(&statics, &description1);
	testing.expect_value(t, add0, physics.Physics_Status.Ok);
	testing.expect_value(t, add1, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.statics_remove(&statics, handle0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.statics_reference_state(&statics, handle0), physics.Reference_State.Missing);
	moved_description, moved_status := physics.statics_get_description(&statics, handle1);
	testing.expect_value(t, moved_status, physics.Physics_Status.Ok);
	testing.expect_value(t, moved_description.pose.position.x, f32(2));
	replacement, replacement_status := physics.statics_add(&statics, &description0);
	testing.expect_value(t, replacement_status, physics.Physics_Status.Ok);
	testing.expect_value(t, replacement.value, handle0.value);
}

@(test)
active_gather_and_scatter_preserve_all_eight_lane_values :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	testing.expect_value(t, physics.bodies_initialize(&bodies, 8, 2, 1, &pool), physics.Physics_Status.Ok);
	defer physics.bodies_dispose(&bodies);
	for index in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		description := dynamic_description(index);
		handle, status := physics.bodies_add(&bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, handle.value, i32(index));
	}
	active := &bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		active.dynamics_state.memory[lane].inertia.world.inverse_inertia_tensor.xx = f32(20 + lane);
		active.dynamics_state.memory[lane].inertia.world.inverse_inertia_tensor.zz = f32(40 + lane);
		active.dynamics_state.memory[lane].inertia.world.inverse_mass = f32(60 + lane);
	}
	indices := util.I32x8{0, 1, 2, 3, 4, 5, 6, 7};
	position, orientation, velocity, inertia, gather_status := physics.bodies_gather_active(&bodies, indices, .Local);
	testing.expect_value(t, gather_status, physics.Physics_Status.Ok);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		testing.expect_value(t, simd.extract(position.x, lane), f32(lane));
		testing.expect_value(t, simd.extract(position.y, lane), f32(lane * 2));
		testing.expect_value(t, simd.extract(orientation.w, lane), f32(1));
		testing.expect_value(t, simd.extract(velocity.linear.x, lane), f32(lane + 1));
		testing.expect_value(t, simd.extract(inertia.inverse_mass, lane), f32(0.5));
	}
	_, _, _, world_inertia, world_gather_status := physics.bodies_gather_active(&bodies, indices, .World);
	testing.expect_value(t, world_gather_status, physics.Physics_Status.Ok);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		testing.expect_value(t, simd.extract(world_inertia.inverse_inertia_tensor.xx, lane), f32(20 + lane));
		testing.expect_value(t, simd.extract(world_inertia.inverse_inertia_tensor.zz, lane), f32(40 + lane));
		testing.expect_value(t, simd.extract(world_inertia.inverse_mass, lane), f32(60 + lane));
	}
	masked_position, masked_orientation, masked_velocity, masked_inertia, masked_status := physics.bodies_gather_active(
		&bodies, indices, .Local, {.Orientation, .Inertia_Tensor, .Angular_Velocity},
	);
	testing.expect_value(t, masked_status, physics.Physics_Status.Ok);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		testing.expect_value(t, simd.extract(masked_position.x, lane), f32(0));
		testing.expect_value(t, simd.extract(masked_orientation.w, lane), f32(1));
		testing.expect_value(t, simd.extract(masked_velocity.linear.x, lane), f32(0));
		testing.expect_value(t, simd.extract(masked_velocity.angular.x, lane), f32(lane + 4));
		testing.expect_value(t, simd.extract(masked_inertia.inverse_inertia_tensor.xx, lane), f32(1));
		testing.expect_value(t, simd.extract(masked_inertia.inverse_mass, lane), f32(0));
	}
	mixed_indices := util.I32x8{
		0, -1, i32(u32(2) | physics.BODY_REFERENCE_KINEMATIC_MASK), 3, 4, 5, 6, 7,
	};
	mixed_position, _, _, _, mixed_status := physics.bodies_gather_active(&bodies, mixed_indices, .Local);
	testing.expect_value(t, mixed_status, physics.Physics_Status.Ok);
	testing.expect_value(t, simd.extract(mixed_position.x, 1), f32(0));
	testing.expect_value(t, simd.extract(mixed_position.x, 2), f32(2));
	velocity.linear.x = util.F32x8{10, 11, 12, 13, 14, 15, 16, 17};
	velocity.angular.z = util.F32x8{20, 21, 22, 23, 24, 25, 26, 27};
	testing.expect_value(
		t,
		physics.bodies_scatter_active_velocities(&bodies, indices, velocity),
		physics.Physics_Status.Ok
	);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		description, status := physics.bodies_get_description(&bodies, physics.Body_Handle{i32(lane)});
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, description.velocity.linear.x, f32(lane + 10));
		testing.expect_value(t, description.velocity.angular.z, f32(lane + 20));
	}
	linear_only := velocity;
	linear_only.linear.y = util.F32x8{30, 31, 32, 33, 34, 35, 36, 37};
	linear_only.angular.z = util.F32x8{80, 81, 82, 83, 84, 85, 86, 87};
	filtered_indices := util.I32x8{
		0, i32(u32(1) | physics.BODY_REFERENCE_KINEMATIC_MASK), -1, 3, 4, 5, 6, 7,
	};
	testing.expect_value(
		t, physics.bodies_scatter_active_velocities(&bodies, filtered_indices, linear_only, {.Linear_Velocity}),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, active.dynamics_state.memory[0].motion.velocity.linear.y, f32(30));
	testing.expect_value(t, active.dynamics_state.memory[0].motion.velocity.angular.z, f32(20));
	testing.expect_value(t, active.dynamics_state.memory[1].motion.velocity.linear.y, f32(3));
	testing.expect_value(t, active.dynamics_state.memory[3].motion.velocity.linear.y, f32(33));
	position.x = util.F32x8{100, 101, 102, 103, 104, 105, 106, 107};
	position.y = util.F32x8{200, 201, 202, 203, 204, 205, 206, 207};
	orientation.w = util.F32x8{1, 1, 1, 1, 1, 1, 1, 1};
	lane_mask := util.I32x8{-1, 0, -1, 0, -1, 0, -1, 0};
	testing.expect_value(
		t, physics.bodies_scatter_active_pose(&bodies, indices, lane_mask, position, orientation),
		physics.Physics_Status.Ok,
	);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		expected_x := f32(lane);
		if lane & 1 == 0
		{
			expected_x = f32(100 + lane);
		}
		testing.expect_value(t, active.dynamics_state.memory[lane].motion.pose.position.x, expected_x);
	}
	inertia.inverse_mass = util.F32x8{2, 3, 4, 5, 6, 7, 8, 9};
	inertia.inverse_inertia_tensor.zz = util.F32x8{10, 11, 12, 13, 14, 15, 16, 17};
	testing.expect_value(
		t, physics.bodies_scatter_active_inertia(&bodies, indices, lane_mask, inertia),
		physics.Physics_Status.Ok,
	);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if lane & 1 == 0
		{
			testing.expect_value(t, active.dynamics_state.memory[lane].inertia.world.inverse_mass, f32(lane + 2));
			testing.expect_value(
				t,
				active.dynamics_state.memory[lane].inertia.world.inverse_inertia_tensor.zz,
				f32(lane + 10)
			);
		}
		else
		{
			testing.expect_value(t, active.dynamics_state.memory[lane].inertia.world.inverse_mass, f32(60 + lane));
		}
	}
}

@(test)
body_and_static_growth_preserves_data_while_pool_expands :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	set: physics.Body_Set;
	testing.expect_value(t, physics.body_set_initialize(&set, 1, &pool), physics.Physics_Status.Ok);
	set.count = 1;
	set.index_to_handle.memory[0] = {37};
	set.dynamics_state.memory[0].motion.pose.position = {1, 2, 3};
	blocked: [32]util.Buffer(i32);
	blocked_count := exhaust_i32_1024(&pool, &blocked);
	testing.expect(t, blocked_count > 0);
	testing.expect_value(t, physics.body_set_ensure_capacity(&set, 1024, &pool), physics.Physics_Status.Ok);
	testing.expect_value(t, set.index_to_handle.memory[0].value, i32(37));
	testing.expect_value(t, set.dynamics_state.memory[0].motion.pose.position, util.Vector3{1, 2, 3});
	return_i32_buffers(&pool, &blocked, blocked_count);
	testing.expect_value(t, physics.body_set_dispose(&set, &pool), physics.Physics_Status.Ok);
	bodies: physics.Bodies;
	testing.expect_value(t, physics.bodies_initialize(&bodies, 1, 1, 1, &pool), physics.Physics_Status.Ok);
	bodies.handle_to_location.memory[0] = {7, 9};
	_, _ = util.id_pool_take(&bodies.handle_pool);
	_, _ = util.id_pool_take(&bodies.handle_pool);
	_ = util.id_pool_return(&bodies.handle_pool, 0, &pool);
	blocked_count = exhaust_i32_1024(&pool, &blocked);
	testing.expect(t, blocked_count > 0);
	testing.expect_value(t, physics.bodies_ensure_handle_capacity(&bodies, 1024), physics.Physics_Status.Ok);
	testing.expect_value(t, bodies.handle_to_location.memory[0], physics.Body_Memory_Location{7, 9});
	testing.expect_value(t, bodies.handle_pool.available_id_count, 1);
	testing.expect_value(t, bodies.handle_pool.next_index, i32(2));
	return_i32_buffers(&pool, &blocked, blocked_count);
	testing.expect_value(t, physics.bodies_dispose(&bodies), physics.Physics_Status.Ok);
	statics: physics.Statics;
	testing.expect_value(t, physics.statics_initialize(&statics, 1, &pool), physics.Physics_Status.Ok);
	statics.count = 1;
	statics.statics.memory[0].pose.position = {4, 5, 6};
	statics.handle_to_index.memory[0] = 0;
	statics.index_to_handle.memory[0] = {0};
	_, _ = util.id_pool_take(&statics.handle_pool);
	_, _ = util.id_pool_take(&statics.handle_pool);
	_ = util.id_pool_return(&statics.handle_pool, 0, &pool);
	blocked_count = exhaust_i32_1024(&pool, &blocked);
	testing.expect(t, blocked_count > 0);
	testing.expect_value(t, physics.statics_ensure_capacity(&statics, 1024), physics.Physics_Status.Ok);
	testing.expect_value(t, statics.statics.memory[0].pose.position, util.Vector3{4, 5, 6});
	testing.expect_value(t, statics.handle_to_index.memory[0], i32(0));
	testing.expect_value(t, statics.index_to_handle.memory[0].value, i32(0));
	testing.expect_value(t, statics.handle_pool.available_id_count, 1);
	return_i32_buffers(&pool, &blocked, blocked_count);
	testing.expect_value(t, statics.handle_to_index.memory[statics.handle_to_index.length - 1], i32(-1));
	testing.expect_value(t, physics.statics_dispose(&statics), physics.Physics_Status.Ok);
}

@(test)
active_gather_masks_match_full_gather_for_every_field_and_mobility :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	if !testing.expect_value(t, physics.bodies_initialize(&bodies, 8, 2, 1, &pool), physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.bodies_dispose(&bodies);
	for index in 0 ..< 8
	{
		description := dynamic_description(index);
		_, status := physics.bodies_add(&bodies, &description);
		if !testing.expect_value(t, status, physics.Physics_Status.Ok)
		{
			return;
		}
		body := &bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[index];
		body.inertia.world = {inverse_mass=f32(index + 9), inverse_inertia_tensor={2, 3, 4, 5, 6, 7}};
	}
	for indices in ([4]util.I32x8{
			{0, 1, 2, 3, 4, 5, 6, 7},
			{0, -1, i32(u32(2) | physics.BODY_REFERENCE_KINEMATIC_MASK), 3, -1, 5, 6, 7},
			{-1, -1, -1, -1, -1, -1, -1, -1},
			{7, 0, 7, 3, i32(physics.BODY_REFERENCE_KINEMATIC_MASK | 1), -1, 5, 2},
	})
	{
		for source in ([2]physics.Inertia_Source{.Local, .World})
		{
			full_position, full_orientation, full_velocity, full_inertia := physics.bodies_gather_active_trusted(&bodies, indices, source);
			combined_velocity: physics.Body_Velocity_Wide;
			combined_inertia: physics.Body_Inertia_Wide;
			physics.bodies_gather_active_no_pose_trusted(&bodies, indices, source, &combined_velocity, &combined_inertia);
			testing.expect_value(t, transmute([48]u32)combined_velocity, transmute([48]u32)full_velocity);
			testing.expect_value(t, transmute([56]u32)combined_inertia, transmute([56]u32)full_inertia);
			for bits in 0 ..< 64
			{
				access := transmute(physics.Body_Access_Mask)u8(bits);
				position, orientation, velocity, inertia := physics.bodies_gather_active_trusted(&bodies, indices, source, access);
				want_position: util.Vector3_Wide;
				want_orientation: util.Quaternion_Wide;
				want_velocity: physics.Body_Velocity_Wide;
				want_inertia: physics.Body_Inertia_Wide;
				if .Position in access
				{
					want_position = full_position;
				}
				if .Orientation in access
				{
					want_orientation = full_orientation;
				}
				if .Linear_Velocity in access
				{
					want_velocity.linear = full_velocity.linear;
				}
				if .Angular_Velocity in access
				{
					want_velocity.angular = full_velocity.angular;
				}
				if .Mass in access
				{
					want_inertia.inverse_mass = full_inertia.inverse_mass;
				}
				if .Inertia_Tensor in access
				{
					want_inertia.inverse_inertia_tensor = full_inertia.inverse_inertia_tensor;
				}
				testing.expect_value(t, transmute([24]f32)position, transmute([24]f32)want_position);
				testing.expect_value(t, transmute([32]f32)orientation, transmute([32]f32)want_orientation);
				testing.expect_value(t, transmute([48]f32)velocity, transmute([48]f32)want_velocity);
				testing.expect_value(t, transmute([56]f32)inertia, transmute([56]f32)want_inertia);
			}
		}
	}
}
