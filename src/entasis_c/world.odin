package entasis_c

import "base:runtime"
import "core:math"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import shared "entasis:entasis_c_shared"

ENTASIS_POSE_DESCRIPTION_DEFAULT :: Entasis_Pose_Policy_Kind(0);
ENTASIS_POSE_UNIFORM             :: Entasis_Pose_Policy_Kind(1);
ENTASIS_POSE_PLANETARY           :: Entasis_Pose_Policy_Kind(2);
ENTASIS_POSE_PER_BODY            :: Entasis_Pose_Policy_Kind(3);
abi_per_body_context :: struct
{
	gravity_by_body_handle: [^]Entasis_Vector3,
	gravity_count:          u64,
	simulation:             ^physics.Simulation,
	linear_damping:         f32,
	angular_damping:        f32,
	linear_damping_dt:      util.F32x8,
	angular_damping_dt:     util.F32x8,
}

abi_world_resource :: struct
{
	header:           shared.World_Header,
	allocator:        shared.Allocator,
	world:            entasis.World,
	pool_header:      ^abi_buffer_pool_header,
	thread_pool:      ^abi_thread_pool_resource,
	external_bridge:  abi_external_dispatch_bridge,
	narrow_policy:    entasis.Default_Narrow_Policy,
	// uses the existing eight-byte gap before the 32-byte-aligned policy
	custom_constraints: ^abi_custom_constraint_binding,
	uniform_policy:   entasis.Uniform_Gravity_Policy,
	planetary_policy: entasis.Planetary_Gravity_Policy,
	per_body_policy:  abi_per_body_context,
	layer_material:   abi_layer_material_context,
	property_resource:       ^abi_collision_property_resource,
	active_contact_trackers: i32,
	_reserved0:             i32,
	stage_callbacks: Entasis_Timestep_Callbacks,
	custom_timestepper: Entasis_Timestepper,
	substep_scheduler: Entasis_Substep_Scheduler,
	velocity_callbacks: Entasis_Velocity_Callbacks,
	contact_callbacks: Entasis_Contact_Callbacks,
	pose_simulation: ^physics.Simulation,
	step_scope_simulation: ^physics.Simulation,
	step_scope_dispatcher: ^util.Thread_Dispatcher_Boundary,
	step_scope_generation: u64,
	step_scope_live: abi_Scope_State,
	step_scope_executing: abi_Execution_State,
	custom_shapes: ^abi_custom_shape_binding,
	custom_tasks: ^abi_custom_task_binding,
}

abi_world_get :: #force_inline proc "contextless" (
	handle: ^Entasis_World,
) -> ^abi_world_resource
{
	if handle == nil
	{
		return nil;
	}
	header := shared.World_Header_Get(handle.opaque);
	if header == nil
	{
		return nil;
	}
	return (^abi_world_resource)(header.owner);
}

abi_per_body_initialize :: proc "contextless" (
	user_context: rawptr,
	simulation: ^physics.Simulation,
) -> entasis.Status
{
	if user_context == nil || simulation == nil
	{
		return .Invalid_Argument;
	}
	policy := (^abi_per_body_context)(user_context);
	if policy.gravity_by_body_handle == nil || policy.gravity_count == 0 ||
	policy.linear_damping < 0 || policy.linear_damping > 1 ||
	policy.angular_damping < 0 || policy.angular_damping > 1
	{
		return .Invalid_Description;
	}
	policy.simulation = simulation;
	return .Ok;
}

abi_per_body_prepare :: proc "contextless" (
	user_context: rawptr,
	dt: f32,
) -> entasis.Status
{
	if user_context == nil || dt <= 0
	{
		return .Invalid_Argument;
	}
	policy := (^abi_per_body_context)(user_context);
	linear_base := clamp(1 - policy.linear_damping, f32(0), f32(1));
	angular_base := clamp(1 - policy.angular_damping, f32(0), f32(1));
	policy.linear_damping_dt = util.F32x8(f32(math.pow(f64(linear_base), f64(dt))));
	policy.angular_damping_dt = util.F32x8(f32(math.pow(f64(angular_base), f64(dt))));
	return .Ok;
}

abi_per_body_integrate :: #force_inline proc "contextless" (
	user_context: rawptr,
	body_indices: util.I32x8,
	position: util.Vector3_Wide,
	orientation: util.Quaternion_Wide,
	inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8,
	worker_index: int,
	dt: util.F32x8,
	velocity: ^physics.Body_Velocity_Wide,
)
{
	_, _, _, _, _ = position, orientation, inertia, integration_mask, worker_index;
	if user_context == nil || velocity == nil
	{
		return;
	}
	policy := (^abi_per_body_context)(user_context);
	if policy.simulation == nil
	{
		return;
	}
	gravity_wide: util.Vector3_Wide;
	indices := transmute([8]i32)body_indices;
	active := &policy.simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		body_index := int(indices[lane]);
		if body_index < 0 || body_index >= active.count
		{
			continue;
		}
		handle := active.index_to_handle.memory[body_index];
		if handle.value < 0 || u64(handle.value) >= policy.gravity_count
		{
			continue;
		}
		gravity := policy.gravity_by_body_handle[handle.value];
		util.vector3_wide_write_slot(
			&gravity_wide, lane, abi_vector3_to_core(gravity),
		);
	}
	velocity.linear = util.vector3_wide_scale(
		util.vector3_wide_add(
			velocity.linear, util.vector3_wide_scale(gravity_wide, dt),
		),
		policy.linear_damping_dt,
	);
	velocity.angular = util.vector3_wide_scale(
		velocity.angular, policy.angular_damping_dt,
	);
}

abi_per_body_dispose :: proc "contextless" (user_context: rawptr)
{
	if user_context == nil
	{
		return;
	}
	policy := (^abi_per_body_context)(user_context);
	policy.simulation = nil;
}

abi_pose_flags_apply :: #force_inline proc "contextless" (
	callbacks: ^entasis.Pose_Callbacks,
	policy: Entasis_Pose_Policy,
) -> entasis.Status
{
	if callbacks == nil || policy.angular_mode > 2 ||
	policy.allow_substeps_for_unconstrained > 1 ||
	policy.integrate_kinematic_velocity > 1
	{
		return .Invalid_Description;
	}
	callbacks.angular_mode = entasis.Angular_Integration_Mode(policy.angular_mode);
	callbacks.allow_substeps_for_unconstrained = entasis.Velocity_Integration_State(
		policy.allow_substeps_for_unconstrained,
	);
	callbacks.integrate_kinematic_velocity = entasis.Velocity_Integration_State(
		policy.integrate_kinematic_velocity,
	);
	return .Ok;
}

abi_description_to_core :: proc "contextless" (
	resource: ^abi_world_resource,
	description: ^Entasis_World_Description,
) -> (entasis.World_Description, entasis.Status)
{
	if resource == nil || description == nil ||
	description.struct_size < u32(size_of(Entasis_World_Description)) ||
	description.struct_version != ENTASIS_STRUCT_VERSION ||
	description.profiling > 1 ||
	description.use_narrow_policy > 1 || description.use_pose_policy > 1 ||
	description.solve.struct_size < u32(size_of(Entasis_Solve_Description)) ||
	description.solve.struct_version != ENTASIS_STRUCT_VERSION ||
	description.threading.struct_size < u32(size_of(Entasis_Threading_Description)) ||
	description.threading.struct_version != ENTASIS_STRUCT_VERSION
	{
		return {}, .Invalid_Description;
	}
	allocator, allocator_status := abi_allocator_to_core(&resource.allocator);
	if allocator_status != .Ok
	{
		return {}, allocator_status;
	}
	low := entasis.World_Description{
		profiling=description.profiling != 0,
		gravity=abi_vector3_to_core(description.gravity),
		damping={
			linear=description.damping.linear,
			angular=description.damping.angular,
		},
		capacity=abi_capacity_to_core(description.capacity),
		solve={
			velocity_iterations=description.solve.velocity_iterations,
			substeps=description.solve.substeps,
			fallback_batch_threshold=description.solve.fallback_batch_threshold,
		},
		threading={
			worker_count=i32(description.threading.worker_count),
			worker_pool_block_size=i32(description.threading.worker_pool_block_size),
		},
		allocator=allocator,
	};
	if description.threading.external_dispatcher != nil
	{
		dispatch_status := abi_external_bridge_init(
			&resource.external_bridge, description.threading.external_dispatcher,
		);
		if dispatch_status != ENTASIS_DISPATCH_OK
		{
			if dispatch_status == ENTASIS_DISPATCH_CAPACITY_MISSING
			{
				return {}, .Capacity_Missing;
			}
			if dispatch_status == ENTASIS_DISPATCH_DISPOSED ||
			dispatch_status == ENTASIS_DISPATCH_SHUTTING_DOWN
			{
				return {}, .Disposed;
			}
			return {}, .Invalid_Description;
		}
		low.threading.external_dispatcher = &resource.external_bridge.boundary;
	}
	if description.narrow_policy_v1 != nil
	{
		if description.use_narrow_policy != 0 ||
		description.narrow_policy_v1.struct_size < u32(size_of(Entasis_Narrow_Policy)) ||
		description.narrow_policy_v1.struct_version != ENTASIS_STRUCT_VERSION
		{
			return {}, .Invalid_Description;
		}
		switch description.narrow_policy_v1.kind
		{
			case ENTASIS_NARROW_POLICY_DEFAULT_KIND:
			resource.narrow_policy = {
				material=abi_material_to_core(description.narrow_policy_v1.default_policy.material),
			};
			low.narrow_callbacks = entasis.narrow_policy_default(&resource.narrow_policy);
			case ENTASIS_NARROW_POLICY_LAYERS_MATERIALS_KIND:
			policy_status := abi_layer_material_context_build(
				&resource.layer_material, description.narrow_policy_v1.layer_material_policy,
			);
			if policy_status != .Ok
			{
				return {}, policy_status;
			}
			resource.property_resource = resource.layer_material.property_resource;
			low.narrow_callbacks = entasis.narrow_policy_layers_materials(&resource.layer_material.core_policy);
			case:
			return {}, .Invalid_Description;
		}
	}
	else if description.use_narrow_policy != 0
	{
		resource.narrow_policy = {
			material=abi_material_to_core(description.narrow_policy.material),
		};
		low.narrow_callbacks = entasis.narrow_policy_default(&resource.narrow_policy);
	}
	if description.use_pose_policy != 0
	{
		policy := description.pose_policy;
		if policy.struct_size < u32(size_of(Entasis_Pose_Policy)) ||
		policy.struct_version != ENTASIS_STRUCT_VERSION
		{
			return {}, .Invalid_Description;
		}
		callbacks: entasis.Pose_Callbacks;
		switch policy.kind
		{
			case ENTASIS_POSE_DESCRIPTION_DEFAULT, ENTASIS_POSE_UNIFORM:
			resource.uniform_policy = entasis.uniform_gravity_policy(
				abi_vector3_to_core(policy.vector),
				policy.linear_damping,
				policy.angular_damping,
			);
			callbacks = entasis.pose_policy_uniform(&resource.uniform_policy);
			case ENTASIS_POSE_PLANETARY:
			resource.planetary_policy = entasis.planetary_gravity_policy(
				abi_vector3_to_core(policy.vector),
				policy.gravity,
				policy.linear_damping,
				policy.angular_damping,
			);
			callbacks = entasis.pose_policy_planetary(&resource.planetary_policy);
			case ENTASIS_POSE_PER_BODY:
			resource.per_body_policy = {
				gravity_by_body_handle=policy.gravity_by_body_handle,
				gravity_count=policy.gravity_count,
				linear_damping=policy.linear_damping,
				angular_damping=policy.angular_damping,
			};
			callbacks = {
				initialize=abi_per_body_initialize,
				prepare_for_integration=abi_per_body_prepare,
				integrate_velocity=abi_per_body_integrate,
				dispose=abi_per_body_dispose,
				user_context=&resource.per_body_policy,
			};
			case:
			return {}, .Invalid_Description;
		}
		flag_status := abi_pose_flags_apply(&callbacks, policy);
		if flag_status != .Ok
		{
			return {}, flag_status;
		}
		low.pose_callbacks = callbacks;
	}
	return low, .Ok;
}

abi_capacity_hints_default :: proc "contextless" () -> Entasis_Capacity_Hints
{
	return abi_capacity_from_core(entasis.capacity_hints_default());
}

abi_solve_description_default :: proc "contextless" () -> Entasis_Solve_Description
{
	value := entasis.solve_description_default();
	return {
		struct_size=u32(size_of(Entasis_Solve_Description)),
		struct_version=ENTASIS_STRUCT_VERSION,
		velocity_iterations=value.velocity_iterations,
		substeps=value.substeps,
		fallback_batch_threshold=value.fallback_batch_threshold,
	};
}

abi_threading_description_default :: proc "contextless" () -> Entasis_Threading_Description
{
	value := entasis.threading_description_default();
	return {
		struct_size=u32(size_of(Entasis_Threading_Description)),
		struct_version=ENTASIS_STRUCT_VERSION,
		worker_count=u32(value.worker_count),
		worker_pool_block_size=u32(value.worker_pool_block_size),
	};
}

abi_world_description_default :: proc "contextless" () -> Entasis_World_Description
{
	context = runtime.default_context();
	value := entasis.world_description_default();
	return {
		struct_size=u32(size_of(Entasis_World_Description)),
		struct_version=ENTASIS_STRUCT_VERSION,
		profiling=abi_bool(value.profiling),
		gravity=abi_vector3_from_core(value.gravity),
		damping={linear=value.damping.linear, angular=value.damping.angular},
		capacity=abi_capacity_from_core(value.capacity),
		solve=abi_solve_description_default(),
		threading=abi_threading_description_default(),
		allocator={
			struct_size=u32(size_of(Entasis_Allocator)),
			struct_version=ENTASIS_STRUCT_VERSION,
		},
		narrow_policy=abi_default_narrow_policy(),
		pose_policy={
			struct_size=u32(size_of(Entasis_Pose_Policy)),
			struct_version=ENTASIS_STRUCT_VERSION,
			kind=ENTASIS_POSE_DESCRIPTION_DEFAULT,
			vector=abi_vector3_from_core(value.gravity),
			linear_damping=value.damping.linear,
			angular_damping=value.damping.angular,
		},
	};
}

abi_spring_settings :: #force_inline proc "contextless" (
	frequency, damping_ratio: f32,
) -> Entasis_Spring_Settings
{
	return abi_spring_from_core(entasis.spring_settings(frequency, damping_ratio));
}

abi_contact_material :: #force_inline proc "contextless" (
	friction_coefficient, maximum_recovery_velocity: f32,
	spring: Entasis_Spring_Settings,
) -> Entasis_Contact_Material
{
	return abi_material_from_core(entasis.contact_material(
			friction_coefficient,
			maximum_recovery_velocity,
			abi_spring_to_core(spring),
	));
}

abi_contact_material_default :: #force_inline proc "contextless" () -> Entasis_Contact_Material
{
	return abi_material_from_core(entasis.contact_material_default());
}

abi_default_narrow_policy :: #force_inline proc "contextless" () -> Entasis_Default_Narrow_Policy
{
	return {material=abi_contact_material_default()};
}

abi_narrow_policy_default :: #force_inline proc "contextless" (
	material: Entasis_Contact_Material,
) -> Entasis_Default_Narrow_Policy
{
	return {material=material};
}

abi_uniform_gravity_policy :: #force_inline proc "contextless" (
	gravity: Entasis_Vector3,
	linear_damping, angular_damping: f32,
) -> Entasis_Uniform_Gravity_Policy
{
	return {gravity=gravity, linear_damping=linear_damping, angular_damping=angular_damping};
}

abi_planetary_gravity_policy :: #force_inline proc "contextless" (
	center: Entasis_Vector3,
	gravity, linear_damping, angular_damping: f32,
) -> Entasis_Planetary_Gravity_Policy
{
	return {
		center=center,
		gravity=gravity,
		linear_damping=linear_damping,
		angular_damping=angular_damping,
	};
}

abi_per_body_gravity_policy :: #force_inline proc "contextless" (
	gravity_by_body_handle: [^]Entasis_Vector3,
	gravity_count: u64,
	linear_damping, angular_damping: f32,
) -> Entasis_Per_Body_Gravity_Policy
{
	return {
		gravity_by_body_handle=gravity_by_body_handle,
		gravity_count=gravity_count,
		linear_damping=linear_damping,
		angular_damping=angular_damping,
	};
}

abi_pose_policy_base :: #force_inline proc "contextless" () -> Entasis_Pose_Policy
{
	return {
		struct_size=u32(size_of(Entasis_Pose_Policy)),
		struct_version=ENTASIS_STRUCT_VERSION,
	};
}

abi_pose_policy_uniform_mode :: proc "contextless" (
	policy: ^Entasis_Uniform_Gravity_Policy,
	angular_mode: Entasis_Angular_Integration_Mode,
) -> Entasis_Pose_Policy
{
	result := abi_pose_policy_base();
	result.kind = ENTASIS_POSE_UNIFORM;
	result.angular_mode = angular_mode;
	if policy != nil
	{
		result.vector = policy.gravity;
		result.linear_damping = policy.linear_damping;
		result.angular_damping = policy.angular_damping;
	}
	return result;
}

abi_pose_policy_uniform :: #force_inline proc "contextless" (
	policy: ^Entasis_Uniform_Gravity_Policy,
) -> Entasis_Pose_Policy
{
	return abi_pose_policy_uniform_mode(policy, 0);
}

abi_pose_policy_planetary :: proc "contextless" (
	policy: ^Entasis_Planetary_Gravity_Policy,
) -> Entasis_Pose_Policy
{
	result := abi_pose_policy_base();
	result.kind = ENTASIS_POSE_PLANETARY;
	if policy != nil
	{
		result.vector = policy.center;
		result.gravity = policy.gravity;
		result.linear_damping = policy.linear_damping;
		result.angular_damping = policy.angular_damping;
	}
	return result;
}

abi_pose_policy_per_body :: proc "contextless" (
	policy: ^Entasis_Per_Body_Gravity_Policy,
) -> Entasis_Pose_Policy
{
	result := abi_pose_policy_base();
	result.kind = ENTASIS_POSE_PER_BODY;
	if policy != nil
	{
		result.gravity_by_body_handle = policy.gravity_by_body_handle;
		result.gravity_count = policy.gravity_count;
		result.linear_damping = policy.linear_damping;
		result.angular_damping = policy.angular_damping;
	}
	return result;
}

abi_world_description_set_callbacks :: proc "contextless" (
	description: ^Entasis_World_Description,
	narrow: ^Entasis_Default_Narrow_Policy,
	pose: ^Entasis_Pose_Policy,
) -> Entasis_Status
{
	if description == nil ||
	description.struct_size < u32(size_of(Entasis_World_Description)) ||
	description.struct_version != ENTASIS_STRUCT_VERSION
	{
		return abi_status(.Invalid_Argument);
	}
	if narrow != nil
	{
		description.narrow_policy = narrow^;
		description.use_narrow_policy = ENTASIS_TRUE;
	}
	if pose != nil
	{
		if pose.struct_size < u32(size_of(Entasis_Pose_Policy)) ||
		pose.struct_version != ENTASIS_STRUCT_VERSION
		{
			return abi_status(.Invalid_Description);
		}
		description.pose_policy = pose^;
		description.use_pose_policy = ENTASIS_TRUE;
	}
	return abi_status(.Ok);
}

abi_world_init_internal :: proc "contextless" (
	world: ^Entasis_World,
	description_input: ^Entasis_World_Description,
	pool: ^Entasis_Buffer_Pool,
	diagnostic: ^Entasis_Diagnostic,
	extensions_input: ^Entasis_World_Extensions = nil,
) -> Entasis_Status
{
	description := description_input;
	extensions := extensions_input;
	if world == nil || world.opaque != nil || description == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	if abi_world_extensions_validate(extensions) != .Ok ||
	(extensions != nil && ((extensions.velocity_callbacks != nil && description.use_pose_policy != 0) ||
			(extensions.contact_callbacks != nil && (description.use_narrow_policy != 0 || description.narrow_policy_v1 != nil))))
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .World_Initialize);
	}
	// copy descriptors before the first application allocation. only explicitly
	// borrowed user data survives. callback-table pointers below are call-local
	description_copy := description^;
	description = &description_copy;
	extension_copy: Entasis_World_Extensions;
	velocity_copy: Entasis_Velocity_Callbacks;
	contact_copy: Entasis_Contact_Callbacks;
	timestep_callbacks_copy: Entasis_Timestep_Callbacks;
	timestepper_copy: Entasis_Timestepper;
	scheduler_copy: Entasis_Substep_Scheduler;
	if extensions != nil
	{
		extension_copy = extensions^;
		if extensions.velocity_callbacks != nil
		{
			velocity_copy = extensions.velocity_callbacks^;
			extension_copy.velocity_callbacks = &velocity_copy;
		}
		if extensions.contact_callbacks != nil
		{
			contact_copy = extensions.contact_callbacks^;
			extension_copy.contact_callbacks = &contact_copy;
		}
		if extensions.timestep_callbacks != nil
		{
			timestep_callbacks_copy = extensions.timestep_callbacks^;
			extension_copy.timestep_callbacks = &timestep_callbacks_copy;
		}
		if extensions.timestepper != nil
		{
			timestepper_copy = extensions.timestepper^;
			extension_copy.timestepper = &timestepper_copy;
		}
		if extensions.substep_scheduler != nil
		{
			scheduler_copy = extensions.substep_scheduler^;
			extension_copy.substep_scheduler = &scheduler_copy;
		}
		extensions = &extension_copy;
	}
	allocation_scope := entasis.Allocation_Scope.Legacy;
	if extensions != nil
	{
		allocation_scope = entasis.Allocation_Scope(extensions.allocation_scope);
	}
	pool_header: ^abi_buffer_pool_header;
	if pool != nil
	{
		pool_header = abi_buffer_pool_header_get(pool);
		if pool_header == nil || pool_header.borrowed
		{
			return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
		}
		if pool_header.query_context_attached == .Attached
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
		}
	}
	// a supplied allocator can reenter. attach before its first call, not after
	// native initialization, so borrowed storage cannot be destroyed underneath it
	pool_committed: abi_Initialization_State = .Pending;
	if pool_header != nil
	{
		pool_header.attached += 1;
	}
	defer
	{
		if pool_header != nil && pool_committed != .Committed
		{
			pool_header.attached -= 1;
		}
	}
	allocator_status := abi_allocator_validate(&description.allocator);
	if allocator_status != .Ok
	{
		return abi_status_finish(allocator_status, diagnostic, .World_Initialize);
	}
	memory, allocator_copy, allocation_status := abi_resource_allocate(
		size_of(abi_world_resource), align_of(abi_world_resource), &description.allocator, allocation_scope,
	);
	if allocation_status != .Ok
	{
		return abi_status_finish(allocation_status, diagnostic, .World_Initialize);
	}
	resource := (^abi_world_resource)(memory);
	resource.header = {magic=shared.WORLD_MAGIC, world=&resource.world, owner=resource, access=.Exclusive};
	resource.allocator = allocator_copy;
	low, convert_status := abi_description_to_core(resource, description);
	if convert_status != .Ok
	{
		allocator_copy = resource.allocator;
		resource.header.magic = 0;
		abi_resource_free(
			resource, size_of(abi_world_resource), align_of(abi_world_resource), &allocator_copy,
		);
		return abi_status_finish(convert_status, diagnostic, .World_Initialize);
	}
	abi_world_extensions_bind(resource, &low, extensions);
	context = runtime.default_context();
	status := entasis.Status.Ok;
	if pool_header == nil
	{
		status = entasis.world_init_with_allocation_scope(&resource.world, low, allocation_scope);
	}
	else
	{
		status = entasis.world_init_with_pool_and_allocation_scope(&resource.world, low, pool_header.pool, allocation_scope);
	}
	if status != .Ok
	{
		allocator_copy = resource.allocator;
		resource.header.magic = 0;
		abi_resource_free(
			resource, size_of(abi_world_resource), align_of(abi_world_resource), &allocator_copy,
		);
		return abi_status_finish(status, diagnostic, .World_Initialize);
	}
	resource.pool_header = pool_header;
	pool_committed = .Committed;
	resource.thread_pool = abi_thread_pool_from_interface(description.threading.external_dispatcher);
	if resource.thread_pool != nil
	{
		resource.thread_pool.attached_worlds += 1;
	}
	if resource.property_resource != nil
	{
		resource.property_resource.attached_worlds += 1;
	}
	resource.header.access = .Ready;
	world.opaque = &resource.header;
	return abi_status_finish(.Ok, diagnostic, .World_Initialize);
}

abi_world_init :: proc "contextless" (
	world: ^Entasis_World,
	description: ^Entasis_World_Description,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	return abi_world_init_internal(world, description, nil, diagnostic);
}

abi_world_init_with_pool :: proc "contextless" (
	world: ^Entasis_World,
	description: ^Entasis_World_Description,
	pool: ^Entasis_Buffer_Pool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if pool == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	return abi_world_init_internal(world, description, pool, diagnostic);
}

abi_world_clear :: proc "contextless" (
	world: ^Entasis_World,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
	}
	if resource.header.access != .Ready
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	resource.header.access = .Exclusive;
	defer abi_world_release(resource);
	context = runtime.default_context();
	return abi_status_finish(entasis.world_clear(&resource.world), diagnostic, .World_Initialize);
}

abi_world_ensure_capacity :: proc "contextless" (
	world: ^Entasis_World,
	hints: ^Entasis_Capacity_Hints,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Ensure_Capacity);
	}
	if resource.header.access != .Ready || hints == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Ensure_Capacity);
	}
	resource.header.access = .Exclusive;
	defer abi_world_release(resource);
	context = runtime.default_context();
	return abi_status_finish(
		entasis.world_ensure_capacity(&resource.world, abi_capacity_to_core(hints^)),
		diagnostic,
		.World_Ensure_Capacity,
	);
}

abi_world_resize :: proc "contextless" (
	world: ^Entasis_World,
	hints: ^Entasis_Capacity_Hints,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Ensure_Capacity);
	}
	if resource.header.access != .Ready || hints == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Ensure_Capacity);
	}
	resource.header.access = .Exclusive;
	defer abi_world_release(resource);
	context = runtime.default_context();
	return abi_status_finish(
		entasis.world_resize(&resource.world, abi_capacity_to_core(hints^)),
		diagnostic,
		.World_Ensure_Capacity,
	);
}

abi_world_step :: proc "contextless" (
	world: ^Entasis_World,
	dt: f32,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
	}
	if resource.header.access != .Ready || dt <= 0
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	resource.header.access = .Exclusive;
	defer abi_world_release(resource);
	context = runtime.default_context();
	status := entasis.world_step(&resource.world, dt);
	return abi_status_finish(status, diagnostic, .World_Initialize);
}

abi_world_step_external :: proc "contextless" (
	world: ^Entasis_World,
	dt: f32,
	interface: ^Entasis_Dispatcher_Interface,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
	}
	if resource.header.access != .Ready || dt <= 0
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	bridge: abi_external_dispatch_bridge;
	dispatch_status := abi_external_bridge_init(&bridge, interface);
	if dispatch_status != ENTASIS_DISPATCH_OK
	{
		if dispatch_status == ENTASIS_DISPATCH_CAPACITY_MISSING
		{
			return abi_status_finish(.Capacity_Missing, diagnostic, .World_Initialize);
		}
		if dispatch_status == ENTASIS_DISPATCH_DISPOSED ||
		dispatch_status == ENTASIS_DISPATCH_SHUTTING_DOWN
		{
			return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
		}
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	resource.header.access = .Exclusive;
	defer abi_world_release(resource);
	context = runtime.default_context();
	status := entasis.world_step(&resource.world, dt, &bridge.boundary);
	return abi_status_finish(status, diagnostic, .World_Initialize);
}

abi_world_stats :: proc "contextless" (
	world: ^Entasis_World,
	out_stats: ^Entasis_World_Stats,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_stats == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	out_stats^ = {};
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
	}
	if resource.header.access != .Ready && resource.header.access != .Read_Phase
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	stats, status := entasis.world_stats(&resource.world);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .World_Initialize);
	}
	out_stats^ = {
		step_index=stats.step_index,
		active_bodies=i64(stats.active_bodies),
		sleeping_bodies=i64(stats.sleeping_bodies),
		sleeping_islands=i64(stats.sleeping_islands),
		statics=i64(stats.statics),
		active_constraints=i64(stats.active_constraints),
		sleeping_constraints=i64(stats.sleeping_constraints),
		active_pairs=i64(stats.active_pairs),
		inactive_pairs=i64(stats.inactive_pairs),
		registered_shapes=i64(stats.registered_shapes),
		registered_shape_types=i64(stats.registered_shape_types),
	};
	return abi_status_finish(.Ok, diagnostic, .World_Initialize);
}

abi_world_destroy :: proc "contextless" (
	world: ^Entasis_World,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
	}
	if resource.header.access != .Ready || resource.active_contact_trackers != 0 || resource.header.query_context_count != 0
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	resource.header.access = .Exclusive;
	context = runtime.default_context();
	restitution_binding := abi_restitution_binding_get(resource);
	status := entasis.world_destroy(&resource.world);
	if status != .Ok
	{
		resource.header.access = .Ready;
		return abi_status_finish(status, diagnostic, .World_Initialize);
	}
	if resource.pool_header != nil
	{
		resource.pool_header.attached -= 1;
	}
	if resource.thread_pool != nil
	{
		resource.thread_pool.attached_worlds -= 1;
	}
	if resource.property_resource != nil
	{
		resource.property_resource.attached_worlds -= 1;
	}
	// native teardown has completed all shape disposal callbacks
	abi_custom_shape_bindings_release(resource);
	abi_custom_task_bindings_release(resource);
	abi_custom_constraint_bindings_release(resource);
	abi_restitution_binding_release(resource, restitution_binding);
	allocator := resource.allocator;
	resource.header.access = .Disposed;
	resource.header.magic = 0;
	world.opaque = nil;
	abi_resource_free(
		resource, size_of(abi_world_resource), align_of(abi_world_resource), &allocator,
	);
	return abi_status_finish(.Ok, diagnostic, .World_Initialize);
}
#assert(size_of(entasis.World) == size_of(Entasis_World));
#assert(int(entasis.Angular_Integration_Mode.Nonconserving) == 0);
#assert(int(entasis.Velocity_Integration_State.Enabled) == 1);
// the owner opens a read phase before publishing jobs and closes it only after
// joining every reader. no shared state is written by supported read calls
abi_world_begin_read :: proc "contextless" (
	world: ^Entasis_World, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if resource.header.access != .Ready
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource.header.access = .Read_Phase;
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_world_end_read :: proc "contextless" (
	world: ^Entasis_World, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if resource.header.access != .Read_Phase
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource.header.access = .Ready;
	return abi_status_finish(.Ok, diagnostic, .None);
}
