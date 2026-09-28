package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

// Contact_Material is the zero-copy built-in contact material description
Contact_Material :: physics.Contact_Material_Properties;

// Spring_Settings is the zero-copy built-in spring description used by contact
// materials and constraints
Spring_Settings :: physics.Spring_Settings;

// Narrow_Callbacks and Pose_Callbacks are the advanced raw callback tables used
// directly by the runtime. they allocate nothing and contain only
// contextless procedure pointers, policy flags, and a caller-owned context pointer
Narrow_Callbacks :: physics.Narrow_Phase_Callbacks;
// Pose_Callbacks is the advanced raw pose integration callback table
Pose_Callbacks   :: physics.Pose_Integrator_Callbacks;

// advanced callback value types. custom callback implementations may import the
// low-level package for the full SIMD surface. the facade policy builders below
// cover the ordinary constant-material and uniform-gravity cases
Collision_Testing_State    :: physics.Collision_Testing_State;
// Angular_Integration_Mode selects the angular velocity integration model
Angular_Integration_Mode   :: physics.Angular_Integration_Mode;
// Velocity_Integration_State identifies the current velocity integration pass
Velocity_Integration_State :: physics.Velocity_Integration_State;
// Manifold_Result stores one narrow-phase manifold result
Manifold_Result            :: physics.Manifold_Result;
// Convex_Contact_Manifold stores contacts for one convex pair
Convex_Contact_Manifold    :: physics.Convex_Contact_Manifold;

// Default_Narrow_Policy allows every collidable and child pair and assigns one
// constant material to every accepted manifold. the policy is caller owned and
// must remain alive and immutable while its world can step on worker threads
Default_Narrow_Policy :: struct
{
	material: Contact_Material,
}

// Uniform_Gravity_Policy is the runtime's uniform-gravity callback
// context. gravity, linear_damping, and angular_damping are caller-configured.
// the remaining wide fields are prepared by the runtime before integration and
// must not be modified by the caller while the world is ready
Uniform_Gravity_Policy :: physics.Default_Pose_Integrator_Context;

// Planetary_Gravity_Policy applies inverse-square radial gravity around a fixed
// center. the policy is caller owned and must remain alive and immutable while
// the world can step. prepared fields are owned by the pose callback
Planetary_Gravity_Policy :: struct
{
	center:             Vector3,
	gravity:            f32,
	linear_damping:     f32,
	angular_damping:    f32,
	gravity_dt:         f32,
	linear_damping_dt:  util.F32x8,
	angular_damping_dt: util.F32x8,
}

// Per_Body_Gravity_Policy gathers one caller-owned gravity vector per body
// handle. the property table and policy must remain alive, stable, and read-only
// while the world can step. missing properties apply zero gravity
Per_Body_Gravity_Policy :: struct
{
	gravities:          ^Body_Property_Table(Vector3),
	simulation:         ^physics.Simulation,
	linear_damping:     f32,
	angular_damping:    f32,
	linear_damping_dt:  util.F32x8,
	angular_damping_dt: util.F32x8,
}

// spring_settings converts cycles per second and damping ratio into the exact
// spring representation used by the runtime
// allocation: none
spring_settings :: #force_inline proc "contextless" (
	frequency, damping_ratio: f32,
) -> Spring_Settings
{
	return physics.spring_settings_create(frequency, damping_ratio);
}

// contact_material constructs one constant built-in contact material. the
// policy is validated when its callbacks are activated by world_init
// allocation: none
contact_material :: #force_inline proc "contextless" (
	friction_coefficient: f32,
	maximum_recovery_velocity: f32,
	spring: Spring_Settings,
) -> Contact_Material
{
	return {
		friction_coefficient=friction_coefficient,
		spring_settings=spring,
		maximum_recovery_velocity=maximum_recovery_velocity,
	};
}

// contact_material_default returns the material used by the low-level default
// narrow-phase callbacks
// allocation: none
contact_material_default :: #force_inline proc "contextless" () -> Contact_Material
{
	return contact_material(1, 2, spring_settings(30, 1));
}

// default_narrow_policy returns the no-filter, constant-material policy matching
// the low-level default
// allocation: none
default_narrow_policy :: #force_inline proc "contextless" () -> Default_Narrow_Policy
{
	return {material=contact_material_default()};
}

// uniform_gravity_policy returns a caller-owned context for the runtime's
// uniform-gravity and damping implementation
// allocation: none
uniform_gravity_policy :: #force_inline proc "contextless" (
	gravity: Vector3 = {0, -9.81, 0},
	linear_damping: f32 = 0.01,
	angular_damping: f32 = 0.01,
) -> Uniform_Gravity_Policy
{
	return {
		gravity=gravity,
		linear_damping=linear_damping,
		angular_damping=angular_damping,
	};
}

@(private)
default_narrow_initialize :: proc "contextless" (
	user_context: rawptr,
	simulation: ^physics.Simulation,
) -> Status
{
	_ = simulation;
	if user_context == nil
	{
		return .Invalid_Argument;
	}
	policy := (^Default_Narrow_Policy)(user_context);
	if physics.constraint_contact_material_validate(policy.material) != .Ok
	{
		return .Invalid_Description;
	}
	return .Ok;
}

@(private)
default_narrow_configure :: #force_inline proc "contextless" (
	user_context: rawptr,
	worker_index: int,
	a, b: Collidable_Reference,
	manifold: ^Manifold_Result,
	material: ^Contact_Material,
) -> Collision_Testing_State
{
	_, _, _, _ = worker_index, a, b, manifold;
	if user_context == nil || material == nil
	{
		return .Reject;
	}
	policy := (^Default_Narrow_Policy)(user_context);
	material^ = policy.material;
	return .Allow;
}

// narrow_policy_default builds a no-filter, constant-material callback table
// around caller-owned context. it reuses the low-level allow, child,
// and dispose callbacks and adds no allocation, closure, or extra dispatch
narrow_policy_default :: #force_inline proc "contextless" (
	policy: ^Default_Narrow_Policy,
) -> Narrow_Callbacks
{
	callbacks := physics.narrow_phase_default_callbacks();
	callbacks.initialize = default_narrow_initialize;
	callbacks.configure = default_narrow_configure;
	callbacks.user_context = policy;
	return callbacks;
}

// narrow_policy_default_stored_material returns the material only when callbacks exactly match narrow_policy_default
narrow_policy_default_stored_material :: proc "contextless" (
	callbacks: Narrow_Callbacks,
) -> (Contact_Material, physics.Reference_State)
{
	if callbacks.user_context == nil
	{
		return {}, .Missing;
	}
	policy := (^Default_Narrow_Policy)(callbacks.user_context);
	qualified := narrow_policy_default(policy);
	if callbacks.initialize != qualified.initialize ||
		callbacks.allow != qualified.allow ||
		callbacks.allow_child != qualified.allow_child ||
		callbacks.configure != qualified.configure ||
		callbacks.configure_child != qualified.configure_child ||
		callbacks.dispose != qualified.dispose ||
		callbacks.select_constraint != qualified.select_constraint ||
		callbacks.user_context != qualified.user_context
	{
		return {}, .Missing;
	}
	return policy.material, .Present;
}

// pose_policy_uniform builds the existing wide uniform-gravity
// callback table around caller-owned context. no facade callback is inserted in
// the per-body integration path
pose_policy_uniform :: #force_inline proc "contextless" (
	policy: ^Uniform_Gravity_Policy,
) -> Pose_Callbacks
{
	return physics.pose_integrator_default_callbacks(policy);
}

// pose_policy_uniform_mode builds the same uniform policy with an
// explicit angular integration mode. this is useful for gyroscopes and other
// bodies whose angular momentum behavior matters. allocation: none
pose_policy_uniform_mode :: #force_inline proc "contextless" (
	policy: ^Uniform_Gravity_Policy,
	angular_mode: Angular_Integration_Mode,
) -> Pose_Callbacks
{
	callbacks := physics.pose_integrator_default_callbacks(policy);
	callbacks.angular_mode = angular_mode;
	return callbacks;
}

// planetary_gravity_policy creates a caller-owned inverse-square gravity policy.
// gravity must be nonnegative. damping values use the same [0, 1] contract as
// uniform_gravity_policy. allocation: none
planetary_gravity_policy :: #force_inline proc "contextless" (
	center: Vector3 = {},
	gravity: f32 = 100000,
	linear_damping: f32 = 0,
	angular_damping: f32 = 0,
) -> Planetary_Gravity_Policy
{
	return {
		center=center,
		gravity=gravity,
		linear_damping=linear_damping,
		angular_damping=angular_damping,
	};
}

@(private)
planetary_gravity_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation,
) -> Status
{
	_ = simulation;
	if user_context == nil
	{
		return .Invalid_Argument;
	}
	policy := (^Planetary_Gravity_Policy)(user_context);
	if policy.gravity < 0 || policy.linear_damping < 0 || policy.linear_damping > 1 ||
		policy.angular_damping < 0 || policy.angular_damping > 1
	{
		return .Invalid_Description;
	}
	return .Ok;
}

@(private)
planetary_gravity_prepare :: proc "contextless" (
	user_context: rawptr, dt: f32,
) -> Status
{
	if user_context == nil || dt <= 0
	{
		return .Invalid_Argument;
	}
	policy := (^Planetary_Gravity_Policy)(user_context);
	policy.gravity_dt = policy.gravity * dt;
	linear_base := clamp(1 - policy.linear_damping, f32(0), f32(1));
	angular_base := clamp(1 - policy.angular_damping, f32(0), f32(1));
	policy.linear_damping_dt = util.F32x8(f32(math.pow(f64(linear_base), f64(dt))));
	policy.angular_damping_dt = util.F32x8(f32(math.pow(f64(angular_base), f64(dt))));
	return .Ok;
}

@(private)
planetary_gravity_velocity :: #force_inline proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	_, _, _, _, _ = body_indices, orientation, inertia, integration_mask, worker_index;
	policy := (^Planetary_Gravity_Policy)(user_context);
	offset := util.vector3_wide_subtract(
		position, util.vector3_wide_broadcast(policy.center),
	);
	distance_squared := util.vector3_wide_length_squared(offset);
	distance := simd.sqrt(distance_squared);
	distance_cubed := simd.mul(distance_squared, distance);
	denominator := simd.max(util.F32x8(1), distance_cubed);
	gravity_scale := simd.div(util.F32x8(-policy.gravity_dt), denominator);
	velocity.linear = util.vector3_wide_scale(
		util.vector3_wide_add(
		velocity.linear, util.vector3_wide_scale(offset, gravity_scale),
	),
		policy.linear_damping_dt,
	);
	velocity.angular = util.vector3_wide_scale(
		velocity.angular, policy.angular_damping_dt,
	);
	_ = dt;
}

@(private)
planetary_gravity_dispose :: proc "contextless" (user_context: rawptr)
{
	_ = user_context;
}

// pose_policy_planetary builds the wide radial-gravity callback table. the hot
// integration path remains SIMD and performs no allocation
pose_policy_planetary :: #force_inline proc "contextless" (
	policy: ^Planetary_Gravity_Policy,
) -> Pose_Callbacks
{
	return {
		initialize=planetary_gravity_initialize,
		prepare_for_integration=planetary_gravity_prepare,
		integrate_velocity=planetary_gravity_velocity,
		dispose=planetary_gravity_dispose,
		angular_mode=.Nonconserving,
		allow_substeps_for_unconstrained=.Disabled,
		integrate_kinematic_velocity=.Disabled,
		user_context=policy,
	};
}

// per_body_gravity_policy creates a caller-owned gravity gather policy around a
// Body_Property_Table(Vector3). allocation: none
per_body_gravity_policy :: #force_inline proc "contextless" (
	gravities: ^Body_Property_Table(Vector3),
	linear_damping: f32 = 0,
	angular_damping: f32 = 0,
) -> Per_Body_Gravity_Policy
{
	return {
		gravities=gravities,
		linear_damping=linear_damping,
		angular_damping=angular_damping,
	};
}

@(private)
per_body_gravity_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation,
) -> Status
{
	if user_context == nil || simulation == nil
	{
		return .Invalid_Argument;
	}
	policy := (^Per_Body_Gravity_Policy)(user_context);
	if policy.gravities == nil || policy.gravities.storage.state != .Ready ||
		policy.linear_damping < 0 || policy.linear_damping > 1 ||
		policy.angular_damping < 0 || policy.angular_damping > 1
	{
		return .Invalid_Description;
	}
	policy.simulation = simulation;
	return .Ok;
}

@(private)
per_body_gravity_prepare :: proc "contextless" (
	user_context: rawptr, dt: f32,
) -> Status
{
	if user_context == nil || dt <= 0
	{
		return .Invalid_Argument;
	}
	policy := (^Per_Body_Gravity_Policy)(user_context);
	linear_base := clamp(1 - policy.linear_damping, f32(0), f32(1));
	angular_base := clamp(1 - policy.angular_damping, f32(0), f32(1));
	policy.linear_damping_dt = util.F32x8(f32(math.pow(f64(linear_base), f64(dt))));
	policy.angular_damping_dt = util.F32x8(f32(math.pow(f64(angular_base), f64(dt))));
	return .Ok;
}

@(private)
per_body_gravity_velocity :: #force_inline proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	_, _, _, _, _ = position, orientation, inertia, integration_mask, worker_index;
	policy := (^Per_Body_Gravity_Policy)(user_context);
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
		gravity, status := body_property_get(policy.gravities, handle);
		if status == .Ok
		{
			util.vector3_wide_write_slot(&gravity_wide, lane, gravity^);
		}
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

@(private)
per_body_gravity_dispose :: proc "contextless" (user_context: rawptr)
{
	if user_context == nil
	{
		return;
	}
	policy := (^Per_Body_Gravity_Policy)(user_context);
	policy.simulation = nil;
}

// pose_policy_per_body builds the wide per-body gravity callback table. reads
// are direct handle-indexed table lookups and perform no allocation
pose_policy_per_body :: #force_inline proc "contextless" (
	policy: ^Per_Body_Gravity_Policy,
) -> Pose_Callbacks
{
	return {
		initialize=per_body_gravity_initialize,
		prepare_for_integration=per_body_gravity_prepare,
		integrate_velocity=per_body_gravity_velocity,
		dispose=per_body_gravity_dispose,
		angular_mode=.Nonconserving,
		allow_substeps_for_unconstrained=.Disabled,
		integrate_kinematic_velocity=.Disabled,
		user_context=policy,
	};
}

// Callback_Table_State distinguishes an empty callback table from a configured table
Callback_Table_State :: enum u8
{
	Empty,
	Configured,
}

@(private)
narrow_callbacks_state :: #force_inline proc "contextless" (
	callbacks: Narrow_Callbacks,
) -> Callback_Table_State
{
	if callbacks.initialize == nil && callbacks.allow == nil &&
		callbacks.allow_child == nil && callbacks.configure == nil &&
		callbacks.configure_child == nil && callbacks.dispose == nil &&
		callbacks.select_constraint == nil && callbacks.user_context == nil
	{
		return .Empty;
	}
	return .Configured;
}

@(private)
pose_callbacks_state :: #force_inline proc "contextless" (
	callbacks: Pose_Callbacks,
) -> Callback_Table_State
{
	if callbacks.initialize == nil &&
		callbacks.prepare_for_integration == nil &&
		callbacks.integrate_velocity == nil && callbacks.dispose == nil &&
		callbacks.angular_mode == .Nonconserving &&
		callbacks.allow_substeps_for_unconstrained == .Disabled &&
		callbacks.integrate_kinematic_velocity == .Disabled &&
		callbacks.user_context == nil
	{
		return .Empty;
	}
	return .Configured;
}

// world_description_set_callbacks installs caller-owned raw callback tables.
// both contexts must outlive the initialized world and must not be moved or
// mutated concurrently with world_step. the world calls each table's dispose
// callback during destruction but never frees the context pointer itself
world_description_set_callbacks :: proc "contextless" (
	description: ^World_Description,
	narrow: Narrow_Callbacks,
	pose: Pose_Callbacks,
) -> Status
{
	if description == nil
	{
		return .Invalid_Argument;
	}
	if physics.narrow_phase_callbacks_validate(narrow) != .Ok ||
		physics.pose_integrator_callbacks_validate(pose) != .Ok
	{
		return .Invalid_Description;
	}
	description.narrow_callbacks = narrow;
	description.pose_callbacks = pose;
	return .Ok;
}
