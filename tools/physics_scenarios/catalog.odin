package physics_scenarios

import scene "../physics_scene"

Scenario :: enum
{
	Falling_Box,
	Minimal_World,
	Fixed_Step_Loop,
	Sleep_And_Awaken,
	Box_Pile_Batch,
	Bulk_Body_Update,
	Planetary_Gravity,
	Compounds,
	Constraints,
	Kinematic_Platform,
	Per_Body_Gravity,
	Continuous_Collision,
	Substepping,
	Gyroscope,
	Convex_Hulls,
	Ray_Queries,
	Sweep_Queries,
	Batched_Queries,
	Meshes,
	Background_Cooking,
	Cloth,
	Rope,
	External_Dispatcher,
	Profiling_And_State_Export,
	Materials_And_Filtering,
	Ragdoll,
	Direct_Collision_Queries,
	Impulses_Solver_Contacts,
	Overlap_And_Events,
	Custom_Shape,
	Custom_Constraint,
	Character_Controller,
	Vehicle,
	Tank_Controller,
	Triggers,
}
Capability :: enum
{
	Physics, Operations, Input, Queries, Events, Workers, Extended,
}
Capabilities :: bit_set[Capability];
Presentation :: enum
{
	Motion, Diagnostic, Correctness,
}
Descriptor :: struct
{
	id, title: string,
	capabilities: Capabilities,
	presentation: Presentation,
}
CATALOG :: [Scenario]Descriptor{
	.Falling_Box = {"example/falling_box", "Falling box", {.Physics}, .Motion},
	.Minimal_World = {"example/minimal_world", "Minimal world", {.Physics}, .Correctness},
	.Fixed_Step_Loop = {"example/fixed_step_loop", "Fixed step loop", {.Operations}, .Correctness},
	.Sleep_And_Awaken = {"example/sleep_and_awaken", "Sleep and awaken", {.Operations}, .Correctness},
	.Box_Pile_Batch = {"example/box_pile_batch", "Box pile batch", {.Physics}, .Motion},
	.Bulk_Body_Update = {"example/bulk_body_update", "Bulk body update", {.Operations}, .Correctness},
	.Planetary_Gravity = {"example/planetary_gravity", "Planetary gravity", {.Physics}, .Correctness},
	.Compounds = {"example/compounds", "Compounds", {.Physics}, .Motion},
	.Constraints = {"example/constraints", "Constraints", {.Physics}, .Motion},
	.Kinematic_Platform = {"example/kinematic_platform", "Kinematic platform", {.Physics}, .Motion},
	.Per_Body_Gravity = {"example/per_body_gravity", "Per-body gravity", {.Physics}, .Motion},
	.Continuous_Collision = {"example/continuous_collision", "Continuous collision", {.Physics}, .Diagnostic},
	.Substepping = {"example/substepping", "Substepping", {.Physics}, .Diagnostic},
	.Gyroscope = {"example/gyroscope", "Gyroscope", {.Physics}, .Diagnostic},
	.Convex_Hulls = {"example/convex_hulls", "Convex hulls", {.Operations}, .Diagnostic},
	.Ray_Queries = {"example/ray_queries", "Ray queries", {.Operations, .Queries}, .Diagnostic},
	.Sweep_Queries = {"example/sweep_queries", "Sweep queries", {.Operations, .Queries}, .Diagnostic},
	.Batched_Queries = {"example/batched_queries", "Batched queries", {.Operations, .Queries}, .Diagnostic},
	.Meshes = {"example/meshes", "Meshes", {.Physics, .Queries}, .Motion},
	.Background_Cooking = {"example/background_cooking", "Background cooking", {.Operations, .Queries}, .Correctness},
	.Cloth = {"example/cloth", "Cloth", {.Physics}, .Motion},
	.Rope = {"example/rope", "Rope", {.Physics}, .Motion},
	.External_Dispatcher = {"example/external_dispatcher", "External dispatcher", {.Physics, .Workers}, .Correctness},
	.Profiling_And_State_Export = {"example/profiling_and_state_export", "Profiling / state", {.Physics}, .Correctness},
	.Materials_And_Filtering = {"example/materials_and_filtering", "Materials and filtering", {.Physics}, .Diagnostic},
	.Ragdoll = {"example/ragdoll", "Ragdoll", {.Physics}, .Motion},
	.Direct_Collision_Queries = {"example/direct_collision_queries", "Direct collision queries", {.Operations, .Queries}, .Diagnostic},
	.Impulses_Solver_Contacts = {"example/impulses_solver_contacts", "Impulses / solver contacts", {.Operations, .Events}, .Diagnostic},
	.Overlap_And_Events = {"example/overlap_and_events", "Overlap and events", {.Physics, .Queries, .Events}, .Diagnostic},
	.Custom_Shape = {"example/custom_shape", "Custom shape", {.Operations, .Queries}, .Diagnostic},
	.Custom_Constraint = {"example/custom_constraint", "Custom constraint", {.Physics}, .Correctness},
	.Character_Controller = {"example/character_controller", "Character controller", {.Physics, .Queries, .Input}, .Motion},
	.Vehicle = {"example/vehicle", "Vehicle", {.Physics, .Input}, .Motion},
	.Tank_Controller = {"example/tank_controller", "Tank controller", {.Physics, .Input}, .Motion},
	.Triggers = {"example/triggers", "Triggers", {.Physics, .Events}, .Diagnostic},
};

resolve :: proc(id: string) -> (Scenario, scene.Status)
{
	for entry, index in CATALOG
	{
		if entry.id == id
		{
			return Scenario(index), .Ok;
		}
	}
	return {}, .Invalid_Data;
}
