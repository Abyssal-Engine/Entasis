package entasis

import "base:runtime"
import "core:mem"
import physics "entasis:entasis_physics"

// Damping contains the per-second damping fractions used by the default pose
// integrator. both values must be in the inclusive range [0, 1]
Damping :: struct
{
	linear:  f32,
	angular: f32,
}

// Capacity_Hints contains initial storage hints. each storage area may grow after
// initialization. fallback batch storage and worker scratch are derived from
// Solve_Description and Threading_Description rather than duplicated here
Capacity_Hints :: struct
{
	bodies:                             i32,
	statics:                            i32,
	inactive_body_sets:                 i32,
	shapes_per_type:                    i32,
	constraints:                        i32,
	initial_constraints_per_type_batch: i32,
	minimum_constraints_per_body:       i32,
	broad_phase_candidates:             i32,
	pairs:                              i32,
	collision_child_pairs:              i32,
	inactive_pairs:                     i32,
	pending_pairs_per_worker:           i32,
}

// Substep_Velocity_Iteration_Scheduler_Proc selects a positive velocity iteration count for one substep
Substep_Velocity_Iteration_Scheduler_Proc :: physics.Substep_Velocity_Iteration_Scheduler_Proc;
// Solve_Description exposes the stable built-in solve settings. constraint
// batch capacity is derived from fallback_batch_threshold + 1
Solve_Description :: struct
{
	velocity_iterations:      i32,
	substeps:                 i32,
	fallback_batch_threshold: i32,
	velocity_iteration_scheduler: Substep_Velocity_Iteration_Scheduler_Proc,
	scheduler_context: rawptr,
}

// Threading_Description selects either the included dispatcher owned by the
// world or an external dispatcher owned by the caller
//
// when external_dispatcher is non-nil, its worker_count is authoritative and
// worker_count and worker_pool_block_size are ignored. the world never shuts
// down an external dispatcher
Threading_Description :: struct
{
	worker_count:           i32,
	worker_pool_block_size: i32,
	external_dispatcher:    ^Dispatcher,
}

// World_Description contains all ordinary world initialization policy
//
// allocator owns the facade resource block in Legacy. explicit All_Owned world
// initialization extends it to owned pools, workers and extension storage
World_Description :: struct
{
	// profiling enables the existing low-level per-stage profiler. it adds timestamp reads
	// around simulation stages and is intended for diagnostics rather than production timing
	profiling: bool,
	gravity:   Vector3,
	damping:   Damping,
	capacity:  Capacity_Hints,
	solve:     Solve_Description,
	threading: Threading_Description,
	allocator: mem.Allocator,

	// zero callback tables keep the built-in callbacks and use
	// gravity and damping above. world_description_set_callbacks installs
	// caller-owned custom tables
	narrow_callbacks: Narrow_Callbacks,
	pose_callbacks:   Pose_Callbacks,

	// advanced caller-supplied timestep policy. zero values retain the
	// default timestepper and no stage callbacks
	timestepper:       Timestepper,
	timestep_callbacks: Timestep_Callbacks,
}

// capacity_hints_default returns the low-level default initial capacities while
// omitting settings derived from solve and threading policy
capacity_hints_default :: proc "contextless" () -> Capacity_Hints
{
	sizes := physics.simulation_allocation_sizes_default();
	return {
		bodies=sizes.bodies,
		statics=sizes.statics,
		inactive_body_sets=sizes.inactive_body_sets,
		shapes_per_type=sizes.shapes_per_type,
		constraints=sizes.constraints,
		initial_constraints_per_type_batch=sizes.initial_constraints_per_type_batch,
		minimum_constraints_per_body=sizes.minimum_constraints_per_body,
		broad_phase_candidates=sizes.broad_phase_candidates,
		pairs=sizes.pairs,
		collision_child_pairs=sizes.collision_child_pairs,
		inactive_pairs=sizes.inactive_pairs,
		pending_pairs_per_worker=sizes.pending_pairs_per_worker,
	};
}

// solve_description_default returns the existing solver defaults
solve_description_default :: proc "contextless" () -> Solve_Description
{
	description := physics.solve_description_default();
	return {
		velocity_iterations=description.velocity_iteration_count,
		substeps=description.substep_count,
		fallback_batch_threshold=description.fallback_batch_threshold,
		velocity_iteration_scheduler=description.velocity_iteration_scheduler,
		scheduler_context=description.scheduler_context,
	};
}

// threading_description_default selects a caller-thread-only world and creates
// no background worker
threading_description_default :: proc "contextless" () -> Threading_Description
{
	return {
		worker_count=1,
		worker_pool_block_size=16384,
	};
}

// world_description_default returns a complete ordinary world description
world_description_default :: proc () -> World_Description
{
	return {
		profiling=false,
		gravity={0, -9.81, 0},
		damping={linear=0.01, angular=0.01},
		capacity=capacity_hints_default(),
		solve=solve_description_default(),
		threading=threading_description_default(),
		allocator=runtime.heap_allocator(),
	};
}
