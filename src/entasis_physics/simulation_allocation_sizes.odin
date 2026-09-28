// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

Simulation_Allocation_Sizes :: struct
{
	bodies:                               i32,
	statics:                              i32,
	inactive_body_sets:                   i32,
	shapes_per_type:                      i32,
	constraints:                          i32,
	constraint_batches:                   i32,
	initial_constraints_per_type_batch:   i32,
	minimum_constraints_per_body:         i32,
	broad_phase_candidates:               i32,
	pairs:                                i32,
	collision_child_pairs:                i32,
	inactive_pairs:                       i32,
	pending_pairs_per_worker:             i32,
	workers:                              i32,
}

simulation_allocation_sizes_default :: proc "contextless" () -> Simulation_Allocation_Sizes
{
	return {
		bodies=4096,
		statics=4096,
		inactive_body_sets=64,
		shapes_per_type=128,
		constraints=8192,
		constraint_batches=DEFAULT_FALLBACK_BATCH_THRESHOLD + 1,
		initial_constraints_per_type_batch=64,
		minimum_constraints_per_body=8,
		broad_phase_candidates=16384,
		pairs=16384,
		collision_child_pairs=65536,
		inactive_pairs=8192,
		pending_pairs_per_worker=2048,
		workers=1,
	};
}

simulation_allocation_sizes_normalize :: proc "contextless" (
	sizes: Simulation_Allocation_Sizes, solve_description: Solve_Description,
) -> (Simulation_Allocation_Sizes, Physics_Status)
{
	if solve_description_validate(solve_description) != .Ok ||
		solve_description.fallback_batch_threshold >= max(i32)
	{
		return {}, .Invalid_Description;
	}
	normalized := sizes;
	normalized.constraint_batches = solve_description.fallback_batch_threshold + 1;
	normalized.collision_child_pairs = max(
		normalized.collision_child_pairs, normalized.workers,
	);
	if simulation_allocation_sizes_validate(normalized) != .Ok
	{
		return {}, .Invalid_Description;
	}
	return normalized, .Ok;
}

simulation_transaction_capacity :: proc "contextless" (
	broad_phase_candidate_capacity, pair_capacity: int,
) -> (int, Physics_Status)
{
	if broad_phase_candidate_capacity <= 0 || pair_capacity <= 0 ||
		broad_phase_candidate_capacity > int(max(i32)) - pair_capacity
	{
		return 0, .Invalid_Description;
	}
	transaction_capacity := broad_phase_candidate_capacity + pair_capacity;
	if transaction_capacity > max(int) / size_of(Narrow_Phase_Transaction_Header) ||
		broad_phase_candidate_capacity > max(int) / size_of(Narrow_Phase_Contact_Transaction) ||
		pair_capacity > max(int) / size_of(Narrow_Phase_Remove_Transaction)
	{
		return 0, .Invalid_Description;
	}
	return transaction_capacity, .Ok;
}

simulation_allocation_sizes_validate :: proc "contextless" (sizes: Simulation_Allocation_Sizes) -> Physics_Status
{
	if sizes.bodies <= 0 || sizes.statics <= 0 || sizes.inactive_body_sets <= 0 || sizes.shapes_per_type <= 0 ||
		sizes.constraints <= 0 || sizes.constraint_batches <= 1 || sizes.initial_constraints_per_type_batch <= 0 ||
		sizes.minimum_constraints_per_body <= 0 || sizes.broad_phase_candidates <= 0 || sizes.pairs <= 0 ||
		sizes.collision_child_pairs <= 0 || sizes.inactive_pairs <= 0 ||
		sizes.pending_pairs_per_worker <= 0 || sizes.workers <= 0 ||
		sizes.workers > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Invalid_Description;
	}
	_, transaction_status := simulation_transaction_capacity(
		int(sizes.broad_phase_candidates), int(sizes.pairs),
	);
	if transaction_status != .Ok
	{
		return transaction_status;
	}
	return .Ok;
}
