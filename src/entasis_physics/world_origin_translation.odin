package entasis_physics

import util "entasis:entasis_utilities"

// simulation_translate_origin applies one exclusive allocation-free local shift
// while preserving handles, topology, sleeping state, impulses and relative caches
simulation_translate_origin :: proc "contextless" (
	simulation: ^Simulation,
	delta: util.Vector3,
) -> Physics_Status
{
	if simulation == nil || simulation.state == .Disposed
	{
		return .Disposed;
	}
	if simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	if world_precision_vector_finite(delta) == .Missing
	{
		return .Invalid_Argument;
	}
	if delta == {}
	{
		return .Ok;
	}
	simulation.state = .Stepping;
	defer simulation.state = .Ready;
	status: Physics_Status = world_precision_validate_body_positions(&simulation.bodies, delta);
	if status != .Ok
	{
		return status;
	}
	status = world_precision_validate_static_positions(&simulation.statics, delta);
	if status != .Ok
	{
		return status;
	}
	status = world_precision_update_origin_targets(simulation, delta, .Validate);
	if status != .Ok
	{
		return status;
	}
	status = world_precision_validate_broad_phase(&simulation.broad_phase, delta);
	if status != .Ok
	{
		return status;
	}

	world_precision_translate_body_positions(&simulation.bodies, delta);
	world_precision_translate_static_positions(&simulation.statics, delta);
	_ = world_precision_update_origin_targets(simulation, delta, .Commit);
	world_precision_commit_origin_bounds(&simulation.broad_phase.active_tree);
	world_precision_commit_origin_bounds(&simulation.broad_phase.static_tree);
	return .Ok;
}
