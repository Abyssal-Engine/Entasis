#+vet !unused-imports

package entasis
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// capture/apply selected authoring values in fixed stream order: bodies,
	// statics, absolute constraint targets, then queued kinematic targets. separate
	// streams avoid a largest-variant union per entity and need no owned scratch
	//
	// every buffer is admitted before work. later record failure returns committed
	// prefix counts. capture leaves later output untouched, import does not execute
	// later records. earlier body mutations may already have cancelled queued targets
	// import is not a multi-record transaction: underlying wake side effects on a
	// failing mutation retain their contract. duplicate handles execute in order
	//
	// coordinates use the native frame at capture. a snapshot does not follow later
	// origin shifts. the caller must translate all positions consistently before
	// importing into a different frame. read-only capture may use independent
	// buffers concurrently, mutations require exclusive idle-world ownership
	// neither operation restores topology, ID history, sleep counters, contacts,
	// impulses, callbacks, optional settings, hidden target phases or exact replay

	global_state_buffers_admit :: proc "contextless" (
		selection: Global_State_Selection, snapshot: Global_State_Snapshot,
	) -> Status
	{
		if len(snapshot.bodies) < len(selection.bodies) || len(snapshot.statics) < len(selection.statics) ||
		len(snapshot.constraint_targets) < len(selection.constraint_targets) ||
		len(snapshot.kinematic_targets) < len(selection.kinematic_targets)
		{
			return .Capacity_Missing;
		}
		return .Ok;
	}

	world_export_global_states :: proc (
		world: ^World, selection: Global_State_Selection, output: Global_State_Snapshot,
	) -> (Global_State_Counts, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return {}, status;
		}
		status = global_state_buffers_admit(selection, output);
		if status != .Ok
		{
			return {}, status;
		}
		counts: Global_State_Counts;
		for handle, index in selection.bodies
		{
			value: Global_Body_State;
			value, status = body_get_global_state_ready(&data.simulation, handle);
			if status != .Ok
			{
				return counts, status;
			}
			output.bodies[index] = value;
			counts.bodies += 1;
		}
		for handle, index in selection.statics
		{
			value: Global_Static_State;
			value, status = static_get_global_state_ready(&data.simulation, handle);
			if status != .Ok
			{
				return counts, status;
			}
			output.statics[index] = value;
			counts.statics += 1;
		}
		for handle, index in selection.constraint_targets
		{
			value: Global_Constraint_Target_State;
			value, status = constraint_get_global_target_state_ready(&data.simulation, handle);
			if status != .Ok
			{
				return counts, status;
			}
			output.constraint_targets[index] = value;
			counts.constraint_targets += 1;
		}
		for handle, index in selection.kinematic_targets
		{
			value: Global_Kinematic_Target_State;
			value, status = body_get_global_target_state_ready(&data.simulation, handle);
			if status != .Ok
			{
				return counts, status;
			}
			output.kinematic_targets[index] = value;
			counts.kinematic_targets += 1;
		}
		return counts, .Ok;
	}

	// complete body mutation cancels queued targets, so their stream is deliberately
	// last. import after authoring the destination's shapes/constraints and enabling
	// its optional controls. no implicit world construction or borrowed-ID remap
	world_apply_global_states :: proc (
		world: ^World, selection: Global_State_Selection, snapshot: Global_State_Snapshot,
		awakening: Awakening_Policy = .Overlaps, target_wake: Body_Control_Wake = .Wake,
	) -> (Global_State_Counts, Status)
	{
		status: Status;
		_, status = query_world_data(world);
		if status != .Ok
		{
			return {}, status;
		}
		status = global_state_buffers_admit(selection, snapshot);
		if status != .Ok
		{
			return {}, status;
		}
		if (awakening != .Overlaps && awakening != .None) ||
		(target_wake != .Wake && target_wake != .Preserve_Sleep)
		{
			return {}, .Invalid_Argument;
		}
		counts: Global_State_Counts;
		for handle, index in selection.bodies
		{
			status = body_apply_global_state(world, handle, snapshot.bodies[index]);
			if status != .Ok
			{
				return counts, status;
			}
			counts.bodies += 1;
		}
		for handle, index in selection.statics
		{
			status = static_apply_global_state(world, handle, snapshot.statics[index], awakening);
			if status != .Ok
			{
				return counts, status;
			}
			counts.statics += 1;
		}
		for handle, index in selection.constraint_targets
		{
			status = constraint_apply_global_target_state(world, handle, snapshot.constraint_targets[index]);
			if status != .Ok
			{
				return counts, status;
			}
			counts.constraint_targets += 1;
		}
		for handle, index in selection.kinematic_targets
		{
			status = body_apply_global_target_state(world, handle, snapshot.kinematic_targets[index], target_wake);
			if status != .Ok
			{
				return counts, status;
			}
			counts.kinematic_targets += 1;
		}
		return counts, .Ok;
	}
}
