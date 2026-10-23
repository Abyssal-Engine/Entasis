#+vet !unused-imports

package entasis_c

import "base:runtime"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	abi_global_state_views :: proc "contextless" (
		selection: ^Entasis_Global_State_Selection, snapshot: ^Entasis_Global_State_Snapshot,
	) -> (entasis.Global_State_Selection, entasis.Global_State_Snapshot, entasis.Status)
	{
		if selection == nil || snapshot == nil
		{
			return {}, {}, .Invalid_Argument;
		}
		if !abi_count_valid(selection.bodies_count) || !abi_count_valid(snapshot.bodies_capacity) ||
			(selection.bodies_count > 0 && selection.bodies == nil) || (snapshot.bodies_capacity > 0 && snapshot.bodies == nil)
		{
			return {}, {}, .Invalid_Argument;
		}
		if !abi_count_valid(selection.statics_count) || !abi_count_valid(snapshot.statics_capacity) ||
			(selection.statics_count > 0 && selection.statics == nil) || (snapshot.statics_capacity > 0 && snapshot.statics == nil)
		{
			return {}, {}, .Invalid_Argument;
		}
		if !abi_count_valid(selection.constraint_targets_count) || !abi_count_valid(snapshot.constraint_targets_capacity) ||
			(selection.constraint_targets_count > 0 && selection.constraint_targets == nil) || (snapshot.constraint_targets_capacity > 0 && snapshot.constraint_targets == nil)
		{
			return {}, {}, .Invalid_Argument;
		}
		if !abi_count_valid(selection.kinematic_targets_count) || !abi_count_valid(snapshot.kinematic_targets_capacity) ||
			(selection.kinematic_targets_count > 0 && selection.kinematic_targets == nil) || (snapshot.kinematic_targets_capacity > 0 && snapshot.kinematic_targets == nil)
		{
			return {}, {}, .Invalid_Argument;
		}
		return {
			bodies=(cast([^]entasis.Body_Handle)selection.bodies)[:int(selection.bodies_count)],
			statics=(cast([^]entasis.Static_Handle)selection.statics)[:int(selection.statics_count)],
			constraint_targets=(cast([^]entasis.Constraint_Handle)selection.constraint_targets)[:int(selection.constraint_targets_count)],
			kinematic_targets=(cast([^]entasis.Body_Handle)selection.kinematic_targets)[:int(selection.kinematic_targets_count)],
		}, {
			bodies=(cast([^]entasis.Global_Body_State)snapshot.bodies)[:int(snapshot.bodies_capacity)],
			statics=(cast([^]entasis.Global_Static_State)snapshot.statics)[:int(snapshot.statics_capacity)],
			constraint_targets=(cast([^]entasis.Global_Constraint_Target_State)snapshot.constraint_targets)[:int(snapshot.constraint_targets_capacity)],
			kinematic_targets=(cast([^]entasis.Global_Kinematic_Target_State)snapshot.kinematic_targets)[:int(snapshot.kinematic_targets_capacity)],
		}, .Ok;
	}
	
	abi_world_export_global_states :: proc "contextless" (
		world: ^Entasis_World,
		selection: ^Entasis_Global_State_Selection,
		snapshot: ^Entasis_Global_State_Snapshot,
		out_counts: ^Entasis_Global_State_Counts,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_counts == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_counts^ = {};
		core_selection: entasis.Global_State_Selection;
		core_snapshot: entasis.Global_State_Snapshot;
		status: entasis.Status;
		core_selection, core_snapshot, status = abi_global_state_views(selection, snapshot);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		counts: entasis.Global_State_Counts;
		counts, status = entasis.world_export_global_states(&resource.world, core_selection, core_snapshot);
		out_counts^ = {bodies=u64(counts.bodies), statics=u64(counts.statics),
			constraint_targets=u64(counts.constraint_targets), kinematic_targets=u64(counts.kinematic_targets)};
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_world_apply_global_states :: proc "contextless" (
		world: ^Entasis_World,
		selection: ^Entasis_Global_State_Selection,
		snapshot: ^Entasis_Global_State_Snapshot,
		awakening: Entasis_Awakening_Policy,
		target_wake: Entasis_Body_Control_Wake,
		out_counts: ^Entasis_Global_State_Counts,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_counts == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_counts^ = {};
		core_selection: entasis.Global_State_Selection;
		core_snapshot: entasis.Global_State_Snapshot;
		status: entasis.Status;
		core_selection, core_snapshot, status = abi_global_state_views(selection, snapshot);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None);
		}
		wake: entasis.Awakening_Policy;
		wake, status = abi_awakening_to_core(awakening);
		if status != .Ok || target_wake > 1
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		counts: entasis.Global_State_Counts;
		counts, status = entasis.world_apply_global_states(&resource.world, core_selection, core_snapshot, wake, entasis.Body_Control_Wake(target_wake));
		out_counts^ = {bodies=u64(counts.bodies), statics=u64(counts.statics),
			constraint_targets=u64(counts.constraint_targets), kinematic_targets=u64(counts.kinematic_targets)};
		return abi_status_finish(status, diagnostic, .None);
	}
}
