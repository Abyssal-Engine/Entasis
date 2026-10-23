#+vet !unused-imports

package entasis_c

import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	#assert(size_of(Entasis_World_Precision_Segment) == size_of(entasis.World_Precision_Segment));
	#assert(align_of(Entasis_World_Precision_Segment) == align_of(entasis.World_Precision_Segment));
	#assert(offset_of(Entasis_World_Precision_Segment, start) == offset_of(entasis.World_Precision_Segment, start));
	#assert(offset_of(Entasis_World_Precision_Segment, end) == offset_of(entasis.World_Precision_Segment, end));
	#assert(size_of(Entasis_Global_Bounds) == size_of(entasis.Global_Bounds));
	#assert(align_of(Entasis_Global_Bounds) == align_of(entasis.Global_Bounds));
	#assert(offset_of(Entasis_Global_Bounds, min) == offset_of(entasis.Global_Bounds, min));
	#assert(offset_of(Entasis_Global_Bounds, max) == offset_of(entasis.Global_Bounds, max));
	#assert(size_of(Entasis_Global_Ray_Hit) == size_of(entasis.Global_Ray_Hit));
	#assert(align_of(Entasis_Global_Ray_Hit) == align_of(entasis.Global_Ray_Hit));
	#assert(offset_of(Entasis_Global_Ray_Hit, t) == offset_of(entasis.Global_Ray_Hit, t));
	#assert(offset_of(Entasis_Global_Ray_Hit, location) == offset_of(entasis.Global_Ray_Hit, location));
	#assert(offset_of(Entasis_Global_Ray_Hit, normal) == offset_of(entasis.Global_Ray_Hit, normal));
	#assert(offset_of(Entasis_Global_Ray_Hit, collidable) == offset_of(entasis.Global_Ray_Hit, collidable));
	#assert(offset_of(Entasis_Global_Ray_Hit, child_index) == offset_of(entasis.Global_Ray_Hit, child_index));
	#assert(size_of(Entasis_Global_Sweep_Result) == size_of(entasis.Global_Sweep_Result));
	#assert(align_of(Entasis_Global_Sweep_Result) == align_of(entasis.Global_Sweep_Result));
	#assert(offset_of(Entasis_Global_Sweep_Result, state) == offset_of(entasis.Global_Sweep_Result, state));
	#assert(offset_of(Entasis_Global_Sweep_Result, t0) == offset_of(entasis.Global_Sweep_Result, t0));
	#assert(offset_of(Entasis_Global_Sweep_Result, t1) == offset_of(entasis.Global_Sweep_Result, t1));
	#assert(offset_of(Entasis_Global_Sweep_Result, location) == offset_of(entasis.Global_Sweep_Result, location));
	#assert(offset_of(Entasis_Global_Sweep_Result, normal) == offset_of(entasis.Global_Sweep_Result, normal));
	#assert(offset_of(Entasis_Global_Sweep_Result, child_a) == offset_of(entasis.Global_Sweep_Result, child_a));
	#assert(offset_of(Entasis_Global_Sweep_Result, child_b) == offset_of(entasis.Global_Sweep_Result, child_b));
	#assert(size_of(Entasis_Global_Sweep_Hit) == size_of(entasis.Global_Sweep_Hit));
	#assert(align_of(Entasis_Global_Sweep_Hit) == align_of(entasis.Global_Sweep_Hit));
	#assert(offset_of(Entasis_Global_Sweep_Hit, collidable) == offset_of(entasis.Global_Sweep_Hit, collidable));
	#assert(offset_of(Entasis_Global_Sweep_Hit, sweep) == offset_of(entasis.Global_Sweep_Hit, sweep));
	abi_query_context_ray_cast_global_any :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		ray_value: Entasis_World_Precision_Segment,
		filter: ^Entasis_Query_Filter,
		out_hit: ^Entasis_Bool,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_hit == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_hit^ = ENTASIS_FALSE;
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
		hit: entasis.Query_Hit_State;
		status: entasis.Status;
		hit, status = entasis.ray_cast_global_any_with_context(
			&resource.native, transmute(entasis.World_Precision_Segment)ray_value, stack, core_filter,
		);
		if status == .Ok
		{
			out_hit^ = abi_bool(hit == .Hit);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_ray_cast_global_closest :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		ray_value: Entasis_World_Precision_Segment,
		filter: ^Entasis_Query_Filter,
		out_hit: ^Entasis_Global_Ray_Hit,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_hit == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_hit^ = {child_index=-1};
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
		hit: entasis.Global_Ray_Hit;
		status: entasis.Status;
		hit, status = entasis.ray_cast_global_closest_with_context(
			&resource.native, transmute(entasis.World_Precision_Segment)ray_value, stack, core_filter,
		);
		if status == .Ok
		{
			out_hit^ = transmute(Entasis_Global_Ray_Hit)hit;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_ray_cast_global_all :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		ray_value: Entasis_World_Precision_Segment,
		filter: ^Entasis_Query_Filter,
		hits: [^]Entasis_Global_Ray_Hit,
		capacity: u64,
		out_written, out_required: ^u64,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_written != nil
		{
			out_written^ = 0;
		}
		if out_required != nil
		{
			out_required^ = 0;
		}
		if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
		(capacity > 0 && hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
		core_hits: [^]entasis.Global_Ray_Hit = cast([^]entasis.Global_Ray_Hit)hits;
		written: int;
		status: entasis.Status;
		written, status = entasis.ray_cast_global_all_with_context(
			&resource.native,
			transmute(entasis.World_Precision_Segment)ray_value,
			core_hits[:int(capacity)],
			stack, core_filter,
		);
		out_written^ = u64(max(written, 0));
		out_required^ = abi_query_required_lower_bound(written, status);
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_sweep_global_any :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		shape: Entasis_Shape_Handle,
		pose_value: Entasis_Global_Pose,
		velocity_value: Entasis_Body_Velocity,
		maximum_t: f32,
		filter: ^Entasis_Query_Filter,
		settings: ^Entasis_Sweep_Settings,
		out_hit: ^Entasis_Bool,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_hit == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_hit^ = ENTASIS_FALSE;
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
		hit: entasis.Query_Hit_State;
		status: entasis.Status;
		hit, status = entasis.sweep_global_any_with_context(
			&resource.native,
			abi_shape_handle_to_core(shape),
			abi_global_pose_to_core(pose_value),
			abi_velocity_to_core(velocity_value),
			maximum_t,
			stack, core_filter,
			abi_sweep_settings_to_core(abi_sweep_settings_value(settings)),
		);
		if status == .Ok
		{
			out_hit^ = abi_bool(hit == .Hit);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_sweep_global_closest :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		shape: Entasis_Shape_Handle,
		pose_value: Entasis_Global_Pose,
		velocity_value: Entasis_Body_Velocity,
		maximum_t: f32,
		filter: ^Entasis_Query_Filter,
		settings: ^Entasis_Sweep_Settings,
		out_hit: ^Entasis_Global_Sweep_Hit,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_hit == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_hit^ = {};
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
		hit: entasis.Global_Sweep_Hit;
		status: entasis.Status;
		hit, status = entasis.sweep_global_closest_with_context(
			&resource.native,
			abi_shape_handle_to_core(shape),
			abi_global_pose_to_core(pose_value),
			abi_velocity_to_core(velocity_value),
			maximum_t,
			stack, core_filter,
			abi_sweep_settings_to_core(abi_sweep_settings_value(settings)),
		);
		if status == .Ok
		{
			out_hit^ = transmute(Entasis_Global_Sweep_Hit)hit;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_sweep_global_all :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		shape: Entasis_Shape_Handle,
		pose_value: Entasis_Global_Pose,
		velocity_value: Entasis_Body_Velocity,
		maximum_t: f32,
		filter: ^Entasis_Query_Filter,
		settings: ^Entasis_Sweep_Settings,
		hits: [^]Entasis_Global_Sweep_Hit,
		capacity: u64,
		out_written, out_required: ^u64,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_written != nil
		{
			out_written^ = 0;
		}
		if out_required != nil
		{
			out_required^ = 0;
		}
		if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
		(capacity > 0 && hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
		core_hits: [^]entasis.Global_Sweep_Hit = cast([^]entasis.Global_Sweep_Hit)hits;
		written: int;
		status: entasis.Status;
		written, status = entasis.sweep_global_all_with_context(
			&resource.native,
			abi_shape_handle_to_core(shape),
			abi_global_pose_to_core(pose_value),
			abi_velocity_to_core(velocity_value),
			maximum_t,
			core_hits[:int(capacity)],
			stack, core_filter,
			abi_sweep_settings_to_core(abi_sweep_settings_value(settings)),
		);
		out_written^ = u64(max(written, 0));
		out_required^ = abi_query_required_lower_bound(written, status);
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_overlap_global_any :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		filter: ^Entasis_Query_Filter,
		out_state: ^Entasis_Overlap_State,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		filter_value: Entasis_Query_Filter = abi_query_filter_value(filter);
		if out_state == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_state^ = Entasis_Overlap_State(entasis.Overlap_State.Separated);
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(filter_value, &bridge);
		state: entasis.Overlap_State;
		status: entasis.Status;
		state, status = entasis.overlap_global_any_with_context(&resource.native, abi_shape_handle_to_core(shape), abi_global_pose_to_core(pose), stack, core_filter);
		if status == .Ok
		{
			out_state^ = Entasis_Overlap_State(state);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_overlap_global_all :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		shape: Entasis_Shape_Handle,
		pose_value: Entasis_Global_Pose,
		filter: ^Entasis_Query_Filter,
		hits: [^]Entasis_Overlap_Hit,
		capacity: u64,
		out_written, out_required: ^u64,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_written != nil
		{
			out_written^ = 0;
		}
		if out_required != nil
		{
			out_required^ = 0;
		}
		if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
		(capacity > 0 && hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
		core_hits: [^]entasis.Overlap_Hit = cast([^]entasis.Overlap_Hit)hits;
		written: int;
		status: entasis.Status;
		written, status = entasis.overlap_global_all_with_context(
			&resource.native,
			abi_shape_handle_to_core(shape),
			abi_global_pose_to_core(pose_value),
			core_hits[:int(capacity)],
			stack, core_filter,
		);
		out_written^ = u64(max(written, 0));
		out_required^ = abi_query_required_lower_bound(written, status);
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_volume_global_all :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		bounds: Entasis_Global_Bounds,
		filter: ^Entasis_Query_Filter,
		hits: [^]Entasis_Volume_Hit,
		capacity: u64,
		out_written, out_required: ^u64,
		traversal_stack: [^]i32, stack_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_written != nil
		{
			out_written^ = 0;
		}
		if out_required != nil
		{
			out_required^ = 0;
		}
		if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
		(capacity > 0 && hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(stack_capacity) || (stack_capacity > 0 && traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		stack: []i32 = traversal_stack[:int(stack_capacity)];
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		bridge: abi_query_filter_bridge;
		core_filter: entasis.Query_Filter = abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
		core_hits: [^]entasis.Volume_Hit = cast([^]entasis.Volume_Hit)hits;
		written: int;
		status: entasis.Status;
		written, status = entasis.volume_global_all_with_context(
			&resource.native,
			transmute(entasis.Global_Bounds)bounds,
			core_hits[:int(capacity)],
			stack, core_filter,
		);
		out_written^ = u64(max(written, 0));
		out_required^ = abi_query_required_lower_bound(written, status);
		return abi_status_finish(status, diagnostic, .None);
	}
}
