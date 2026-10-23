#+vet !unused-imports

package entasis_c

import "base:intrinsics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	abi_global_query_ray_payload :: struct
	{
		segment: Entasis_World_Precision_Segment,
		filter: Entasis_Query_Filter,
		output: Entasis_Query_Output,
	}
	#assert(size_of(abi_global_query_ray_payload) <= size_of(Entasis_Global_Query) - offset_of(Entasis_Global_Query, payload));
	
	abi_global_query_sweep_payload :: struct
	{
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		velocity: Entasis_Body_Velocity,
		maximum_t: f32,
		settings: Entasis_Sweep_Settings,
		filter: Entasis_Query_Filter,
		output: Entasis_Query_Output,
	}
	#assert(size_of(abi_global_query_sweep_payload) <= size_of(Entasis_Global_Query) - offset_of(Entasis_Global_Query, payload));
	
	abi_global_query_overlap_payload :: struct
	{
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		filter: Entasis_Query_Filter,
		output: Entasis_Query_Output,
	}
	#assert(size_of(abi_global_query_overlap_payload) <= size_of(Entasis_Global_Query) - offset_of(Entasis_Global_Query, payload));
	
	abi_global_query_volume_payload :: struct
	{
		bounds: Entasis_Global_Bounds,
		filter: Entasis_Query_Filter,
		output: Entasis_Query_Output,
	}
	#assert(size_of(abi_global_query_volume_payload) <= size_of(Entasis_Global_Query) - offset_of(Entasis_Global_Query, payload));
	
	abi_query_ray_global_any :: proc "contextless" (
		segment: Entasis_World_Precision_Segment,
		filter: ^Entasis_Query_Filter,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=0};
		stored: abi_global_query_ray_payload = {segment=segment, filter=abi_query_filter_value(filter)};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
	
	abi_query_ray_global_closest :: proc "contextless" (
		segment: Entasis_World_Precision_Segment,
		filter: ^Entasis_Query_Filter,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=1};
		stored: abi_global_query_ray_payload = {segment=segment, filter=abi_query_filter_value(filter)};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
	
	abi_query_ray_global_all :: proc "contextless" (
		segment: Entasis_World_Precision_Segment,
		output: Entasis_Query_Output,
		filter: ^Entasis_Query_Filter,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=2};
		stored: abi_global_query_ray_payload = {segment=segment, filter=abi_query_filter_value(filter), output=output};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
	
	abi_query_sweep_global_any :: proc "contextless" (
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		velocity: Entasis_Body_Velocity,
		maximum_t: f32,
		filter: ^Entasis_Query_Filter,
		settings: ^Entasis_Sweep_Settings,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=3};
		stored: abi_global_query_sweep_payload = {shape=shape, pose=pose, velocity=velocity, maximum_t=maximum_t, settings=abi_sweep_settings_value(settings), filter=abi_query_filter_value(filter)};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
	
	abi_query_sweep_global_closest :: proc "contextless" (
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		velocity: Entasis_Body_Velocity,
		maximum_t: f32,
		filter: ^Entasis_Query_Filter,
		settings: ^Entasis_Sweep_Settings,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=4};
		stored: abi_global_query_sweep_payload = {shape=shape, pose=pose, velocity=velocity, maximum_t=maximum_t, settings=abi_sweep_settings_value(settings), filter=abi_query_filter_value(filter)};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
	
	abi_query_sweep_global_all :: proc "contextless" (
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		velocity: Entasis_Body_Velocity,
		maximum_t: f32,
		output: Entasis_Query_Output,
		filter: ^Entasis_Query_Filter,
		settings: ^Entasis_Sweep_Settings,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=5};
		stored: abi_global_query_sweep_payload = {shape=shape, pose=pose, velocity=velocity, maximum_t=maximum_t, settings=abi_sweep_settings_value(settings), filter=abi_query_filter_value(filter), output=output};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
	
	abi_query_overlap_global_any :: proc "contextless" (
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		filter: ^Entasis_Query_Filter,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=6};
		stored: abi_global_query_overlap_payload = {shape=shape, pose=pose, filter=abi_query_filter_value(filter)};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
	
	abi_query_overlap_global_all :: proc "contextless" (
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		output: Entasis_Query_Output,
		filter: ^Entasis_Query_Filter,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=7};
		stored: abi_global_query_overlap_payload = {shape=shape, pose=pose, filter=abi_query_filter_value(filter), output=output};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
	
	abi_query_volume_global_all :: proc "contextless" (
		bounds: Entasis_Global_Bounds,
		output: Entasis_Query_Output,
		filter: ^Entasis_Query_Filter,
	) -> Entasis_Global_Query
	{
		result: Entasis_Global_Query = {kind=8};
		stored: abi_global_query_volume_payload = {bounds=bounds, filter=abi_query_filter_value(filter), output=output};
		intrinsics.mem_copy_non_overlapping(&result.payload[0], &stored, size_of(stored));
		return result;
	}
}
