package phase6_simulation_codegen

import "base:intrinsics"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(export, enable_target_feature="avx")
phase6_integration_flag_merge_256_kernel :: proc "c" (target, source: [^]u8)
{
	target_buffer := util.Buffer(u8){memory=target, length=64, id=-1};
	source_buffer := util.Buffer(u8){memory=source, length=64, id=-1};
	if physics.solver_merge_integration_flags_256(&target_buffer, &source_buffer, 64) != .Ok
	{
		intrinsics.trap();
	}
}

@(export, enable_target_feature="avx512f")
phase6_integration_flag_merge_512_kernel :: proc "c" (target, source: [^]u8)
{
	target_buffer := util.Buffer(u8){memory=target, length=64, id=-1};
	source_buffer := util.Buffer(u8){memory=source, length=64, id=-1};
	if physics.solver_merge_integration_flags_512_kernel(&target_buffer, &source_buffer, 64) != .Ok
	{
		intrinsics.trap();
	}
}

main :: proc()
{
	target: [64]u8;
	source: [64]u8;
	for index in 0 ..< len(source)
	{
		source[index] = u8(1 << u32(index & 7));
	}
	phase6_integration_flag_merge_256_kernel(&target[0], &source[0]);
	for index in 0 ..< len(target)
	{
		if target[index] != source[index]
		{
			intrinsics.trap();
		}
	}
}
