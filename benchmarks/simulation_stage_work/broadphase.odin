package simulation_stage_work

import physics "entasis:entasis_physics"

pair_key :: proc "contextless" (a, b: physics.Collidable_Reference) -> u64
{
	lo, hi: u32 = a.packed, b.packed;
	if lo > hi
	{
		lo, hi = hi, lo;
	}
	return u64(lo) << 32 | u64(hi);
}

pair_collect :: proc "contextless" (user_context: rawptr, worker: int, pair: physics.Broad_Phase_Pair) -> physics.Physics_Status
{
	output: ^Pair_Output = (^Pair_Output)(user_context);
	count: int = output.counts[worker];
	if count == output.capacity
	{
		return .Capacity_Missing;
	}
	output.keys[worker*output.capacity+count] = pair_key(pair.a, pair.b);
	output.counts[worker] = count+1;
	return .Ok;
}

pairs_reset :: proc(output: ^Pair_Output)
{
	for &count in output.counts
	{
		count = 0;
	}
}
