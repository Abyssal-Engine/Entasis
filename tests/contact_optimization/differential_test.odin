package contact_optimization_tests

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Comparison_Result :: enum
{
	Equal,
	Different,
}

random_u32 :: proc(seed: ^u32) -> u32
{
	seed^ = seed^ * 1664525 + 1013904223;
	return seed^;
}

random_unit :: proc(seed: ^u32) -> f32
{
	return f32(random_u32(seed) >> 8) / 16777216;
}

random_orientation :: proc(seed: ^u32, mode: int) -> util.Quaternion
{
	if mode == 0
	{
		return {0, 0, 0, 1};
	}
	scale: f32 = 1;
	if mode == 1
	{
		scale = 0.00001;
	}
	q: util.Quaternion = {
		(random_unit(seed) * 2 - 1) * scale,
		(random_unit(seed) * 2 - 1) * scale,
		(random_unit(seed) * 2 - 1) * scale,
		1,
	};
	length: f32 = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	return {q.x / length, q.y / length, q.z / length, q.w / length};
}

compare_vector_bits :: proc(a, b: util.Vector3) -> Comparison_Result
{
	if transmute(u32)a.x != transmute(u32)b.x ||
		transmute(u32)a.y != transmute(u32)b.y ||
		transmute(u32)a.z != transmute(u32)b.z
	{
		return .Different;
	}
	return .Equal;
}

compare_manifold_bits :: proc(
	a, b: ^physics.Convex_4_Contact_Manifold_Wide,
	oa, ob: util.Vector3_Wide,
) -> Comparison_Result
{
	for lane in 0 ..< 8
	{
		ma, mb: physics.Convex_Contact_Manifold;
		physics.convex_4_manifold_wide_read_lane_core(a, oa, lane, &ma);
		physics.convex_4_manifold_wide_read_lane_core(b, ob, lane, &mb);
		if ma.count != mb.count
		{
			return .Different;
		}
		if ma.count == 0
		{
			continue;
		}
		if compare_vector_bits(ma.normal, mb.normal) != .Equal ||
			compare_vector_bits(ma.offset_b, mb.offset_b) != .Equal
		{
			return .Different;
		}
		for i in 0 ..< int(ma.count)
		{
			ca: physics.Convex_Contact = ma.contacts[i];
			cb: physics.Convex_Contact = mb.contacts[i];
			if ca.feature_id != cb.feature_id ||
				transmute(u32)ca.depth != transmute(u32)cb.depth ||
				compare_vector_bits(ca.offset, cb.offset) != .Equal
			{
				return .Different;
			}
		}
	}
	return .Equal;
}

@(test)
contact_reduction_matches_frozen_reference_for_all_lane_counts :: proc(t: ^testing.T)
{
	seed: u32 = 0x24c5a190;
	for pair_count in 0 ..= 8
	{
		for _ in 0 ..< 10000
		{
			ca: [8]physics.Manifold_Candidate_Wide;
			counts: util.I32x8 = util.I32x8(0);
			max_count: int = 0;
			for lane in 0 ..< 8
			{
				n: int = int(random_u32(&seed) % 9);
				counts = simd.replace(counts, lane, i32(n));
				if lane < pair_count
				{
					max_count = max(max_count, n);
				}
				for j in 0 ..< 8
				{
					ca[j].x = simd.replace(ca[j].x, lane, random_unit(&seed) * 4 - 2);
					ca[j].y = simd.replace(ca[j].y, lane, random_unit(&seed) * 4 - 2);
					ca[j].depth = simd.replace(ca[j].depth, lane, random_unit(&seed));
					ca[j].feature_id = simd.replace(ca[j].feature_id, lane, i32(j + 17 * lane));
				}
			}
			cb: [8]physics.Manifold_Candidate_Wide = ca;
			normal: util.Vector3_Wide = {util.F32x8(0.1), util.F32x8(1), util.F32x8(-0.2)};
			center: util.Vector3_Wide = {
				util.F32x8(0.2), util.F32x8(random_unit(&seed) - 0.5), util.F32x8(0.1),
			};
			tx: util.Vector3_Wide = {util.F32x8(1), util.F32x8(0), util.F32x8(0)};
			ty: util.Vector3_Wide = {util.F32x8(0), util.F32x8(0), util.F32x8(1)};
			a, ae := reference_manifold_candidate_wide_reduce(
				&ca, counts, max_count, normal, util.F32x8(-1), center, tx, ty,
				util.F32x8(1), util.F32x8(-0.1), pair_count,
			);
			b, be := physics.manifold_candidate_wide_reduce(
				&cb, counts, max_count, normal, util.F32x8(-1), center, tx, ty,
				util.F32x8(1), util.F32x8(-0.1), pair_count,
			);
			for c in 0 ..< 4
			{
				for lane in 0 ..< 8
				{
					if simd.extract(ae[c], lane) != simd.extract(be[c], lane) ||
						transmute(u32)simd.extract(a[c].x, lane) != transmute(u32)simd.extract(b[c].x, lane) ||
						transmute(u32)simd.extract(a[c].y, lane) != transmute(u32)simd.extract(b[c].y, lane) ||
						transmute(u32)simd.extract(a[c].depth, lane) != transmute(u32)simd.extract(b[c].depth, lane) ||
						simd.extract(a[c].feature_id, lane) != simd.extract(b[c].feature_id, lane)
					{
						testing.fail_now(t, "candidate reduction differs from frozen reference");
					}
				}
			}
		}
	}
}

@(test)
box_gather_and_manifold_match_frozen_reference_for_full_and_partial_bundles :: proc(t: ^testing.T)
{
	seed: u32 = 0xaf732569;
	for pair_count in 1 ..= 8
	{
		for trial in 0 ..< 12500
		{
			boxes_a, boxes_b: [12]physics.Box;
			records: [12]physics.Narrow_Phase_Convex_Direct_Record;
			start: int = trial % 5;
			for i in 0 ..< 12
			{
				boxes_a[i] = {
					random_unit(&seed) * 2 + 0.01,
					random_unit(&seed) * 2 + 0.01,
					random_unit(&seed) * 2 + 0.01,
				};
				boxes_b[i] = {
					random_unit(&seed) * 2 + 0.01,
					random_unit(&seed) * 2 + 0.01,
					random_unit(&seed) * 2 + 0.01,
				};
				qa: util.Quaternion = random_orientation(&seed, trial % 4);
				qb: util.Quaternion = random_orientation(&seed, trial % 4);
				pa: util.Vector3 = {random_unit(&seed) * 10, random_unit(&seed) * 10, random_unit(&seed) * 10};
				offset: util.Vector3 = {
					random_unit(&seed) * 8 - 4, random_unit(&seed) * 8 - 4, random_unit(&seed) * 8 - 4,
				};
				if trial % 4 == 0
				{
					offset = {0, boxes_a[i].half_height + boxes_b[i].half_height + f32(trial % 3 - 1) * 0.001, 0};
				}
				records[i] = {
					pair_id=i32(i), speculative_margin=random_unit(&seed) * 0.2,
					shape_data_a=&boxes_a[i], shape_data_b=&boxes_b[i],
					pose_a={position=pa, orientation=qa},
					pose_b={position=util.vector3_add(pa, offset), orientation=qb},
				};
			}
			a, b: physics.Convex_4_Contact_Manifold_Wide;
			oa, ob: util.Vector3_Wide;
			reference_narrow_phase_box_direct_test_group(&records[0], start, pair_count, &oa, &a);
			physics.narrow_phase_box_direct_test_group(&records[0], start, pair_count, &ob, &b);
			if compare_manifold_bits(&a, &b, oa, ob) != .Equal
			{
				testing.fail_now(t, "box gather or manifold differs from frozen reference");
			}
		}
	}
}
