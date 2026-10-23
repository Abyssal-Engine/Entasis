package precision_queries_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

@(test)
global_ray_two_millimeter_gaps_and_closed_endpoints :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		shape: physics.Typed_Index = add_shape(t, &scene, .Box);
		center: physics.Global_Position = {origin + 0.125, -origin + 0.25, origin - 0.375};
		body: physics.Body_Handle = add_body(t, &scene, shape, center);
		stack: [128]i32;
		for axis in 0 ..< 3
		{
			for reverse in 0 ..< 2
			{
				for side in ([2]f64{-1, 1})
				{
					for gap in ([2]f64{-0.002, 0.002})
					{
						start: [3]f64;
						end: [3]f64;
						start[axis] = -2;
						end[axis] = 2;
						start[(axis + 1) % 3] = side * (0.5 + gap);
						end[(axis + 1) % 3] = start[(axis + 1) % 3];
						if reverse != 0
						{
							start, end = end, start;
						}
						segment: physics.World_Precision_Segment = translated_segment({start={start[0], start[1], start[2]}, end={end[0], end[1], end[2]}}, center);
						hit: entasis.Global_Ray_Hit;
						status: entasis.Status;
						hit, status = entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
						presence: entasis.Query_Hit_State;
						any_status: entasis.Status;
						presence, any_status = entasis.ray_cast_global_any_with_context(&scene.query, segment, stack[:]);
						testing.expect_value(t, any_status, entasis.Status.Ok);
						if gap > 0
						{
							testing.expect_value(t, status, entasis.Status.Not_Found);
							testing.expect_value(t, presence, entasis.Query_Hit_State.Miss);
						}
						else
						{
							testing.expect_value(t, status, entasis.Status.Ok);
							testing.expect_value(t, presence, entasis.Query_Hit_State.Hit);
							testing.expect_value(t, physics.collidable_reference_raw_handle(hit.collidable), body.value);
							testing.expectf(t, abs(hit.t - 0.375) <= 1e-8, "origin %v axis %v fraction %v", origin, axis, hit.t);
						}
					}
				}
			}
		}
		for endpoint_delta in ([3]f64{-0.002, 0, 0.002})
		{
			segment: physics.World_Precision_Segment = translated_segment({start={-2, 0, 0}, end={-0.5 + endpoint_delta, 0, 0}}, center);
			hit: entasis.Global_Ray_Hit;
			status: entasis.Status;
			hit, status = entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
			if endpoint_delta < 0
			{
				testing.expect_value(t, status, entasis.Status.Not_Found);
			}
			else
			{
				testing.expect_value(t, status, entasis.Status.Ok);
				testing.expect(t, hit.t <= 1);
				expect_position(t, hit.location, {center.x - 0.5, center.y, center.z});
			}
		}
		scene_dispose(t, &scene);
	}
}

// independent scalar slab oracle operates on exact f64 input endpoints and
// known box geometry. it does not call production clipping/localization code
box_segment_oracle :: proc "contextless" (segment: physics.World_Precision_Segment, center: physics.Global_Position) -> (f64, physics.Reference_State)
{
	a: [3]f64 = {segment.start.x, segment.start.y, segment.start.z};
	b: [3]f64 = {segment.end.x, segment.end.y, segment.end.z};
	c: [3]f64 = {center.x, center.y, center.z};
	enter: f64 = 0;
	exit: f64 = 1;
	for axis in 0 ..< 3
	{
		d: f64 = b[axis] - a[axis];
		if d == 0
		{
			if a[axis] < c[axis] - 0.5 || a[axis] > c[axis] + 0.5
			{
				return 0, .Missing;
			}
		}
		else
		{
			first: f64 = (c[axis] - 0.5 - a[axis]) / d;
			last: f64 = (c[axis] + 0.5 - a[axis]) / d;
			enter = max(enter, min(first, last));
			exit = min(exit, max(first, last));
		}
	}
	return enter, .Present if enter <= exit else .Missing;
}

@(test)
global_ray_planetary_segments_match_independent_oracle :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	centers: [5]physics.Global_Position;
	for x, index in ([5]f64{-1e9, -1e6, 0, 1e6, 1e9})
	{
		centers[index] = {x + 0.125, 0.5 * x + 0.25, -0.25 * x - 0.375};
		if index % 2 == 0
		{
			_ = add_body(t, &scene, shape, centers[index]);
		}
		else
		{
			_ = add_static(t, &scene, shape, centers[index]);
		}
	}
	stack: [128]i32;
	hits: [8]entasis.Global_Ray_Hit;
	for distance in ([3]f64{2e9, 4e9, 1e12})
	{
		for reverse in 0 ..< 2
		{
			segment: physics.World_Precision_Segment = {
				start={-distance + 0.125, -0.5 * distance + 0.25, 0.25 * distance - 0.375},
				end={distance + 0.125, 0.5 * distance + 0.25, -0.25 * distance - 0.375},
			};
			if reverse != 0
			{
				segment.start, segment.end = segment.end, segment.start;
			}
			count: int;
			status: entasis.Status;
			count, status = entasis.ray_cast_global_all_with_context(&scene.query, segment, hits[:], stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, count, len(centers));
			matched: u32;
			for hit in hits[:count]
			{
				for center, index in centers
				{
					expected: f64;
					state: physics.Reference_State;
					expected, state = box_segment_oracle(segment, center);
					if abs(hit.location.x - center.x) <= 0.50001
					{
						testing.expect_value(t, state, physics.Reference_State.Present);
						testing.expectf(t, abs(hit.t - expected) <= 2e-15, "actual %v expected %v", hit.t, expected);
						matched |= u32(1) << u32(index);
					}
				}
			}
			testing.expect_value(t, matched, u32(31));
		}
	}
}

@(test)
global_ray_equal_fraction_order_and_remote_binary_slope :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	_ = add_body(t, &scene, shape, {1e9 + 4, 0, 0});
	near: physics.Static_Handle = add_static(t, &scene, shape, {1e9 - 4, 0, 0});
	stack: [128]i32;
	hit, status := entasis.ray_cast_global_closest_with_context(&scene.query,
		{start={-1e40, 0, 0}, end={1e40, 0, 0}}, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, physics.collidable_reference_mobility(hit.collidable), physics.Body_Mobility.Static);
	testing.expect_value(t, physics.collidable_reference_raw_handle(hit.collidable), near.value);
	testing.expect_value(t, hit.location.x, 1e9 - 4.5);
	// nonzero target offsets must survive subtraction from very remote ray
	// endpoints, even when the normalized output fractions round identically
	for offset in ([2]f64{0.748, 0.752})
	{
		handle: physics.Static_Handle = add_static(t, &scene, shape, {1e9, 0.5e9 + offset, 8});
		hit, status = entasis.ray_cast_global_closest_with_context(&scene.query,
			{start={-1e40, -0.5e40, 8}, end={1e40, 0.5e40, 8}}, stack[:]);
		if offset < 0.75
		{
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, physics.collidable_reference_raw_handle(hit.collidable), handle.value);
			testing.expect(t, abs(hit.location.y - (0.5e9 + offset - 0.5)) <= 0.000002);
		}
		else
		{
			testing.expect_value(t, status, entasis.Status.Not_Found);
		}
		testing.expect_value(t, entasis.static_remove(&scene.world, handle), entasis.Status.Ok);
	}
}

@(test)
global_ray_initial_inside_hit_preserves_original_high_start :: proc(t: ^testing.T)
{
	for kind in ([2]Shape_Kind{.Sphere, .Box})
	{
		for origin in ([3]f64{0, 1e4, 1e9})
		{
			scene: Scene;
			scene_initialize(t, &scene);
			shape: physics.Typed_Index = add_shape(t, &scene, kind);
			center: physics.Global_Position = {origin + 0.125, -origin + 0.25, origin - 0.375};
			_ = add_body(t, &scene, shape, center);
			stack: [128]i32;
			for axis in 0 ..< 3
			{
				for direction in ([2]f64{-1, 1})
				{
					start: physics.Global_Position = {center.x + 0.123456789, center.y + 0.234567891, center.z + 0.135791113};
					end: [3]f64 = {start.x, start.y, start.z};
					end[axis] += direction * 2;
					hit, status := entasis.ray_cast_global_closest_with_context(&scene.query, {start=start, end={end[0], end[1], end[2]}}, stack[:]);
					testing.expectf(t, status == .Ok, "kind %v origin %v axis %v direction %v status %v", kind, origin, axis, direction, status);
					testing.expect_value(t, hit.t, f64(0));
					expect_position(t, hit.location, start, 0);
				}
			}
			scene_dispose(t, &scene);
		}
	}
}
