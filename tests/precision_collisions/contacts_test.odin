package precision_collisions_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_sphere_contacts_resolve_two_millimeter_hits_and_gaps :: proc(t: ^testing.T)
{
	offsets: [4]f64 = {0, 1e4, 1e6, 1e9};
	gaps: [2]f64 = {-0.002, 0.002};
	mobilities: [3]physics.Body_Mobility = {.Static, .Dynamic, .Kinematic};
	for offset in offsets
	{
		for axis in 0 ..< 3
		{
			for gap in gaps
			{
				for mobility in mobilities
				{
					scene: Scene;
					scene_initialize(t, &scene);
					shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
					position_a: physics.Global_Position = {offset + 0.03125, -offset + 0.0625, offset + 0.125};
					position_b: physics.Global_Position = position_a;
					distance: f64;
					switch axis
					{
					case 0:
						position_b.x += 1 + gap;
						distance = position_b.x - position_a.x;
					case 1:
						position_b.y += 1 + gap;
						distance = position_b.y - position_a.y;
					case 2:
						position_b.z += 1 + gap;
						distance = position_b.z - position_a.z;
					}
					_ = add_body(t, &scene, shape, position_a);
					if mobility == .Static
					{
						_ = add_static(t, &scene, shape, position_b);
					}
					else
					{
						_ = add_body(t, &scene, shape, position_b, mobility);
					}
					testing.expect_value(t, physics.narrow_phase_execute(&scene.simulation.narrow_phase), physics.Physics_Status.Ok);
					count: int = scene.simulation.narrow_phase.pair_cache.mapping.count;
					if gap > 0
					{
						testing.expect_value(t, count, 0);
					}
					else
					{
						testing.expect_value(t, count, 1);
						if count == 1
						{
							view: physics.Contact_Constraint_Data_View = contact_view(t, &scene, 0);
							testing.expect_value(t, view.contact_count, i32(1));
							depth: f32;
							if mobility == .Static
							{
								testing.expect_value(t, view.body_count, i32(1));
								contact: ^physics.Contact_1_One_Body = (^physics.Contact_1_One_Body)(&view.prestep[0]);
								depth = contact.contact_0.penetration_depth;
							}
							else
							{
								testing.expect_value(t, view.body_count, i32(2));
								contact: ^physics.Contact_1 = (^physics.Contact_1)(&view.prestep[0]);
								depth = contact.contact_0.penetration_depth;
								expected: util.Vector3 = {f32(position_b.x - position_a.x), f32(position_b.y - position_a.y), f32(position_b.z - position_a.z)};
								testing.expect_value(t, contact.offset_b, expected);
							}
							// independent analytic sphere distance, using actual stored f64 inputs
							testing.expect(t, abs(f64(depth) - (1 - distance)) < 1e-6);
						}
					}
					scene_dispose(t, &scene);
				}
			}
		}
	}
}

@(test)
precision_convex_and_child_contacts_match_near_frame :: proc(t: ^testing.T)
{
	kinds: [9]Shape_Kind = {.Sphere, .Capsule, .Box, .Triangle, .Cylinder, .Hull, .Compound, .Big_Compound, .Mesh};
	completions: [2]Completion = {.Stored, .Materialized};
	for completion in completions
	{
		for kind in kinds
		{
			for swap in 0 ..< 2
			{
				scenes: [2]Scene;
				for index in 0 ..< 2
				{
					scene: ^Scene = &scenes[index];
					scene_initialize(t, scene, completion);
					shape_a: physics.Typed_Index = add_shape(t, scene, .Sphere);
					shape_b: physics.Typed_Index = add_shape(t, scene, kind);
					base: f64 = f64(index) * 1e9;
					origin: physics.Global_Position = {base + 0.03125, -base + 0.0625, base + 0.125};
					a: physics.Global_Position = {origin.x - 0.125, origin.y + 0.875, origin.z - 0.125};
					if kind == .Triangle || kind == .Mesh
					{
						a.y = origin.y + 0.375;
					}
					if swap == 0
					{
						_ = add_body(t, scene, shape_a, a);
						_ = add_static(t, scene, shape_b, origin);
					}
					else
					{
						_ = add_body(t, scene, shape_b, origin);
						_ = add_static(t, scene, shape_a, a);
					}
					testing.expect_value(t, physics.narrow_phase_execute(&scene.simulation.narrow_phase), physics.Physics_Status.Ok);
					testing.expectf(t, scene.simulation.narrow_phase.pair_cache.mapping.count == 1, "kind=%v swap=%d frame=%d completion=%v", kind, swap, index, completion);
				}
				expect_contact_sets_equal(t, &scenes[0], &scenes[1]);
				scene_dispose(t, &scenes[1]);
				scene_dispose(t, &scenes[0]);
			}
		}
	}
}

@(test)
precision_nonconvex_child_manifolds_preserve_features :: proc(t: ^testing.T)
{
	kinds: [2]Shape_Kind = {.Compound, .Big_Compound};
	for kind in kinds
	{
		scenes: [2]Scene;
		for index in 0 ..< 2
		{
			scene: ^Scene = &scenes[index];
			scene_initialize(t, scene);
			shape: physics.Typed_Index = add_shape(t, scene, kind);
			base: f64 = f64(index) * 1e9;
			_ = add_body(t, scene, shape, {base + 0.03125, -base + 0.9375, base + 0.125});
			_ = add_static(t, scene, shape, {base + 0.03125, -base, base + 0.125});
			testing.expect_value(t, physics.narrow_phase_execute(&scene.simulation.narrow_phase), physics.Physics_Status.Ok);
			testing.expect_value(t, scene.simulation.narrow_phase.pair_cache.mapping.count, 1);
			if scene.simulation.narrow_phase.pair_cache.mapping.count == 1
			{
				view: physics.Contact_Constraint_Data_View = contact_view(t, scene, 0);
				testing.expect_value(t, view.kind, physics.Contact_Constraint_Kind.Nonconvex);
				testing.expect(t, view.contact_count >= 2);
			}
		}
		expect_contact_sets_equal(t, &scenes[0], &scenes[1]);
		scene_dispose(t, &scenes[1]);
		scene_dispose(t, &scenes[0]);
	}
}
