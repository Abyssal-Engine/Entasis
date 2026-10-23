package precision_broadphase_tests

import "core:testing"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

body_precise_bounds :: body_bounds;
static_precise_bounds :: static_bounds;

// fixture coordinates are exact dyadic values. this oracle never converts the
// widened tree bounds back to double and does not call a production converter
expect_precise_enclosure :: proc(t: ^testing.T, actual: p.Global_Bounds, position: p.Global_Position, local: u.Bounding_Box)
{
	testing.expect(t, actual.min.x <= position.x + f64(local.min.x));
	testing.expect(t, actual.min.y <= position.y + f64(local.min.y));
	testing.expect(t, actual.min.z <= position.z + f64(local.min.z));
	testing.expect(t, actual.max.x >= position.x + f64(local.max.x));
	testing.expect(t, actual.max.y >= position.y + f64(local.max.y));
	testing.expect(t, actual.max.z >= position.z + f64(local.max.z));
}

expect_precise_exact :: proc(t: ^testing.T, actual: p.Global_Bounds, position: p.Global_Position, local: u.Bounding_Box)
{
	testing.expect_value(t, actual, p.Global_Bounds{
		min={position.x + f64(local.min.x), position.y + f64(local.min.y), position.z + f64(local.min.z)},
		max={position.x + f64(local.max.x), position.y + f64(local.max.y), position.z + f64(local.max.z)},
	});
}

pair_count :: proc(t: ^testing.T, scene: ^Scene) -> int
{
	storage: [8192]p.Broad_Phase_Pair;
	count: int;
	status: p.Physics_Status;
	count, status = p.broad_phase_find_pairs(&scene.simulation.broad_phase,
		u.Buffer(p.Broad_Phase_Pair){memory=&storage[0], length=i32(len(storage)), id=-1});
	testing.expect_value(t, status, p.Physics_Status.Ok);
	return count;
}
