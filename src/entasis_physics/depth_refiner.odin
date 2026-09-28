// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
// the generated C# file, not DepthRefiner.tt, is the semantic source for the vertex,
// edge, face, and degenerate simplex cases
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

DEPTH_REFINER_INITIAL_MAXIMUM_ITERATIONS :: 25;
DEPTH_REFINER_REUSED_SIMPLEX_MAXIMUM_ITERATIONS :: 50;
DEPTH_REFINER_SIMPLEX_DEGENERACY_SCALE :: f32(1e-10);
DEPTH_REFINER_VERTEX_DEGENERACY_EPSILON :: f32(1e-14);
Depth_Refiner_Vertex :: struct
{
	support:    util.Vector3,
	support_a:  util.Vector3,
}

Depth_Refiner_Simplex :: struct
{
	vertices: [3]Depth_Refiner_Vertex,
	count:    int,
}

Depth_Refiner_Closest :: struct
{
	point:      util.Vector3,
	witness_a:  util.Vector3,
	distance_2: f32,
	mask:       u8,
}

depth_refiner_support :: proc "contextless" (
	a, b: Collision_Shape_View, direction: util.Vector3, shapes: ^Shape_Registry,
) -> (Depth_Refiner_Vertex, Physics_Status)
{
	support_a, status_a := collision_support_world(a, direction, shapes);
	if status_a != .Ok
	{
		return {}, status_a;
	}
	support_b, status_b := collision_support_world(b, util.vector3_negate(direction), shapes);
	if status_b != .Ok
	{
		return {}, status_b;
	}
	return {support=util.vector3_subtract(support_a, support_b), support_a=support_a}, .Ok;
}

depth_refiner_closest_segment :: proc "contextless" (
	a, b: Depth_Refiner_Vertex, target: util.Vector3,
) -> Depth_Refiner_Closest
{
	edge := util.vector3_subtract(b.support, a.support);
	length_squared := util.vector3_length_squared(edge);
	if length_squared <= DEPTH_REFINER_VERTEX_DEGENERACY_EPSILON
	{
		offset := util.vector3_subtract(a.support, target);
		return {a.support, a.support_a, util.vector3_length_squared(offset), 1};
	}
	t := max(f32(0), min(f32(1), util.vector3_dot(util.vector3_subtract(target, a.support), edge) / length_squared));
	point := util.vector3_add(a.support, util.vector3_scale(edge, t));
	witness := util.vector3_add(a.support_a, util.vector3_scale(util.vector3_subtract(b.support_a, a.support_a), t));
	mask := u8(3);
	if t == 0
	{
		mask = 1;
	}
	else if t == 1
	{
		mask = 2;
	}
	return {point, witness, util.vector3_length_squared(util.vector3_subtract(point, target)), mask};
}

depth_refiner_closest_triangle :: proc "contextless" (
	a, b, c: Depth_Refiner_Vertex, target: util.Vector3,
) -> Depth_Refiner_Closest
{
	ab := util.vector3_subtract(b.support, a.support);
	ac := util.vector3_subtract(c.support, a.support);
	triangle_normal := util.vector3_cross(ab, ac);
	longest_edge_squared := max(
		util.vector3_length_squared(ab),
		max(
			util.vector3_length_squared(util.vector3_subtract(c.support, b.support)),
			util.vector3_length_squared(ac),
		),
	);
	if util.vector3_length_squared(triangle_normal) <= longest_edge_squared * DEPTH_REFINER_SIMPLEX_DEGENERACY_SCALE
	{
		ab_closest := depth_refiner_closest_segment(a, b, target);
		bc_closest := depth_refiner_closest_segment(b, c, target);
		ca_closest := depth_refiner_closest_segment(c, a, target);
		if ab_closest.distance_2 <= bc_closest.distance_2 && ab_closest.distance_2 <= ca_closest.distance_2
		{
			return ab_closest;
		}
		if bc_closest.distance_2 <= ca_closest.distance_2
		{
			bc_closest.mask = (bc_closest.mask << 1) & 6;
			return bc_closest;
		}
		if ca_closest.mask == 1
		{
			ca_closest.mask = 4;
		}
		else if ca_closest.mask == 2
		{
			ca_closest.mask = 1;
		}
		else
		{
			ca_closest.mask = 5;
		}
		return ca_closest;
	}
	ap := util.vector3_subtract(target, a.support);
	d1 := util.vector3_dot(ab, ap);
	d2 := util.vector3_dot(ac, ap);
	if d1 <= 0 && d2 <= 0
	{
		return {a.support, a.support_a, util.vector3_length_squared(ap), 1};
	}
	bp := util.vector3_subtract(target, b.support);
	d3 := util.vector3_dot(ab, bp);
	d4 := util.vector3_dot(ac, bp);
	if d3 >= 0 && d4 <= d3
	{
		return {b.support, b.support_a, util.vector3_length_squared(bp), 2};
	}
	vc := d1*d4 - d3*d2;
	if vc <= 0 && d1 >= 0 && d3 <= 0
	{
		v := d1 / (d1 - d3);
		point := util.vector3_add(a.support, util.vector3_scale(ab, v));
		witness := util.vector3_add(
			a.support_a,
			util.vector3_scale(util.vector3_subtract(b.support_a, a.support_a), v)
		);
		return {point, witness, util.vector3_length_squared(util.vector3_subtract(point, target)), 3};
	}
	cp := util.vector3_subtract(target, c.support);
	d5 := util.vector3_dot(ab, cp);
	d6 := util.vector3_dot(ac, cp);
	if d6 >= 0 && d5 <= d6
	{
		return {c.support, c.support_a, util.vector3_length_squared(cp), 4};
	}
	vb := d5*d2 - d1*d6;
	if vb <= 0 && d2 >= 0 && d6 <= 0
	{
		w := d2 / (d2 - d6);
		point := util.vector3_add(a.support, util.vector3_scale(ac, w));
		witness := util.vector3_add(
			a.support_a,
			util.vector3_scale(util.vector3_subtract(c.support_a, a.support_a), w)
		);
		return {point, witness, util.vector3_length_squared(util.vector3_subtract(point, target)), 5};
	}
	va := d3*d6 - d5*d4;
	if va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0
	{
		bc := util.vector3_subtract(c.support, b.support);
		w := (d4 - d3) / ((d4 - d3) + (d5 - d6));
		point := util.vector3_add(b.support, util.vector3_scale(bc, w));
		witness := util.vector3_add(
			b.support_a,
			util.vector3_scale(util.vector3_subtract(c.support_a, b.support_a), w)
		);
		return {point, witness, util.vector3_length_squared(util.vector3_subtract(point, target)), 6};
	}
	denominator := 1 / (va + vb + vc);
	v := vb * denominator;
	w := vc * denominator;
	u := 1 - v - w;
	point := util.vector3_add(
		util.vector3_scale(a.support, u),
		util.vector3_add(util.vector3_scale(b.support, v), util.vector3_scale(c.support, w)),
	);
	witness := util.vector3_add(
		util.vector3_scale(a.support_a, u),
		util.vector3_add(util.vector3_scale(b.support_a, v), util.vector3_scale(c.support_a, w)),
	);
	return {point, witness, util.vector3_length_squared(util.vector3_subtract(point, target)), 7};
}

depth_refiner_reduce_simplex :: proc "contextless" (
	simplex: ^Depth_Refiner_Simplex, mask: u8,
)
{
	if simplex == nil
	{
		return;
	}
	reduced: [3]Depth_Refiner_Vertex;
	count := 0;
	for index in 0 ..< simplex.count
	{
		if mask & (u8(1) << u8(index)) != 0
		{
			reduced[count] = simplex.vertices[index];
			count += 1;
		}
	}
	simplex.vertices = reduced;
	simplex.count = count;
}

depth_refiner_closest_simplex :: proc "contextless" (
	simplex: ^Depth_Refiner_Simplex, target: util.Vector3,
) -> Depth_Refiner_Closest
{
	if simplex.count <= 1
	{
		vertex := simplex.vertices[0];
		offset := util.vector3_subtract(vertex.support, target);
		return {vertex.support, vertex.support_a, util.vector3_length_squared(offset), 1};
	}
	if simplex.count == 2
	{
		return depth_refiner_closest_segment(simplex.vertices[0], simplex.vertices[1], target);
	}
	return depth_refiner_closest_triangle(simplex.vertices[0], simplex.vertices[1], simplex.vertices[2], target);
}

depth_refiner_insert_support :: proc "contextless" (
	simplex: ^Depth_Refiner_Simplex, support: Depth_Refiner_Vertex, target: util.Vector3,
)
{
	if simplex.count < 3
	{
		simplex.vertices[simplex.count] = support;
		simplex.count += 1;
		return;
	}
	old := simplex^;
	best: Depth_Refiner_Closest;
	best.distance_2 = f32(math.F32_MAX);
	best_simplex := old;
	triangles := [3][3]Depth_Refiner_Vertex{
		{old.vertices[0], old.vertices[1], support},
		{old.vertices[1], old.vertices[2], support},
		{old.vertices[2], old.vertices[0], support},
	};
	for vertices in triangles
	{
		candidate := depth_refiner_closest_triangle(vertices[0], vertices[1], vertices[2], target);
		if candidate.distance_2 < best.distance_2
		{
			best = candidate;
			best_simplex = {vertices=vertices, count=3};
		}
	}
	simplex^ = best_simplex;
	depth_refiner_reduce_simplex(simplex, best.mask);
}

depth_refiner_find_minimum_depth :: proc "contextless" (
	a, b: Collision_Shape_View,
	initial_normal: util.Vector3,
	convergence_threshold, minimum_depth_threshold: f32,
	shapes: ^Shape_Registry,
	maximum_iterations: int = DEPTH_REFINER_INITIAL_MAXIMUM_ITERATIONS,
) -> (depth: f32, refined_normal, witness_on_a: util.Vector3, status: Physics_Status)
{
	if a.batch == nil || b.batch == nil
	{
		return 0, {}, {}, .Invalid_Argument;
	}
	if a.batch.dispatch == .Contextual || b.batch.dispatch == .Contextual
	{
		return depth_refiner_find_contextual(a, b, initial_normal, convergence_threshold,
			minimum_depth_threshold, shapes, maximum_iterations);
	}
	return depth_refiner_find_minimum_depth_kernel(a, b,
		initial_normal, convergence_threshold, minimum_depth_threshold, shapes, maximum_iterations);
}

// keep contextual view construction and specialization branching out of the
// native function's inlining budget. the native loop still resolves no binding
@(private)
depth_refiner_find_contextual :: #force_no_inline proc "contextless" (
	a, b: Collision_Shape_View, initial_normal: util.Vector3,
	convergence_threshold, minimum_depth_threshold: f32,
	shapes: ^Shape_Registry, maximum_iterations: int,
) -> (f32, util.Vector3, util.Vector3, Physics_Status)
{
	if a.batch.dispatch == .Contextual
	{
		contextual_a: Contextual_Collision_Shape_View = collision_contextual_shape_view(a);
		if b.batch.dispatch == .Contextual
		{
			return #force_no_inline depth_refiner_find_minimum_depth_kernel(contextual_a, collision_contextual_shape_view(b),
				initial_normal, convergence_threshold, minimum_depth_threshold, shapes, maximum_iterations);
		}
		return #force_no_inline depth_refiner_find_minimum_depth_kernel(contextual_a, b,
			initial_normal, convergence_threshold, minimum_depth_threshold, shapes, maximum_iterations);
	}
	return #force_no_inline depth_refiner_find_minimum_depth_kernel(a, collision_contextual_shape_view(b),
		initial_normal, convergence_threshold, minimum_depth_threshold, shapes, maximum_iterations);
}

depth_refiner_find_minimum_depth_kernel :: proc "contextless" (
	a: $A, b: $B,
	initial_normal: util.Vector3,
	convergence_threshold, minimum_depth_threshold: f32,
	shapes: ^Shape_Registry,
	maximum_iterations: int = DEPTH_REFINER_INITIAL_MAXIMUM_ITERATIONS,
) -> (depth: f32, refined_normal, witness_on_a: util.Vector3, status: Physics_Status)
{
	if shapes == nil || maximum_iterations <= 0 || convergence_threshold <= 0 ||
	util.vector3_length_squared(initial_normal) <= 1e-12
	{
		return 0, {}, {}, .Invalid_Argument;
	}
	normal := util.vector3_normalize(initial_normal);
	initial_support: Depth_Refiner_Vertex;
	support_status: Physics_Status;
	initial_support, support_status = depth_refiner_support_resolved(a, b, normal, shapes);
	if support_status != .Ok
	{
		return 0, {}, {}, support_status;
	}
	refined_normal = normal;
	depth = util.vector3_dot(initial_support.support, normal);
	witness_on_a = initial_support.support_a;
	if depth <= minimum_depth_threshold
	{
		return depth, refined_normal, witness_on_a, .Ok;
	}
	simplex := Depth_Refiner_Simplex{vertices={0=initial_support}, count=1};
	for _ in 0 ..< maximum_iterations
	{
		search_target := util.vector3_scale(refined_normal, max(f32(0), depth));
		closest: Depth_Refiner_Closest;
		when A == Collision_Shape_View && B == Collision_Shape_View
		{
			closest = #force_inline depth_refiner_closest_simplex(&simplex, search_target);
		}
		else
		{
			closest = depth_refiner_closest_simplex(&simplex, search_target);
		}
		depth_refiner_reduce_simplex(&simplex, closest.mask);
		offset := util.vector3_subtract(search_target, closest.point);
		termination_epsilon := convergence_threshold;
		if depth < 0
		{
			termination_epsilon -= depth;
		}
		if util.vector3_length_squared(offset) <= termination_epsilon*termination_epsilon
		{
			witness_on_a = closest.witness_a;
			break;
		}
		if depth > 0 && simplex.count < 3
		{
			offset = util.vector3_add(search_target, util.vector3_scale(offset, 4));
		}
		if util.vector3_length_squared(offset) <= DEPTH_REFINER_VERTEX_DEGENERACY_EPSILON
		{
			break;
		}
		normal = util.vector3_normalize(offset);
		support: Depth_Refiner_Vertex;
		next_support_status: Physics_Status;
		support, next_support_status = depth_refiner_support_resolved(a, b, normal, shapes);
		if next_support_status != .Ok
		{
			return 0, {}, {}, next_support_status;
		}
		candidate_depth := util.vector3_dot(support.support, normal);
		if candidate_depth < depth
		{
			depth = candidate_depth;
			refined_normal = normal;
			witness_on_a = support.support_a;
		}
		if depth <= minimum_depth_threshold
		{
			break;
		}
		when A == Collision_Shape_View && B == Collision_Shape_View
		{
			#force_inline depth_refiner_insert_support(&simplex, support, search_target);
		}
		else
		{
			depth_refiner_insert_support(&simplex, support, search_target);
		}
	}
	return depth, refined_normal, witness_on_a, .Ok;
}
