package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

// on-demand, tolerance-bounded f32 geometry. simplex predicates and barycentric
// weights use f64. existing collision/sweep arithmetic is not changed
Distance_Query_Settings :: struct
{
	absolute_tolerance: f32,
	relative_tolerance: f32,
	maximum_iterations: i32,
}

Distance_Query_State :: enum u8
{
	Unresolved,
	Separated,
	Intersecting,
	Touching,
	Penetrating,
}

Distance_Query_Mode :: enum u8
{
	Distance,
	Penetration,
}

Distance_Query_Result :: struct
{
	state: Distance_Query_State,
	point_a, point_b: util.Vector3,
	normal: util.Vector3,
	distance, depth: f32,
	iterations: i32,
}

Distance_Query_Vertex :: struct
{
	point, point_a, point_b: util.Vector3,
}

Distance_Query_Face :: struct
{
	indices: [3]i32,
	normal: util.Vector3,
	distance: f32,
}

Distance_Query_Edge :: struct
{
	a, b: i32,
}

// exclusive caller-owned scratch. counts are reset per invocation. no pointer
// escapes the call, no allocation is performed, and errors publish no result
Distance_Query_Scratch :: struct
{
	vertices: []Distance_Query_Vertex,
	faces: []Distance_Query_Face,
	edges: []Distance_Query_Edge,
}

Distance_Query_Simplex :: struct
{
	vertices: [4]Distance_Query_Vertex,
	weights: [4]f64,
	count: int,
}

Distance_Query_Point :: struct
{
	point: util.Vector3,
}

distance_query_settings_default :: proc "contextless" () -> Distance_Query_Settings
{
	return {absolute_tolerance=1e-5, relative_tolerance=1e-5, maximum_iterations=128};
}

distance_query_settings_validate :: proc "contextless" (settings: Distance_Query_Settings) -> Physics_Status
{
	if !(settings.absolute_tolerance > 0 && settings.absolute_tolerance <= math.F32_MAX) ||
	!(settings.relative_tolerance > 0 && settings.relative_tolerance < 1) ||
	settings.maximum_iterations <= 0 || settings.maximum_iterations > 4096
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

distance_query_dot :: #force_inline proc "contextless" (a, b: util.Vector3) -> f64
{
	return f64(a.x)*f64(b.x) + f64(a.y)*f64(b.y) + f64(a.z)*f64(b.z);
}

distance_query_triple :: proc "contextless" (a, b, c: util.Vector3) -> f64
{
	return f64(a.x)*(f64(b.y)*f64(c.z)-f64(b.z)*f64(c.y)) +
	f64(a.y)*(f64(b.z)*f64(c.x)-f64(b.x)*f64(c.z)) +
	f64(a.z)*(f64(b.x)*f64(c.y)-f64(b.y)*f64(c.x));
}

distance_query_tetra_weights :: proc "contextless" (vertices: []Distance_Query_Vertex) -> ([4]f64, Reference_State)
{
	a: util.Vector3 = vertices[0].point;
	b: util.Vector3 = util.vector3_subtract(vertices[1].point, a);
	c: util.Vector3 = util.vector3_subtract(vertices[2].point, a);
	d: util.Vector3 = util.vector3_subtract(vertices[3].point, a);
	r: util.Vector3 = util.vector3_negate(a);
	determinant: f64 = distance_query_triple(b, c, d);
	if determinant == 0
	{
		return {}, .Missing;
	}
	x: f64 = distance_query_triple(r, c, d) / determinant;
	y: f64 = distance_query_triple(b, r, d) / determinant;
	z: f64 = distance_query_triple(b, c, r) / determinant;
	weights: [4]f64 = {1-x-y-z, x, y, z};
	for weight in weights
	{
		if !(weight >= 0 && weight <= 1)
		{
			return {}, .Missing;
		}
	}
	return weights, .Present;
}

// a simplex has at most six edges and four faces. degenerate faces reduce to
// their edges. no fixed epsilon from the conservative sweep solver is reused
distance_query_reduce :: proc "contextless" (simplex: ^Distance_Query_Simplex) -> util.Vector3
{
	if simplex.count == 4
	{
		weights: [4]f64;
		contains: Reference_State;
		weights, contains = distance_query_tetra_weights(simplex.vertices[:]);
		if contains == .Present
		{
			simplex.weights = weights;
			return {};
		}
	}
	best: f64 = math.F64_MAX;
	weights: [4]f64;
	for i in 0..<simplex.count
	{
		point: util.Vector3 = simplex.vertices[i].point;
		distance: f64 = distance_query_dot(point, point);
		if distance < best
		{
			best = distance;
			weights = {};
			weights[i] = 1;
		}
	}
	for i in 0..<simplex.count
	{
		for j in i+1..<simplex.count
		{
			a: util.Vector3 = simplex.vertices[i].point;
			edge: util.Vector3 = util.vector3_subtract(simplex.vertices[j].point, a);
			length_squared: f64 = distance_query_dot(edge, edge);
			if length_squared == 0
			{
				continue;
			}
			t: f64 = -distance_query_dot(a, edge) / length_squared;
			if t <= 0 || t >= 1
			{
				continue;
			}
			x: f64 = f64(a.x)+f64(edge.x)*t;
			y: f64 = f64(a.y)+f64(edge.y)*t;
			z: f64 = f64(a.z)+f64(edge.z)*t;
			distance: f64 = x*x+y*y+z*z;
			if distance < best
			{
				best = distance;
				weights = {};
				weights[i] = 1-t;
				weights[j] = t;
			}
		}
	}
	for i in 0..<simplex.count
	{
		for j in i+1..<simplex.count
		{
			for k in j+1..<simplex.count
			{
				a: util.Vector3 = simplex.vertices[i].point;
				ab: util.Vector3 = util.vector3_subtract(simplex.vertices[j].point, a);
				ac: util.Vector3 = util.vector3_subtract(simplex.vertices[k].point, a);
				aa: f64 = distance_query_dot(ab, ab);
				bb: f64 = distance_query_dot(ac, ac);
				cross: f64 = distance_query_dot(ab, ac);
				determinant: f64 = aa*bb-cross*cross;
				if determinant <= 0
				{
					continue;
				}
				r0: f64 = -distance_query_dot(a, ab);
				r1: f64 = -distance_query_dot(a, ac);
				v: f64 = (r0*bb-r1*cross) / determinant;
				w: f64 = (r1*aa-r0*cross) / determinant;
				u: f64 = 1-v-w;
				if u <= 0 || v <= 0 || w <= 0
				{
					continue;
				}
				x: f64 = f64(a.x)+f64(ab.x)*v+f64(ac.x)*w;
				y: f64 = f64(a.y)+f64(ab.y)*v+f64(ac.y)*w;
				z: f64 = f64(a.z)+f64(ab.z)*v+f64(ac.z)*w;
				distance: f64 = x*x+y*y+z*z;
				if distance < best
				{
					best = distance;
					weights = {};
					weights[i] = u;
					weights[j] = v;
					weights[k] = w;
				}
			}
		}
	}
	count: int;
	point: [3]f64;
	for i in 0..<simplex.count
	{
		if weights[i] <= 0
		{
			continue;
		}
		vertex: Distance_Query_Vertex = simplex.vertices[i];
		point[0] += f64(vertex.point.x)*weights[i];
		point[1] += f64(vertex.point.y)*weights[i];
		point[2] += f64(vertex.point.z)*weights[i];
		simplex.vertices[count] = vertex;
		simplex.weights[count] = weights[i];
		count += 1;
	}
	simplex.count = count;
	return {f32(point[0]), f32(point[1]), f32(point[2])};
}

distance_query_witnesses :: proc "contextless" (simplex: ^Distance_Query_Simplex) -> (util.Vector3, util.Vector3)
{
	a, b: [3]f64;
	for i in 0..<simplex.count
	{
		vertex: ^Distance_Query_Vertex = &simplex.vertices[i];
		w: f64 = simplex.weights[i];
		a[0] += f64(vertex.point_a.x)*w;
		a[1] += f64(vertex.point_a.y)*w;
		a[2] += f64(vertex.point_a.z)*w;
		b[0] += f64(vertex.point_b.x)*w;
		b[1] += f64(vertex.point_b.y)*w;
		b[2] += f64(vertex.point_b.z)*w;
	}
	return {f32(a[0]), f32(a[1]), f32(a[2])}, {f32(b[0]), f32(b[1]), f32(b[2])};
}

distance_query_support_point :: proc "contextless" (view: $V, direction: util.Vector3, shapes: ^Shape_Registry) -> (util.Vector3, Physics_Status)
{
	when V == Distance_Query_Point
	{
		return view.point, .Ok;
	}
	else when V == Contextual_Collision_Shape_View
	{
		return collision_support_world_contextual(view, direction, shapes);
	}
	else
	{
		#assert(V == Collision_Shape_View);
		return collision_support_world_native(view, direction, shapes);
	}
}

distance_query_support :: proc "contextless" (a: $A, b: $B, direction: util.Vector3, shapes: ^Shape_Registry) -> (Distance_Query_Vertex, Physics_Status)
{
	point_a, point_b: util.Vector3;
	status: Physics_Status;
	point_a, status = distance_query_support_point(a, direction, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	point_b, status = distance_query_support_point(b, util.vector3_negate(direction), shapes);
	if status != .Ok
	{
		return {}, status;
	}
	point: util.Vector3 = util.vector3_subtract(point_a, point_b);
	// valid input may still exceed representable f32 intermediate geometry
	if !(abs(point.x) <= math.F32_MAX && abs(point.y) <= math.F32_MAX && abs(point.z) <= math.F32_MAX)
	{
		return {}, .No_Convergence;
	}
	return {point=point, point_a=point_a, point_b=point_b}, .Ok;
}

// the closest-simplex norm is an upper bound. the support plane is a lower
// bound. stagnation without their convergence certificate is not a valid miss
distance_query_gjk :: proc "contextless" (
	a: $A, b: $B, shapes: ^Shape_Registry, settings: Distance_Query_Settings,
	simplex: ^Distance_Query_Simplex,
) -> (Distance_Query_Result, Physics_Status)
{
	initial: Distance_Query_Vertex;
	status: Physics_Status;
	initial, status = distance_query_support(a, b, {1, 0, 0}, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	simplex^ = {vertices={0=initial}, count=1};
	for iteration in 0..<settings.maximum_iterations
	{
		closest: util.Vector3 = distance_query_reduce(simplex);
		distance: f32 = f32(math.sqrt(distance_query_dot(closest, closest)));
		point_a, point_b: util.Vector3;
		point_a, point_b = distance_query_witnesses(simplex);
		if simplex.count == 0 || !(distance <= math.F32_MAX)
		{
			return {}, .No_Convergence;
		}
		if distance <= settings.absolute_tolerance
		{
			return {state=.Intersecting, point_a=point_a, point_b=point_b, iterations=iteration+1}, .Ok;
		}
		normal: util.Vector3 = util.vector3_scale(closest, 1/distance);
		next: Distance_Query_Vertex;
		next, status = distance_query_support(a, b, util.vector3_negate(normal), shapes);
		if status != .Ok
		{
			return {}, status;
		}
		lower: f32 = max(f32(0), f32(distance_query_dot(normal, next.point)));
		tolerance: f32 = max(settings.absolute_tolerance, settings.relative_tolerance*distance);
		if distance-lower <= tolerance
		{
			return {state=.Separated, point_a=point_a, point_b=point_b, normal=normal, distance=distance, iterations=iteration+1}, .Ok;
		}
		for i in 0..<simplex.count
		{
			if next.point == simplex.vertices[i].point
			{
				return {}, .No_Convergence;
			}
		}
		if simplex.count == 4
		{
			return {}, .No_Convergence;
		}
		simplex.vertices[simplex.count] = next;
		simplex.count += 1;
	}
	return {}, .No_Convergence;
}

distance_query_view :: proc "contextless" (
	shape: rawptr, type_id: int, pose: Rigid_Pose, shapes: ^Shape_Registry,
) -> (Collision_Shape_View, Physics_Status)
{
	view: Collision_Shape_View;
	status: Physics_Status;
	view, status = collision_shape_view(shape, type_id, pose, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	if view.batch.state != .Registered
	{
		return {}, .Not_Found;
	}
	if view.batch.metadata.batch_type != .Convex
	{
		return {}, .Invalid_Argument;
	}
	if view.batch.dispatch == .Native && view.batch.metadata.support == nil
	{
		return {}, .Not_Found;
	}
	if view.batch.dispatch == .Contextual && (view.batch.contextual_metadata.binding == nil || view.batch.contextual_metadata.binding.support == nil)
	{
		return {}, .Not_Found;
	}
	q: util.Quaternion = pose.orientation;
	length_squared: f32 = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
	if !(abs(pose.position.x) <= math.F32_MAX && abs(pose.position.y) <= math.F32_MAX && abs(pose.position.z) <= math.F32_MAX) ||
	!(abs(length_squared-1) <= 1e-4)
	{
		return {}, .Invalid_Argument;
	}
	return view, .Ok;
}

// low-level lockstep geometry reused by World/Query_Context entrypoints. payloads
// must be live, valid registered convex instances throughout the call
distance_query_convex :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int, pose_a, pose_b: Rigid_Pose,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings,
	mode: Distance_Query_Mode = .Distance, scratch: Distance_Query_Scratch = {},
) -> (Distance_Query_Result, Physics_Status)
{
	status: Physics_Status = distance_query_settings_validate(settings);
	if status != .Ok || mode > .Penetration
	{
		return {}, .Invalid_Argument;
	}
	a, b: Collision_Shape_View;
	a, status = distance_query_view(shape_a, type_a, pose_a, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	b, status = distance_query_view(shape_b, type_b, pose_b, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	return distance_query_convex_admitted(a, b, type_a, type_b, shapes, settings, mode, scratch);
}

// views/settings have been admitted by the scalar entry or composite owner.
// dispatch and common-origin conversion occur once per tested leaf pair
distance_query_convex_admitted :: proc "contextless" (
	input_a, input_b: Collision_Shape_View, type_a, type_b: int, shapes: ^Shape_Registry,
	settings: Distance_Query_Settings, mode: Distance_Query_Mode, scratch: Distance_Query_Scratch,
) -> (Distance_Query_Result, Physics_Status)
{
	a: Collision_Shape_View = input_a;
	b: Collision_Shape_View = input_b;
	if type_a == SPHERE_TYPE_ID && type_b == SPHERE_TYPE_ID
	{
		return distance_query_spheres((^Sphere)(a.shape)^, (^Sphere)(b.shape)^, a.pose.position, b.pose.position, settings, mode);
	}
	if type_a == SPHERE_TYPE_ID && type_b == TRIANGLE_TYPE_ID
	{
		return distance_query_sphere_triangle(a, b, settings, mode);
	}
	if type_a == TRIANGLE_TYPE_ID && type_b == SPHERE_TYPE_ID
	{
		result: Distance_Query_Result;
		status: Physics_Status;
		result, status = distance_query_sphere_triangle(b, a, settings, mode);
		if status != .Ok
		{
			return {}, status;
		}
		result.point_a, result.point_b = result.point_b, result.point_a;
		result.normal = util.vector3_negate(result.normal);
		return result, status;
	}
	origin: util.Vector3 = a.pose.position;
	a.pose.position = {};
	b.pose.position = util.vector3_subtract(b.pose.position, origin);
	result: Distance_Query_Result;
	status: Physics_Status;
	if a.batch.dispatch == .Contextual
	{
		if b.batch.dispatch == .Contextual
		{
			result, status = distance_query_kernel(collision_contextual_shape_view(a), collision_contextual_shape_view(b), shapes, settings, mode, scratch);
		}
		else
		{
			result, status = distance_query_kernel(collision_contextual_shape_view(a), b, shapes, settings, mode, scratch);
		}
	}
	else if b.batch.dispatch == .Contextual
	{
		result, status = distance_query_kernel(a, collision_contextual_shape_view(b), shapes, settings, mode, scratch);
	}
	else
	{
		result, status = distance_query_kernel(a, b, shapes, settings, mode, scratch);
	}
	if status == .Ok
	{
		result.point_a = util.vector3_add(result.point_a, origin);
		result.point_b = util.vector3_add(result.point_b, origin);
	}
	return result, status;
}

distance_query_point :: proc "contextless" (
	point: util.Vector3, shape: rawptr, type_id: int, pose: Rigid_Pose,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings,
) -> (Distance_Query_Result, Physics_Status)
{
	status: Physics_Status = distance_query_settings_validate(settings);
	if status != .Ok || !(abs(point.x) <= math.F32_MAX && abs(point.y) <= math.F32_MAX && abs(point.z) <= math.F32_MAX)
	{
		return {}, .Invalid_Argument;
	}
	view: Collision_Shape_View;
	view, status = distance_query_view(shape, type_id, pose, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	return distance_query_point_admitted(point, &view, type_id, shapes, settings);
}

distance_query_point_admitted :: proc "contextless" (
	point: util.Vector3, input_view: ^Collision_Shape_View, type_id: int,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings,
) -> (Distance_Query_Result, Physics_Status)
{
	local: util.Vector3 = rigid_pose_transform_by_inverse(point, input_view.pose);
	nearest: util.Vector3;
	primitive: Reference_State;
	nearest, primitive = distance_query_point_primitive(local, input_view.shape, type_id);
	if primitive == .Present
	{
		delta: util.Vector3 = util.vector3_subtract(local, nearest);
		distance: f32 = f32(math.sqrt(distance_query_dot(delta, delta)));
		if !(distance <= math.F32_MAX)
		{
			return {}, .No_Convergence;
		}
		if distance <= settings.absolute_tolerance
		{
			return {state=.Intersecting, point_a=point, point_b=point}, .Ok;
		}
		nearest_world: util.Vector3 = #force_inline util.quaternion_transform(nearest, input_view.pose.orientation);
		return {state=.Separated, point_a=point, point_b=util.vector3_add(nearest_world, input_view.pose.position),
			normal=#force_inline util.quaternion_transform(util.vector3_scale(delta, 1/distance), input_view.pose.orientation), distance=distance}, .Ok;
	}
	return distance_query_point_support(point, input_view^, shapes, settings);
}

distance_query_point_support :: #force_no_inline proc "contextless" (
	point: util.Vector3, input_view: Collision_Shape_View,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings,
) -> (Distance_Query_Result, Physics_Status)
{
	view: Collision_Shape_View = input_view;
	status: Physics_Status;
	view.pose.position = util.vector3_subtract(view.pose.position, point);
	result: Distance_Query_Result;
	simplex: Distance_Query_Simplex;
	if view.batch.dispatch == .Contextual
	{
		result, status = distance_query_gjk(Distance_Query_Point{}, collision_contextual_shape_view(view), shapes, settings, &simplex);
	}
	else
	{
		result, status = distance_query_gjk(Distance_Query_Point{}, view, shapes, settings, &simplex);
	}
	if status == .Ok
	{
		result.point_a = point;
		result.point_b = util.vector3_add(result.point_b, point);
		if result.state == .Intersecting
		{
			result.point_b = point;
		}
	}
	return result, status;
}

distance_query_kernel :: proc "contextless" (
	a: $A, b: $B, shapes: ^Shape_Registry, settings: Distance_Query_Settings,
	mode: Distance_Query_Mode, scratch: Distance_Query_Scratch,
) -> (Distance_Query_Result, Physics_Status)
{
	simplex: Distance_Query_Simplex;
	result: Distance_Query_Result;
	status: Physics_Status;
	result, status = distance_query_gjk(a, b, shapes, settings, &simplex);
	if status != .Ok || mode == .Distance || result.state == .Separated
	{
		return result, status;
	}
	return distance_query_epa(a, b, shapes, settings, &simplex, scratch);
}

// face orientation uses a fixed point inside the initial tetrahedron. EPA also
// expands an initially non-enclosing hull before accepting an origin boundary
distance_query_face :: proc "contextless" (vertices: []Distance_Query_Vertex, indices: [3]i32, interior: util.Vector3) -> (Distance_Query_Face, Physics_Status)
{
	a: util.Vector3 = vertices[indices[0]].point;
	ab: util.Vector3 = util.vector3_subtract(vertices[indices[1]].point, a);
	ac: util.Vector3 = util.vector3_subtract(vertices[indices[2]].point, a);
	x: f64 = f64(ab.y)*f64(ac.z)-f64(ab.z)*f64(ac.y);
	y: f64 = f64(ab.z)*f64(ac.x)-f64(ab.x)*f64(ac.z);
	z: f64 = f64(ab.x)*f64(ac.y)-f64(ab.y)*f64(ac.x);
	length: f64 = math.sqrt(x*x+y*y+z*z);
	if !(length > 0 && length <= math.F64_MAX)
	{
		return {}, .No_Convergence;
	}
	normal: util.Vector3 = {f32(x/length), f32(y/length), f32(z/length)};
	face: Distance_Query_Face = {indices=indices, normal=normal, distance=f32(distance_query_dot(normal, a))};
	if distance_query_dot(normal, util.vector3_subtract(interior, a)) > 0
	{
		face.indices[1], face.indices[2] = face.indices[2], face.indices[1];
		face.normal = util.vector3_negate(normal);
		face.distance = -face.distance;
	}
	return face, .Ok;
}

distance_query_edge_add :: proc "contextless" (edges: []Distance_Query_Edge, count: ^int, a, b: i32) -> Physics_Status
{
	for i in 0..<count^
	{
		if edges[i].a == b && edges[i].b == a
		{
			count^ -= 1;
			edges[i] = edges[count^];
			return .Ok;
		}
	}
	if count^ == len(edges)
	{
		return .Capacity_Missing;
	}
	edges[count^] = {a, b};
	count^ += 1;
	return .Ok;
}

distance_query_seed :: proc "contextless" (a: $A, b: $B, shapes: ^Shape_Registry, settings: Distance_Query_Settings, vertices: []Distance_Query_Vertex, volume: ^Reference_State) -> Physics_Status
{
	volume^ = .Missing;
	status: Physics_Status;
	vertices[0], status = distance_query_support(a, b, {1, 0, 0}, shapes);
	if status != .Ok
	{
		return status;
	}
	vertices[1], status = distance_query_support(a, b, {-1, 0, 0}, shapes);
	if status != .Ok
	{
		return status;
	}
	edge: util.Vector3 = util.vector3_subtract(vertices[1].point, vertices[0].point);
	tolerance_squared: f64 = f64(settings.absolute_tolerance)*f64(settings.absolute_tolerance);
	if distance_query_dot(edge, edge) <= tolerance_squared
	{
		for axis in ([2]util.Vector3{{0, 1, 0}, {0, 0, 1}})
		{
			vertices[0], status = distance_query_support(a, b, axis, shapes);
			if status != .Ok
			{
				return status;
			}
			vertices[1], status = distance_query_support(a, b, util.vector3_negate(axis), shapes);
			if status != .Ok
			{
				return status;
			}
			edge = util.vector3_subtract(vertices[1].point, vertices[0].point);
			if distance_query_dot(edge, edge) > tolerance_squared
			{
				break;
			}
		}
		if distance_query_dot(edge, edge) <= tolerance_squared
		{
			return .Ok;
		}
	}
	edge = util.vector3_scale(edge, f32(1/math.sqrt(distance_query_dot(edge, edge))));
	axis: util.Vector3 = {1, 0, 0};
	if abs(edge.y) < abs(edge.x)
	{
		axis = {0, 1, 0};
	}
	if abs(edge.z) < min(abs(edge.x), abs(edge.y))
	{
		axis = {0, 0, 1};
	}
	u: util.Vector3 = util.vector3_normalize(util.vector3_cross(edge, axis));
	v: util.Vector3 = util.vector3_cross(edge, u);
	best_area: f64 = -1;
	for direction in ([4]util.Vector3{u, util.vector3_negate(u), v, util.vector3_negate(v)})
	{
		vertex: Distance_Query_Vertex;
		vertex, status = distance_query_support(a, b, direction, shapes);
		if status != .Ok
		{
			return status;
		}
		perpendicular: util.Vector3 = util.vector3_cross(edge, util.vector3_subtract(vertex.point, vertices[0].point));
		area: f64 = distance_query_dot(perpendicular, perpendicular);
		if area > best_area
		{
			best_area = area;
			vertices[2] = vertex;
		}
	}
	if best_area <= tolerance_squared
	{
		return .Ok;
	}
	normal: util.Vector3 = util.vector3_normalize(util.vector3_cross(edge, util.vector3_subtract(vertices[2].point, vertices[0].point)));
	best_height: f64 = -1;
	for direction in ([2]util.Vector3{normal, util.vector3_negate(normal)})
	{
		vertex: Distance_Query_Vertex;
		vertex, status = distance_query_support(a, b, direction, shapes);
		if status != .Ok
		{
			return status;
		}
		height: f64 = abs(distance_query_dot(normal, util.vector3_subtract(vertex.point, vertices[0].point)));
		if height > best_height
		{
			best_height = height;
			vertices[3] = vertex;
		}
	}
	if best_height <= f64(settings.absolute_tolerance)
	{
		return .Ok;
	}
	volume^ = .Present;
	return .Ok;
}

distance_query_epa :: proc "contextless" (
	a: $A, b: $B, shapes: ^Shape_Registry, settings: Distance_Query_Settings,
	simplex: ^Distance_Query_Simplex, scratch: Distance_Query_Scratch,
) -> (Distance_Query_Result, Physics_Status)
{
	if len(scratch.vertices)<4 || len(scratch.faces)<4 || len(scratch.edges)<3
	{
		return {}, .Capacity_Missing;
	}
	volume: Reference_State;
	status: Physics_Status = distance_query_seed(a, b, shapes, settings, scratch.vertices, &volume);
	if status != .Ok
	{
		return {}, status;
	}
	if volume == .Missing
	{
		point_a, point_b: util.Vector3;
		point_a, point_b = distance_query_witnesses(simplex);
		return {state=.Touching, point_a=point_a, point_b=point_b}, .Ok;
	}
	interior: util.Vector3;
	for i in 0..<4
	{
		interior = util.vector3_add(interior, util.vector3_scale(scratch.vertices[i].point, 0.25));
	}
	for indices, i in ([4][3]i32{{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}})
	{
		scratch.faces[i], status = distance_query_face(scratch.vertices, indices, interior);
		if status != .Ok
		{
			return {}, status;
		}
	}
	vertex_count: int = 4;
	face_count: int = 4;
	for iteration in 0..<settings.maximum_iterations
	{
		nearest: int;
		for i in 1..<face_count
		{
			if scratch.faces[i].distance < scratch.faces[nearest].distance
			{
				nearest = i;
			}
		}
		face: Distance_Query_Face = scratch.faces[nearest];
		next: Distance_Query_Vertex;
		next, status = distance_query_support(a, b, face.normal, shapes);
		if status != .Ok
		{
			return {}, status;
		}
		upper: f32 = f32(distance_query_dot(face.normal, next.point));
		tolerance: f32 = max(settings.absolute_tolerance, settings.relative_tolerance*abs(upper));
		if upper-face.distance <= tolerance
		{
			if upper < -settings.absolute_tolerance
			{
				return {}, .No_Convergence;
			}
			witness: Distance_Query_Simplex = {count=3};
			for index, i in face.indices
			{
				witness.vertices[i] = scratch.vertices[index];
			}
			_ = distance_query_reduce(&witness);
			point_a, point_b: util.Vector3;
			point_a, point_b = distance_query_witnesses(&witness);
			if upper <= settings.absolute_tolerance
			{
				return {state=.Touching, point_a=point_a, point_b=point_b, iterations=iteration+1}, .Ok;
			}
			// the support plane at upper, not the inner face alone, certifies
			// separation after translating A. their gap bounds the MTD error
			return {state=.Penetrating, point_a=point_a, point_b=point_b, normal=util.vector3_negate(face.normal), depth=upper, iterations=iteration+1}, .Ok;
		}
		if vertex_count == len(scratch.vertices)
		{
			return {}, .Capacity_Missing;
		}
		for i in 0..<vertex_count
		{
			if next.point == scratch.vertices[i].point
			{
				return {}, .No_Convergence;
			}
		}
		edge_count, retained: int;
		for i in 0..<face_count
		{
			candidate: Distance_Query_Face = scratch.faces[i];
			if distance_query_dot(candidate.normal, util.vector3_subtract(next.point, scratch.vertices[candidate.indices[0]].point)) > 0
			{
				for edge in 0..<3
				{
					status = distance_query_edge_add(scratch.edges, &edge_count, candidate.indices[edge], candidate.indices[(edge+1)%3]);
					if status != .Ok
					{
						return {}, status;
					}
				}
			}
			else
			{
				scratch.faces[retained] = candidate;
				retained += 1;
			}
		}
		if edge_count == 0
		{
			return {}, .No_Convergence;
		}
		if retained+edge_count > len(scratch.faces)
		{
			return {}, .Capacity_Missing;
		}
		scratch.vertices[vertex_count] = next;
		for edge in scratch.edges[:edge_count]
		{
			scratch.faces[retained], status = distance_query_face(scratch.vertices, {edge.a, edge.b, i32(vertex_count)}, interior);
			if status != .Ok
			{
				return {}, status;
			}
			retained += 1;
		}
		vertex_count += 1;
		face_count = retained;
	}
	return {}, .No_Convergence;
}

// form the plane in f64 before normalization. reconstructing an interior point
// from huge triangle vertices can lose a small local distance to cancellation
distance_query_triangle_plane :: proc "contextless" (triangle: Triangle) -> ([3]f64, f64)
{
	ab: [3]f64 = {f64(triangle.b.x)-f64(triangle.a.x), f64(triangle.b.y)-f64(triangle.a.y), f64(triangle.b.z)-f64(triangle.a.z)};
	ac: [3]f64 = {f64(triangle.c.x)-f64(triangle.a.x), f64(triangle.c.y)-f64(triangle.a.y), f64(triangle.c.z)-f64(triangle.a.z)};
	normal: [3]f64 = {ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]};
	return normal, normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];
}

// analytic closed-solid projections. Triangle projection is two-sided surface
// distance. hulls and user convex types retain the admitted GJK support route
distance_query_point_primitive :: proc "contextless" (
	point: util.Vector3, shape: rawptr, type_id: int,
) -> (util.Vector3, Reference_State)
{
	switch type_id
	{
		case SPHERE_TYPE_ID, CAPSULE_TYPE_ID:
		center: util.Vector3;
		radius: f32;
		if type_id == SPHERE_TYPE_ID
		{
			radius = (^Sphere)(shape).radius;
		}
		else
		{
			capsule: ^Capsule = (^Capsule)(shape);
			radius = capsule.radius;
			center.y = clamp(point.y, -capsule.half_length, capsule.half_length);
		}
		delta: util.Vector3 = util.vector3_subtract(point, center);
		length: f32 = f32(math.sqrt(distance_query_dot(delta, delta)));
		if length <= radius
		{
			return point, .Present;
		}
		return util.vector3_add(center, util.vector3_scale(delta, radius/length)), .Present;
		case BOX_TYPE_ID:
		box: ^Box = (^Box)(shape);
		return {clamp(point.x, -box.half_width, box.half_width),
			clamp(point.y, -box.half_height, box.half_height),
			clamp(point.z, -box.half_length, box.half_length)}, .Present;
		case CYLINDER_TYPE_ID:
		cylinder: ^Cylinder = (^Cylinder)(shape);
		nearest: util.Vector3 = {point.x, clamp(point.y, -cylinder.half_length, cylinder.half_length), point.z};
		radial: f32 = f32(math.sqrt(f64(point.x)*f64(point.x)+f64(point.z)*f64(point.z)));
		if radial > cylinder.radius
		{
			scale: f32 = cylinder.radius/radial;
			nearest.x *= scale;
			nearest.z *= scale;
		}
		return nearest, .Present;
		case TRIANGLE_TYPE_ID:
		triangle: ^Triangle = (^Triangle)(shape);
		normal: [3]f64;
		normal_squared: f64;
		normal, normal_squared = distance_query_triangle_plane(triangle^);
		if normal_squared > 0 && normal_squared <= math.F64_MAX
		{
			offset: f64 = ((f64(point.x)-f64(triangle.a.x))*normal[0]+(f64(point.y)-f64(triangle.a.y))*normal[1]+(f64(point.z)-f64(triangle.a.z))*normal[2])/normal_squared;
			projected: [3]f64 = {f64(point.x)-normal[0]*offset, f64(point.y)-normal[1]*offset, f64(point.z)-normal[2]*offset};
			vertices: [3]util.Vector3 = {triangle.a, triangle.b, triangle.c};
			inside: Reference_State = .Present;
			for vertex, i in vertices
			{
				next: util.Vector3 = vertices[(i+1)%3];
				edge: [3]f64 = {f64(next.x)-f64(vertex.x), f64(next.y)-f64(vertex.y), f64(next.z)-f64(vertex.z)};
				to_point: [3]f64 = {projected[0]-f64(vertex.x), projected[1]-f64(vertex.y), projected[2]-f64(vertex.z)};
				side: f64 = (edge[1]*to_point[2]-edge[2]*to_point[1])*normal[0]+(edge[2]*to_point[0]-edge[0]*to_point[2])*normal[1]+(edge[0]*to_point[1]-edge[1]*to_point[0])*normal[2];
				if !(side >= 0)
				{
					inside = .Missing;
					break;
				}
			}
			if inside == .Present
			{
				return {f32(projected[0]), f32(projected[1]), f32(projected[2])}, .Present;
			}
		}
		// reuse the bounded simplex reducer: double-precision predicates
		// handle very thin valid triangles without an arbitrary area epsilon
		simplex: Distance_Query_Simplex = {count=3};
		for vertex, i in ([3]util.Vector3{triangle.a, triangle.b, triangle.c})
		{
			simplex.vertices[i] = {point=util.vector3_subtract(vertex, point), point_b=vertex};
		}
		_ = distance_query_reduce(&simplex);
		unused, nearest: util.Vector3;
		unused, nearest = distance_query_witnesses(&simplex);
		return nearest, .Present;
	}
	return {}, .Missing;
}

distance_query_spheres :: proc "contextless" (
	a, b: Sphere, position_a, position_b: util.Vector3,
	settings: Distance_Query_Settings, mode: Distance_Query_Mode,
) -> (Distance_Query_Result, Physics_Status)
{
	delta: util.Vector3 = util.vector3_subtract(position_a, position_b);
	length: f32 = f32(math.sqrt(distance_query_dot(delta, delta)));
	radii: f32 = a.radius+b.radius;
	if !(length <= math.F32_MAX && radii <= math.F32_MAX)
	{
		return {}, .No_Convergence;
	}
	normal: util.Vector3 = {1, 0, 0};
	if length > 0
	{
		normal = util.vector3_scale(delta, 1/length);
	}
	result: Distance_Query_Result = {
		point_a=util.vector3_subtract(position_a, util.vector3_scale(normal, a.radius)),
		point_b=util.vector3_add(position_b, util.vector3_scale(normal, b.radius)),
	};
	gap: f32 = length-radii;
	if gap > settings.absolute_tolerance
	{
		result.state = .Separated;
		result.normal = normal;
		result.distance = gap;
	}
	else if mode == .Distance
	{
		result.state = .Intersecting;
		// the axial intersection point belongs to both closed spheres
		result.point_a = util.vector3_add(position_b, util.vector3_scale(normal, min(b.radius, length+a.radius)));
		result.point_b = result.point_a;
	}
	else if gap >= -settings.absolute_tolerance
	{
		result.state = .Touching;
	}
	else
	{
		result.state = .Penetrating;
		result.normal = normal;
		result.depth = -gap;
	}
	return result, .Ok;
}

// the sphere offset of a triangle has an analytic distance/translation. this
// also avoids iterative polytope seeding at exact sphere/triangle tangency.
// Mesh correction applies the registered sidedness gate before entering here
distance_query_sphere_triangle :: proc "contextless" (
	sphere_view, triangle_view: Collision_Shape_View,
	settings: Distance_Query_Settings, mode: Distance_Query_Mode,
) -> (Distance_Query_Result, Physics_Status)
{
	center: util.Vector3 = rigid_pose_transform_by_inverse(sphere_view.pose.position, triangle_view.pose);
	closest: util.Vector3;
	found: Reference_State;
	closest, found = distance_query_point_primitive(center, triangle_view.shape, TRIANGLE_TYPE_ID);
	delta: util.Vector3 = util.vector3_subtract(center, closest);
	length: f32 = f32(math.sqrt(distance_query_dot(delta, delta)));
	if !(length <= math.F32_MAX)
	{
		return {}, .No_Convergence;
	}
	normal: util.Vector3;
	if length > 0
	{
		normal = util.vector3_scale(delta, 1/length);
	}
	else
	{
		triangle: ^Triangle = (^Triangle)(triangle_view.shape);
		plane: [3]f64;
		normal_squared: f64;
		plane, normal_squared = distance_query_triangle_plane(triangle^);
		normal_length: f64 = math.sqrt(normal_squared);
		if !(normal_length > 0 && normal_length <= math.F64_MAX)
		{
			return {}, .No_Convergence;
		}
		normal = {f32(plane[0]/normal_length), f32(plane[1]/normal_length), f32(plane[2]/normal_length)};
	}
	normal = util.quaternion_transform(normal, triangle_view.pose.orientation);
	radius: f32 = (^Sphere)(sphere_view.shape).radius;
	gap: f32 = length-radius;
	result: Distance_Query_Result = {
		point_a=util.vector3_subtract(sphere_view.pose.position, util.vector3_scale(normal, radius)),
		point_b=rigid_pose_transform(closest, triangle_view.pose),
	};
	if gap > settings.absolute_tolerance
	{
		result.state = .Separated;
		result.distance = gap;
		result.normal = normal;
	}
	else if mode == .Distance
	{
		result.state = .Intersecting;
		result.point_a = result.point_b;
	}
	else if gap >= -settings.absolute_tolerance
	{
		result.state = .Touching;
	}
	else
	{
		result.state = .Penetrating;
		result.normal = normal;
		result.depth = -gap;
	}
	return result, .Ok;
}

// lockstep query-only descriptions, not additions to old heterogeneous results.
// child IDs are -1 for a convex root and the stored child index for a composite
Distance_Query_Pair_Result :: struct
{
	geometry: Distance_Query_Result,
	child_a, child_b: i32,
}

Distance_Query_Correction :: struct
{
	state: Distance_Query_State,
	translation: util.Vector3,
	iterations: i32,
}

Distance_Query_Root :: struct
{
	view: Collision_Shape_View,
	type_id, count: int,
	tree: ^Tree,
}

// parent links already stored by Tree provide bounded-memory traversal.
// no recursive call stack, new tree, allocation or persistent query cache
Distance_Query_Cursor :: struct
{
	node, side, linear: int,
}

distance_query_root :: proc "contextless" (
	shape: rawptr, type_id: int, pose: Rigid_Pose, shapes: ^Shape_Registry,
) -> (Distance_Query_Root, Physics_Status)
{
	view: Collision_Shape_View;
	status: Physics_Status;
	view, status = collision_shape_view(shape, type_id, pose, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	if view.batch.state != .Registered
	{
		return {}, .Not_Found;
	}
	if view.batch.metadata.batch_type == .Convex
	{
		view, status = distance_query_view(shape, type_id, pose, shapes);
		return {view=view, type_id=type_id, count=1}, status;
	}
	q: util.Quaternion = pose.orientation;
	if !(abs(pose.position.x) <= math.F32_MAX && abs(pose.position.y) <= math.F32_MAX && abs(pose.position.z) <= math.F32_MAX) ||
	!(abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) <= 1e-4)
	{
		return {}, .Invalid_Argument;
	}
	count: int;
	count, status = collision_compound_child_count(shape, type_id);
	if status != .Ok
	{
		return {}, status;
	}
	if count <= 0
	{
		return {}, .Invalid_Description;
	}
	tree: ^Tree;
	if type_id == BIG_COMPOUND_TYPE_ID || type_id == MESH_TYPE_ID
	{
		tree, status = collision_tree_for_shape(shape, type_id);
		if status != .Ok
		{
			return {}, status;
		}
		if tree.state != .Ready || tree.leaf_count != count
		{
			return {}, .Invalid_Description;
		}
	}
	return {view=view, type_id=type_id, count=count, tree=tree}, .Ok;
}

distance_query_leaf :: proc "contextless" (
	root: Distance_Query_Root, index: int, triangle: ^Triangle, shapes: ^Shape_Registry,
) -> (Collision_Shape_View, int, Physics_Status)
{
	if root.view.batch.metadata.batch_type == .Convex
	{
		return root.view, root.type_id, .Ok;
	}
	if root.type_id == MESH_TYPE_ID
	{
		pose: Rigid_Pose;
		status: Physics_Status;
		triangle^, pose, status = collision_mesh_child((^Mesh)(root.view.shape), index, root.view.pose);
		if status != .Ok
		{
			return {}, 0, status;
		}
		return {shape=triangle, batch=&shapes.batches[TRIANGLE_TYPE_ID], pose=pose}, TRIANGLE_TYPE_ID, .Ok;
	}
	// declaration initialization avoids Odin's alias-safe multi-assignment copy
	child, status := collision_compound_child(root.view.shape, root.type_id, index, root.view.pose, shapes);
	if status != .Ok
	{
		return {}, 0, status;
	}
	view: Collision_Shape_View;
	view, status = distance_query_view(child.shape, child.type_id, child.pose, shapes);
	return view, child.type_id, status;
}

distance_query_child_id :: proc "contextless" (root: Distance_Query_Root, index: int) -> i32
{
	if root.view.batch.metadata.batch_type == .Convex
	{
		return -1;
	}
	return i32(index);
}

distance_query_bounds_distance_squared :: proc "contextless" (a, b: util.Bounding_Box) -> f64
{
	x: f64 = max(f64(0), max(f64(a.min.x)-f64(b.max.x), f64(b.min.x)-f64(a.max.x)));
	y: f64 = max(f64(0), max(f64(a.min.y)-f64(b.max.y), f64(b.min.y)-f64(a.max.y)));
	z: f64 = max(f64(0), max(f64(a.min.z)-f64(b.max.z), f64(b.min.z)-f64(a.max.z)));
	return x*x+y*y+z*z;
}

distance_query_relative_bounds :: proc "contextless" (
	view: Collision_Shape_View, frame: Rigid_Pose, shapes: ^Shape_Registry,
) -> (util.Bounding_Box, Physics_Status)
{
	pose: Rigid_Pose = {position=rigid_pose_transform_by_inverse(view.pose.position, frame),
		orientation=util.quaternion_concatenate(view.pose.orientation, util.quaternion_conjugate(frame.orientation))};
	bounds: Shape_Bounds;
	status: Physics_Status;
	bounds, status = shape_batch_compute_bounds(view.batch, view.shape, pose.orientation, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	return {min=util.vector3_add(bounds.min, pose.position), max=util.vector3_add(bounds.max, pose.position)}, .Ok;
}

distance_query_cursor_advance :: proc "contextless" (tree: ^Tree, cursor: ^Distance_Query_Cursor)
{
	for cursor.node >= 0
	{
		if cursor.side == 0 && (cursor.node != 0 || tree.leaf_count > 1)
		{
			cursor.side = 1;
			return;
		}
		meta: Tree_Metanode = tree.metanodes.memory[cursor.node];
		cursor.node = int(meta.parent);
		cursor.side = int(meta.index_in_parent);
	}
}

// query_bounds is in the composite's local frame. the dynamic distance limit
// preserves equal-distance candidates for deterministic stored-child ties
distance_query_next_leaf :: proc "contextless" (
	root: Distance_Query_Root, cursor: ^Distance_Query_Cursor, query_bounds: util.Bounding_Box,
	limit: f32,
) -> (int, Reference_State)
{
	if root.tree == nil
	{
		if cursor.linear == root.count
		{
			return 0, .Missing;
		}
		index: int = cursor.linear;
		cursor.linear += 1;
		return index, .Present;
	}
	for cursor.node >= 0
	{
		child: ^Tree_Node_Child = tree_child(&root.tree.nodes.memory[cursor.node], cursor.side);
		if distance_query_bounds_distance_squared(query_bounds, tree_child_bounds(child^)) > f64(limit)*f64(limit)
		{
			distance_query_cursor_advance(root.tree, cursor);
			continue;
		}
		if child.index >= 0
		{
			cursor.node = int(child.index);
			cursor.side = 0;
			continue;
		}
		index: int = tree_decode_leaf(child.index);
		distance_query_cursor_advance(root.tree, cursor);
		return index, .Present;
	}
	return 0, .Missing;
}

distance_query_limit :: proc "contextless" (best: Distance_Query_Pair_Result, settings: Distance_Query_Settings) -> f32
{
	if best.geometry.state == .Unresolved
	{
		return math.F32_MAX;
	}
	return best.geometry.distance+max(settings.absolute_tolerance, settings.relative_tolerance*best.geometry.distance);
}

distance_query_select :: proc "contextless" (
	best: ^Distance_Query_Pair_Result, result: Distance_Query_Result, child_a, child_b: i32,
)
{
	if best.geometry.state == .Unresolved || result.distance < best.geometry.distance ||
	(result.distance == best.geometry.distance &&
		(child_a < best.child_a || (child_a == best.child_a && child_b < best.child_b)))
	{
		best^ = {geometry=result, child_a=child_a, child_b=child_b};
	}
}

// distance between unions of supported convex leaves. Tree-backed B prunes
// branches against each A leaf. simple compounds scan their contiguous leaves.
// no compound-versus-compound MTD is implied by this operation
distance_query_shape_pair :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int, pose_a, pose_b: Rigid_Pose,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings,
) -> (Distance_Query_Pair_Result, Physics_Status)
{
	if distance_query_settings_validate(settings) != .Ok
	{
		return {}, .Invalid_Argument;
	}
	if shape_hierarchy_raw_depth(shapes, shape_a, type_a) > 0 || shape_hierarchy_raw_depth(shapes, shape_b, type_b) > 0
	{
		return query_hierarchy_distance(shape_a, shape_b, type_a, type_b, pose_a, pose_b, shapes, settings);
	}
	a, b: Distance_Query_Root;
	status: Physics_Status;
	a, status = distance_query_root(shape_a, type_a, pose_a, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	b, status = distance_query_root(shape_b, type_b, pose_b, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	if a.view.batch.metadata.batch_type == .Convex && b.view.batch.metadata.batch_type == .Convex
	{
		geometry: Distance_Query_Result;
		geometry, status = distance_query_convex_admitted(a.view, b.view, type_a, type_b, shapes, settings, .Distance, {});
		return {geometry=geometry, child_a=-1, child_b=-1}, status;
	}
	if a.tree != nil && b.view.batch.metadata.batch_type == .Convex
	{
		// the convex root is the outer leaf. traverse the existing tree on either
		// side, rather than visiting every A leaf just because it was first.
		// swapping a one-leaf side preserves original stored-child tie order
		result: Distance_Query_Pair_Result;
		result, status = distance_query_shape_pair(shape_b, shape_a, type_b, type_a, pose_b, pose_a, shapes, settings);
		if status != .Ok
		{
			return {}, status;
		}
		result.geometry.point_a, result.geometry.point_b = result.geometry.point_b, result.geometry.point_a;
		result.geometry.normal = util.vector3_negate(result.geometry.normal);
		result.child_a, result.child_b = result.child_b, result.child_a;
		return result, .Ok;
	}
	outer_limit_bounds: util.Bounding_Box;
	if a.count > 1 && b.count == 1
	{
		outer_limit_bounds, status = distance_query_relative_bounds(b.view, b.view.pose, shapes);
		if status != .Ok
		{
			return {}, status;
		}
	}
	best: Distance_Query_Pair_Result;
	for index_a in 0..<a.count
	{
		triangle_a: Triangle;
		leaf_a: Collision_Shape_View;
		leaf_type_a: int;
		leaf_a, leaf_type_a, status = distance_query_leaf(a, index_a, &triangle_a, shapes);
		if status != .Ok
		{
			return {}, status;
		}
		bounds: util.Bounding_Box;
		bounds, status = distance_query_relative_bounds(leaf_a, b.view.pose, shapes);
		if status != .Ok
		{
			return {}, status;
		}
		if a.count > 1 && b.count == 1
		{
			limit: f32 = distance_query_limit(best, settings);
			if distance_query_bounds_distance_squared(bounds, outer_limit_bounds) > f64(limit)*f64(limit)
			{
				continue;
			}
		}
		cursor: Distance_Query_Cursor;
		for
		{
			index_b: int;
			found: Reference_State;
			limit: f32 = distance_query_limit(best, settings);
			index_b, found = distance_query_next_leaf(b, &cursor, bounds, limit);
			if found == .Missing
			{
				break;
			}
			triangle_b: Triangle;
			leaf_b: Collision_Shape_View;
			leaf_type_b: int;
			leaf_b, leaf_type_b, status = distance_query_leaf(b, index_b, &triangle_b, shapes);
			if status != .Ok
			{
				return {}, status;
			}
			if b.tree == nil && b.count > 1
			{
				bounds_b: util.Bounding_Box;
				bounds_b, status = distance_query_relative_bounds(leaf_b, b.view.pose, shapes);
				if status != .Ok
				{
					return {}, status;
				}
				if distance_query_bounds_distance_squared(bounds, bounds_b) > f64(limit)*f64(limit)
				{
					continue;
				}
			}
			result: Distance_Query_Result;
			result, status = distance_query_convex_admitted(leaf_a, leaf_b, leaf_type_a, leaf_type_b, shapes, settings, .Distance, {});
			if status != .Ok
			{
				return {}, status;
			}
			distance_query_select(&best, result, distance_query_child_id(a, index_a), distance_query_child_id(b, index_b));
		}
	}
	if best.geometry.state == .Unresolved
	{
		return {}, .No_Convergence;
	}
	return best, .Ok;
}

distance_query_shape_point :: proc "contextless" (
	point: util.Vector3, shape: rawptr, type_id: int, pose: Rigid_Pose,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings,
) -> (Distance_Query_Pair_Result, Physics_Status)
{
	if distance_query_settings_validate(settings) != .Ok ||
	!(abs(point.x) <= math.F32_MAX && abs(point.y) <= math.F32_MAX && abs(point.z) <= math.F32_MAX)
	{
		return {}, .Invalid_Argument;
	}
	if shape_hierarchy_raw_depth(shapes, shape, type_id) > 0
	{
		return query_hierarchy_point(point, shape, type_id, pose, shapes, settings);
	}
	root: Distance_Query_Root;
	status: Physics_Status;
	root, status = distance_query_root(shape, type_id, pose, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	if root.view.batch.metadata.batch_type == .Convex
	{
		result: Distance_Query_Result;
		result, status = #force_no_inline distance_query_point_admitted(point, &root.view, root.type_id, shapes, settings);
		if status != .Ok
		{
			return {}, status;
		}
		if result.state == .Unresolved
		{
			return {}, .No_Convergence;
		}
		return {geometry=result, child_a=-1, child_b=-1}, .Ok;
	}
	local: util.Vector3 = rigid_pose_transform_by_inverse(point, pose);
	bounds: util.Bounding_Box = {min=local, max=local};
	cursor: Distance_Query_Cursor;
	best: Distance_Query_Pair_Result;
	for
	{
		index: int;
		found: Reference_State;
		limit: f32 = distance_query_limit(best, settings);
		index, found = distance_query_next_leaf(root, &cursor, bounds, limit);
		if found == .Missing
		{
			break;
		}
		triangle: Triangle;
		leaf: Collision_Shape_View;
		leaf_type: int;
		leaf, leaf_type, status = #force_inline distance_query_leaf(root, index, &triangle, shapes);
		if status != .Ok
		{
			return {}, status;
		}
		if root.tree == nil && root.count > 1
		{
			leaf_bounds: util.Bounding_Box;
			leaf_bounds, status = distance_query_relative_bounds(leaf, pose, shapes);
			if status != .Ok
			{
				return {}, status;
			}
			if (#force_inline distance_query_bounds_distance_squared(bounds, leaf_bounds)) > f64(limit)*f64(limit)
			{
				continue;
			}
		}
		result: Distance_Query_Result;
		result, status = #force_no_inline distance_query_point_admitted(point, &leaf, leaf_type, shapes, settings);
		if status != .Ok
		{
			return {}, status;
		}
		distance_query_select(&best, result, -1, distance_query_child_id(root, index));
	}
	if best.geometry.state == .Unresolved
	{
		return {}, .No_Convergence;
	}
	return best, .Ok;
}

// a is convex. each accepted correction is followed by a complete candidate
// requery. success is issued only by a pass with no remaining penetrating leaf.
// Mesh eligibility uses the registered zero-margin collision task, preserving
// its sidedness. the returned translation is verified, not globally minimal
distance_query_depenetrate :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int, pose_a, pose_b: Rigid_Pose,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings, scratch: Distance_Query_Scratch,
	tasks: ^Collision_Task_Registry = nil,
) -> (Distance_Query_Correction, Physics_Status)
{
	if distance_query_settings_validate(settings) != .Ok || (type_b == MESH_TYPE_ID && tasks == nil)
	{
		return {}, .Invalid_Argument;
	}
	if shape_hierarchy_raw_depth(shapes, shape_b, type_b) > 0
	{
		return query_hierarchy_depenetrate(shape_a, shape_b, type_a, type_b, pose_a, pose_b, shapes, settings, scratch, tasks);
	}
	a: Collision_Shape_View;
	b: Distance_Query_Root;
	status: Physics_Status;
	a, status = distance_query_view(shape_a, type_a, pose_a, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	b, status = distance_query_root(shape_b, type_b, pose_b, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	translation: util.Vector3;
	for iteration in 0..=settings.maximum_iterations
	{
		bounds: util.Bounding_Box;
		bounds, status = distance_query_relative_bounds(a, pose_b, shapes);
		if status != .Ok
		{
			return {}, status;
		}
		cursor: Distance_Query_Cursor;
		deepest: Distance_Query_Result;
		deepest_child: int = b.count;
		final_state: Distance_Query_State = .Separated;
		for
		{
			index: int;
			found: Reference_State;
			index, found = distance_query_next_leaf(b, &cursor, bounds, settings.absolute_tolerance);
			if found == .Missing
			{
				break;
			}
			triangle: Triangle;
			leaf: Collision_Shape_View;
			leaf_type: int;
			leaf, leaf_type, status = distance_query_leaf(b, index, &triangle, shapes);
			if status != .Ok
			{
				return {}, status;
			}
			if b.type_id == MESH_TYPE_ID
			{
				manifold: Convex_Contact_Manifold;
				manifold, status = collision_task_registry_test_convex(tasks, type_a, leaf_type, a.shape, leaf.shape, a.pose, leaf.pose, 0, shapes);
				if status != .Ok
				{
					return {}, status;
				}
				accepted: Reference_State;
				for contact in manifold.contacts[:manifold.count]
				{
					if contact.depth >= 0
					{
						accepted = .Present;
						break;
					}
				}
				if accepted == .Missing
				{
					continue;
				}
			}
			result: Distance_Query_Result;
			result, status = distance_query_convex_admitted(a, leaf, type_a, leaf_type, shapes, settings, .Penetration, scratch);
			if status != .Ok
			{
				return {}, status;
			}
			if result.state == .Touching
			{
				final_state = .Touching;
			}
			if result.state == .Penetrating && (result.depth > deepest.depth || (result.depth == deepest.depth && index < deepest_child))
			{
				deepest = result;
				deepest_child = index;
			}
		}
		if deepest.state != .Penetrating
		{
			return {state=final_state, translation=translation, iterations=iteration}, .Ok;
		}
		if iteration == settings.maximum_iterations
		{
			return {}, .No_Convergence;
		}
		translation = util.vector3_add(translation, util.vector3_scale(deepest.normal, deepest.depth));
		position: util.Vector3 = util.vector3_add(pose_a.position, translation);
		if position == a.pose.position || !(abs(position.x) <= math.F32_MAX && abs(position.y) <= math.F32_MAX && abs(position.z) <= math.F32_MAX)
		{
			return {}, .No_Convergence;
		}
		a.pose.position = position;
	}
	return {}, .No_Convergence;
}
