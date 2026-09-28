// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

Convex_Hull_Wide :: struct
{
	hulls: [util.PRODUCTION_LANE_COUNT]^Convex_Hull,
}

Hull_Bounding_Plane_Bundle :: struct
{
	normal: util.Vector3_Wide,
	offset: util.F32x8,
}

Convex_Hull :: struct
{
	points:              util.Buffer(util.Vector3_Wide),
	bounding_planes:     util.Buffer(Hull_Bounding_Plane_Bundle),
	face_start_indices:  util.Buffer(i32),
	face_vertex_indices: util.Buffer(i32),
	point_count:         i32,
}

Compound_Child :: struct
{
	local_position:    util.Vector3,
	local_orientation: util.Quaternion,
	shape_index:       Typed_Index,
}

Compound :: struct
{
	children: util.Buffer(Compound_Child),
}

Big_Compound :: struct
{
	children:     util.Buffer(Compound_Child),
	child_bounds: util.Buffer(util.Bounding_Box),
	tree:         Tree,
}

Mesh :: struct
{
	triangles:      util.Buffer(Triangle),
	triangle_bounds: util.Buffer(util.Bounding_Box),
	tree:           Tree,
	scale:          util.Vector3,
}

shape_copy_buffer :: proc (
	pool: ^util.Buffer_Pool, source: [^]$T, count: int,
) -> (util.Buffer(T), Physics_Status)
{
	if pool == nil || source == nil || count <= 0
	{
		return {}, .Invalid_Argument;
	}
	buffer, status := util.buffer_pool_take_at_least(pool, T, count);
	if status != .Ok
	{
		return {}, physics_memory_status(status);
	}
	copy_status := util.buffer_copy(
		util.Buffer_View(T){memory=source, length=count, capacity=count}, 0, util.buffer_view(buffer), 0, count,
	);
	if copy_status != .Ok
	{
		physics_return_buffer(pool, &buffer);
		return {}, physics_memory_status(copy_status);
	}
	buffer.length = i32(count);
	return buffer, .Ok;
}

convex_hull_get_point :: #force_inline proc "contextless" (hull: ^Convex_Hull, point_index: int) -> util.Vector3
{
	bundle_index := point_index / util.PRODUCTION_LANE_COUNT;
	lane := point_index % util.PRODUCTION_LANE_COUNT;
	return util.vector3_wide_read_slot(hull.points.memory[bundle_index], lane);
}

convex_hull_get_face_plane :: #force_inline proc "contextless" (
	hull: ^Convex_Hull, face_index: int,
) -> (normal: util.Vector3, offset: f32)
{
	bundle_index := face_index / util.PRODUCTION_LANE_COUNT;
	lane := face_index % util.PRODUCTION_LANE_COUNT;
	bundle := hull.bounding_planes.memory[bundle_index];
	return util.vector3_wide_read_slot(bundle.normal, lane), simd.extract(bundle.offset, lane);
}

convex_hull_create :: proc (
	hull: ^Convex_Hull,
	points: [^]util.Vector3, point_count: int,
	face_start_indices: [^]i32, face_count: int,
	face_vertex_indices: [^]i32, face_vertex_count: int,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if hull == nil || pool == nil || points == nil || point_count < 4 ||
		face_start_indices == nil || face_count < 4 || face_vertex_indices == nil || face_vertex_count < 12
	{
		return .Invalid_Description;
	}
	previous_start := -1;
	for face_index in 0 ..< face_count
	{
		start := int(face_start_indices[face_index]);
		end := face_vertex_count;
		if face_index + 1 < face_count
		{
			end = int(face_start_indices[face_index + 1]);
		}
		if start <= previous_start || start < 0 || end - start < 3 || end > face_vertex_count
		{
			return .Invalid_Description;
		}
		for index in start ..< end
		{
			if face_vertex_indices[index] < 0 || int(face_vertex_indices[index]) >= point_count
			{
				return .Invalid_Description;
			}
		}
		previous_start = start;
	}
	point_bundle_count := (point_count + util.PRODUCTION_LANE_COUNT - 1) / util.PRODUCTION_LANE_COUNT;
	point_buffer, point_status := util.buffer_pool_take(pool, util.Vector3_Wide, point_bundle_count);
	if point_status != .Ok
	{
		return physics_memory_status(point_status);
	}
	_ = util.buffer_clear(point_buffer, 0, point_bundle_count);
	for point_index in 0 ..< point_bundle_count * util.PRODUCTION_LANE_COUNT
	{
		point := points[0];
		if point_index < point_count
		{
			point = points[point_index];
		}
		util.vector3_wide_write_slot(
			&point_buffer.memory[point_index / util.PRODUCTION_LANE_COUNT],
			point_index % util.PRODUCTION_LANE_COUNT, point,
		);
	}
	plane_bundle_count := (face_count + util.PRODUCTION_LANE_COUNT - 1) / util.PRODUCTION_LANE_COUNT;
	bounding_planes, plane_status := util.buffer_pool_take(pool, Hull_Bounding_Plane_Bundle, plane_bundle_count);
	if plane_status != .Ok
	{
		physics_return_buffer(pool, &point_buffer);
		return physics_memory_status(plane_status);
	}
	_ = util.buffer_clear(bounding_planes, 0, plane_bundle_count);
	for face_index in 0 ..< plane_bundle_count * util.PRODUCTION_LANE_COUNT
	{
		normal: util.Vector3;
		offset := -f32(math.F32_MAX);
		if face_index < face_count
		{
			start := int(face_start_indices[face_index]);
			a := points[face_vertex_indices[start]];
			b := points[face_vertex_indices[start + 1]];
			c := points[face_vertex_indices[start + 2]];
			normal = util.vector3_cross(util.vector3_subtract(b, a), util.vector3_subtract(c, a));
			length_squared := util.vector3_length_squared(normal);
			if length_squared <= 1e-20
			{
				physics_return_buffer(pool, &bounding_planes);
				physics_return_buffer(pool, &point_buffer);
				return .Invalid_Description;
			}
			normal = util.vector3_scale(normal, 1 / math.sqrt(length_squared));
			offset = util.vector3_dot(normal, a);
		}
		bundle_index := face_index / util.PRODUCTION_LANE_COUNT;
		lane := face_index % util.PRODUCTION_LANE_COUNT;
		util.vector3_wide_write_slot(&bounding_planes.memory[bundle_index].normal, lane, normal);
		bounding_planes.memory[bundle_index].offset = simd.replace(
			bounding_planes.memory[bundle_index].offset, lane, offset,
		);
	}
	face_start_buffer, face_status := shape_copy_buffer(pool, face_start_indices, face_count);
	if face_status != .Ok
	{
		physics_return_buffer(pool, &bounding_planes);
		physics_return_buffer(pool, &point_buffer);
		return face_status;
	}
	face_vertex_buffer, vertex_status := shape_copy_buffer(pool, face_vertex_indices, face_vertex_count);
	if vertex_status != .Ok
	{
		physics_return_buffer(pool, &face_start_buffer);
		physics_return_buffer(pool, &bounding_planes);
		physics_return_buffer(pool, &point_buffer);
		return vertex_status;
	}
	hull^ = {
		points=point_buffer,
		bounding_planes=bounding_planes,
		face_start_indices=face_start_buffer,
		face_vertex_indices=face_vertex_buffer,
		point_count=i32(point_count),
	};
	return .Ok;
}

convex_hull_dispose :: proc (hull: ^Convex_Hull, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if hull == nil || pool == nil || hull.points.memory == nil
	{
		return .Disposed;
	}
	physics_return_buffer(pool, &hull.face_vertex_indices);
	physics_return_buffer(pool, &hull.face_start_indices);
	physics_return_buffer(pool, &hull.bounding_planes);
	physics_return_buffer(pool, &hull.points);
	hull^ = {};
	return .Ok;
}

convex_hull_validate :: proc "contextless" (hull: ^Convex_Hull) -> Physics_Status
{
	if hull == nil || hull.points.memory == nil || hull.point_count < 4 ||
		hull.bounding_planes.memory == nil || hull.bounding_planes.length <= 0 ||
		hull.face_start_indices.memory == nil || hull.face_start_indices.length < 4 ||
		hull.face_vertex_indices.memory == nil || hull.face_vertex_indices.length < 12
	{
		return .Invalid_Description;
	}
	return .Ok;
}

convex_hull_bounds :: proc "contextless" (hull: ^Convex_Hull, orientation: util.Quaternion) -> Shape_Bounds
{
	orientation_wide := util.quaternion_wide_broadcast(orientation);
	orientation_matrix := util.matrix3x3_wide_from_quaternion(orientation_wide);
	min_wide := util.vector3_wide_broadcast({f32(math.F32_MAX), f32(math.F32_MAX), f32(math.F32_MAX)});
	max_wide := util.vector3_wide_broadcast({-f32(math.F32_MAX), -f32(math.F32_MAX), -f32(math.F32_MAX)});
	maximum_radius_squared_wide := util.F32x8(0);
	minimum_radius_squared_wide := util.F32x8(f32(math.F32_MAX));
	for bundle_index in 0 ..< int(hull.points.length)
	{
		point := hull.points.memory[bundle_index];
		world := util.matrix3x3_wide_transform(point, orientation_matrix);
		min_wide = util.vector3_wide_min(min_wide, world);
		max_wide = util.vector3_wide_max(max_wide, world);
		radius_squared := util.vector3_wide_length_squared(point);
		maximum_radius_squared_wide = simd.max(maximum_radius_squared_wide, radius_squared);
		minimum_radius_squared_wide = simd.min(minimum_radius_squared_wide, radius_squared);
	}
	min_bound := util.vector3_wide_read_slot(min_wide, 0);
	max_bound := util.vector3_wide_read_slot(max_wide, 0);
	maximum_radius_squared := simd.extract(maximum_radius_squared_wide, 0);
	minimum_radius_squared := simd.extract(minimum_radius_squared_wide, 0);
	for lane in 1 ..< util.PRODUCTION_LANE_COUNT
	{
		min_bound = util.vector3_min(min_bound, util.vector3_wide_read_slot(min_wide, lane));
		max_bound = util.vector3_max(max_bound, util.vector3_wide_read_slot(max_wide, lane));
		maximum_radius_squared = max(maximum_radius_squared, simd.extract(maximum_radius_squared_wide, lane));
		minimum_radius_squared = min(minimum_radius_squared, simd.extract(minimum_radius_squared_wide, lane));
	}
	maximum_radius := math.sqrt(maximum_radius_squared);
	minimum_radius := math.sqrt(minimum_radius_squared);
	return {
		min=min_bound,
		max=max_bound,
		maximum_radius=maximum_radius,
		maximum_angular_expansion=maximum_radius - minimum_radius,
	};
}

convex_hull_wide_bounds :: proc "contextless" (
	shape: Convex_Hull_Wide, orientation: util.Quaternion_Wide,
) -> (min_bound, max_bound: util.Vector3_Wide, status: Physics_Status)
{
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if convex_hull_validate(shape.hulls[lane]) != .Ok
		{
			return {}, {}, .Invalid_Description;
		}
		bounds := convex_hull_bounds(shape.hulls[lane], util.quaternion_wide_read_slot(orientation, lane));
		util.vector3_wide_write_slot(&min_bound, lane, bounds.min);
		util.vector3_wide_write_slot(&max_bound, lane, bounds.max);
	}
	return min_bound, max_bound, .Ok;
}

convex_hull_support_trusted :: proc "contextless" (
	hull: ^Convex_Hull,
	direction: util.Vector3,
) -> util.Vector3
{
	direction_wide := util.vector3_wide_broadcast(direction);
	best_points := hull.points.memory[0];
	best_dots := util.vector3_wide_dot(best_points, direction_wide);
	for bundle_index in 1 ..< int(hull.points.length)
	{
		candidate := hull.points.memory[bundle_index];
		candidate_dots := util.vector3_wide_dot(candidate, direction_wide);
		use_candidate := transmute(util.I32x8)simd.lanes_gt(candidate_dots, best_dots);
		best_points = util.vector3_wide_select(use_candidate, candidate, best_points);
		best_dots = util.wide_select_f32(use_candidate, candidate_dots, best_dots);
	}
	best_lane := 0;
	best_dot := simd.extract(best_dots, 0);
	for lane in 1 ..< util.PRODUCTION_LANE_COUNT
	{
		candidate_dot := simd.extract(best_dots, lane);
		if candidate_dot > best_dot
		{
			best_dot = candidate_dot;
			best_lane = lane;
		}
	}
	return util.vector3_wide_read_slot(best_points, best_lane);
}

convex_hull_support :: proc "contextless" (
	hull: ^Convex_Hull,
	direction: util.Vector3,
) -> (util.Vector3, Physics_Status)
{
	if convex_hull_validate(hull) != .Ok || util.vector3_length_squared(direction) <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	return convex_hull_support_trusted(hull, direction), .Ok;
}

mesh_tetrahedron_volume :: proc "contextless" (a, b, c: util.Vector3) -> f32
{
	return (1.0 / 6.0) * util.vector3_dot(util.vector3_cross(b, a), c);
}

mesh_tetrahedron_inertia_tensor :: proc "contextless" (a, b, c: util.Vector3, mass: f32) -> util.Symmetric3x3
{
	diagonal_scaling := mass * (6.0 / 60.0);
	off_scaling := mass * (6.0 / 120.0);
	return {
		diagonal_scaling * (
			a.y*a.y + a.z*a.z + b.y*b.y + b.z*b.z + c.y*c.y + c.z*c.z +
			b.y*c.y + b.z*c.z + a.y*(b.y+c.y) + a.z*(b.z+c.z)),
		off_scaling * (
			-2*b.x*b.y - 2*c.x*c.y - b.y*c.x - b.x*c.y -
			a.y*(b.x+c.x) - a.x*(2*a.y+b.y+c.y)),
		diagonal_scaling * (
			a.x*a.x + a.z*a.z + b.x*b.x + b.z*b.z + c.x*c.x + c.z*c.z +
			b.x*c.x + b.z*c.z + a.x*(b.x+c.x) + a.z*(b.z+c.z)),
		off_scaling * (
			-2*b.x*b.z - 2*c.x*c.z - b.z*c.x - b.x*c.z -
			a.z*(b.x+c.x) - a.x*(2*a.z+b.z+c.z)),
		off_scaling * (
			-2*b.y*b.z - 2*c.y*c.z - b.z*c.y - b.y*c.z -
			a.z*(b.y+c.y) - a.y*(2*a.z+b.z+c.z)),
		diagonal_scaling * (
			a.x*a.x + a.y*a.y + b.x*b.x + b.y*b.y + c.x*c.x + c.y*c.y +
			b.x*c.x + b.y*c.y + a.x*(b.x+c.x) + a.y*(b.y+c.y)),
	};
}

convex_hull_inertia :: proc "contextless" (hull: ^Convex_Hull, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if convex_hull_validate(hull) != .Ok || mass <= 0
	{
		return {}, .Invalid_Argument;
	}
	volume: f32;
	summed: util.Symmetric3x3;
	for face_index in 0 ..< int(hull.face_start_indices.length)
	{
		start := int(hull.face_start_indices.memory[face_index]);
		end := int(hull.face_vertex_indices.length);
		if face_index + 1 < int(hull.face_start_indices.length)
		{
			end = int(hull.face_start_indices.memory[face_index + 1]);
		}
		a := convex_hull_get_point(hull, int(hull.face_vertex_indices.memory[start]));
		for face_vertex in start + 1 ..< end - 1
		{
			b := convex_hull_get_point(hull, int(hull.face_vertex_indices.memory[face_vertex]));
			c := convex_hull_get_point(hull, int(hull.face_vertex_indices.memory[face_vertex + 1]));
			tetrahedron_volume := mesh_tetrahedron_volume(a, b, c);
			volume += tetrahedron_volume;
			summed = util.symmetric3x3_add(summed, mesh_tetrahedron_inertia_tensor(a, b, c, tetrahedron_volume));
		}
	}
	if abs(volume) <= 1e-10
	{
		return {}, .Invalid_Description;
	}
	if volume < 0
	{
		volume = -volume;
		summed = util.symmetric3x3_scale(summed, -1);
	}
	tensor := util.symmetric3x3_scale(summed, mass / volume);
	return shape_inertia_from_tensor(tensor, mass);
}

compound_create :: proc (
	compound: ^Compound, children: [^]Compound_Child, child_count: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if compound == nil || children == nil || child_count <= 0 || pool == nil
	{
		return .Invalid_Description;
	}
	for index in 0 ..< child_count
	{
		if typed_index_state(children[index].shape_index) != .Present ||
			abs(util.quaternion_length_squared(children[index].local_orientation) - 1) > 1e-3
		{
			return .Invalid_Description;
		}
	}
	buffer, status := shape_copy_buffer(pool, children, child_count);
	if status != .Ok
	{
		return status;
	}
	compound^ = {children=buffer};
	return .Ok;
}

compound_dispose :: proc (compound: ^Compound, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if compound == nil || pool == nil || compound.children.memory == nil
	{
		return .Disposed;
	}
	physics_return_buffer(pool, &compound.children);
	compound^ = {};
	return .Ok;
}

compound_bounds :: proc "contextless" (
	compound: ^Compound, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	if compound == nil || compound.children.memory == nil || compound.children.length <= 0 || registry == nil
	{
		return {}, .Invalid_Description;
	}
	result := Shape_Bounds{
		min={f32(math.F32_MAX), f32(math.F32_MAX), f32(math.F32_MAX)},
		max={-f32(math.F32_MAX), -f32(math.F32_MAX), -f32(math.F32_MAX)},
	};
	for index in 0 ..< compound.children.length
	{
		child := &compound.children.memory[index];
		child_orientation := util.quaternion_concatenate(child.local_orientation, orientation);
		child_bounds, status := shape_registry_compute_bounds(registry, child.shape_index, child_orientation);
		if status != .Ok
		{
			return {}, status;
		}
		child_offset := util.quaternion_transform(child.local_position, orientation);
		child_bounds.min = util.vector3_add(child_bounds.min, child_offset);
		child_bounds.max = util.vector3_add(child_bounds.max, child_offset);
		result.min = util.vector3_min(result.min, child_bounds.min);
		result.max = util.vector3_max(result.max, child_bounds.max);
		child_radius := util.vector3_length(child.local_position) + child_bounds.maximum_radius;
		result.maximum_radius = max(result.maximum_radius, child_radius);
	}
	result.maximum_angular_expansion = result.maximum_radius;
	return result, .Ok;
}

compound_center_of_mass :: proc "contextless" (
	compound: ^Compound, child_masses: [^]f32, mass_count: int,
) -> (center: util.Vector3, inverse_mass: f32, status: Physics_Status)
{
	if compound == nil || compound.children.memory == nil || child_masses == nil ||
		mass_count != int(compound.children.length) || mass_count <= 0
	{
		return {}, 0, .Invalid_Argument;
	}
	mass_sum: f32;
	for index in 0 ..< mass_count
	{
		mass := child_masses[index];
		if mass <= 0
		{
			return {}, 0, .Invalid_Description;
		}
		center = util.vector3_add(center, util.vector3_scale(compound.children.memory[index].local_position, mass));
		mass_sum += mass;
	}
	if mass_sum <= 0
	{
		return {}, 0, .Invalid_Description;
	}
	inverse_mass = 1 / mass_sum;
	center = util.vector3_scale(center, inverse_mass);
	return center, inverse_mass, .Ok;
}

compound_inertia_weighted_at_center :: proc "contextless" (
	compound: ^Compound, registry: ^Shape_Registry, child_masses: [^]f32, mass_count: int,
	center: util.Vector3,
) -> (Body_Inertia, Physics_Status)
{
	_, inverse_mass, center_status := compound_center_of_mass(compound, child_masses, mass_count);
	if center_status != .Ok || registry == nil
	{
		return {}, .Invalid_Argument;
	}
	summed: util.Symmetric3x3;
	for index in 0 ..< mass_count
	{
		child := &compound.children.memory[index];
		child_mass := child_masses[index];
		child_inertia, status := shape_registry_compute_inertia(registry, child.shape_index, child_mass);
		if status != .Ok
		{
			return {}, status;
		}
		local_tensor := util.symmetric3x3_invert(child_inertia.inverse_inertia_tensor);
		basis := util.matrix3x3_from_quaternion(child.local_orientation);
		rotated := util.symmetric3x3_rotation_sandwich(basis, local_tensor);
		p := util.vector3_subtract(child.local_position, center);
		offset := util.Symmetric3x3{
			child_mass * (p.y*p.y + p.z*p.z),
			-child_mass * p.x*p.y,
			child_mass * (p.x*p.x + p.z*p.z),
			-child_mass * p.x*p.z,
			-child_mass * p.y*p.z,
			child_mass * (p.x*p.x + p.y*p.y),
		};
		summed = util.symmetric3x3_add(summed, util.symmetric3x3_add(rotated, offset));
	}
	tensor_inertia, tensor_status := shape_inertia_from_tensor(summed, 1 / inverse_mass);
	return tensor_inertia, tensor_status;
}

compound_inertia_weighted :: proc "contextless" (
	compound: ^Compound, registry: ^Shape_Registry, child_masses: [^]f32, mass_count: int,
) -> (Body_Inertia, Physics_Status)
{
	return compound_inertia_weighted_at_center(compound, registry, child_masses, mass_count, {});
}

compound_inertia_weighted_recenter :: proc "contextless" (
	compound: ^Compound, registry: ^Shape_Registry, child_masses: [^]f32, mass_count: int,
) -> (Body_Inertia, util.Vector3, Physics_Status)
{
	center, _, center_status := compound_center_of_mass(compound, child_masses, mass_count);
	if center_status != .Ok
	{
		return {}, {}, center_status;
	}
	inertia, inertia_status := compound_inertia_weighted_at_center(
		compound, registry, child_masses, mass_count, center,
	);
	if inertia_status != .Ok
	{
		return {}, {}, inertia_status;
	}
	for index in 0 ..< mass_count
	{
		compound.children.memory[index].local_position = util.vector3_subtract(
			compound.children.memory[index].local_position, center,
		);
	}
	return inertia, center, .Ok;
}

big_compound_inertia_weighted :: proc "contextless" (
	compound: ^Big_Compound, registry: ^Shape_Registry, child_masses: [^]f32, mass_count: int,
) -> (Body_Inertia, Physics_Status)
{
	if compound == nil
	{
		return {}, .Invalid_Argument;
	}
	proxy := Compound{children=compound.children};
	return compound_inertia_weighted(&proxy, registry, child_masses, mass_count);
}

big_compound_create :: proc (
	compound: ^Big_Compound, children: [^]Compound_Child, child_count: int,
	registry: ^Shape_Registry, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if compound == nil || registry == nil || pool == nil || children == nil || child_count <= 0
	{
		return .Invalid_Description;
	}
	child_buffer, child_status := shape_copy_buffer(pool, children, child_count);
	if child_status != .Ok
	{
		return child_status;
	}
	bounds_buffer, bounds_status := util.buffer_pool_take_at_least(pool, util.Bounding_Box, child_count);
	if bounds_status != .Ok
	{
		physics_return_buffer(pool, &child_buffer);
		return physics_memory_status(bounds_status);
	}
	bounds_buffer.length = i32(child_count);
	for index in 0 ..< child_count
	{
		child := &children[index];
		bounds, status := shape_registry_compute_bounds(registry, child.shape_index, child.local_orientation);
		if status != .Ok
		{
			physics_return_buffer(pool, &bounds_buffer);
			physics_return_buffer(pool, &child_buffer);
			return status;
		}
		bounds_buffer.memory[index] = {
			min=util.vector3_add(bounds.min, child.local_position),
			max=util.vector3_add(bounds.max, child.local_position),
		};
	}
	tree: Tree;
	tree_status := tree_initialize(&tree, child_count, pool);
	if tree_status != .Ok
	{
		physics_return_buffer(pool, &bounds_buffer);
		physics_return_buffer(pool, &child_buffer);
		return tree_status;
	}
	build_status := tree_build(&tree, bounds_buffer.memory, child_count);
	if build_status != .Ok
	{
		_ = tree_dispose(&tree);
		physics_return_buffer(pool, &bounds_buffer);
		physics_return_buffer(pool, &child_buffer);
		return build_status;
	}
	compound^ = {children=child_buffer, child_bounds=bounds_buffer, tree=tree};
	return .Ok;
}

big_compound_dispose :: proc (compound: ^Big_Compound, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if compound == nil || pool == nil || compound.children.memory == nil
	{
		return .Disposed;
	}
	_ = tree_dispose(&compound.tree);
	physics_return_buffer(pool, &compound.child_bounds);
	physics_return_buffer(pool, &compound.children);
	compound^ = {};
	return .Ok;
}

big_compound_bounds :: proc "contextless" (compound: ^Big_Compound, orientation: util.Quaternion) -> Shape_Bounds
{
	result := Shape_Bounds{
		min={f32(math.F32_MAX), f32(math.F32_MAX), f32(math.F32_MAX)},
		max={-f32(math.F32_MAX), -f32(math.F32_MAX), -f32(math.F32_MAX)},
	};
	for index in 0 ..< compound.child_bounds.length
	{
		bounds := compound.child_bounds.memory[index];
		corners := [8]util.Vector3{
			{bounds.min.x, bounds.min.y, bounds.min.z}, {bounds.max.x, bounds.min.y, bounds.min.z},
			{bounds.min.x, bounds.max.y, bounds.min.z}, {bounds.max.x, bounds.max.y, bounds.min.z},
			{bounds.min.x, bounds.min.y, bounds.max.z}, {bounds.max.x, bounds.min.y, bounds.max.z},
			{bounds.min.x, bounds.max.y, bounds.max.z}, {bounds.max.x, bounds.max.y, bounds.max.z},
		};
		for corner in corners
		{
			rotated := util.quaternion_transform(corner, orientation);
			result.min = util.vector3_min(result.min, rotated);
			result.max = util.vector3_max(result.max, rotated);
			result.maximum_radius = max(result.maximum_radius, util.vector3_length(corner));
		}
	}
	result.maximum_angular_expansion = result.maximum_radius;
	return result;
}

mesh_create :: proc (
	mesh: ^Mesh, triangles: [^]Triangle, triangle_count: int, scale: util.Vector3, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if mesh == nil || pool == nil || triangles == nil || triangle_count <= 0 ||
		abs(scale.x) <= 1e-20 || abs(scale.y) <= 1e-20 || abs(scale.z) <= 1e-20
	{
		return .Invalid_Description;
	}
	for index in 0 ..< triangle_count
	{
		if triangle_validate(triangles[index]) != .Ok
		{
			return .Invalid_Description;
		}
	}
	buffer, status := shape_copy_buffer(pool, triangles, triangle_count);
	if status != .Ok
	{
		return status;
	}
	bounds_buffer, bounds_status := util.buffer_pool_take_at_least(pool, util.Bounding_Box, triangle_count);
	if bounds_status != .Ok
	{
		physics_return_buffer(pool, &buffer);
		return physics_memory_status(bounds_status);
	}
	bounds_buffer.length = i32(triangle_count);
	for index in 0 ..< triangle_count
	{
		triangle := triangles[index];
		a := util.vector3_multiply(triangle.a, scale);
		b := util.vector3_multiply(triangle.b, scale);
		c := util.vector3_multiply(triangle.c, scale);
		bounds_buffer.memory[index] = {
			min=util.vector3_min(a, util.vector3_min(b, c)),
			max=util.vector3_max(a, util.vector3_max(b, c)),
		};
	}
	tree: Tree;
	tree_status := tree_initialize(&tree, triangle_count, pool);
	if tree_status != .Ok
	{
		physics_return_buffer(pool, &bounds_buffer);
		physics_return_buffer(pool, &buffer);
		return tree_status;
	}
	build_status := tree_build(&tree, bounds_buffer.memory, triangle_count);
	if build_status != .Ok
	{
		_ = tree_dispose(&tree);
		physics_return_buffer(pool, &bounds_buffer);
		physics_return_buffer(pool, &buffer);
		return build_status;
	}
	mesh^ = {triangles=buffer, triangle_bounds=bounds_buffer, tree=tree, scale=scale};
	return .Ok;
}

mesh_dispose :: proc (mesh: ^Mesh, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if mesh == nil || pool == nil || mesh.triangles.memory == nil
	{
		return .Disposed;
	}
	_ = tree_dispose(&mesh.tree);
	physics_return_buffer(pool, &mesh.triangle_bounds);
	physics_return_buffer(pool, &mesh.triangles);
	mesh^ = {};
	return .Ok;
}

mesh_bounds :: proc "contextless" (mesh: ^Mesh, orientation: util.Quaternion) -> Shape_Bounds
{
	result := Shape_Bounds{
		min={f32(math.F32_MAX), f32(math.F32_MAX), f32(math.F32_MAX)},
		max={-f32(math.F32_MAX), -f32(math.F32_MAX), -f32(math.F32_MAX)},
	};
	for index in 0 ..< mesh.triangles.length
	{
		triangle := mesh.triangles.memory[index];
		vertices := [3]util.Vector3{
			util.vector3_multiply(triangle.a, mesh.scale),
			util.vector3_multiply(triangle.b, mesh.scale),
			util.vector3_multiply(triangle.c, mesh.scale),
		};
		for vertex in vertices
		{
			world := util.quaternion_transform(vertex, orientation);
			result.min = util.vector3_min(result.min, world);
			result.max = util.vector3_max(result.max, world);
			result.maximum_radius = max(result.maximum_radius, util.vector3_length(vertex));
		}
	}
	result.maximum_angular_expansion = result.maximum_radius;
	return result;
}

mesh_closed_inertia :: proc "contextless" (mesh: ^Mesh, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if mesh == nil || mesh.triangles.memory == nil || mesh.triangles.length <= 0 || mass <= 0
	{
		return {}, .Invalid_Argument;
	}
	volume: f32;
	summed: util.Symmetric3x3;
	for index in 0 ..< mesh.triangles.length
	{
		triangle := mesh.triangles.memory[index];
		a := util.vector3_multiply(triangle.a, mesh.scale);
		b := util.vector3_multiply(triangle.b, mesh.scale);
		c := util.vector3_multiply(triangle.c, mesh.scale);
		tetrahedron_volume := mesh_tetrahedron_volume(a, b, c);
		volume += tetrahedron_volume;
		summed = util.symmetric3x3_add(summed, mesh_tetrahedron_inertia_tensor(a, b, c, tetrahedron_volume));
	}
	if abs(volume) <= 1e-10
	{
		return {}, .Invalid_Description;
	}
	if volume < 0
	{
		volume = -volume;
		summed = util.symmetric3x3_scale(summed, -1);
	}
	return shape_inertia_from_tensor(util.symmetric3x3_scale(summed, mass / volume), mass);
}

mesh_open_inertia :: proc "contextless" (mesh: ^Mesh, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if mesh == nil || mesh.triangles.memory == nil || mesh.triangles.length <= 0 || mass <= 0
	{
		return {}, .Invalid_Argument;
	}
	total_area: f32;
	summed: util.Symmetric3x3;
	for index in 0 ..< mesh.triangles.length
	{
		triangle := mesh.triangles.memory[index];
		triangle.a = util.vector3_multiply(triangle.a, mesh.scale);
		triangle.b = util.vector3_multiply(triangle.b, mesh.scale);
		triangle.c = util.vector3_multiply(triangle.c, mesh.scale);
		area := 0.5 * util.vector3_length(util.vector3_cross(
			util.vector3_subtract(triangle.b, triangle.a), util.vector3_subtract(triangle.c, triangle.a),
		));
		total_area += area;
		summed = util.symmetric3x3_add(summed, triangle_inertia_tensor(triangle, area));
	}
	if total_area <= 1e-10
	{
		return {}, .Invalid_Description;
	}
	return shape_inertia_from_tensor(util.symmetric3x3_scale(summed, mass / total_area), mass);
}

mesh_closed_center_of_mass :: proc "contextless" (
	mesh: ^Mesh,
) -> (volume: f32, center: util.Vector3, status: Physics_Status)
{
	if mesh == nil || mesh.triangles.memory == nil || mesh.triangles.length <= 0
	{
		return 0, {}, .Invalid_Argument;
	}
	for index in 0 ..< mesh.triangles.length
	{
		triangle := mesh.triangles.memory[index];
		a := util.vector3_multiply(triangle.a, mesh.scale);
		b := util.vector3_multiply(triangle.b, mesh.scale);
		c := util.vector3_multiply(triangle.c, mesh.scale);
		tetrahedron_volume := mesh_tetrahedron_volume(a, b, c);
		volume += tetrahedron_volume;
		center = util.vector3_add(center, util.vector3_scale(
			util.vector3_add(util.vector3_add(a, b), c), tetrahedron_volume,
		));
	}
	if abs(volume) <= 1e-10
	{
		return 0, {}, .Invalid_Description;
	}
	center = util.vector3_scale(center, 1 / (volume * 4));
	return volume, center, .Ok;
}

mesh_open_center_of_mass :: proc "contextless" (
	mesh: ^Mesh,
) -> (center: util.Vector3, status: Physics_Status)
{
	if mesh == nil || mesh.triangles.memory == nil || mesh.triangles.length <= 0
	{
		return {}, .Invalid_Argument;
	}
	total_area: f32;
	for index in 0 ..< mesh.triangles.length
	{
		triangle := mesh.triangles.memory[index];
		a := util.vector3_multiply(triangle.a, mesh.scale);
		b := util.vector3_multiply(triangle.b, mesh.scale);
		c := util.vector3_multiply(triangle.c, mesh.scale);
		area := 0.5 * util.vector3_length(util.vector3_cross(
			util.vector3_subtract(b, a), util.vector3_subtract(c, a),
		));
		total_area += area;
		center = util.vector3_add(center, util.vector3_scale(
			util.vector3_add(util.vector3_add(a, b), c), area,
		));
	}
	if total_area <= 1e-10
	{
		return {}, .Invalid_Description;
	}
	return util.vector3_scale(center, 1 / (total_area * 3)), .Ok;
}

mesh_inertia_offset :: #force_inline proc "contextless" (
	mass: f32, offset: util.Vector3,
) -> util.Symmetric3x3
{
	squared := util.vector3_multiply(offset, offset);
	diagonal := squared.x + squared.y + squared.z;
	return {
		mass * (squared.x - diagonal),
		mass * (offset.x * offset.y),
		mass * (squared.y - diagonal),
		mass * (offset.x * offset.z),
		mass * (offset.y * offset.z),
		mass * (squared.z - diagonal),
	};
}

mesh_recenter :: proc "contextless" (
	mesh: ^Mesh, center: util.Vector3,
) -> Physics_Status
{
	if mesh == nil || mesh.triangles.memory == nil || mesh.triangles.length <= 0 ||
		mesh.scale.x == 0 || mesh.scale.y == 0 || mesh.scale.z == 0
	{
		return .Invalid_Argument;
	}
	scaled_offset := util.Vector3{
		center.x / mesh.scale.x,
		center.y / mesh.scale.y,
		center.z / mesh.scale.z,
	};
	for index in 0 ..< mesh.triangles.length
	{
		triangle := &mesh.triangles.memory[index];
		triangle.a = util.vector3_subtract(triangle.a, scaled_offset);
		triangle.b = util.vector3_subtract(triangle.b, scaled_offset);
		triangle.c = util.vector3_subtract(triangle.c, scaled_offset);
	}
	for index in 0 ..< mesh.triangle_bounds.length
	{
		bounds := &mesh.triangle_bounds.memory[index];
		bounds.min = util.vector3_subtract(bounds.min, center);
		bounds.max = util.vector3_subtract(bounds.max, center);
	}
	for index in 0 ..< mesh.tree.node_count
	{
		node := &mesh.tree.nodes.memory[index];
		node.a.min = util.vector3_subtract(node.a.min, center);
		node.a.max = util.vector3_subtract(node.a.max, center);
		node.b.min = util.vector3_subtract(node.b.min, center);
		node.b.max = util.vector3_subtract(node.b.max, center);
	}
	return .Ok;
}

mesh_closed_inertia_recenter :: proc "contextless" (
	mesh: ^Mesh, mass: f32,
) -> (inertia: Body_Inertia, center: util.Vector3, status: Physics_Status)
{
	if mass <= 0
	{
		return {}, {}, .Invalid_Argument;
	}
	_, computed_center, center_status := mesh_closed_center_of_mass(mesh);
	if center_status != .Ok
	{
		return {}, {}, center_status;
	}
	origin_inertia, inertia_status := mesh_closed_inertia(mesh, mass);
	if inertia_status != .Ok
	{
		return {}, {}, inertia_status;
	}
	origin_tensor := util.symmetric3x3_invert(origin_inertia.inverse_inertia_tensor);
	recentered_tensor := util.symmetric3x3_add(origin_tensor, mesh_inertia_offset(mass, computed_center));
	inertia, status = shape_inertia_from_tensor(recentered_tensor, mass);
	if status != .Ok
	{
		return {}, {}, status;
	}
	status = mesh_recenter(mesh, computed_center);
	if status != .Ok
	{
		return {}, {}, status;
	}
	return inertia, computed_center, .Ok;
}

mesh_open_inertia_recenter :: proc "contextless" (
	mesh: ^Mesh, mass: f32,
) -> (inertia: Body_Inertia, center: util.Vector3, status: Physics_Status)
{
	if mass <= 0
	{
		return {}, {}, .Invalid_Argument;
	}
	computed_center, center_status := mesh_open_center_of_mass(mesh);
	if center_status != .Ok
	{
		return {}, {}, center_status;
	}
	origin_inertia, inertia_status := mesh_open_inertia(mesh, mass);
	if inertia_status != .Ok
	{
		return {}, {}, inertia_status;
	}
	origin_tensor := util.symmetric3x3_invert(origin_inertia.inverse_inertia_tensor);
	recentered_tensor := util.symmetric3x3_add(origin_tensor, mesh_inertia_offset(mass, computed_center));
	inertia, status = shape_inertia_from_tensor(recentered_tensor, mass);
	if status != .Ok
	{
		return {}, {}, status;
	}
	status = mesh_recenter(mesh, computed_center);
	if status != .Ok
	{
		return {}, {}, status;
	}
	return inertia, computed_center, .Ok;
}
