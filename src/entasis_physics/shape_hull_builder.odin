// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
// Convex_Hull owns the runtime shape's allocation. all gift-wrapping scratch is borrowed
// from the pool and returned before this procedure exits
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

Hull_Builder_Edge :: struct
{
	a, b:       i32,
	face_normal: util.Vector3,
}

Hull_Builder_Edge_Count :: struct
{
	a, b: i32,
	count: i32,
}

hull_builder_perpendicular :: proc "contextless" (direction: util.Vector3) -> util.Vector3
{
	ax := abs(direction.x);
	ay := abs(direction.y);
	az := abs(direction.z);
	axis := util.Vector3{1, 0, 0};
	if ay <= ax && ay <= az
	{
		axis = {0, 1, 0};
	}
	else if az <= ax && az <= ay
	{
		axis = {0, 0, 1};
	}
	return util.vector3_normalize(util.vector3_cross(direction, axis));
}

hull_builder_find_extreme_face :: proc "contextless" (
	points: [^]util.Vector3, point_count: int, basis_x, basis_y, basis_origin: util.Vector3,
	source_a, source_b: int, epsilon: f32, raw_indices: [^]i32,
) -> (int, util.Vector3, Physics_Status)
{
	best_x := f32(1);
	best_y := -f32(math.F32_MAX);
	best_index := -1;
	for index in 0 ..< point_count
	{
		if index == source_a || index == source_b
		{
			continue;
		}
		to_candidate := util.vector3_subtract(points[index], basis_origin);
		x := max(f32(0), util.vector3_dot(basis_x, to_candidate));
		y := util.vector3_dot(basis_y, to_candidate);
		if x <= epsilon && y <= epsilon
		{
			continue;
		}
		if best_index < 0 || y * best_x > best_y * x
		{
			best_x = x;
			best_y = y;
			best_index = index;
		}
	}
	if best_index < 0
	{
		return 0, {}, .Invalid_Description;
	}
	normal_2d := util.Vector2{-best_y, best_x};
	normal_length := util.vector2_length(normal_2d);
	if normal_length <= 1e-20
	{
		return 0, {}, .Invalid_Description;
	}
	normal_2d = util.vector2_scale(normal_2d, 1 / normal_length);
	face_normal := util.vector3_add(
		util.vector3_scale(basis_x, normal_2d.x), util.vector3_scale(basis_y, normal_2d.y),
	);
	raw_count := 0;
	for index in 0 ..< point_count
	{
		to_candidate := util.vector3_subtract(points[index], basis_origin);
		if util.vector3_dot(to_candidate, face_normal) >= -epsilon
		{
			raw_indices[raw_count] = i32(index);
			raw_count += 1;
		}
	}
	if raw_count < 2
	{
		return raw_count, face_normal, .Invalid_Description;
	}
	return raw_count, face_normal, .Ok;
}

hull_builder_find_next_face_index :: proc "contextless" (
	start, previous_edge_direction: util.Vector2, epsilon: f32,
	face_points: [^]util.Vector2, point_count: int,
) -> int
{
	basis_x := util.Vector2{previous_edge_direction.y, -previous_edge_direction.x};
	basis_y := util.vector2_scale(previous_edge_direction, -1);
	best_x := f32(1);
	best_y := f32(math.F32_MAX);
	best_index := -1;
	for index in 0 ..< point_count
	{
		to_candidate := util.vector2_subtract(face_points[index], start);
		x := max(f32(0), util.vector2_dot(to_candidate, basis_x));
		y := util.vector2_dot(to_candidate, basis_y);
		if x <= epsilon && y >= -epsilon
		{
			continue;
		}
		if y * best_x < best_y * x
		{
			best_x = x;
			best_y = y;
			best_index = index;
		}
	}
	if best_index < 0
	{
		return -1;
	}
	best_direction := util.Vector2{best_x, best_y};
	length := util.vector2_length(best_direction);
	if length <= 1e-20
	{
		return best_index;
	}
	best_direction = util.vector2_scale(best_direction, 1 / length);
	edge_direction := util.vector2_add(
		util.vector2_scale(basis_x, best_direction.x), util.vector2_scale(basis_y, best_direction.y),
	);
	face_normal := util.Vector2{-edge_direction.y, edge_direction.x};
	distance: f32;
	most_distant := -1;
	for index in 0 ..< point_count
	{
		to_candidate := util.vector2_subtract(face_points[index], start);
		if util.vector2_dot(to_candidate, face_normal) > -epsilon
		{
			along_edge := util.vector2_dot(to_candidate, edge_direction);
			if along_edge > distance
			{
				distance = along_edge;
				most_distant = index;
			}
		}
	}
	if most_distant >= 0
	{
		return most_distant;
	}
	return best_index;
}

hull_builder_reduce_face :: proc "contextless" (
	points: [^]util.Vector3, raw_indices: [^]i32, raw_count: int,
	face_normal: util.Vector3, epsilon: f32,
	projected: [^]util.Vector2, reduced_indices: [^]i32,
) -> (int, Physics_Status)
{
	if raw_count < 2
	{
		return 0, .Invalid_Description;
	}
	if raw_count <= 3
	{
		for index in 0 ..< raw_count
		{
			reduced_indices[index] = raw_indices[index];
		}
		if raw_count == 3
		{
			a := points[reduced_indices[0]];
			b := points[reduced_indices[1]];
			c := points[reduced_indices[2]];
			normal := util.vector3_cross(util.vector3_subtract(b, a), util.vector3_subtract(c, a));
			if util.vector3_length_squared(normal) <= 1e-14
			{
				return 0, .Invalid_Description;
			}
			if util.vector3_dot(face_normal, normal) < 0
			{
				reduced_indices[0], reduced_indices[1] = reduced_indices[1], reduced_indices[0];
			}
		}
		return raw_count, .Ok;
	}
	basis_x := hull_builder_perpendicular(face_normal);
	basis_y := util.vector3_cross(face_normal, basis_x);
	centroid: util.Vector2;
	for index in 0 ..< raw_count
	{
		point := points[raw_indices[index]];
		projected[index] = {util.vector3_dot(basis_x, point), util.vector3_dot(basis_y, point)};
		centroid = util.vector2_add(centroid, projected[index]);
	}
	centroid = util.vector2_scale(centroid, 1 / f32(raw_count));
	initial_index := 0;
	greatest_distance_squared: f32 = -1;
	for index in 0 ..< raw_count
	{
		distance_squared := util.vector2_length_squared(util.vector2_subtract(projected[index], centroid));
		if distance_squared > greatest_distance_squared
		{
			greatest_distance_squared = distance_squared;
			initial_index = index;
		}
	}
	if greatest_distance_squared <= 1e-14
	{
		return 0, .Invalid_Description;
	}
	initial_direction := util.vector2_scale(
		util.vector2_subtract(projected[initial_index], centroid), 1 / math.sqrt(greatest_distance_squared),
	);
	previous_direction := util.Vector2{initial_direction.y, -initial_direction.x};
	reduced_count := 1;
	reduced_indices[0] = raw_indices[initial_index];
	previous_end := initial_index;
	for _ in 0 ..< raw_count
	{
		next_index := hull_builder_find_next_face_index(
			projected[previous_end], previous_direction, epsilon, projected, raw_count,
		);
		if next_index < 0
		{
			return 0, .Invalid_Description;
		}
		next_original := raw_indices[next_index];
		cycle_index := -1;
		for existing_index in 0 ..< reduced_count
		{
			if reduced_indices[existing_index] == next_original
			{
				cycle_index = existing_index;
				break;
			}
		}
		if cycle_index >= 0
		{
			if cycle_index > 0
			{
				for index in cycle_index ..< reduced_count
				{
					reduced_indices[index - cycle_index] = reduced_indices[index];
				}
				reduced_count -= cycle_index;
			}
			break;
		}
		reduced_indices[reduced_count] = next_original;
		reduced_count += 1;
		edge := util.vector2_subtract(projected[next_index], projected[previous_end]);
		edge_length := util.vector2_length(edge);
		if edge_length <= 1e-20
		{
			return 0, .Invalid_Description;
		}
		previous_direction = util.vector2_scale(edge, 1 / edge_length);
		previous_end = next_index;
	}
	if reduced_count < 3
	{
		return reduced_count, .Invalid_Description;
	}
	a := points[reduced_indices[0]];
	b := points[reduced_indices[1]];
	c := points[reduced_indices[2]];
	uncalibrated := util.vector3_cross(util.vector3_subtract(b, a), util.vector3_subtract(c, a));
	if util.vector3_dot(face_normal, uncalibrated) < 0
	{
		for index in 0 ..< reduced_count / 2
		{
			opposite := reduced_count - 1 - index;
			reduced_indices[index], reduced_indices[opposite] = reduced_indices[opposite], reduced_indices[index];
		}
	}
	return reduced_count, .Ok;
}

hull_builder_find_edge_record :: proc "contextless" (
	edges: [^]Hull_Builder_Edge_Count, edge_count, a, b: int,
) -> int
{
	minimum := min(a, b);
	maximum := max(a, b);
	for index in 0 ..< edge_count
	{
		if edges[index].a == i32(minimum) && edges[index].b == i32(maximum)
		{
			return index;
		}
	}
	return -1;
}

hull_builder_face_exists :: proc "contextless" (
	points: [^]util.Vector3, face_starts: [^]i32, face_indices: [^]i32, face_normals: [^]util.Vector3,
	face_count, face_index_count: int, normal: util.Vector3, candidate_first_index: int, epsilon: f32,
) -> Reference_State
{
	candidate_offset := util.vector3_dot(normal, points[candidate_first_index]);
	for face_index in 0 ..< face_count
	{
		if util.vector3_dot(face_normals[face_index], normal) <= 1 - 1e-5
		{
			continue;
		}
		start := int(face_starts[face_index]);
		if start >= face_index_count
		{
			continue;
		}
		offset := util.vector3_dot(face_normals[face_index], points[face_indices[start]]);
		if abs(offset - candidate_offset) <= epsilon * 2
		{
			return .Present;
		}
	}
	return .Missing;
}

hull_builder_add_face :: proc "contextless" (
	points: [^]util.Vector3,
	face_starts: [^]i32, face_indices: [^]i32, face_normals: [^]util.Vector3,
	maximum_face_count, maximum_face_indices: int, face_count, face_index_count: ^int,
	edge_records: [^]Hull_Builder_Edge_Count, maximum_edges: int, edge_count: ^int,
	edge_work: [^]Hull_Builder_Edge, maximum_work_count: int, work_count: ^int,
	epsilon: f32, normal: util.Vector3, reduced: [^]i32, count: int,
) -> Physics_Status
{
	if count < 3 || face_count^ >= maximum_face_count || face_index_count^ > maximum_face_indices - count
	{
		return .Capacity_Missing;
	}
	if hull_builder_face_exists(
		points, face_starts, face_indices, face_normals,
		face_count^, face_index_count^, normal, int(reduced[0]), epsilon,
	) == .Present
	{
		return .Ok;
	}
	face_starts[face_count^] = i32(face_index_count^);
	face_normals[face_count^] = normal;
	for index in 0 ..< count
	{
		face_indices[face_index_count^ + index] = reduced[index];
	}
	face_index_count^ += count;
	face_count^ += 1;
	previous := int(reduced[count - 1]);
	for index in 0 ..< count
	{
		next := int(reduced[index]);
		record_index := hull_builder_find_edge_record(edge_records, edge_count^, previous, next);
		if record_index < 0
		{
			if edge_count^ >= maximum_edges || work_count^ >= maximum_work_count
			{
				return .Capacity_Missing;
			}
			edge_records[edge_count^] = {i32(min(previous, next)), i32(max(previous, next)), 1};
			edge_count^ += 1;
			edge_work[work_count^] = {i32(previous), i32(next), normal};
			work_count^ += 1;
		}
		else
		{
			edge_records[record_index].count += 1;
			if edge_records[record_index].count == 1
			{
				if work_count^ >= maximum_work_count
				{
					return .Capacity_Missing;
				}
				edge_work[work_count^] = {i32(previous), i32(next), normal};
				work_count^ += 1;
			}
		}
		previous = next;
	}
	return .Ok;
}

convex_hull_create_from_point_cloud :: proc (
	hull: ^Convex_Hull, center: ^util.Vector3,
	points: [^]util.Vector3, point_count: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if hull == nil || center == nil || points == nil || point_count < 4 || pool == nil
	{
		return .Invalid_Description;
	}
	maximum_face_count := max(point_count * 2, 8);
	maximum_face_indices := maximum_face_count * point_count;
	maximum_edges := max(point_count * 6, 16);
	raw_indices, raw_status := util.buffer_pool_take_at_least(pool, i32, point_count);
	if raw_status != .Ok
	{
		return physics_memory_status(raw_status);
	}
	defer physics_return_buffer(pool, &raw_indices);
	reduced_indices, reduced_status := util.buffer_pool_take_at_least(pool, i32, point_count);
	if reduced_status != .Ok
	{
		return physics_memory_status(reduced_status);
	}
	defer physics_return_buffer(pool, &reduced_indices);
	projected, projected_status := util.buffer_pool_take_at_least(pool, util.Vector2, point_count);
	if projected_status != .Ok
	{
		return physics_memory_status(projected_status);
	}
	defer physics_return_buffer(pool, &projected);
	face_starts, starts_status := util.buffer_pool_take_at_least(pool, i32, maximum_face_count);
	if starts_status != .Ok
	{
		return physics_memory_status(starts_status);
	}
	defer physics_return_buffer(pool, &face_starts);
	face_normals, normals_status := util.buffer_pool_take_at_least(pool, util.Vector3, maximum_face_count);
	if normals_status != .Ok
	{
		return physics_memory_status(normals_status);
	}
	defer physics_return_buffer(pool, &face_normals);
	face_indices, indices_status := util.buffer_pool_take_at_least(pool, i32, maximum_face_indices);
	if indices_status != .Ok
	{
		return physics_memory_status(indices_status);
	}
	defer physics_return_buffer(pool, &face_indices);
	edge_records, records_status := util.buffer_pool_take_at_least(pool, Hull_Builder_Edge_Count, maximum_edges);
	if records_status != .Ok
	{
		return physics_memory_status(records_status);
	}
	defer physics_return_buffer(pool, &edge_records);
	edge_work, work_status := util.buffer_pool_take_at_least(pool, Hull_Builder_Edge, maximum_edges * 2);
	if work_status != .Ok
	{
		return physics_memory_status(work_status);
	}
	defer physics_return_buffer(pool, &edge_work);

	point_centroid: util.Vector3;
	for index in 0 ..< point_count
	{
		point_centroid = util.vector3_add(point_centroid, points[index]);
	}
	point_centroid = util.vector3_scale(point_centroid, 1 / f32(point_count));
	initial_index := 0;
	best_distance_squared: f32;
	for index in 0 ..< point_count
	{
		distance_squared := util.vector3_distance_squared(points[index], point_centroid);
		if distance_squared > best_distance_squared
		{
			best_distance_squared = distance_squared;
			initial_index = index;
		}
	}
	if best_distance_squared <= 1e-14
	{
		return .Invalid_Description;
	}
	initial_to_centroid := util.vector3_subtract(point_centroid, points[initial_index]);
	initial_basis_x := util.vector3_normalize(initial_to_centroid);
	initial_basis_y := hull_builder_perpendicular(initial_basis_x);
	epsilon := math.sqrt(best_distance_squared) * 1e-4;
	raw_count, initial_normal, find_status := hull_builder_find_extreme_face(
		points, point_count, initial_basis_x, initial_basis_y, points[initial_index],
		initial_index, initial_index, epsilon, raw_indices.memory,
	);
	if find_status != .Ok
	{
		return find_status;
	}
	reduced_count, reduce_status := hull_builder_reduce_face(
		points, raw_indices.memory, raw_count, initial_normal, epsilon, projected.memory, reduced_indices.memory,
	);
	if reduce_status != .Ok
	{
		return reduce_status;
	}

	face_count := 0;
	face_index_count := 0;
	edge_count := 0;
	work_count := 0;
	status: Physics_Status;
	if reduced_count >= 3
	{
		status = hull_builder_add_face(
			points,
			face_starts.memory, face_indices.memory, face_normals.memory,
			maximum_face_count, maximum_face_indices, &face_count, &face_index_count,
			edge_records.memory, maximum_edges, &edge_count,
			edge_work.memory, int(edge_work.length), &work_count,
			epsilon, initial_normal, reduced_indices.memory, reduced_count,
		);
		if status != .Ok
		{
			return status;
		}
	}
	else
	{
		a := int(reduced_indices.memory[0]);
		b := int(reduced_indices.memory[1]);
		edge_offset := util.vector3_subtract(points[b], points[a]);
		basis_y := util.vector3_cross(edge_offset, initial_normal);
		basis_x := util.vector3_cross(edge_offset, basis_y);
		if util.vector3_dot(basis_x, initial_normal) > 0
		{
			a, b = b, a;
		}
		edge_records.memory[0] = {i32(min(a, b)), i32(max(a, b)), 0};
		edge_count = 1;
		edge_work.memory[0] = {i32(a), i32(b), initial_normal};
		work_count = 1;
	}
	for work_count > 0
	{
		work_count -= 1;
		edge := edge_work.memory[work_count];
		record_index := hull_builder_find_edge_record(edge_records.memory, edge_count, int(edge.a), int(edge.b));
		if record_index < 0 || edge_records.memory[record_index].count >= 2
		{
			continue;
		}
		edge_a := points[edge.a];
		edge_b := points[edge.b];
		edge_offset := util.vector3_subtract(edge_b, edge_a);
		if util.vector3_length_squared(edge_offset) <= 1e-20
		{
			return .Invalid_Description;
		}
		basis_y := util.vector3_normalize(util.vector3_cross(edge_offset, edge.face_normal));
		basis_x := util.vector3_normalize(util.vector3_cross(edge_offset, basis_y));
		if util.vector3_dot(basis_x, edge.face_normal) > 0
		{
			edge.a, edge.b = edge.b, edge.a;
			edge_a = points[edge.a];
			edge_b = points[edge.b];
			edge_offset = util.vector3_subtract(edge_b, edge_a);
			basis_y = util.vector3_normalize(util.vector3_cross(edge_offset, edge.face_normal));
			basis_x = util.vector3_normalize(util.vector3_cross(edge_offset, basis_y));
		}
		normal: util.Vector3;
		extreme_status: Physics_Status;
		raw_count, normal, extreme_status = hull_builder_find_extreme_face(
			points, point_count, basis_x, basis_y, edge_a, int(edge.a), int(edge.b), epsilon, raw_indices.memory,
		);
		if extreme_status != .Ok
		{
			return extreme_status;
		}
		reduced_count, reduce_status = hull_builder_reduce_face(
			points, raw_indices.memory, raw_count, normal, epsilon, projected.memory, reduced_indices.memory,
		);
		if reduce_status != .Ok
		{
			return reduce_status;
		}
		prior_face_count := face_count;
		status = hull_builder_add_face(
			points,
			face_starts.memory, face_indices.memory, face_normals.memory,
			maximum_face_count, maximum_face_indices, &face_count, &face_index_count,
			edge_records.memory, maximum_edges, &edge_count,
			edge_work.memory, int(edge_work.length), &work_count,
			epsilon, normal, reduced_indices.memory, reduced_count,
		);
		if status != .Ok
		{
			return status;
		}
		if face_count == prior_face_count
		{
			// the adjacent search returned an already known coplanar face. the source edge is closed
			edge_records.memory[record_index].count = 2;
		}
	}
	if face_count < 4
	{
		return .Invalid_Description;
	}
	for edge_index in 0 ..< edge_count
	{
		if edge_records.memory[edge_index].count != 2
		{
			return .Invalid_Description;
		}
	}

	volume: f32;
	computed_center: util.Vector3;
	for face_index in 0 ..< face_count
	{
		start := int(face_starts.memory[face_index]);
		end := face_index_count;
		if face_index + 1 < face_count
		{
			end = int(face_starts.memory[face_index + 1]);
		}
		a := points[face_indices.memory[start]];
		for index in start + 1 ..< end - 1
		{
			b := points[face_indices.memory[index]];
			c := points[face_indices.memory[index + 1]];
			contribution := mesh_tetrahedron_volume(a, b, c);
			volume += contribution;
			computed_center = util.vector3_add(
				computed_center, util.vector3_scale(util.vector3_add(util.vector3_add(a, b), c), contribution),
			);
		}
	}
	if abs(volume) <= 1e-10
	{
		return .Invalid_Description;
	}
	computed_center = util.vector3_scale(computed_center, 1 / (volume * 4));
	centered_points, centered_status := util.buffer_pool_take_at_least(pool, util.Vector3, point_count);
	if centered_status != .Ok
	{
		return physics_memory_status(centered_status);
	}
	defer physics_return_buffer(pool, &centered_points);
	for index in 0 ..< point_count
	{
		centered_points.memory[index] = util.vector3_subtract(points[index], computed_center);
	}
	status = convex_hull_create(
		hull, centered_points.memory, point_count,
		face_starts.memory, face_count, face_indices.memory, face_index_count, pool,
	);
	if status == .Ok
	{
		center^ = computed_center;
	}
	return status;
}
