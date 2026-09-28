// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

MESH_REDUCTION_FACE_COLLISION_FLAG :: i32(32768);
MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION :: f32(0.999999);
Collision_Child :: struct
{
	shape:       rawptr,
	type_id:     int,
	pose:        Rigid_Pose,
	child_index: int,
	mesh_owner:  ^Mesh,
	mesh_pose:   Rigid_Pose,
}

Collision_Child_Contact_Payload :: struct
{
	offset:     util.Vector3,
	depth:      f32,
	feature_id: i32,
}

Collision_Child_Contact_Data :: struct
{
	offset:     util.Vector3,
	feature_id: i32,
}

Collision_Child_Manifold_Header :: struct
{
	normal:       util.Vector3,
	child_a:      i32,
	child_b:      i32,
	mesh_child_b: i32,
	contact_start: i32,
	contact_count: i8,
	completed:    Reference_State,
}

Collision_Child_Manifold_Record :: struct
{
	offset_a: util.Vector3,
	header:   Collision_Child_Manifold_Header,
	contacts: [MAXIMUM_MANIFOLD_CONTACT_COUNT]Collision_Child_Contact_Payload,
}
#assert(size_of(Collision_Child_Contact_Payload) == 20);
#assert(size_of(Collision_Child_Contact_Data) == 16);
#assert(size_of(Collision_Child_Manifold_Header) == 32);
#assert(size_of(Collision_Child_Manifold_Record) == 124);
collision_reduction_candidate_reference :: proc "contextless" (
	child_index, contact_index: int,
) -> Collision_Reduction_Candidate_Reference
{
	return {packed=(u32(child_index) << 2) | u32(contact_index)};
}

collision_reduction_candidate_child :: proc "contextless" (
	reference: Collision_Reduction_Candidate_Reference,
) -> int
{
	return int(reference.packed >> 2);
}

collision_reduction_candidate_contact :: proc "contextless" (
	reference: Collision_Reduction_Candidate_Reference,
) -> int
{
	return int(reference.packed & 3);
}

collision_child_feature_id :: proc "contextless" (child_a, child_b: int, feature_id: i32) -> i32
{
	return feature_id ~ (i32(child_a) << 8) ~ (i32(child_b) << 16);
}

collision_contact_distinctiveness :: proc "contextless" (
	candidate, reduced: Contact,
	distance_squared_interpolation_min, inverse_distance_squared_interpolation_span, depth_scale: f32,
) -> f32
{
	normal_dot := util.vector3_dot(candidate.normal, reduced.normal);
	normal_distinctiveness := (normal_dot - 0.99999) * (1 / -0.99999);
	offset_distinctiveness := (
		util.vector3_length_squared(util.vector3_subtract(reduced.offset, candidate.offset)) -
		distance_squared_interpolation_min
	) * inverse_distance_squared_interpolation_span;
	combined_distinctiveness := normal_distinctiveness * offset_distinctiveness;
	distinctiveness := max(offset_distinctiveness, max(normal_distinctiveness, combined_distinctiveness));
	if distinctiveness <= 0
	{
		return 0;
	}
	depth_multiplier := max(f32(0.01), 1 + candidate.depth * depth_scale);
	return distinctiveness * depth_multiplier;
}

collision_reduction_record_contact :: proc "contextless" (
	records: Collision_Batcher_Child_Storage,
	reference: Collision_Reduction_Candidate_Reference,
) -> Contact
{
	child_index := collision_reduction_candidate_child(reference);
	contact_index := collision_reduction_candidate_contact(reference);
	record := collision_child_storage_header(records, child_index);
	payload := collision_child_storage_read_contact(records, child_index, contact_index);
	return {
		offset=util.vector3_add(collision_child_storage_offset_a(records, child_index)^, payload.offset),
		depth=payload.depth,
		normal=record.normal,
		feature_id=collision_child_feature_id(
			int(record.child_a), int(record.child_b), payload.feature_id,
		),
	};
}

collision_reduction_prepare_records :: proc "contextless" (
	records: Collision_Batcher_Child_Storage,
	references: Collision_Reduction_Candidate_Storage,
	parent_position_a: util.Vector3,
	mesh_b: ^Mesh,
	mesh_pose_b: Rigid_Pose,
) -> Physics_Status
{
	if collision_child_storage_validate(records, int(records.length)) != .Ok ||
	references.memory == nil ||
	records.length < 0 || int(records.length) > int(references.length) / MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		return .Invalid_Argument;
	}
	for record_index in 0 ..< int(records.length)
	{
		state_index := record_index * MAXIMUM_MANIFOLD_CONTACT_COUNT;
		collision_candidate_storage_write(references, state_index, {});
	}
	if mesh_b == nil
	{
		return .Ok;
	}
	for source_index in 0 ..< int(records.length)
	{
		source := collision_child_storage_header(records, source_index);
		source_count := int(source.contact_count);
		if source_count <= 0
		{
			continue;
		}
		first_contact := collision_child_storage_read_contact(records, source_index, 0);
		face_collision := first_contact.feature_id &
		MESH_REDUCTION_FACE_COLLISION_FLAG != 0;
		if face_collision
		{
			for contact_index in 0 ..< source_count
			{
				contact := collision_child_storage_read_contact(records, source_index, contact_index);
				contact.feature_id &= ~MESH_REDUCTION_FACE_COLLISION_FLAG;
				collision_child_storage_write_contact(
					records, source_index, contact_index, contact,
				);
			}
			continue;
		}
		deepest_index := 0;
		for contact_index in 1 ..< source_count
		{
			contact := collision_child_storage_read_contact(records, source_index, contact_index);
			deepest := collision_child_storage_read_contact(records, source_index, deepest_index);
			if contact.depth > deepest.depth
			{
				deepest_index = contact_index;
			}
		}
		deepest := collision_child_storage_read_contact(records, source_index, deepest_index);
		world_contact := util.vector3_add(
			parent_position_a,
			util.vector3_add(
				collision_child_storage_offset_a(records, source_index)^,
				deepest.offset,
			),
		);
		mesh_contact := rigid_pose_transform_by_inverse(world_contact, mesh_pose_b);
		mesh_normal := util.quaternion_transform(
			source.normal, util.quaternion_conjugate(mesh_pose_b.orientation),
		);
		for target_index in 0 ..< int(records.length)
		{
			target := collision_child_storage_header(records, target_index);
			if target.child_a != source.child_a
			{
				continue;
			}
			triangle, triangle_status := mesh_reduction_local_triangle(mesh_b, int(target.mesh_child_b));
			if triangle_status != .Ok
			{
				return triangle_status;
			}
			test_triangle, test_status := mesh_reduction_test_triangle_create(triangle);
			if test_status != .Ok
			{
				continue;
			}
			if mesh_reduction_should_block(test_triangle, mesh_contact, mesh_normal) == .Present
			{
				source_state_index := source_index * MAXIMUM_MANIFOLD_CONTACT_COUNT;
				target_state_index := target_index * MAXIMUM_MANIFOLD_CONTACT_COUNT;
				source_state := collision_candidate_storage_read(references, source_state_index);
				target_state := collision_candidate_storage_read(references, target_state_index);
				source_state.packed |= 1;
				target_state.packed |= 2;
				collision_candidate_storage_write(references, source_state_index, source_state);
				collision_candidate_storage_write(references, target_state_index, target_state);
				source.normal = util.quaternion_transform(
					util.vector3_negate(test_triangle.normals[0]), mesh_pose_b.orientation,
				);
				break;
			}
		}
	}
	for record_index in 0 ..< int(records.length)
	{
		record := collision_child_storage_header(records, record_index);
		state_index := record_index * MAXIMUM_MANIFOLD_CONTACT_COUNT;
		state := collision_candidate_storage_read(references, state_index).packed;
		if state & 1 != 0
		{
			if state & 2 == 0
			{
				record.contact_count = 0;
			}
			else
			{
				has_positive_depth := Reference_State.Missing;
				for contact_index in 0 ..< int(record.contact_count)
				{
					contact := collision_child_storage_read_contact(records, record_index, contact_index);
					if contact.depth > 0
					{
						has_positive_depth = .Present;
						break;
					}
				}
				if has_positive_depth != .Present
				{
					record.contact_count = 0;
				}
			}
		}
	}
	return .Ok;
}

collision_reduction_fast_remove_dynamic :: proc "contextless" (
	references: Collision_Reduction_Candidate_Storage, count: ^int, index: int,
)
{
	count^ -= 1;
	if index < count^
	{
		collision_candidate_storage_write(
			references, index, collision_candidate_storage_read(references, count^),
		);
	}
}

collision_reduction_use_record_contact :: proc "contextless" (
	records: Collision_Batcher_Child_Storage,
	references: Collision_Reduction_Candidate_Storage,
	remaining_count: ^int, remaining_index: int,
	result: ^Nonconvex_Contact_Manifold,
) -> Physics_Status
{
	contact := collision_reduction_record_contact(
		records, collision_candidate_storage_read(references, remaining_index),
	);
	status := nonconvex_manifold_append(result, contact);
	if status != .Ok
	{
		return status;
	}
	collision_reduction_fast_remove_dynamic(references, remaining_count, remaining_index);
	return .Ok;
}

collision_reduction_finish_records :: proc "contextless" (
	records: Collision_Batcher_Child_Storage,
	references: Collision_Reduction_Candidate_Storage,
	parent_position_a: util.Vector3,
	mesh_b: ^Mesh,
	mesh_pose_b: Rigid_Pose,
	result: ^Nonconvex_Contact_Manifold,
) -> Physics_Status
{
	if collision_child_storage_validate(records, int(records.length)) != .Ok ||
	references.memory == nil || result == nil || records.length < 0
	{
		return .Invalid_Argument;
	}
	prepare_status := collision_reduction_prepare_records(
		records, references, parent_position_a, mesh_b, mesh_pose_b,
	);
	if prepare_status != .Ok
	{
		return prepare_status;
	}
	contact_count := 0;
	for child_index in 0 ..< int(records.length)
	{
		record := collision_child_storage_header(records, child_index);
		child_contact_count := int(record.contact_count);
		if child_contact_count < 0 || child_contact_count > MAXIMUM_MANIFOLD_CONTACT_COUNT
		{
			return .Invalid_Description;
		}
		for contact_index in 0 ..< child_contact_count
		{
			if contact_count >= int(references.length)
			{
				return .Capacity_Missing;
			}
			collision_candidate_storage_write(
				references, contact_count,
				collision_reduction_candidate_reference(child_index, contact_index),
			);
			contact_count += 1;
		}
	}
	result.count = 0;
	if contact_count == 0
	{
		return .Ok;
	}
	if contact_count <= MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		for index in 0 ..< contact_count
		{
			status := nonconvex_manifold_append(
				result, collision_reduction_record_contact(
					records, collision_candidate_storage_read(references, index),
				),
			);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
	}
	extent_axis := util.Vector3{0.280454652, 0.558735445, 0.7804869574};
	minimum_extent := f32(math.F32_MAX);
	minimum_extent_position: util.Vector3;
	for index in 0 ..< contact_count
	{
		contact := collision_reduction_record_contact(
			records, collision_candidate_storage_read(references, index),
		);
		extent := util.vector3_dot(contact.offset, extent_axis);
		if extent < minimum_extent
		{
			minimum_extent = extent;
			minimum_extent_position = contact.offset;
		}
	}
	maximum_distance_squared: f32;
	for index in 0 ..< contact_count
	{
		contact := collision_reduction_record_contact(
			records, collision_candidate_storage_read(references, index),
		);
		distance_squared := util.vector3_length_squared(
			util.vector3_subtract(contact.offset, minimum_extent_position),
		);
		maximum_distance_squared = max(maximum_distance_squared, distance_squared);
	}
	maximum_distance := math.sqrt(maximum_distance_squared);
	extremity_scale := maximum_distance * 5e-3;
	initial_best_score := -f32(math.F32_MAX);
	initial_best_index := 0;
	for index in 0 ..< contact_count
	{
		contact := collision_reduction_record_contact(
			records, collision_candidate_storage_read(references, index),
		);
		candidate_score := contact.depth;
		if contact.depth >= 0
		{
			extent := util.vector3_dot(contact.offset, extent_axis) - minimum_extent;
			candidate_score += extent * extremity_scale;
		}
		if candidate_score > initial_best_score
		{
			initial_best_score = candidate_score;
			initial_best_index = index;
		}
	}
	remaining_count := contact_count;
	status := collision_reduction_use_record_contact(
		records, references, &remaining_count, initial_best_index, result,
	);
	if status != .Ok
	{
		return status;
	}
	distance_squared_interpolation_min := maximum_distance_squared * 1e-6;
	inverse_distance_squared_interpolation_span := f32(0);
	depth_scale := f32(0);
	if maximum_distance_squared > 0
	{
		inverse_distance_squared_interpolation_span = 1 / maximum_distance_squared;
	}
	if maximum_distance > 0
	{
		depth_scale = 400 / maximum_distance;
	}
	for remaining_count > 0 && result.count < MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		best_score := f32(-1);
		best_score_index := -1;
		index := 0;
		for index < remaining_count
		{
			candidate := collision_reduction_record_contact(
				records, collision_candidate_storage_read(references, index),
			);
			candidate_score := f32(math.F32_MAX);
			redundant := Reference_State.Missing;
			for reduced_index in 0 ..< int(result.count)
			{
				distinctiveness := collision_contact_distinctiveness(
					candidate, result.contacts[reduced_index],
					distance_squared_interpolation_min, inverse_distance_squared_interpolation_span, depth_scale,
				);
				if distinctiveness <= 0
				{
					redundant = .Present;
					break;
				}
				candidate_score = min(candidate_score, distinctiveness);
			}
			if redundant == .Present
			{
				collision_reduction_fast_remove_dynamic(references, &remaining_count, index);
				continue;
			}
			if candidate_score > best_score
			{
				best_score = candidate_score;
				best_score_index = index;
			}
			index += 1;
		}
		if best_score_index < 0
		{
			break;
		}
		status = collision_reduction_use_record_contact(
			records, references, &remaining_count, best_score_index, result,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

Mesh_Reduction_Test_Triangle :: struct
{
	anchors:             [4]util.Vector3,
	normals:             [4]util.Vector3,
	distance_threshold:  f32,
}

mesh_reduction_test_triangle_create :: proc "contextless" (
	triangle: Triangle,
) -> (Mesh_Reduction_Test_Triangle, Physics_Status)
{
	ab := util.vector3_subtract(triangle.b, triangle.a);
	bc := util.vector3_subtract(triangle.c, triangle.b);
	ca := util.vector3_subtract(triangle.a, triangle.c);
	n := util.vector3_cross(ab, ca);
	if util.vector3_length_squared(n) <= 1e-20
	{
		return {}, .Invalid_Description;
	}
	edge_ab := util.vector3_cross(n, ab);
	edge_bc := util.vector3_cross(n, bc);
	edge_ca := util.vector3_cross(n, ca);
	result := Mesh_Reduction_Test_Triangle{
		anchors={triangle.a, triangle.a, triangle.b, triangle.c},
		normals={n, edge_ab, edge_bc, edge_ca},
		distance_threshold=1e-3 * math.sqrt(max(
				util.vector3_length_squared(triangle.a) * 1e-4,
				max(util.vector3_length_squared(ab), util.vector3_length_squared(ca)),
		)),
	};
	for index in 0 ..< len(result.normals)
	{
		length_squared := util.vector3_length_squared(result.normals[index]);
		if length_squared <= 1e-20
		{
			return {}, .Invalid_Description;
		}
		result.normals[index] = util.vector3_scale(result.normals[index], 1 / math.sqrt(length_squared));
	}
	return result, .Ok;
}

mesh_reduction_should_block :: proc "contextless" (
	triangle: Mesh_Reduction_Test_Triangle, mesh_contact, mesh_normal: util.Vector3,
) -> Reference_State
{
	distances: [4]f32;
	for index in 0 ..< len(distances)
	{
		distances[index] = util.vector3_dot(
			util.vector3_subtract(mesh_contact, triangle.anchors[index]), triangle.normals[index],
		);
	}
	threshold := triangle.distance_threshold;
	if abs(distances[0]) > threshold || distances[1] > threshold ||
	distances[2] > threshold || distances[3] > threshold
	{
		return .Missing;
	}
	normal_dots: [4]f32;
	for index in 0 ..< len(normal_dots)
	{
		normal_dots[index] = util.vector3_dot(triangle.normals[index], mesh_normal);
	}
	if normal_dots[0] > -TRIANGLE_BACKFACE_REJECTION_THRESHOLD
	{
		return .Missing;
	}
	negative_threshold := threshold * -1e-2;
	on_edge: [3]Reference_State;
	if distances[1] >= negative_threshold
	{
		on_edge[0] = .Present;
	}
	if distances[2] >= negative_threshold
	{
		on_edge[1] = .Present;
	}
	if distances[3] >= negative_threshold
	{
		on_edge[2] = .Present;
	}
	if on_edge[0] == .Missing && on_edge[1] == .Missing && on_edge[2] == .Missing
	{
		return .Present;
	}
	primary_epsilon :: f32(1e-6);
	primary := on_edge[0] == .Present && normal_dots[1] > primary_epsilon ||
	on_edge[1] == .Present && normal_dots[2] > primary_epsilon ||
	on_edge[2] == .Present && normal_dots[3] > primary_epsilon;
	if !primary
	{
		return .Missing;
	}
	secondary_epsilon :: f32(-1e-2);
	if (on_edge[0] == .Missing || normal_dots[1] > secondary_epsilon) &&
	(on_edge[1] == .Missing || normal_dots[2] > secondary_epsilon) &&
	(on_edge[2] == .Missing || normal_dots[3] > secondary_epsilon)
	{
		return .Present;
	}
	return .Missing;
}

mesh_reduction_local_triangle :: proc "contextless" (
	mesh: ^Mesh, child_index: int,
) -> (Triangle, Physics_Status)
{
	if mesh == nil || mesh.triangles.memory == nil || child_index < 0 || child_index >= int(mesh.triangles.length)
	{
		return {}, .Invalid_Argument;
	}
	source := mesh.triangles.memory[child_index];
	return {
		a=util.vector3_multiply(source.a, mesh.scale),
		b=util.vector3_multiply(source.b, mesh.scale),
		c=util.vector3_multiply(source.c, mesh.scale),
	}, .Ok;
}

collision_compound_child_count :: proc "contextless" (shape: rawptr, type_id: int) -> (int, Physics_Status)
{
	if shape == nil
	{
		return 0, .Invalid_Argument;
	}
	switch type_id
	{
		case COMPOUND_TYPE_ID:
		compound := (^Compound)(shape);
		if compound.children.memory == nil
		{
			return 0, .Invalid_Description;
		}
		return int(compound.children.length), .Ok;
		case BIG_COMPOUND_TYPE_ID:
		compound := (^Big_Compound)(shape);
		if compound.children.memory == nil
		{
			return 0, .Invalid_Description;
		}
		return int(compound.children.length), .Ok;
		case MESH_TYPE_ID:
		mesh := (^Mesh)(shape);
		if mesh.triangles.memory == nil
		{
			return 0, .Invalid_Description;
		}
		return int(mesh.triangles.length), .Ok;
	}
	return 0, .Invalid_Argument;
}

collision_compound_child :: proc "contextless" (
	shape: rawptr, type_id, child_index: int, parent_pose: Rigid_Pose, shapes: ^Shape_Registry,
) -> (Collision_Child, Physics_Status)
{
	if shape == nil || shapes == nil || child_index < 0 || type_id == MESH_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	child: Compound_Child;
	switch type_id
	{
		case COMPOUND_TYPE_ID:
		compound := (^Compound)(shape);
		if child_index >= int(compound.children.length)
		{
			return {}, .Invalid_Argument;
		}
		child = compound.children.memory[child_index];
		case BIG_COMPOUND_TYPE_ID:
		compound := (^Big_Compound)(shape);
		if child_index >= int(compound.children.length)
		{
			return {}, .Invalid_Argument;
		}
		child = compound.children.memory[child_index];
		case:
		return {}, .Invalid_Argument;
	}
	child_shape, child_batch, status := shape_registry_resolve(shapes, child.shape_index);
	if status != .Ok
	{
		return {}, status;
	}
	child_type := int(typed_index_type(child.shape_index));
	if child_batch == nil || child_batch.state != .Registered ||
	child_batch.metadata.batch_type != .Convex || shape_batch_bounds_state(child_batch) == .Missing
	{
		return {}, .Invalid_Description;
	}
	local_pose := Rigid_Pose{orientation=child.local_orientation, position=child.local_position};
	return {
		shape=child_shape, type_id=child_type,
		pose=rigid_pose_concatenate(local_pose, parent_pose), child_index=child_index,
	}, .Ok;
}

collision_mesh_child :: proc "contextless" (
	mesh: ^Mesh, child_index: int, parent_pose: Rigid_Pose,
) -> (Triangle, Rigid_Pose, Physics_Status)
{
	if mesh == nil || mesh.triangles.memory == nil || child_index < 0 || child_index >= int(mesh.triangles.length)
	{
		return {}, {}, .Invalid_Argument;
	}
	source := mesh.triangles.memory[child_index];
	triangle := Triangle{
		a=util.vector3_multiply(source.a, mesh.scale),
		b=util.vector3_multiply(source.b, mesh.scale),
		c=util.vector3_multiply(source.c, mesh.scale),
	};
	center := util.vector3_scale(util.vector3_add(triangle.a, util.vector3_add(triangle.b, triangle.c)), 1.0 / 3.0);
	triangle.a = util.vector3_subtract(triangle.a, center);
	triangle.b = util.vector3_subtract(triangle.b, center);
	triangle.c = util.vector3_subtract(triangle.c, center);
	local_pose := Rigid_Pose{orientation=util.quaternion_identity(), position=center};
	return triangle, rigid_pose_concatenate(local_pose, parent_pose), .Ok;
}

collision_tree_for_shape :: proc "contextless" (shape: rawptr, type_id: int) -> (^Tree, Physics_Status)
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Collision_Batcher phase=compound_traversal reason=checked_shape_tree_reference lifetime=until_shape_storage_mutation_or_disposal
	if shape == nil
	{
		return nil, .Invalid_Argument;
	}
	switch type_id
	{
		case BIG_COMPOUND_TYPE_ID:
		return &(^Big_Compound)(shape).tree, .Ok;
		case MESH_TYPE_ID:
		return &(^Mesh)(shape).tree, .Ok;
	}
	return nil, .Not_Found;
}

collision_child_bounds_in_parent_space :: proc "contextless" (
	child: Collision_Child, parent_pose_a, parent_pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity, dt, maximum_expansion: f32,
	shapes: ^Shape_Registry,
) -> (util.Bounding_Box, Physics_Status)
{
	if child.shape == nil || shapes == nil || child.type_id < 0 ||
	child.type_id >= shapes.registered_type_count || dt < 0 || maximum_expansion < 0
	{
		return {}, .Invalid_Argument;
	}
	child_batch := &shapes.batches[child.type_id];
	if child_batch.state != .Registered || child_batch.metadata.batch_type != .Convex ||
	shape_batch_bounds_state(child_batch) == .Missing
	{
		return {}, .Invalid_Description;
	}
	relative_pose := rigid_pose_concatenate(child.pose, rigid_pose_invert(parent_pose_b));
	bounds: Shape_Bounds;
	status: Physics_Status;
	bounds, status = shape_batch_compute_bounds(child_batch, child.shape, relative_pose.orientation, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	child_pose_in_a := rigid_pose_concatenate(child.pose, rigid_pose_invert(parent_pose_a));
	radius_a := util.vector3_length(child_pose_in_a.position);
	local_relative_linear := util.quaternion_transform(
		util.vector3_subtract(velocity_a.linear, velocity_b.linear),
		util.quaternion_conjugate(parent_pose_b.orientation),
	);
	min_expansion, max_expansion := broad_phase_motion_bounds_expansion(
		local_relative_linear, velocity_a.angular, dt,
		bounds.maximum_radius + radius_a, bounds.maximum_angular_expansion + radius_a, maximum_expansion,
	);
	worst_case_radius := util.vector3_length(relative_pose.position) + util.vector3_length(local_relative_linear) * dt;
	angular_expansion_b := broad_phase_angular_bounds_expansion(
		util.vector3_length(velocity_b.angular), dt,
		bounds.maximum_radius + worst_case_radius, bounds.maximum_angular_expansion + worst_case_radius,
	);
	angular_b := util.Vector3{angular_expansion_b, angular_expansion_b, angular_expansion_b};
	min_expansion = util.vector3_max(
		util.Vector3{-maximum_expansion, -maximum_expansion, -maximum_expansion},
		util.vector3_subtract(min_expansion, angular_b),
	);
	max_expansion = util.vector3_min(
		util.Vector3{maximum_expansion, maximum_expansion, maximum_expansion},
		util.vector3_add(max_expansion, angular_b),
	);
	return {
		min=util.vector3_add(relative_pose.position, util.vector3_add(bounds.min, min_expansion)),
		max=util.vector3_add(relative_pose.position, util.vector3_add(bounds.max, max_expansion)),
	}, .Ok;
}

collision_batcher_queue_child_pair :: proc "contextless" (
	batcher: ^Collision_Batcher, continuation_index: int,
	child_a, child_b: Collision_Child,
	parent_pose_a: Rigid_Pose,
	report_child_a, report_child_b: int,
) -> Physics_Status
{
	if batcher == nil || continuation_index < 0 || continuation_index >= batcher.continuation_count ||
	child_a.shape == nil || child_b.shape == nil
	{
		return .Invalid_Argument;
	}
	if batcher.child_count >= int(batcher.children.length)
	{
		return .Capacity_Missing;
	}
	continuation := &batcher.continuations.memory[continuation_index];
	record_index := batcher.child_count;
	record := collision_child_storage_header(batcher.children, record_index);
	collision_child_storage_offset_a(batcher.children, record_index)^ =
	util.vector3_subtract(child_a.pose.position, parent_pose_a.position);
	record^ = {
		child_a=i32(report_child_a),
		child_b=i32(report_child_b),
		mesh_child_b=i32(child_b.child_index),
	};
	batcher.child_count += 1;
	continuation.child_count += 1;
	if batcher.procedures.allow_child_pair != nil &&
	batcher.procedures.allow_child_pair(
		batcher.user_context, batcher.pairs.memory[continuation.parent_pair_index].pair_id,
		i32(report_child_a), i32(report_child_b),
	) != .Allow
	{
		record.completed = .Present;
		return .Ok;
	}
	task, reference, lookup_status := collision_task_registry_lookup(
		batcher.tasks, child_a.type_id, child_b.type_id,
	);
	if lookup_status == .Not_Found
	{
		return collision_batcher_complete_child_record(
			batcher, continuation_index, record_index, {},
		);
	}
	if lookup_status != .Ok
	{
		return lookup_status;
	}
	if task.kind != .Convex || collision_task_wide_test_state(task) == .Missing
	{
		return .Invalid_Description;
	}
	if batcher.subpair_count >= int(batcher.children.length)
	{
		return .Capacity_Missing;
	}
	source_child_a := i32(child_a.child_index);
	if reference.order == .Flipped
	{
		source_child_a = i32(u32(source_child_a) | (u32(1) << 31));
	}
	subpair_index := i32(batcher.subpair_count);
	subpair := Collision_Batcher_Subpair{
		continuation_index=i32(continuation_index),
		child_record_index=i32(record_index),
		source_child_a=source_child_a,
		source_child_b=i32(child_b.child_index),
	};
	collision_child_storage_subpair(batcher.children, batcher.subpair_count)^ = subpair;
	collision_child_storage_next(batcher.children, batcher.subpair_count)^ = -1;
	if batcher.subtask_tails[reference.task_id] >= 0
	{
		collision_child_storage_next(
			batcher.children, int(batcher.subtask_tails[reference.task_id]),
		)^ = subpair_index;
	}
	else
	{
		batcher.subtask_heads[reference.task_id] = subpair_index;
	}
	batcher.subtask_tails[reference.task_id] = subpair_index;
	batcher.subpair_count += 1;
	return .Ok;
}

Collision_Batcher_Child_Tree_Context :: struct
{
	batcher:              ^Collision_Batcher,
	continuation_index:   int,
	child_a:              Collision_Child,
	shape_b:              rawptr,
	type_b:               int,
	pose_b:               Rigid_Pose,
	parent_pose_a:        Rigid_Pose,
	parent_flipped:       Reference_State,
}

collision_batcher_child_tree_leaf :: proc "contextless" (
	user_context: rawptr, leaf_index: int,
) -> Physics_Status
{
	query_context := (^Collision_Batcher_Child_Tree_Context)(user_context);
	if query_context == nil || leaf_index < 0
	{
		return .Invalid_Argument;
	}
	triangle_b: Triangle;
	child_b: Collision_Child;
	if query_context.type_b == MESH_TYPE_ID
	{
		triangle, child_pose, status := collision_mesh_child(
			(^Mesh)(query_context.shape_b), leaf_index, query_context.pose_b,
		);
		if status != .Ok
		{
			return status;
		}
		triangle_b = triangle;
		child_b = {
			shape=&triangle_b,
			type_id=TRIANGLE_TYPE_ID,
			pose=child_pose,
			child_index=leaf_index,
			mesh_owner=(^Mesh)(query_context.shape_b),
			mesh_pose=query_context.pose_b,
		};
	}
	else
	{
		child, status := collision_compound_child(
			query_context.shape_b, query_context.type_b, leaf_index, query_context.pose_b,
			query_context.batcher.shapes,
		);
		if status != .Ok
		{
			return status;
		}
		child_b = child;
	}
	report_child_a := query_context.child_a.child_index;
	report_child_b := child_b.child_index;
	if query_context.parent_flipped == .Present
	{
		report_child_a, report_child_b = report_child_b, report_child_a;
	}
	return collision_batcher_queue_child_pair(
		query_context.batcher, query_context.continuation_index,
		query_context.child_a, child_b, query_context.parent_pose_a,
		report_child_a, report_child_b,
	);
}

collision_batcher_test_child_against_tree :: proc "contextless" (
	batcher: ^Collision_Batcher, continuation_index: int,
	child_a: Collision_Child,
	shape_b: rawptr, type_b: int, pose_b, parent_pose_a: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity, dt, maximum_expansion: f32,
	parent_flipped: Reference_State,
) -> Physics_Status
{
	tree, tree_status := collision_tree_for_shape(shape_b, type_b);
	if tree_status != .Ok
	{
		return tree_status;
	}
	query_bounds, bounds_status := collision_child_bounds_in_parent_space(
		child_a, parent_pose_a, pose_b, velocity_a, velocity_b, dt, maximum_expansion, batcher.shapes,
	);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	query_context := Collision_Batcher_Child_Tree_Context{
		batcher=batcher,
		continuation_index=continuation_index,
		child_a=child_a,
		shape_b=shape_b,
		type_b=type_b,
		pose_b=pose_b,
		parent_pose_a=parent_pose_a,
		parent_flipped=parent_flipped,
	};
	return tree_volume_traverse(
		tree, query_bounds, collision_batcher_child_tree_leaf, &query_context,
		batcher.traversal_pool,
	);
}

collision_batcher_for_each_child_pair :: proc "contextless" (
	batcher: ^Collision_Batcher, continuation_index: int,
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity, dt, maximum_expansion: f32,
	parent_flipped: Reference_State,
) -> Physics_Status
{
	count_a, count_a_status := collision_compound_child_count(shape_a, type_a);
	if count_a_status != .Ok
	{
		return count_a_status;
	}
	count_b, count_b_status := collision_compound_child_count(shape_b, type_b);
	if count_b_status != .Ok
	{
		return count_b_status;
	}
	for index_a in 0 ..< count_a
	{
		triangle_a: Triangle;
		child_a: Collision_Child;
		if type_a == MESH_TYPE_ID
		{
			triangle, child_pose, status := collision_mesh_child((^Mesh)(shape_a), index_a, pose_a);
			if status != .Ok
			{
				return status;
			}
			triangle_a = triangle;
			child_a = {
				shape=&triangle_a,
				type_id=TRIANGLE_TYPE_ID,
				pose=child_pose,
				child_index=index_a,
				mesh_owner=(^Mesh)(shape_a),
				mesh_pose=pose_a,
			};
		}
		else
		{
			child, status := collision_compound_child(shape_a, type_a, index_a, pose_a, batcher.shapes);
			if status != .Ok
			{
				return status;
			}
			child_a = child;
		}
		if type_b == BIG_COMPOUND_TYPE_ID || type_b == MESH_TYPE_ID
		{
			status := collision_batcher_test_child_against_tree(
				batcher, continuation_index, child_a, shape_b, type_b, pose_b, pose_a,
				velocity_a, velocity_b, dt, maximum_expansion, parent_flipped,
			);
			if status != .Ok
			{
				return status;
			}
			continue;
		}
		for index_b in 0 ..< count_b
		{
			triangle_b: Triangle;
			child_b: Collision_Child;
			if type_b == MESH_TYPE_ID
			{
				triangle, child_pose, status := collision_mesh_child((^Mesh)(shape_b), index_b, pose_b);
				if status != .Ok
				{
					return status;
				}
				triangle_b = triangle;
				child_b = {
					shape=&triangle_b,
					type_id=TRIANGLE_TYPE_ID,
					pose=child_pose,
					child_index=index_b,
					mesh_owner=(^Mesh)(shape_b),
					mesh_pose=pose_b,
				};
			}
			else
			{
				child, status := collision_compound_child(shape_b, type_b, index_b, pose_b, batcher.shapes);
				if status != .Ok
				{
					return status;
				}
				child_b = child;
			}
			report_child_a := child_a.child_index;
			report_child_b := child_b.child_index;
			if parent_flipped == .Present
			{
				report_child_a, report_child_b = report_child_b, report_child_a;
			}
			status := collision_batcher_queue_child_pair(
				batcher, continuation_index, child_a, child_b, pose_a,
				report_child_a, report_child_b,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

collision_batcher_convex_against_children :: proc "contextless" (
	batcher: ^Collision_Batcher, continuation_index: int,
	shape_a: rawptr, type_a: int, pose_a: Rigid_Pose,
	shape_b: rawptr, type_b: int, pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity, dt, maximum_expansion: f32,
	parent_flipped: Reference_State,
) -> Physics_Status
{
	count_b, count_status := collision_compound_child_count(shape_b, type_b);
	if count_status != .Ok
	{
		return count_status;
	}
	child_a := Collision_Child{shape=shape_a, type_id=type_a, pose=pose_a};
	if type_b == BIG_COMPOUND_TYPE_ID || type_b == MESH_TYPE_ID
	{
		return collision_batcher_test_child_against_tree(
			batcher, continuation_index, child_a, shape_b, type_b, pose_b, pose_a,
			velocity_a, velocity_b, dt, maximum_expansion, parent_flipped,
		);
	}
	for index_b in 0 ..< count_b
	{
		child_b, status := collision_compound_child(shape_b, type_b, index_b, pose_b, batcher.shapes);
		if status != .Ok
		{
			return status;
		}
		report_child_a := child_a.child_index;
		report_child_b := child_b.child_index;
		if parent_flipped == .Present
		{
			report_child_a, report_child_b = report_child_b, report_child_a;
		}
		status = collision_batcher_queue_child_pair(
			batcher, continuation_index, child_a, child_b, pose_a,
			report_child_a, report_child_b,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

collision_batcher_expand_compound_pair :: proc "contextless" (
	batcher: ^Collision_Batcher, pair_index: int,
) -> Physics_Status
{
	if batcher == nil || pair_index < 0 || pair_index >= batcher.pair_count ||
	batcher.continuation_count >= int(batcher.continuations.length)
	{
		return .Capacity_Missing;
	}
	pair := &batcher.pairs.memory[pair_index];
	original_type_a := int(typed_index_type(pair.input.shape_a));
	original_type_b := int(typed_index_type(pair.input.shape_b));
	task, reference, lookup_status := collision_task_registry_lookup(
		batcher.tasks, original_type_a, original_type_b,
	);
	if lookup_status != .Ok
	{
		return lookup_status;
	}
	if task.kind == .Convex
	{
		return .Invalid_Description;
	}
	shape_a, batch_a, resolve_a_status := shape_registry_resolve(
		batcher.shapes, pair.input.shape_a,
	);
	if resolve_a_status != .Ok
	{
		return resolve_a_status;
	}
	shape_b, batch_b, resolve_b_status := shape_registry_resolve(
		batcher.shapes, pair.input.shape_b,
	);
	if resolve_b_status != .Ok
	{
		return resolve_b_status;
	}
	type_a, type_b := original_type_a, original_type_b;
	pose_a, pose_b := pair.input.pose_a, pair.input.pose_b;
	velocity_a, velocity_b := pair.input.velocity_a, pair.input.velocity_b;
	dt := pair.input.dt;
	maximum_expansion := pair.input.maximum_expansion;
	parent_flipped := Reference_State.Missing;
	if reference.order == .Flipped
	{
		shape_a, shape_b = shape_b, shape_a;
		batch_a, batch_b = batch_b, batch_a;
		type_a, type_b = type_b, type_a;
		pose_a, pose_b = pose_b, pose_a;
		velocity_a, velocity_b = velocity_b, velocity_a;
		parent_flipped = .Present;
	}
	switch task.kind
	{
		case .Convex_Compound:
		if batch_a == nil || batch_b == nil ||
		batch_a.metadata.batch_type != .Convex ||
		(batch_b.metadata.batch_type != .Compound &&
			batch_b.metadata.batch_type != .Homogeneous_Compound) ||
		type_b < COMPOUND_TYPE_ID || type_b > MESH_TYPE_ID
		{
			return .Invalid_Description;
		}
		case .Compound_Pair:
		if batch_a == nil || batch_b == nil ||
		(batch_a.metadata.batch_type != .Compound &&
			batch_a.metadata.batch_type != .Homogeneous_Compound) ||
		(batch_b.metadata.batch_type != .Compound &&
			batch_b.metadata.batch_type != .Homogeneous_Compound) ||
		type_a < COMPOUND_TYPE_ID || type_a > MESH_TYPE_ID ||
		type_b < COMPOUND_TYPE_ID || type_b > MESH_TYPE_ID
		{
			return .Invalid_Description;
		}
		case .Convex:
		return .Invalid_Description;
	}
	continuation_index := batcher.continuation_count;
	batcher.continuations.memory[continuation_index] = {
		parent_pair_index=i32(pair_index),
		child_start=i32(batcher.child_count),
		offset_b=util.vector3_subtract(pose_b.position, pose_a.position),
		parent_position_a=pose_a.position,
		flip=parent_flipped,
	};
	if type_b == MESH_TYPE_ID
	{
		batcher.continuations.memory[continuation_index].mesh_b = (^Mesh)(shape_b);
		batcher.continuations.memory[continuation_index].mesh_pose_b = pose_b;
	}
	batcher.continuation_count += 1;
	switch task.kind
	{
		case .Convex_Compound:
		return #force_inline collision_batcher_convex_against_children(
			batcher, continuation_index,
			shape_a, type_a, pose_a, shape_b, type_b, pose_b,
			velocity_a, velocity_b, dt, maximum_expansion, parent_flipped,
		);
		case .Compound_Pair:
		return #force_inline collision_batcher_for_each_child_pair(
			batcher, continuation_index,
			shape_a, shape_b, type_a, type_b, pose_a, pose_b,
			velocity_a, velocity_b, dt, maximum_expansion, parent_flipped,
		);
		case .Convex:
		return .Invalid_Description;
	}
	return .Invalid_Description;
}

collision_batcher_finish_continuation :: proc "contextless" (
	batcher: ^Collision_Batcher, continuation_index: int,
) -> Physics_Status
{
	if batcher == nil || continuation_index < 0 || continuation_index >= batcher.continuation_count
	{
		return .Invalid_Argument;
	}
	continuation := &batcher.continuations.memory[continuation_index];
	start := int(continuation.child_start);
	count := int(continuation.child_count);
	if start < 0 || count < 0 || start > batcher.child_count - count
	{
		return .Invalid_Description;
	}
	for child_index in start ..< start + count
	{
		if collision_child_storage_header(batcher.children, child_index).completed != .Present
		{
			return .Invalid_Description;
		}
	}
	records := batcher.children;
	records.length = i32(count);
	if count > 0
	{
		slice_status: Physics_Status;
		records, slice_status = collision_child_storage_record_slice(batcher.children, start, count);
		if slice_status != .Ok
		{
			return slice_status;
		}
	}
	result := Nonconvex_Contact_Manifold{offset_b=continuation.offset_b};
	status := #force_inline collision_reduction_finish_records(
		records, batcher.candidate_refs,
		continuation.parent_position_a, continuation.mesh_b, continuation.mesh_pose_b, &result,
	);
	if status != .Ok
	{
		return status;
	}
	if continuation.flip == .Present
	{
		result = nonconvex_manifold_flip(result);
	}
	parent := &batcher.pairs.memory[continuation.parent_pair_index];
	parent.result = {kind=.Nonconvex, nonconvex=result};
	parent.result_state = .Complete;
	return .Ok;
}
