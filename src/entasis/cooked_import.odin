package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// Compound_Child is the exact low-level compound child representation. shape
// handles are retained by the registry when a compound is imported
Compound_Child :: physics.Compound_Child;

// compound_child creates one compound child from a registered shape and local
// pose. the returned value owns no resources
compound_child :: #force_inline proc "contextless" (
	shape: Shape_Handle,
	local_pose: Rigid_Pose = {orientation={0, 0, 0, 1}},
) -> Compound_Child
{
	return {
		local_position=local_pose.position,
		local_orientation=local_pose.orientation,
		shape_index=shape,
	};
}

@(private)
shape_import_copy_buffer :: proc (
	pool: ^util.Buffer_Pool,
	source: util.Buffer($T),
) -> (util.Buffer(T), Status)
{
	if pool == nil || source.memory == nil || source.length <= 0
	{
		return {}, .Invalid_Description;
	}
	target, take_status := util.buffer_pool_take_at_least(pool, T, int(source.length));
	if take_status != .Ok
	{
		return {}, world_memory_status(take_status);
	}
	copy_status := util.buffer_copy(
		util.buffer_view(source), 0, util.buffer_view(target), 0, int(source.length),
	);
	if copy_status != .Ok
	{
		_ = util.buffer_pool_return(pool, &target);
		return {}, world_memory_status(copy_status);
	}
	target.length = source.length;
	return target, .Ok;
}

@(private)
shape_import_release_tree_buffers :: proc (
	pool: ^util.Buffer_Pool,
	tree: ^physics.Tree,
)
{
	if pool == nil || tree == nil
	{
		return;
	}
	if tree.build_scratch.memory != nil
	{
		_ = util.buffer_pool_return(pool, &tree.build_scratch);
	}
	if tree.build_references.memory != nil
	{
		_ = util.buffer_pool_return(pool, &tree.build_references);
	}
	if tree.leaves.memory != nil
	{
		_ = util.buffer_pool_return(pool, &tree.leaves);
	}
	if tree.metanodes.memory != nil
	{
		_ = util.buffer_pool_return(pool, &tree.metanodes);
	}
	if tree.nodes.memory != nil
	{
		_ = util.buffer_pool_return(pool, &tree.nodes);
	}
	tree^ = {};
}

@(private)
shape_import_copy_tree :: proc (
	pool: ^util.Buffer_Pool,
	source: ^physics.Tree,
) -> (physics.Tree, Status)
{
	if pool == nil || source == nil || source.state != .Ready ||
		source.nodes.memory == nil || source.metanodes.memory == nil ||
		source.leaves.memory == nil || source.build_references.memory == nil ||
		source.build_scratch.memory == nil
	{
		return {}, .Invalid_Description;
	}
	target: physics.Tree;
	status: Status;
	target.nodes, status = shape_import_copy_buffer(pool, source.nodes);
	if status != .Ok
	{
		return {}, status;
	}
	target.metanodes, status = shape_import_copy_buffer(pool, source.metanodes);
	if status != .Ok
	{
		shape_import_release_tree_buffers(pool, &target);
		return {}, status;
	}
	target.leaves, status = shape_import_copy_buffer(pool, source.leaves);
	if status != .Ok
	{
		shape_import_release_tree_buffers(pool, &target);
		return {}, status;
	}
	target.build_references, status = shape_import_copy_buffer(pool, source.build_references);
	if status != .Ok
	{
		shape_import_release_tree_buffers(pool, &target);
		return {}, status;
	}
	target.build_scratch, status = shape_import_copy_buffer(pool, source.build_scratch);
	if status != .Ok
	{
		shape_import_release_tree_buffers(pool, &target);
		return {}, status;
	}
	target.node_count = source.node_count;
	target.leaf_count = source.leaf_count;
	target.refinement_frame = source.refinement_frame;
	target.pool = pool;
	target.state = .Ready;
	return target, .Ok;
}

@(private)
shape_import_copy_hull :: proc (
	pool: ^util.Buffer_Pool,
	source: ^physics.Convex_Hull,
) -> (physics.Convex_Hull, Status)
{
	if pool == nil || physics.convex_hull_validate(source) != .Ok
	{
		return {}, .Invalid_Description;
	}
	target: physics.Convex_Hull;
	status: Status;
	target.points, status = shape_import_copy_buffer(pool, source.points);
	if status != .Ok
	{
		return {}, status;
	}
	target.bounding_planes, status = shape_import_copy_buffer(pool, source.bounding_planes);
	if status != .Ok
	{
		_ = util.buffer_pool_return(pool, &target.points);
		return {}, status;
	}
	target.face_start_indices, status = shape_import_copy_buffer(pool, source.face_start_indices);
	if status != .Ok
	{
		_ = util.buffer_pool_return(pool, &target.bounding_planes);
		_ = util.buffer_pool_return(pool, &target.points);
		return {}, status;
	}
	target.face_vertex_indices, status = shape_import_copy_buffer(pool, source.face_vertex_indices);
	if status != .Ok
	{
		_ = util.buffer_pool_return(pool, &target.face_start_indices);
		_ = util.buffer_pool_return(pool, &target.bounding_planes);
		_ = util.buffer_pool_return(pool, &target.points);
		return {}, status;
	}
	target.point_count = source.point_count;
	return target, .Ok;
}

@(private)
shape_import_copy_mesh :: proc (
	pool: ^util.Buffer_Pool,
	source: ^physics.Mesh,
) -> (physics.Mesh, Status)
{
	if pool == nil || source == nil || source.triangles.memory == nil ||
		source.triangles.length <= 0 || source.triangle_bounds.memory == nil ||
		source.tree.state != .Ready
	{
		return {}, .Invalid_Description;
	}
	target: physics.Mesh;
	status: Status;
	target.triangles, status = shape_import_copy_buffer(pool, source.triangles);
	if status != .Ok
	{
		return {}, status;
	}
	target.triangle_bounds, status = shape_import_copy_buffer(pool, source.triangle_bounds);
	if status != .Ok
	{
		_ = util.buffer_pool_return(pool, &target.triangles);
		return {}, status;
	}
	target.tree, status = shape_import_copy_tree(pool, &source.tree);
	if status != .Ok
	{
		_ = util.buffer_pool_return(pool, &target.triangle_bounds);
		_ = util.buffer_pool_return(pool, &target.triangles);
		return {}, status;
	}
	target.scale = source.scale;
	return target, .Ok;
}

// shape_import_convex_hull copies a fully cooked hull into the world's pool and
// registers it without rebuilding hull topology. the source remains caller owned
shape_import_convex_hull :: proc (
	world: ^World,
	source: ^physics.Convex_Hull,
) -> (Shape_Handle, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready
	{
		return shape_handle_invalid(), .Disposed;
	}
	copy, copy_status := shape_import_copy_hull(data.pool, source);
	if copy_status != .Ok
	{
		return shape_handle_invalid(), copy_status;
	}
	handle, add_status := physics.shape_registry_add(
		physics.simulation_shape_registry(&data.simulation),
		physics.CONVEX_HULL_TYPE_ID,
		&copy,
	);
	if add_status != .Ok
	{
		_ = physics.convex_hull_dispose(&copy, data.pool);
		return shape_handle_invalid(), add_status;
	}
	return handle, .Ok;
}

// shape_import_mesh copies a fully cooked mesh, including its acceleration tree,
// into the world's pool and registers it without rebuilding the tree
shape_import_mesh :: proc (
	world: ^World,
	source: ^physics.Mesh,
) -> (Shape_Handle, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready
	{
		return shape_handle_invalid(), .Disposed;
	}
	copy, copy_status := shape_import_copy_mesh(data.pool, source);
	if copy_status != .Ok
	{
		return shape_handle_invalid(), copy_status;
	}
	handle, add_status := physics.shape_registry_add(
		physics.simulation_shape_registry(&data.simulation),
		physics.MESH_TYPE_ID,
		&copy,
	);
	if add_status != .Ok
	{
		_ = physics.mesh_dispose(&copy, data.pool);
		return shape_handle_invalid(), add_status;
	}
	return handle, .Ok;
}

// shape_import_compound copies caller-owned child descriptors into the world's
// pool and registers one compound. child shapes are retained by the registry
shape_import_compound :: proc (
	world: ^World,
	children: []Compound_Child,
) -> (Shape_Handle, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready
	{
		return shape_handle_invalid(), .Disposed;
	}
	if len(children) <= 0
	{
		return shape_handle_invalid(), .Invalid_Description;
	}
	compound: physics.Compound;
	create_status := physics.compound_create(
		&compound, raw_data(children), len(children), data.pool,
	);
	if create_status != .Ok
	{
		return shape_handle_invalid(), create_status;
	}
	handle, add_status := physics.shape_registry_add(
		physics.simulation_shape_registry(&data.simulation),
		physics.COMPOUND_TYPE_ID,
		&compound,
	);
	if add_status != .Ok
	{
		_ = physics.compound_dispose(&compound, data.pool);
		return shape_handle_invalid(), add_status;
	}
	return handle, .Ok;
}

// shape_import_big_compound copies caller-owned child descriptors, builds the
// internal child tree in the world's pool, and registers one Big_Compound
shape_import_big_compound :: proc (
	world: ^World,
	children: []Compound_Child,
) -> (Shape_Handle, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready
	{
		return shape_handle_invalid(), .Disposed;
	}
	if len(children) <= 0
	{
		return shape_handle_invalid(), .Invalid_Description;
	}
	compound: physics.Big_Compound;
	create_status := physics.big_compound_create(
		&compound,
		raw_data(children),
		len(children),
		physics.simulation_shape_registry(&data.simulation),
		data.pool,
	);
	if create_status != .Ok
	{
		return shape_handle_invalid(), create_status;
	}
	handle, add_status := physics.shape_registry_add(
		physics.simulation_shape_registry(&data.simulation),
		physics.BIG_COMPOUND_TYPE_ID,
		&compound,
	);
	if add_status != .Ok
	{
		_ = physics.big_compound_dispose(&compound, data.pool);
		return shape_handle_invalid(), add_status;
	}
	return handle, .Ok;
}

// shape_import_big_compound_cooked imports a prebuilt Big_Compound tree. the
// source buffers remain caller owned and are copied into the world's pool
shape_import_big_compound_cooked :: proc (
	world: ^World,
	children: []Compound_Child,
	child_bounds: []Bounding_Box,
	tree: ^physics.Tree,
) -> (Shape_Handle, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready
	{
		return shape_handle_invalid(), .Disposed;
	}
	if len(children) <= 0 || len(children) != len(child_bounds) || tree == nil ||
		tree.state != .Ready || tree.leaf_count != len(children)
	{
		return shape_handle_invalid(), .Invalid_Description;
	}
	copy: physics.Big_Compound;
	status: Status;
	copy.children, status = shape_import_copy_buffer(
		data.pool,
		physics.Compound{children={memory=raw_data(children), length=i32(len(children)), id=-1}}.children,
	);
	if status != .Ok
	{
		return shape_handle_invalid(), status;
	}
	copy.child_bounds, status = shape_import_copy_buffer(
		data.pool,
		util.Buffer(Bounding_Box){memory=raw_data(child_bounds), length=i32(len(child_bounds)), id=-1},
	);
	if status != .Ok
	{
		_ = util.buffer_pool_return(data.pool, &copy.children);
		return shape_handle_invalid(), status;
	}
	copy.tree, status = shape_import_copy_tree(data.pool, tree);
	if status != .Ok
	{
		_ = util.buffer_pool_return(data.pool, &copy.child_bounds);
		_ = util.buffer_pool_return(data.pool, &copy.children);
		return shape_handle_invalid(), status;
	}
	handle, add_status := physics.shape_registry_add(
		physics.simulation_shape_registry(&data.simulation),
		physics.BIG_COMPOUND_TYPE_ID,
		&copy,
	);
	if add_status != .Ok
	{
		_ = physics.big_compound_dispose(&copy, data.pool);
		return shape_handle_invalid(), add_status;
	}
	return handle, .Ok;
}
