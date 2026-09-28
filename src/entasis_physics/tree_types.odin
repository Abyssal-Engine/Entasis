// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

TREE_LEAF_NODE_MASK :: u32(0x7fff_ffff);
TREE_LEAF_CHILD_MASK :: u32(0x8000_0000);
TREE_BIN_MIN_COUNT :: 16;
TREE_BIN_MAX_COUNT :: 64;
TREE_BIN_LEAF_MULTIPLIER :: f32(1.0 / 16.0);
TREE_MICROSWEEP_THRESHOLD :: 64;

Tree_Leaf :: struct
{
	packed: u32,
}

tree_leaf_create :: proc "contextless" (node_index, child_index: int) -> Tree_Leaf
{
	return {u32(node_index) & TREE_LEAF_NODE_MASK | u32(child_index) << 31};
}

tree_leaf_node_index :: proc "contextless" (leaf: Tree_Leaf) -> int
{
	return int(leaf.packed & TREE_LEAF_NODE_MASK);
}

tree_leaf_child_index :: proc "contextless" (leaf: Tree_Leaf) -> int
{
	return int((leaf.packed & TREE_LEAF_CHILD_MASK) >> 31);
}

Tree_Node_Child :: struct
{
	min:        util.Vector3,
	index:      i32,
	max:        util.Vector3,
	leaf_count: i32,
}

Tree_Node :: struct
{
	a: Tree_Node_Child,
	b: Tree_Node_Child,
}

Tree_Metanode :: struct
{
	parent:          i32,
	index_in_parent: i32,
	refine_or_cost:  u32,
}

Tree_Build_Reference :: struct
{
	bounds:     util.Bounding_Box,
	leaf_index: i32,
}

Tree_State :: enum u8
{
	Unallocated,
	Ready,
	Disposed,
}

Tree :: struct
{
	nodes:            util.Buffer(Tree_Node),
	metanodes:        util.Buffer(Tree_Metanode),
	leaves:           util.Buffer(Tree_Leaf),
	build_references: util.Buffer(Tree_Build_Reference),
	build_scratch:    util.Buffer(Tree_Build_Reference),
	node_count:       int,
	leaf_count:       int,
	refinement_frame: u32,
	pool:             ^util.Buffer_Pool,
	state:            Tree_State,
}

Tree_Remove_Result :: struct
{
	moved_leaf_original_index: i32,
	moved_leaf_new_index:      i32,
}

Tree_Leaf_Pair :: struct
{
	a, b: i32,
}

tree_encode_leaf :: proc "contextless" (leaf_index: int) -> i32
{
	return i32(-1 - leaf_index);
}
tree_decode_leaf :: proc "contextless" (encoded_leaf: i32) -> int
{
	return int(-1 - encoded_leaf);
}

tree_child :: proc "contextless" (node: ^Tree_Node, child_index: int) -> ^Tree_Node_Child
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Tree phase=traversal reason=checked_node_child_reference lifetime=until_tree_resize_or_disposal
	if child_index == 0
	{
		return &node.a;
	}
	return &node.b;
}

// Tree query bounds and rays obey the engine's finite-math contract. these
// ordered helpers avoid generic NaN-preserving compare-and-blend sequences in
// slab intersection while retaining identical results for valid operands
tree_ordered_min :: #force_inline proc "contextless" (a, b: f32) -> f32
{
	if a < b
	{
		return a;
	}
	return b;
}

tree_ordered_max :: #force_inline proc "contextless" (a, b: f32) -> f32
{
	if a > b
	{
		return a;
	}
	return b;
}

tree_bounds_merge :: proc "contextless" (a, b: util.Bounding_Box) -> util.Bounding_Box
{
	return {min=util.vector3_min(a.min, b.min), max=util.vector3_max(a.max, b.max)};
}

tree_child_bounds :: proc "contextless" (child: Tree_Node_Child) -> util.Bounding_Box
{
	return {min=child.min, max=child.max};
}

tree_child_from_bounds :: proc "contextless" (bounds: util.Bounding_Box, index, leaf_count: i32) -> Tree_Node_Child
{
	return {min=bounds.min, index=index, max=bounds.max, leaf_count=leaf_count};
}

tree_bounds_intersect :: proc "contextless" (a, b: util.Bounding_Box) -> Reference_State
{
	if a.max.x < b.min.x || a.min.x > b.max.x ||
		a.max.y < b.min.y || a.min.y > b.max.y ||
		a.max.z < b.min.z || a.min.z > b.max.z
	{
		return .Missing;
	}
	return .Present;
}

tree_bounds_metric :: proc "contextless" (bounds: util.Bounding_Box) -> f32
{
	extent := util.vector3_subtract(bounds.max, bounds.min);
	return extent.x * extent.y + extent.y * extent.z + extent.x * extent.z;
}

tree_initialize_root :: proc "contextless" (tree: ^Tree)
{
	tree.node_count = 1;
	tree.leaf_count = 0;
	tree.nodes.memory[0] = {};
	tree.metanodes.memory[0] = {parent=-1, index_in_parent=-1};
}

tree_initialize :: proc (tree: ^Tree, initial_leaf_capacity: int, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if tree == nil || pool == nil || initial_leaf_capacity <= 0
	{
		return .Invalid_Argument;
	}
	node_capacity := max(initial_leaf_capacity - 1, 1);
	nodes, nodes_status := util.buffer_pool_take_at_least(pool, Tree_Node, node_capacity);
	if nodes_status != .Ok
	{
		return physics_memory_status(nodes_status);
	}
	metanodes, meta_status := util.buffer_pool_take_at_least(pool, Tree_Metanode, node_capacity);
	if meta_status != .Ok
	{
		physics_return_buffer(pool, &nodes);
		return physics_memory_status(meta_status);
	}
	leaves, leaves_status := util.buffer_pool_take_at_least(pool, Tree_Leaf, initial_leaf_capacity);
	if leaves_status != .Ok
	{
		physics_return_buffer(pool, &metanodes);
		physics_return_buffer(pool, &nodes);
		return physics_memory_status(leaves_status);
	}
	references, references_status := util.buffer_pool_take_at_least(pool, Tree_Build_Reference, initial_leaf_capacity);
	if references_status != .Ok
	{
		physics_return_buffer(pool, &leaves);
		physics_return_buffer(pool, &metanodes);
		physics_return_buffer(pool, &nodes);
		return physics_memory_status(references_status);
	}
	scratch, scratch_status := util.buffer_pool_take_at_least(pool, Tree_Build_Reference, initial_leaf_capacity);
	if scratch_status != .Ok
	{
		physics_return_buffer(pool, &references);
		physics_return_buffer(pool, &leaves);
		physics_return_buffer(pool, &metanodes);
		physics_return_buffer(pool, &nodes);
		return physics_memory_status(scratch_status);
	}
	_ = util.buffer_clear(nodes, 0, int(nodes.length));
	_ = util.buffer_clear(metanodes, 0, int(metanodes.length));
	_ = util.buffer_clear(leaves, 0, int(leaves.length));
	tree^ = {
		nodes=nodes,
		metanodes=metanodes,
		leaves=leaves,
		build_references=references,
		build_scratch=scratch,
		pool=pool,
		state=.Ready,
	};
	tree_initialize_root(tree);
	return .Ok;
}

tree_ensure_capacity :: proc (tree: ^Tree, leaf_capacity: int) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || leaf_capacity <= 0
	{
		return .Invalid_Argument;
	}
	node_capacity := max(leaf_capacity - 1, 1);
	if leaf_capacity <= int(tree.leaves.length) &&
		leaf_capacity <= int(tree.build_references.length) &&
		leaf_capacity <= int(tree.build_scratch.length) &&
		node_capacity <= int(tree.nodes.length) &&
		node_capacity <= int(tree.metanodes.length)
	{
		return .Ok;
	}
	new_leaves, leaves_status := util.buffer_pool_take_at_least(tree.pool, Tree_Leaf, leaf_capacity);
	if leaves_status != .Ok
	{
		return physics_memory_status(leaves_status);
	}
	new_nodes, nodes_status := util.buffer_pool_take_at_least(tree.pool, Tree_Node, node_capacity);
	if nodes_status != .Ok
	{
		physics_return_buffer(tree.pool, &new_leaves);
		return physics_memory_status(nodes_status);
	}
	new_metanodes, metanodes_status := util.buffer_pool_take_at_least(tree.pool, Tree_Metanode, node_capacity);
	if metanodes_status != .Ok
	{
		physics_return_buffer(tree.pool, &new_nodes);
		physics_return_buffer(tree.pool, &new_leaves);
		return physics_memory_status(metanodes_status);
	}
	new_references, references_status := util.buffer_pool_take_at_least(tree.pool, Tree_Build_Reference, leaf_capacity);
	if references_status != .Ok
	{
		physics_return_buffer(tree.pool, &new_metanodes);
		physics_return_buffer(tree.pool, &new_nodes);
		physics_return_buffer(tree.pool, &new_leaves);
		return physics_memory_status(references_status);
	}
	new_scratch, scratch_status := util.buffer_pool_take_at_least(tree.pool, Tree_Build_Reference, leaf_capacity);
	if scratch_status != .Ok
	{
		physics_return_buffer(tree.pool, &new_references);
		physics_return_buffer(tree.pool, &new_metanodes);
		physics_return_buffer(tree.pool, &new_nodes);
		physics_return_buffer(tree.pool, &new_leaves);
		return physics_memory_status(scratch_status);
	}
	_ = util.buffer_clear(new_leaves, 0, int(new_leaves.length));
	_ = util.buffer_clear(new_nodes, 0, int(new_nodes.length));
	_ = util.buffer_clear(new_metanodes, 0, int(new_metanodes.length));
	copy_status := util.buffer_copy(util.buffer_view(tree.leaves), 0, util.buffer_view(new_leaves), 0, tree.leaf_count);
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(tree.nodes),
			0,
			util.buffer_view(new_nodes),
			0,
			tree.node_count
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(tree.metanodes),
			0,
			util.buffer_view(new_metanodes),
			0,
			tree.node_count
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(tree.build_references), 0, util.buffer_view(new_references), 0, tree.leaf_count,
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(tree.build_scratch), 0, util.buffer_view(new_scratch), 0, tree.leaf_count,
		);
	}
	if copy_status != .Ok
	{
		physics_return_buffer(tree.pool, &new_scratch);
		physics_return_buffer(tree.pool, &new_references);
		physics_return_buffer(tree.pool, &new_metanodes);
		physics_return_buffer(tree.pool, &new_nodes);
		physics_return_buffer(tree.pool, &new_leaves);
		return physics_memory_status(copy_status);
	}
	physics_return_buffer(tree.pool, &tree.build_scratch);
	physics_return_buffer(tree.pool, &tree.build_references);
	physics_return_buffer(tree.pool, &tree.metanodes);
	physics_return_buffer(tree.pool, &tree.nodes);
	physics_return_buffer(tree.pool, &tree.leaves);
	tree.leaves = new_leaves;
	tree.nodes = new_nodes;
	tree.metanodes = new_metanodes;
	tree.build_references = new_references;
	tree.build_scratch = new_scratch;
	return .Ok;
}

tree_resize :: proc (tree: ^Tree, leaf_capacity: int) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || leaf_capacity <= 0
	{
		return .Invalid_Argument;
	}
	target_leaf_capacity := max(leaf_capacity, tree.leaf_count);
	target_node_capacity := max(
		max(target_leaf_capacity - 1, 1), tree.node_count,
	);
	status := physics_resize_buffer_capacity(
		tree.pool, &tree.leaves, target_leaf_capacity, tree.leaf_count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		tree.pool, &tree.nodes, target_node_capacity, tree.node_count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		tree.pool, &tree.metanodes, target_node_capacity, tree.node_count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		tree.pool, &tree.build_references, target_leaf_capacity,
		tree.leaf_count,
	);
	if status != .Ok
	{
		return status;
	}
	return physics_resize_buffer_capacity(
		tree.pool, &tree.build_scratch, target_leaf_capacity,
		tree.leaf_count,
	);
}

tree_clear :: proc "contextless" (tree: ^Tree) -> Physics_Status
{
	if tree == nil || tree.state != .Ready
	{
		return .Disposed;
	}
	tree_initialize_root(tree);
	return .Ok;
}

tree_get_leaf_bounds :: proc "contextless" (tree: ^Tree, leaf_index: int) -> (util.Bounding_Box, Physics_Status)
{
	if tree == nil || tree.state != .Ready || leaf_index < 0 || leaf_index >= tree.leaf_count
	{
		return {}, .Not_Found;
	}
	leaf := tree.leaves.memory[leaf_index];
	child := tree_child(&tree.nodes.memory[tree_leaf_node_index(leaf)], tree_leaf_child_index(leaf));
	if tree_decode_leaf(child.index) != leaf_index
	{
		return {}, .Not_Found;
	}
	return {min=child.min, max=child.max}, .Ok;
}

tree_dispose :: proc (tree: ^Tree) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || tree.pool == nil
	{
		return .Disposed;
	}
	pool := tree.pool;
	physics_return_buffer(pool, &tree.build_scratch);
	physics_return_buffer(pool, &tree.build_references);
	physics_return_buffer(pool, &tree.leaves);
	physics_return_buffer(pool, &tree.metanodes);
	physics_return_buffer(pool, &tree.nodes);
	tree^ = {state=.Disposed};
	return .Ok;
}

#assert(size_of(Tree_Leaf) == 4);
#assert(size_of(Tree_Node_Child) == 32);
#assert(size_of(Tree_Node) == 64);
#assert(size_of(Tree_Metanode) == 12);
