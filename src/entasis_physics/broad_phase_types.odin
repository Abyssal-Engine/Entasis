package entasis_physics

import util "entasis:entasis_utilities"

Broad_Phase_State :: enum u8
{
	Unallocated,
	Ready,
	Disposed,
}
Broad_Phase_Tree :: enum u8
{
	Active,
	Static,
}
Broad_Phase_Bounds_State :: enum u8
{
	Current,
	Predicted,
}
Broad_Phase_Pair :: struct
{
	a: Collidable_Reference,
	b: Collidable_Reference,
}
Broad_Phase_Pair_Visitor_Proc :: #type proc "contextless" (
	user_context: rawptr, worker_index: int, pair: Broad_Phase_Pair,
) -> Physics_Status;
Broad_Phase_Body_Migration :: struct
{
	bounds:      util.Bounding_Box,
	reference:   Collidable_Reference,
	handle:      Body_Handle,
	source_tree: Broad_Phase_Tree,
	target_tree: Broad_Phase_Tree,
	state:       Reference_State,
}
Broad_Phase_Body_Description_Change_State :: enum u8
{
	Unchanged,
	Add,
	Remove,
	Update,
	Move,
}
Broad_Phase_Body_Description_Change :: struct
{
	bounds:           util.Bounding_Box,
	reference:        Collidable_Reference,
	handle:           Body_Handle,
	source_tree:      Broad_Phase_Tree,
	target_tree:      Broad_Phase_Tree,
	source_leaf_index: i32,
	state:            Broad_Phase_Body_Description_Change_State,
}
Broad_Phase_Predict_Type_Batch :: struct
{
	body_indices: [util.PRODUCTION_LANE_COUNT]i32,
	position:     util.Vector3_Wide,
	orientation:  util.Quaternion_Wide,
	velocity:     Body_Velocity_Wide,
	count:        int,
}
Broad_Phase :: struct
{
	active_tree:     Tree,
	static_tree:     Tree,
	active_leaves:   util.Buffer(Collidable_Reference),
	static_leaves:   util.Buffer(Collidable_Reference),
	leaf_scratch:    util.Buffer(i32),
	query_workspace: Tree_Parallel_Query_Workspace,
	maintenance_workspace: Tree_Refit_Refine_Workspace,
	shapes:          ^Shape_Registry,
	bodies:          ^Bodies,
	statics:         ^Statics,
	pool:            ^util.Buffer_Pool,
	frame_index:     i32,
	active_subtree_refinement_start_index: int,
	static_subtree_refinement_start_index: int,
	bounds_state:     Broad_Phase_Bounds_State,
	state:           Broad_Phase_State,
}
