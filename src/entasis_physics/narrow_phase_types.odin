package entasis_physics

import util "entasis:entasis_utilities"

Narrow_Phase_Callback_Initialize_Proc :: #type proc "contextless" (
	user_context: rawptr, simulation: ^Simulation,
) -> Physics_Status;
Narrow_Phase_Allow_Proc :: #type proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: Collidable_Reference, speculative_margin: ^f32,
) -> Collision_Testing_State;
Narrow_Phase_Allow_Child_Proc :: #type proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: Collidable_Reference, child_a, child_b: int,
) -> Collision_Testing_State;
Narrow_Phase_Configure_Proc :: #type proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: Collidable_Reference,
	manifold: ^Manifold_Result, material: ^Contact_Material_Properties,
) -> Collision_Testing_State;
Narrow_Phase_Configure_Child_Proc :: #type proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: Collidable_Reference,
	child_a, child_b: int, manifold: ^Convex_Contact_Manifold,
) -> Collision_Testing_State;
Narrow_Phase_Dispose_Proc :: #type proc "contextless" (user_context: rawptr);
Narrow_Phase_Select_Constraint_Proc :: #type proc "contextless" (
	user_context: rawptr, a, b: Collidable_Reference, manifold: ^Manifold_Result,
	default_type_id: i32,
) -> i32;
Narrow_Phase_Callbacks :: struct
{
	initialize: Narrow_Phase_Callback_Initialize_Proc,
	allow:      Narrow_Phase_Allow_Proc,
	allow_child: Narrow_Phase_Allow_Child_Proc,
	configure:  Narrow_Phase_Configure_Proc,
	configure_child: Narrow_Phase_Configure_Child_Proc,
	dispose:    Narrow_Phase_Dispose_Proc,
	select_constraint: Narrow_Phase_Select_Constraint_Proc,
	user_context: rawptr,
}
Narrow_Phase_State :: enum u8
{
	Uninitialized,
	Ready,
	Stepping,
	Disposed,
}
Narrow_Phase_Stage :: enum u8
{
	Idle,
	Prepared,
	Pairs_Found,
	Pairs_Queued,
	Batch_Flushed,
	Stale_Removed,
	Changes_Flushed,
}
Narrow_Phase_Continuation_Type :: enum u8
{
	Discrete,
	Continuous,
}
Narrow_Phase_CCD_Continuation :: struct
{
	pair:                     Collidable_Pair,
	relative_linear_velocity: util.Vector3,
	angular_a:                util.Vector3,
	angular_b:                util.Vector3,
	t:                        f32,
	worker_index:             i32,
	kind:                      Narrow_Phase_Continuation_Type,
}
Narrow_Phase_Result_State :: enum u8
{
	Skipped,
	Rejected,
	Accepted,
}
Narrow_Phase_Pair_Result :: struct
{
	manifold:     Collision_Stored_Manifold,
	material:     Contact_Material_Properties,
	state:        Narrow_Phase_Result_State,
}
Narrow_Phase_Fused_Deferred_Record :: struct
{
	pair_slot:     i32,
	mapping_index: i32,
}
Narrow_Phase_Convex_Direct_Record :: struct
{
	pair_id:            i32,
	speculative_margin: f32,
	shape_data_a:       rawptr,
	shape_data_b:       rawptr,
	pose_a:             Rigid_Pose,
	pose_b:             Rigid_Pose,
}
Narrow_Phase_Box_Sphere_Record :: struct
{
	pair_id:            i32,
	speculative_margin: f32,
	shape_data_a:       rawptr,
	shape_data_b:       rawptr,
	pose_a:             Rigid_Pose,
	pose_b:             Rigid_Pose,
	next_in_task:       i32,
	order:              Collision_Task_Route_Order,
	_padding:           [3]u8,
}
Narrow_Phase_Convex_Deferred_Record :: struct
{
	pair:          Collidable_Pair,
	mapping_index: i32,
	manifold:      Collision_Stored_Manifold,
}
Narrow_Phase_Collision_Route :: enum u8
{
	Predecessor,
	Box_Direct,
	Hull_Direct,
	Box_Sphere_Direct,
}
Narrow_Phase_Transaction_Kind :: enum u8
{
	None,
	Pending,
	Update,
	Add,
	Replace,
	Remove,
	Pending_Prepared,
}
Narrow_Phase_Transaction_Header :: struct
{
	pair:               Collidable_Pair,
	payload_index:      i32,
	pair_mapping_index: i32,
	worker_index:       i32,
	type_id:            i16,
	body_count:         u8,
	kind:               Narrow_Phase_Transaction_Kind,
}
Narrow_Phase_Contact_Add_Data :: struct
{
	solver_add: Solver_Constraint_Add_Transaction,
	pair_add:   Pair_Cache_Add_Transaction,
}
Narrow_Phase_Contact_Replace_Data :: struct
{
	solver_add:    Solver_Constraint_Add_Transaction,
	solver_remove: Solver_Constraint_Remove_Transaction,
}
Narrow_Phase_Contact_Prepared_Data :: struct #raw_union
{
	update:  Solver_Constraint_Update_Transaction,
	add:     Narrow_Phase_Contact_Add_Data,
	replace: Narrow_Phase_Contact_Replace_Data,
}
Narrow_Phase_Contact_Transaction :: struct
{
	body_handles:      [4]Body_Handle,
	old_cache:         Constraint_Cache,
	feature_ids:       [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
	old_impulses:      [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32,
	old_contact_count: i32,
	description:       [CONSTRAINT_DESCRIPTION_STORAGE_BYTES]u8,
	accessor:          ^Contact_Constraint_Accessor_Record,
	prepared:          Narrow_Phase_Contact_Prepared_Data,
}
Narrow_Phase_Remove_Transaction :: struct
{
	constraint_handle: Constraint_Handle,
	solver_remove:     Solver_Constraint_Remove_Transaction,
}
Narrow_Phase_Prepared_Description :: struct
{
	body_handles: [4]Body_Handle,
	feature_ids:  [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
	type_record:  ^Constraint_Type_Record,
	accessor:     ^Contact_Constraint_Accessor_Record,
	type_id:      i32,
	body_count:   i32,
}
Narrow_Phase_Transaction_Range :: struct
{
	start: i32,
	count: i32,
}
NARROW_PHASE_TRANSACTION_STREAM_MIN_PAGE_CAPACITY :: 16;
NARROW_PHASE_TRANSACTION_STREAM_MAX_PAGE_CAPACITY :: 256;
Narrow_Phase_Transaction_Stream :: struct #align(64)
{
	primary_count:             i32,
	first_overflow_page:       i32,
	last_overflow_page:        i32,
	total_count:               i32,
	traversal_candidate_count: i32,
}
Narrow_Phase_Transaction_Reservation :: struct
{
	primary_start: i32,
	primary_count: i32,
	tail_start:    i32,
	tail_count:    i32,
	overflow_start: i32,
	overflow_count: i32,
}
Narrow_Phase_Transaction_Stream_Layout :: struct
{
	primary_capacity:    int,
	page_capacity:       int,
	overflow_start:      int,
	overflow_page_count: int,
	storage_capacity:    int,
	header_capacity:     int,
	order_capacity:      int,
}
Narrow_Phase_Preflight_Worker_Counts :: struct
{
	updates:      i32,
	adds:         i32,
	replacements: i32,
	removals:     i32,
}
Narrow_Phase_Preflight_Summary :: struct
{
	new_constraint_count: int,
	pair_add_count:        int,
	remove_count:          int,
	write_count:           int,
	worker_counts_state:   Reference_State,
}
Narrow_Phase_Worker_Context :: struct
{
	narrow:       ^Narrow_Phase,
	worker_index: i32,
}
Narrow_Phase :: struct
{
	pair_cache:         Pair_Cache,
	accessors:          Contact_Constraint_Accessors,
	candidates:         util.Buffer(Broad_Phase_Pair),
	collision_storage:  util.Buffer(Collision_Batcher_Pair),
	collision_scratch:  Collision_Batcher_Scratch,
	continuations:      util.Buffer(Narrow_Phase_CCD_Continuation),
	results:            util.Buffer(Narrow_Phase_Pair_Result),
	transaction_headers: util.Buffer(Narrow_Phase_Transaction_Header),
	contact_transactions: util.Buffer(Narrow_Phase_Contact_Transaction),
	remove_transactions:  util.Buffer(Narrow_Phase_Remove_Transaction),
	transaction_order:  util.Buffer(i32),
	remove_order:       util.Buffer(Solver_Remove_Order_Entry),
	remove_sort_scratch: util.Buffer(Solver_Remove_Order_Entry),
	body_claims:         util.Buffer(Solver_Transaction_Body_Claim),
	body_batch_claims:  util.Buffer(Solver_Transaction_Body_Batch_Claim),
	type_claims:         util.Buffer(Solver_Transaction_Count_Claim),
	touched_claim_indices: util.Buffer(i32),
	pair_table_claim_generations: util.Buffer(u32),
	batchers:           [MAXIMUM_SOLVER_WORKER_COUNT]Collision_Batcher,
	worker_contexts:    [MAXIMUM_SOLVER_WORKER_COUNT]Narrow_Phase_Worker_Context,
	transaction_statuses: [MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
	transaction_ranges: [MAXIMUM_SOLVER_WORKER_COUNT]Narrow_Phase_Transaction_Range,
	candidate_counts:    [MAXIMUM_SOLVER_WORKER_COUNT]i32,
	transaction_overflow_page_next:   util.Buffer(i32),
	transaction_overflow_page_counts: util.Buffer(i32),
	transaction_streams: [MAXIMUM_SOLVER_WORKER_COUNT]Narrow_Phase_Transaction_Stream,
	transaction_overflow_page_cursor: i32,
	transaction_stream_primary_capacity:    int,
	transaction_stream_page_capacity:       int,
	transaction_stream_overflow_start:      int,
	transaction_stream_overflow_page_count: int,
	transaction_stream_storage_capacity:    int,
	remove_count:       int,
	touched_body_count: int,
	touched_type_count: int,
	prepared_pair_add_count: int,
	remove_handle_return_start: int,
	inactive_sets_present: Reference_State,
	claim_generation:   u32,
	collision_capacity_per_worker: int,
	collision_child_capacity: int,
	pending_capacity_per_worker: int,
	current_transaction_capacity: int,
	callbacks:          Narrow_Phase_Callbacks,
	stored_completion_material: Contact_Material_Properties,
	stored_completion_state: Reference_State,
	broad_phase:        ^Broad_Phase,
	bodies:             ^Bodies,
	statics:            ^Statics,
	shapes:             ^Shape_Registry,
	solver:             ^Solver,
	tasks:              ^Collision_Task_Registry,
	sweep_tasks:        ^Sweep_Task_Registry,
	awakener:           ^Island_Awakener,
	pool:               ^util.Buffer_Pool,
	active_worker_count: int,
	last_stage:          Narrow_Phase_Stage,
	callback_activation: Reference_State,
	state:              Narrow_Phase_State,
}
#assert(size_of(Narrow_Phase_Transaction_Stream) == 64);
#assert(align_of(Narrow_Phase_Transaction_Stream) == 64);
#assert(size_of(Narrow_Phase_Fused_Deferred_Record) == 8);
#assert(size_of(Narrow_Phase_Fused_Deferred_Record) <= size_of(Narrow_Phase_Pair_Result));
#assert(size_of(Narrow_Phase_Convex_Direct_Record) == 88);
#assert(align_of(Narrow_Phase_Convex_Direct_Record) == 8);
#assert(align_of(Collision_Batcher_Pair) >= align_of(Narrow_Phase_Convex_Direct_Record));
#assert(size_of(Narrow_Phase_Convex_Direct_Record) <= size_of(Collision_Batcher_Pair));
#assert(size_of(Narrow_Phase_Box_Sphere_Record) == 96);
#assert(align_of(Narrow_Phase_Box_Sphere_Record) == 8);
#assert(align_of(Collision_Batcher_Pair) >= align_of(Narrow_Phase_Box_Sphere_Record));
#assert(size_of(Narrow_Phase_Box_Sphere_Record) <= size_of(Collision_Batcher_Pair));
#assert(size_of(Narrow_Phase_Convex_Deferred_Record) == 160);
#assert(size_of(Narrow_Phase_Convex_Deferred_Record) <= size_of(Narrow_Phase_Pair_Result));
#assert(int(Narrow_Phase_Transaction_Kind.None) == 0);
#assert(int(Narrow_Phase_Transaction_Kind.Pending) == 1);
#assert(int(Narrow_Phase_Transaction_Kind.Update) == 2);
#assert(int(Narrow_Phase_Transaction_Kind.Add) == 3);
#assert(int(Narrow_Phase_Transaction_Kind.Replace) == 4);
#assert(int(Narrow_Phase_Transaction_Kind.Remove) == 5);
#assert(int(Narrow_Phase_Transaction_Kind.Pending_Prepared) == 6);
#assert(size_of(Narrow_Phase_Transaction_Header) == 24);
#assert(size_of(Narrow_Phase_Contact_Transaction) == 576);
#assert(size_of(Narrow_Phase_Remove_Transaction) == 120);
#assert(size_of(Narrow_Phase_Pair_Result) <= CONSTRAINT_DESCRIPTION_STORAGE_BYTES);
