package entasis_physics

import util "entasis:entasis_utilities"

Solver_State :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}
Solver :: struct
{
	registry:                    Constraint_Type_Registry,
	active_set:                  Constraint_Set,
	handle_to_constraint: util.Buffer(Constraint_Location),
	integration_flags:     util.Buffer(u64),
	integration_body_seen: util.Buffer(u64),
	work_blocks:          util.Buffer(Solver_Work_Block),
	compression_candidates: util.Buffer(Compression_Candidate),
	handle_pool:                 util.Id_Pool,
	bodies:                      ^Bodies,
	integrator:                  ^Pose_Integrator,
	pool:                        ^util.Buffer_Pool,
	sequential_batch:            Sequential_Fallback_Batch,
	fallback_batch_threshold:    i32,
	fallback_batch_index:        i32,
	initial_type_batch_capacity: i32,
	compression_batch_index:     i32,
	compression_type_batch_index: i32,
	state:                       Solver_State,
	contact_coefficient_offsets: util.Buffer(i32),
	contact_coefficients: util.Buffer(util.F32x8),
	joint_breaks: ^Joint_Break_System,
}
Compression_Candidate :: struct
{
	handle:       Constraint_Handle,
	target_batch: i32,
}
Solver_Compression_Discovery_Job :: struct
{
	solver:                 ^Solver,
	source_batch_index:     int,
	first_type_batch_index: int,
	type_batch_count:       int,
	scheduled_count:        int,
	worker_count:           int,
	statuses:               ^[MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
}
Solver_Constraint_Add_Transaction :: struct
{
	type_record:       ^Constraint_Type_Record,
	batch:             ^Constraint_Batch,
	type_batch:        ^Type_Batch,
	body_lists:        [4]^util.Quick_List(Body_Constraint_Reference),
	body_list_indices: [4]i32,
	reference:         Constraint_Reference,
	batch_index:       i32,
	type_batch_index:  i32,
	record_index:      i32,
}
Solver_Constraint_Remove_Transaction :: struct
{
	type_record:      ^Constraint_Type_Record,
	batch:            ^Constraint_Batch,
	type_batch:       ^Type_Batch,
	body_lists:       [4]^util.Quick_List(Body_Constraint_Reference),
	reference:        Constraint_Reference,
	batch_index:      i32,
	type_batch_index: i32,
	record_index:     i32,
}
Solver_Constraint_Update_Transaction :: struct
{
	type_record:  ^Constraint_Type_Record,
	type_batch:   ^Type_Batch,
	batch_index:  i32,
	record_index: i32,
}
Solver_Body_Mobility_Change_Action :: enum u8
{
	Update,
	Remove,
	Move,
}
Solver_Body_Mobility_Change_State :: enum u8
{
	Unprepared,
	Prepared,
}
Solver_Body_Mobility_Change_Entry :: struct
{
	reference:                Constraint_Reference,
	handle:                   Constraint_Handle,
	source_batch_index:       i32,
	target_batch_index:       i32,
	type_id:                  i32,
	body_index_in_constraint: i32,
	action:                   Solver_Body_Mobility_Change_Action,
}
Solver_Body_Mobility_Change :: struct
{
	entries:      util.Buffer(Solver_Body_Mobility_Change_Entry),
	body_handle:  Body_Handle,
	body_index:   i32,
	entry_count:  i32,
	previous:     Body_Mobility,
	current:      Body_Mobility,
	state:        Solver_Body_Mobility_Change_State,
}
Solver_Transaction_Body_Claim :: struct
{
	list_count: i32,
	generation: u32,
}
Solver_Transaction_Body_Batch_Claim :: struct
{
	bits:       u64,
	generation: u32,
}
Solver_Transaction_Count_Claim :: struct
{
	count:      i32,
	generation: u32,
}
#assert(size_of(Solver_Transaction_Body_Batch_Claim) == 16);
Solver_Remove_Order_Entry :: struct
{
	key:          u64,
	record_index: i32,
}
