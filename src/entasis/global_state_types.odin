#+vet !unused-imports

package entasis
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// native authoring-value snapshots, not whole-simulation checkpoints. they
	// contain no borrowed pointers, rounded position cache, IDs or hidden solver
	// history. shape handles still refer to the caller's live shape registry
	// capture does not wake bodies. apply uses ordinary complete-description
	// mutation semantics, including waking and cancelling queued body targets
	Global_Body_State :: struct
	{
		pose: Global_Pose,
		velocity: Body_Velocity,
		local_inertia: Body_Inertia,
		collidable: Collidable_Description,
		activity: Activity_Description,
	}

	Global_Static_State :: struct
	{
		pose: Global_Pose,
		shape: Shape_Handle,
		continuity: Continuous_Detection,
	}

	// only these built-in types own absolute position payloads. relative/custom
	// descriptions are not serialized or reinterpreted by this target snapshot
	Global_Constraint_Target_Kind :: enum u8
	{
		Invalid,
		Linear_Servo,
		Axis_Lock,
	}

	Global_Constraint_Target_State :: struct
	{
		position: Global_Position,
		kind: Global_Constraint_Target_Kind,
	}

	Global_Target_Presence :: enum u8
	{
		Absent,
		Present,
	}

	// expired targets export as Absent. applying Present queues a new pending
	// target, this does not restore hidden phase, previous velocity or step state
	Global_Kinematic_Target_State :: struct
	{
		pose: Global_Pose,
		presence: Global_Target_Presence,
	}

	// explicit caller-owned selections. numeric handles have the existing live
	// world lifetime contract: no implicit cross-world remapping or stale-ID check
	Global_State_Selection :: struct
	{
		bodies: []Body_Handle,
		statics: []Static_Handle,
		constraint_targets: []Constraint_Handle,
		kinematic_targets: []Body_Handle,
	}

	// caller-owned disjoint buffers, one value for each selected handle. these
	// slices are transport views, not an owning or durable serialization format
	Global_State_Snapshot :: struct
	{
		bodies: []Global_Body_State,
		statics: []Global_Static_State,
		constraint_targets: []Global_Constraint_Target_State,
		kinematic_targets: []Global_Kinematic_Target_State,
	}

	Global_State_Counts :: struct
	{
		bodies, statics, constraint_targets, kinematic_targets: int,
	}
}
