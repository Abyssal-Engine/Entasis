package entasis_physics

import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Position_Bundle :: [util.PRODUCTION_LANE_COUNT]Global_Position;

	// explicit native callback contract. positions use the current simulation frame
	// and the same evaluation instant as orientation/inertia. the read-only bundle
	// is borrowed only for this invocation, callbacks must not retain its pointer
	// only integration_mask lanes are meaningful. their indices are active dense
	// indices, not handles. unselected slots are zero in the outer producer, but
	// wrappers may further narrow the mask without clearing their position values
	// callbacks may mutate only their velocity result and caller-owned disjoint
	// worker data, not world topology, poses, registrations or other body lanes
	Pose_Integrator_Global_Velocity_Proc :: #type proc "contextless" (
		user_context: rawptr, body_indices: util.I32x8,
		position: ^Global_Position_Bundle, orientation: util.Quaternion_Wide, inertia: Body_Inertia_Wide,
		integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^Body_Velocity_Wide,
	);

}
