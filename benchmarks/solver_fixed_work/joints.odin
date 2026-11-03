package solver_fixed_work

import "core:math"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

fixed_add_joints :: proc(s: ^Fixed_State) -> physics.Physics_Status
{
	s.edges = make([]Joint_Edge, s.options.chains*s.options.links);
	cursor: int;
	for chain in 0 ..< s.options.chains
	{
		for link in 1 ..= s.options.links
		{
			a: int = chain*(s.options.links+1)+link-1;
			b: int = a+1;
			connected: [4]physics.Body_Handle = {0=s.handles[a], 1=s.handles[b]};
			kind: support.Solver_Joint_Scenario = s.options.scenario;
			if kind == .Mixed
			{
				kind = .Weld if link%3 == 0 else .Hinge;
			}
			status: physics.Physics_Status;
			if kind == .Weld
			{
				description: physics.Weld = {local_offset={0, -1, 0}, local_orientation={w=1},
					spring_settings=physics.spring_settings_create(FIXED_SPRING_FREQUENCY_HZ, FIXED_SPRING_DAMPING_RATIO)};
				_, status = physics.solver_add(&s.solver, &connected, &description);
			}
			else
			{
				description: physics.Hinge = {local_offset_a={0, -0.5, 0}, local_offset_b={0, 0.5, 0},
					local_hinge_axis_a={0, 0, 1}, local_hinge_axis_b={0, 0, 1},
					spring_settings=physics.spring_settings_create(FIXED_SPRING_FREQUENCY_HZ, FIXED_SPRING_DAMPING_RATIO)};
				_, status = physics.solver_add(&s.solver, &connected, &description);
			}
			if status != .Ok
			{
				return status;
			}
			s.edges[cursor] = {a, b, kind};
			cursor += 1;
		}
	}
	return .Ok;
}

fixed_observe_joints :: proc(s: ^Fixed_State, q: ^Fixed_Quality)
{
	active: ^physics.Body_Set = &s.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for edge in s.edges
	{
		a: physics.Rigid_Pose = active.dynamics_state.memory[edge.a].motion.pose;
		b: physics.Rigid_Pose = active.dynamics_state.memory[edge.b].motion.pose;
		pa: util.Vector3 = util.vector3_add(a.position, util.quaternion_transform({0, -1 if edge.kind == .Weld else -0.5, 0}, a.orientation));
		pb: util.Vector3 = b.position;
		angle: f64;
		if edge.kind == .Weld
		{
			qa, qb: util.Quaternion = a.orientation, b.orientation;
			dot: f64 = abs(f64(qa.x)*f64(qb.x)+f64(qa.y)*f64(qb.y)+f64(qa.z)*f64(qb.z)+f64(qa.w)*f64(qb.w));
			na: f64 = math.sqrt(f64(qa.x)*f64(qa.x)+f64(qa.y)*f64(qa.y)+f64(qa.z)*f64(qa.z)+f64(qa.w)*f64(qa.w));
			nb: f64 = math.sqrt(f64(qb.x)*f64(qb.x)+f64(qb.y)*f64(qb.y)+f64(qb.z)*f64(qb.z)+f64(qb.w)*f64(qb.w));
			if na*nb == 0
			{
				q.failures += 1;
				continue;
			}
			angle = 2*math.acos(clamp(dot/(na*nb), f64(0), f64(1)));
		}
		else
		{
			pb = util.vector3_add(pb, util.quaternion_transform({0, 0.5, 0}, b.orientation));
			axis_a: util.Vector3 = util.quaternion_transform({0, 0, 1}, a.orientation);
			axis_b: util.Vector3 = util.quaternion_transform({0, 0, 1}, b.orientation);
			angle = math.atan2(f64(util.vector3_length(util.vector3_cross(axis_a, axis_b))), f64(util.vector3_dot(axis_a, axis_b)));
		}
		d: util.Vector3 = util.vector3_subtract(pa, pb);
		position: f64 = math.sqrt(f64(d.x)*f64(d.x)+f64(d.y)*f64(d.y)+f64(d.z)*f64(d.z));
		if math.is_nan(position) || math.is_inf(position) || math.is_nan(angle) || math.is_inf(angle)
		{
			q.failures += 1;
		}
		q.position_error_m = max(q.position_error_m, position);
		q.joint_angle_error_rad = max(q.joint_angle_error_rad, angle);
	}
}
