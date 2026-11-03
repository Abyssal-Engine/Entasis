package solver_fixed_work

import "core:fmt"
import "core:math"
import "core:os"
import util "entasis:entasis_utilities"
import physics "entasis:entasis_physics"
import support "../benchmark_support"

SELECTED_CASE :: support.Solver_Fixed_Case(#config(ENTASIS_BENCHMARK_FIXED_CASE, 0));
#assert(SELECTED_CASE >= .Conserving && SELECTED_CASE <= .Kinematic);
when SELECTED_CASE != .Conserving
{
	#assert(util.PRECISION_MODE == .Float_Coordinates);
}

write_legacy_conserving :: proc(o: Options) -> int
{
	when SELECTED_CASE == .Conserving
	{
		elapsed: f64;
		hash: u64;
		admitted_units: int;
		status: support.Admission;
		elapsed, hash, admitted_units, status = conserving_measure(o);
		if status != .Ok || elapsed <= 0 || math.is_nan(elapsed) || math.is_inf(elapsed)
		{
			return 3;
		}
		file: ^os.File;
		error: os.Error;
		file, error = os.open(o.output, {.Write, .Append, .Create});
		if error != nil
		{
			return 1;
		}
		defer os.close(file);
		size: i64;
		size, error = os.file_size(file);
		if error != nil
		{
			return 1;
		}
		if size == 0
		{
			fmt.fprintln(file, "case_status,metric_status,worker_count,case,layout,angular_mode,bundles,bodies,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,hash,precision,benchmark_parameters"+support.RECORDING_HEADER);
		}
		fmt.fprintfln(file, "ok,ok,%d,conserving,%s,%s,%d,%d,%d,%d,%.9f,%.9f,%016x,%s,\"workload=solver_fixed_work;case=conserving;layout=%s;angular-mode=%s;bundles=%d;bodies=%d;passes=%d;warmup-passes=%d;region=reset-and-four-substeps;admitted-units=%d;work-units=%d;gravity=0,-9.81,0;linear-damping=0.02;angular-damping=0.03;velocity-callback=default;kinematic-velocity=disabled;unconstrained-substeps=disabled\",,,off,whole_loop",
			o.workers, o.layout, o.angular, o.bundles, o.bundles*8, o.passes, o.warmup, elapsed, elapsed/f64(o.passes), hash,
			PRECISION_NAME, o.layout, o.angular, o.bundles, o.bundles*8, o.passes, o.warmup, admitted_units, i64(admitted_units)*i64(o.passes)*4);
		return 0;
	}
	else
	{
		return 2;
	}
}

fixed_write :: proc(o: Options, m: Fixed_Measurement) -> int
{
	case_status, metric_status, physics_status: string = "ok", "ok", "pass";
	exit_code: int;
	if m.status != .Ok || m.completed != o.passes || m.elapsed_ms <= 0 || math.is_nan(m.elapsed_ms) || math.is_inf(m.elapsed_ms)
	{
		case_status, metric_status = "failed", "failed";
		exit_code = 3;
	}
	if m.quality_failures > 0 || m.completed != o.passes
	{
		physics_status = "fail";
		if exit_code == 0
		{
			exit_code = 2;
		}
	}
	if m.pool_after != m.pool_before || m.worker_pool_after != m.worker_pool_before
	{
		case_status, metric_status = "failed", "failed";
		exit_code = 3;
	}
	file: ^os.File;
	error: os.Error;
	file, error = os.open(o.output, {.Write, .Append, .Create});
	if error != nil
	{
		return 1;
	}
	defer os.close(file);
	size: i64;
	size, error = os.file_size(file);
	if error != nil
	{
		return 1;
	}
	if size == 0
	{
		fmt.fprintln(file, "case_status,metric_status,worker_count,case,layout,angular_mode,scenario,bundles,bodies,dynamic_bodies,kinematic_bodies,admitted_units,constraints,contact_points,work_units,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,physics_status,quality_coverage,quality_failures,residual_max_mps,residual_rms_mps,residual_p95_mps,position_error_m,orientation_norm_error,joint_angle_error_rad,pool_capacity_before,pool_capacity_after,worker_pool_capacity_before,worker_pool_capacity_after,precision,benchmark_parameters"+support.RECORDING_HEADER);
	}
	case_name: string = support.solver_fixed_case_name(o.case_name);
	region: string = fixed_region(o.case_name);
	scenario: string = "hinge";
	if o.scenario == .Weld
	{
		scenario = "weld";
	}
	else if o.scenario == .Mixed
	{
		scenario = "mixed";
	}
	material_parameters: string;
	switch o.case_name
	{
	case .Convex, .Cached_Convex, .Nonconvex:
		material: physics.Contact_Material_Properties = fixed_contact_material();
		material_parameters = fmt.aprintf(";friction=%.9g;spring-frequency-hz=%.9g;spring-damping-ratio=%.9g;maximum-recovery-velocity=%.9g",
			material.friction_coefficient, FIXED_SPRING_FREQUENCY_HZ, FIXED_SPRING_DAMPING_RATIO, material.maximum_recovery_velocity);
	case .Joints:
		material_parameters = fmt.aprintf(";spring-frequency-hz=%.9g;spring-damping-ratio=%.9g", FIXED_SPRING_FREQUENCY_HZ, FIXED_SPRING_DAMPING_RATIO);
	case .Conserving, .Nonconserving, .Pose, .Constrained_Final, .Free_Final, .Kinematic:
	}
	defer delete(material_parameters);
	kinematic_velocity: string = "enabled" if o.case_name == .Kinematic else "disabled";
	unconstrained_substeps: string = "enabled" if o.unconstrained_substeps == .Enabled else "disabled";
	parameters: string = fmt.aprintf("workload=solver_fixed_work;case=%s;region=%s;layout=%s;angular-mode=%s;scenario=%s;bundles=%d;bodies=%d;dynamic-bodies=%d;kinematic-bodies=%d;admitted-units=%d;constraints=%d;contact-points=%d;passes=%d;warmup-passes=%d;warmup-cycle=restore-solve-observe;velocity-iterations=%d;substeps=%d;chains=%d;links=%d;mass-ratio=%d;unconstrained-substeps=%s;timestep-hz=60;gravity=%.9g,%.9g,%.9g;linear-damping=%.9g;angular-damping=%.9g;velocity-callback=default;kinematic-velocity=%s%s",
		case_name, region, o.layout, o.angular, scenario, o.bundles, m.bodies, m.dynamic_bodies, m.kinematic_bodies,
		m.admitted_units, m.constraints, m.points, o.passes, o.warmup, o.velocity_iterations, o.substeps,
		o.chains, o.links, o.mass_ratio, unconstrained_substeps,
		m.pose_context.gravity.x, m.pose_context.gravity.y, m.pose_context.gravity.z, m.pose_context.linear_damping, m.pose_context.angular_damping,
		kinematic_velocity, material_parameters);
	defer delete(parameters);
	fmt.fprintfln(file, "%s,%s,%d,%s,%s,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%s,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,%d,%s,\"%s\",,,off,native_step_sum",
		case_status, metric_status, o.workers, case_name, o.layout, o.angular, scenario, o.bundles,
		m.bodies, m.dynamic_bodies, m.kinematic_bodies, m.admitted_units, m.constraints, m.points, m.work_units,
		o.passes, o.warmup, m.elapsed_ms, m.elapsed_ms/f64(max(m.completed, 1)), physics_status, m.completed, m.quality_failures,
		m.quality.residual_max_mps, m.quality.residual_rms_mps, m.quality.residual_p95_mps,
		m.quality.position_error_m, m.quality.orientation_norm_error, m.quality.joint_angle_error_rad,
		m.pool_before, m.pool_after, m.worker_pool_before, m.worker_pool_after, util.PRECISION_NAME, parameters);
	fmt.printfln("FIXED_WORK_COMPLETE case=%s samples=%d/%d status=%v quality_failures=%d observed_units=%d exit_code=%d",
		case_name, m.completed, o.passes, m.status, m.quality_failures, m.quality.observed_units, exit_code);
	return exit_code;
}

main :: proc()
{
	o: Options;
	status: support.Admission;
	o, status = support.parse_solver_fixed_options(os.args[1:]);
	if status != .Ok || o.case_name != SELECTED_CASE
	{
		os.exit(2);
	}
	when SELECTED_CASE == .Conserving
	{
		os.exit(write_legacy_conserving(o));
	}
	else
	{
		os.exit(fixed_write(o, fixed_measure(o)));
	}
}
