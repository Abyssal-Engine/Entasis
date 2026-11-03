package solver_joint_angular

import "core:fmt"
import support "../benchmark_support"

Scenario :: support.Solver_Joint_Scenario;
Observation_Mode :: support.Solver_Observation_Mode;
Options :: support.Solver_Angular_Options;
SCENARIO_NAMES :: [Scenario]string{.Hinge="hinge", .Weld="weld", .Mixed="mixed"};

canonical_parameters :: proc(o: Options, buffer: []u8) -> string
{
	angular: string = "gyro" if o.angular_mode == .Conserve_Momentum_With_Gyroscopic_Torque else "momentum";
	names: [Scenario]string = SCENARIO_NAMES;
	return fmt.bprintf(buffer, "workload=solver_joint_angular;scenario=%s;angular-mode=%s;steps=%d;chains=%d;links=%d;mass-ratio=%d;bodies=%d;constraints=%d;warmup-steps=0;observation=terminal-only;velocity-iterations=1;substeps=4;timestep-hz=60;sleep=disabled;record=off;gravity=0,-10,0;damping=0,0;timing=native-step-sum",
		names[o.scenario], angular, o.steps, o.chains, o.links, o.mass_ratio, o.chains*(o.links+1), o.chains*o.links);
}
