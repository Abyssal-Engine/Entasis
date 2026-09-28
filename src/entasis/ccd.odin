package entasis

import physics "entasis:entasis_physics"

// ccd_discrete disables sweep tests and limits velocity expansion to the
// collidable's speculative-margin range
ccd_discrete :: #force_inline proc "contextless" () -> Continuous_Detection
{
	return physics.continuous_detection_discrete();
}

// ccd_passive disables sweep tests but allows velocity expansion beyond the
// calculated speculative margin so Continuous partners can discover the pair
ccd_passive :: #force_inline proc "contextless" () -> Continuous_Detection
{
	return physics.continuous_detection_passive();
}

// ccd_continuous enables swept continuous collision detection. larger positive
// thresholds terminate sweeps earlier. smaller positive thresholds refine the
// estimated time of impact more closely
ccd_continuous :: #force_inline proc "contextless" (
	minimum_sweep_timestep: f32 = 1e-3,
	convergence_threshold: f32 = 1e-3,
) -> Continuous_Detection
{
	return physics.continuous_detection_continuous(
		minimum_sweep_timestep,
		convergence_threshold,
	);
}
