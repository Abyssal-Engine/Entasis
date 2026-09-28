// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import sysinfo "core:sys/info"

@(private)
host_configuration: Simd_Configuration;

@(private)
host_configuration_status: Simd_Status;

@(init, private)
initialize_host_simd_configuration :: proc "contextless" ()
{
	host_configuration, host_configuration_status = simd_configuration_from_cpu_features(sysinfo.cpu_features());
}

simd_configuration_from_cpu_features :: proc "contextless" (
	features: sysinfo.CPU_Features,
) -> (configuration: Simd_Configuration, status: Simd_Status)
{
	if .sse41 in features
	{
		configuration.features += {.SSE41};
	}
	if .avx in features
	{
		configuration.features += {.AVX};
	}
	if .avx2 in features
	{
		configuration.features += {.AVX2};
	}
	if .fma in features
	{
		configuration.features += {.FMA};
	}
	if .popcnt in features
	{
		configuration.features += {.POPCNT};
	}
	if .avx512f in features
	{
		configuration.features += {.AVX512F};
	}

	if .AVX2 not_in configuration.features
	{
		configuration.tier = .Unsupported;
		status = .Unsupported_Hardware;
		return;
	}

	configuration.tier = .AVX2;
	if .AVX512F in configuration.features
	{
		configuration.tier = .AVX512_Bulk;
	}
	status = .Ok;
	return;
}

host_simd_configuration :: proc "contextless" () -> (Simd_Configuration, Simd_Status)
{
	return host_configuration, host_configuration_status;
}
