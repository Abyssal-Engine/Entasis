package benchmark_report

import "core:math"
import "core:fmt"
import "core:strconv"
import "core:strings"

Paired_Domain_Metric :: struct
{
	name, unit: string,
	baseline, candidate, gain, lower, upper: f64,
}

Paired_Summary :: struct
{
	metrics: [64]Paired_Domain_Metric,
	metric_count, primary_metric_index: int,
	baseline_domain, candidate_domain: Domain_Work_Summary,
	gain, lower, upper, median_absolute_gain: f64,
	baseline_ms_per_call, candidate_ms_per_call: f64,
	baseline_contact, candidate_contact: Contact_Work_Summary,
	baseline_physical, candidate_physical: Physical_Summary,
}

paired_median :: proc(values: []f64) -> f64
{
	sort_measurements(values);
	middle: int = len(values)/2;
	if len(values)%2 == 0
	{
		return (values[middle-1]+values[middle])*0.5;
	}
	return values[middle];
}

paired_gain_interval :: proc(baseline, candidate: []f64) -> (Paired_Summary, Report_Status)
{
	if len(baseline) == 0 || len(baseline) != len(candidate)
	{
		return {}, .Invalid_Row_Count;
	}
	gains: []f64 = make([]f64, len(baseline));
	defer delete(gains);
	sample: []f64 = make([]f64, len(baseline));
	defer delete(sample);
	for a, index in baseline
	{
		b: f64 = candidate[index];
		if a <= 0 || b <= 0 || math.is_nan(a) || math.is_inf(a) || math.is_nan(b) || math.is_inf(b)
		{
			return {}, .Invalid_Measurement;
		}
		gains[index] = (a-b)/a;
		sample[index] = math.abs(gains[index]);
	}
	result: Paired_Summary;
	result.median_absolute_gain = paired_median(sample);
	for value, index in gains
	{
		sample[index] = value;
	}
	result.gain = paired_median(sample);
	resamples: []f64 = make([]f64, 10000);
	defer delete(resamples);
	state: u64 = 230023;
	for &median in resamples
	{
		for &value in sample
		{
			state = state*6364136223846793005+1442695040888963407;
			value = gains[int((state >> 32)%u64(len(gains)))];
		}
		median = paired_median(sample);
	}
	sort_measurements(resamples);
	result.lower, result.upper = resamples[249], resamples[9749];
	return result, .Ok;
}

paired_compare_records :: proc(
	baseline, candidate: [][]string, a, b: Lane_Summary, package_name: string,
) -> (Paired_Summary, Report_Result)
{
	switch package_name
	{
	case "container", "contact_islands", "pyramid", "solver_joint_angular", "solver_fixed_work", "solver_contact_work", "simulation_stage_work",
		"shape_mixed_bounds", "query_extensions", "spatial_query_batch", "spatial_query_trace", "world_lifecycle", "awakening_duplicate_sets",
		"contact_churn_grid_4k", "noncontact_constraint_mix", "noncontact_fallback_smoke":
	case:
		return {}, report_error(.Invalid_Arguments, "unsupported paired workload: %s", package_name);
	}
	if len(baseline[0]) != len(candidate[0])
	{
		return {}, report_error(.Invalid_Header, "paired domain headers differ");
	}
	if a.profile != b.profile || a.benchmark_parameters != b.benchmark_parameters ||
		a.measured_steps != b.measured_steps || a.measured_passes != b.measured_passes || a.measured_units != b.measured_units ||
		a.recording_mode != b.recording_mode || a.timing_method != b.timing_method || a.query != b.query
	{
		return {}, report_error(.Invalid_Measurement, "paired fixture, profile or timing conditions differ");
	}
	metadata: [63]string = {"step_index", "precision", "benchmark_components", "case", "phase", "layout", "scenario", "angular_mode", "callback", "bodies", "constraints", "bundles", "measured_passes", "warmup_passes", "seed", "chains", "links", "contact_input", "processor_affinity", "contact_points", "point_updates", "worker_count", "stage", "fixture", "motion", "body_count", "shape_type_count", "shape_count", "static_body_count", "statics", "dynamic_bodies", "kinematic_bodies", "admitted_units", "work_units", "points", "region", "unconstrained_substeps", "mass_ratio", "cycles", "sleeping_body_count", "active_body_count", "awakened_body_count", "inactive_set_count", "expected_pair_count", "query_count_per_operation", "query_count", "query_case", "query_operation", "query_cell", "query_group", "query_family", "query_repetitions", "query_warmup_repetitions", "batch_size", "warmup_batches", "measured_batches", "ray_query_count", "sphere_cast_query_count", "overlap_query_count", "ray_pattern", "query_execution", "warmup_steps", "benchmark_parameters"};
	for name in metadata
	{
		ac, bc: int;
		ac_status, bc_status: Report_Status;
		ac, ac_status = header_index(baseline[0], name);
		bc, bc_status = header_index(candidate[0], name);
		if ac_status != bc_status
		{
			return {}, report_error(.Invalid_Header, "paired metadata missing: %s", name);
		}
		if ac_status == .Ok
		{
			for row in 1 ..< len(baseline)
			{
				if baseline[row][ac] != candidate[row][bc] || baseline[row][ac] != baseline[1][ac]
				{
					return {}, report_error(.Invalid_Measurement, "paired %s differs at repetition %d", name, row);
				}
			}
		}
	}
	paired: Paired_Summary;
	if package_name == "solver_contact_work"
	{
		ac, bc: int;
		ac, _ = header_index(baseline[0], "fixed_elapsed_ms");
		bc, _ = header_index(candidate[0], "fixed_elapsed_ms");
		av: []f64 = make([]f64, a.sample_count);
		defer delete(av);
		bv: []f64 = make([]f64, a.sample_count);
		defer delete(bv);
		for index in 0 ..< a.sample_count
		{
			av[index], _ = strconv.parse_f64(baseline[index+1][ac]);
			bv[index], _ = strconv.parse_f64(candidate[index+1][bc]);
		}
		status: Report_Status;
		paired, status = paired_gain_interval(av, bv);
		if status != .Ok
		{
			return {}, {status=status};
		}
		paired.baseline_ms_per_call = a.mean/f64(a.measured_passes);
		paired.candidate_ms_per_call = b.mean/f64(b.measured_passes);
		paired.baseline_contact, paired.candidate_contact = a.contact, b.contact;
	}
	else
	{
		measurement: string = "ray_elapsed_ms" if package_name == "spatial_query_batch" else "fixed_elapsed_ms" if workload_kind(package_name) == .Fixed else "physics_elapsed_ms";
		metrics_status: Report_Result = paired_domain_metrics(&paired, baseline, candidate, a, package_name, measurement);
		if metrics_status.status != .Ok
		{
			return {}, metrics_status;
		}
	}
	paired.baseline_domain, paired.candidate_domain = a.domain, b.domain;
	paired.baseline_physical, paired.candidate_physical = a.physical, b.physical;
	return paired, {status=.Ok};
}

metric_normalization :: proc(package_name, column: string, summary: Lane_Summary, records: [][]string) -> (string, f64, Report_Result)
{
	unit: string = "ms/process total";
	divisor: f64 = 1;
	count_name: string;
	if package_name == "solver_contact_work"
	{
		return "ms/solve", f64(summary.measured_passes), {status=.Ok};
	}
	if workload_kind(package_name) == .Fixed
	{
		return summary.domain.unit if summary.domain.unit != "" else "ms/pass", f64(summary.measured_passes), {status=.Ok};
	}
	if workload_kind(package_name) == .Parameterized || workload_kind(package_name) == .Domain
	{
		return "ms/step", f64(summary.measured_steps), {status=.Ok};
	}
	switch package_name
	{
	case "shape_mixed_bounds":
		count_name, unit = "measured_passes", "ms/bounds call";
	case "world_lifecycle":
		count_name, unit = "cycles", "ms/cycle";
	case "spatial_query_batch", "spatial_query_trace":
		count_name, unit = "measured_batches", "ms/batch";
	case "query_extensions":
		if summary.query.case_name == "hierarchy" || summary.query.case_name == "global" || summary.query.case_name == "collision-batch"
		{
			return "ns/pair" if summary.query.case_name == "collision-batch" else "ns/query", f64(summary.query.queries)/1e6, {status=.Ok};
		}
		if summary.query.group != ""
		{
			return "ns/API call", f64(summary.query.repetitions)*8192/1e6, {status=.Ok};
		}
		if column != "physics_elapsed_ms" && column != "cq01_elapsed_ms"
		{
			count_name, unit = "query_count_per_operation", "ns/API call";
		}
	case "contact_churn_grid_4k", "noncontact_constraint_mix", "noncontact_fallback_smoke", "awakening_duplicate_sets":
		count_name, unit = "step_index", "ms/world step";
	}
	if count_name != ""
	{
		index: int;
		status: Report_Status;
		index, status = header_index(records[0], count_name);
		if status != .Ok
		{
			return "ms/process total", 1, {status=.Ok};
		}
		value, parsed := strconv.parse_i64(records[1][index]);
		if !parsed || value <= 0
		{
			return "", 0, report_error(.Invalid_Measurement, "invalid paired normalization: %s", count_name);
		}
		for row in records[2:]
		{
			if row[index] != records[1][index]
			{
				return "", 0, report_error(.Invalid_Measurement, "measurement count changes across processes: %s", count_name);
			}
		}
		divisor = f64(value);
		if unit == "ns/API call"
		{
			divisor /= 1e6;
		}
	}
	return unit, divisor, {status=.Ok};
}

paired_domain_metrics :: proc(
	paired: ^Paired_Summary, baseline, candidate: [][]string, summary: Lane_Summary, package_name, primary: string,
) -> Report_Result
{
	for column, index in baseline[0]
	{
		selected: int = 1 if column == primary else 0;
		if package_name == "shape_mixed_bounds" && column == "bounds_clustered_ms"
		{
			selected = 1;
		}
		if package_name == "world_lifecycle"
		{
			for component in ([5]string{"initialization_ms", "growth_ms", "cooking_import_ms", "clear_reuse_ms", "destruction_ms"})
			{
				if column == component
				{
					selected = 1;
				}
			}
		}
		if package_name == "query_extensions" && summary.query.group == "" && strings.has_suffix(column, "_ms") && !strings.has_prefix(column, "cycle")
		{
			selected = 1;
		}
		if package_name == "spatial_query_trace" && (column == "ray_elapsed_ms" || column == "sphere_cast_elapsed_ms" || column == "overlap_elapsed_ms")
		{
			selected = 1;
		}
		if selected == 0
		{
			continue;
		}
		other: int;
		status: Report_Status;
		other, status = header_index(candidate[0], column);
		if status != .Ok || paired.metric_count == len(paired.metrics)
		{
			return report_error(.Invalid_Header, "paired domain metric missing or duplicated: %s", column);
		}
		if baseline[1][index] == "excluded" && candidate[1][other] == "excluded"
		{
			for row in 1 ..< len(baseline)
			{
				if baseline[row][index] != "excluded" || candidate[row][other] != "excluded"
				{
					return report_error(.Invalid_Measurement, "paired excluded metric changes across repetitions");
				}
			}
			continue;
		}
		unit: string;
		divisor: f64;
		result: Report_Result;
		unit, divisor, result = metric_normalization(package_name, column, summary, baseline);
		if result.status != .Ok || divisor <= 0
		{
			return result;
		}
		av: []f64 = make([]f64, len(baseline)-1);
		bv: []f64 = make([]f64, len(candidate)-1);
		for row in 1 ..< len(baseline)
		{
			a: f64;
			a_ok: bool;
			b: f64;
			b_ok: bool;
			a, a_ok = strconv.parse_f64(baseline[row][index]);
			b, b_ok = strconv.parse_f64(candidate[row][other]);
			if !a_ok || !b_ok || a < 0 || b < 0 || math.is_nan(a) || math.is_nan(b) || math.is_inf(a) || math.is_inf(b)
			{
				return report_error(.Invalid_Measurement, "invalid paired domain metric: %s", column);
			}
			av[row-1], bv[row-1] = a, b;
		}
		// disabled query families retain zero columns, with no fabricated gain
		nonzero: int;
		for value, row in av
		{
			if value != 0 || bv[row] != 0
			{
				nonzero = 1;
			}
		}
		if nonzero == 0
		{
			continue;
		}
		interval: Paired_Summary;
		interval, status = paired_gain_interval(av, bv);
		if status != .Ok
		{
			return report_error(status, "paired domain metric cannot be compared: %s", column);
		}
		for sample_index in 0 ..< len(av)
		{
			av[sample_index] /= divisor;
			bv[sample_index] /= divisor;
		}
		metric: Paired_Domain_Metric = {name=column, unit=unit, baseline=arithmetic_mean(av), candidate=arithmetic_mean(bv), gain=interval.gain, lower=interval.lower, upper=interval.upper};
		if column == primary
		{
			paired.primary_metric_index = paired.metric_count;
			paired.gain, paired.lower, paired.upper = interval.gain, interval.lower, interval.upper;
			paired.median_absolute_gain = interval.median_absolute_gain;
		}
		paired.metrics[paired.metric_count] = metric;
		paired.metric_count += 1;
	}
	return {status=.Ok};
}

render_paired_comparison :: proc(request: Report_Request, summaries: []Paired_Summary) -> string
{
	builder: strings.Builder;
	strings.builder_init(&builder);
	report_line(&builder);
	report_line(&builder, "## Paired comparison");
	report_line(&builder);
	report_line(&builder, "Costs are arithmetic means of process averages in the declared unit. Gain = 100*(baseline average-candidate average)/baseline average, calculated before rounding. Positive=better and negative=worse. Threads includes the caller. These results make no keep/reject decision");
	if len(summaries) > 0 && summaries[0].metric_count > 0
	{
		metrics: [64]Paired_Domain_Metric;
		metric_count: int;
		for &p in summaries
		{
			for m in p.metrics[:p.metric_count]
			{
				present: int;
				for existing in metrics[:metric_count]
				{
					if existing.name == m.name
					{
						present = 1;
						break;
					}
				}
				if present == 0
				{
					metrics[metric_count] = m;
					metric_count += 1;
				}
			}
		}
		for metric in metrics[:metric_count]
		{
			report_line(&builder);
			report_format_line(&builder, "### %s", metric.name);
			report_line(&builder);
			report_format_line(&builder, "| Threads | Baseline %s | Candidate %s | Gain %% |", metric.unit, metric.unit);
			report_line(&builder, "| ---: | ---: | ---: | ---: |");
			for worker, index in request.workers
			{
				p: Paired_Summary = summaries[index];
				for m in p.metrics[:p.metric_count]
				{
					if m.name == metric.name
					{
						report_format_line(&builder, "| %d | %.9f | %.9f | %.6f |", worker, m.baseline, m.candidate, 100*(m.baseline-m.candidate)/m.baseline);
						break;
					}
				}
			}
		}
	}
	if request.package_name == "solver_contact_work"
	{
		report_line(&builder);
		report_line(&builder, "| Threads | Baseline ms/solve | Candidate ms/solve | Gain % |");
		report_line(&builder, "| ---: | ---: | ---: | ---: |");
		for worker, index in request.workers
		{
			p: Paired_Summary = summaries[index];
			report_format_line(&builder, "| %d | %.9f | %.9f | %.6f |", worker,
				p.baseline_ms_per_call, p.candidate_ms_per_call, 100*(p.baseline_ms_per_call-p.candidate_ms_per_call)/p.baseline_ms_per_call);
		}
	}
	report_line(&builder);
	report_line(&builder, "<details><summary>Paired variation and intervals</summary>");
	report_line(&builder);
	report_line(&builder, "Supporting statistics use the median of CSV append-order paired percentage gains. The 95% percentile intervals describe that median paired gain, a different statistic from the average-cost headline gain. They use 10,000 paired bootstrap resamples with seed 230023");
	report_line(&builder);
	report_line(&builder, "| Threads | Pairs | Median paired gain % | 95% interval % | Median absolute paired gain % |");
	report_line(&builder, "| ---: | ---: | ---: | --- | ---: |");
	for worker, index in request.workers
	{
		p: Paired_Summary = summaries[index];
		report_format_line(&builder, "| %d | %d | %.6f | [%.6f, %.6f] | %.6f |", worker, request.run_count,
			p.gain*100, p.lower*100, p.upper*100, p.median_absolute_gain*100);
	}
	secondary_metric_count: int;
	for summary in summaries
	{
		secondary_metric_count += max(0, summary.metric_count-1);
	}
	if secondary_metric_count > 0
	{
		report_line(&builder);
		report_line(&builder, "| Threads | Metric | Median paired gain % | 95% interval % |");
		report_line(&builder, "| ---: | --- | ---: | --- |");
		for worker, index in request.workers
		{
			p: Paired_Summary = summaries[index];
			for m, metric_index in p.metrics[:p.metric_count]
			{
				if metric_index == p.primary_metric_index
				{
					continue;
				}
				report_format_line(&builder, "| %d | %s | %.6f | [%.6f, %.6f] |", worker, m.name, 100*m.gain, 100*m.lower, 100*m.upper);
			}
		}
	}
	report_line(&builder);
	report_line(&builder, "</details>");
	if workload_kind(request.package_name) == .Parameterized
	{
		report_line(&builder);
		report_line(&builder, "| Workers | Arm | Measured | Physical outcome | Passed | Failed | Unassessed |");
		report_line(&builder, "| ---: | --- | ---: | --- | ---: | ---: | ---: |");
		for worker, index in request.workers
		{
			p: Paired_Summary = summaries[index];
			for physical, arm in ([2]Physical_Summary{p.baseline_physical, p.candidate_physical})
			{
				names: [Result_Physical]string = {.Unassessed="Unassessed", .Pass="Pass", .Fail="Fail"};
				report_format_line(&builder, "| %d | %s | %d | %s | %d | %d | %d |", worker,
					"baseline" if arm == 0 else "candidate", request.run_count,
					names[physical.status], physical.passed, physical.failed, physical.unassessed);
			}
		}
	}
	if len(summaries) > 0 && summaries[0].baseline_domain.case_name != ""
	{
		report_line(&builder);
		report_line(&builder, "Admitted/work-unit zero means that normalization is not supplied by this producer. Stage work is described by population and its produced pair/contact counts");
		if request.package_name == "simulation_stage_work"
		{
			report_line(&builder, "Stage pool columns report capacity growth across the measured-frame observation window, including untimed prerequisites and observation. They cannot be attributed solely to the timed stage");
		}
		report_line(&builder, "| Workers | Arm | Bodies | Admitted units | Work units | Observed calls | Failed calls | Pool growth bytes | Worker-pool growth bytes |");
		report_line(&builder, "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
		for worker, index in request.workers
		{
			p: Paired_Summary = summaries[index];
			for d, arm in ([2]Domain_Work_Summary{p.baseline_domain, p.candidate_domain})
			{
				report_format_line(&builder, "| %d | %s | %d | %d | %d | %d | %d | %d | %d |", worker,
					"baseline" if arm == 0 else "candidate", d.bodies, d.admitted_units, d.work_units, d.covered, d.failed, d.pool_growth, d.worker_pool_growth);
			}
		}
	}
	if len(summaries) > 0 && summaries[0].baseline_domain.case_name != ""
	{
		report_line(&builder);
		report_line(&builder, "Produced counts and quality below are maxima of the producer CSV fields across processes. Stage contact counts accumulate measured frames, while fixed-solver contact counts describe one restored input. Different quality magnitudes remain visible beside timings");
		report_line(&builder, "| Workers | Arm | Pair count | Manifold count | Contact points | Max residual m/s | RMS residual m/s | p95 residual m/s | Position error m | Orientation norm error | Joint angle error rad |");
		report_line(&builder, "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
		for worker, index in request.workers
		{
			p: Paired_Summary = summaries[index];
			for d, arm in ([2]Domain_Work_Summary{p.baseline_domain, p.candidate_domain})
			{
				quality: [6]string;
				for value, metric in d.residuals
				{
					quality[metric] = fmt.aprintf("%.9g", value) if d.observed_metrics & (u8(1)<<uint(metric)) != 0 else "not assessed";
				}
				report_format_line(&builder, "| %d | %s | %d | %d | %d | %s | %s | %s | %s | %s | %s |", worker,
					"baseline" if arm == 0 else "candidate", d.contact_counts[0], d.contact_counts[1], d.contact_counts[2],
					quality[0], quality[1], quality[2], quality[3], quality[4], quality[5]);
			}
		}
	}
	if request.package_name == "solver_contact_work"
	{
		report_line(&builder);
		report_line(&builder, "| Threads | Baseline ms/1000 point updates | Candidate ms/1000 point updates | Gain % |");
		report_line(&builder, "| ---: | ---: | ---: | ---: |");
		for worker, index in request.workers
		{
			p: Paired_Summary = summaries[index];
			baseline_cost: f64 = 1000*p.baseline_ms_per_call*100/f64(p.baseline_contact.updates);
			candidate_cost: f64 = 1000*p.candidate_ms_per_call*100/f64(p.candidate_contact.updates);
			report_format_line(&builder, "| %d | %.9f | %.9f | %.6f |", worker,
				baseline_cost, candidate_cost, 100*(baseline_cost-candidate_cost)/baseline_cost);
		}
		report_line(&builder);
		report_line(&builder, "Call p50 and p95 are medians of the per-process percentiles for each arm. Maximum call is the largest per-process call maximum. These are not pooled-call quantiles");
		report_line(&builder);
		report_line(&builder, "| Workers | Arm | Median process call p50 ms | Median process call p95 ms | Maximum call ms |");
		report_line(&builder, "| ---: | --- | ---: | ---: | ---: |");
		for worker, index in request.workers
		{
			p: Paired_Summary = summaries[index];
			for contact, arm in ([2]Contact_Work_Summary{p.baseline_contact, p.candidate_contact})
			{
				name: string = "baseline" if arm == 0 else "candidate";
				report_format_line(&builder, "| %d | %s | %.9f | %.9f | %.9f |", worker, name,
					contact.call_p50_median_ms, contact.call_p95_median_ms, contact.call_max_ms);
			}
		}
		report_line(&builder);
		report_line(&builder, "| Workers | Arm | Observed calls | Hard-failed calls | Max residual m/s | Worst call RMS m/s | Worst call p95 m/s | Negative impulse N s | Tangent excess N s | Twist excess N m s |");
		report_line(&builder, "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
		for worker, index in request.workers
		{
			p: Paired_Summary = summaries[index];
			for contact, arm in ([2]Contact_Work_Summary{p.baseline_contact, p.candidate_contact})
			{
				name: string = "baseline" if arm == 0 else "candidate";
				report_format_line(&builder, "| %d | %s | %d | %d | %.9f | %.9f | %.9f | %.9f | %.9f | %.9f |", worker, name,
					contact.covered, contact.failed, contact.residual_max, contact.residual_rms, contact.residual_p95, contact.negative_impulse, contact.tangent_violation, contact.twist_violation);
			}
		}
	}
	return strings.to_string(builder);
}
