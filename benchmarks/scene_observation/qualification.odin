package scene_observation

import "base:runtime"
import "core:fmt"
import "core:math"
import entasis "entasis:entasis"

Verification_Mode :: enum u8
{
	On,
	Off,
}
Physical_Status :: enum u8
{
	Unassessed,
	Pass,
	Fail,
}
Coverage :: enum u8
{
	Incomplete,
	Complete,
}
Criterion :: enum u8
{
	Container_Safety,
	Island_Boxes,
	Pyramid_Boxes,
	Wall_Boxes,
}
Assessment_Phase :: enum u8
{
	None,
	Construction,
	Warmup_Construction,
	Warmup,
	Measured,
}
Breach_Reason :: enum u8
{
	None,
	Verification_Off,
	Unsupported_Fixture,
	Insufficient_Window,
	Incomplete_Coverage,
	Invalid_State,
	Slot_Excess,
	Neighbor_Compression,
	Support_Plane,
	Container_Escape,
	Impact_Before_Assessment,
}
Evidence :: enum u8
{
	Absent,
	Present,
}
Container_Entry :: enum u8
{
	Outside,
	Entered,
}
Physical_Observation :: struct
{
	evidence: Evidence,
	phase: Assessment_Phase,
	step: int,
	body: int,
	value: f64,
}
Qualification_Result :: struct
{
	mode: Verification_Mode,
	criterion: Criterion,
	status: Physical_Status,
	coverage: Coverage,
	reason: Breach_Reason,
	sample_count: int,
	warmup_samples: int,
	measured_samples: int,
	terminal_samples: int,
	shape_limit: f64,
	first: Physical_Observation,
	maximum_slot: Physical_Observation,
	maximum_neighbor: Physical_Observation,
	impact: Physical_Observation,
}
Qualification :: struct
{
	configuration: Fixture_Configuration,
	eligibility: Eligibility,
	result: Qualification_Result,
	references: []Box_Reference,
	current: []entasis.Body_State,
	previous: []entasis.Body_State,
	radii: [][3]f64,
	entries: []Container_Entry,
	measured_construction: Evidence,
	warmup_construction: Evidence,
	sequence: Coverage,
	shape_step_samples: int,
}

qualification_create :: proc(q: ^Qualification, c: Fixture_Configuration, mode: Verification_Mode) -> runtime.Allocator_Error
{
	q.configuration = c
	q.eligibility = fixture_eligibility(c)
	q.result = {mode=mode, criterion=Criterion(c.fixture), coverage=.Incomplete}
	q.sequence = .Complete
	q.result.shape_limit = 0.1
	if c.fixture == .Contact_Islands
	{
		q.result.shape_limit = 0.2*min(c.half_extent[0], c.half_extent[1], c.half_extent[2])
	}
	if mode == .Off
	{
		q.result.reason = .Verification_Off
		return nil
	}
	allocation_error: runtime.Allocator_Error
	q.current, allocation_error = make([]entasis.Body_State, c.body_count)
	if allocation_error != nil
	{
		return allocation_error
	}
	q.references, allocation_error = make([]Box_Reference, c.population)
	if allocation_error != nil
	{
		return allocation_error
	}
	q.radii, allocation_error = make([][3]f64, c.body_count)
	if allocation_error != nil
	{
		return allocation_error
	}
	if c.fixture == .Pyramid && c.body_count > c.population
	{
		q.previous, allocation_error = make([]entasis.Body_State, c.body_count)
	}
	if c.fixture == .Container
	{
		q.entries, allocation_error = make([]Container_Entry, c.population)
	}
	return allocation_error
}

qualification_destroy :: proc(q: ^Qualification)
{
	delete(q.references)
	delete(q.current)
	delete(q.previous)
	delete(q.radii)
	delete(q.entries)
	q^ = {}
}

qualification_bind :: proc(q: ^Qualification, phase: Assessment_Phase)
{
	if q.result.mode == .Off
	{
		return
	}
	// references belong to the newly constructed world, while first breach belongs to the invocation
	build_box_references(q.configuration, q.current, q.references)
	if q.configuration.fixture == .Container && phase == .Construction
	{
		for &entry in q.entries
		{
			entry = .Outside
		}
		q.result.measured_samples = 0
		q.result.terminal_samples = 0
		q.measured_construction = .Absent
	}
	if len(q.previous) > 0
	{
		for state, index in q.current
		{
			q.previous[index] = state
		}
	}
	qualification_assess(q, phase, 0)
}

qualification_breach :: proc(q: ^Qualification, reason: Breach_Reason, phase: Assessment_Phase, step, body: int, value: f64)
{
	if q.result.status == .Fail
	{
		return
	}
	q.result.status = .Fail
	q.result.reason = reason
	q.result.first = {evidence=.Present, phase=phase, step=step, body=body, value=value}
}

qualification_maximum :: proc(observation: ^Physical_Observation, phase: Assessment_Phase, step, body: int, value: f64)
{
	if observation.evidence == .Absent || value > observation.value
	{
		observation^ = {evidence=.Present, phase=phase, step=step, body=body, value=value}
	}
}

qualification_island_support_safety :: proc(q: ^Qualification, phase: Assessment_Phase, step: int)
{
	c: Fixture_Configuration = q.configuration
	layer_size: int = c.grid[0]*c.grid[2]
	island_size: int = layer_size*c.grid[1]
	for state, body in q.current[:c.population]
	{
		plane: f64 = c.floor_top
		if body%island_size >= layer_size
		{
			plane = f64(q.current[body-layer_size].pose.position.y)+c.half_extent[1]
		}
		center: f64 = f64(state.pose.position.y)
		if center < plane
		{
			qualification_breach(q, .Support_Plane, phase, step, body, plane-center)
		}
	}
}

qualification_assess :: proc(q: ^Qualification, phase: Assessment_Phase, step: int)
{
	if q.result.mode == .Off
	{
		return
	}
	c: Fixture_Configuration = q.configuration
	q.result.sample_count += 1
	switch phase
	{
		case .Construction:
		if q.measured_construction != .Absent || step != 0
		{
			q.sequence = .Incomplete
		}
		q.measured_construction = .Present
		case .Warmup_Construction:
		if q.warmup_construction != .Absent || step != 0
		{
			q.sequence = .Incomplete
		}
		q.warmup_construction = .Present
		case .Warmup:
		if step != q.result.warmup_samples+1
		{
			q.sequence = .Incomplete
		}
		q.result.warmup_samples += 1
		case .Measured:
		if step != q.result.measured_samples+1
		{
			q.sequence = .Incomplete
		}
		q.result.measured_samples += 1
		case .None:
		q.sequence = .Incomplete
	}
	geometry_validity: State_Status = .Valid
	for state, index in q.current
	{
		validity: State_Status
		q.radii[index], validity = projected_box_radius(state, c.half_extent)
		if validity != .Valid
		{
			geometry_validity = .Invalid
			qualification_breach(q, .Invalid_State, phase, step, index, 0)
		}
	}
	// invalid numerical state cannot participate in geometry, but its sample still proves observed coverage
	if geometry_validity == .Invalid
	{
		return
	}
	if c.fixture == .Pyramid && q.result.impact.evidence == .Absent
	{
		h: [3]f64 = c.half_extent
		radius: f64 = c.projectile_radius+0.1*min(h[0], h[1], h[2])+math.sqrt(h[0]*h[0]+h[1]*h[1]+h[2]*h[2])
		for projectile: int = c.population; projectile < c.body_count; projectile += 1
		{
			for box: int = 0; box < c.population; box += 1
			{
				previous: [3]f64 = position_components(q.previous[projectile].pose.position)-position_components(q.previous[box].pose.position)
				current: [3]f64 = position_components(q.current[projectile].pose.position)-position_components(q.current[box].pose.position)
				if possible_projectile_impact(previous, current, radius) == .Invalid
				{
					q.result.impact = {evidence=.Present, phase=phase, step=step, body=projectile}
					break
				}
			}
			if q.result.impact.evidence == .Present
			{
				break
			}
		}
	}
	if len(q.previous) > 0
	{
		for state, index in q.current
		{
			q.previous[index] = state
		}
	}
	if q.eligibility != .Supported
	{
		return
	}
	if c.fixture == .Container
	{
		if phase == .Construction || phase == .Measured
		{
			for state, index in q.current[:c.population]
			{
				p: [3]f64 = position_components(state.pose.position)
				r: [3]f64 = q.radii[index]
				if q.entries[index] == .Outside && math.abs(p[0])+r[0] <= c.container_half_size[0] &&
					math.abs(p[2])+r[2] <= c.container_half_size[1] && p[1]+r[1] <= c.container_rim
				{
					q.entries[index] = .Entered
				}
				if q.entries[index] == .Entered && p[1]-r[1] > c.container_rim
				{
					qualification_breach(q, .Container_Escape, phase, step, index, p[1]-r[1]-c.container_rim)
				}
			}
		}
		return
	}
	if c.fixture == .Contact_Islands
	{
		qualification_island_support_safety(q, phase, step)
		if phase != .Measured || c.steps <= c.timestep_hz || step < c.steps-c.timestep_hz || step > c.steps
		{
			return
		}
	}
	if c.fixture == .Pyramid && q.result.impact.evidence == .Present
	{
		return
	}
	if phase == .Warmup || phase == .Measured
	{
		q.shape_step_samples += 1
	}
	if c.fixture == .Contact_Islands
	{
		q.result.terminal_samples += 1
	}
	for reference, index in q.references
	{
		p: [3]f64 = position_components(q.current[index].pose.position)
		r: [3]f64 = q.radii[index]
		slot: f64 = box_slot_excess(p, r, c.half_extent, reference)
		qualification_maximum(&q.result.maximum_slot, phase, step, index, slot)
		if slot > q.result.shape_limit
		{
			qualification_breach(q, .Slot_Excess, phase, step, index, slot)
		}
		minimum_support: f64 = math.F64_MAX
		for neighbor_index: int = 0; neighbor_index < reference.neighbor_count; neighbor_index += 1
		{
			neighbor: Neighbor = reference.neighbors[neighbor_index]
			np: [3]f64 = position_components(q.current[neighbor.body].pose.position)
			nr: [3]f64 = q.radii[neighbor.body]
			compression: f64
			if neighbor.axis == 1 && reference.support_count > 0
			{
				compression = assigned_compression(np, nr, p, r, 1)
				minimum_support = min(minimum_support, np[1])
			}
			else
			{
				compression = assigned_compression(p, r, np, nr, neighbor.axis)
			}
			qualification_maximum(&q.result.maximum_neighbor, phase, step, index, compression)
			if compression > q.result.shape_limit
			{
				qualification_breach(q, .Neighbor_Compression, phase, step, index, compression)
			}
		}
		if c.fixture == .Pyramid || c.fixture == .Pyramid_Wall
		{
			plane: f64 = minimum_support+c.half_extent[1] if reference.support_count > 0 else c.floor_top
			if p[1] < plane
			{
				qualification_breach(q, .Support_Plane, phase, step, index, plane-p[1])
			}
		}
	}
}

qualification_complete :: proc(q: ^Qualification)
{
	if q.result.mode == .Off
	{
		return
	}
	c: Fixture_Configuration = q.configuration
	if q.sequence == .Complete && q.measured_construction == .Present && q.result.measured_samples == c.steps && q.result.warmup_samples == c.warmup_steps
	{
		q.result.coverage = .Complete
	}
	if q.result.status == .Fail
	{
		return
	}
	q.result.status = .Unassessed
	if q.eligibility != .Supported
	{
		q.result.reason = .Unsupported_Fixture
	}
	else if q.result.coverage != .Complete
	{
		q.result.reason = .Incomplete_Coverage
	}
	else if c.fixture == .Contact_Islands && (c.steps <= c.timestep_hz || q.result.terminal_samples != c.timestep_hz+1)
	{
		q.result.reason = .Insufficient_Window
	}
	else if c.fixture == .Pyramid && q.shape_step_samples == 0
	{
		q.result.reason = .Impact_Before_Assessment
	}
	else
	{
		q.result.status = .Pass
		q.result.reason = .None
	}
}

PHYSICAL_HEADER :: ",verification_mode,physical_criterion,physical_status,physical_coverage,physical_reason,physical_sample_count,physical_shape_limit_m,physical_first_phase,physical_first_step,physical_first_body,physical_first_value_m,physical_max_slot_excess_m,physical_max_slot_excess_phase,physical_max_slot_excess_step,physical_max_slot_excess_body,physical_max_neighbor_compression_m,physical_max_neighbor_compression_phase,physical_max_neighbor_compression_step,physical_max_neighbor_compression_body,physical_warmup_samples,physical_measured_samples,physical_terminal_samples,physical_impact_step"

verification_text :: proc(mode: Verification_Mode) -> string
{
	return "on" if mode == .On else "off"
}
physical_status_text :: proc(status: Physical_Status) -> string
{
	names: [Physical_Status]string = {.Unassessed="unassessed", .Pass="pass", .Fail="fail"}
	return names[status]
}
physical_reason_text :: proc(reason: Breach_Reason) -> string
{
	names: [Breach_Reason]string = {.None="none", .Verification_Off="verification_off", .Unsupported_Fixture="unsupported_fixture",
		.Insufficient_Window="insufficient_window", .Incomplete_Coverage="incomplete_coverage", .Invalid_State="invalid_state",
		.Slot_Excess="slot_excess", .Neighbor_Compression="neighbor_compression", .Support_Plane="support_plane",
		.Container_Escape="container_escape", .Impact_Before_Assessment="impact_before_assessment"}
	return names[reason]
}
assessment_phase_text :: proc(phase: Assessment_Phase) -> string
{
	names: [Assessment_Phase]string = {.None="", .Construction="construction", .Warmup_Construction="warmup_construction", .Warmup="warmup", .Measured="measured"}
	return names[phase]
}

physical_csv :: proc(result: Qualification_Result, buffer: []u8) -> string
{
	first_buffer, slot_buffer, neighbor_buffer: [256]u8
	impact_buffer: [32]u8
	first, slot, neighbor: string = ",,,", ",,,", ",,,"
	impact: string
	if result.first.evidence == .Present
	{
		value_buffer: [64]u8
		value: string
		if result.reason != .Invalid_State
		{
			value = fmt.bprintf(value_buffer[:], "%.17g", result.first.value)
		}
		first = fmt.bprintf(first_buffer[:], "%s,%d,%d,%s", assessment_phase_text(result.first.phase), result.first.step, result.first.body, value)
	}
	if result.maximum_slot.evidence == .Present
	{
		o: Physical_Observation = result.maximum_slot
		slot = fmt.bprintf(slot_buffer[:], "%.17g,%s,%d,%d", o.value, assessment_phase_text(o.phase), o.step, o.body)
	}
	if result.maximum_neighbor.evidence == .Present
	{
		o: Physical_Observation = result.maximum_neighbor
		neighbor = fmt.bprintf(neighbor_buffer[:], "%.17g,%s,%d,%d", o.value, assessment_phase_text(o.phase), o.step, o.body)
	}
	if result.impact.evidence == .Present
	{
		impact = fmt.bprintf(impact_buffer[:], "%d", result.impact.step)
	}
	criteria: [Criterion]string = {.Container_Safety="container_safety", .Island_Boxes="island_boxes", .Pyramid_Boxes="pyramid_boxes", .Wall_Boxes="wall_boxes"}
	return fmt.bprintf(buffer, ",%s,%s,%s,%s,%s,%d,%.17g,%s,%s,%s,%d,%d,%d,%s",
		verification_text(result.mode), criteria[result.criterion],
		physical_status_text(result.status), "complete" if result.coverage == .Complete else "incomplete", physical_reason_text(result.reason),
		result.sample_count, result.shape_limit, first, slot, neighbor, result.warmup_samples, result.measured_samples, result.terminal_samples, impact)
}
