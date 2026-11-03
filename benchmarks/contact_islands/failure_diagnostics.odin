package contact_islands

import "base:runtime"
import "core:bufio"
import "core:fmt"
import "core:io"
import "core:os"
import entasis "entasis:entasis"
import observations "../scene_observation"

// first-event state is written before assessment overwrites it or destroys the world
islands_write_first_failure :: proc(owner: ^Benchmark_Owner, q: ^observations.Qualification)
{
	buffer: [bufio.DEFAULT_BUF_SIZE]byte;
	log: bufio.Writer;
	bufio.writer_init_with_buf(&log, os.to_writer(os.stdout), buffer[:]);
	body: int = q.result.first.body;
	fmt.wprintfln(bufio.writer_to_writer(&log), "ISLAND_FAILURE_BEGIN phase=%v step=%d body=%d reason=%v observed=%.17g shape_limit_m=%.17g",
		q.result.first.phase, q.result.first.step, body, q.result.reason, q.result.first.value, q.result.shape_limit);
	#partial switch q.result.reason
	{
	case .Slot_Excess:
		fmt.wprintfln(bufio.writer_to_writer(&log), "ISLAND_FAILURE_CRITERION comparison=slot_excess>shape_limit_m");
	case .Neighbor_Compression:
		fmt.wprintfln(bufio.writer_to_writer(&log), "ISLAND_FAILURE_CRITERION comparison=neighbor_compression>shape_limit_m");
	case .Invalid_State:
		fmt.wprintfln(bufio.writer_to_writer(&log), "ISLAND_FAILURE_CRITERION comparison=projected_box_radius_status!=Valid geometry=unavailable");
	}
	islands_failure_geometry(owner, q, body, &log);
	if q.result.reason == .Support_Plane
	{
		c: observations.Fixture_Configuration = q.configuration;
		layer_size: int = c.grid[0]*c.grid[2];
		island_size: int = layer_size*c.grid[1];
		below: int = -1;
		plane: f64 = c.floor_top;
		if body%island_size >= layer_size
		{
			below = body-layer_size;
			plane = f64(q.current[below].pose.position.y)+c.half_extent[1];
			islands_failure_geometry(owner, q, below, &log);
		}
		fmt.wprintfln(bufio.writer_to_writer(&log), "ISLAND_FAILURE_SUPPORT comparison=center_y<plane deficit_comparison=deficit>0 body=%d below=%d floor_top=%.17g plane=%.17g center_y=%.17g deficit=%.17g",
			body, below, c.floor_top, plane, f64(q.current[body].pose.position.y), plane-f64(q.current[body].pose.position.y));
	}
	reference: observations.Box_Reference = q.references[body];
	for neighbor in reference.neighbors[:reference.neighbor_count]
	{
		fmt.wprintfln(bufio.writer_to_writer(&log), "ISLAND_FAILURE_NEIGHBOR body=%d neighbor=%d axis=%d support_count=%d",
			body, neighbor.body, neighbor.axis, reference.support_count);
		islands_failure_geometry(owner, q, neighbor.body, &log);
		if q.result.reason != .Invalid_State
		{
			p: [3]f64 = observations.position_components(q.current[body].pose.position);
			np: [3]f64 = observations.position_components(q.current[neighbor.body].pose.position);
			compression: f64;
			if neighbor.axis == 1 && reference.support_count > 0
			{
				compression = observations.assigned_compression(np, q.radii[neighbor.body], p, q.radii[body], 1);
			}
			else
			{
				compression = observations.assigned_compression(p, q.radii[body], np, q.radii[neighbor.body], neighbor.axis);
			}
			fmt.wprintfln(bufio.writer_to_writer(&log), "ISLAND_FAILURE_COMPRESSION body=%d neighbor=%d value=%.17g", body, neighbor.body, compression);
		}
	}
	islands_failure_contacts(owner, body, &log);
	fmt.wprintfln(bufio.writer_to_writer(&log), "ISLAND_FAILURE_END phase=%v step=%d body=%d physical_status=%v",
		q.result.first.phase, q.result.first.step, body, q.result.status);
	write_error: io.Error = log.err;
	flush_error: io.Error = bufio.writer_flush(&log);
	if write_error != nil || flush_error != nil
	{
		fmt.eprintfln("ISLAND_FAILURE_INCOMPLETE operation=write error=%v flush_error=%v", write_error, flush_error);
	}
}

islands_failure_body :: proc(log: ^bufio.Writer, handle: entasis.Body_Handle, state: entasis.Body_State)
{
	fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_STATE handle=%v description=%v p=%.9g,%.9g,%.9g q=%.9g,%.9g,%.9g,%.9g v=%.9g,%.9g,%.9g w=%.9g,%.9g,%.9g",
		handle, state, state.pose.position.x, state.pose.position.y, state.pose.position.z,
		state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w,
		state.velocity.linear.x, state.velocity.linear.y, state.velocity.linear.z,
		state.velocity.angular.x, state.velocity.angular.y, state.velocity.angular.z);
}

islands_failure_geometry :: proc(owner: ^Benchmark_Owner, q: ^observations.Qualification, body: int, log: ^bufio.Writer)
{
	fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_BODY index=%d", body);
	islands_failure_body(log, owner.bodies[body], q.current[body]);
	if q.result.reason == .Invalid_State
	{
		fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_GEOMETRY body=%d availability=unavailable reason=invalid_state", body);
		return;
	}
	p: [3]f64 = observations.position_components(q.current[body].pose.position);
	r: [3]f64 = q.radii[body];
	h: [3]f64 = q.configuration.half_extent;
	reference: observations.Box_Reference = q.references[body];
	lower, upper: [3]f64;
	for axis in 0 ..< 3
	{
		lower[axis] = reference.lower[axis]-h[axis]-p[axis]+r[axis];
		upper[axis] = p[axis]+r[axis]-reference.upper[axis]-h[axis];
	}
	fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_GEOMETRY body=%d availability=available radius=%v half_extent=%v reference_lower=%v reference_upper=%v lower_terms=%v upper_terms=%v slot=%.17g",
		body, r, h, reference.lower, reference.upper, lower, upper, observations.box_slot_excess(p, r, h, reference));
}

islands_failure_contacts :: proc(owner: ^Benchmark_Owner, body: int, log: ^bufio.Writer)
{
	count: int;
	status: entasis.Status;
	count, status = entasis.body_constraint_count(&owner.world, owner.bodies[body]);
	if status != .Ok
	{
		fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_INCOMPLETE operation=body_constraint_count status=%v", status);
		return;
	}
	constraints: []entasis.Constraint_Handle;
	allocation_error: runtime.Allocator_Error;
	constraints, allocation_error = make([]entasis.Constraint_Handle, count);
	if allocation_error != nil
	{
		fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_INCOMPLETE operation=allocate_constraints error=%v", allocation_error);
		return;
	}
	defer delete(constraints);
	written, required: int;
	written, required, status = entasis.body_constraints(&owner.world, owner.bodies[body], constraints);
	fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_CONTACTS body=%d written=%d required=%d status=%v", body, written, required, status);
	if status != .Ok
	{
		fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_INCOMPLETE operation=body_constraints status=%v", status);
		return;
	}
	for handle in constraints
	{
		connected: [4]entasis.Body_Handle;
		written, required, status = entasis.constraint_connected_bodies(&owner.world, handle, connected[:]);
		fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_CONNECTED constraint=%v written=%d required=%d status=%v", handle, written, required, status);
		if status != .Ok
		{
			fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_INCOMPLETE operation=constraint_connected_bodies status=%v", status);
		}
		else
		{
			for connected_body in connected[:written]
			{
				state: entasis.Body_State;
				state, status = entasis.body_get(&owner.world, connected_body);
				if status != .Ok
				{
					fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_INCOMPLETE operation=body_get handle=%v status=%v", connected_body, status);
					continue;
				}
				islands_failure_body(log, connected_body, state);
			}
		}
		data: entasis.Solver_Contact_Data;
		data, status = entasis.solver_contact_data(&owner.world, handle);
		if status != .Ok
		{
			fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_INCOMPLETE operation=solver_contact_data constraint=%v status=%v", handle, status);
			continue;
		}
		fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_CONTACT constraint=%v pair_a=%v pair_b=%v kind=%v bodies=%v body_count=%d contact_count=%d normal=%v offset_b=%v material=%v",
			handle, data.pair_a, data.pair_b, data.kind, data.body_handles, data.body_count, data.contact_count, data.normal, data.offset_b, data.material);
		for point, index in data.contacts[:data.contact_count]
		{
			fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_POINT constraint=%v point=%d offset=%v normal=%v depth=%.9g normal_impulse=%.9g feature=%d",
				handle, index, point.offset_a, point.normal, point.depth, point.normal_impulse, point.feature_id);
		}
		for impulse, index in data.impulses[:data.impulse_count]
		{
			fmt.wprintfln(bufio.writer_to_writer(log), "ISLAND_FAILURE_IMPULSE constraint=%v index=%d value=%.9g", handle, index, impulse);
		}
	}
}
