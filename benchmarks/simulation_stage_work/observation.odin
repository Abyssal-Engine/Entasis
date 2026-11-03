package simulation_stage_work

import "core:math"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import contacts "../solver_contacts"

// two-cell separation is at least 2*0.98-0.25=1.71, beyond the
// unit-box span and zero-velocity prediction margin in this fixture
positions_overlap :: proc "contextless" (a, b: util.Vector3) -> int
{
	if abs(f64(a.x)-f64(b.x)) <= 1 && abs(f64(a.y)-f64(b.y)) <= 1 && abs(f64(a.z)-f64(b.z)) <= 1
	{
		return 1;
	}
	return 0;
}
static_overlap :: proc "contextless" (position: util.Vector3, box: Static_Box) -> int
{
	if f64(position.x)+0.5 >= f64(box.minimum.x) && f64(position.x)-0.5 <= f64(box.maximum.x) &&
		f64(position.y)+0.5 >= f64(box.minimum.y) && f64(position.y)-0.5 <= f64(box.maximum.y) &&
		f64(position.z)+0.5 >= f64(box.minimum.z) && f64(position.z)-0.5 <= f64(box.maximum.z)
	{
		return 1;
	}
	return 0;
}

expected_pairs :: proc(run: ^Run, frame: int) -> int
{
	for &slot in run.expected
	{
		slot = 0;
	}
	count: int;
	gx, gy, gz: int = run.grid[0], run.grid[1], run.grid[2];
	for i in 0 ..< len(run.initial)
	{
		local: int = i % run.group_size;
		x, y, z: int = local%gx, local/(gx*gz), (local/gx)%gz;
		group: int = i/run.group_size;
		a: util.Vector3 = prescribed_position(run, i, frame);
		for dy in -1 ..= 1
		{
			for dz in -1 ..= 1
			{
				for dx in -1 ..= 1
				{
					nx, ny, nz: int = x+dx, y+dy, z+dz;
					if nx < 0 || nx >= gx || ny < 0 || ny >= gy || nz < 0 || nz >= gz
					{
						continue;
					}
					j: int = group*run.group_size+(ny*gz+nz)*gx+nx;
					if j > i && positions_overlap(a, prescribed_position(run, j, frame)) != 0
					{
						slot: int = i*PAIR_SLOTS+(dy+1)*9+(dz+1)*3+dx+1;
						run.expected[slot] = 1;
						count += 1;
					}
				}
			}
		}
		if run.options.fixture == .Islands
		{
			if static_overlap(a, run.statics[group]) != 0
			{
				run.expected[i*PAIR_SLOTS+27] = 1;
				count += 1;
			}
		}
		else
		{
			for box, index in run.statics
			{
				if static_overlap(a, box) != 0
				{
					run.expected[i*PAIR_SLOTS+27+index] = 1;
					count += 1;
				}
			}
		}
	}
	return count;
}

pair_slot :: proc "contextless" (run: ^Run, simulation: ^physics.Simulation, key: u64) -> int
{
	a: physics.Collidable_Reference = {u32(key >> 32)};
	b: physics.Collidable_Reference = {u32(key)};
	if physics.collidable_reference_mobility(a) != .Dynamic
	{
		return -1;
	}
	body_handle: int = int(physics.collidable_reference_raw_handle(a));
	if body_handle < 0 || body_handle >= int(simulation.bodies.handle_to_location.length)
	{
		return -1;
	}
	i: int = int(simulation.bodies.handle_to_location.memory[body_handle].index);
	if i < 0 || i >= len(run.initial)
	{
		return -1;
	}
	if physics.collidable_reference_mobility(b) == .Static
	{
		static_handle: int = int(physics.collidable_reference_raw_handle(b));
		if static_handle < 0 || static_handle >= int(simulation.statics.handle_to_index.length)
		{
			return -1;
		}
		index: int = int(simulation.statics.handle_to_index.memory[static_handle]);
		if run.options.fixture == .Islands
		{
			if index != i/run.group_size
			{
				return -1;
			}
			index = 0;
		}
		if index < 0 || index >= 5
		{
			return -1;
		}
		return i*PAIR_SLOTS+27+index;
	}
	if physics.collidable_reference_mobility(b) != .Dynamic
	{
		return -1;
	}
	other_handle: int = int(physics.collidable_reference_raw_handle(b));
	if other_handle < 0 || other_handle >= int(simulation.bodies.handle_to_location.length)
	{
		return -1;
	}
	j: int = int(simulation.bodies.handle_to_location.memory[other_handle].index);
	if j < i
	{
		i, j = j, i;
	}
	if j <= i || j >= len(run.initial) || i/run.group_size != j/run.group_size
	{
		return -1;
	}
	li, lj: int = i%run.group_size, j%run.group_size;
	gx, gz: int = run.grid[0], run.grid[2];
	dx, dy, dz: int = lj%gx-li%gx, lj/(gx*gz)-li/(gx*gz), (lj/gx)%gz-(li/gx)%gz;
	if abs(dx) > 1 || abs(dy) > 1 || abs(dz) > 1
	{
		return -1;
	}
	return i*PAIR_SLOTS+(dy+1)*9+(dz+1)*3+dx+1;
}

contact_geometry_status :: proc(run: ^Run, simulation: ^physics.Simulation, pair: physics.Collidable_Pair,
	kind: ^physics.Type_Batch, index, frame: int) -> physics.Physics_Status
{
	contact: contacts.Contact_Input = contacts.contact_read(kind, index);
	a_index: int = int(simulation.bodies.handle_to_location.memory[physics.collidable_reference_raw_handle(pair.a)].index);
	a: util.Vector3 = prescribed_position(run, a_index, frame);
	b_min, b_max: util.Vector3;
	if physics.collidable_reference_mobility(pair.b) == .Static
	{
		static_index: int = int(simulation.statics.handle_to_index.memory[physics.collidable_reference_raw_handle(pair.b)]);
		b_min = run.statics[static_index].minimum;
		b_max = run.statics[static_index].maximum;
	}
	else
	{
		b_index: int = int(simulation.bodies.handle_to_location.memory[physics.collidable_reference_raw_handle(pair.b)].index);
		b: util.Vector3 = prescribed_position(run, b_index, frame);
		b_min = {b.x-0.5, b.y-0.5, b.z-0.5};
		b_max = {b.x+0.5, b.y+0.5, b.z+0.5};
	}
	overlaps: [3]f64 = {
		min(f64(a.x)+0.5-f64(b_min.x), f64(b_max.x)-f64(a.x)+0.5),
		min(f64(a.y)+0.5-f64(b_min.y), f64(b_max.y)-f64(a.y)+0.5),
		min(f64(a.z)+0.5-f64(b_min.z), f64(b_max.z)-f64(a.z)+0.5),
	};
	normal: [3]f64 = {f64(contact.normal.x), f64(contact.normal.y), f64(contact.normal.z)};
	center_delta: [3]f64 = {
		2*f64(a.x)-f64(b_min.x)-f64(b_max.x),
		2*f64(a.y)-f64(b_min.y)-f64(b_max.y),
		2*f64(a.z)-f64(b_min.z)-f64(b_max.z),
	};
	if kind.body_count == 2
	{
		for value in ([3]f32{contact.offset_b.x, contact.offset_b.y, contact.offset_b.z})
		{
			if math.is_nan(value) || math.is_inf(value)
			{
				return .Invalid_Argument;
			}
		}
	}
	minimum_overlap: f64 = min(overlaps[0], min(overlaps[1], overlaps[2]));
	axis_count: int;
	for component, axis in normal
	{
		if math.is_nan(component) || math.is_inf(component)
		{
			return .Invalid_Argument;
		}
		if abs(component) > 1e-5
		{
			if abs(abs(component)-1) > 1e-5 || abs(overlaps[axis]-minimum_overlap) > 3e-4 ||
				component*center_delta[axis] < 0
			{
				return .Invalid_Argument;
			}
			axis_count += 1;
		}
	}
	if axis_count != 1
	{
		return .Invalid_Argument;
	}
	for point in 0 ..< int(kind.type_id%4+1)
	{
		offset: util.Vector3 = contact.offsets[point];
		for value in ([3]f32{offset.x, offset.y, offset.z})
		{
			if math.is_nan(value) || math.is_inf(value)
			{
				return .Invalid_Argument;
			}
		}
		depth: f64 = f64(contact.depths[point]);
		if math.is_nan(depth) || math.is_inf(depth) || abs(depth-minimum_overlap) > 3e-4
		{
			return .Invalid_Argument;
		}
	}
	return .Ok;
}

observe :: proc(run: ^Run, simulation: ^physics.Simulation, frame: int) -> physics.Physics_Status
{
	expected: int = expected_pairs(run, frame);
	for &slot in run.seen
	{
		slot = 0;
	}
	actual: int;
	for count, worker in run.pairs.counts
	{
		for index in 0 ..< count
		{
			key: u64 = run.pairs.keys[worker*run.pairs.capacity+index];
			slot: int = pair_slot(run, simulation, key);
			if slot < 0 || run.expected[slot] == 0 || run.seen[slot] != 0
			{
				return .Invalid_Argument;
			}
			run.seen[slot] = 1;
			actual += 1;
		}
	}
	run.last_pairs = actual;
	if actual != expected || actual == 0
	{
		return .Invalid_Argument;
	}
	run.last_manifolds = 0;
	if run.options.stage == .Tree_Update || run.options.stage == .Pair_Traversal
	{
		return .Ok;
	}
	cache: ^physics.Pair_Cache = &simulation.narrow_phase.pair_cache;
	for &slot in run.seen
	{
		slot = 0;
	}
	manifolds, points: int;
	for index in 0 ..< cache.mapping.count
	{
		pair: physics.Collidable_Pair = cache.mapping.keys.memory[index];
		slot: int = pair_slot(run, simulation, pair_key(pair.a, pair.b));
		if slot < 0 || run.expected[slot] == 0 || run.seen[slot] != 0
		{
			return .Invalid_Argument;
		}
		run.seen[slot] = 1;
		location: physics.Constraint_Location;
		status: physics.Physics_Status;
		location, status = physics.solver_resolve(&simulation.solver, cache.mapping.values.memory[index].constraint_handle);
		if status != .Ok || location.type_id < 0 || location.type_id > 7
		{
			return .Invalid_Argument;
		}
		batch: ^physics.Constraint_Batch = &simulation.solver.active_set.batches.memory[location.batch_index];
		kind: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
		if contact_geometry_status(run, simulation, pair, kind, int(location.index_in_type_batch), frame) != .Ok
		{
			return .Invalid_Argument;
		}
		manifolds += 1;
		points += int(kind.type_id%4+1);
	}
	if cache.mapping.count != expected
	{
		return .Invalid_Argument;
	}
	run.last_manifolds = manifolds;
	if frame >= WARMUP_FRAMES
	{
		run.contact_points += i64(points);
	}
	return .Ok;
}
