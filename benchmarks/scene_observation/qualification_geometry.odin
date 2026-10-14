package scene_observation

import "core:math"
import entasis "entasis:entasis"

Fixture :: enum u8
{
	Container,
	Contact_Islands,
	Pyramid,
	Pyramid_Wall,
}
Geometry :: enum u8
{
	Authored_Boxes,
	Unsupported,
}
Eligibility :: enum u8
{
	Supported,
	Unsupported,
}
Fixture_Configuration :: struct
{
	fixture: Fixture,
	geometry: Geometry,
	half_extent: [3]f64,
	grid: [3]int,
	spacing: [3]f64,
	island_grid: [2]int,
	island_spacing: [2]f64,
	floor_half_size: [2]f64,
	floor_top: f64,
	spawn_height: f64,
	rows: int,
	horizontal_spacing: f64,
	container_half_size: [2]f64,
	container_rim: f64,
	population: int,
	body_count: int,
	projectile_radius: f64,
	steps: int,
	warmup_steps: int,
	timestep_hz: int,
}
Neighbor :: struct
{
	body: int,
	axis: int,
}
Box_Reference :: struct
{
	lower: [3]f64,
	upper: [3]f64,
	neighbors: [6]Neighbor,
	neighbor_count: int,
	support_count: int,
}

fixture_eligibility :: proc(c: Fixture_Configuration) -> Eligibility
{
	if c.geometry != .Authored_Boxes
	{
		return .Unsupported
	}
	h: [3]f64 = c.half_extent
	switch c.fixture
	{
		case .Container:
		if h[0] != h[1] || h[0] != h[2] ||
			c.container_half_size[0] <= h[0] || c.container_half_size[1] <= h[2] || c.container_rim <= c.floor_top + 2*h[1]
		{
			return .Unsupported
		}
		case .Contact_Islands:
		if c.spawn_height != c.floor_top+h[1]
		{
			return .Unsupported
		}
		for axis: int = 0; axis < 2; axis += 1
		{
			box_axis: int = 2*axis
			width: f64 = max(f64(c.grid[box_axis]-1)*c.spacing[box_axis], f64(c.grid[box_axis]-1)*2*h[box_axis]) + 2*h[box_axis]
			if width > 2*c.floor_half_size[axis] || (c.island_grid[axis] > 1 && c.island_spacing[axis] < 2*c.floor_half_size[axis])
			{
				return .Unsupported
			}
		}
		case .Pyramid, .Pyramid_Wall:
		if c.horizontal_spacing != 2*h[0] ||
			(c.fixture == .Pyramid && c.horizontal_spacing != 2*h[2]) ||
			f64(c.rows)*h[0] > c.floor_half_size[0] ||
			(c.fixture == .Pyramid && f64(c.rows)*h[2] > c.floor_half_size[1])
		{
			return .Unsupported
		}
	}
	return .Supported
}

projected_box_radius :: proc(state: entasis.Body_State, h: [3]f64) -> ([3]f64, State_Status)
{
	if state_status(state) != .Valid
	{
		return {}, .Invalid
	}
	q: entasis.Quaternion = state.pose.orientation
	x, y, z, w: f64 = f64(q.x), f64(q.y), f64(q.z), f64(q.w)
	length_squared: f64 = x*x + y*y + z*z + w*w
	if length_squared == 0
	{
		return {}, .Invalid
	}
	inverse: f64 = 1/math.sqrt(length_squared)
	x *= inverse
	y *= inverse
	z *= inverse
	w *= inverse
	return {
		math.abs(1-2*(y*y+z*z))*h[0] + math.abs(2*(x*y-w*z))*h[1] + math.abs(2*(x*z+w*y))*h[2],
		math.abs(2*(x*y+w*z))*h[0] + math.abs(1-2*(x*x+z*z))*h[1] + math.abs(2*(y*z-w*x))*h[2],
		math.abs(2*(x*z-w*y))*h[0] + math.abs(2*(y*z+w*x))*h[1] + math.abs(1-2*(x*x+y*y))*h[2],
	}, .Valid
}

position_components :: proc(p: entasis.Vector3) -> [3]f64
{
	return {f64(p.x), f64(p.y), f64(p.z)}
}

box_slot_excess :: proc(p, r, h: [3]f64, reference: Box_Reference) -> f64
{
	value: f64
	for axis: int = 0; axis < 3; axis += 1
	{
		value = max(value, reference.lower[axis]-h[axis]-p[axis]+r[axis], p[axis]+r[axis]-reference.upper[axis]-h[axis])
	}
	return value
}

assigned_compression :: proc(p, r, neighbor_p, neighbor_r: [3]f64, axis: int) -> f64
{
	return max(0, r[axis]+neighbor_r[axis]-(neighbor_p[axis]-p[axis]))
}

possible_projectile_impact :: proc(previous_relative, current_relative: [3]f64, radius: f64) -> State_Status
{
	d: [3]f64 = current_relative-previous_relative
	length_squared, dot: f64
	for axis: int = 0; axis < 3; axis += 1
	{
		if min(previous_relative[axis], current_relative[axis]) > radius || max(previous_relative[axis], current_relative[axis]) < -radius
		{
			return .Valid
		}
		length_squared += d[axis]*d[axis]
		dot += previous_relative[axis]*d[axis]
	}
	t: f64
	if length_squared > 0
	{
		t = clamp(-dot/length_squared, 0, 1)
	}
	closest: [3]f64 = previous_relative+t*d
	if closest[0]*closest[0]+closest[1]*closest[1]+closest[2]*closest[2] <= radius*radius
	{
		return .Invalid
	}
	return .Valid
}

build_box_references :: proc(c: Fixture_Configuration, states: []entasis.Body_State, references: []Box_Reference)
{
	for &reference, index in references
	{
		p: [3]f64 = position_components(states[index].pose.position)
		reference = {lower=p, upper=p}
	}
	if c.fixture == .Contact_Islands
	{
		gx, gy, gz: int = c.grid[0], c.grid[1], c.grid[2]
		group_count: int = gx*gy*gz
		for &reference, index in references
		{
			local: int = index%group_count
			x, y, z: int = local%gx, local/(gx*gz), (local/gx)%gz
			coordinates: [3]f64 = {f64(x)-0.5*f64(gx-1), f64(y), f64(z)-0.5*f64(gz-1)}
			group: int = index/group_count
			floor: [3]f64 = {
				f64(f32(c.island_spacing[0])*(f32(group%c.island_grid[0])-0.5*f32(c.island_grid[0]-1))),
				c.floor_top+c.half_extent[1],
				f64(f32(c.island_spacing[1])*(f32(group/c.island_grid[0])-0.5*f32(c.island_grid[1]-1))),
			}
			for axis: int = 0; axis < 3; axis += 1
			{
				touching: f64 = floor[axis]+coordinates[axis]*2*c.half_extent[axis]
				reference.lower[axis] = min(reference.lower[axis], touching)
				reference.upper[axis] = max(reference.upper[axis], touching)
			}
			if x+1 < gx
			{
				reference.neighbors[reference.neighbor_count] = {index+1, 0}
				reference.neighbor_count += 1
			}
			if y+1 < gy
			{
				reference.neighbors[reference.neighbor_count] = {index+gx*gz, 1}
				reference.neighbor_count += 1
			}
			if z+1 < gz
			{
				reference.neighbors[reference.neighbor_count] = {index+gx, 2}
				reference.neighbor_count += 1
			}
		}
	}
	if c.fixture == .Pyramid || c.fixture == .Pyramid_Wall
	{
		layer_start, previous_start: int
		for layer: int = 0; layer < c.rows; layer += 1
		{
			side: int = c.rows-layer
			depth_count: int = side if c.fixture == .Pyramid else 1
			for depth: int = 0; depth < depth_count; depth += 1
			{
				for column: int = 0; column < side; column += 1
				{
					index: int = layer_start+depth*side+column
					reference: ^Box_Reference = &references[index]
					if column+1 < side
					{
						reference.neighbors[reference.neighbor_count] = {index+1, 0}
						reference.neighbor_count += 1
					}
					if depth+1 < depth_count
					{
						reference.neighbors[reference.neighbor_count] = {index+side, 2}
						reference.neighbor_count += 1
					}
					if layer > 0
					{
						support_depths: int = 2 if c.fixture == .Pyramid else 1
						for dz: int = 0; dz < support_depths; dz += 1
						{
							for dx: int = 0; dx < 2; dx += 1
							{
								support: int = previous_start+(depth+dz)*(side+1)+column+dx
								if c.fixture == .Pyramid_Wall
								{
									support = previous_start+column+dx
								}
								reference.neighbors[reference.neighbor_count] = {support, 1}
								reference.neighbor_count += 1
								reference.support_count += 1
							}
						}
					}
				}
			}
			previous_start = layer_start
			layer_start += side*depth_count
		}
	}
}
