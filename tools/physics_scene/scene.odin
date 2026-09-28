package physics_scene

import "core:math"
import "core:math/linalg"

DEFAULT_BUDGET :: u64(512*1024*1024);

Status :: enum
{
	Ok, Invalid_Data, Out_Of_Memory, Budget_Exceeded, Unsupported, File_Error, End,
}
Vec3 :: [3]f32;
NO_ENTITY :: max(u32);
Pose :: struct
{
	position: Vec3,
	orientation: quaternion128,
}
IDENTITY :: Pose{orientation=quaternion128(1)};
Geometry_Kind :: enum u32
{
	Marker, Box, Sphere, Capsule, Cylinder, Triangles, Compound,
}
Object_Kind :: enum u32
{
	Body, Static, Shape,
}
Time_Axis :: enum u32
{
	Physics, Operation,
}
Phase :: enum u32
{
	Initial, Warmup, Reset, Active, Completed, Teardown,
}
Availability :: enum u32
{
	Unavailable, Available,
}
Completion :: enum u32
{
	Prefix, Completed,
}
State_Flag :: enum u32
{
	Static, Sleeping, Sensor, Selected, Below_Floor, Query,
}
State :: bit_set[State_Flag; u32];
Geometry_Child :: struct
{
	geometry: u32,
	part: u32,
	pose: Pose,
}
Geometry :: struct
{
	kind: Geometry_Kind,
	size: Vec3,
	minimum, maximum: Vec3,
	vertices: []Vec3,
	indices: []u32,
	children: []Geometry_Child,
}
Entity :: struct
{
	kind: Object_Kind,
	source_handle: i32,
	geometry: u32,
	group: u32,
}
Overlay_Kind :: enum u32
{
	Point, Line, Triangle, Ray_Hit, Ray_Miss, Contact, Constraint, Gravity, Enter, Stay, Exit, Joint_Break,
	Angular_Velocity, Bounds, Query_Path, Contact_Impulse, Part_State,
	Sphere_Volume,
}
Overlay :: struct
{
	kind: Overlay_Kind,
	entity_a, entity_b: u32,
	part_a, part_b: i32,
	a, b, c: Vec3,
	value: f32,
}
Metric_Kind :: enum u32
{
	Active_Bodies, Sleeping_Bodies, Constraints, Pairs, Below_Floor_Centers, Operation_Index, Step_Alpha, Stage_Nanoseconds,
	Substeps, Workers, Timestep_Nanoseconds,
	Step_Accumulator_Seconds, Steps_This_Operation, Total_Physics_Steps,
	Below_Inside_Floor, Below_Outside_Floor, Outside_Floor, Finite_Poses,
	Island_Mean_Radius, Island_Worst_Radius, Island_Worst_Index,
	Apex_Displacement_Y, Prelaunch_Apex_Up, Prelaunch_Apex_Up_Tick, Prelaunch_Apex_Down, Prelaunch_Apex_Down_Tick,
	Prelaunch_Top_Up, Prelaunch_Top_Up_Tick, Prelaunch_Top_Down, Prelaunch_Top_Down_Tick,
	Prelaunch_Top_Displacement, Prelaunch_Top_Displacement_Tick,
	Queries_Per_Batch, Query_Hits, Executed_Batches, Query_Result_Batch,
	Query_Route,
	Result_Checksum,
}
Metric :: struct
{
	kind: Metric_Kind,
	value: f64,
}
Input :: struct
{
	mode: Input_Mode,
	x, y: f32,
}
Input_Mode :: enum u32
{
	Scripted, Live,
}
Frame :: struct
{
	tick: u64,
	cycle: u32,
	phase: Phase,
	input: Input,
	ids: [dynamic]u32,
	poses: [dynamic]Pose,
	states: [dynamic]State,
	overlays: [dynamic]Overlay,
	metrics: [dynamic]Metric,
}
Scene_Packet :: struct
{
	geometries: [dynamic]Geometry,
	entities: [dynamic]Entity,
	frame: Frame,
}
Metadata :: struct
{
	scenario, source, revision, compiler, host, configuration, components, settings: string,
	axis: Time_Axis,
	timestep: f32,
	settings_state, events_state: Availability,
	completion: Completion,
}

geometry_destroy :: proc(g: ^Geometry)
{
	delete(g.vertices);
	delete(g.indices);
	delete(g.children);
	g^ = {};
}

frame_destroy :: proc(f: ^Frame)
{
	delete(f.ids);
	delete(f.poses);
	delete(f.states);
	delete(f.overlays);
	delete(f.metrics);
	f^ = {};
}

packet_destroy :: proc(p: ^Scene_Packet)
{
	for &g in p.geometries
	{
		geometry_destroy(&g);
	}
	delete(p.geometries);
	delete(p.entities);
	frame_destroy(&p.frame);
	p^ = {};
}

// reserve at selection or a topology boundary, never within entity traversal
frame_reserve :: proc(f: ^Frame, instances, overlays, metrics: int) -> Status
{
	if reserve(&f.ids, instances) != nil || reserve(&f.poses, instances) != nil ||
		reserve(&f.states, instances) != nil || reserve(&f.overlays, overlays) != nil || reserve(&f.metrics, metrics) != nil
	{
		return .Out_Of_Memory;
	}
	return .Ok;
}

frame_clear :: proc(f: ^Frame)
{
	clear(&f.ids);
	clear(&f.poses);
	clear(&f.states);
	clear(&f.overlays);
	clear(&f.metrics);
}

pose_transform :: proc(p: Pose, v: Vec3) -> Vec3
{
	return linalg.quaternion_mul_vector3(p.orientation, v) + p.position;
}

pose_compose :: proc(parent, child: Pose) -> Pose
{
	return {pose_transform(parent, child.position), parent.orientation * child.orientation};
}

finite :: proc(value: f32) -> Status
{
	return .Invalid_Data if transmute(u32)value & 0x7f80_0000 == 0x7f80_0000 else .Ok;
}

pose_admit :: proc(p: Pose) -> Status
{
	for v in p.position
	{
		if finite(v) != .Ok
		{
			return .Invalid_Data;
		}
	}
	q: quaternion128 = p.orientation;
	length: f32 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
	if finite(length) != .Ok || length < 0.99 || length > 1.01
	{
		return .Invalid_Data;
	}
	return .Ok;
}

// children precede their parents, so external geometry cannot introduce cycles
geometry_admit :: proc(g: ^Geometry, geometry_index: int) -> Status
{
	if u32(g.kind) > u32(Geometry_Kind.Compound)
	{
		return .Invalid_Data;
	}
	for axis in 0 ..< 3
	{
		if finite(g.size[axis]) != .Ok || finite(g.minimum[axis]) != .Ok || finite(g.maximum[axis]) != .Ok || g.minimum[axis] > g.maximum[axis]
		{
			return .Invalid_Data;
		}
	}
	switch g.kind
	{
	case .Box:
		if g.size.x <= 0 || g.size.y <= 0 || g.size.z <= 0
		{
			return .Invalid_Data;
		}
	case .Sphere:
		if g.size.x <= 0
		{
			return .Invalid_Data;
		}
	case .Capsule, .Cylinder:
		if g.size.x <= 0 || g.size.y < 0
		{
			return .Invalid_Data;
		}
	case .Triangles:
		if len(g.vertices) < 3 || len(g.indices) == 0 || len(g.indices)%3 != 0
		{
			return .Invalid_Data;
		}
	case .Compound:
		if len(g.children) == 0
		{
			return .Invalid_Data;
		}
	case .Marker:
	}
	for v in g.vertices
	{
		for component in v
		{
			if finite(component) != .Ok
			{
				return .Invalid_Data;
			}
		}
	}
	for index in g.indices
	{
		if u64(index) >= u64(len(g.vertices))
		{
			return .Invalid_Data;
		}
	}
	for child in g.children
	{
		if u64(child.geometry) >= u64(geometry_index) || pose_admit(child.pose) != .Ok
		{
			return .Invalid_Data;
		}
	}
	return .Ok;
}

frame_admit :: proc(f: ^Frame, entities: []Entity, geometries: []Geometry) -> Status
{
	if len(f.ids) != len(f.poses) || len(f.ids) != len(f.states) ||
		u32(f.phase) > u32(Phase.Teardown) || u32(f.input.mode) > u32(Input_Mode.Live) ||
		finite(f.input.x) != .Ok || finite(f.input.y) != .Ok
	{
		return .Invalid_Data;
	}
	previous: i64 = -1;
	for id, index in f.ids
	{
		if i64(id) <= previous || u64(id) >= u64(len(entities)) || pose_admit(f.poses[index]) != .Ok || transmute(u32)f.states[index] & ~u32(63) != 0
		{
			return .Invalid_Data;
		}
		previous = i64(id);
	}
	for o in f.overlays
	{
		if u32(o.kind) > u32(Overlay_Kind.Sphere_Volume) || finite(o.value) != .Ok || (o.kind == .Sphere_Volume && o.value <= 0)
		{
			return .Invalid_Data;
		}
		if (o.entity_a != NO_ENTITY && u64(o.entity_a) >= u64(len(entities))) ||
			(o.entity_b != NO_ENTITY && u64(o.entity_b) >= u64(len(entities)))
		{
			return .Invalid_Data;
		}
		if o.kind == .Part_State
		{
			if o.entity_a == NO_ENTITY || o.part_a < 0 || (o.value != 0 && o.value != 1)
			{
				return .Invalid_Data;
			}
			if u64(entities[o.entity_a].geometry) >= u64(len(geometries))
			{
				return .Invalid_Data;
			}
			geometry: ^Geometry = &geometries[entities[o.entity_a].geometry];
			part_state: Availability;
			if len(geometry.children) == 0 && o.part_a == 0
			{
				part_state = .Available;
			}
			for child in geometry.children
			{
				if child.part == u32(o.part_a)
				{
					part_state = .Available;
					break;
				}
			}
			if part_state == .Unavailable
			{
				return .Invalid_Data;
			}
		}
		points: [3]Vec3 = {o.a, o.b, o.c};
		for v in points
		{
			for c in v
			{
				if finite(c) != .Ok
				{
					return .Invalid_Data;
				}
			}
		}
	}
	for metric in f.metrics
	{
		if u32(metric.kind) > u32(Metric_Kind.Result_Checksum) || math.is_nan(metric.value) || math.is_inf(metric.value)
		{
			return .Invalid_Data;
		}
	}
	return .Ok;
}

geometry_bytes :: proc(g: ^Geometry) -> u64
{
	return u64(size_of(Geometry) + len(g.vertices)*size_of(Vec3) + len(g.indices)*size_of(u32) + len(g.children)*size_of(Geometry_Child));
}

frame_bytes :: proc(f: ^Frame) -> u64
{
	return u64(cap(f.ids)*size_of(u32) + cap(f.poses)*size_of(Pose) + cap(f.states)*size_of(State) + cap(f.overlays)*size_of(Overlay) + cap(f.metrics)*size_of(Metric));
}

packet_bytes :: proc(p: ^Scene_Packet) -> u64
{
	total: u64 = frame_bytes(&p.frame) + u64(cap(p.entities)*size_of(Entity) + cap(p.geometries)*size_of(Geometry));
	for &g in p.geometries
	{
		total += geometry_bytes(&g)-size_of(Geometry);
	}
	return total;
}
