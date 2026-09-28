package physics_replay

import "core:encoding/endian"
import "core:strings"
import scene "../physics_scene"

Codec :: struct
{
	bytes: []u8,
	at: int,
	status: scene.Status,
}

encode_u32 :: proc(c: ^Codec, value: u32)
{
	if c.at > len(c.bytes)-4
	{
		c.status = .Invalid_Data;
		return;
	}
	endian.put_u32(c.bytes[c.at:c.at+4], .Little, value);
	c.at += 4;
}

encode_u64 :: proc(c: ^Codec, value: u64)
{
	if c.at > len(c.bytes)-8
	{
		c.status = .Invalid_Data;
		return;
	}
	endian.put_u64(c.bytes[c.at:c.at+8], .Little, value);
	c.at += 8;
}

decode_u32 :: proc(c: ^Codec) -> u32
{
	if c.at > len(c.bytes)-4
	{
		c.status = .Invalid_Data;
		return 0;
	}
	value: u32;
	value, _ = endian.get_u32(c.bytes[c.at:c.at+4], .Little);
	c.at += 4;
	return value;
}

decode_u64 :: proc(c: ^Codec) -> u64
{
	if c.at > len(c.bytes)-8
	{
		c.status = .Invalid_Data;
		return 0;
	}
	value: u64;
	value, _ = endian.get_u64(c.bytes[c.at:c.at+8], .Little);
	c.at += 8;
	return value;
}

encode_float :: proc(c: ^Codec, value: f32)
{
	encode_u32(c, transmute(u32)value);
}
decode_float :: proc(c: ^Codec) -> f32
{
	return transmute(f32)decode_u32(c);
}

encode_vector :: proc(c: ^Codec, v: scene.Vec3)
{
	for value in v
	{
		encode_float(c, value);
	}
}

decode_vector :: proc(c: ^Codec) -> scene.Vec3
{
	x: f32 = decode_float(c);
	y: f32 = decode_float(c);
	z: f32 = decode_float(c);
	return {x, y, z};
}

encode_pose :: proc(c: ^Codec, p: scene.Pose)
{
	encode_vector(c, p.position);
	encode_float(c, p.orientation.x);
	encode_float(c, p.orientation.y);
	encode_float(c, p.orientation.z);
	encode_float(c, p.orientation.w);
}

decode_pose :: proc(c: ^Codec) -> scene.Pose
{
	p: scene.Pose;
	p.position = decode_vector(c);
	x: f32 = decode_float(c);
	y: f32 = decode_float(c);
	z: f32 = decode_float(c);
	w: f32 = decode_float(c);
	p.orientation = quaternion(x=x, y=y, z=z, w=w);
	return p;
}

encode_text :: proc(c: ^Codec, text: string)
{
	encode_u32(c, u32(len(text)));
	if len(text) > len(c.bytes)-c.at
	{
		c.status = .Invalid_Data;
		return;
	}
	copy(c.bytes[c.at:], text);
	c.at += len(text);
}

decode_text :: proc(c: ^Codec) -> string
{
	length: int = int(decode_u32(c));
	if length > len(c.bytes)-c.at
	{
		c.status = .Invalid_Data;
		return "";
	}
	text: string = strings.clone(string(c.bytes[c.at:c.at+length]));
	c.at += length;
	return text;
}

metadata_destroy :: proc(m: ^scene.Metadata)
{
	delete(m.scenario);
	delete(m.source);
	delete(m.revision);
	delete(m.compiler);
	delete(m.host);
	delete(m.configuration);
	delete(m.components);
	delete(m.settings);
	m^ = {};
}

encode_metadata :: proc(c: ^Codec, m: scene.Metadata)
{
	encode_text(c, m.scenario);
	encode_text(c, m.source);
	encode_text(c, m.revision);
	encode_text(c, m.compiler);
	encode_text(c, m.host);
	encode_text(c, m.configuration);
	encode_text(c, m.components);
	encode_text(c, m.settings);
	encode_u32(c, u32(m.axis));
	encode_float(c, m.timestep);
	encode_u32(c, u32(m.settings_state));
	encode_u32(c, u32(m.events_state));
}

decode_metadata :: proc(c: ^Codec) -> scene.Metadata
{
	m: scene.Metadata;
	m.scenario = decode_text(c);
	m.source = decode_text(c);
	m.revision = decode_text(c);
	m.compiler = decode_text(c);
	m.host = decode_text(c);
	m.configuration = decode_text(c);
	m.components = decode_text(c);
	m.settings = decode_text(c);
	m.axis = scene.Time_Axis(decode_u32(c));
	m.timestep = decode_float(c);
	m.settings_state = scene.Availability(decode_u32(c));
	m.events_state = scene.Availability(decode_u32(c));
	if len(m.scenario) == 0 || u32(m.axis) > 1 || u32(m.settings_state) > 1 || u32(m.events_state) > 1 ||
		scene.finite(m.timestep) != .Ok || (m.axis == .Physics && m.timestep <= 0)
	{
		c.status = .Invalid_Data;
	}
	return m;
}

geometry_encoded_size :: proc(g: ^scene.Geometry) -> int
{
	return 52 + len(g.vertices)*12 + len(g.indices)*4 + len(g.children)*36;
}

encode_geometry :: proc(c: ^Codec, g: ^scene.Geometry)
{
	encode_u32(c, u32(g.kind));
	encode_vector(c, g.size);
	encode_vector(c, g.minimum);
	encode_vector(c, g.maximum);
	encode_u32(c, u32(len(g.vertices)));
	encode_u32(c, u32(len(g.indices)));
	encode_u32(c, u32(len(g.children)));
	for vertex in g.vertices
	{
		encode_vector(c, vertex);
	}
	for index in g.indices
	{
		encode_u32(c, index);
	}
	for child in g.children
	{
		encode_u32(c, child.geometry);
		encode_u32(c, child.part);
		encode_pose(c, child.pose);
	}
}

decode_geometry :: proc(c: ^Codec) -> scene.Geometry
{
	g: scene.Geometry;
	g.kind = scene.Geometry_Kind(decode_u32(c));
	g.size = decode_vector(c);
	g.minimum = decode_vector(c);
	g.maximum = decode_vector(c);
	vertices: int = int(decode_u32(c));
	indices: int = int(decode_u32(c));
	children: int = int(decode_u32(c));
	if vertices*12+indices*4+children*36 > len(c.bytes)-c.at
	{
		c.status = .Invalid_Data;
		return g;
	}
	g.vertices = make([]scene.Vec3, vertices);
	g.indices = make([]u32, indices);
	g.children = make([]scene.Geometry_Child, children);
	for &vertex in g.vertices
	{
		vertex = decode_vector(c);
	}
	for &index in g.indices
	{
		index = decode_u32(c);
	}
	for &child in g.children
	{
		child.geometry = decode_u32(c);
		child.part = decode_u32(c);
		child.pose = decode_pose(c);
	}
	return g;
}

frame_encoded_size :: proc(f: ^scene.Frame) -> int
{
	return 40 + len(f.ids)*36 + len(f.overlays)*60 + len(f.metrics)*12;
}

encode_frame :: proc(c: ^Codec, f: ^scene.Frame)
{
	encode_u64(c, f.tick);
	encode_u32(c, f.cycle);
	encode_u32(c, u32(f.phase));
	encode_u32(c, u32(f.input.mode));
	encode_float(c, f.input.x);
	encode_float(c, f.input.y);
	encode_u32(c, u32(len(f.ids)));
	encode_u32(c, u32(len(f.overlays)));
	encode_u32(c, u32(len(f.metrics)));
	for id, i in f.ids
	{
		encode_u32(c, id);
		encode_pose(c, f.poses[i]);
		encode_u32(c, transmute(u32)f.states[i]);
	}
	for o in f.overlays
	{
		encode_u32(c, u32(o.kind));
		encode_u32(c, o.entity_a);
		encode_u32(c, o.entity_b);
		encode_u32(c, u32(o.part_a));
		encode_u32(c, u32(o.part_b));
		encode_vector(c, o.a);
		encode_vector(c, o.b);
		encode_vector(c, o.c);
		encode_float(c, o.value);
	}
	for m in f.metrics
	{
		encode_u32(c, u32(m.kind));
		encode_u64(c, transmute(u64)m.value);
	}
}

decode_frame :: proc(c: ^Codec, f: ^scene.Frame) -> scene.Status
{
	f.tick = decode_u64(c);
	f.cycle = decode_u32(c);
	f.phase = scene.Phase(decode_u32(c));
	f.input.mode = scene.Input_Mode(decode_u32(c));
	f.input.x = decode_float(c);
	f.input.y = decode_float(c);
	count: int = int(decode_u32(c));
	overlays: int = int(decode_u32(c));
	metrics: int = int(decode_u32(c));
	if count*36 + overlays*60 + metrics*12 != len(c.bytes)-c.at
	{
		return .Invalid_Data;
	}
	if scene.frame_reserve(f, count, overlays, metrics) != .Ok
	{
		return .Out_Of_Memory;
	}
	scene.frame_clear(f);
	for _ in 0 ..< count
	{
		append(&f.ids, decode_u32(c));
		append(&f.poses, decode_pose(c));
		append(&f.states, transmute(scene.State)decode_u32(c));
	}
	for _ in 0 ..< overlays
	{
		o: scene.Overlay;
		o.kind = scene.Overlay_Kind(decode_u32(c));
		o.entity_a = decode_u32(c);
		o.entity_b = decode_u32(c);
		o.part_a = i32(decode_u32(c));
		o.part_b = i32(decode_u32(c));
		o.a = decode_vector(c);
		o.b = decode_vector(c);
		o.c = decode_vector(c);
		o.value = decode_float(c);
		append(&f.overlays, o);
	}
	for _ in 0 ..< metrics
	{
		m: scene.Metric;
		m.kind = scene.Metric_Kind(decode_u32(c));
		m.value = transmute(f64)decode_u64(c);
		append(&f.metrics, m);
	}
	return c.status;
}
