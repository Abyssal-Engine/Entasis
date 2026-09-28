package physics_replay

import "core:strings"
import scene "../physics_scene"

reader_open_v1 :: proc(r: ^Reader, size: u64) -> scene.Status
{
	bytes: []u8;
	status: scene.Status;
	bytes, status = read_region(r, 0, 96);
	if status != .Ok
	{
		return status;
	}
	c: Codec = {bytes=bytes, at=12};
	h: ^Header = &r.v1;
	h.magic = MAGIC;
	h.version = 1;
	h.scene = Scene(decode_u32(&c));
	h.body_count = decode_u32(&c);
	h.static_count = decode_u32(&c);
	h.frame_count = decode_u32(&c);
	h.workers = decode_u32(&c);
	h.timestep_hz = decode_u32(&c);
	h.measured_start = decode_u32(&c);
	copy(h.source[:], bytes[c.at:c.at+40]);
	c.at += 40;
	h.box_size = decode_vector(&c);
	h.below_floor_y = decode_float(&c);
	if u32(h.scene) > 2 || h.body_count == 0 || h.body_count > 1_000_000 || h.static_count == 0 || h.static_count > 10_000 ||
		h.frame_count == 0 || h.frame_count > 100_000 || h.workers == 0 || h.workers > 64 || h.timestep_hz == 0 || h.timestep_hz > 1000 ||
		h.measured_start >= h.frame_count || scene.finite(h.below_floor_y) != .Ok || h.source[0] == 0
	{
		return .Invalid_Data;
	}
	for value in h.source
	{
		if value != 0 && (value < 32 || value > 126)
		{
			return .Invalid_Data;
		}
	}
	for value in h.box_size
	{
		if value <= 0 || scene.finite(value) != .Ok
		{
			return .Invalid_Data;
		}
	}
	if (h.scene == .Container && h.static_count != 5) || (h.scene == .Pyramid && h.static_count != 1) ||
		(h.scene == .Contact_Islands && h.body_count%h.static_count != 0)
	{
			return .Invalid_Data;
	}
	static_offset: u64 = 96;
	if h.scene == .Pyramid
	{
		pyramid_bytes: []u8;
		pyramid_status: scene.Status;
		pyramid_bytes, pyramid_status = read_region(r, 96, 12);
		if pyramid_status != .Ok
		{
			return pyramid_status;
		}
		c = {bytes=pyramid_bytes};
		r.v1_pyramid = {decode_u32(&c), decode_float(&c), decode_u32(&c)};
		p: Pyramid_Description = r.v1_pyramid;
		if p.box_count == 0 || p.box_count > h.body_count || scene.finite(p.projectile_radius) != .Ok || p.projectile_radius <= 0 ||
			(p.box_count < h.body_count && p.launch_step >= h.frame_count-1)
		{
				return .Invalid_Data;
		}
		static_offset += 12;
	}
	r.v1_offset = static_offset + u64(h.static_count)*24;
	r.v1_stride = 16 + u64(h.body_count)*28;
	if size != r.v1_offset + u64(h.frame_count)*r.v1_stride
	{
		return .Invalid_Data;
	}
	count: int = int(h.body_count+h.static_count);
	required: u64 = u64(count)*(size_of(scene.Entity)+2*(size_of(scene.Pose)+8)) + u64(h.frame_count)*size_of(Index_Entry) +
		u64(h.static_count+2)*size_of(scene.Geometry) + max(r.v1_stride, u64(h.static_count)*24) + reader_bytes(r) + 6*size_of(scene.Metric) + 128;
	r.required_bytes = required;
	if required > r.budget
	{
		return .Budget_Exceeded;
	}
	names: [3]string = {"benchmark/container", "benchmark/contact_islands", "benchmark/pyramid"};
	r.metadata = {scenario=strings.clone(names[int(h.scene)]), source=strings.clone(source_text(h)), axis=.Physics, timestep=1/f32(h.timestep_hz)};
	r.metadata_bytes = u64(len(r.metadata.scenario)+len(r.metadata.source));
	r.index = make([]Index_Entry, int(h.frame_count));
	for &entry, index in r.index
	{
		entry.tick = u64(index);
		entry.phase = .Initial if index == 0 else (.Warmup if index <= int(h.measured_start) else .Active);
	}
	reserve(&r.packet.geometries, int(h.static_count)+2);
	reserve(&r.packet.entities, count);
	append(&r.packet.geometries, scene.Geometry{kind=.Box, size=h.box_size, minimum=-h.box_size/2, maximum=h.box_size/2});
	radius: f32 = r.v1_pyramid.projectile_radius;
	if h.scene == .Pyramid
	{
		append(&r.packet.geometries, scene.Geometry{kind=.Sphere, size={radius, 0, 0}, minimum={-radius, -radius, -radius}, maximum={radius, radius, radius}});
	}
	for i in 0 ..< int(h.body_count)
	{
		geometry: u32;
		if h.scene == .Pyramid && i >= int(r.v1_pyramid.box_count)
		{
			geometry = 1;
		}
		group: u32;
		if h.scene == .Contact_Islands
		{
			group = u32(i)/(h.body_count/h.static_count);
		}
		append(&r.packet.entities, scene.Entity{.Body, i32(i), geometry, group});
	}
	statics: []u8;
	static_status: scene.Status;
	statics, static_status = read_region(r, static_offset, u64(h.static_count)*24);
	if static_status != .Ok
	{
		return static_status;
	}
	c = {bytes=statics};
	scene.frame_reserve(&r.packet.frame, count, 0, 3);
	scene.frame_reserve(&r.spare, count, 0, 3);
	for i in 0 ..< int(h.static_count)
	{
		position: scene.Vec3 = decode_vector(&c);
		size: scene.Vec3 = decode_vector(&c);
		g: scene.Geometry = {kind=.Box, size=size, minimum=-size/2, maximum=size/2};
		if scene.geometry_admit(&g, len(r.packet.geometries)) != .Ok
		{
			return .Invalid_Data;
		}
		for v in position
		{
			if scene.finite(v) != .Ok
			{
				return .Invalid_Data;
			}
		}
		append(&r.packet.entities, scene.Entity{.Static, i32(i), u32(len(r.packet.geometries)), u32(i)});
		append(&r.packet.geometries, g);
		// static poses are retained at the tail of both pre-reserved frame buffers
	}
	r.owned_bytes = reader_bytes(r);
	return .Ok;
}

reader_prepare_v1 :: proc(r: ^Reader, index: int) -> scene.Status
{
	r.required_bytes = reader_bytes(r);
	if r.required_bytes > r.budget
	{
		return .Budget_Exceeded;
	}
	bytes: []u8;
	status: scene.Status;
	bytes, status = read_region(r, r.v1_offset+u64(index)*r.v1_stride, r.v1_stride);
	if status != .Ok
	{
		return status;
	}
	c: Codec = {bytes=bytes};
	tick: u32 = decode_u32(&c);
	constraints: u32 = decode_u32(&c);
	pairs: u32 = decode_u32(&c);
	below: u32 = decode_u32(&c);
	if tick != u32(index)
	{
		return .Invalid_Data;
	}
	f: ^scene.Frame = &r.spare;
	scene.frame_clear(f);
	f.tick, f.phase = u64(index), r.index[index].phase;
	observed_below: u32;
	for i in 0 ..< int(r.v1.body_count)
	{
		p: scene.Pose = decode_pose(&c);
		state: scene.State;
		if p.position.y < r.v1.below_floor_y
		{
			observed_below += 1;
			state = {.Below_Floor};
		}
		append(&f.ids, u32(i));
		append(&f.poses, p);
		append(&f.states, state);
	}
	if observed_below != below
	{
		return .Invalid_Data;
	}
	static_offset: u64 = 96;
	if r.v1.scene == .Pyramid
	{
		static_offset += 12;
	}
	statics: []u8;
	static_status: scene.Status;
	statics, static_status = read_region(r, static_offset, u64(r.v1.static_count)*24);
	if static_status != .Ok
	{
		return static_status;
	}
	c = {bytes=statics};
	for i in 0 ..< int(r.v1.static_count)
	{
		p: scene.Pose = scene.IDENTITY;
		p.position = decode_vector(&c);
		decode_vector(&c);
		append(&f.ids, u32(i)+r.v1.body_count);
		append(&f.poses, p);
		append(&f.states, scene.State{.Static});
	}
	append(&f.metrics, scene.Metric{.Constraints, f64(constraints)}, scene.Metric{.Pairs, f64(pairs)}, scene.Metric{.Below_Floor_Centers, f64(below)});
	if scene.frame_admit(f, r.packet.entities[:], r.packet.geometries[:]) != .Ok
	{
		return .Invalid_Data;
	}
	r.prepared = index;
	r.owned_bytes = reader_bytes(r);
	return .Ok;
}
