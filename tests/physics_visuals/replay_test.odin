package physics_visuals

import "core:testing"
import "core:fmt"
import "core:os"
import "core:time"
import "core:mem"
import scene "../../tools/physics_scene"
import scenarios "../../tools/physics_scenarios"
import replay "../../tools/physics_replay"

@(test)
visual_replay_roundtrip_and_random_seek :: proc(t: ^testing.T)
{
	for compression in replay.Compression
	{
		visual_replay_roundtrip(t, compression);
		visual_replay_topology(t, compression);
	}
	visual_replay_v2_fixture(t);
}

visual_replay_topology :: proc(t: ^testing.T, compression: replay.Compression)
{
	root: string;
	error: os.Error;
	root, error = os.temp_directory(context.allocator);
	testing.expect(t, error == nil);
	defer delete(root);
	path: string = fmt.aprintf("%s/entasis-topology-%d.replay", root, time.tick_now());
	partial: string = fmt.aprintf("%s.partial", path);
	defer delete(path);
	defer delete(partial);
	defer os.remove(path);
	defer os.remove(partial);
	p: scene.Scene_Packet;
	defer scene.packet_destroy(&p);
	append(&p.geometries, scene.Geometry{kind=.Box, size={1, 2, 3}, minimum={-0.5, -1, -1.5}, maximum={0.5, 1, 1.5}});
	append(&p.entities, scene.Entity{kind=.Body, source_handle=7, geometry=0});
	scene.frame_reserve(&p.frame, 4096, 2, 1);
	append(&p.frame.ids, 0);
	append(&p.frame.poses, scene.IDENTITY);
	append(&p.frame.states, scene.State{.Sleeping});
	p.frame.tick = 19;
	w: replay.Writer;
	defer replay.writer_close(&w);
	testing.expect_value(t, replay.writer_open(&w, path, {scenario="example/topology", axis=.Physics, timestep=1.0/60}, compression), scene.Status.Ok);
	testing.expect_value(t, replay.writer_frame(&w, &p), scene.Status.Ok);
	children: []scene.Geometry_Child = make([]scene.Geometry_Child, 2);
	children[0] = {0, 3, {position={2, 3, 4}, orientation=quaternion128(1)}};
	children[1] = {0, 8, {position={-2, -3, -4}, orientation=quaternion(x=f32(1), y=f32(0), z=f32(0), w=f32(0))}};
	append(&p.geometries, scene.Geometry{kind=.Compound, minimum={-3, -5, -6}, maximum={3, 5, 6}, children=children});
	scene.frame_clear(&p.frame);
	p.frame.cycle, p.frame.tick, p.frame.phase = 1, 0, .Reset;
	p.frame.input = {.Live, -0.75, 0.375};
	random: u32 = 0x75319531;
	for index in 0 ..< 4096
	{
		append(&p.entities, scene.Entity{kind=.Body, source_handle=i32(index), geometry=1});
		append(&p.frame.ids, u32(index+1));
		pose: scene.Pose = scene.IDENTITY;
		for axis in 0 ..< 3
		{
			random = random*1664525+1013904223;
			pose.position[axis] = transmute(f32)(u32(0x3f000000)|(random&0x007fffff));
		}
		if index == 0
		{
			pose.position.z = transmute(f32)u32(0x80000000);
		}
		append(&p.frame.poses, pose);
		append(&p.frame.states, scene.State{.Query});
	}
	append(&p.frame.overlays, scene.Overlay{kind=.Enter, entity_a=1, entity_b=2, part_a=3, part_b=8, a={2, 3, 4}, value=0.125});
	append(&p.frame.overlays, scene.Overlay{kind=.Part_State, entity_a=1, entity_b=scene.NO_ENTITY, part_a=7, value=1});
	testing.expect_value(t, scene.frame_admit(&p.frame, p.entities[:], p.geometries[:]), scene.Status.Invalid_Data);
	p.frame.overlays[1].part_a = 8;
	testing.expect_value(t, scene.frame_admit(&p.frame, p.entities[:], p.geometries[:]), scene.Status.Ok);
	append(&p.frame.metrics, scene.Metric{.Step_Alpha, 0.125});
	testing.expect_value(t, replay.writer_frame(&w, &p), scene.Status.Ok);
	testing.expect_value(t, replay.writer_finish(&w, .Prefix), scene.Status.Ok);
	r: replay.Reader;
	defer replay.reader_close(&r);
	status: scene.Status = replay.reader_open(&r, path, 8*1024*1024);
	testing.expect_value(t, status, scene.Status.Ok);
	if status != .Ok
	{
		return;
	}
	testing.expect_value(t, replay.reader_seek(&r, 1), scene.Status.Ok);
	testing.expect_value(t, r.packet.frame.input, p.frame.input);
	testing.expect_value(t, r.packet.frame.cycle, u32(1));
	testing.expect_value(t, r.packet.geometries[1].children[1], children[1]);
	testing.expect_value(t, r.packet.frame.overlays[0], p.frame.overlays[0]);
	testing.expect_value(t, r.packet.frame.overlays[1], p.frame.overlays[1]);
	testing.expect_value(t, r.packet.frame.metrics[0], p.frame.metrics[0]);
	for pose, index in p.frame.poses
	{
		for value, axis in pose.position
		{
			testing.expect_value(t, transmute(u32)r.packet.frame.poses[index].position[axis], transmute(u32)value);
		}
	}
	testing.expect_value(t, replay.reader_seek(&r, 0), scene.Status.Ok);
	testing.expect_value(t, len(r.packet.frame.ids), 1);
	testing.expect_value(t, r.packet.frame.ids[0], u32(0));
	testing.expect_value(t, r.packet.frame.states[0], scene.State{.Sleeping});
	testing.expect_value(t, replay.reader_seek(&r, 1), scene.Status.Ok);
	testing.expect_value(t, r.packet.frame.ids[0], u32(1));
}

visual_replay_roundtrip :: proc(t: ^testing.T, compression: replay.Compression)
{
	root: string;
	error: os.Error;
	root, error = os.temp_directory(context.allocator);
	testing.expect(t, error == nil);
	defer delete(root);
	path: string = fmt.aprintf("%s/entasis-visual-%d.replay", root, time.tick_now());
	defer delete(path);
	partial: string = fmt.aprintf("%s.partial", path);
	defer delete(partial);
	defer os.remove(path);
	defer os.remove(partial);
	p: scene.Scene_Packet;
	defer scene.packet_destroy(&p);
	append(&p.geometries, scene.Geometry{kind=.Box, size={1, 2, 3}, minimum={-0.5, -1, -1.5}, maximum={0.5, 1, 1.5}});
	append(&p.geometries, scene.Geometry{kind=.Marker});
	append(&p.geometries, scene.Geometry{kind=.Sphere, size={0.75, 0, 0}, minimum={-0.75, -0.75, -0.75}, maximum={0.75, 0.75, 0.75}});
	append(&p.geometries, scene.Geometry{kind=.Capsule, size={0.5, 3, 0}, minimum={-0.5, -2, -0.5}, maximum={0.5, 2, 0.5}});
	append(&p.geometries, scene.Geometry{kind=.Cylinder, size={0.6, 2, 0}, minimum={-0.6, -1, -0.6}, maximum={0.6, 1, 0.6}});
	vertices: []scene.Vec3 = make([]scene.Vec3, 4);
	copy(vertices, []scene.Vec3{{0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {0, 0, 4}});
	indices: []u32 = make([]u32, 12);
	copy(indices, []u32{0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3});
	append(&p.geometries, scene.Geometry{kind=.Triangles, minimum={0, 0, 0}, maximum={2, 3, 4}, vertices=vertices, indices=indices});
	for level in 0 ..< 2
	{
		children: []scene.Geometry_Child = make([]scene.Geometry_Child, 2);
		children[0] = {u32(5+level), 7, {position={2, 3, 4}, orientation=quaternion128(1)}};
		children[1] = {2, 11, {position={-2, -3, -4}, orientation=quaternion(x=f32(1), y=f32(0), z=f32(0), w=f32(0))}};
		append(&p.geometries, scene.Geometry{kind=.Compound, minimum={-3, -5, -6}, maximum={8, 12, 16}, children=children});
	}
	scene.frame_reserve(&p.frame, len(p.geometries), 1, 1);
	for &geometry, index in p.geometries
	{
		testing.expect_value(t, scene.geometry_admit(&geometry, index), scene.Status.Ok);
		append(&p.entities, scene.Entity{kind=.Shape, source_handle=i32(index), geometry=u32(index), group=u32(index%3)});
		append(&p.frame.ids, u32(index));
		append(&p.frame.poses, scene.IDENTITY);
		append(&p.frame.states, scene.State{});
	}
	append(&p.frame.overlays, scene.Overlay{kind=.Sphere_Volume, entity_a=scene.NO_ENTITY, entity_b=scene.NO_ENTITY, a={-2, 3, 4}, value=0.75});
	w: replay.Writer;
	defer replay.writer_close(&w);
	status: scene.Status = replay.writer_open(&w, path, {scenario="example/test", source="test", axis=.Physics, timestep=1.0/60, settings_state=.Available}, compression);
	testing.expect_value(t, status, scene.Status.Ok);
	if status != .Ok
	{
		return;
	}
	for tick in 0 ..< 3
	{
		p.frame.tick = u64(tick);
		p.frame.poses[0].position.x = f32(tick);
		status = replay.writer_frame(&w, &p);
		testing.expect_value(t, status, scene.Status.Ok);
		if status != .Ok
		{
			return;
		}
	}
	testing.expect_value(t, replay.writer_finish(&w, .Completed), scene.Status.Ok);
	r: replay.Reader;
	defer replay.reader_close(&r);
	status = replay.reader_open(&r, path);
	testing.expect_value(t, status, scene.Status.Ok);
	if status != .Ok
	{
		return;
	}
	testing.expect_value(t, r.version, u32(3));
	testing.expect_value(t, r.compression, compression);
	order: [5]int = {2, 0, 1, 2, 0};
	for tick in order
	{
		testing.expect_value(t, replay.reader_seek(&r, tick), scene.Status.Ok);
		testing.expect_value(t, r.packet.frame.tick, u64(tick));
		testing.expect_value(t, r.packet.frame.poses[0].position.x, f32(tick));
		testing.expect_value(t, r.packet.frame.overlays[0], p.frame.overlays[0]);
		testing.expect_value(t, len(r.packet.geometries), len(p.geometries));
		for &geometry, index in p.geometries
		{
			actual: ^scene.Geometry = &r.packet.geometries[index];
			testing.expect_value(t, actual.kind, geometry.kind);
			testing.expect_value(t, actual.size, geometry.size);
			testing.expect_value(t, actual.minimum, geometry.minimum);
			testing.expect_value(t, actual.maximum, geometry.maximum);
			testing.expect_value(t, len(actual.vertices), len(geometry.vertices));
			for vertex, vertex_index in geometry.vertices
			{
				testing.expect_value(t, actual.vertices[vertex_index], vertex);
			}
			testing.expect_value(t, len(actual.indices), len(geometry.indices));
			for triangle_index, element in geometry.indices
			{
				testing.expect_value(t, actual.indices[element], triangle_index);
			}
			testing.expect_value(t, len(actual.children), len(geometry.children));
			for child, child_index in geometry.children
			{
				testing.expect_value(t, actual.children[child_index], child);
			}
			testing.expect_value(t, r.packet.entities[index], p.entities[index]);
		}
	}
}

@(test)
visual_capture_failure_preserves_files :: proc(t: ^testing.T)
{
	root: string;
	error: os.Error;
	root, error = os.temp_directory(context.allocator);
	testing.expect(t, error == nil);
	defer delete(root);
	path: string = fmt.aprintf("%s/entasis-capture-%d.replay", root, time.tick_now());
	partial: string = fmt.aprintf("%s.partial", path);
	defer delete(path);
	defer delete(partial);
	defer os.remove(path);
	defer os.remove(partial);
	r: scenarios.Recipe;
	status: scene.Status;
	r, status = scenarios.recipe_admit(.Falling_Box, nil);
	testing.expect_value(t, status, scene.Status.Ok);
	s: scenarios.Session;
	defer scenarios.session_destroy(&s);
	status = scenarios.session_create(&s, r);
	testing.expect_value(t, status, scene.Status.Ok);
	if status != .Ok
	{
		return;
	}
	for _ in 0 ..< 7
	{
		testing.expect_value(t, scenarios.session_step(&s), scene.Status.Ok);
	}
	w: replay.Writer;
	defer replay.writer_close(&w);
	m: scene.Metadata = {scenario="example/falling_box", axis=.Physics, timestep=s.timestep};
	testing.expect_value(t, replay.writer_open(&w, path, m), scene.Status.Ok);
	testing.expect_value(t, replay.writer_frame(&w, &s.observation.packet), scene.Status.Ok);
	testing.expect_value(t, replay.writer_finish(&w, .Prefix), scene.Status.Ok);
	testing.expect_value(t, s.tick, 7);
	testing.expect_value(t, len(w.index), 1);
	reader: replay.Reader;
	defer replay.reader_close(&reader);
	testing.expect_value(t, replay.reader_open(&reader, path), scene.Status.Ok);
	testing.expect_value(t, replay.reader_seek(&reader, 0), scene.Status.Ok);
	testing.expect_value(t, reader.packet.frame.tick, u64(7));
	testing.expect_value(t, reader.metadata.completion, scene.Completion.Prefix);
	other: replay.Writer;
	defer replay.writer_close(&other);
	testing.expect_value(t, replay.writer_open(&other, path, m), scene.Status.File_Error);
	replay.reader_close(&reader);
	replay.writer_close(&w);
	os.remove(path);
	testing.expect_value(t, replay.writer_open(&w, path, m), scene.Status.Ok);
	testing.expect_value(t, replay.writer_frame(&w, &s.observation.packet), scene.Status.Ok);
	w.budget = 1;
	testing.expect_value(t, scenarios.session_step(&s), scene.Status.Ok);
	testing.expect_value(t, replay.writer_frame(&w, &s.observation.packet), scene.Status.Budget_Exceeded);
	testing.expect(t, w.required_bytes > w.budget);
	testing.expect_value(t, replay.writer_finish(&w, .Completed), scene.Status.Invalid_Data);
	replay.writer_close(&w);
	testing.expect(t, !os.exists(path));
	testing.expect(t, os.exists(partial));
	testing.expect_value(t, replay.writer_open(&other, path, m), scene.Status.File_Error);
	os.remove(partial);
	testing.expect_value(t, replay.writer_open(&w, path, m), scene.Status.Ok);
	testing.expect_value(t, replay.writer_frame(&w, &s.observation.packet), scene.Status.Ok);
	replay.writer_close(&w);
	testing.expect(t, !os.exists(path));
	testing.expect_value(t, replay.reader_open(&reader, partial), scene.Status.Invalid_Data);
}

@(test)
visual_replay_rejects_invalid_or_incomplete_input :: proc(t: ^testing.T)
{
	visual_replay_failed_growth(t);
	root: string;
	error: os.Error;
	root, error = os.temp_directory(context.allocator);
	testing.expect(t, error == nil);
	defer delete(root);
	path: string = fmt.aprintf("%s/entasis-invalid-%d.replay", root, time.tick_now());
	partial: string = fmt.aprintf("%s.partial", path);
	defer delete(path);
	defer delete(partial);
	defer os.remove(path);
	defer os.remove(partial);
	p: scene.Scene_Packet;
	defer scene.packet_destroy(&p);
	w: replay.Writer;
	defer replay.writer_close(&w);
	testing.expect_value(t, replay.writer_open(&w, path, {scenario="example/empty", axis=.Operation}), scene.Status.Ok);
	testing.expect_value(t, replay.writer_frame(&w, &p), scene.Status.Ok);
	p.frame.tick = 1;
	testing.expect_value(t, replay.writer_frame(&w, &p), scene.Status.Ok);
	testing.expect_value(t, replay.writer_finish(&w, .Completed), scene.Status.Ok);
	replay.writer_close(&w);
	bytes: []u8;
	bytes, error = os.read_entire_file(path, context.allocator);
	testing.expect(t, error == nil);
	defer delete(bytes);
	r: replay.Reader;
	defer replay.reader_close(&r);
	testing.expect_value(t, replay.reader_open(&r, path), scene.Status.Ok);
	testing.expect_value(t, replay.reader_seek(&r, 0), scene.Status.Ok);
	io_address: rawptr = raw_data(r.io);
	decode_address: rawptr = raw_data(r.decoded);
	testing.expect_value(t, replay.reader_seek(&r, 1), scene.Status.Ok);
	testing.expect_value(t, replay.reader_seek(&r, 0), scene.Status.Ok);
	testing.expect_value(t, rawptr(raw_data(r.io)), io_address);
	testing.expect_value(t, rawptr(raw_data(r.decoded)), decode_address);
	testing.expect_value(t, r.owned_bytes, replay.reader_bytes(&r));
	bad_frame: int = int(r.index[1].frame_offset);
	length: int = int(r.index[1].frame_length);
	file: ^os.File;
	file, error = os.open(path, {.Write});
	testing.expect(t, error == nil);
	corrupt: []u8 = make([]u8, length);
	defer delete(corrupt);
	_, error = os.write_at(file, corrupt, i64(bad_frame));
	testing.expect(t, error == nil);
	os.close(file);
	testing.expect_value(t, replay.reader_prepare(&r, 1), scene.Status.Invalid_Data);
	testing.expect_value(t, r.selected, 0);
	testing.expect_value(t, r.packet.frame.tick, u64(0));
	replay.reader_close(&r);
	testing.expect_value(t, r.owned_bytes, u64(0));
	testing.expect(t, os.write_entire_file(path, bytes)==nil);
	testing.expect_value(t, replay.reader_open(&r, path, 1), scene.Status.Budget_Exceeded);
	testing.expect(t, r.required_bytes > r.budget);
	header: replay.Codec = {bytes=bytes, at=16};
	directory: int = int(replay.decode_u64(&header));
	Mutation :: struct
	{
		offset: int,
		value: u8,
	}
	mutations: [7]Mutation = {
		{32, 255}, {36, 255}, {len(bytes)-1, 255},
		{directory+replay.V3_INDEX_BYTES+40, 0},
		{directory+replay.V3_INDEX_BYTES+56, 0},
		{directory+replay.V3_INDEX_BYTES+63, 255},
		{directory+52, 255},
	};
	for mutation in mutations
	{
		offset: int = mutation.offset;
		original: u8 = bytes[offset];
		bytes[offset] = mutation.value;
		testing.expect(t, os.write_entire_file(path, bytes)==nil);
		testing.expect_value(t, replay.reader_open(&r, path), scene.Status.Invalid_Data);
		bytes[offset] = original;
	}
	testing.expect(t, os.write_entire_file(path, bytes[:len(bytes)-1])==nil);
	testing.expect_value(t, replay.reader_open(&r, path), scene.Status.Invalid_Data);
}

visual_replay_failed_growth :: proc(t: ^testing.T)
{
	root: string;
	error: os.Error;
	root, error = os.temp_directory(context.allocator);
	testing.expect(t, error == nil);
	defer delete(root);
	path: string = fmt.aprintf("%s/entasis-growth-%d.replay", root, time.tick_now());
	defer delete(path);
	defer os.remove(path);
	bytes: [4096]u8;
	copy(bytes[:8], "ENTRPLY\x00");
	c: replay.Codec = {bytes=bytes[:], at=32};
	replay.encode_metadata(&c, {scenario="example/growth", axis=.Operation});
	metadata_end: int = c.at;
	entries: [3]replay.Index_Entry;
	for &entry, index in entries
	{
		entry = {definitions_offset=u64(c.at), geometries=1, entities=8, tick=u64(index), phase=.Active};
		replay.encode_u32(&c, 1 if index == 0 else 0);
		replay.encode_u32(&c, 8 if index == 0 else 0);
		if index == 0
		{
			geometry: scene.Geometry = {kind=.Box, size={1, 1, 1}, minimum={-0.5, -0.5, -0.5}, maximum={0.5, 0.5, 0.5}};
			replay.encode_geometry(&c, &geometry);
			for id in 0 ..< 8
			{
				replay.encode_u32(&c, u32(scene.Object_Kind.Body));
				replay.encode_u32(&c, u32(id));
				replay.encode_u32(&c, 0);
				replay.encode_u32(&c, 0);
			}
		}
		entry.definitions_length = u64(c.at)-entry.definitions_offset;
		entry.frame_offset = u64(c.at);
		replay.encode_u64(&c, u64(index));
		replay.encode_u32(&c, 0);
		replay.encode_u32(&c, u32(scene.Phase.Active));
		for _ in 0 ..< 3
		{
			replay.encode_u32(&c, 0);
		}
		count: int = 1 if index == 0 else 8;
		replay.encode_u32(&c, u32(count));
		replay.encode_u32(&c, 0);
		replay.encode_u32(&c, 0);
		for id in 0 ..< count
		{
			replay.encode_u32(&c, 99 if index == 1 && id == 7 else u32(id));
			replay.encode_pose(&c, {position={f32(id), 2, 3}, orientation=quaternion128(1)});
			replay.encode_u32(&c, 0);
		}
		entry.frame_length = u64(c.at)-entry.frame_offset;
	}
	directory: int = c.at;
	for entry in entries
	{
		replay.encode_u64(&c, entry.definitions_offset);
		replay.encode_u64(&c, entry.definitions_length);
		replay.encode_u64(&c, entry.frame_offset);
		replay.encode_u64(&c, entry.frame_length);
		replay.encode_u32(&c, entry.geometries);
		replay.encode_u32(&c, entry.entities);
		replay.encode_u64(&c, entry.tick);
		replay.encode_u32(&c, u32(entry.phase));
		replay.encode_u32(&c, entry.cycle);
	}
	replay.encode_u64(&c, replay.COMPLETE);
	length: int = c.at;
	c.at = 8;
	replay.encode_u32(&c, 2);
	replay.encode_u32(&c, u32(metadata_end-32));
	replay.encode_u64(&c, u64(directory));
	replay.encode_u32(&c, 3);
	replay.encode_u32(&c, u32(scene.Completion.Completed));
	testing.expect_value(t, c.status, scene.Status.Ok);
	testing.expect(t, os.write_entire_file(path, bytes[:length]) == nil);
	r: replay.Reader;
	defer replay.reader_close(&r);
	testing.expect_value(t, replay.reader_open(&r, path, 1024*1024), scene.Status.Ok);
	testing.expect_value(t, replay.reader_seek(&r, 0), scene.Status.Ok);
	selected_pose: scene.Pose = r.packet.frame.poses[0];
	before: u64 = r.owned_bytes;
	// the first column grows, then the existing nil allocator rejects the pose column
	r.spare.poses.allocator = mem.nil_allocator();
	testing.expect_value(t, replay.reader_prepare(&r, 1), scene.Status.Out_Of_Memory);
	r.spare.poses.allocator = context.allocator;
	testing.expect(t, cap(r.spare.ids) >= 8 && cap(r.spare.poses) == 0);
	testing.expect(t, r.owned_bytes > before);
	testing.expect_value(t, r.owned_bytes, replay.reader_bytes(&r));
	testing.expect_value(t, r.selected, 0);
	testing.expect_value(t, r.packet.frame.poses[0], selected_pose);
	r.budget = before;
	testing.expect_value(t, replay.reader_prepare(&r, 2), scene.Status.Budget_Exceeded);
	r.budget = 1024*1024;
	testing.expect_value(t, replay.reader_prepare(&r, 1), scene.Status.Invalid_Data);
	testing.expect_value(t, r.owned_bytes, replay.reader_bytes(&r));
	testing.expect_value(t, r.selected, 0);
	testing.expect_value(t, r.packet.frame.poses[0], selected_pose);
	r.budget = r.owned_bytes;
	testing.expect_value(t, replay.reader_seek(&r, 2), scene.Status.Ok);
	testing.expect_value(t, len(r.packet.frame.ids), 8);
	testing.expect_value(t, r.packet.frame.tick, u64(2));
	replay.reader_close(&r);
	testing.expect_value(t, r.owned_bytes, u64(0));
}

visual_replay_v2_fixture :: proc(t: ^testing.T)
{
	root: string;
	error: os.Error;
	root, error = os.temp_directory(context.allocator);
	testing.expect(t, error==nil);
	defer delete(root);
	path: string = fmt.aprintf("%s/entasis-v2-%d.replay", root, time.tick_now());
	defer delete(path);
	defer os.remove(path);
	// hand-authored little-endian V2 wire fixture, independent of the current encoder
	// 32-byte header, 72-byte metadata, definitions at 104, frames at 180 and 264, index at 340
	bytes :: "ENTRPLY\x00\x02\x00\x00\x00\x48\x00\x00\x00" +
		"\x54\x01\x00\x00\x00\x00\x00\x00\x02\x00\x00\x00\x01\x00\x00\x00" +
		"\x0e\x00\x00\x00example/legacy\x0a\x00\x00\x00v2-fixture" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		// one box with size (2, 4, 6), bounds (-1, -2, -3)..(1, 2, 3), one body handle 42 in group 9
		"\x01\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00" +
		"\x00\x00\x00\x40\x00\x00\x80\x40\x00\x00\xc0\x40" +
		"\x00\x00\x80\xbf\x00\x00\x00\xc0\x00\x00\x40\xc0" +
		"\x00\x00\x80\x3f\x00\x00\x00\x40\x00\x00\x40\x40" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x00\x00\x2a\x00\x00\x00\x00\x00\x00\x00\x09\x00\x00\x00" +
		// tick 7, active, scripted, body 0 at (1, 2, 3), identity orientation, sleeping
		"\x07\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x80\x3f\x00\x00\x00\x40\x00\x00\x40\x40" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f\x02\x00\x00\x00" +
		// no new definitions, tick 11 completed, live input (1, -1), pose (-2, 4, 1), half-turn about Z
		"\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x0b\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x04\x00\x00\x00" +
		"\x01\x00\x00\x00\x00\x00\x80\x3f\x00\x00\x80\xbf" +
		"\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x00\xc0\x00\x00\x80\x40\x00\x00\x80\x3f" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f\x00\x00\x00\x00\x00\x00\x00\x00" +
		// two 56-byte index entries and the historical completion marker
		"\x68\x00\x00\x00\x00\x00\x00\x00\x4c\x00\x00\x00\x00\x00\x00\x00" +
		"\xb4\x00\x00\x00\x00\x00\x00\x00\x4c\x00\x00\x00\x00\x00\x00\x00" +
		"\x01\x00\x00\x00\x01\x00\x00\x00\x07\x00\x00\x00\x00\x00\x00\x00" +
		"\x03\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x01\x00\x00\x00\x00\x00\x00\x08\x00\x00\x00\x00\x00\x00\x00" +
		"\x08\x01\x00\x00\x00\x00\x00\x00\x4c\x00\x00\x00\x00\x00\x00\x00" +
		"\x01\x00\x00\x00\x01\x00\x00\x00\x0b\x00\x00\x00\x00\x00\x00\x00" +
		"\x04\x00\x00\x00\x00\x00\x00\x00RPLDONE2";
	testing.expect(t, os.write_entire_file(path, bytes)==nil);
	r: replay.Reader;
	defer replay.reader_close(&r);
	if !testing.expect_value(t, replay.reader_open(&r, path), scene.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, r.version, u32(2));
	testing.expect_value(t, r.metadata.scenario, "example/legacy");
	testing.expect_value(t, r.metadata.source, "v2-fixture");
	testing.expect_value(t, r.metadata.axis, scene.Time_Axis.Operation);
	testing.expect_value(t, r.metadata.timestep, f32(0));
	testing.expect_value(t, r.metadata.settings_state, scene.Availability.Unavailable);
	testing.expect_value(t, r.metadata.events_state, scene.Availability.Unavailable);
	testing.expect_value(t, r.metadata.completion, scene.Completion.Completed);
	testing.expect_value(t, len(r.index), 2);
	if !testing.expect_value(t, len(r.packet.geometries), 1) || !testing.expect_value(t, len(r.packet.entities), 1)
	{
		return;
	}
	testing.expect_value(t, r.packet.geometries[0].kind, scene.Geometry_Kind.Box);
	testing.expect_value(t, r.packet.geometries[0].size, scene.Vec3{2, 4, 6});
	testing.expect_value(t, r.packet.geometries[0].minimum, scene.Vec3{-1, -2, -3});
	testing.expect_value(t, r.packet.geometries[0].maximum, scene.Vec3{1, 2, 3});
	testing.expect_value(t, r.packet.entities[0], scene.Entity{kind=.Body, source_handle=42, geometry=0, group=9});
	for index in ([3]int{0, 1, 0})
	{
		if !testing.expect_value(t, replay.reader_seek(&r, index), scene.Status.Ok) ||
			!testing.expect_value(t, len(r.packet.frame.ids), 1)
		{
			return;
		}
		testing.expect_value(t, r.selected, index);
		testing.expect_value(t, r.packet.frame.ids[0], u32(0));
		testing.expect_value(t, r.packet.frame.cycle, u32(0));
		testing.expect_value(t, r.packet.frame.tick, u64(7 if index == 0 else 11));
		testing.expect_value(t, r.packet.frame.phase, scene.Phase.Active if index == 0 else scene.Phase.Completed);
		expected_pose: scene.Pose = {position={1, 2, 3}, orientation=quaternion128(1)};
		expected_input: scene.Input;
		expected_state: scene.State = {.Sleeping};
		if index == 1
		{
			expected_pose = {position={-2, 4, 1}, orientation=quaternion(x=0, y=0, z=1, w=0)};
			expected_input = {mode=.Live, x=1, y=-1};
			expected_state = {};
		}
		testing.expect_value(t, r.packet.frame.poses[0], expected_pose);
		testing.expect_value(t, r.packet.frame.input, expected_input);
		testing.expect_value(t, r.packet.frame.states[0], expected_state);
	}
}

@(test)
visual_replay_v1_retains_known_and_unknown_data :: proc(t: ^testing.T)
{
	// hand-authored V1 compatibility bytes, not captured output or current-encoder output
	// each header has two bodies, two frames, two workers, 60 Hz and the source legacy-fixture
	fixtures: [3]string = {
		// Container: five static boxes, followed by two 72-byte frames
		"ENTRPLY\x00\x01\x00\x00\x00\x00\x00\x00\x00" +
		"\x02\x00\x00\x00\x05\x00\x00\x00\x02\x00\x00\x00\x02\x00\x00\x00" +
		"\x3c\x00\x00\x00\x00\x00\x00\x00legacy-fixture" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x80\x3f\x00\x00\x80\x3f\x00\x00\x80\x3f\xcd\xcc\x4c\xbe" +
		"\x00\x00\x00\x00\x00\x00\x00\xbf\x00\x00\x00\x00\x00\x00\xa0\x41\x00\x00\x80\x3f\x00\x00\xa0\x41" +
		"\x00\x00\x00\x00\x00\x00\x00\xbf\x00\x00\x00\x00\x00\x00\xa0\x41\x00\x00\x80\x3f\x00\x00\xa0\x41" +
		"\x00\x00\x00\x00\x00\x00\x00\xbf\x00\x00\x00\x00\x00\x00\xa0\x41\x00\x00\x80\x3f\x00\x00\xa0\x41" +
		"\x00\x00\x00\x00\x00\x00\x00\xbf\x00\x00\x00\x00\x00\x00\xa0\x41\x00\x00\x80\x3f\x00\x00\xa0\x41" +
		"\x00\x00\x00\x00\x00\x00\x00\xbf\x00\x00\x00\x00\x00\x00\xa0\x41\x00\x00\x80\x3f\x00\x00\xa0\x41" +
		"\x00\x00\x00\x00\x03\x00\x00\x00\x04\x00\x00\x00\x01\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x80\xbf\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x00\x00\x00\x40\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x01\x00\x00\x00\x03\x00\x00\x00\x04\x00\x00\x00\x01\x00\x00\x00" +
		"\x00\x00\x80\x3f\x00\x00\x80\xbf\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x00\x00\x00\x40\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f",
		// Contact Islands: one static box and two bodies in the same island
		"ENTRPLY\x00\x01\x00\x00\x00\x01\x00\x00\x00" +
		"\x02\x00\x00\x00\x01\x00\x00\x00\x02\x00\x00\x00\x02\x00\x00\x00" +
		"\x3c\x00\x00\x00\x00\x00\x00\x00legacy-fixture" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x80\x3f\x00\x00\x80\x3f\x00\x00\x80\x3f\xcd\xcc\x4c\xbe" +
		"\x00\x00\x00\x00\x00\x00\x00\xbf\x00\x00\x00\x00\x00\x00\xa0\x41\x00\x00\x80\x3f\x00\x00\xa0\x41" +
		"\x00\x00\x00\x00\x03\x00\x00\x00\x04\x00\x00\x00\x01\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x80\xbf\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x00\x00\x00\x40\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x01\x00\x00\x00\x03\x00\x00\x00\x04\x00\x00\x00\x01\x00\x00\x00" +
		"\x00\x00\x80\x3f\x00\x00\x80\xbf\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x00\x00\x00\x40\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f",
		// Pyramid: one box, a radius-0.5 projectile launched at tick 0 and one static box
		"ENTRPLY\x00\x01\x00\x00\x00\x02\x00\x00\x00" +
		"\x02\x00\x00\x00\x01\x00\x00\x00\x02\x00\x00\x00\x02\x00\x00\x00" +
		"\x3c\x00\x00\x00\x00\x00\x00\x00legacy-fixture" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
		"\x00\x00\x80\x3f\x00\x00\x80\x3f\x00\x00\x80\x3f\xcd\xcc\x4c\xbe" +
		"\x01\x00\x00\x00\x00\x00\x00\x3f\x00\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x00\xbf\x00\x00\x00\x00\x00\x00\xa0\x41\x00\x00\x80\x3f\x00\x00\xa0\x41" +
		"\x00\x00\x00\x00\x03\x00\x00\x00\x04\x00\x00\x00\x01\x00\x00\x00" +
		"\x00\x00\x00\x00\x00\x00\x80\xbf\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x00\x00\x00\x40\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x01\x00\x00\x00\x03\x00\x00\x00\x04\x00\x00\x00\x01\x00\x00\x00" +
		"\x00\x00\x80\x3f\x00\x00\x80\xbf\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f" +
		"\x00\x00\x00\x40\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x3f",
	};
	scenarios: [3]string = {"benchmark/container", "benchmark/contact_islands", "benchmark/pyramid"};
	for bytes, fixture in fixtures
	{
		root: string;
		error: os.Error;
		root, error = os.temp_directory(context.allocator);
		testing.expect(t, error==nil);
		defer delete(root);
		path: string = fmt.aprintf("%s/entasis-v1-%d.replay", root, time.tick_now());
		defer delete(path);
		defer os.remove(path);
		statics: int = 5 if fixture == 0 else 1;
		testing.expect(t, os.write_entire_file(path, bytes)==nil);
		r: replay.Reader;
		defer replay.reader_close(&r);
		status: scene.Status = replay.reader_open(&r, path);
		testing.expect_value(t, status, scene.Status.Ok);
		if status!=.Ok
		{
			return;
		}
		testing.expect_value(t, r.metadata.source, "legacy-fixture");
		testing.expect_value(t, r.version, u32(1));
		testing.expect_value(t, r.metadata.scenario, scenarios[fixture]);
		testing.expect_value(t, r.metadata.axis, scene.Time_Axis.Physics);
		testing.expect_value(t, r.metadata.timestep, f32(1.0/60.0));
		testing.expect_value(t, r.metadata.settings_state, scene.Availability.Unavailable);
		testing.expect_value(t, r.metadata.events_state, scene.Availability.Unavailable);
		if !testing.expect_value(t, len(r.packet.entities), statics+2) ||
			!testing.expect_value(t, len(r.packet.geometries), statics+1+(1 if fixture == 2 else 0))
		{
			return;
		}
		testing.expect_value(t, r.packet.geometries[0].kind, scene.Geometry_Kind.Box);
		testing.expect_value(t, r.packet.geometries[0].size, scene.Vec3{1, 1, 1});
		testing.expect_value(t, r.packet.entities[0], scene.Entity{kind=.Body, source_handle=0});
		testing.expect_value(t, r.packet.entities[1], scene.Entity{kind=.Body, source_handle=1, geometry=1 if fixture == 2 else 0});
		if fixture == 2
		{
			testing.expect_value(t, r.packet.geometries[1].kind, scene.Geometry_Kind.Sphere);
			testing.expect_value(t, r.packet.geometries[1].size, scene.Vec3{0.5, 0, 0});
		}
		if !testing.expect_value(t, replay.reader_seek(&r, 1), scene.Status.Ok) ||
			!testing.expect_value(t, len(r.packet.frame.ids), statics+2)
		{
			return;
		}
		testing.expect_value(t, r.packet.frame.tick, u64(1));
		testing.expect_value(t, r.packet.frame.phase, scene.Phase.Active);
		testing.expect_value(t, r.packet.frame.poses[0].position, scene.Vec3{1, -1, 0});
		testing.expect_value(t, r.packet.frame.poses[1], scene.Pose{position={2, 3, 0}, orientation=quaternion128(1)});
		testing.expect_value(t, r.packet.frame.states[0], scene.State{.Below_Floor});
		for i in 0 ..< statics
		{
			testing.expect_value(t, r.packet.entities[i+2].kind, scene.Object_Kind.Static);
			testing.expect_value(t, r.packet.geometries[r.packet.entities[i+2].geometry].size, scene.Vec3{20, 1, 20});
			testing.expect_value(t, r.packet.frame.poses[i+2].position, scene.Vec3{0, -0.5, 0});
			testing.expect_value(t, r.packet.frame.states[i+2], scene.State{.Static});
		}
		if !testing.expect_value(t, len(r.packet.frame.metrics), 3)
		{
			return;
		}
		testing.expect_value(t, r.packet.frame.metrics[0], scene.Metric{.Constraints, 3});
		testing.expect_value(t, r.packet.frame.metrics[1], scene.Metric{.Pairs, 4});
		testing.expect_value(t, r.packet.frame.metrics[2], scene.Metric{.Below_Floor_Centers, 1});
		testing.expect_value(t, replay.reader_seek(&r, 0), scene.Status.Ok);
		testing.expect_value(t, r.packet.frame.tick, u64(0));
		testing.expect_value(t, r.packet.frame.phase, scene.Phase.Initial);
		testing.expect_value(t, r.packet.frame.poses[0].position, scene.Vec3{0, -1, 0});
	}
}
