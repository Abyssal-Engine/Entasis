package physics_replay

import "core:fmt"
import "core:os"
import "core:strings"
import "core:time"
import scene "../physics_scene"

Index_Entry :: struct
{
	definitions_offset, definitions_length, frame_offset, frame_length: u64,
	geometries, entities: u32,
	tick: u64,
	phase: scene.Phase,
	cycle: u32,
	decoded_length: u64,
}
INDEX_BYTES :: 56;
V2_HEADER_BYTES :: 32;
V3_INDEX_BYTES :: 64;
V3_HEADER_BYTES :: 40;
COMPLETE :: u64(0x32454e4f444c5052);
DEFAULT_BUDGET :: scene.DEFAULT_BUDGET;
Writer :: struct
{
	file: ^os.File,
	path, partial: string,
	index: [dynamic]Index_Entry,
	scratch: [dynamic]u8,
	offset: u64,
	geometries, entities: int,
	status: scene.Status,
	compression: Compression,
	compressed: []u8,
	encoder: []u64,
	budget, peak_bytes, raw_bytes, compressed_bytes, required_bytes: u64,
	encode_time: time.Duration,
}
Reader :: struct
{
	file: ^os.File,
	metadata: scene.Metadata,
	packet: scene.Scene_Packet,
	spare: scene.Frame,
	index: []Index_Entry,
	selected: int,
	budget, owned_bytes, required_bytes: u64,
	version: u32,
	v1: Header,
	v1_pyramid: Pyramid_Description,
	v1_stride, v1_offset: u64,
	io, decoded: []u8,
	metadata_bytes: u64,
	prepared: int,
	compression: Compression,
	decode_time: time.Duration,
}

encode_index :: proc(c: ^Codec, e: Index_Entry)
{
	encode_u64(c, e.definitions_offset);
	encode_u64(c, e.definitions_length);
	encode_u64(c, e.frame_offset);
	encode_u64(c, e.frame_length);
	encode_u32(c, e.geometries);
	encode_u32(c, e.entities);
	encode_u64(c, e.tick);
	encode_u32(c, u32(e.phase));
	encode_u32(c, e.cycle);
	encode_u64(c, e.decoded_length);
}

decode_index :: proc(c: ^Codec, version: u32) -> Index_Entry
{
	e: Index_Entry;
	e.definitions_offset=decode_u64(c);
	e.definitions_length=decode_u64(c);
	e.frame_offset=decode_u64(c);
	e.frame_length=decode_u64(c);
	e.geometries=decode_u32(c);
	e.entities=decode_u32(c);
	e.tick=decode_u64(c);
	e.phase=scene.Phase(decode_u32(c));
	e.cycle=decode_u32(c);
	e.decoded_length = decode_u64(c) if version == 3 else e.frame_length;
	return e;
}

writer_write :: proc(w: ^Writer, bytes: []u8) -> scene.Status
{
	if w.status != .Ok
	{
		return w.status;
	}
	written: int;
	error: os.Error;
	written, error = os.write(w.file, bytes);
	if error != nil || written != len(bytes)
	{
		w.status = .File_Error;
	}
	w.offset += u64(written);
	return w.status;
}

writer_close :: proc(w: ^Writer)
{
	if w.file != nil
	{
		os.close(w.file);
	}
	delete(w.path);
	delete(w.partial);
	delete(w.index);
	delete(w.scratch);
	delete(w.compressed);
	delete(w.encoder);
	w^ = {};
}

writer_bytes :: proc(w: ^Writer) -> u64
{
	return u64(len(w.path)+len(w.partial)+cap(w.index)*size_of(Index_Entry)+cap(w.scratch)+len(w.compressed)+len(w.encoder)*8);
}

writer_open :: proc(w: ^Writer, path: string, metadata: scene.Metadata, compression: Compression = .LZ4, budget: u64 = DEFAULT_BUDGET) -> scene.Status
{
	if os.exists(path)
	{
		return .File_Error;
	}
	if u32(compression) > u32(Compression.LZ4HC)
	{
		return .Invalid_Data;
	}
	state_words: int = (compression_state_size(compression)+7)/8;
	metadata_length: int = 48+len(metadata.scenario)+len(metadata.source)+len(metadata.revision)+len(metadata.compiler)+
		len(metadata.host)+len(metadata.configuration)+len(metadata.components)+len(metadata.settings);
	if metadata_length>65536
	{
		return .Invalid_Data;
	}
	w.required_bytes = u64(state_words*8+len(path)*2+8+metadata_length+V3_HEADER_BYTES);
	if w.required_bytes > budget
	{
		return .Budget_Exceeded;
	}
	w.budget, w.compression = budget, compression;
	w.path = strings.clone(path);
	w.partial = fmt.aprintf("%s.partial", path);
	error: os.Error;
	w.file, error = os.open(w.partial, {.Write, .Create, .Excl});
	if error != nil
	{
		writer_close(w);
		return .File_Error;
	}
	if resize(&w.scratch, metadata_length+V3_HEADER_BYTES)!=nil
	{
		writer_close(w);
		return .Out_Of_Memory;
	}
	c: Codec = {bytes=w.scratch[V3_HEADER_BYTES:]};
	encode_metadata(&c, metadata);
	if c.status != .Ok
	{
		writer_close(w);
		return c.status;
	}
	copy(w.scratch[:8], "ENTRPLY\x00");
	h: Codec = {bytes=w.scratch[8:V3_HEADER_BYTES]};
	encode_u32(&h, 3);
	encode_u32(&h, u32(c.at));
	encode_u64(&h, 0);
	encode_u32(&h, 0);
	encode_u32(&h, 0);
	encode_u32(&h, BLOCK_CODEC);
	encode_u32(&h, u32(compression));
	w.encoder = make([]u64, state_words);
	w.peak_bytes = writer_bytes(w);
	return writer_write(w, w.scratch[:V3_HEADER_BYTES+c.at]);
}

writer_frame :: proc(w: ^Writer, p: ^scene.Scene_Packet) -> (result: scene.Status)
{
	defer if result != .Ok
	{
		w.status = result;
	}
	if w.file == nil || w.status != .Ok || len(w.index) >= 1_000_000 || len(p.geometries) < w.geometries || len(p.entities) < w.entities
	{
		return .Invalid_Data;
	}
	if len(w.index) > 0
	{
		previous: Index_Entry = w.index[len(w.index)-1];
		if p.frame.cycle < previous.cycle || (p.frame.cycle == previous.cycle && p.frame.tick <= previous.tick)
		{
			return .Invalid_Data;
		}
	}
	definitions_length: int = 8 + (len(p.entities)-w.entities)*16;
	for &g, i in p.geometries[w.geometries:]
	{
		if scene.geometry_admit(&g, w.geometries+i) != .Ok
		{
			return .Invalid_Data;
		}
		definitions_length += geometry_encoded_size(&g);
	}
	if scene.frame_admit(&p.frame, p.entities[:], p.geometries[:]) != .Ok
	{
		return .Invalid_Data;
	}
	frame_length: int = frame_encoded_size(&p.frame);
	compressed_capacity: int;
	compressed_capacity, result = compression_capacity(u64(frame_length));
	if result != .Ok
	{
		return result;
	}
	scratch_capacity: int = cap(w.scratch);
	if definitions_length+frame_length > scratch_capacity
	{
		scratch_capacity = max(definitions_length+frame_length, scratch_capacity*2);
	}
	if compressed_capacity > len(w.compressed)
	{
		compressed_capacity = max(compressed_capacity, min(int(max(i32)), len(w.compressed)*2));
	}
	index_capacity: int = cap(w.index);
	if len(w.index) == index_capacity
	{
		index_capacity = min(1_000_000, max(64, index_capacity*2));
	}
	// admit simultaneous old and replacement allocations, not only retained payloads
	required: u64 = writer_bytes(w);
	if scratch_capacity > cap(w.scratch)
	{
		required += u64(scratch_capacity);
	}
	if compressed_capacity > len(w.compressed)
	{
		required += u64(compressed_capacity);
	}
	if index_capacity > cap(w.index)
	{
		required += u64(index_capacity)*size_of(Index_Entry);
	}
	w.required_bytes = required;
	if required > w.budget
	{
		result = .Budget_Exceeded;
		return result;
	}
	w.peak_bytes = max(w.peak_bytes, required);
	if reserve(&w.scratch, scratch_capacity) != nil || reserve(&w.index, index_capacity) != nil
	{
		result = .Out_Of_Memory;
		return result;
	}
	if compressed_capacity > len(w.compressed)
	{
		replacement: []u8 = make([]u8, compressed_capacity);
		delete(w.compressed);
		w.compressed = replacement;
	}
	resize(&w.scratch, definitions_length+frame_length);
	c: Codec = {bytes=w.scratch[:]};
	encode_u32(&c, u32(len(p.geometries)-w.geometries));
	encode_u32(&c, u32(len(p.entities)-w.entities));
	for &g in p.geometries[w.geometries:]
	{
		encode_geometry(&c, &g);
	}
	for e in p.entities[w.entities:]
	{
		if int(e.geometry) >= len(p.geometries) || u32(e.kind) > 2
		{
			return .Invalid_Data;
		}
		encode_u32(&c, u32(e.kind));
		encode_u32(&c, u32(e.source_handle));
		encode_u32(&c, e.geometry);
		encode_u32(&c, e.group);
	}
	encode_frame(&c, &p.frame);
	if c.status != .Ok || c.at != len(w.scratch)
	{
		return .Invalid_Data;
	}
	compressed: []u8;
	compressed, result = compress_frame(w, w.scratch[definitions_length:]);
	if result != .Ok
	{
		return result;
	}
	e: Index_Entry = {w.offset, u64(definitions_length), w.offset+u64(definitions_length), u64(len(compressed)), u32(len(p.geometries)), u32(len(p.entities)), p.frame.tick, p.frame.phase, p.frame.cycle, u64(frame_length)};
	if writer_write(w, w.scratch[:definitions_length]) != .Ok || writer_write(w, compressed) != .Ok
	{
		return w.status;
	}
	append(&w.index, e);
	w.raw_bytes += u64(frame_length);
	w.compressed_bytes += u64(len(compressed));
	w.geometries, w.entities = len(p.geometries), len(p.entities);
	return .Ok;
}

writer_finish :: proc(w: ^Writer, completion: scene.Completion) -> scene.Status
{
	if len(w.index) == 0 || w.status != .Ok
	{
		return .Invalid_Data;
	}
	directory: u64 = w.offset;
	index_buffer: [V3_INDEX_BYTES]u8;
	for entry in w.index
	{
		c: Codec = {bytes=index_buffer[:]};
		encode_index(&c, entry);
		if writer_write(w, index_buffer[:]) != .Ok
		{
			return w.status;
		}
	}
	c: Codec = {bytes=index_buffer[:8]};
	encode_u64(&c, COMPLETE);
	if writer_write(w, index_buffer[:8]) != .Ok
	{
		return w.status;
	}
	header: [16]u8;
	c = {bytes=header[:]};
	encode_u64(&c, directory);
	encode_u32(&c, u32(len(w.index)));
	encode_u32(&c, u32(completion));
	written: int;
	error: os.Error;
	written, error = os.write_at(w.file, header[:], 16);
	if error != nil || written != len(header)
	{
		return .File_Error;
	}
	error = os.close(w.file);
	w.file = nil;
	if error != nil
	{
		return .File_Error;
	}
	// exclusive hard-link publication refuses a concurrently created destination
	if os.link(w.partial, w.path) != nil
	{
		return .File_Error;
	}
	if os.remove(w.partial) != nil
	{
		return .File_Error;
	}
	return .Ok;
}

reader_close :: proc(r: ^Reader)
{
	if r.file != nil
	{
		os.close(r.file);
	}
	metadata_destroy(&r.metadata);
	scene.packet_destroy(&r.packet);
	scene.frame_destroy(&r.spare);
	delete(r.index);
	delete(r.io);
	delete(r.decoded);
	r^ = {};
}

reader_bytes :: proc(r: ^Reader) -> u64
{
	return r.metadata_bytes + scene.packet_bytes(&r.packet) + scene.frame_bytes(&r.spare) + u64(len(r.index))*size_of(Index_Entry) + u64(len(r.io)+len(r.decoded));
}

read_region :: proc(r: ^Reader, offset, length: u64) -> ([]u8, scene.Status)
{
	if length > u64(max(int)) || offset > u64(max(i64))
	{
		return nil, .Invalid_Data;
	}
	if length > u64(len(r.io))
	{
		r.required_bytes = reader_bytes(r)+length;
		if r.required_bytes > r.budget
		{
			return nil, .Budget_Exceeded;
		}
		replacement: []u8 = make([]u8, int(length));
		delete(r.io);
		r.io = replacement;
	}
	bytes: []u8 = r.io[:int(length)];
	read: int;
	error: os.Error;
	read, error = os.read_at(r.file, bytes, i64(offset));
	if error != nil || read != len(bytes)
	{
		return nil, .Invalid_Data;
	}
	r.owned_bytes = reader_bytes(r);
	return bytes, .Ok;
}

reader_open :: proc(r: ^Reader, path: string, budget: u64 = DEFAULT_BUDGET) -> scene.Status
{
	r.budget = budget;
	r.selected = -1;
	r.prepared = -1;
	error: os.Error;
	r.file, error = os.open(path);
	if error != nil
	{
		return .File_Error;
	}
	status: scene.Status = .Invalid_Data;
	defer if status != .Ok
	{
		required: u64 = r.required_bytes;
		reader_close(r);
		r.required_bytes, r.budget = required, budget;
	}
	size: i64;
	size_error: os.Error;
	size, size_error = os.file_size(r.file);
	if size_error != nil || size < V2_HEADER_BYTES
	{
		return status;
	}
	header: [V2_HEADER_BYTES]u8;
	read: int;
	read_error: os.Error;
	read, read_error = os.read_at(r.file, header[:], 0);
	if read_error != nil || read != len(header) || string(header[:8]) != "ENTRPLY\x00"
	{
		return status;
	}
	c: Codec = {bytes=header[8:]};
	r.version = decode_u32(&c);
	if r.version == 1
	{
		status = reader_open_v1(r, u64(size));
		return status;
	}
	if r.version != 2 && r.version != 3
	{
		return status;
	}
	meta_length: u64 = u64(decode_u32(&c));
	directory: u64 = decode_u64(&c);
	count: u64 = u64(decode_u32(&c));
	r.metadata.completion = scene.Completion(decode_u32(&c));
	header_length: u64 = V2_HEADER_BYTES;
	index_length: u64 = INDEX_BYTES;
	if r.version == 3
	{
		envelope: [8]u8;
		read, read_error = os.read_at(r.file, envelope[:], V2_HEADER_BYTES);
		if read_error != nil || read != len(envelope)
		{
			return status;
		}
		c = {bytes=envelope[:]};
		if decode_u32(&c) != BLOCK_CODEC
		{
			return status;
		}
		r.compression = Compression(decode_u32(&c));
		if u32(r.compression) > u32(Compression.LZ4HC)
		{
			return status;
		}
		header_length, index_length = V3_HEADER_BYTES, V3_INDEX_BYTES;
	}
	if meta_length > 65536 || count == 0 || count > 1_000_000 || directory < header_length+meta_length ||
		directory > u64(size) || count*index_length+8 != u64(size)-directory || u32(r.metadata.completion) > 1
	{
		return status;
	}
	completion: scene.Completion = r.metadata.completion;
	r.required_bytes = meta_length*2+count*size_of(Index_Entry);
	if r.required_bytes > budget
	{
		return .Budget_Exceeded;
	}
	metadata: []u8;
	meta_status: scene.Status;
	metadata, meta_status = read_region(r, header_length, meta_length);
	if meta_status != .Ok
	{
		return meta_status;
	}
	c = {bytes=metadata};
	r.metadata = decode_metadata(&c);
	r.metadata.completion = completion;
	if c.status != .Ok || c.at != len(metadata)
	{
		return status;
	}
	r.metadata_bytes = u64(len(r.metadata.scenario)+len(r.metadata.source)+len(r.metadata.revision)+len(r.metadata.compiler)+len(r.metadata.host)+len(r.metadata.configuration)+len(r.metadata.components)+len(r.metadata.settings));
	r.index = make([]Index_Entry, int(count));
	end: u64 = header_length+meta_length;
	maximum_io, maximum_decoded: u64;
	index_buffer: [V3_INDEX_BYTES]u8;
	for &entry, i in r.index
	{
		read, read_error = os.read_at(r.file, index_buffer[:int(index_length)], i64(directory+u64(i)*index_length));
		if read_error != nil || u64(read) != index_length
		{
			return status;
		}
		c = {bytes=index_buffer[:int(index_length)]};
		entry = decode_index(&c, r.version);
		if entry.definitions_offset != end || entry.definitions_length < 8 || entry.definitions_length > directory-end
		{
			return status;
		}
		end += entry.definitions_length;
		if entry.frame_offset != end || entry.frame_length == 0 || entry.frame_length > directory-end || entry.frame_length > u64(max(i32)) ||
			entry.decoded_length < 40 || entry.decoded_length > MAX_BLOCK_BYTES || u32(entry.phase) > u32(scene.Phase.Teardown)
		{
				return status;
		}
		end += entry.frame_length;
		if i > 0
		{
			previous: Index_Entry = r.index[i-1];
			if entry.geometries < previous.geometries || entry.entities < previous.entities ||
				entry.cycle < previous.cycle || (entry.cycle == previous.cycle && entry.tick <= previous.tick)
			{
				return status;
			}
		}
		maximum_io = max(maximum_io, entry.definitions_length, entry.frame_length);
		if r.version == 3
		{
			maximum_decoded = max(maximum_decoded, entry.decoded_length);
		}
	}
	read, read_error = os.read_at(r.file, index_buffer[:8], size-8);
	c = {bytes=index_buffer[:8]};
	if read_error != nil || read != 8 || end != directory || decode_u64(&c) != COMPLETE
	{
		return status;
	}
	last: Index_Entry = r.index[len(r.index)-1];
	required: u64 = reader_bytes(r) + maximum_io + maximum_decoded + u64(last.geometries)*size_of(scene.Geometry) + u64(last.entities)*size_of(scene.Entity);
	r.required_bytes = required;
	if required > budget
	{
		return .Budget_Exceeded;
	}
	delete(r.io);
	r.io = make([]u8, int(maximum_io));
	r.decoded = make([]u8, int(maximum_decoded));
	if reserve(&r.packet.geometries, int(last.geometries)) != nil || reserve(&r.packet.entities, int(last.entities)) != nil
	{
		return .Out_Of_Memory;
	}
	for entry in r.index
	{
		definition_status: scene.Status = reader_definitions(r, entry);
		if definition_status != .Ok
		{
			return definition_status;
		}
	}
	r.owned_bytes = reader_bytes(r);
	status = .Ok;
	return status;
}

reader_definitions :: proc(r: ^Reader, entry: Index_Entry) -> scene.Status
{
	bytes: []u8;
	status: scene.Status;
	bytes, status = read_region(r, entry.definitions_offset, entry.definitions_length);
	if status != .Ok
	{
		return status;
	}
	c: Codec = {bytes=bytes};
	geometries: int = int(decode_u32(&c));
	entities: int = int(decode_u32(&c));
	if geometries+len(r.packet.geometries) != int(entry.geometries) || entities+len(r.packet.entities) != int(entry.entities) ||
		geometries*52+entities*16 > len(bytes)-8
	{
			return .Invalid_Data;
	}
	// encoded vertex/index/child payload bounds the corresponding decoded slices
	required: u64 = reader_bytes(r) + u64(len(bytes));
	r.required_bytes = required;
	if required > r.budget
	{
		return .Budget_Exceeded;
	}
	for _ in 0 ..< geometries
	{
		g: scene.Geometry = decode_geometry(&c);
		if c.status != .Ok || scene.geometry_admit(&g, len(r.packet.geometries)) != .Ok
		{
			scene.geometry_destroy(&g);
			return .Invalid_Data;
		}
		append(&r.packet.geometries, g);
	}
	for _ in 0 ..< entities
	{
		e: scene.Entity;
		e.kind = scene.Object_Kind(decode_u32(&c));
		e.source_handle = i32(decode_u32(&c));
		e.geometry = decode_u32(&c);
		e.group = decode_u32(&c);
		if u32(e.kind) > 2 || int(e.geometry) >= len(r.packet.geometries)
		{
			return .Invalid_Data;
		}
		append(&r.packet.entities, e);
	}
	r.owned_bytes = reader_bytes(r);
	if c.status != .Ok || c.at != len(bytes)
	{
		return .Invalid_Data;
	}
	return .Ok;
}

reader_prepare :: proc(r: ^Reader, index: int) -> scene.Status
{
	defer r.owned_bytes = reader_bytes(r);
	r.prepared = -1;
	if index < 0 || index >= len(r.index)
	{
		return .Invalid_Data;
	}
	if r.version == 1
	{
		return reader_prepare_v1(r, index);
	}
	e: Index_Entry = r.index[index];
	r.required_bytes = reader_bytes(r);
	if r.required_bytes > r.budget
	{
		return .Budget_Exceeded;
	}
	bytes: []u8;
	status: scene.Status;
	bytes, status = read_region(r, e.frame_offset, e.frame_length);
	if status != .Ok
	{
		return status;
	}
	if r.version == 3
	{
		bytes, status = decompress_frame(r, bytes, e.decoded_length);
		if status != .Ok
		{
			return status;
		}
	}
	if len(bytes) < 40
	{
		return .Invalid_Data;
	}
	c: Codec = {bytes=bytes, at=28};
	instances: int = int(decode_u32(&c));
	overlays: int = int(decode_u32(&c));
	metrics: int = int(decode_u32(&c));
	if u64(instances)*36+u64(overlays)*60+u64(metrics)*12 != u64(len(bytes)-40)
	{
		return .Invalid_Data;
	}
	f: ^scene.Frame = &r.spare;
	instance_capacity: int = max(cap(f.ids), instances);
	overlay_capacity: int = max(cap(f.overlays), overlays);
	metric_capacity: int = max(cap(f.metrics), metrics);
	if instances > cap(f.ids)
	{
		instance_capacity = max(instances, cap(f.ids)*2);
	}
	if overlays > cap(f.overlays)
	{
		overlay_capacity = max(overlays, cap(f.overlays)*2);
	}
	if metrics > cap(f.metrics)
	{
		metric_capacity = max(metrics, cap(f.metrics)*2);
	}
	required: u64 = reader_bytes(r);
	if instance_capacity > cap(f.ids)
	{
		required += u64(instance_capacity)*4;
	}
	if instance_capacity > cap(f.poses)
	{
		required += u64(instance_capacity)*size_of(scene.Pose);
	}
	if instance_capacity > cap(f.states)
	{
		required += u64(instance_capacity)*size_of(scene.State);
	}
	if overlay_capacity > cap(f.overlays)
	{
		required += u64(overlay_capacity)*size_of(scene.Overlay);
	}
	if metric_capacity > cap(f.metrics)
	{
		required += u64(metric_capacity)*size_of(scene.Metric);
	}
	r.required_bytes = required;
	if required > r.budget
	{
		return .Budget_Exceeded;
	}
	if scene.frame_reserve(f, instance_capacity, overlay_capacity, metric_capacity) != .Ok
	{
		return .Out_Of_Memory;
	}
	c = {bytes=bytes};
	status = decode_frame(&c, &r.spare);
	if status != .Ok
	{
		return status;
	}
	if r.spare.tick != e.tick || r.spare.phase != e.phase || r.spare.cycle != e.cycle ||
		scene.frame_admit(&r.spare, r.packet.entities[:int(e.entities)], r.packet.geometries[:int(e.geometries)]) != .Ok
	{
			return .Invalid_Data;
	}
	r.prepared = index;
	return .Ok;
}

reader_commit :: proc(r: ^Reader)
{
	r.packet.frame, r.spare = r.spare, r.packet.frame;
	r.selected = r.prepared;
	r.prepared = -1;
}

reader_seek :: proc(r: ^Reader, index: int) -> scene.Status
{
	status: scene.Status = reader_prepare(r, index);
	if status != .Ok
	{
		return status;
	}
	reader_commit(r);
	return .Ok;
}

comparison_index :: proc(a, b: ^Reader, budget: u64) -> ([][2]int, scene.Status)
{
	if scene.comparison_admit(a.metadata, b.metadata) != .Ok
	{
		return nil, .Unsupported;
	}
	pairs: [][2]int;
	for pass in 0 ..< 2
	{
		i, j, count: int;
		for i<len(a.index) && j<len(b.index)
		{
			left, right: Index_Entry = a.index[i], b.index[j];
			if left.cycle<right.cycle || (left.cycle==right.cycle && left.tick<right.tick)
			{
				i+=1;
				continue;
			}
			if right.cycle<left.cycle || (left.cycle==right.cycle && right.tick<left.tick)
			{
				j+=1;
				continue;
			}
			if left.phase==right.phase
			{
				if pass==1
				{
					pairs[count]={i, j};
				}
				count+=1;
			}
			i+=1;
			j+=1;
		}
		if pass==0
		{
			if count==0
			{
				return nil, .Unsupported;
			}
			a.required_bytes = u64(count)*size_of([2]int);
			if a.required_bytes>budget
			{
				return nil, .Budget_Exceeded;
			}
			pairs=make([][2]int, count);
		}
	}
	return pairs, .Ok;
}
