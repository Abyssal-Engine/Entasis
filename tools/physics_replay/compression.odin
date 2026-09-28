package physics_replay

import "core:c"
import "core:time"
import lz4 "vendor:compress/lz4"
import scene "../physics_scene"

Compression :: enum u32
{
	LZ4, LZ4HC,
}
BLOCK_CODEC :: u32(1);
MAX_BLOCK_BYTES :: u64(lz4.MAX_INPUT_SIZE);

compression_parse :: proc(text: string) -> (Compression, scene.Status)
{
	switch text
	{
	case "lz4": return .LZ4, .Ok;
	case "lz4hc": return .LZ4HC, .Ok;
	}
	return {}, .Invalid_Data;
}

compression_capacity :: proc(length: u64) -> (int, scene.Status)
{
	if length == 0 || length > MAX_BLOCK_BYTES
	{
		return 0, .Invalid_Data;
	}
	return int(lz4.compressBound(c.int(length))), .Ok;
}

compression_state_size :: proc(mode: Compression) -> int
{
	return int(lz4.sizeofStateHC()) if mode == .LZ4HC else int(lz4.sizeofState());
}

compress_frame :: proc(w: ^Writer, bytes: []u8) -> ([]u8, scene.Status)
{
	start: time.Tick = time.tick_now();
	defer w.encode_time += time.tick_since(start);
	length: c.int;
	switch w.compression
	{
	case .LZ4:
		length = lz4.compress_fast_extState(raw_data(w.encoder), raw_data(bytes), raw_data(w.compressed), c.int(len(bytes)), c.int(len(w.compressed)), 1);
	case .LZ4HC:
		length = lz4.compress_HC_extStateHC(raw_data(w.encoder), raw_data(bytes), raw_data(w.compressed), c.int(len(bytes)), c.int(len(w.compressed)), lz4.CLEVEL_DEFAULT);
	}
	if length <= 0
	{
		return nil, .Invalid_Data;
	}
	return w.compressed[:int(length)], .Ok;
}

decompress_frame :: proc(r: ^Reader, bytes: []u8, decoded_length: u64) -> ([]u8, scene.Status)
{
	start: time.Tick = time.tick_now();
	defer r.decode_time += time.tick_since(start);
	length: c.int = lz4.decompress_safe(raw_data(bytes), raw_data(r.decoded), c.int(len(bytes)), c.int(decoded_length));
	if length < 0 || u64(length) != decoded_length
	{
		return nil, .Invalid_Data;
	}
	return r.decoded[:int(decoded_length)], .Ok;
}
