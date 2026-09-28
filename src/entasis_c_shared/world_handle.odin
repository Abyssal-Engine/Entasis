package entasis_c_shared

import entasis "entasis:entasis"

// private layout cookie, not the public ABI generation. mismatched runtime and
// cooking libraries must reject the handle before reading the shared state or
// its native world pointee. hierarchy and joint watches change native root offsets
WORLD_MAGIC :: u64(0x454E54574F524C4B);
World_Access :: enum u8
{
	Ready,
	Exclusive,
	Read_Phase,
	Disposed,
}

// caller-synchronized state shared by matching runtime/cooking libraries.
// begin/end read surround the application's job barrier. this is not a lock
World_Header :: struct
{
	magic: u64,
	world: ^entasis.World,
	owner: rawptr,
	access: World_Access,
	query_context_count: i32,
}

World_Header_Get :: #force_inline proc "contextless" (opaque: rawptr) -> ^World_Header
{
	if opaque == nil
	{
		return nil;
	}
	header := (^World_Header)(opaque);
	if header.magic != WORLD_MAGIC || header.world == nil || header.owner == nil
	{
		return nil;
	}
	return header;
}

World_Get :: #force_inline proc "contextless" (opaque: rawptr) -> ^entasis.World
{
	header := World_Header_Get(opaque);
	if header == nil
	{
		return nil;
	}
	return header.world;
}
