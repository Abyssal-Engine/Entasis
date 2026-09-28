package physics_replay

// version 1 is little endian and contains no struct padding
MAGIC :: [8]u8{'E', 'N', 'T', 'R', 'P', 'L', 'Y', 0};
Scene :: enum u32
{
	Container, Contact_Islands, Pyramid
}
Status :: enum
{
	Ok, File_Error, Invalid_Data, Incompatible
}
Header :: struct
{
	magic: [8]u8,
	version: u32,
	scene: Scene,
	body_count: u32,
	static_count: u32,
	frame_count: u32,
	workers: u32,
	timestep_hz: u32,
	measured_start: u32,
	source: [40]u8,
	box_size: [3]f32,
	below_floor_y: f32,
}
Static_Box :: struct
{
	position, size: [3]f32
}
// pyramid records this section between the common header and static boxes
Pyramid_Description :: struct
{
	box_count: u32,
	projectile_radius: f32,
	launch_step: u32,
}
Pose :: [7]f32;
Frame :: struct
{
	step, constraints, pairs, below_floor: u32
}
#assert(size_of(Header) == 96);
#assert(size_of(Static_Box) == 24);
#assert(size_of(Frame) == 16);
#assert(size_of(Pose) == 28);
#assert(size_of(Pyramid_Description) == 12);

finite :: proc(value: f32) -> Status
{
	return .Ok if transmute(u32)value & 0x7f80_0000 != 0x7f80_0000 else .Invalid_Data;
}

source_text :: proc(header: ^Header) -> string
{
	length: int;
	for value in header.source
	{
		if value == 0
		{
			break;
		}
		length += 1;
	}
	return string(header.source[:length]);
}
