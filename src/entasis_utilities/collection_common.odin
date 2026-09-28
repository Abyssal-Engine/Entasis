// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:runtime"

Collection_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Capacity_Missing,
	Out_Of_Memory,
	Duplicate,
	Not_Found,
	Empty,
	Disposed,
}

Presence_Status :: enum u8
{
	Missing,
	Present,
}

Comparison_Status :: enum u8
{
	Different,
	Equal,
}

Replacement_Status :: enum u8
{
	Added,
	Replaced,
}

Compare_Proc :: #type proc "contextless" (a, b, comparer_context: rawptr) -> int;
Hash_Proc :: #type proc "contextless" (value: rawptr) -> i32;
Equal_Proc :: #type proc "contextless" (a, b: rawptr) -> Comparison_Status;
Predicate_Proc :: #type proc "contextless" (value: rawptr, predicate_context: rawptr) -> Comparison_Status;
For_Each_Ref_Proc :: #type proc "contextless" (value: rawptr, loop_context: rawptr);
Breakable_For_Each_Ref_Proc :: #type proc "contextless" (value: rawptr, loop_context: rawptr) -> Loop_Control;

Loop_Control :: enum u8
{
	Break,
	Continue,
}

Comparer_Procs :: struct
{
	compare:          Compare_Proc,
	comparer_context: rawptr, // immutable and valid for the duration of a synchronous sort call
}

Hash_Equal_Procs :: struct
{
	hash:  Hash_Proc,
	equal: Equal_Proc,
}

hash_rehash :: proc "contextless" (hash: i32) -> i32
{
	a :: 6;
	b :: 13;
	c :: 25;
	value := u32(hash) * 982451653;
	redongled := ((value << a) | (value >> (32 - a))) ~
	((value << b) | (value >> (32 - b))) ~
	((value << c) | (value >> (32 - c)));
	return i32(redongled);
}

primitive_compare_i32 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^i32)(a)^;
	right := (^i32)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_u64 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^u64)(a)^;
	right := (^u64)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_i8 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^i8)(a)^;
	right := (^i8)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_u8 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^u8)(a)^;
	right := (^u8)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_i16 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^i16)(a)^;
	right := (^i16)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_u16 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^u16)(a)^;
	right := (^u16)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_u32 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^u32)(a)^;
	right := (^u32)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_i64 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^i64)(a)^;
	right := (^i64)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_f32 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^f32)(a)^;
	right := (^f32)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_compare_f64 :: proc "contextless" (a, b, _context: rawptr) -> int
{
	left := (^f64)(a)^;
	right := (^f64)(b)^;
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

primitive_hash_i32 :: proc "contextless" (value: rawptr) -> i32
{
	return (^i32)(value)^;
}

primitive_hash_u64 :: proc "contextless" (value: rawptr) -> i32
{
	bits := (^u64)(value)^;
	return i32(u32(bits) ~ u32(bits >> 32));
}

primitive_hash_i8 :: proc "contextless" (value: rawptr) -> i32
{
	return i32((^i8)(value)^);
}
primitive_hash_u8 :: proc "contextless" (value: rawptr) -> i32
{
	return i32((^u8)(value)^);
}
primitive_hash_i16 :: proc "contextless" (value: rawptr) -> i32
{
	return i32((^i16)(value)^);
}
primitive_hash_u16 :: proc "contextless" (value: rawptr) -> i32
{
	return i32((^u16)(value)^);
}
primitive_hash_u32 :: proc "contextless" (value: rawptr) -> i32
{
	return i32((^u32)(value)^);
}
primitive_hash_i64 :: proc "contextless" (value: rawptr) -> i32
{
	bits := u64((^i64)(value)^);
	return i32(u32(bits) ~ u32(bits >> 32));
}
primitive_hash_f32 :: proc "contextless" (value: rawptr) -> i32
{
	bits := transmute(u32)(^f32)(value)^;
	if bits == 0x8000_0000 || (bits & 0x7f80_0000) == 0x7f80_0000 && (bits & 0x007f_ffff) != 0
	{
		bits &= 0x7fff_ffff;
	}
	return i32(bits);
}
primitive_hash_f64 :: proc "contextless" (value: rawptr) -> i32
{
	bits := transmute(u64)(^f64)(value)^;
	if bits == 0x8000_0000_0000_0000 ||
		(bits & 0x7ff0_0000_0000_0000) == 0x7ff0_0000_0000_0000 &&
		(bits & 0x000f_ffff_ffff_ffff) != 0
	{
		bits &= 0x7fff_ffff_ffff_ffff;
	}
	return i32(u32(bits) ~ u32(bits >> 32));
}

primitive_equal_i8 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^i8)(a)^ == (^i8)(b)^
	{
		return .Equal;
	}
	return .Different;
}
primitive_equal_u8 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^u8)(a)^ == (^u8)(b)^
	{
		return .Equal;
	}
	return .Different;
}
primitive_equal_i16 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^i16)(a)^ == (^i16)(b)^
	{
		return .Equal;
	}
	return .Different;
}
primitive_equal_u16 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^u16)(a)^ == (^u16)(b)^
	{
		return .Equal;
	}
	return .Different;
}
primitive_equal_u32 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^u32)(a)^ == (^u32)(b)^
	{
		return .Equal;
	}
	return .Different;
}
primitive_equal_i64 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^i64)(a)^ == (^i64)(b)^
	{
		return .Equal;
	}
	return .Different;
}
primitive_equal_f32 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^f32)(a)^ == (^f32)(b)^
	{
		return .Equal;
	}
	return .Different;
}
primitive_equal_f64 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^f64)(a)^ == (^f64)(b)^
	{
		return .Equal;
	}
	return .Different;
}

reference_hash :: proc "contextless" (value: rawptr) -> i32
{
	address := uintptr((^rawptr)(value)^);
	return i32(u32(address) ~ u32(address >> 32));
}

reference_equal :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^rawptr)(a)^ == (^rawptr)(b)^
	{
		return .Equal;
	}
	return .Different;
}

primitive_equal_i32 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^i32)(a)^ == (^i32)(b)^
	{
		return .Equal;
	}
	return .Different;
}

primitive_equal_u64 :: proc "contextless" (a, b: rawptr) -> Comparison_Status
{
	if (^u64)(a)^ == (^u64)(b)^
	{
		return .Equal;
	}
	return .Different;
}

primitive_i32_comparer :: proc "contextless" () -> Comparer_Procs
{
	return {compare=primitive_compare_i32};
}

primitive_u64_comparer :: proc "contextless" () -> Comparer_Procs
{
	return {compare=primitive_compare_u64};
}

primitive_i32_hash_equal :: proc "contextless" () -> Hash_Equal_Procs
{
	return {hash=primitive_hash_i32, equal=primitive_equal_i32};
}

primitive_u64_hash_equal :: proc "contextless" () -> Hash_Equal_Procs
{
	return {hash=primitive_hash_u64, equal=primitive_equal_u64};
}

bytewise_equal :: proc "contextless" (a, b: rawptr, byte_count: int) -> Comparison_Status
{
	if runtime.memory_compare(a, b, byte_count) == 0
	{
		return .Equal;
	}
	return .Different;
}

memory_to_collection_status :: proc "contextless" (status: Memory_Status) -> Collection_Status
{
	switch status
	{
		case .Ok:
			return .Ok;
		case .Capacity_Missing:
			return .Capacity_Missing;
		case .Out_Of_Memory:
			return .Out_Of_Memory;
		case .Pool_Disposed:
			return .Disposed;
		case .Invalid_Count, .Invalid_Alignment, .Invalid_Power, .Invalid_Buffer,
			.Overflow:
			return .Invalid_Argument;
	}
	return .Invalid_Argument;
}
