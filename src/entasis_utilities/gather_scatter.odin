// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

@(private)
gather_get_unsafe :: proc "contextless" (vector: ^$Vector, $Element: typeid, index: int) -> ^Element
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=GatherScatter phase=private_kernel reason=checked_lane_reference lifetime=until_owner_storage_mutation
	values := ([^]Element)(vector);
	return &values[index];
}

gather_get_checked :: proc "contextless" (vector: ^$Vector, $Element: typeid, index: int) -> (^Element, Memory_Status)
{
	when size_of(Vector) % size_of(Element) != 0
	{
		return nil, .Invalid_Count;
	}
	if vector == nil
	{
		return nil, .Invalid_Buffer;
	}
	if uintptr(vector) & (align_of(Element) - 1) != 0
	{
		return nil, .Invalid_Alignment;
	}
	if index < 0 || index >= size_of(Vector) / size_of(Element)
	{
		return nil, .Invalid_Count;
	}
	return gather_get_unsafe(vector, Element, index), .Ok;
}

@(private)
gather_copy_lane_kernel :: proc "contextless" (source, target: ^$T, source_lane, target_lane: int)
{
	size_in_i32 := (size_of(T) >> 2) & ~int(BUNDLE_VECTOR_MASK);
	source_values := ([^]i32)(source);
	target_values := ([^]i32)(target);
	target_values[target_lane] = source_values[source_lane];

	offset := PRODUCTION_LANE_COUNT;
	for offset + PRODUCTION_LANE_COUNT * 8 <= size_in_i32
	{
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
	}
	if offset + PRODUCTION_LANE_COUNT * 4 <= size_in_i32
	{
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
	}
	if offset + PRODUCTION_LANE_COUNT * 2 <= size_in_i32
	{
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
		target_values[target_lane + offset] = source_values[source_lane + offset];
		offset += PRODUCTION_LANE_COUNT;
	}
	if offset + PRODUCTION_LANE_COUNT <= size_in_i32
	{
		target_values[target_lane + offset] = source_values[source_lane + offset];
	}
}

gather_copy_lane_checked :: proc "contextless" (source, target: ^$T, source_lane, target_lane: int) -> Memory_Status
{
	if source == nil || target == nil
	{
		return .Invalid_Buffer;
	}
	if uintptr(source) & (PRODUCTION_ALIGNMENT - 1) != 0 || uintptr(target) & (PRODUCTION_ALIGNMENT - 1) != 0
	{
		return .Invalid_Alignment;
	}
	if size_of(T) < PRODUCTION_ALIGNMENT ||
		source_lane < 0 ||
		source_lane >= PRODUCTION_LANE_COUNT ||
		target_lane < 0 ||
		target_lane >= PRODUCTION_LANE_COUNT
	{
		return .Invalid_Count;
	}
	gather_copy_lane_kernel(source, target, source_lane, target_lane);
	return .Ok;
}

@(private)
gather_clear_lane_kernel :: proc "contextless" (target: ^$Outer, $Lane: typeid, lane: int)
{
	target_values := ([^]Lane)(target);
	vector_count := size_of(Outer) / (PRODUCTION_LANE_COUNT * size_of(Lane));
	for vector_index in 0 ..< vector_count
	{
		target_values[lane + vector_index * PRODUCTION_LANE_COUNT] = {};
	}
}

gather_clear_lane_checked :: proc "contextless" (target: ^$Outer, $Lane: typeid, lane: int) -> Memory_Status
{
	VECTOR_BYTE_SIZE :: PRODUCTION_LANE_COUNT * size_of(Lane);
	when size_of(Outer) % VECTOR_BYTE_SIZE != 0
	{
		return .Invalid_Count;
	}
	if target == nil
	{
		return .Invalid_Buffer;
	}
	if uintptr(target) & (PRODUCTION_ALIGNMENT - 1) != 0
	{
		return .Invalid_Alignment;
	}
	if size_of(Outer) < VECTOR_BYTE_SIZE || lane < 0 || lane >= PRODUCTION_LANE_COUNT
	{
		return .Invalid_Count;
	}
	gather_clear_lane_kernel(target, Lane, lane);
	return .Ok;
}

@(private)
gather_get_offset_instance_unsafe :: proc "contextless" (bundle_container: ^$T, inner_index: int) -> ^T
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=GatherScatter phase=private_kernel reason=checked_shifted_bundle_reference lifetime=until_owner_storage_mutation
	return (^T)(rawptr(uintptr(bundle_container) + uintptr(inner_index * size_of(f32))));
}

gather_get_offset_instance_checked :: proc "contextless" (
	bundle_container: ^$T,
	inner_index: int
) -> (^T, Memory_Status)
{
	if bundle_container == nil
	{
		return nil, .Invalid_Buffer;
	}
	if uintptr(bundle_container) & (PRODUCTION_ALIGNMENT - 1) != 0
	{
		return nil, .Invalid_Alignment;
	}
	if size_of(T) < PRODUCTION_ALIGNMENT || inner_index < 0 || inner_index >= PRODUCTION_LANE_COUNT
	{
		return nil, .Invalid_Count;
	}
	return gather_get_offset_instance_unsafe(bundle_container, inner_index), .Ok;
}

@(private)
gather_get_first_unsafe :: proc "contextless" (vector: ^$Vector, $Element: typeid) -> ^Element
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=GatherScatter phase=private_kernel reason=checked_first_lane_reference lifetime=until_owner_storage_mutation
	return (^Element)(vector);
}

gather_get_first_checked :: proc "contextless" (vector: ^$Vector, $Element: typeid) -> (^Element, Memory_Status)
{
	when size_of(Vector) < size_of(Element)
	{
		return nil, .Invalid_Count;
	}
	if vector == nil
	{
		return nil, .Invalid_Buffer;
	}
	if uintptr(vector) & (align_of(Element) - 1) != 0
	{
		return nil, .Invalid_Alignment;
	}
	return gather_get_first_unsafe(vector, Element), .Ok;
}
