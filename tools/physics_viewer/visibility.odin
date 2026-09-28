package physics_viewer

import scene "../physics_scene"

Visibility_Flag :: enum
{
	Statics, Sleeping, Sensors, Markers, Overlays,
}
View_Filter :: struct
{
	hidden: bit_set[Visibility_Flag],
	group, query_first, query_count: i32,
}

entity_visible :: proc(p: ^scene.Scene_Packet, id: u32, frame_index: int, filter: View_Filter) -> scene.Availability
{
	entity: scene.Entity = p.entities[id];
	state: scene.State = p.frame.states[frame_index];
	if (filter.group > 0 && u64(entity.group)+1 != u64(filter.group)) ||
		(.Statics in filter.hidden && .Static in state) ||
		(.Sleeping in filter.hidden && .Sleeping in state) ||
		(.Markers in filter.hidden && p.geometries[entity.geometry].kind == .Marker)
	{
		return .Unavailable;
	}
	return .Available;
}

overlay_visible :: proc(p: ^scene.Scene_Packet, overlay: scene.Overlay, filter: View_Filter, query: i32) -> scene.Availability
{
	if .Overlays in filter.hidden || overlay.kind == .Part_State ||
		(query >= 0 && filter.query_count > 0 && (query < filter.query_first || query-filter.query_first >= filter.query_count))
	{
		return .Unavailable;
	}
	if filter.group > 0 && (overlay.entity_a != scene.NO_ENTITY || overlay.entity_b != scene.NO_ENTITY)
	{
		group: u64 = u64(filter.group)-1;
		if (overlay.entity_a == scene.NO_ENTITY || u64(p.entities[overlay.entity_a].group) != group) &&
			(overlay.entity_b == scene.NO_ENTITY || u64(p.entities[overlay.entity_b].group) != group)
		{
			return .Unavailable;
		}
	}
	return .Available;
}

