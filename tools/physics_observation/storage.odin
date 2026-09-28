package physics_observation

import "base:runtime"
import scene "../physics_scene"

observation_bytes :: proc(o: ^Observation) -> u64
{
	return size_of(Observation)+scene.packet_bytes(&o.packet)+u64(cap(o.shapes))*size_of(Shape_Binding)+
		u64(cap(o.lifetimes))*size_of(scene.Availability)+u64(cap(o.previews))*size_of(Shape_Preview);
}

Observation_Growth :: struct
{
	current, requested: int,
	stride: u64,
}

observation_reserve :: proc(o: ^Observation, geometries: int = 0, entities: int = 0, instances: int = 0, previews: int = 0, overlays: int = 0, metrics: int = 0) -> scene.Status
{
	f: ^scene.Frame = &o.packet.frame;
	growth: [9]Observation_Growth = {
		{cap(o.packet.geometries), geometries, size_of(scene.Geometry)},
		{cap(o.shapes), geometries, size_of(Shape_Binding)},
		{cap(o.packet.entities), entities, size_of(scene.Entity)},
		{cap(o.lifetimes), entities, size_of(scene.Availability)},
		{cap(o.previews), previews, size_of(Shape_Preview)},
		{cap(f.ids), instances, size_of(u32)},
		{cap(f.poses), instances, size_of(scene.Pose)},
		{cap(f.states), instances, size_of(scene.State)},
		{cap(f.overlays), overlays, size_of(scene.Overlay)},
	};
	added, temporary: u64;
	for request in growth
	{
		if request.requested > request.current
		{
			added += u64(request.requested-request.current)*request.stride;
			temporary = max(temporary, u64(request.current)*request.stride);
		}
	}
	if metrics > cap(f.metrics)
	{
		added += u64(metrics-cap(f.metrics))*size_of(scene.Metric);
		temporary = max(temporary, u64(cap(f.metrics))*size_of(scene.Metric));
	}
	if added == 0
	{
		return .Ok;
	}
	// a resize may hold its old buffer until the admitted replacement is copied
	o.required_bytes = observation_bytes(o)+o.pending_bytes+added+temporary;
	if o.required_bytes > o.budget
	{
		return .Budget_Exceeded;
	}
	if reserve(&o.packet.geometries, geometries) != nil || reserve(&o.shapes, geometries) != nil ||
		reserve(&o.packet.entities, entities) != nil || reserve(&o.lifetimes, entities) != nil ||
		reserve(&o.previews, previews) != nil
	{
		return .Out_Of_Memory;
	}
	return scene.frame_reserve(f, instances, overlays, metrics);
}

observation_geometry_allocate :: proc(o: ^Observation, geometry: ^scene.Geometry, vertices: int = 0, indices: int = 0, children: int = 0) -> scene.Status
{
	bytes: u64 = u64(vertices)*size_of(scene.Vec3)+u64(indices)*size_of(u32)+u64(children)*size_of(scene.Geometry_Child);
	o.required_bytes = observation_bytes(o)+o.pending_bytes+bytes;
	if o.required_bytes > o.budget
	{
		return .Budget_Exceeded;
	}
	error: runtime.Allocator_Error;
	geometry.vertices, error = make([]scene.Vec3, vertices);
	if error != nil
	{
		return .Out_Of_Memory;
	}
	o.pending_bytes += u64(vertices)*size_of(scene.Vec3);
	geometry.indices, error = make([]u32, indices);
	if error != nil
	{
		return .Out_Of_Memory;
	}
	o.pending_bytes += u64(indices)*size_of(u32);
	geometry.children, error = make([]scene.Geometry_Child, children);
	if error != nil
	{
		return .Out_Of_Memory;
	}
	o.pending_bytes += u64(children)*size_of(scene.Geometry_Child);
	return .Ok;
}
