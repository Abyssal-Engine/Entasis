package physics_observation

import "core:simd"
import entasis "entasis:entasis"
import scene "../physics_scene"

Shape_Binding :: struct
{
	handle: entasis.Shape_Handle,
	geometry: u32,
}
Shape_Preview :: struct
{
	id: u32,
	pose: scene.Pose,
	state: scene.State,
}
Observation :: struct
{
	packet: scene.Scene_Packet,
	shapes: [dynamic]Shape_Binding,
	lifetimes: [dynamic]scene.Availability,
	previews: [dynamic]Shape_Preview,
	part_capacity, overlay_capacity: int,
	budget, pending_bytes, required_bytes: u64,
}

copy_pose :: proc(p: entasis.Rigid_Pose) -> scene.Pose
{
	return {{p.position.x, p.position.y, p.position.z}, quaternion(x=p.orientation.x, y=p.orientation.y, z=p.orientation.z, w=p.orientation.w)};
}

copy_vector :: proc(v: entasis.Vector3) -> scene.Vec3
{
	return {v.x, v.y, v.z};
}

geometry_copy :: proc(o: ^Observation, world: ^entasis.World, handle: entasis.Shape_Handle) -> (u32, scene.Status)
{
	for binding in o.shapes
	{
		if binding.handle == handle
		{
			return binding.geometry, .Ok;
		}
	}
	g: scene.Geometry;
	status: scene.Status = .Invalid_Data;
	defer
	{
		o.pending_bytes -= scene.geometry_bytes(&g)-size_of(scene.Geometry);
		if status != .Ok
		{
			scene.geometry_destroy(&g);
		}
	}
	if handle.packed == 0
	{
		g.kind = .Marker;
		g.minimum, g.maximum = {-0.1, -0.1, -0.1}, {0.1, 0.1, 0.1};
	}
	else
	{
		value: rawptr;
		info: entasis.Shape_Info;
		physics_status: entasis.Status;
		value, info, physics_status = entasis.shape_borrow_raw(world, handle);
		if physics_status != .Ok
		{
			return 0, .Invalid_Data;
		}
		bounds: entasis.Shape_Bounds;
		bounds, physics_status = entasis.shape_bounds(world, handle);
		if physics_status != .Ok
		{
			return 0, .Invalid_Data;
		}
		g.minimum, g.maximum = copy_vector(bounds.min), copy_vector(bounds.max);
		switch info.type_id
		{
		case entasis.SHAPE_TYPE_BOX:
			shape: ^entasis.Box = (^entasis.Box)(value);
			g.kind, g.size = .Box, {2*shape.half_width, 2*shape.half_height, 2*shape.half_length};
		case entasis.SHAPE_TYPE_SPHERE:
			g.kind, g.size = .Sphere, {(^entasis.Sphere)(value).radius, 0, 0};
		case entasis.SHAPE_TYPE_CAPSULE:
			shape: ^entasis.Capsule = (^entasis.Capsule)(value);
			g.kind, g.size = .Capsule, {shape.radius, 2*shape.half_length, 0};
		case entasis.SHAPE_TYPE_CYLINDER:
			shape: ^entasis.Cylinder = (^entasis.Cylinder)(value);
			g.kind, g.size = .Cylinder, {shape.radius, 2*shape.half_length, 0};
		case entasis.SHAPE_TYPE_TRIANGLE:
			shape: ^entasis.Triangle = (^entasis.Triangle)(value);
			g.kind = .Triangles;
			status = observation_geometry_allocate(o, &g, vertices=3, indices=3);
			if status != .Ok
			{
				return 0, status;
			}
			g.vertices[0], g.vertices[1], g.vertices[2] = copy_vector(shape.a), copy_vector(shape.b), copy_vector(shape.c);
			g.indices[0], g.indices[1], g.indices[2] = 0, 1, 2;
		case entasis.SHAPE_TYPE_CONVEX_HULL:
			hull: ^entasis.Convex_Hull = (^entasis.Convex_Hull)(value);
			g.kind = .Triangles;
			triangle_count: int = int(hull.face_vertex_indices.length)-2*int(hull.face_start_indices.length);
			status = observation_geometry_allocate(o, &g, vertices=int(hull.point_count), indices=triangle_count*3);
			if status != .Ok
			{
				return 0, status;
			}
			for &point, index in g.vertices
			{
				bundle := hull.points.memory[index/8];
				point = {simd.extract(bundle.x, index%8), simd.extract(bundle.y, index%8), simd.extract(bundle.z, index%8)};
			}
			index: int;
			for face in 0 ..< int(hull.face_start_indices.length)
			{
				start: int = int(hull.face_start_indices.memory[face]);
				end: int = int(hull.face_vertex_indices.length);
				if face+1 < int(hull.face_start_indices.length)
				{
					end = int(hull.face_start_indices.memory[face+1]);
				}
				for vertex in start+1 ..< end-1
				{
					g.indices[index] = u32(hull.face_vertex_indices.memory[start]);
					g.indices[index+1] = u32(hull.face_vertex_indices.memory[vertex]);
					g.indices[index+2] = u32(hull.face_vertex_indices.memory[vertex+1]);
					index += 3;
				}
			}
		case entasis.SHAPE_TYPE_MESH:
			mesh: ^entasis.Mesh = (^entasis.Mesh)(value);
			g.kind = .Triangles;
			status = observation_geometry_allocate(o, &g, vertices=int(mesh.triangles.length)*3, indices=int(mesh.triangles.length)*3);
			if status != .Ok
			{
				return 0, status;
			}
			for triangle in 0 ..< int(mesh.triangles.length)
			{
				t: entasis.Triangle = mesh.triangles.memory[triangle];
				g.vertices[triangle*3] = copy_vector(t.a)*copy_vector(mesh.scale);
				g.vertices[triangle*3+1] = copy_vector(t.b)*copy_vector(mesh.scale);
				g.vertices[triangle*3+2] = copy_vector(t.c)*copy_vector(mesh.scale);
			}
			for &index, i in g.indices
			{
				index = u32(i);
			}
		case entasis.SHAPE_TYPE_COMPOUND, entasis.SHAPE_TYPE_BIG_COMPOUND:
			children := (^entasis.Compound)(value).children;
			if info.type_id == entasis.SHAPE_TYPE_BIG_COMPOUND
			{
				children = (^entasis.Big_Compound)(value).children;
			}
			g.kind = .Compound;
			status = observation_geometry_allocate(o, &g, children=int(children.length));
			if status != .Ok
			{
				return 0, status;
			}
			for &child, index in g.children
			{
				source := children.memory[index];
				child.geometry, status = geometry_copy(o, world, source.shape_index);
				if status != .Ok
				{
					return 0, status;
				}
				child.part = u32(index);
				child.pose = {copy_vector(source.local_position), quaternion(x=source.local_orientation.x, y=source.local_orientation.y, z=source.local_orientation.z, w=source.local_orientation.w)};
			}
		case:
			return 0, .Unsupported;
		}
	}
	index: u32 = u32(len(o.packet.geometries));
	if len(o.packet.geometries) == cap(o.packet.geometries)
	{
		status = observation_reserve(o, geometries=max(8, cap(o.packet.geometries)*2));
		if status != .Ok
		{
			return 0, status;
		}
	}
	append(&o.packet.geometries, g);
	append(&o.shapes, Shape_Binding{handle, index});
	status = .Ok;
	return index, status;
}

observation_add :: proc(o: ^Observation, world: ^entasis.World, kind: scene.Object_Kind, handle: i32, group: u32 = 0) -> (u32, scene.Status)
{
	shape: entasis.Shape_Handle;
	status: entasis.Status;
	switch kind
	{
	case .Body:
		state: entasis.Body_State;
		state, status = entasis.body_get(world, {handle});
		shape = state.collidable.shape;
	case .Static:
		state: entasis.Static_State;
		state, status = entasis.static_get(world, {handle});
		shape = state.shape;
	case .Shape:
		return 0, .Unsupported;
	}
	if status != .Ok
	{
		return 0, .Invalid_Data;
	}
	geometry: u32;
	geometry_status: scene.Status;
	geometry, geometry_status = geometry_copy(o, world, shape);
	if geometry_status != .Ok
	{
		return 0, geometry_status;
	}
	return observation_define(o, {kind, handle, geometry, group});
}

observation_define :: proc(o: ^Observation, entity: scene.Entity) -> (u32, scene.Status)
{
	capacity: int = max(len(o.packet.entities)+1, max(16, cap(o.packet.entities)*2));
	if len(o.packet.entities)==cap(o.packet.entities)
	{
		status: scene.Status = observation_reserve(o, entities=capacity, instances=capacity, metrics=4);
		if status != .Ok
		{
			return 0, status;
		}
	}
	id: u32 = u32(len(o.packet.entities));
	parts: int = 0 if entity.kind == .Shape else max(1, len(o.packet.geometries[entity.geometry].children));
	needed: int = o.part_capacity+parts+o.overlay_capacity;
	if needed > cap(o.packet.frame.overlays)
	{
		status: scene.Status = observation_reserve(o, overlays=max(needed, max(16, cap(o.packet.frame.overlays)*2)));
		if status != .Ok
		{
			return 0, status;
		}
	}
	o.part_capacity += parts;
	append(&o.packet.entities, entity);
	append(&o.lifetimes, scene.Availability.Available);
	return id, .Ok;
}

observation_add_shape :: proc(o: ^Observation, world: ^entasis.World, shape: entasis.Shape_Handle, pose: scene.Pose = scene.IDENTITY, state: scene.State = {}) -> scene.Status
{
	geometry: u32;
	status: scene.Status;
	geometry, status = geometry_copy(o, world, shape);
	if status != .Ok
	{
		return status;
	}
	if len(o.previews) == cap(o.previews)
	{
		status = observation_reserve(o, previews=max(4, cap(o.previews)*2));
		if status != .Ok
		{
			return status;
		}
	}
	id: u32;
	id, status = observation_define(o, {.Shape, i32(shape.packed), geometry, 0});
	if status != .Ok
	{
		return status;
	}
	append(&o.previews, Shape_Preview{id, pose, state});
	return .Ok;
}

observation_remove :: proc(o: ^Observation, kind: scene.Object_Kind, handle: i32) -> scene.Status
{
	for entity, index in o.packet.entities
	{
		if entity.kind==kind && entity.source_handle==handle && o.lifetimes[index]==.Available
		{
			o.lifetimes[index]=.Unavailable;
			return .Ok;
		}
	}
	return .Invalid_Data;
}

// clear invalidates engine handles, not the recording's monotonic definition IDs
observation_clear :: proc(o: ^Observation)
{
	for &lifetime in o.lifetimes
	{
		lifetime=.Unavailable;
	}
	clear(&o.shapes);
	clear(&o.previews);
	o.part_capacity = 0;
	scene.frame_clear(&o.packet.frame);
}

observation_create :: proc(o: ^Observation, world: ^entasis.World, handles: []entasis.Body_Handle) -> scene.Status
{
	view: entasis.Static_View;
	status: entasis.Status;
	view, status = entasis.static_view(world);
	if status != .Ok
	{
		return .Invalid_Data;
	}
	count: int = len(handles)+view.count;
	visual_status: scene.Status = observation_reserve(o, entities=len(o.packet.entities)+count, instances=count, overlays=count+o.overlay_capacity, metrics=4);
	if visual_status != .Ok
	{
		return visual_status;
	}
	for handle in handles
	{
		_, visual_status = observation_add(o, world, .Body, handle.value);
		if visual_status!=.Ok
		{
			return visual_status;
		}
	}
	for index in 0 ..< view.count
	{
		_, visual_status = observation_add(o, world, .Static, view.handles[index].value, u32(index));
		if visual_status!=.Ok
		{
			return visual_status;
		}
	}
	return .Ok;
}

observation_read :: proc(o: ^Observation, world: ^entasis.World) -> scene.Status
{
	f: ^scene.Frame = &o.packet.frame;
	scene.frame_clear(f);
	for entity, id in o.packet.entities
	{
		if o.lifetimes[id]==.Unavailable
		{
			continue;
		}
		if entity.kind == .Shape
		{
			found: scene.Availability;
			for preview in o.previews
			{
				if preview.id == u32(id)
				{
					append(&f.ids, u32(id));
					append(&f.poses, preview.pose);
					append(&f.states, preview.state);
					found = .Available;
					break;
				}
			}
			if found == .Unavailable
			{
				return .Invalid_Data;
			}
			continue;
		}
		pose: entasis.Rigid_Pose;
		state: scene.State;
		status: entasis.Status;
		switch entity.kind
		{
		case .Body:
			body: entasis.Body_State;
			body, status = entasis.body_get(world, {entity.source_handle});
			if status!=.Ok
			{
				return .Invalid_Data;
			}
			pose=body.pose;
			activation: entasis.Body_Activation_State;
			activation, status = entasis.body_activation_state(world, {entity.source_handle});
			if activation==.Sleeping
			{
				state+={.Sleeping};
			}
		case .Static:
			body: entasis.Static_State;
			body, status = entasis.static_get(world, {entity.source_handle});
			pose=body.pose;
			state={.Static};
		case .Shape:
			return .Unsupported;
		}
		if status!=.Ok
		{
			return .Invalid_Data;
		}
		reference: entasis.Collidable_Reference;
		if entity.kind == .Body
		{
			reference, status = entasis.body_collidable_reference(world, {entity.source_handle});
		}
		else
		{
			reference, status = entasis.static_collidable_reference(world, {entity.source_handle});
		}
		if status != .Ok
		{
			return .Invalid_Data;
		}
		_, status = entasis.trigger_get(world, reference);
		if status == .Ok
		{
			state += {.Sensor};
		}
		else if status != .Not_Found
		{
			return .Invalid_Data;
		}
		for part in 0 ..< max(1, len(o.packet.geometries[entity.geometry].children))
		{
			info: entasis.Collider_Part_Info;
			info, status = entasis.collider_part_get(world, reference, i32(part));
			if status == .Ok
			{
				append(&f.overlays, scene.Overlay{kind=.Part_State, entity_a=u32(id), entity_b=scene.NO_ENTITY,
					part_a=info.identity.child_index, part_b=-1, value=1 if info.settings.role == .Trigger else 0});
			}
			else if status != .Not_Found
			{
				return .Invalid_Data;
			}
		}
		append(&f.ids, u32(id));
		append(&f.poses, copy_pose(pose));
		append(&f.states, state);
	}
	stats: entasis.World_Stats;
	status: entasis.Status;
	stats, status = entasis.world_stats(world);
	if status!=.Ok
	{
		return .Invalid_Data;
	}
	append(&f.metrics, scene.Metric{.Active_Bodies, f64(stats.active_bodies)}, scene.Metric{.Sleeping_Bodies, f64(stats.sleeping_bodies)},
		scene.Metric{.Constraints, f64(stats.active_constraints)}, scene.Metric{.Pairs, f64(stats.active_pairs)});
	return .Ok;
}

observation_destroy :: proc(o: ^Observation)
{
	scene.packet_destroy(&o.packet);
	delete(o.shapes);
	delete(o.lifetimes);
	delete(o.previews);
	o^ = {};
}
