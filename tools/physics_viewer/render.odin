package physics_viewer

import "core:math"
import "base:runtime"
import rl "vendor:raylib"
import gl "vendor:raylib/rlgl"
import scene "../physics_scene"

STYLE_COUNT :: 6;
Surface :: enum i32
{
	Filled, Outline, Edges,
}
Part_Role :: enum u8
{
	Inherited, Solid, Sensor,
}
Batch :: struct
{
	buffer: u32,
	transforms: [][16]f32,
	count: int,
}
Renderer :: struct
{
	batches: []Batch,
	meshes: []rl.Mesh,
	shader: rl.Shader,
	instance_location, mvp_location, color_location, scale_location: i32,
	surface_location, box_size_location, line_width_location: i32,
	static_surface: Surface,
	geometry_count, entity_count: int,
	bytes, required_bytes: u64,
	gpu_bytes: u64,
	part_offsets: []int,
	part_roles: []Part_Role,
	marker_scale: f32,
}
VERTEX_SHADER :: `#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in mat4 instanceTransform;
uniform mat4 mvp;
uniform float markerScale;
uniform int surface;
out vec3 normal;
out vec3 localPosition;
out vec3 localNormal;
void main()
{
    normal = mat3(instanceTransform) * vertexNormal;
    localPosition = vertexPosition;
    localNormal = vertexNormal;
    gl_Position = mvp * instanceTransform * vec4(vertexPosition * markerScale, 1.0);
    if (surface == 2) gl_Position.z -= 0.000001 * gl_Position.w;
}`;
FRAGMENT_SHADER :: `#version 330
in vec3 normal;
in vec3 localPosition;
in vec3 localNormal;
uniform vec4 color;
uniform int surface;
uniform vec3 boxHalfSize;
uniform float lineWidth;
out vec4 finalColor;
void main()
{
    vec3 faceNormal = gl_FrontFacing ? normal : -normal;
    float light = 0.4 + 0.6 * max(dot(normalize(faceNormal), normalize(vec3(-0.5, 1.0, 0.7))), 0.0);
    vec3 fill = color.rgb * light;
    float edge = 0.0;
    if (boxHalfSize.x > 0.0)
    {
        vec3 distanceToFace = boxHalfSize - abs(localPosition);
        vec3 width = fwidth(localPosition) * lineWidth;
        vec2 distanceToEdge, edgeWidth;
        if (abs(localNormal.x) > 0.5)
        {
            distanceToEdge = distanceToFace.yz;
            edgeWidth = width.yz;
        }
        else if (abs(localNormal.y) > 0.5)
        {
            distanceToEdge = distanceToFace.xz;
            edgeWidth = width.xz;
        }
        else
        {
            distanceToEdge = distanceToFace.xy;
            edgeWidth = width.xy;
        }
        vec2 interior = smoothstep(vec2(0.0), edgeWidth, distanceToEdge);
        edge = 1.0 - min(interior.x, interior.y);
    }
    if (surface == 1)
    {
        if (edge < 0.01) discard;
        finalColor = vec4(color.rgb, color.a * edge);
    }
    else if (surface == 2)
    {
        finalColor = color;
    }
    else
    {
        finalColor = vec4(mix(fill, vec3(0.10, 0.09, 0.07), edge * 0.85), color.a);
    }
}`;

renderer_destroy :: proc(r: ^Renderer)
{
	for &batch in r.batches
	{
		if batch.buffer != 0
		{
			gl.UnloadVertexBuffer(batch.buffer);
		}
		delete(batch.transforms);
	}
	delete(r.batches);
	for mesh in r.meshes
	{
		if mesh.vaoId != 0 || mesh.vertices != nil
		{
			rl.UnloadMesh(mesh);
		}
	}
	delete(r.meshes);
	delete(r.part_offsets);
	delete(r.part_roles);
	if r.shader.id != 0
	{
		rl.UnloadShader(r.shader);
	}
	r^ = {};
}

mesh_triangles :: proc(vertices: []scene.Vec3, indices: []u32) -> (rl.Mesh, scene.Status)
{
	mesh: rl.Mesh = {vertexCount=i32(len(indices)), triangleCount=i32(len(indices)/3)};
	mesh.vertices = ([^]f32)(rl.MemAlloc(u32(len(indices)*12)));
	mesh.normals = ([^]f32)(rl.MemAlloc(u32(len(indices)*12)));
	if mesh.vertices == nil || mesh.normals == nil
	{
		rl.MemFree(mesh.vertices);
		rl.MemFree(mesh.normals);
		return {}, .Out_Of_Memory;
	}
	for triangle in 0 ..< len(indices)/3
	{
		a: scene.Vec3 = vertices[indices[triangle*3]];
		b: scene.Vec3 = vertices[indices[triangle*3+1]];
		c: scene.Vec3 = vertices[indices[triangle*3+2]];
		n: rl.Vector3 = rl.Vector3Normalize(rl.Vector3CrossProduct(b-a, c-a));
		for corner in 0 ..< 3
		{
			v: scene.Vec3 = vertices[indices[triangle*3+corner]];
			for axis in 0 ..< 3
			{
				mesh.vertices[(triangle*3+corner)*3+axis] = v[axis];
				mesh.normals[(triangle*3+corner)*3+axis] = n[axis];
			}
		}
	}
	rl.UploadMesh(&mesh, false);
	return mesh, .Ok;
}

mesh_capsule :: proc(radius, length: f32) -> (rl.Mesh, scene.Status)
{
	vertices: [18*25]scene.Vec3;
	indices: [17*24*6]u32;
	for ring in 0 ..< 18
	{
		latitude: f32 = -math.PI/2 + f32(min(ring, 8)+max(0, ring-9))*(math.PI/16);
		for segment in 0 ..< 25
		{
			angle: f32 = f32(segment)*math.PI/12;
			vertices[ring*25+segment] = {radius*math.cos(latitude)*math.cos(angle), radius*math.sin(latitude) + (-length/2 if ring <= 8 else length/2), radius*math.cos(latitude)*math.sin(angle)};
		}
	}
	for ring in 0 ..< 17
	{
		for segment in 0 ..< 24
		{
			a: u32 = u32(ring*25+segment);
			i: int = (ring*24+segment)*6;
			copy(indices[i:i+6], []u32{a, a+25, a+1, a+1, a+25, a+26});
		}
	}
	return mesh_triangles(vertices[:], indices[:]);
}

renderer_mesh_bound :: proc(g: ^scene.Geometry) -> u64
{
	switch g.kind
	{
	case .Box: return 36*32;
	case .Sphere: return 6*18*25*32;
	case .Cylinder: return 24*12*32;
	case .Capsule: return 17*24*6*24;
	case .Marker: return 6*8*9*32;
	case .Triangles: return u64(len(g.indices))*24;
	case .Compound: return 0;
	}
	unreachable();
}

renderer_create :: proc(r: ^Renderer, p: ^scene.Scene_Packet, budget: u64, static_surface: Surface = .Filled) -> scene.Status
{
	r.static_surface = static_surface;
	// the first style's count holds geometry demand until transforms are admitted
	r.required_bytes = u64(len(p.geometries))*STYLE_COUNT*size_of(Batch);
	if r.required_bytes > budget
	{
		return .Budget_Exceeded;
	}
	allocation_error: runtime.Allocator_Error;
	r.batches, allocation_error = make([]Batch, len(p.geometries)*STYLE_COUNT);
	if allocation_error != nil
	{
		return .Out_Of_Memory;
	}
	for e in p.entities
	{
		r.batches[int(e.geometry)*STYLE_COUNT].count += 1;
	}
	#reverse for g, index in p.geometries
	{
		if r.batches[index*STYLE_COUNT].count > int(max(i32))/64
		{
			return .Unsupported;
		}
		if g.kind==.Compound
		{
			for child in g.children
			{
				r.batches[int(child.geometry)*STYLE_COUNT].count += r.batches[index*STYLE_COUNT].count;
			}
			r.batches[index*STYLE_COUNT].count = 0;
		}
	}
	r.bytes = u64(len(p.geometries))*(STYLE_COUNT*size_of(Batch)+size_of(rl.Mesh))+32*4;
	part_count: int;
	for entity in p.entities
	{
		part_count += max(1, len(p.geometries[entity.geometry].children));
	}
	r.bytes += u64(part_count)*size_of(Part_Role)+u64(len(p.entities)+1)*size_of(int);
	for _, index in p.geometries
	{
		count: int = r.batches[index*STYLE_COUNT].count;
		if count==0
		{
			continue;
		}
		mesh_bytes: u64 = renderer_mesh_bound(&p.geometries[index]);
		if mesh_bytes > u64(max(i32))
		{
			return .Unsupported;
		}
		r.bytes += u64(count)*64*STYLE_COUNT + mesh_bytes;
		r.gpu_bytes += u64(count)*64*STYLE_COUNT + mesh_bytes;
	}
	r.required_bytes = r.bytes;
	if r.required_bytes > budget
	{
		return .Budget_Exceeded;
	}
	r.part_offsets, allocation_error = make([]int, len(p.entities)+1);
	if allocation_error != nil
	{
		return .Out_Of_Memory;
	}
	r.part_roles, allocation_error = make([]Part_Role, part_count);
	if allocation_error != nil
	{
		return .Out_Of_Memory;
	}
	for entity, index in p.entities
	{
		r.part_offsets[index+1] = r.part_offsets[index]+max(1, len(p.geometries[entity.geometry].children));
	}
	r.meshes = make([]rl.Mesh, len(p.geometries));
	if len(p.geometries) > 0 && r.meshes == nil
	{
		return .Out_Of_Memory;
	}
	r.shader = rl.LoadShaderFromMemory(VERTEX_SHADER, FRAGMENT_SHADER);
	if !rl.IsShaderValid(r.shader)
	{
		return .Unsupported;
	}
	r.instance_location = rl.GetShaderLocationAttrib(r.shader, "instanceTransform");
	r.mvp_location = rl.GetShaderLocation(r.shader, "mvp");
	r.scale_location = rl.GetShaderLocation(r.shader, "markerScale");
	r.color_location = rl.GetShaderLocation(r.shader, "color");
	r.surface_location = rl.GetShaderLocation(r.shader, "surface");
	r.box_size_location = rl.GetShaderLocation(r.shader, "boxHalfSize");
	r.line_width_location = rl.GetShaderLocation(r.shader, "lineWidth");
	if r.instance_location < 0 || r.mvp_location < 0 || r.color_location < 0 || r.scale_location < 0 ||
		r.surface_location < 0 || r.box_size_location < 0 || r.line_width_location < 0
	{
		return .Unsupported;
	}
	for &g, index in p.geometries
	{
		capacity: int = r.batches[index*STYLE_COUNT].count;
		if capacity == 0
		{
			continue;
		}
		mesh: ^rl.Mesh = &r.meshes[index];
		mesh_status: scene.Status;
		switch g.kind
		{
		case .Box: mesh^ = rl.GenMeshCube(g.size.x, g.size.y, g.size.z);
		case .Sphere: mesh^ = rl.GenMeshSphere(g.size.x, 16, 24);
		case .Cylinder: mesh^ = rl.GenMeshCylinder(g.size.x, g.size.y, 24);
			for i in 0 ..< int(mesh.vertexCount)
			{
				mesh.vertices[i*3+1] -= g.size.y/2;
			}
			rl.UpdateMeshBuffer(mesh^, 0, mesh.vertices, mesh.vertexCount*12, 0);
		case .Capsule: mesh^, mesh_status = mesh_capsule(g.size.x, g.size.y);
		case .Triangles: mesh^, mesh_status = mesh_triangles(g.vertices, g.indices);
		case .Marker: mesh^ = rl.GenMeshSphere(0.08, 6, 8);
		case .Compound: unreachable();
		}
		if mesh_status != .Ok
		{
			return mesh_status;
		}
		if mesh.vaoId == 0
		{
			return .Unsupported;
		}
		for style in 0 ..< STYLE_COUNT
		{
			batch: ^Batch = &r.batches[index*STYLE_COUNT+style];
			batch.transforms = make([][16]f32, capacity);
			batch.count = 0;
			if batch.transforms == nil
			{
				return .Out_Of_Memory;
			}
			batch.buffer = gl.LoadVertexBuffer(nil, i32(len(batch.transforms)*size_of(rl.Matrix)), true);
			if batch.buffer == 0
			{
				return .Unsupported;
			}
		}
	}
	r.geometry_count, r.entity_count = len(p.geometries), len(p.entities);
	return .Ok;
}

renderer_part_roles :: proc(r: ^Renderer, p: ^scene.Scene_Packet)
{
	for &role in r.part_roles
	{
		role = .Inherited;
	}
	for overlay in p.frame.overlays
	{
		if overlay.kind == .Part_State
		{
			role: Part_Role = .Sensor if overlay.value == 1 else .Solid;
			geometry: ^scene.Geometry = &p.geometries[p.entities[overlay.entity_a].geometry];
			if len(geometry.children) == 0
			{
				r.part_roles[r.part_offsets[overlay.entity_a]] = role;
			}
			for child, index in geometry.children
			{
				if child.part == u32(overlay.part_a)
				{
					r.part_roles[r.part_offsets[overlay.entity_a]+index] = role;
				}
			}
		}
	}
}

renderer_effective_state :: proc(r: ^Renderer, entity: u32, part: i32, state: scene.State) -> scene.State
{
	role: Part_Role = r.part_roles[r.part_offsets[entity]+int(max(0, part))];
	effective: scene.State = state;
	if role == .Sensor
	{
		effective += {.Sensor};
	}
	else if role == .Solid
	{
		effective -= {.Sensor};
	}
	return effective;
}

renderer_transform :: proc(r: ^Renderer, geometry: ^scene.Geometry, pose: scene.Pose, marker_scale: f32 = -1) -> rl.Matrix
{
	transform: rl.Matrix = rl.QuaternionToMatrix(pose.orientation);
	if geometry.kind == .Marker
	{
		for row in 0 ..< 3
		{
			for column in 0 ..< 3
			{
				transform[row, column] *= max(1, r.marker_scale if marker_scale < 0 else marker_scale);
			}
		}
	}
	transform[0, 3], transform[1, 3], transform[2, 3] = pose.position.x, pose.position.y, pose.position.z;
	return transform;
}

renderer_instance :: proc(r: ^Renderer, geometries: []scene.Geometry, id: u32, pose: scene.Pose, state: scene.State,
	entity: u32, filter: View_Filter, part: i32 = -1)
{
	g: ^scene.Geometry = &geometries[id];
	if g.kind == .Compound
	{
		for child, index in g.children
		{
			renderer_instance(r, geometries, child.geometry, scene.pose_compose(pose, child.pose), state,
				entity, filter, i32(index) if part < 0 else part);
		}
		return;
	}
	effective: scene.State = renderer_effective_state(r, entity, part, state);
	if .Sensors in filter.hidden && .Sensor in effective
	{
		return;
	}
	style: int;
	if .Static in effective
	{
		style = 1;
	}
	if .Below_Floor in effective
	{
		style = 2;
	}
	if .Sleeping in effective
	{
		style = 3;
	}
	if .Selected in effective
	{
		style = 5;
	}
	if .Sensor in effective || .Query in effective
	{
		style = 4;
	}
	b: ^Batch = &r.batches[int(id)*STYLE_COUNT+style];
	m: rl.Matrix = renderer_transform(r, g, pose, 1);
	b.transforms[b.count] = rl.MatrixToFloatV(m);
	b.count += 1;
}

renderer_reconcile :: proc(r: ^Renderer, p: ^scene.Scene_Packet, budget: u64) -> scene.Status
{
	if r.geometry_count == len(p.geometries) && r.entity_count == len(p.entities)
	{
		return .Ok;
	}
	r.required_bytes = r.bytes;
	if r.bytes > budget
	{
		return .Budget_Exceeded;
	}
	// topology changes prepare replacement resources before releasing the displayed set
	replacement: Renderer;
	status: scene.Status = renderer_create(&replacement, p, budget-r.bytes, r.static_surface);
	if status != .Ok
	{
		r.required_bytes = r.bytes+replacement.required_bytes;
		renderer_destroy(&replacement);
		return status;
	}
	renderer_destroy(r);
	r^ = replacement;
	return .Ok;
}

renderer_prepare :: proc(r: ^Renderer, p: ^scene.Scene_Packet, filter: View_Filter)
{
	for &batch in r.batches
	{
		batch.count = 0;
	}
	for id, index in p.frame.ids
	{
		if entity_visible(p, id, index, filter) == .Unavailable
		{
			continue;
		}
		state: scene.State = p.frame.states[index];
		renderer_instance(r, p.geometries[:], p.entities[id].geometry, p.frame.poses[index], state, id, filter);
	}
	for &batch in r.batches
	{
		if batch.count > 0
		{
			gl.UpdateVertexBuffer(batch.buffer, raw_data(batch.transforms), i32(batch.count*size_of(rl.Matrix)), 0);
		}
	}
}

renderer_draw_batch :: proc(r: ^Renderer, batch: ^Batch, mesh: rl.Mesh, geometry: ^scene.Geometry,
	surface: Surface, color: ^[4]f32)
{
	box_half_size: scene.Vec3 = geometry.size*0.5 if geometry.kind == .Box else {};
	scale: f32 = max(1, r.marker_scale) if geometry.kind == .Marker else 1;
	shader_surface: Surface = surface;
	if surface == .Edges || (surface == .Outline && geometry.kind != .Box)
	{
		gl.EnableWireMode();
		shader_surface = .Edges;
	}
	defer gl.DisableWireMode();
	gl.SetUniform(r.surface_location, &shader_surface, 4, 1);
	gl.SetUniform(r.color_location, color, 3, 1);
	gl.SetUniform(r.scale_location, &scale, 0, 1);
	gl.SetUniform(r.box_size_location, &box_half_size, 2, 1);
	if geometry.kind == .Triangles || surface == .Outline
	{
		gl.DisableBackfaceCulling();
	}
	defer gl.EnableBackfaceCulling();
	gl.EnableVertexArray(mesh.vaoId);
	gl.EnableVertexBuffer(batch.buffer);
	for column in 0 ..< 4
	{
		attribute: u32 = u32(r.instance_location)+u32(column);
		gl.SetVertexAttribute(attribute, 4, 0x1406, false, size_of(rl.Matrix), i32(column*16));
		gl.EnableVertexAttribute(attribute);
		gl.SetVertexAttributeDivisor(attribute, 1);
	}
	if mesh.indices != nil
	{
		gl.DrawVertexArrayElementsInstanced(0, mesh.triangleCount*3, nil, i32(batch.count));
	}
	else
	{
		gl.DrawVertexArrayInstanced(0, mesh.vertexCount, i32(batch.count));
	}
}

renderer_draw :: proc(r: ^Renderer, p: ^scene.Scene_Packet, camera: rl.Camera3D, filter: View_Filter)
{
	// fitted benchmark rows can extend beyond raylib's fixed far plane
	near_plane: f64 = gl.GetCullDistanceNear();
	far_plane: f64 = gl.GetCullDistanceFar();
	gl.SetClipPlanes(near_plane, max(far_plane, f64(rl.Vector3Distance(camera.position, camera.target))*4));
	defer gl.SetClipPlanes(near_plane, far_plane);
	rl.BeginMode3D(camera);
	gl.DrawRenderBatchActive();
	gl.EnableShader(r.shader.id);
	gl.SetUniformMatrix(r.mvp_location, gl.GetMatrixProjection()*gl.GetMatrixModelview());
	colors: [STYLE_COUNT][4]f32 = {
		{232.0/255, 155.0/255, 67.0/255, 1}, {0.45, 0.48, 0.52, 1}, {0.98, 0.25, 0.24, 1},
		{185.0/255, 121.0/255, 62.0/255, 1}, {0.85, 0.63, 0.25, 0.3}, {0.93, 0.85, 0.31, 1},
	};
	line_width: f32 = gl.GetLineWidth();
	gl.SetUniform(r.line_width_location, &line_width, 0, 1);
	for pass in 0 ..< 3
	{
		if pass == 1
		{
			gl.DisableDepthMask();
		}
		for &batch, index in r.batches
		{
			if batch.count == 0
			{
				continue;
			}
			style: int = index%STYLE_COUNT;
			g: ^scene.Geometry = &p.geometries[index/STYLE_COUNT];
			surface: Surface = r.static_surface if style == 1 else .Filled;
			color: [4]f32 = colors[style];
			if pass == 2
			{
				if g.kind == .Box || g.kind == .Marker || surface == .Outline
				{
					continue;
				}
				surface = .Edges;
				color = {0.10, 0.09, 0.07, color.a};
			}
			else if (style == 4) != (pass == 1)
			{
				continue;
			}
			if surface == .Outline
			{
				color = {0.55, 0.65, 0.75, 1};
			}
			renderer_draw_batch(r, &batch, r.meshes[index/STYLE_COUNT], g, surface, &color);
		}
	}
	gl.EnableDepthMask();
	gl.DisableVertexArray();
	gl.DisableVertexBuffer();
	gl.DisableShader();
	axis_colors: [3]rl.Color = {rl.RED, rl.GREEN, rl.BLUE};
	for id, index in p.frame.ids
	{
		if entity_visible(p, id, index, filter) == .Unavailable
		{
			continue;
		}
		if p.geometries[p.entities[id].geometry].kind == .Marker
		{
			pose: scene.Pose = p.frame.poses[index];
			for axis in 0 ..< 3
			{
				endpoint: scene.Vec3;
				endpoint[axis] = 0.3*max(1, r.marker_scale);
				rl.DrawLine3D(pose.position, scene.pose_transform(pose, endpoint), axis_colors[axis]);
			}
		}
	}
	gl.DrawRenderBatchActive();
	gl.DisableDepthTest();
	query: i32 = -1;
	for o in p.frame.overlays
	{
		if o.kind == .Query_Path
		{
			query += 1;
		}
		if overlay_visible(p, o, filter, query) == .Unavailable
		{
			continue;
		}
		switch o.kind
		{
		case .Point, .Enter, .Stay, .Exit:
			rl.DrawSphere(o.a, max(f32(0.09), rl.Vector3Distance(camera.position, o.a)*0.008), {244, 182, 77, 255});
		case .Joint_Break:
			rl.DrawLine3D(o.a, o.b, rl.RED);
			rl.DrawSphere((o.a+o.b)*0.5, max(f32(0.09), rl.Vector3Distance(camera.position, o.a)*0.008), rl.RED);
		case .Triangle: rl.DrawTriangle3D(o.a, o.b, o.c, {244, 182, 77, 120});
		case .Bounds: rl.DrawBoundingBox({min=o.a, max=o.b}, {244, 182, 77, 255});
		case .Sphere_Volume: rl.DrawSphereWires(o.a, o.value, 8, 12, {244, 182, 77, 255});
		case .Ray_Hit, .Contact:
			rl.DrawSphere(o.a, 0.05, {244, 182, 77, 255});
			renderer_vector(o.a, o.b, {244, 182, 77, 255});
		case .Gravity:
			rl.DrawSphere(o.a, max(f32(0.09), rl.Vector3Distance(camera.position, o.a)*0.008), {95, 183, 232, 255});
			renderer_vector(o.a, o.b, {95, 183, 232, 255});
		case .Angular_Velocity, .Contact_Impulse:
			renderer_vector(o.a, o.b, {244, 182, 77, 255});
		case .Ray_Miss: rl.DrawLine3D(o.a, o.b, rl.RED);
		case .Constraint:
			rl.DrawLine3D(o.a, o.b, {244, 182, 77, 255});
			rl.DrawSphere(o.a, 0.06, {244, 182, 77, 255});
			rl.DrawSphere(o.b, 0.06, {244, 182, 77, 255});
		case .Line, .Query_Path: rl.DrawLine3D(o.a, o.b, {244, 182, 77, 255});
		case .Part_State:
		}
	}
	gl.DrawRenderBatchActive();
	gl.EnableDepthTest();
	rl.EndMode3D();
}

renderer_vector :: proc(a, b: scene.Vec3, color: rl.Color)
{
	rl.DrawLine3D(a, b, color);
	direction: scene.Vec3 = b-a;
	length: f32 = rl.Vector3Length(direction);
	if length > 1e-6
	{
		head: f32 = min(length*0.2, 0.25);
		rl.DrawCylinderEx(b-direction/length*head, b, head*0.3, 0, 8, color);
	}
}
