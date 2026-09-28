package physics_viewer

import "core:math"
import rl "vendor:raylib"
import scene "../physics_scene"

Camera :: struct
{
	target: rl.Vector3,
	yaw, pitch, distance: f32,
}
Camera_Link :: enum
{
	Independent, Linked,
}

camera_value :: proc(c: Camera) -> rl.Camera3D
{
	return {position=c.target+rl.Vector3{math.cos(c.yaw)*math.cos(c.pitch), math.sin(c.pitch), math.sin(c.yaw)*math.cos(c.pitch)}*c.distance,
		target=c.target, up={0, 1, 0}, fovy=45, projection=.PERSPECTIVE};
}

camera_geometry_bounds :: proc(r: ^Renderer, geometries: []scene.Geometry, id: u32, pose: scene.Pose,
	state: scene.State, entity: u32, filter: View_Filter, minimum, maximum: ^scene.Vec3, part: i32 = -1)
{
	g: ^scene.Geometry = &geometries[id];
	if g.kind == .Compound
	{
		for child, index in g.children
		{
			camera_geometry_bounds(r, geometries, child.geometry, scene.pose_compose(pose, child.pose),
				state, entity, filter, minimum, maximum, i32(index) if part < 0 else part);
		}
		return;
	}
	if .Sensors in filter.hidden && .Sensor in renderer_effective_state(r, entity, part, state)
	{
		return;
	}
	for corner in 0 ..< 8
	{
		point: scene.Vec3;
		for axis in 0 ..< 3
		{
			point[axis] = g.minimum[axis] if corner&(1<<uint(axis)) == 0 else g.maximum[axis];
		}
		point = scene.pose_transform(pose, point);
		for axis in 0 ..< 3
		{
			minimum[axis] = min(minimum[axis], point[axis]);
			maximum[axis] = max(maximum[axis], point[axis]);
		}
	}
}

camera_visible_bounds :: proc(p: ^scene.Scene_Packet, r: ^Renderer, filter: View_Filter) -> (scene.Vec3, scene.Vec3)
{
	renderer_part_roles(r, p);
	minimum: scene.Vec3={math.F32_MAX, math.F32_MAX, math.F32_MAX};
	maximum: scene.Vec3=-minimum;
	subject: scene.Availability;
	for id, index in p.frame.ids
	{
		if entity_visible(p, id, index, filter) == .Available && p.entities[id].kind != .Static
		{
			camera_geometry_bounds(r, p.geometries[:], p.entities[id].geometry, p.frame.poses[index],
				p.frame.states[index], id, filter, &minimum, &maximum);
		}
	}
	if minimum.x != math.F32_MAX
	{
		subject = .Available;
	}
	for id, index in p.frame.ids
	{
		if p.entities[id].kind != .Static || entity_visible(p, id, index, filter) == .Unavailable
		{
			continue;
		}
		entity_minimum: scene.Vec3 = {math.F32_MAX, math.F32_MAX, math.F32_MAX};
		entity_maximum: scene.Vec3 = -entity_minimum;
		camera_geometry_bounds(r, p.geometries[:], p.entities[id].geometry, p.frame.poses[index],
			p.frame.states[index], id, filter, &entity_minimum, &entity_maximum);
		if entity_minimum.x == math.F32_MAX
		{
			continue;
		}
		extent: scene.Vec3 = entity_maximum-entity_minimum;
		if subject == .Available && .Sensor not_in p.frame.states[index] &&
			extent.x > 4*max(0.01, extent.y) && extent.z > 4*max(0.01, extent.y)
		{
			continue;
		}
		for axis in 0 ..< 3
		{
			minimum[axis] = min(minimum[axis], entity_minimum[axis]);
			maximum[axis] = max(maximum[axis], entity_maximum[axis]);
		}
	}
	query: i32 = -1;
	for overlay in p.frame.overlays
	{
		if overlay.kind == .Query_Path
		{
			query += 1;
		}
		if overlay_visible(p, overlay, filter, query) == .Unavailable
		{
			continue;
		}
		points: [3]scene.Vec3 = {overlay.a, overlay.b, overlay.c};
		count: int = 2;
		if overlay.kind == .Sphere_Volume
		{
			radius: scene.Vec3 = {overlay.value, overlay.value, overlay.value};
			points[0], points[1] = overlay.a-radius, overlay.a+radius;
		}
		#partial switch overlay.kind
		{
		case .Point, .Enter, .Stay, .Exit, .Joint_Break: count = 1;
		case .Triangle: count = 3;
		}
		for point in points[:count]
		{
			for axis in 0 ..< 3
			{
				minimum[axis] = min(minimum[axis], point[axis]);
				maximum[axis] = max(maximum[axis], point[axis]);
			}
		}
	}
	return minimum, maximum;
}

camera_fit :: proc(p: ^scene.Scene_Packet, r: ^Renderer, pane_count: int, filter: View_Filter = {},
	comparison: ^scene.Scene_Packet = nil, comparison_renderer: ^Renderer = nil, source_bounds: rl.Rectangle = {}) -> Camera
{
	minimum, maximum: scene.Vec3 = camera_visible_bounds(p, r, filter);
	if comparison != nil
	{
		other_minimum, other_maximum: scene.Vec3 = camera_visible_bounds(comparison, comparison_renderer, filter);
		for axis in 0 ..< 3
		{
			minimum[axis] = min(minimum[axis], other_minimum[axis]);
			maximum[axis] = max(maximum[axis], other_maximum[axis]);
		}
	}
	if minimum.x == math.F32_MAX
	{
		return {yaw=0.7, pitch=0.4, distance=10};
	}
	aspect: f32 = source_bounds.width/source_bounds.height if source_bounds.height > 0 else f32(rl.GetScreenWidth())/f32(max(1, pane_count)*int(rl.GetScreenHeight()));
	half_fov: f32 = math.atan(math.tan(f32(math.PI/8))*min(1, aspect));
	distance: f32 = rl.Vector3Length(maximum-minimum)*0.5/math.sin(half_fov)*1.25;
	yaw: f32 = 0.7;
	query: i32 = -1;
	for overlay in p.frame.overlays
	{
		if overlay.kind == .Query_Path
		{
			query += 1;
			if overlay_visible(p, overlay, filter, query) == .Unavailable
			{
				continue;
			}
			direction: scene.Vec3 = overlay.a-overlay.b;
			if direction.x != 0 || direction.z != 0
			{
				yaw = math.atan2(direction.z, direction.x)-0.7;
				break;
			}
		}
	}
	return {target=(minimum+maximum)/2, yaw=yaw, pitch=0.45, distance=max(1, distance)};
}

viewer_camera_fit :: proc(v: ^Viewer)
{
	bounds: rl.Rectangle = v.ui.images[v.active_pane];
	if v.count == 2 && v.camera_link == .Linked
	{
		camera: Camera = camera_fit(packet(v, 0), &v.panes[0].renderer, 2, v.ui.filter,
			packet(v, 1), &v.panes[1].renderer, bounds);
		v.panes[0].camera, v.panes[1].camera = camera, camera;
	}
	else if v.count > 0
	{
		v.panes[v.active_pane].camera = camera_fit(packet(v, v.active_pane), &v.panes[v.active_pane].renderer,
			v.count, v.ui.filter, source_bounds=bounds);
	}
}

camera_geometry_hit :: proc(r: ^Renderer, geometries: []scene.Geometry, id: u32, pose: scene.Pose,
	state: scene.State, entity: u32, filter: View_Filter, ray: rl.Ray, nearest: ^f32, part: i32 = -1)
{
	g: ^scene.Geometry = &geometries[id];
	if g.kind == .Compound
	{
		for child, index in g.children
		{
			camera_geometry_hit(r, geometries, child.geometry, scene.pose_compose(pose, child.pose),
				state, entity, filter, ray, nearest, i32(index) if part < 0 else part);
		}
		return;
	}
	if .Sensors in filter.hidden && .Sensor in renderer_effective_state(r, entity, part, state)
	{
		return;
	}
	hit: rl.RayCollision = rl.GetRayCollisionMesh(ray, r.meshes[id], renderer_transform(r, g, pose));
	if hit.hit && hit.distance < nearest^
	{
		nearest^ = hit.distance;
	}
}

camera_orbit_pivot :: proc(v: ^Viewer, side: int, mouse: rl.Vector2) -> rl.Vector3
{
	p: ^scene.Scene_Packet = packet(v, side);
	r: ^Renderer = &v.panes[side].renderer;
	bounds: rl.Rectangle = v.ui.images[side];
	ray: rl.Ray = rl.GetScreenToWorldRayEx(mouse-rl.Vector2{bounds.x, bounds.y},
		camera_value(v.panes[side].camera), i32(bounds.width), i32(bounds.height));
	nearest: f32 = math.F32_MAX;
	renderer_part_roles(r, p);
	for id, index in p.frame.ids
	{
		if entity_visible(p, id, index, v.ui.filter) == .Available &&
			(r.static_surface == .Filled || .Static not_in p.frame.states[index])
		{
			camera_geometry_hit(r, p.geometries[:], p.entities[id].geometry, p.frame.poses[index],
				p.frame.states[index], id, v.ui.filter, ray, &nearest);
		}
	}
	if nearest != math.F32_MAX
	{
		return ray.position+ray.direction*nearest;
	}
	fitted: Camera = camera_fit(p, r, v.count, v.ui.filter, source_bounds=bounds);
	return fitted.target;
}

camera_rotate :: proc(c: ^Camera, delta: rl.Vector2, pivot: rl.Vector3)
{
	if delta == {}
	{
		return;
	}
	right: rl.Vector3 = {math.sin(c.yaw), 0, -math.cos(c.yaw)};
	up: rl.Vector3 = {-math.cos(c.yaw)*math.sin(c.pitch), math.cos(c.pitch), -math.sin(c.yaw)*math.sin(c.pitch)};
	back: rl.Vector3 = {math.cos(c.yaw)*math.cos(c.pitch), math.sin(c.pitch), math.sin(c.yaw)*math.cos(c.pitch)};
	offset: rl.Vector3 = c.target-pivot;
	local: rl.Vector3 = {rl.Vector3DotProduct(offset, right), rl.Vector3DotProduct(offset, up), rl.Vector3DotProduct(offset, back)};
	c.yaw += delta.x*0.006;
	c.pitch = clamp(c.pitch+delta.y*0.006, -1.45, 1.45);
	right = {math.sin(c.yaw), 0, -math.cos(c.yaw)};
	up = {-math.cos(c.yaw)*math.sin(c.pitch), math.cos(c.pitch), -math.sin(c.yaw)*math.sin(c.pitch)};
	back = {math.cos(c.yaw)*math.cos(c.pitch), math.sin(c.pitch), math.sin(c.yaw)*math.cos(c.pitch)};
	c.target = pivot+right*local.x+up*local.y+back*local.z;
}

viewer_camera_update :: proc(v: ^Viewer)
{
	if !rl.IsWindowFocused() || v.ui.input != .Scene
	{
		v.ui.drag = .None;
		return;
	}
	if v.ui.drag != .None && !rl.IsMouseButtonDown(v.ui.drag_button)
	{
		v.ui.drag = .None;
	}
	side: int = v.ui.drag_pane if v.ui.drag != .None else v.ui.hovered_pane;
	if side < 0 || side >= min(v.count, v.ui.image_count)
	{
		return;
	}
	wheel: f32 = rl.GetMouseWheelMove() if v.ui.hovered_pane == side && v.ui.drag == .None else 0;
	if v.ui.drag == .None || !rl.IsMouseButtonPressed(v.ui.drag_button)
	{
		camera_update(&v.panes[side].camera, v.ui.images[side], v.ui.drag, wheel, v.ui.orbit_pivot);
	}
	if v.mode != .Live || v.live.recipe.input != .Live || v.cursor.playback != .Running
	{
		camera_move(&v.panes[side].camera);
	}
	if v.count == 2 && v.camera_link == .Linked
	{
		v.panes[1-side].camera = v.panes[side].camera;
	}
}

camera_update :: proc(c: ^Camera, bounds: rl.Rectangle, drag: Scene_Drag, wheel: f32, orbit_pivot: rl.Vector3)
{
	delta: rl.Vector2 = rl.GetMouseDelta();
	switch drag
	{
	case .Look:
		camera_rotate(c, delta, camera_value(c^).position);
	case .Orbit:
		camera_rotate(c, delta, orbit_pivot);
	case .Pan:
		right: rl.Vector3 = {math.sin(c.yaw), 0, -math.cos(c.yaw)};
		up: rl.Vector3 = {-math.cos(c.yaw)*math.sin(c.pitch), math.cos(c.pitch), -math.sin(c.yaw)*math.sin(c.pitch)};
		scale: f32 = 2*c.distance*math.tan(f32(math.PI/8))/bounds.height;
		c.target += (up*delta.y-right*delta.x)*scale;
	case .None:
	}
	c.distance = clamp(c.distance*math.pow(0.9, wheel), 0.05, 1e7);
}


camera_move :: proc(c: ^Camera)
{
	forward: rl.Vector3 = {-math.cos(c.yaw)*math.cos(c.pitch), -math.sin(c.pitch), -math.sin(c.yaw)*math.cos(c.pitch)};
	right: rl.Vector3 = {math.sin(c.yaw), 0, -math.cos(c.yaw)};
	movement: rl.Vector3;
	if rl.IsKeyDown(.W)
	{
		movement += forward;
	}
	if rl.IsKeyDown(.S)
	{
		movement -= forward;
	}
	if rl.IsKeyDown(.D)
	{
		movement += right;
	}
	if rl.IsKeyDown(.A)
	{
		movement -= right;
	}
	if rl.IsKeyDown(.E)
	{
		movement.y += 1;
	}
	if rl.IsKeyDown(.Q)
	{
		movement.y -= 1;
	}
	speed: f32 = max(1, c.distance*0.5);
	if rl.IsKeyDown(.LEFT_SHIFT) || rl.IsKeyDown(.RIGHT_SHIFT)
	{
		speed *= 3;
	}
	length: f32 = rl.Vector3Length(movement);
	if length > 0
	{
		c.target += movement/length*speed*min(rl.GetFrameTime(), 0.05);
	}
}
