package physics_viewer

import "core:os"
import "core:math"
import rl "vendor:raylib"
import gl "vendor:raylib/rlgl"
import scene "../physics_scene"

VIEWPORT_RENDER_SCALE :: 2;

viewport_release :: proc(pane: ^Pane)
{
	if pane.texture.id != 0
	{
		rl.UnloadRenderTexture(pane.texture);
		pane.texture = {};
	}
}

save_screenshot :: proc(path: string) -> scene.Status
{
	image: rl.Image = rl.LoadImageFromScreen();
	defer rl.UnloadImage(image);
	length: i32;
	bytes: [^]u8 = ([^]u8)(rl.ExportImageToMemory(image, ".png", &length));
	if bytes == nil
	{
		return .File_Error;
	}
	defer rl.MemFree(bytes);
	file: ^os.File;
	error: os.Error;
	file, error = os.open(path, {.Write, .Create, .Excl});
	if error != nil
	{
		return .File_Error;
	}
	defer os.close(file);
	written: int;
	write_error: os.Error;
	written, write_error = os.write(file, bytes[:int(length)]);
	if write_error != nil || written != int(length)
	{
		return .File_Error;
	}
	return .Ok;
}

viewport_render :: proc(v: ^Viewer) -> scene.Status
{
	for side in 0 ..< min(v.count, v.ui.image_count)
	{
		pane: ^Pane = &v.panes[side];
		bounds: rl.Rectangle = v.ui.images[side];
		pane_width, pane_height: i32 = max(1, i32(bounds.width)), max(1, i32(bounds.height));
		render_width, render_height: i32 = pane_width*VIEWPORT_RENDER_SCALE, pane_height*VIEWPORT_RENDER_SCALE;
		p: ^scene.Scene_Packet = packet(v, side);
		pane.renderer.marker_scale = max(1, 3*pane.camera.distance*math.tan(f32(math.PI/8))/(0.08*f32(pane_height)));
		if pane.texture.id == 0 || pane.texture.texture.width != render_width || pane.texture.texture.height != render_height
		{
			viewport_release(pane);
			pane.cached = .Unavailable;
			pane.texture = rl.LoadRenderTexture(render_width, render_height);
			if pane.texture.id == 0 || !gl.FramebufferComplete(pane.texture.id)
			{
				return .Out_Of_Memory;
			}
			rl.SetTextureFilter(pane.texture.texture, .BILINEAR);
		}
		changed: scene.Availability = .Available if pane.cached == .Unavailable || pane.data_dirty == .Available ||
			pane.cached_filter != v.ui.filter else .Unavailable;
		if changed == .Available
		{
			renderer_part_roles(&pane.renderer, p);
			renderer_prepare(&pane.renderer, p, v.ui.filter);
		}
		if (changed == .Available || pane.cached_camera != pane.camera) &&
			pane.renderer.geometry_count == len(p.geometries) && pane.renderer.entity_count == len(p.entities)
		{
			rl.BeginTextureMode(pane.texture);
			rl.ClearBackground({39, 41, 45, 255});
			line_width: f32 = gl.GetLineWidth();
			gl.SetLineWidth(line_width*VIEWPORT_RENDER_SCALE);
			renderer_draw(&pane.renderer, p, camera_value(pane.camera), v.ui.filter);
			gl.SetLineWidth(line_width);
			rl.EndTextureMode();
			pane.cached, pane.data_dirty = .Available, .Unavailable;
			pane.cached_camera, pane.cached_filter = pane.camera, v.ui.filter;
		}
	}
	return .Ok;
}

viewport_layout :: proc(v: ^Viewer)
{
	v.ui.image_count = v.count;
	if v.count == 0
	{
		return;
	}
	area: rl.Rectangle = v.ui.body;
	if v.count == 2
	{
		area.y += 56*v.ui.dpi;
		area.height = max(1, area.height-84*v.ui.dpi);
	}
	width: f32 = (area.width-8*v.ui.dpi*f32(v.count-1))/f32(v.count);
	for side in 0 ..< v.count
	{
		bounds: rl.Rectangle = {area.x+f32(side)*(width+8*v.ui.dpi), area.y, width, area.height};
		v.ui.images[side] = bounds;
		mouse: rl.Vector2 = rl.GetMousePosition();
		if v.ui.input == .List && rl.CheckCollisionPointRec(mouse, bounds)
		{
			v.ui.hovered_pane = side;
			v.ui.input = .Scene;
			if rl.IsMouseButtonPressed(.LEFT) || rl.IsMouseButtonPressed(.RIGHT) || rl.IsMouseButtonPressed(.MIDDLE)
			{
				v.ui.focused_pane = side;
				if v.active_pane != side
				{
					v.active_pane = side;
					if v.mode == .Replay && v.synchronization == .Independent
					{
						v.cursor.index, v.cursor.count = v.panes[side].reader.selected, len(v.panes[side].reader.index);
						v.cursor.playback, v.cursor.accumulator = .Paused, 0;
					}
				}
				if v.ui.drag == .None
				{
					v.ui.drag_pane = side;
					if rl.IsMouseButtonPressed(.RIGHT)
					{
						v.ui.drag = .Look;
						v.ui.drag_button = .RIGHT;
						if rl.IsKeyDown(.LEFT_SHIFT) || rl.IsKeyDown(.RIGHT_SHIFT)
						{
							v.ui.drag = .Pan;
						}
					}
					else if rl.IsMouseButtonPressed(.LEFT) && (rl.IsKeyDown(.LEFT_ALT) || rl.IsKeyDown(.RIGHT_ALT))
					{
						v.ui.drag = .Orbit;
						v.ui.drag_button = .LEFT;
						v.ui.orbit_pivot = camera_orbit_pivot(v, side, mouse);
					}
					else if rl.IsMouseButtonPressed(.MIDDLE)
					{
						v.ui.drag = .Pan;
						v.ui.drag_button = .MIDDLE;
					}
				}
			}
		}
	}
	if v.ui.drag != .None && v.ui.input == .List
	{
		v.ui.input = .Scene;
	}
}

viewport_composite :: proc(v: ^Viewer)
{
	for side in 0 ..< min(v.count, v.ui.image_count)
	{
		texture: rl.Texture = v.panes[side].texture.texture;
		rl.DrawTexturePro(texture, {0, 0, f32(texture.width), -f32(texture.height)}, v.ui.images[side], {}, 0, rl.WHITE);
	}
}
