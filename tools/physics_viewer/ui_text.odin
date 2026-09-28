package physics_viewer

import "core:fmt"
import "core:io"
import rl "vendor:raylib"

Text_Output_Mode :: enum
{
	Measure, Draw, Label,
}
Text_Output :: struct
{
	font: rl.Font,
	position, origin: rl.Vector2,
	size, spacing, width: f32,
	bounds: rl.Rectangle,
	limit: f32,
	color: rl.Color,
	mode: Text_Output_Mode,
}

ui_text_glyph :: proc(out: ^Text_Output, codepoint: rune, glyph: rl.GlyphInfo, scale: f32)
{
	source: rl.Rectangle = rl.GetGlyphAtlasRec(out.font, codepoint);
	padding: f32 = f32(out.font.glyphPadding);
	source = {source.x-padding, source.y-padding, source.width+2*padding, source.height+2*padding};
	destination: rl.Rectangle = {out.position.x+(f32(glyph.offsetX)-padding)*scale,
		out.position.y+(f32(glyph.offsetY)-padding)*scale, source.width*scale, source.height*scale};
	left: f32 = max(destination.x, out.bounds.x);
	top: f32 = max(destination.y, out.bounds.y);
	right: f32 = min(destination.x+destination.width, out.bounds.x+out.bounds.width, out.limit);
	bottom: f32 = min(destination.y+destination.height, out.bounds.y+out.bounds.height);
	if right <= left || bottom <= top
	{
		return;
	}
	// crop the atlas quad without replacing a caller's scissor region
	source = {source.x+(left-destination.x)/scale, source.y+(top-destination.y)/scale,
		(right-left)/scale, (bottom-top)/scale};
	rl.DrawTexturePro(out.font.texture, source, {left, top, right-left, bottom-top}, {}, 0, out.color);
}

ui_text_write :: proc(data: rawptr, mode: io.Stream_Mode, bytes: []u8, offset: i64, whence: io.Seek_From) -> (i64, io.Error)
{
	if mode == .Flush
	{
		return 0, .None;
	}
	if mode != .Write
	{
		return 0, .Unsupported;
	}
	out: ^Text_Output = cast(^Text_Output)data;
	scale: f32 = out.size/f32(out.font.baseSize);
	for codepoint in string(bytes)
	{
		if codepoint == '\n'
		{
			out.position.x = out.origin.x;
			out.position.y += out.size*1.5;
			continue;
		}
		glyph: rl.GlyphInfo = rl.GetGlyphInfo(out.font, codepoint);
		advance: f32 = f32(glyph.advanceX);
		if glyph.advanceX == 0
		{
			advance = rl.GetGlyphAtlasRec(out.font, codepoint).width+f32(glyph.offsetX);
		}
		if codepoint != ' ' && codepoint != '\t'
		{
			switch out.mode
			{
			case .Measure:
			case .Draw:
				rl.DrawTextCodepoint(out.font, codepoint, out.position, out.size, out.color);
			case .Label:
				if out.position.x+advance*scale <= out.limit
				{
					ui_text_glyph(out, codepoint, glyph, scale);
				}
			}
		}
		out.position.x += advance*scale+out.spacing;
		out.width = max(out.width, out.position.x-out.origin.x-out.spacing);
	}
	return i64(len(bytes)), .None;
}

ui_measure_text :: proc(font: rl.Font, text: string, size, spacing: f32) -> f32
{
	out: Text_Output = {font=font, size=size, spacing=spacing};
	ui_text_write(&out, .Write, transmute([]u8)text, 0, .Start);
	return out.width;
}

ui_draw_text :: proc(font: rl.Font, text: string, position: rl.Vector2, size, spacing: f32, color: rl.Color)
{
	out: Text_Output = {font=font, position=position, origin=position, size=size, spacing=spacing, color=color, mode=.Draw};
	ui_text_write(&out, .Write, transmute([]u8)text, 0, .Start);
}

ui_text :: proc(bounds: rl.Rectangle, format: string, args: ..any)
{
	size: f32 = f32(rl.GuiGetStyle(.DEFAULT, i32(rl.GuiDefaultProperty.TEXT_SIZE)));
	spacing: f32 = f32(rl.GuiGetStyle(.DEFAULT, i32(rl.GuiDefaultProperty.TEXT_SPACING)));
	padding: f32 = f32(rl.GuiGetStyle(.LABEL, i32(rl.GuiControlProperty.TEXT_PADDING)));
	position: rl.Vector2 = {bounds.x+padding, bounds.y+(bounds.height-size)/2};
	color: rl.Color = rl.GetColor(u32(rl.GuiGetStyle(.LABEL, i32(rl.GuiControlProperty.TEXT_COLOR_NORMAL)+rl.GuiGetState()*3)));
	out: Text_Output = {font=rl.GuiGetFont(), position=position, origin=position, size=size, spacing=spacing, color=color,
		bounds=bounds, limit=bounds.x+bounds.width-padding};
	fmt.wprintf({procedure=ui_text_write, data=&out}, format, ..args);
	width: f32 = out.width;
	ellipsis_text: string = "...";
	ellipsis: f32 = ui_measure_text(out.font, ellipsis_text, size, spacing);
	out.position, out.width, out.mode = position, 0, .Label;
	if width > bounds.width-2*padding
	{
		out.limit -= ellipsis+spacing;
	}
	fmt.wprintf({procedure=ui_text_write, data=&out}, format, ..args);
	if width > bounds.width-2*padding
	{
		out.limit = bounds.x+bounds.width-padding;
		out.position = {max(position.x, out.limit-ellipsis), position.y};
		ui_text_write(&out, .Write, transmute([]u8)ellipsis_text, 0, .Start);
	}
}
