package physics_viewer

import "core:os"
import "core:unicode"
import "core:unicode/utf8"
import "core:unicode/utf16"
import scene "../physics_scene"
import win "core:sys/windows"

result_recording_file :: proc(path: string, kind: Recording_Path_Kind = .File) -> Recording_File
{
	storage: [32768]u16;
	wide: []u16 = win.utf8_to_utf16(storage[:], path);
	if len(wide) == 0
	{
		return {state=.Failed, error=os.General_Error.Invalid_Path};
	}
	data: win.WIN32_FILE_ATTRIBUTE_DATA;
	if win.GetFileAttributesExW(cast(win.LPCWSTR)raw_data(wide), win.GetFileExInfoStandard, &data) == win.FALSE
	{
		error: win.DWORD = win.GetLastError();
		return {state=.Missing} if error == win.ERROR_FILE_NOT_FOUND || error == win.ERROR_PATH_NOT_FOUND else Recording_File{state=.Failed, error=os.Platform_Error(error)};
	}
	if data.dwFileAttributes & (win.FILE_ATTRIBUTE_REPARSE_POINT | win.FILE_ATTRIBUTE_DEVICE) != 0 ||
		((data.dwFileAttributes & win.FILE_ATTRIBUTE_DIRECTORY) != 0) != (kind == .Directory)
	{
		return {state=.Rejected};
	}
	return {state=.Present, bytes=(u64(data.nFileSizeHigh)<<32)|u64(data.nFileSizeLow)};
}

recording_remove :: proc(path: string) -> Recording_File
{
	directory: Recording_File = recording_directory_inspect(os.dir(path));
	if directory.state != .Present
	{
		return directory;
	}
	file: Recording_File = result_recording_file(path);
	if file.state != .Present
	{
		return file;
	}
	storage: [32768]u16;
	wide: []u16 = win.utf8_to_utf16(storage[:], path);
	if len(wide) == 0
	{
		return {state=.Failed, error=os.General_Error.Invalid_Path};
	}
	if win.DeleteFileW(cast(win.LPCWSTR)raw_data(wide)) == win.FALSE
	{
		error: win.DWORD = win.GetLastError();
		return {state=.Missing} if error == win.ERROR_FILE_NOT_FOUND else Recording_File{state=.Failed, error=os.Platform_Error(error)};
	}
	return file;
}

recording_absolute_path :: proc(path: string, buffer: []u8) -> (string, scene.Status)
{
	input, output: [32768]u16;
	wide: []u16 = win.utf8_to_utf16(input[:], path);
	if len(wide) == 0
	{
		return "", .File_Error;
	}
	length: u32 = win.GetFullPathNameW(cast(win.LPCWSTR)raw_data(wide), u32(len(output)), cast(cstring16)raw_data(output[:]), nil);
	if length == 0 || length >= u32(len(output))
	{
		return "", .File_Error;
	}
	remaining: string16 = transmute(string16)output[:length];
	count: int;
	for len(remaining) > 0
	{
		r: rune;
		width: int;
		r, width = utf16.decode_rune_in_string(remaining);
		remaining = remaining[width:];
		r = '/' if r == '\\' else unicode.to_lower(r);
		bytes: [4]u8;
		size: int;
		bytes, size = utf8.encode_rune(r);
		if count+size >= len(buffer)
		{
			return "", .File_Error;
		}
		copy(buffer[count:], bytes[:size]);
		count += size;
	}
	buffer[count] = 0;
	return string(buffer[:count]), .Ok;
}

viewer_open_file :: proc(filename: string, leaf: string = "")
{
	path_buffer: File_Path_Buffer;
	path: string = filename;
	status: scene.Status;
	if len(leaf) > 0
	{
		path, status = recording_join_path(path_buffer[:], filename, leaf);
		if status != .Ok
		{
			return;
		}
	}
	storage: [32768]u16;
	wide: []u16 = win.utf8_to_utf16(storage[:], path);
	if len(wide) == 0
	{
		return;
	}
	win.ShellExecuteW(nil, nil, cast(win.LPCWSTR)raw_data(wide), nil, nil, win.SW_SHOWNORMAL);
}
