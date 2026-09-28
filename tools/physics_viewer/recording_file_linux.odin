package physics_viewer

import "core:os"
import linux "core:sys/linux"
import scene "../physics_scene"

result_recording_file :: proc(path: string, kind: Recording_Path_Kind = .File) -> Recording_File
{
	storage: [4096]u8;
	if len(path) >= len(storage)
	{
		return {state=.Failed, error=os.General_Error.Invalid_Path};
	}
	copy(storage[:], path);
	info: linux.Stat;
	error: linux.Errno = linux.lstat(cast(cstring)raw_data(storage[:]), &info);
	if error != nil
	{
		return {state=.Missing} if error == .ENOENT else Recording_File{state=.Failed, error=os.Platform_Error(error)};
	}
	return {state=.Present, bytes=u64(info.size)} if (info.mode & linux.S_IFMT) == (linux.S_IFREG if kind == .File else linux.S_IFDIR) else Recording_File{state=.Rejected};
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
	storage: File_Path_Buffer;
	if len(path) >= len(storage)
	{
		return {state=.Failed, error=os.General_Error.Invalid_Path};
	}
	copy(storage[:], path);
	error: linux.Errno = linux.unlink(cast(cstring)raw_data(storage[:]));
	if error != nil
	{
		return {state=.Missing} if error == .ENOENT else Recording_File{state=.Failed, error=os.Platform_Error(error)};
	}
	return file;
}

recording_absolute_path :: proc(path: string, buffer: []u8) -> (string, scene.Status)
{
	if len(path) == 0
	{
		return "", .File_Error;
	}
	// retain every component, including link/.., for directory inspection
	if path[0] == '/'
	{
		if len(path) >= len(buffer)
		{
			return "", .File_Error;
		}
		copy(buffer, path);
		buffer[len(path)] = 0;
		return string(buffer[:len(path)]), .Ok;
	}
	storage: File_Path_Buffer;
	count: int;
	error: linux.Errno;
	count, error = linux.getcwd(storage[:]);
	if error != nil || count <= 0
	{
		return "", .File_Error;
	}
	// path may alias buffer, so move it before writing the cwd prefix
	length: int = count+len(path);
	if length >= len(buffer)
	{
		return "", .File_Error;
	}
	copy(buffer[count:], path);
	copy(buffer[:count-1], storage[:count-1]);
	buffer[count-1], buffer[length] = '/', 0;
	return string(buffer[:length]), .Ok;
}
