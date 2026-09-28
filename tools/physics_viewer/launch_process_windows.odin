#+build windows
package physics_viewer

import "base:intrinsics"
import "core:os"
import "core:unicode/utf16"
import win "core:sys/windows"

launch_available_workers :: proc() -> int
{
	process_mask, system_mask: win.DWORD_PTR;
	if win.GetProcessAffinityMask(win.GetCurrentProcess(), &process_mask, &system_mask) == win.FALSE
	{
		return 0;
	}
	return int(intrinsics.count_ones(process_mask));
}

foreign import launch_kernel "system:kernel32.lib"
@(default_calling_convention="system")
foreign launch_kernel
{
	CreateJobObjectW :: proc(attributes: ^win.SECURITY_ATTRIBUTES, name: win.LPCWSTR) -> win.HANDLE ---;
	SetInformationJobObject :: proc(job: win.HANDLE, information_class: i32, information: rawptr, length: u32) -> win.BOOL ---;
	AssignProcessToJobObject :: proc(job, process: win.HANDLE) -> win.BOOL ---;
	TerminateJobObject :: proc(job: win.HANDLE, exit_code: u32) -> win.BOOL ---;
}
Launch_Job_Limits :: struct
{
	process_time, job_time: i64,
	flags: u32,
	minimum_working_set, maximum_working_set: uintptr,
	active_process_limit: u32,
	affinity: uintptr,
	priority_class, scheduling_class: u32,
	io_counters: [6]u64,
	process_memory, job_memory, peak_process_memory, peak_job_memory: uintptr,
}

Launch_Process :: struct
{
	job, handle: win.HANDLE,
	pid: u32,
}

launch_command_line :: proc(arguments: []string, buffer: []u16) -> []u16
{
	count: int;
	for argument, index in arguments
	{
		if count+3 >= len(buffer)
		{
			return nil;
		}
		if index > 0
		{
			buffer[count] = ' ';
			count += 1;
		}
		buffer[count] = '"';
		count += 1;
		slashes: int;
		for c in argument
		{
			if c == '\\'
			{
				slashes += 1;
				continue;
			}
			escaped: int = 2*slashes+1 if c == '"' else slashes;
			if count+escaped+2 >= len(buffer)
			{
				return nil;
			}
			for _ in 0 ..< escaped
			{
				buffer[count] = '\\';
				count += 1;
			}
			slashes = 0;
			if c > 0xffff
			{
				first, second: rune;
				first, second = utf16.encode_surrogate_pair(c);
				buffer[count], buffer[count+1] = u16(first), u16(second);
				count += 2;
			}
			else
			{
				buffer[count] = u16(c);
				count += 1;
			}
		}
		if count+2*slashes+1 >= len(buffer)
		{
			return nil;
		}
		for _ in 0 ..< 2*slashes
		{
			buffer[count] = '\\';
			count += 1;
		}
		buffer[count] = '"';
		count += 1;
	}
	buffer[count] = 0;
	return buffer[:count];
}

launch_process_start :: proc(process: ^Launch_Process, request: ^Launch_Request, root: string, log: ^os.File) -> os.Error
{
	process.job = CreateJobObjectW(nil, nil);
	if process.job == nil
	{
		return os.Platform_Error(win.GetLastError());
	}
	limits: Launch_Job_Limits = {flags=0x2000};
	if SetInformationJobObject(process.job, 9, &limits, size_of(limits)) == win.FALSE
	{
		return os.Platform_Error(win.GetLastError());
	}
	command_buffer, directory_buffer: [32768]u16;
	command: []u16 = launch_command_line(request.arguments[:], command_buffer[:]);
	directory: []u16 = win.utf8_to_utf16(directory_buffer[:], root);
	if len(command) == 0 || len(directory) == 0
	{
		return os.General_Error.Invalid_Command;
	}
	startup: win.STARTUPINFOW = {cb=size_of(win.STARTUPINFOW), dwFlags=win.STARTF_USESTDHANDLES,
		hStdOutput=win.HANDLE(os.fd(log)), hStdError=win.HANDLE(os.fd(log)), hStdInput=win.HANDLE(os.fd(os.stdin))};
	child: win.PROCESS_INFORMATION;
	if win.CreateProcessW(nil, cast(win.LPCWSTR)raw_data(command), nil, nil, true, win.CREATE_SUSPENDED | win.CREATE_NO_WINDOW,
		nil, cast(win.LPCWSTR)raw_data(directory), &startup, &child) == win.FALSE
	{
		return os.Platform_Error(win.GetLastError());
	}
	process.handle, process.pid = child.hProcess, child.dwProcessId;
	defer win.CloseHandle(child.hThread);
	if AssignProcessToJobObject(process.job, child.hProcess) == win.FALSE
	{
		error: os.Error = os.Platform_Error(win.GetLastError());
		win.TerminateProcess(child.hProcess, 1);
		win.WaitForSingleObject(child.hProcess, win.INFINITE);
		return error;
	}
	if win.ResumeThread(child.hThread) == max(u32)
	{
		return os.Platform_Error(win.GetLastError());
	}
	return nil;
}

launch_process_poll :: proc(process: ^Launch_Process) -> (int, os.Error)
{
	switch win.WaitForSingleObject(process.handle, 0)
	{
	case win.WAIT_TIMEOUT: return 0, os.General_Error.Timeout;
	case win.WAIT_OBJECT_0:
		code: u32;
		if win.GetExitCodeProcess(process.handle, &code) != win.FALSE
		{
			return int(code), nil;
		}
	}
	return 1, os.Platform_Error(win.GetLastError());
}

launch_process_cancel :: proc(process: ^Launch_Process) -> os.Error
{
	if TerminateJobObject(process.job, 130) == win.FALSE
	{
		return os.Platform_Error(win.GetLastError());
	}
	if win.WaitForSingleObject(process.handle, win.INFINITE) == win.WAIT_FAILED
	{
		return os.Platform_Error(win.GetLastError());
	}
	return nil;
}

launch_process_close :: proc(process: ^Launch_Process)
{
	if process.job != nil
	{
		win.CloseHandle(process.job);
	}
	if process.handle != nil
	{
		win.WaitForSingleObject(process.handle, win.INFINITE);
		win.CloseHandle(process.handle);
	}
	process^ = {};
}
