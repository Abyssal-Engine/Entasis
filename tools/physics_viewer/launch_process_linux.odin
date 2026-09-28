#+build linux
package physics_viewer

import "core:os"
import support "../../benchmarks/benchmark_support"
import linux "core:sys/linux"

launch_available_workers :: proc() -> int
{
	return os.get_processor_core_count();
}

Launch_Process :: struct
{
	child: os.Process,
	pid: u32,
}

launch_process_start :: proc(process: ^Launch_Process, request: ^Launch_Request, root: string, log: ^os.File) -> os.Error
{
	// the launch producer emits fixed command words and one pair per option
	arguments: [7+2*(len(support.OPTION_NAMES)+8)]string;
	count: int = len(request.arguments)+1;
	if count > len(arguments)
	{
		return os.General_Error.Invalid_Command;
	}
	arguments[0] = "setsid";
	copy(arguments[1:], request.arguments[:]);
	error: os.Error;
	process.child, error = os.process_start({command=arguments[:count], working_dir=root, stdout=log, stderr=log, stdin=os.stdin});
	if error == nil
	{
		process.pid = u32(process.child.pid);
	}
	return error;
}

launch_process_poll :: proc(process: ^Launch_Process) -> (int, os.Error)
{
	state: os.Process_State;
	error: os.Error;
	state, error = os.process_wait(process.child, 0);
	if error == nil
	{
		process.child = {};
	}
	return state.exit_code, error;
}

launch_process_cancel :: proc(process: ^Launch_Process) -> os.Error
{
	linux.kill(-linux.Pid(process.pid), .SIGKILL);
	_ = os.process_kill(process.child);
	error: os.Error;
	_, error = os.process_wait(process.child);
	if error == nil
	{
		process.child = {};
	}
	return error;
}

launch_process_close :: proc(process: ^Launch_Process)
{
	if process.pid != 0
	{
		linux.kill(-linux.Pid(process.pid), .SIGKILL);
	}
	if process.child.pid != 0
	{
		_ = os.process_kill(process.child);
		_, _ = os.process_wait(process.child);
	}
	process^ = {};
}
