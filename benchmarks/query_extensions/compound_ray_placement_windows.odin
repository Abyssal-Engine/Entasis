package query_extensions

import "core:fmt"
import win32 "core:sys/windows"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

compound_ray_pin_workers :: proc(dispatcher: ^util.Thread_Dispatcher, cell: string) -> (uintptr, e.Status)
{
	when ODIN_OS == .Windows
	{
		previous_caller: win32.DWORD_PTR;
		worker_lps: [32]u32 = COMPOUND_RAY_WORKER_LPS;
		// background threads are idle until dispatch, and each owns its recorded native handle
		for worker in 0..=dispatcher.background_count
		{
			handle: win32.HANDLE = win32.GetCurrentThread();
			native_tid: win32.DWORD = win32.GetCurrentThreadId();
			if worker > 0
			{
				background: ^util.Thread_Dispatcher_Worker = &dispatcher.background_workers[worker-1];
				handle = background.thread_handle.win32_thread;
				native_tid = background.thread_handle.win32_thread_id;
			}
			requested: win32.DWORD_PTR = win32.DWORD_PTR(1)<<worker_lps[worker];
			previous: win32.DWORD_PTR = win32.SetThreadAffinityMask(handle, requested);
			if worker == 0
			{
				previous_caller = previous;
			}
			effective: win32.DWORD_PTR;
			if previous != 0
			{
				effective = win32.SetThreadAffinityMask(handle, requested);
			}
			if previous == 0 || effective != requested
			{
				fmt.eprintfln("WORKER_AFFINITY_FAILED cell=%s worker=%d requested=%d effective=%d native_tid=%d", cell, worker, requested, effective, native_tid);
				return uintptr(previous_caller), .Invalid_Argument;
			}
			fmt.printfln("WORKER_AFFINITY cell=%s worker=%d requested=%d effective=%d native_tid=%d", cell, worker, requested, effective, native_tid);
		}
		return uintptr(previous_caller), .Ok;
	}
	else
	{
		return 0, .Invalid_Argument;
	}
}

compound_ray_restore_caller :: proc(previous: uintptr) -> e.Status
{
	when ODIN_OS == .Windows
	{
		if win32.SetThreadAffinityMask(win32.GetCurrentThread(), win32.DWORD_PTR(previous)) == 0
		{
			return .Invalid_Argument;
		}
	}
	return .Ok;
}
