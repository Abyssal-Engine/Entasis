// SPDX-License-Identifier: Apache-2.0
// derived from the Entasis donor mailbox protocol
package entasis_utilities

import "core:sync"
import "core:time"

THREAD_DISPATCHER_MAILBOX_WAIT_US :: 750;
THREAD_DISPATCHER_MAILBOX_MAX_POLLS :: 65536;

Thread_Dispatcher_Mailbox :: struct #align(128)
{
	command: sync.Futex,
	command_pad: [124]u8,
	completion: sync.Futex,
	completion_pad: [124]u8,
}
#assert(size_of(Thread_Dispatcher_Mailbox) == 256);
#assert(offset_of(Thread_Dispatcher_Mailbox, completion) == 128);

Thread_Dispatcher_Mailbox_Relation :: enum u8
{
	Changed,
	Matched,
}

thread_dispatcher_mailboxes :: proc "contextless" (
	session: ^Thread_Dispatcher_Session,
) -> [^]Thread_Dispatcher_Mailbox
{
	// the aligned startup block owns one slot per background resident
	return ([^]Thread_Dispatcher_Mailbox)(rawptr(uintptr(session) + size_of(Thread_Dispatcher_Session)));
}

thread_dispatcher_mailbox_reset :: proc "contextless" (session: ^Thread_Dispatcher_Session)
{
	lanes: [^]Thread_Dispatcher_Mailbox = thread_dispatcher_mailboxes(session);
	for index in 0 ..< session.resident_count - 1
	{
		lanes[index] = {};
	}
}

thread_dispatcher_mailbox_publish :: proc "contextless" (word: ^sync.Futex, token: u32)
{
	observed: sync.Futex = sync.atomic_load_explicit(word, .Acquire);
	for
	{
		previous, exchanged := sync.atomic_compare_exchange_strong_explicit(
			word, observed, sync.Futex(token), .Acq_Rel, .Acquire,
		);
		if exchanged
		{
			if u32(observed) & 1 != 0
			{
				sync.futex_signal(word);
			}
			return;
		}
		observed = previous;
	}
}

thread_dispatcher_mailbox_wait :: proc "contextless" (
	word: ^sync.Futex, expected: u32, $relation: Thread_Dispatcher_Mailbox_Relation,
	started: time.Tick, budget: time.Duration, maximum_polls: int,
) -> u32
{
	current: sync.Futex = sync.atomic_load_explicit(word, .Acquire);
	clean: u32 = u32(current) & ~u32(1);
	if (relation == .Matched && clean == expected) || (relation == .Changed && clean != expected)
	{
		return clean;
	}
	if budget > 0
	{
		for poll_index in 0 ..< maximum_polls
		{
			if poll_index & 15 == 0 && time.tick_since(started) >= budget
			{
				break;
			}
			sync.cpu_relax();
			current = sync.atomic_load_explicit(word, .Acquire);
			clean = u32(current) & ~u32(1);
			if (relation == .Matched && clean == expected) || (relation == .Changed && clean != expected)
			{
				return clean;
			}
		}
	}
	// once parked, spurious wakes never restart the spin budget
	for
	{
		current = sync.atomic_load_explicit(word, .Acquire);
		clean = u32(current) & ~u32(1);
		if (relation == .Matched && clean == expected) || (relation == .Changed && clean != expected)
		{
			return clean;
		}
		sleeping: sync.Futex = sync.Futex(u32(current) | 1);
		if u32(current) & 1 == 0
		{
			_, exchanged := sync.atomic_compare_exchange_strong_explicit(
				word, current, sleeping, .Acq_Rel, .Acquire,
			);
			if !exchanged
			{
				continue;
			}
		}
		sync.futex_wait(word, u32(sleeping));
	}
}

thread_dispatcher_mailbox_resident :: proc "contextless" (
	session: ^Thread_Dispatcher_Session, worker_index: int,
	boundary: ^Thread_Dispatcher_Boundary,
)
{
	lane: ^Thread_Dispatcher_Mailbox = &thread_dispatcher_mailboxes(session)[worker_index - 1];
	observed: u32;
	for
	{
		current: u32 = u32(sync.atomic_load_explicit(&lane.command, .Acquire)) & ~u32(1);
		if current == observed
		{
			current = thread_dispatcher_mailbox_wait(
				&lane.command, observed, .Changed, time.tick_now(),
				time.Duration(THREAD_DISPATCHER_MAILBOX_WAIT_US) * time.Microsecond,
				THREAD_DISPATCHER_MAILBOX_MAX_POLLS,
			);
		}
		if current == THREAD_DISPATCHER_SESSION_STOP_GENERATION << 1
		{
			return;
		}
		observed = current;
		callback: Dispatcher_Worker_Proc = session.phase_worker;
		callback(worker_index, boundary);
		thread_dispatcher_mailbox_publish(&lane.completion, current);
	}
}

thread_dispatcher_mailbox_stop :: proc "contextless" (session: ^Thread_Dispatcher_Session)
{
	lanes: [^]Thread_Dispatcher_Mailbox = thread_dispatcher_mailboxes(session);
	for index in 0 ..< session.resident_count - 1
	{
		thread_dispatcher_mailbox_publish(&lanes[index].command, THREAD_DISPATCHER_SESSION_STOP_GENERATION << 1);
	}
}

thread_dispatcher_mailbox_phase :: proc "contextless" (
	session: ^Thread_Dispatcher_Session, boundary: ^Thread_Dispatcher_Boundary,
	callback: Dispatcher_Worker_Proc, admitted_count: int, generation: u32,
)
{
	lanes: [^]Thread_Dispatcher_Mailbox = thread_dispatcher_mailboxes(session);
	token: u32 = generation << 1;
	for index in 0 ..< admitted_count - 1
	{
		thread_dispatcher_mailbox_publish(&lanes[index].command, token);
	}
	callback(0, boundary);
	if admitted_count > 1
	{
		// every lane shares the owner's single residual join deadline
		join_started: time.Tick = time.tick_now();
		for index in 0 ..< admitted_count - 1
		{
			_ = thread_dispatcher_mailbox_wait(
				&lanes[index].completion, token, .Matched, join_started,
				time.Duration(THREAD_DISPATCHER_MAILBOX_WAIT_US) * time.Microsecond,
				THREAD_DISPATCHER_MAILBOX_MAX_POLLS,
			);
		}
	}
}
