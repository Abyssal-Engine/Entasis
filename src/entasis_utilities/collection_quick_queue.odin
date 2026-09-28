// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

Quick_Queue :: struct($T: typeid)
{
	count:         int,
	first_index:   int,
	last_index:    int,
	capacity_mask: int,
	span:          Buffer(T),
}

quick_queue_initialize_from_buffer :: proc (queue: ^Quick_Queue($T), span: Buffer(T)) -> Collection_Status
{
	if queue == nil || span.memory == nil || power_of_two_state(int(span.length)) != .Allocated
	{
		return .Invalid_Argument;
	}
	queue^ = Quick_Queue(T){span=span, capacity_mask=int(span.length) - 1, last_index=int(span.length) - 1};
	return .Ok;
}

quick_queue_initialize :: proc (
	queue: ^Quick_Queue($T),
	minimum_initial_count: int,
	pool: ^Buffer_Pool
) -> Collection_Status
{
	if queue == nil || minimum_initial_count <= 0 || pool == nil
	{
		return .Invalid_Argument;
	}
	span, status := buffer_pool_take_at_least(pool, T, minimum_initial_count);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	return quick_queue_initialize_from_buffer(queue, span);
}

quick_queue_backing_index :: proc "contextless" (queue: ^Quick_Queue($T), queue_index: int) -> int
{
	return (queue.first_index + queue_index) & queue.capacity_mask;
}

quick_queue_at :: proc (queue: ^Quick_Queue($T), queue_index: int) -> (^T, Collection_Status)
{
	if queue == nil || queue_index < 0 || queue_index >= queue.count
	{
		return nil, .Invalid_Argument;
	}
	return &queue.span.memory[quick_queue_backing_index(queue, queue_index)], .Ok;
}

quick_queue_resize :: proc (queue: ^Quick_Queue($T), new_size: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if queue == nil || pool == nil || new_size <= 0
	{
		return .Invalid_Argument;
	}
	target_capacity, capacity_status := buffer_pool_capacity_for_count(T, new_size);
	if capacity_status != .Ok
	{
		return memory_to_collection_status(capacity_status);
	}
	if target_capacity == int(queue.span.length)
	{
		return .Ok;
	}
	new_span, take_status := buffer_pool_take_at_least(pool, T, new_size);
	if take_status != .Ok
	{
		return memory_to_collection_status(take_status);
	}
	new_count := min(queue.count, int(new_span.length));
	for index in 0 ..< new_count
	{
		new_span.memory[index] = queue.span.memory[quick_queue_backing_index(queue, index)];
	}
	return_status := buffer_pool_return(pool, &queue.span);
	if return_status != .Ok
	{
		_ = buffer_pool_return(pool, &new_span);
		return memory_to_collection_status(return_status);
	}
	queue.span = new_span;
	queue.count = new_count;
	queue.first_index = 0;
	queue.last_index = new_count - 1;
	queue.capacity_mask = int(new_span.length) - 1;
	return .Ok;
}

quick_queue_ensure_capacity :: proc (queue: ^Quick_Queue($T), count: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if queue == nil || count < 0
	{
		return .Invalid_Argument;
	}
	if count <= int(queue.span.length)
	{
		return .Ok;
	}
	return quick_queue_resize(queue, count, pool);
}

quick_queue_compact :: proc (queue: ^Quick_Queue($T), pool: ^Buffer_Pool) -> Collection_Status
{
	if queue == nil || pool == nil
	{
		return .Invalid_Argument;
	}
	target, status := buffer_pool_capacity_for_count(T, max(queue.count, 1));
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	if target >= int(queue.span.length)
	{
		return .Ok;
	}
	return quick_queue_resize(queue, target, pool);
}

quick_queue_enqueue_unsafely :: proc (queue: ^Quick_Queue($T), value: T) -> Collection_Status
{
	if queue == nil || queue.count >= int(queue.span.length)
	{
		return .Capacity_Missing;
	}
	queue.count += 1;
	queue.last_index = (queue.last_index + 1) & queue.capacity_mask;
	queue.span.memory[queue.last_index] = value;
	return .Ok;
}

quick_queue_enqueue_first_unsafely :: proc (queue: ^Quick_Queue($T), value: T) -> Collection_Status
{
	if queue == nil || queue.count >= int(queue.span.length)
	{
		return .Capacity_Missing;
	}
	queue.count += 1;
	queue.first_index = (queue.first_index - 1) & queue.capacity_mask;
	queue.span.memory[queue.first_index] = value;
	return .Ok;
}

quick_queue_enqueue :: proc (queue: ^Quick_Queue($T), value: T, pool: ^Buffer_Pool) -> Collection_Status
{
	if queue == nil
	{
		return .Invalid_Argument;
	}
	if queue.count == int(queue.span.length)
	{
		status := quick_queue_resize(queue, int(queue.span.length) * 2, pool);
		if status != .Ok
		{
			return status;
		}
	}
	return quick_queue_enqueue_unsafely(queue, value);
}

quick_queue_enqueue_first :: proc (queue: ^Quick_Queue($T), value: T, pool: ^Buffer_Pool) -> Collection_Status
{
	if queue == nil
	{
		return .Invalid_Argument;
	}
	if queue.count == int(queue.span.length)
	{
		status := quick_queue_resize(queue, int(queue.span.length) * 2, pool);
		if status != .Ok
		{
			return status;
		}
	}
	return quick_queue_enqueue_first_unsafely(queue, value);
}

quick_queue_dequeue :: proc (queue: ^Quick_Queue($T)) -> (T, Collection_Status)
{
	if queue == nil || queue.count <= 0
	{
		return {}, .Empty;
	}
	value := queue.span.memory[queue.first_index];
	queue.span.memory[queue.first_index] = {};
	queue.first_index = (queue.first_index + 1) & queue.capacity_mask;
	queue.count -= 1;
	return value, .Ok;
}

quick_queue_dequeue_last :: proc (queue: ^Quick_Queue($T)) -> (T, Collection_Status)
{
	if queue == nil || queue.count <= 0
	{
		return {}, .Empty;
	}
	value := queue.span.memory[queue.last_index];
	queue.span.memory[queue.last_index] = {};
	queue.last_index = (queue.last_index - 1) & queue.capacity_mask;
	queue.count -= 1;
	return value, .Ok;
}

quick_queue_remove_at :: proc (queue: ^Quick_Queue($T), queue_index: int) -> Collection_Status
{
	if queue == nil || queue_index < 0 || queue_index >= queue.count
	{
		return .Not_Found;
	}
	if queue_index < queue.count / 2
	{
		for index := queue_index; index > 0; index -= 1
		{
			queue.span.memory[quick_queue_backing_index(queue, index)] = queue.span.memory[quick_queue_backing_index(
				queue,
				index - 1
			)];
		}
		queue.span.memory[queue.first_index] = {};
		queue.first_index = (queue.first_index + 1) & queue.capacity_mask;
	}
	else
	{
		for index := queue_index; index < queue.count - 1; index += 1
		{
			queue.span.memory[quick_queue_backing_index(queue, index)] = queue.span.memory[quick_queue_backing_index(
				queue,
				index + 1
			)];
		}
		queue.span.memory[queue.last_index] = {};
		queue.last_index = (queue.last_index - 1) & queue.capacity_mask;
	}
	queue.count -= 1;
	return .Ok;
}

quick_queue_clear :: proc (queue: ^Quick_Queue($T))
{
	if queue == nil
	{
		return;
	}
	for index in 0 ..< queue.count
	{
		queue.span.memory[quick_queue_backing_index(queue, index)] = {};
	}
	queue.count = 0;
	queue.first_index = 0;
	queue.last_index = queue.capacity_mask;
}

quick_queue_fast_clear :: proc (queue: ^Quick_Queue($T))
{
	if queue == nil
	{
		return;
	}
	queue.count = 0;
	queue.first_index = 0;
	queue.last_index = queue.capacity_mask;
}

quick_queue_dispose :: proc (queue: ^Quick_Queue($T), pool: ^Buffer_Pool) -> Collection_Status
{
	if queue == nil || pool == nil || queue.span.memory == nil
	{
		return .Disposed;
	}
	status := buffer_pool_return(pool, &queue.span);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	queue^ = {};
	return .Ok;
}
