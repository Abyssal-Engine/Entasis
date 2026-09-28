package external_dispatcher

import "core:os"
import entasis "entasis:entasis"

STEPS_PER_CASE :: 30;
STEPS :: 2*STEPS_PER_CASE+1;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	pool: entasis.Thread_Pool,
	interface: entasis.Dispatcher_Interface,
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [16]entasis.Body_Handle,
	included: [16]entasis.Body_State,
	cycle, tick: int,
}

available_worker_count :: proc() -> int
{
	return clamp(os.get_processor_core_count(), 1, 64);
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	pool_status: entasis.Dispatcher_Status = entasis.thread_pool_init(&c.pool, available_worker_count(), 65536);
	if pool_status != .Ok
	{
		return .Capacity_Missing if pool_status == .Capacity_Missing else .Invalid_Argument;
	}
	c.interface = entasis.dispatcher_from_thread_pool(&c.pool);
	if entasis.dispatcher_interface_validate(c.interface) != .Ok
	{
		return .Invalid_Description;
	}
	return case_world_create(c);
}

case_world_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {0, -5, 0};
	if c.cycle == 1
	{
		description.threading.worker_count = i32(c.interface.worker_count);
	}
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1};
	for &body, index in c.bodies
	{
		body, status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
			entasis.pose({f32(index), 10, 0}), {}, entasis.body_activity(-1, 255)));
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	if c.tick == STEPS_PER_CASE
	{
		if c.cycle != 0
		{
			return .Invalid_Argument;
		}
		entasis.world_destroy(&c.world);
		c.cycle, c.tick = 1, 0;
		return case_world_create(c);
	}
	status: entasis.Status;
	if c.cycle == 0
	{
		status = entasis.world_step(&c.world, TIMESTEP);
	}
	else
	{
		status = entasis.world_step_external(&c.world, TIMESTEP, c.interface);
	}
	if status != .Ok
	{
		return status;
	}
	c.tick += 1;
	if c.tick == STEPS_PER_CASE
	{
		for body, index in c.bodies
		{
			state: entasis.Body_State;
			state, status = entasis.body_get(&c.world, body);
			if status != .Ok
			{
				return status;
			}
			if c.cycle == 0
			{
				c.included[index] = state;
			}
			else if state != c.included[index]
			{
				return .Invalid_Argument;
			}
		}
	}
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	entasis.thread_pool_destroy(&c.pool);
	c^ = {};
}
