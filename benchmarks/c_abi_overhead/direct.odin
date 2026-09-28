package native_overhead

import "core:fmt"
import "core:os"
import "core:time"
import entasis "entasis:entasis"

BODY_COUNT :: 10_000;
CREATE_BODY_COUNT :: 10_000;
CREATE_STATIC_COUNT :: 10_000;
QUERY_STATIC_COUNT :: 1_024;
QUERY_COUNT :: 256;
EVENT_PAIR_COUNT :: 2_048;
STEP_REPEATS :: 64;
MUTATION_REPEATS :: 8;
QUERY_REPEATS :: 64;
STEP_WARMUP :: 4;
TIMESTEP :: f32(1.0/60.0);

Operation :: enum
{
	Step, Create, Mutate, Query, Events,
}
Owner :: struct
{
	world: entasis.World,
	body_descriptions, a, b: []entasis.Body_Description,
	bodies: []entasis.Body_Handle,
	static_descriptions: []entasis.Static_Description,
	statics: []entasis.Static_Handle,
	queries: []entasis.Query,
	results: []entasis.Query_Result,
	tracker: entasis.Contact_Tracker,
	events: []entasis.Contact_Event,
	written, required: int,
}

fail :: proc(message: string) -> !
{
	fmt.eprintln(message);
	os.exit(2);
}

shapeless_inertia :: proc "contextless" () -> entasis.Body_Inertia
{
	return {inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1};
}

world_description :: proc(body_count, static_count, pair_count: int) -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.threading.worker_count = 1;
	description.capacity.bodies = i32(max(body_count, 1));
	description.capacity.statics = i32(max(static_count, 1));
	description.capacity.shapes_per_type = 8;
	description.capacity.broad_phase_candidates = i32(max(body_count+static_count, 1024));
	description.capacity.pairs = i32(max(pair_count, 1024));
	description.capacity.inactive_pairs = i32(max(pair_count, 1024));
	return description;
}

case_create :: proc(o: ^Owner, operation: Operation) -> entasis.Status
{
	body_count, static_count, pair_count: int;
	switch operation
	{
	case .Step, .Mutate: body_count = BODY_COUNT;
	case .Create: body_count, static_count = CREATE_BODY_COUNT, CREATE_STATIC_COUNT;
	case .Query: static_count = QUERY_STATIC_COUNT;
	case .Events: body_count, static_count, pair_count = EVENT_PAIR_COUNT, EVENT_PAIR_COUNT, EVENT_PAIR_COUNT*2;
	}
	status: entasis.Status = entasis.world_init(&o.world, world_description(body_count, static_count, pair_count));
	if status != .Ok
	{
		return status;
	}
	o.body_descriptions = make([]entasis.Body_Description, body_count);
	o.bodies = make([]entasis.Body_Handle, body_count);
	o.static_descriptions = make([]entasis.Static_Description, static_count);
	o.statics = make([]entasis.Static_Handle, static_count);
	if (body_count > 0 && (o.body_descriptions == nil || o.bodies == nil)) ||
		(static_count > 0 && (o.static_descriptions == nil || o.statics == nil))
	{
		return .Capacity_Missing;
	}
	shape: entasis.Shape_Handle;
	if static_count > 0
	{
		shape, status = entasis.shape_add(&o.world, entasis.sphere(0.5 if operation == .Events else 0.25));
		if status != .Ok
		{
			return status;
		}
	}
	inertia: entasis.Body_Inertia = shapeless_inertia();
	if operation == .Events
	{
		inertia, status = entasis.shape_registered_inertia(&o.world, shape, 1);
		if status != .Ok
		{
			return status;
		}
	}
	for &description, index in o.body_descriptions
	{
		position: entasis.Vector3;
		velocity: entasis.Body_Velocity;
		#partial switch operation
		{
		case .Step:
			position = {f32(index%100), f32(index/100), 0};
			velocity = entasis.velocity({f32((index%7)+1)*0.01, 0, 0});
		case .Create: position = {f32(index), 10, 0};
		case .Mutate: position = {f32(index), 0, 0};
		case .Events: position = {f32(index)*3, 0, 0};
		}
		description = entasis.body_shapeless(inertia, entasis.pose(position), velocity, entasis.body_activity(-1, 255));
		if operation == .Events
		{
			description = entasis.body_dynamic(shape, inertia, entasis.pose(position), {}, entasis.body_activity(-1, 255));
		}
	}
	for &description, index in o.static_descriptions
	{
		position: entasis.Vector3 = {f32(index)*2, -10 if operation == .Create else 0, 0};
		if operation == .Events
		{
			position = {f32(index)*3, 0, 0};
		}
		description = entasis.static_body(shape, entasis.pose(position));
	}
	if operation == .Create
	{
		return .Ok;
	}
	if body_count > 0
	{
		written: int;
		written, status = entasis.body_add_batch(&o.world, o.body_descriptions, o.bodies);
		if status != .Ok || written != body_count
		{
			return .Invalid_Argument;
		}
	}
	if static_count > 0
	{
		written: int;
		written, status = entasis.static_add_batch(&o.world, o.static_descriptions, o.statics, .None);
		if status != .Ok || written != static_count
		{
			return .Invalid_Argument;
		}
	}
	#partial switch operation
	{
	case .Mutate:
		o.a = make([]entasis.Body_Description, body_count);
		o.b = make([]entasis.Body_Description, body_count);
		if o.a == nil || o.b == nil
		{
			return .Capacity_Missing;
		}
		for description, index in o.body_descriptions
		{
			o.a[index], o.b[index] = description, description;
			o.a[index].pose.position.y, o.b[index].pose.position.y = 1, 2;
		}
	case .Query:
		o.queries = make([]entasis.Query, QUERY_COUNT);
		o.results = make([]entasis.Query_Result, QUERY_COUNT);
		if o.queries == nil || o.results == nil
		{
			return .Capacity_Missing;
		}
		for &query, index in o.queries
		{
			query = entasis.query_ray_closest(entasis.ray({-1, f32(index%8)*0.01, 0}, {1, 0, 0}, f32(QUERY_STATIC_COUNT*2+2)));
		}
	case .Events:
		o.events = make([]entasis.Contact_Event, EVENT_PAIR_COUNT*2);
		if o.events == nil
		{
			return .Capacity_Missing;
		}
		status = entasis.contact_tracker_init(&o.tracker, EVENT_PAIR_COUNT*2);
		if status != .Ok
		{
			return status;
		}
		status = entasis.contact_tracker_bind(&o.tracker, &o.world);
		if status != .Ok
		{
			return status;
		}
		return entasis.world_step(&o.world, TIMESTEP);
	}
	return .Ok;
}

create_batches :: #force_inline proc(o: ^Owner) -> (int, int, entasis.Status, entasis.Status)
{
	body_written, body_status := entasis.body_add_batch(&o.world, o.body_descriptions, o.bodies);
	static_written, static_status := entasis.static_add_batch(&o.world, o.static_descriptions, o.statics, .None);
	return body_written, static_written, body_status, static_status;
}

mutate :: #force_inline proc(o: ^Owner, iteration: int) -> (int, entasis.Status)
{
	values: []entasis.Body_Description = o.a;
	if iteration&1 != 0
	{
		values = o.b;
	}
	return entasis.body_apply_batch(&o.world, o.bodies, values);
}

case_checksum :: proc(o: ^Owner, operation: Operation) -> (f64, entasis.Status)
{
	switch operation
	{
	case .Step, .Mutate:
		state, status := entasis.body_get(&o.world, o.bodies[len(o.bodies)-1]);
		return f64(state.pose.position.x if operation == .Step else state.pose.position.y), status;
	case .Create:
		return f64(o.bodies[len(o.bodies)-1].value)+f64(o.statics[len(o.statics)-1].value), .Ok;
	case .Query:
		checksum: f64;
		for result in o.results
		{
			if result.hit
			{
				checksum += f64(result.ray_hit.t);
			}
		}
		return checksum, .Ok if checksum > 0 else .Invalid_Argument;
	case .Events:
		return f64(o.required), .Ok if o.written == o.required && o.written >= EVENT_PAIR_COUNT else .Invalid_Argument;
	}
	unreachable();
}

case_destroy :: proc(o: ^Owner)
{
	entasis.contact_tracker_destroy(&o.tracker);
	entasis.world_destroy(&o.world);
	delete(o.body_descriptions);
	delete(o.a);
	delete(o.b);
	delete(o.bodies);
	delete(o.static_descriptions);
	delete(o.statics);
	delete(o.queries);
	delete(o.results);
	delete(o.events);
	o^ = {};
}

Result :: struct
{
	nanoseconds: i64,
	operations: i64,
	checksum: f64,
}

step_10k :: proc() -> Result
{
	owner: Owner;
	if case_create(&owner, .Step) != .Ok
	{
		fail("step setup");
	}
	defer case_destroy(&owner);
	for _ in 0 ..< STEP_WARMUP
	{
		if entasis.world_step(&owner.world, TIMESTEP) != .Ok
		{
			fail("step warmup");
		}
	}
	start := time.tick_now();
	for _ in 0 ..< STEP_REPEATS
	{
		if entasis.world_step(&owner.world, TIMESTEP) != .Ok
		{
			fail("step measured");
		}
	}
	end := time.tick_now();
	checksum, status := case_checksum(&owner, .Step);
	if status != .Ok
	{
		fail("step checksum");
	}
	return {i64(time.tick_diff(start, end)), BODY_COUNT*STEP_REPEATS, checksum};
}

create_10k :: proc() -> Result
{
	owner: Owner;
	if case_create(&owner, .Create) != .Ok
	{
		fail("create setup");
	}
	defer case_destroy(&owner);
	start := time.tick_now();
	body_written, static_written, body_status, static_status := create_batches(&owner);
	end := time.tick_now();
	if body_status != .Ok || static_status != .Ok || body_written != CREATE_BODY_COUNT || static_written != CREATE_STATIC_COUNT
	{
		fail("create batches");
	}
	checksum, _ := case_checksum(&owner, .Create);
	return {i64(time.tick_diff(start, end)), CREATE_BODY_COUNT+CREATE_STATIC_COUNT, checksum};
}

mutate_10k :: proc() -> Result
{
	owner: Owner;
	if case_create(&owner, .Mutate) != .Ok
	{
		fail("mutate setup");
	}
	defer case_destroy(&owner);
	start := time.tick_now();
	for iteration in 0 ..< MUTATION_REPEATS
	{
		applied, status := mutate(&owner, iteration);
		if status != .Ok || applied != BODY_COUNT
		{
			fail("mutate apply");
		}
	}
	end := time.tick_now();
	checksum, status := case_checksum(&owner, .Mutate);
	if status != .Ok
	{
		fail("mutate checksum");
	}
	return {i64(time.tick_diff(start, end)), BODY_COUNT*MUTATION_REPEATS, checksum};
}

query_batch :: proc() -> Result
{
	owner: Owner;
	if case_create(&owner, .Query) != .Ok
	{
		fail("query setup");
	}
	defer case_destroy(&owner);
	if entasis.query_batch(&owner.world, owner.queries, owner.results) != .Ok
	{
		fail("query warmup");
	}
	start := time.tick_now();
	for _ in 0 ..< QUERY_REPEATS
	{
		if entasis.query_batch(&owner.world, owner.queries, owner.results) != .Ok
		{
			fail("query measured");
		}
	}
	end := time.tick_now();
	checksum, status := case_checksum(&owner, .Query);
	if status != .Ok
	{
		fail("query checksum");
	}
	return {i64(time.tick_diff(start, end)), QUERY_COUNT*QUERY_REPEATS, checksum};
}

event_drain :: proc() -> Result
{
	owner: Owner;
	if case_create(&owner, .Events) != .Ok
	{
		fail("event setup");
	}
	defer case_destroy(&owner);
	start := time.tick_now();
	written, required, status := entasis.contact_events_drain(&owner.tracker, owner.events);
	end := time.tick_now();
	if status != .Ok || written != required || written < EVENT_PAIR_COUNT
	{
		fail("event drain");
	}
	return {i64(time.tick_diff(start, end)), i64(written), f64(required)};
}

main :: proc()
{
	if len(os.args) != 2
	{
		fail("usage: direct <step_10k|create_10k|mutate_10k|query_batch|event_drain>");
	}
	scenario := os.args[1];
	result: Result;
	switch scenario
	{
		case "step_10k":
			result = step_10k();
		case "create_10k":
			result = create_10k();
		case "mutate_10k":
			result = mutate_10k();
		case "query_batch":
			result = query_batch();
		case "event_drain":
			result = event_drain();
		case:
			fail("unknown scenario");
	}
	fmt.printfln("%s,%d,%d,%.9f", scenario, result.nanoseconds, result.operations, result.checksum);
}
