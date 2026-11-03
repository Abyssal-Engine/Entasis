package simulation_stage_work

import "core:fmt"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import container "../container"
import islands "../contact_islands"
import support "../benchmark_support"

#assert(util.PRECISION_MODE == .Float_Coordinates);
WARMUP_FRAMES :: 30;
MEASURED_FRAMES :: 300;
PAIR_SLOTS :: 32;
TIMESTEP :: f32(1.0 / 60.0);

Static_Box :: struct
{
	minimum, maximum: util.Vector3,
}
Pair_Output :: struct
{
	keys: []u64,
	counts: []int,
	capacity: int,
}
Run :: struct
{
	options: Options,
	container_owner: container.Benchmark_Owner,
	islands_owner: islands.Benchmark_Owner,
	pool: entasis.Buffer_Pool,
	simulation: ^physics.Simulation,
	initial: []util.Vector3,
	statics: []Static_Box,
	grid: [3]int,
	group_size: int,
	pairs: Pair_Output,
	expected, seen: []u8,
	frame_index, completed, failures, first_failure: int,
	elapsed_ms: f64,
	pair_count, manifold_count, contact_points: i64,
	last_pairs, last_manifolds: int,
	pool_before, pool_after, worker_before, worker_after: u64,
	status: physics.Physics_Status,
}

run_world :: proc(run: ^Run) -> ^entasis.World
{
	if run.options.fixture == .Container
	{
		return &run.container_owner.world;
	}
	return &run.islands_owner.world;
}

run_scene_options :: proc(o: Options, grid: [3]int, groups: [2]int) -> (support.Options, support.Admission)
{
	worker_argument: string = fmt.aprintf("--worker-count=%d", o.workers);
	grid_argument: string = fmt.aprintf("--grid=%d,%d,%d", grid[0], grid[1], grid[2]);
	group_argument: string = fmt.aprintf("--island-grid=%d,%d", groups[0], groups[1]);
	defer delete(worker_argument);
	defer delete(grid_argument);
	defer delete(group_argument);
	arguments: [15]string = {
		worker_argument, "--shape=box", "--shape-size=1,1,1", "--output=unused.csv",
		"--steps=300", "--warmup-steps=30", "--velocity-iterations=1", "--substeps=4",
		"--timestep-hz=60", "--sleep=disabled", "--record=off", "--verify=on",
		grid_argument, "--spawn-height=0.49", group_argument,
	};
	scene: support.Workload = .Container;
	if o.fixture == .Islands
	{
		scene = .Contact_Islands;
	}
	options: support.Options;
	diagnostic: string;
	argument_count: int = 14;
	if o.fixture == .Islands
	{
		argument_count = 15;
	}
	options, diagnostic = support.parse_options(scene, arguments[:argument_count]);
	if diagnostic != ""
	{
		fmt.eprintfln("STAGE_WORK_FIXTURE_REJECT reason=%s", diagnostic);
		return {}, .Invalid;
	}
	if o.fixture == .Container
	{
		options.spacing = {1.02, 0.98, 1.02};
	}
	else
	{
		options.spacing = {0.98, 0.98, 0.98};
	}
	return options, .Ok;
}

run_create :: proc(run: ^Run, o: Options, grid: [3]int, groups: [2]int = {1, 1}) -> physics.Physics_Status
{
	run.options = o;
	run.grid = grid;
	run.group_size = grid[0]*grid[1]*grid[2];
	options: support.Options;
	admission: support.Admission;
	options, admission = run_scene_options(o, grid, groups);
	if admission != .Ok
	{
		return .Invalid_Argument;
	}
	if entasis.buffer_pool_init(&run.pool, 65536) != .Ok
	{
		return .Capacity_Missing;
	}
	callback: entasis.Timestepper = {step=stage_timestep, user_context=run};
	result: support.Benchmark_Result;
	if o.fixture == .Container
	{
		result = container.benchmark_owner_create(&run.container_owner, &run.pool, options, callback);
	}
	else
	{
		result = islands.benchmark_owner_create(&run.islands_owner, &run.pool, options, callback);
	}
	if result.status != .Ok
	{
		fmt.eprintfln("STAGE_WORK_FIXTURE_FAILED status=%v physics_status=%v bodies=%d", result.status, result.physics_status, options.body_count);
		return .Invalid_Argument;
	}
	body_count: int = options.body_count;
	run.initial = make([]util.Vector3, body_count);
	run.expected = make([]u8, body_count*PAIR_SLOTS);
	run.seen = make([]u8, body_count*PAIR_SLOTS);
	run.pairs.capacity = body_count*PAIR_SLOTS;
	run.pairs.keys = make([]u64, run.pairs.capacity*o.workers);
	run.pairs.counts = make([]int, o.workers);
	run.statics = make([]Static_Box, options.static_count);
	view: entasis.Active_Body_View;
	status: entasis.Status;
	view, status = entasis.active_body_view(run_world(run));
	if status != .Ok || view.count != body_count
	{
		return .Invalid_Argument;
	}
	for i in 0 ..< view.count
	{
		run.initial[i] = view.dynamics[i].motion.pose.position;
	}
	if o.fixture == .Container
	{
		t: f32 = 1;
		x, y, z: f32 = options.container_size[0]/2, (options.container_size[1]-t)/2, options.container_size[2]/2;
		centers: [5]util.Vector3 = {{0, -t/2, 0}, {-x, y, 0}, {x, y, 0}, {0, y, -z}, {0, y, z}};
		extents: [5]util.Vector3 = {
			{x, t/2, z}, {t/2, options.container_size[1]/2, z}, {t/2, options.container_size[1]/2, z},
			{x, options.container_size[1]/2, t/2}, {x, options.container_size[1]/2, t/2},
		};
		for i in 0 ..< len(centers)
		{
			run.statics[i] = {minimum=util.vector3_subtract(centers[i], extents[i]), maximum=util.vector3_add(centers[i], extents[i])};
		}
	}
	else
	{
		for z in 0 ..< groups[1]
		{
			for x in 0 ..< groups[0]
			{
				center: util.Vector3 = {islands.island_origin(x, groups[0], options.island_spacing[0]), -options.floor_size[1]/2, islands.island_origin(z, groups[1], options.island_spacing[1])};
				extent: util.Vector3 = util.Vector3{options.floor_size[0]/2, options.floor_size[1]/2, options.floor_size[2]/2};
				run.statics[z*groups[0]+x] = {minimum=util.vector3_subtract(center, extent), maximum=util.vector3_add(center, extent)};
			}
		}
	}
	return .Ok;
}

run_destroy :: proc(run: ^Run)
{
	if run.options.fixture == .Container
	{
		_ = container.benchmark_owner_destroy(&run.container_owner);
	}
	else
	{
		_ = islands.benchmark_owner_destroy(&run.islands_owner);
	}
	_ = entasis.buffer_pool_destroy(&run.pool);
	delete(run.initial);
	delete(run.statics);
	delete(run.expected);
	delete(run.seen);
	delete(run.pairs.keys);
	delete(run.pairs.counts);
	run^ = {};
}

prescribed_position :: proc "contextless" (run: ^Run, index, frame: int) -> util.Vector3
{
	position: util.Vector3 = run.initial[index];
	if run.options.motion == .Churn && frame & 1 != 0 && index & 1 != 0
	{
		position.y += 0.25;
	}
	return position;
}

prescribe :: proc(run: ^Run, simulation: ^physics.Simulation, frame: int)
{
	active: ^physics.Body_Set = &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for index in 0 ..< active.count
	{
		active.dynamics_state.memory[index].motion.pose = {position=prescribed_position(run, index, frame), orientation={w=1}};
		active.dynamics_state.memory[index].motion.velocity = {};
	}
}

worker_capacity :: proc(dispatcher: ^util.Thread_Dispatcher_Boundary) -> u64
{
	if dispatcher == nil
	{
		return 0;
	}
	total: u64;
	for worker in 0 ..< dispatcher.worker_count
	{
		pool: ^util.Buffer_Pool;
		status: util.Threading_Status;
		pool, status = dispatcher.worker_pool(dispatcher, worker);
		if status == .Ok
		{
			total += util.buffer_pool_total_allocated_byte_count(pool);
		}
	}
	return total;
}
