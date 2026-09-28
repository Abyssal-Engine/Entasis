package world_lifecycle

import e "entasis:entasis"
import c "entasis:entasis_cooking"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

CYCLES :: 20;
BODY_COUNT :: 2048;
COMPONENTS :: [5]string{"initialization", "growth", "cooking_import", "clear_reuse", "destruction"};
Geometry :: struct
{
	points: [8]e.Vector3,
	triangles: [2]e.Triangle,
	children: [2]c.Cooked_Compound_Child,
}
Owner :: struct
{
	world: e.World,
	cooking: c.Cooking_Context,
	hull: c.Cooked_Hull,
	mesh: c.Cooked_Mesh,
	compound: c.Cooked_Compound,
	box: e.Shape_Handle,
	imported: [3]e.Shape_Handle,
}
Result :: struct
{
	elapsed: [5]f64,
	world_native_bytes, worker_native_bytes, cooking_native_bytes: u64,
}

// caller-owned status storage keeps result checking outside each timed region.
// shape/body inputs and operation counts are fixed by this fixture
populate :: proc(owner: ^Owner, statuses: ^[BODY_COUNT*2+2]e.Status)
{
	value: e.Box = e.box(1, 1, 1);
	owner.box, statuses[0] = e.shape_add(&owner.world, value);
	inertia: e.Body_Inertia;
	inertia, statuses[1] = e.shape_inertia(value, 1);
	constraint: e.One_Body_Linear_Motor = {target_velocity={1, 0, 0}, settings={maximum_force=10, damping=1}};
	for i in 0..<BODY_COUNT
	{
		body: e.Body_Handle;
		body, statuses[2+i*2] = e.body_add(&owner.world, e.body_dynamic(owner.box, inertia,
			e.pose({f32(i%32)*2, 2, f32(i/32)*2}), {}, e.body_activity(-1, 255)));
		_, statuses[3+i*2] = e.constraint_add(&owner.world, []e.Body_Handle{body}, constraint);
	}
}

validate_world :: proc(owner: ^Owner) -> e.Status
{
	stats: e.World_Stats;
	status: e.Status;
	stats, status = e.world_stats(&owner.world);
	if status != .Ok
	{
		return status;
	}
	if stats.active_bodies != BODY_COUNT || stats.active_constraints != BODY_COUNT || stats.sleeping_bodies != 0
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

measure_pools :: proc(owner: ^Owner, result: ^Result)
{
	pool: ^e.Buffer_Pool;
	pool_status: e.Status;
	pool, pool_status = e.world_borrow_pool(&owner.world);
	support.extension_require(pool_status, "lifecycle pool");
	result.world_native_bytes = max(result.world_native_bytes, util.buffer_pool_total_allocated_byte_count(pool));
	dispatcher: ^e.Dispatcher;
	dispatcher_status: e.Status;
	dispatcher, dispatcher_status = e.world_borrow_dispatcher(&owner.world);
	support.extension_require(dispatcher_status, "lifecycle dispatcher");
	worker_bytes: u64;
	if dispatcher != nil
	{
		for i in 0..<dispatcher.worker_count
		{
			worker: ^util.Buffer_Pool;
			status: util.Threading_Status;
			worker, status = dispatcher.worker_pool(dispatcher, i);
			if status != .Ok
			{
				panic("lifecycle worker pool");
			}
			worker_bytes += util.buffer_pool_total_allocated_byte_count(worker);
		}
	}
	result.worker_native_bytes = max(result.worker_native_bytes, worker_bytes);
	result.cooking_native_bytes = max(result.cooking_native_bytes, util.buffer_pool_total_allocated_byte_count(&owner.cooking.pool));
}

fixture_geometry :: proc() -> Geometry
{
	return {
		points={{-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1}, {-1, -1, 1}, {1, -1, 1}, {-1, 1, 1}, {1, 1, 1}},
		triangles={e.triangle({-2, 0, -2}, {2, 0, -2}, {2, 0, 2}), e.triangle({-2, 0, -2}, {2, 0, 2}, {-2, 0, 2})},
		children={{local_pose=e.pose({-2, 0, 0}), shape_slot=0}, {local_pose=e.pose({2, 0, 0}), shape_slot=1}},
	};
}

world_description :: proc(workers: int) -> e.World_Description
{
	desc: e.World_Description = e.world_description_default();
	desc.threading.worker_count = i32(workers);
	desc.capacity.bodies = 128;
	desc.capacity.constraints = 128;
	desc.capacity.initial_constraints_per_type_batch = 128;
	return desc;
}

initialize :: proc(owner: ^Owner, desc: e.World_Description, statuses: ^[2]e.Status)
{
	statuses[0] = e.world_init(&owner.world, desc);
	statuses[1] = c.cooking_context_init(&owner.cooking);
}

import_cooked :: proc(owner: ^Owner, geometry: ^Geometry, statuses: ^[6]e.Status)
{
	owner.hull, statuses[0] = c.cook_hull(&owner.cooking, geometry.points[:]);
	owner.mesh, statuses[1] = c.cook_mesh(&owner.cooking, geometry.triangles[:]);
	owner.compound, statuses[2] = c.cook_compound(&owner.cooking, geometry.children[:]);
	owner.imported[0], statuses[3] = c.cooked_hull_import(&owner.world, &owner.hull);
	owner.imported[1], statuses[4] = c.cooked_mesh_import(&owner.world, &owner.mesh);
	slots: [2]e.Shape_Handle = {owner.imported[0], owner.box};
	owner.imported[2], statuses[5] = c.cooked_compound_import(&owner.world, &owner.compound, slots[:]);
}

clear_repopulate :: proc(owner: ^Owner, statuses: ^[BODY_COUNT*2+2]e.Status) -> e.Status
{
	status: e.Status = e.world_clear(&owner.world);
	populate(owner, statuses);
	return status;
}

destroy :: proc(owner: ^Owner, statuses: ^[5]e.Status)
{
	statuses[0] = e.world_destroy(&owner.world);
	statuses[1] = c.cooked_compound_destroy(&owner.compound);
	statuses[2] = c.cooked_mesh_destroy(&owner.mesh);
	statuses[3] = c.cooked_hull_destroy(&owner.hull);
	statuses[4] = c.cooking_context_destroy(&owner.cooking);
}
