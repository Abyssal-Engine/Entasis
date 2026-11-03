package solver_joint_angular

import "base:runtime"
import "core:math"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

Joint_Kind :: enum u8
{
	Hinge, Weld,
}
Link :: struct
{
	a, b: entasis.Body_Handle,
	kind: Joint_Kind,
}
Owner_State :: enum u8
{
	Empty, Ready,
}
Benchmark_Owner :: struct
{
	world: entasis.World,
	policy: entasis.Uniform_Gravity_Policy,
	bodies: []entasis.Body_Handle,
	edges: []Link,
	options: Options,
	state: Owner_State,
}

benchmark_owner_destroy :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	status: entasis.Status;
	if owner.state == .Ready
	{
		status = entasis.world_destroy(&owner.world);
	}
	delete(owner.bodies);
	delete(owner.edges);
	owner^ = {};
	return status;
}

add_body :: proc(owner: ^Benchmark_Owner, position: entasis.Vector3, orientation: entasis.Quaternion, mass: f32) -> (entasis.Body_Handle, entasis.Status)
{
	inertia: entasis.Body_Inertia;
	if mass > 0
	{
		inertia = {inverse_mass=1/mass, inverse_inertia_tensor={xx=11/mass, yy=66/mass, zz=11/mass}};
	}
	return entasis.body_add(&owner.world, entasis.body_shapeless(inertia, entasis.pose(position, orientation), {}, entasis.body_activity(-1, 255)));
}

benchmark_owner_create :: proc(owner: ^Benchmark_Owner, pool: ^entasis.Buffer_Pool, o: Options) -> support.Benchmark_Result
{
	owner.options = o;
	allocation_error: runtime.Allocator_Error;
	owner.bodies, allocation_error = make([]entasis.Body_Handle, o.chains*(o.links+1));
	if allocation_error != nil
	{
		return {status=.Allocation_Failed};
	}
	owner.edges, allocation_error = make([]Link, o.chains*o.links);
	if allocation_error != nil
	{
		return {status=.Allocation_Failed};
	}
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {0, -10, 0};
	description.damping = {};
	description.solve = {velocity_iterations=1, substeps=4, fallback_batch_threshold=64};
	description.threading.worker_count = i32(o.workers);
	description.capacity.bodies = i32(len(owner.bodies)+16);
	description.capacity.constraints = i32(len(owner.edges)+16);
	owner.policy = entasis.uniform_gravity_policy(description.gravity, 0, 0);
	description.pose_callbacks = entasis.pose_policy_uniform_mode(&owner.policy, o.angular_mode);
	status: entasis.Status = entasis.world_init_with_pool(&owner.world, description, pool);
	if status != .Ok
	{
		return {status=.Creation_Failed, physics_status=status};
	}
	owner.state = .Ready;
	for chain in 0 ..< o.chains
	{
		angle: f32 = 0.18+f32(chain%7)*0.017;
		qz: util.Quaternion = {z=math.sin(angle/2), w=math.cos(angle/2)};
		qx: util.Quaternion = {x=math.sin(f32(0.11)), w=math.cos(f32(0.11))};
		q: util.Quaternion = util.quaternion_concatenate(qz, qx);
		origin: entasis.Vector3 = {f32(chain%32)*3, f32(o.links+2), f32(chain/32)*3};
		previous: entasis.Body_Handle;
		previous, status = add_body(owner, origin, q, 0);
		if status != .Ok
		{
			return {status=.Fixture_Failed, physics_status=status};
		}
		owner.bodies[chain*(o.links+1)] = previous;
		for index in 1 ..= o.links
		{
			position: entasis.Vector3 = util.vector3_add(origin, util.quaternion_transform({0, -f32(index), 0}, q));
			mass: f32 = f32(o.mass_ratio) if index == o.links else 1;
			body: entasis.Body_Handle;
			body, status = add_body(owner, position, q, mass);
			if status != .Ok
			{
				return {status=.Fixture_Failed, physics_status=status};
			}
			owner.bodies[chain*(o.links+1)+index] = body;
			kind: Joint_Kind = .Hinge;
			if o.scenario == .Weld || (o.scenario == .Mixed && index%3 == 0)
			{
				kind = .Weld;
				_, status = entasis.constraint_add_2(&owner.world, previous, body, entasis.Weld{
					local_offset={0, -1, 0}, local_orientation={w=1}, spring_settings=entasis.spring_settings(30, 1),
				});
			}
			else
			{
				_, status = entasis.constraint_add_2(&owner.world, previous, body, entasis.Hinge{
					local_offset_a={0, -0.5, 0}, local_offset_b={0, 0.5, 0},
					local_hinge_axis_a={0, 0, 1}, local_hinge_axis_b={0, 0, 1}, spring_settings=entasis.spring_settings(30, 1),
				});
			}
			if status != .Ok
			{
				return {status=.Fixture_Failed, physics_status=status};
			}
			owner.edges[chain*o.links+index-1] = {previous, body, kind};
			previous = body;
		}
	}
	return {status=.Ok};
}
