package solver_fixed_work

import "core:simd"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import contacts "../solver_contacts"

FIXED_SPRING_FREQUENCY_HZ :: f32(30);
FIXED_SPRING_DAMPING_RATIO :: f32(1);

fixed_contact_material :: proc() -> physics.Contact_Material_Properties
{
	return {friction_coefficient=0.5, spring_settings=physics.spring_settings_create(FIXED_SPRING_FREQUENCY_HZ, FIXED_SPRING_DAMPING_RATIO), maximum_recovery_velocity=2};
}

fixed_constraint :: proc(s: ^Fixed_State, connected: ^[4]physics.Body_Handle, description: ^$T, points, carried: int) -> physics.Physics_Status
{
	handle: physics.Constraint_Handle;
	status: physics.Physics_Status;
	handle, status = physics.solver_add(&s.solver, connected, description);
	if status != .Ok
	{
		return status;
	}
	s.points += points;
	if carried != 0
	{
		location: physics.Constraint_Location;
		location, status = physics.solver_resolve(&s.solver, handle);
		if status != .Ok
		{
			return status;
		}
		batch: ^physics.Constraint_Batch = &s.solver.active_set.batches.memory[location.batch_index];
		kind: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
		impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(kind, int(location.index_in_type_batch)));
		lane: int = int(location.index_in_type_batch)%8;
		for point in 0 ..< points
		{
			component: int = point+2;
			if s.options.case_name == .Nonconvex
			{
				component = point*3+2;
			}
			impulses[component] = simd.replace(impulses[component], lane, 0.015);
		}
	}
	return .Ok;
}

fixed_add_contacts :: proc(s: ^Fixed_State) -> physics.Physics_Status
{
	c0: physics.Constraint_Contact_Data = {{-0.25, -0.5, -0.25}, 0.02};
	c1: physics.Constraint_Contact_Data = {{0.25, -0.5, -0.25}, -0.01};
	c2: physics.Constraint_Contact_Data = {{-0.25, -0.5, 0.25}, 0.015};
	c3: physics.Constraint_Contact_Data = {{0.25, -0.5, 0.25}, 0.01};
	n0: physics.Nonconvex_Constraint_Contact_Data = {c0.offset_a, {0, 1, 0}, c0.penetration_depth};
	n1: physics.Nonconvex_Constraint_Contact_Data = {c1.offset_a, {0.6, 0.8, 0}, c1.penetration_depth};
	n2: physics.Nonconvex_Constraint_Contact_Data = {c2.offset_a, {0, 0.8, 0.6}, c2.penetration_depth};
	n3: physics.Nonconvex_Constraint_Contact_Data = {c3.offset_a, {-0.6, 0.8, 0}, c3.penetration_depth};
	material: physics.Contact_Material_Properties = fixed_contact_material();
	one: physics.Nonconvex_One_Body_Properties = {material.friction_coefficient, material.spring_settings, material.maximum_recovery_velocity};
	two: physics.Nonconvex_Two_Body_Properties = {{0, 1.05, 0}, material.friction_coefficient, material.spring_settings, material.maximum_recovery_velocity};
	for pass in 0 ..< 3
	{
		for i in 0 ..< len(s.handles)
		{
			connected: [4]physics.Body_Handle = {0=s.handles[i], 1=s.handles[(i+pass+1)%len(s.handles)]};
			active: ^physics.Body_Set = &s.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
			offset_b: util.Vector3 = util.vector3_subtract(active.dynamics_state.memory[connected[1].value].motion.pose.position,
				active.dynamics_state.memory[connected[0].value].motion.pose.position);
			two.offset_b = offset_b;
			carried: int = i%2;
			status: physics.Physics_Status;
			if s.options.case_name == .Nonconvex
			{
				kind: int = i%3;
				if pass == 0
				{
					switch kind
					{
					case 0:
						description: physics.Contact_2_Nonconvex_One_Body = {one, n0, n1};
						status = fixed_constraint(s, &connected, &description, 2, carried);
					case 1:
						description: physics.Contact_3_Nonconvex_One_Body = {one, n0, n1, n2};
						status = fixed_constraint(s, &connected, &description, 3, carried);
					case 2:
						description: physics.Contact_4_Nonconvex_One_Body = {one, n0, n1, n2, n3};
						status = fixed_constraint(s, &connected, &description, 4, carried);
					}
				}
				else
				{
					switch kind
					{
					case 0:
						description: physics.Contact_2_Nonconvex = {two, n0, n1};
						status = fixed_constraint(s, &connected, &description, 2, carried);
					case 1:
						description: physics.Contact_3_Nonconvex = {two, n0, n1, n2};
						status = fixed_constraint(s, &connected, &description, 3, carried);
					case 2:
						description: physics.Contact_4_Nonconvex = {two, n0, n1, n2, n3};
						status = fixed_constraint(s, &connected, &description, 4, carried);
					}
				}
			}
			else if pass == 0
			{
				switch i%4
				{
				case 0:
					description: physics.Contact_1_One_Body = {c0, {0, 1, 0}, material};
					status = fixed_constraint(s, &connected, &description, 1, carried);
				case 1:
					description: physics.Contact_2_One_Body = {c0, c1, {0, 1, 0}, material};
					status = fixed_constraint(s, &connected, &description, 2, carried);
				case 2:
					description: physics.Contact_3_One_Body = {c0, c1, c2, {0, 1, 0}, material};
					status = fixed_constraint(s, &connected, &description, 3, carried);
				case 3:
					description: physics.Contact_4_One_Body = {c0, c1, c2, c3, {0, 1, 0}, material};
					status = fixed_constraint(s, &connected, &description, 4, carried);
				}
			}
			else
			{
				switch i%4
				{
				case 0:
					description: physics.Contact_1 = {c0, offset_b, {0, 1, 0}, material};
					status = fixed_constraint(s, &connected, &description, 1, carried);
				case 1:
					description: physics.Contact_2 = {c0, c1, offset_b, {0, 1, 0}, material};
					status = fixed_constraint(s, &connected, &description, 2, carried);
				case 2:
					description: physics.Contact_3 = {c0, c1, c2, offset_b, {0, 1, 0}, material};
					status = fixed_constraint(s, &connected, &description, 3, carried);
				case 3:
					description: physics.Contact_4 = {c0, c1, c2, c3, offset_b, {0, 1, 0}, material};
					status = fixed_constraint(s, &connected, &description, 4, carried);
				}
			}
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

fixed_observe_contacts :: proc(s: ^Fixed_State, q: ^Fixed_Quality)
{
	result: contacts.Quality_Result = {expected_rows=s.points};
	active: ^physics.Body_Set = &s.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for canonical in s.canonical
	{
		kind: ^physics.Type_Batch = canonical.native;
		for i in 0 ..< kind.count
		{
			lane: int = int(i)%8;
			references: [^]util.I32x8 = physics.type_batch_body_bundle(kind, int(i));
			if s.options.case_name != .Nonconvex
			{
				contact: contacts.Contact_Input = contacts.contact_read(kind, int(i));
				m: contacts.Quality_Manifold = {
					body_count=int(kind.body_count), contact_count=int(kind.type_id%4+1),
					normal={f64(contact.normal.x), f64(contact.normal.y), f64(contact.normal.z)},
					offset_b={f64(contact.offset_b.x), f64(contact.offset_b.y), f64(contact.offset_b.z)},
					tangent_impulse={f64(contact.tangent_impulse.x), f64(contact.tangent_impulse.y)},
					twist_impulse=f64(contact.twist_impulse), friction=f64(contact.material.friction_coefficient),
					angular_frequency=f64(contact.material.spring_settings.angular_frequency),
					twice_damping_ratio=f64(contact.material.spring_settings.twice_damping_ratio),
					maximum_recovery_velocity=f64(contact.material.maximum_recovery_velocity), dt=1.0/(60*f64(s.options.substeps)),
				};
				for body in 0 ..< m.body_count
				{
					index: int = int(u32(contact.bodies[body]) & physics.BODY_REFERENCE_INDEX_MASK);
					m.bodies[body] = contacts.quality_body(&active.dynamics_state.memory[index]);
				}
				for point in 0 ..< m.contact_count
				{
					offset: util.Vector3 = contact.offsets[point];
					m.offsets[point] = {f64(offset.x), f64(offset.y), f64(offset.z)};
					m.depths[point] = f64(contact.depths[point]);
					m.normal_impulses[point] = f64(contact.normal_impulses[point]);
				}
				contacts.quality_manifold_observe(&m, &result, s.scratch);
			}
			else
			{
				prestep: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_prestep_bundle(kind, int(i)));
				impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(kind, int(i)));
				point_count: int = int(kind.type_id-physics.CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID+2);
				cursor: int = 4;
				offset_b: [3]f64;
				if kind.body_count == 2
				{
					point_count = int(kind.type_id-physics.CONTACT_2_NONCONVEX_TYPE_ID+2);
					for axis in 0 ..< 3
					{
						offset_b[axis] = f64(simd.extract(prestep[4+axis], lane));
					}
					cursor += 3;
				}
				for point in 0 ..< point_count
				{
					m: contacts.Quality_Manifold = {
						body_count=int(kind.body_count), contact_count=1, offset_b=offset_b,
						friction=f64(simd.extract(prestep[0], lane)),
						angular_frequency=f64(simd.extract(prestep[1], lane)),
						twice_damping_ratio=f64(simd.extract(prestep[2], lane)),
						maximum_recovery_velocity=f64(simd.extract(prestep[3], lane)),
						dt=1.0/(60*f64(s.options.substeps)),
						tangent_impulse={f64(simd.extract(impulses[3*point], lane)), f64(simd.extract(impulses[3*point+1], lane))},
					};
					for body in 0 ..< m.body_count
					{
						index: int = int(u32(simd.extract(references[body], lane)) & physics.BODY_REFERENCE_INDEX_MASK);
						m.bodies[body] = contacts.quality_body(&active.dynamics_state.memory[index]);
					}
					for axis in 0 ..< 3
					{
						m.offsets[0][axis] = f64(simd.extract(prestep[cursor+7*point+axis], lane));
						m.normal[axis] = f64(simd.extract(prestep[cursor+7*point+3+axis], lane));
					}
					m.depths[0] = f64(simd.extract(prestep[cursor+7*point+6], lane));
					m.normal_impulses[0] = f64(simd.extract(impulses[3*point+2], lane));
					contacts.quality_manifold_observe(&m, &result, s.scratch);
				}
			}
		}
	}
	contacts.quality_finish(&result, s.scratch);
	q.observed_units = result.observed_rows;
	q.failures += result.finite_failures+result.invalid_coefficients+result.feasibility_failures;
	if result.observed_rows != result.expected_rows || result.residual_rows != result.expected_rows
	{
		q.failures += 1;
	}
	q.residual_max_mps = result.residual_max_mps;
	q.residual_rms_mps = result.residual_rms_mps;
	q.residual_p95_mps = result.residual_p95_mps;
}
