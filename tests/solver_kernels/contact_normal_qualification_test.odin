package solver_kernel_tests
import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
contact_oracle_dot :: proc(a, b: [3]f64) -> f64
{
	return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
contact_oracle_cross :: proc(a, b: [3]f64) -> [3]f64
{
	return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
contact_oracle_read :: proc(v: util.Vector3_Wide, lane: int) -> [3]f64
{
	return {f64(simd.extract(v.x, lane)), f64(simd.extract(v.y, lane)), f64(simd.extract(v.z, lane))};
}
contact_oracle_inertia :: proc(v: [3]f64, tensor: util.Symmetric3x3_Wide, lane: int) -> [3]f64
{
	xx, yx, yy, zx, zy, zz: f64;
	xx=f64(simd.extract(tensor.xx, lane));
	yx=f64(simd.extract(tensor.yx, lane));
	yy=f64(simd.extract(tensor.yy, lane));
	zx=f64(simd.extract(tensor.zx, lane));
	zy=f64(simd.extract(tensor.zy, lane));
	zz=f64(simd.extract(tensor.zz, lane));
	return {xx*v[0]+yx*v[1]+zx*v[2], yx*v[0]+yy*v[1]+zy*v[2], zx*v[0]+zy*v[1]+zz*v[2]};
}
// independent pivoted f64 elimination and active-set enumeration
contact_oracle_lcp :: proc(a: [4][4]f64, rhs: [4]f64, count: int) -> ([4]f64, physics.Reference_State)
{
	for subset: int = 0; subset < (1 << uint(count)); subset += 1
	{
		indices: [4]int;
		active_count: int;
		for i: int = 0; i < count; i += 1
		{
			if subset & (1 << uint(i)) != 0
			{
				indices[active_count]=i;
				active_count+=1;
			}
		}
		system: [4][5]f64;
		for row: int = 0; row < active_count; row += 1
		{
			for col: int = 0; col < active_count; col += 1
			{
				system[row][col]=a[indices[row]][indices[col]];
			}
			system[row][active_count]=-rhs[indices[row]];
		}
		validity: physics.Reference_State = .Present;
		for col: int = 0; col < active_count; col += 1
		{
			pivot: int=col;
			for row: int = col+1; row < active_count; row += 1
			{
				if math.abs(system[row][col]) > math.abs(system[pivot][col])
				{
					pivot=row;
				}
			}
			if math.abs(system[pivot][col]) < 1e-20
			{
				validity=.Missing;
				break;
			}
			system[col], system[pivot]=system[pivot], system[col];
			inverse: f64=1/system[col][col];
			for k: int = col; k <= active_count; k += 1
			{
				system[col][k]*=inverse;
			}
			for row: int = 0; row < active_count; row += 1
			{
				if row == col
				{
					continue;
				}
				factor: f64=system[row][col];
				for k: int = col; k <= active_count; k += 1
				{
					system[row][k]-=factor*system[col][k];
				}
			}
		}
		if validity == .Missing
		{
			continue;
		}
		result: [4]f64;
		for row: int = 0; row < active_count; row += 1
		{
			result[indices[row]]=system[row][active_count];
		}
		for i: int = 0; i < count; i += 1
		{
			residual: f64=rhs[i];
			for j: int = 0; j < count; j += 1
			{
				residual+=a[i][j]*result[j];
			}
			if result[i] < -1e-10 || residual < -1e-9 || math.abs(result[i]*residual)>1e-8
			{
				validity=.Missing;
			}
		}
		if validity == .Present
		{
			return result, .Present;
		}
	}
	return {}, .Missing;
}
contact_normal_double_case :: proc(t: ^testing.T, $body_count, $contact_count: int, $response: physics.Solver_Response_Mode)
{
	for ownership: int = 0; ownership < 256; ownership += 1
	{
		bodies: [2]physics.Constraint_Contact_Body_Wide;
		before: [2]physics.Constraint_Contact_Body_Wide;
		prestep: [40]util.F32x8;
		impulses: [12]util.F32x8;
		targets: [4]physics.Restitution_Target_Wide;
		contacts: [^]physics.Convex_Contact_Wide=([^]physics.Convex_Contact_Wide)(&prestep[0]);
		normal_index: int=contact_count*4+(body_count-1)*3;
		prestep[normal_index+1]=util.F32x8(1);
		when body_count == 2
		{
			prestep[contact_count*4+1]=util.F32x8(1);
		}
		material: ^physics.Contact_Material_Wide=(^physics.Contact_Material_Wide)(&prestep[normal_index+3]);
		material.spring_settings={angular_frequency=util.F32x8(30), twice_damping_ratio=util.F32x8(2)};
		material.maximum_recovery_velocity=util.F32x8(2);
		positions: [4][2]f32={{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}};
		active: util.I32x8;
		for lane: int = 0; lane < 8; lane += 1
		{
			if ownership & (1 << uint(lane)) != 0
			{
				active=simd.replace(active, lane, -1);
			}
			for role: int = 0; role < body_count; role += 1
			{
				body: ^physics.Constraint_Contact_Body_Wide=&bodies[role];
				mass: f32=0.5+f32((lane+role)%4)*0.25;
				if role == 1 && lane == 6
				{
					mass=0;
				}
				if role == 1 && lane == 7
				{
					mass=8;
				}
				body.inverse_mass=simd.replace(body.inverse_mass, lane, mass);
				body.inverse_inertia.xx=simd.replace(body.inverse_inertia.xx, lane, 0.75+mass);
				body.inverse_inertia.yy=simd.replace(body.inverse_inertia.yy, lane, 1+mass);
				body.inverse_inertia.zz=simd.replace(body.inverse_inertia.zz, lane, 1.25+mass);
				body.inverse_inertia.yx=simd.replace(body.inverse_inertia.yx, lane, 0.01);
				body.inverse_inertia.zx=simd.replace(body.inverse_inertia.zx, lane, -0.02);
				body.inverse_inertia.zy=simd.replace(body.inverse_inertia.zy, lane, 0.015);
				body.linear_velocity.y=simd.replace(body.linear_velocity.y, lane, -0.3-f32(lane)*0.125+f32(role)*0.2);
				body.angular_velocity.x=simd.replace(body.angular_velocity.x, lane, f32(lane-3)*0.03);
				body.angular_velocity.z=simd.replace(body.angular_velocity.z, lane, f32(role)*0.1-0.075);
			}
			for i: int = 0; i < contact_count; i += 1
			{
				contacts[i].offset_a.x=simd.replace(contacts[i].offset_a.x, lane, positions[i][0]);
				contacts[i].offset_a.z=simd.replace(contacts[i].offset_a.z, lane, positions[i][1]);
				depth: f32=0.01;
				if lane == 5
				{
					depth=-0.01;
				}
				contacts[i].depth=simd.replace(contacts[i].depth, lane, depth);
				impulses[2+i]=simd.replace(impulses[2+i], lane, 0.025*f32(i+1));
				if (lane+i)%2 == 0
				{
					targets[i].active_mask=simd.replace(targets[i].active_mask, lane, simd.extract(active, lane));
					targets[i].velocity=simd.replace(targets[i].velocity, lane, 0.8);
				}
			}
		}
		before=bodies;
		carried: [12]util.F32x8=impulses;
		physics.constraint_contact_convex_kernel_response(&prestep[0], &bodies[0], 1.0/60, 60, &impulses[0], active, .Solve, body_count, contact_count, &targets[0], response, .Serial);
		for lane: int = 0; lane < 8; lane += 1
		{
			if simd.extract(active, lane)==0
			{
				for role: int = 0; role < body_count; role += 1
				{
					a: [6]f32={simd.extract(before[role].linear_velocity.x, lane), simd.extract(before[role].linear_velocity.y, lane), simd.extract(before[role].linear_velocity.z, lane), simd.extract(before[role].angular_velocity.x, lane), simd.extract(before[role].angular_velocity.y, lane), simd.extract(before[role].angular_velocity.z, lane)};
					b: [6]f32={simd.extract(bodies[role].linear_velocity.x, lane), simd.extract(bodies[role].linear_velocity.y, lane), simd.extract(bodies[role].linear_velocity.z, lane), simd.extract(bodies[role].angular_velocity.x, lane), simd.extract(bodies[role].angular_velocity.y, lane), simd.extract(bodies[role].angular_velocity.z, lane)};
					for k: int = 0; k < 6; k += 1
					{
						testing.expect_value(t, transmute(u32)a[k], transmute(u32)b[k]);
					}
				}
				continue;
			}
			w: [4][4]f64;
			csv, bias, gamma, lambda, old: [4]f64;
			jacobian: [4][3]f64;
			normal: [3]f64={0, 1, 0};
			x: f64=f64(f32(1.0/60))*30;
			extra: f64=1/(x*(x+2));
			mass_scale: f64=1/(1+extra);
			softness: f64=extra*mass_scale;
			position_scale: f64=30/(x+2);
			for i: int = 0; i < contact_count; i += 1
			{
				jacobian[i]={-f64(positions[i][1]), 0, f64(positions[i][0])};
				old[i]=f64(simd.extract(carried[2+i], lane));
				lambda[i]=old[i];
				depth: f64=f64(simd.extract(contacts[i].depth, lane));
				bias[i]=min(depth*60, min(depth*position_scale, 2));
				for role: int = 0; role < body_count; role += 1
				{
					sign: f64=1;
					if role==1
					{
						sign=-1;
					}
					csv[i]+=sign*(contact_oracle_dot(contact_oracle_read(before[role].linear_velocity, lane), normal)+contact_oracle_dot(contact_oracle_read(before[role].angular_velocity, lane), jacobian[i]));
					for j: int = 0; j < contact_count; j += 1
					{
						w[i][j]+=f64(simd.extract(before[role].inverse_mass, lane))+contact_oracle_dot(jacobian[i], contact_oracle_inertia({-f64(positions[j][1]), 0, f64(positions[j][0])}, before[role].inverse_inertia, lane));
					}
				}
				gamma[i]=w[i][i]*softness/mass_scale;
				when response == .Restitution
				{
					if simd.extract(targets[i].active_mask, lane)!=0
					{
						bias[i]=max(bias[i], f64(simd.extract(targets[i].velocity, lane)));
						gamma[i]=0;
					}
				}
			}
			when response == .Default
			{
				for start: int = 0; start+1 < contact_count; start+=2
				{
					system: [4][4]f64;
					rhs: [4]f64;
					for i: int = 0; i < 2; i+=1
					{
						rhs[i]=csv[start+i]-bias[start+i];
						for j: int = 0; j < 2; j+=1
						{
							system[i][j]=w[start+i][start+j];
							rhs[i]-=system[i][j]*lambda[start+j];
						}
						system[i][i]+=gamma[start+i];
					}
					result: [4]f64;
					status: physics.Reference_State;
					result, status=contact_oracle_lcp(system, rhs, 2);
					testing.expect_value(t, status, physics.Reference_State.Present);
					for i: int = 0; i < contact_count; i+=1
					{
						csv[i]+=w[i][start]*(result[0]-lambda[start])+w[i][start+1]*(result[1]-lambda[start+1]);
					}
					lambda[start]=result[0];
					lambda[start+1]=result[1];
				}
				if contact_count%2!=0
				{
					i: int=contact_count-1;
					lambda[i]=max(0, lambda[i]-(csv[i]-bias[i]+gamma[i]*lambda[i])/(w[i][i]+gamma[i]));
				}
			}
			else
			{
				for i: int = 0; i < contact_count; i+=1
				{
					next: f64=max(0, lambda[i]-(csv[i]-bias[i]+gamma[i]*lambda[i])/(w[i][i]+gamma[i]));
					delta: f64=next-lambda[i];
					lambda[i]=next;
					for j: int = 0; j < contact_count; j+=1
					{
						csv[j]+=w[j][i]*delta;
					}
				}
			}
			for i: int = 0; i < contact_count; i+=1
			{
				testing.expectf(t, math.abs(f64(simd.extract(impulses[2+i], lane))-lambda[i])<2e-5*(1+math.abs(lambda[i])), "response=%v bodies=%d contacts=%d mask=%d lane=%d point=%d expected=%g got=%g", response, body_count, contact_count, ownership, lane, i, lambda[i], simd.extract(impulses[2+i], lane));
			}
			for role: int = 0; role < body_count; role+=1
			{
				linear: [3]f64=contact_oracle_read(before[role].linear_velocity, lane);
				angular: [3]f64=contact_oracle_read(before[role].angular_velocity, lane);
				sign: f64=1;
				if role==1
				{
					sign=-1;
				}
				for i: int = 0; i < contact_count; i+=1
				{
					delta: f64=sign*(lambda[i]-old[i]);
					torque: [3]f64=contact_oracle_inertia(jacobian[i], before[role].inverse_inertia, lane);
					linear[1]+=delta*f64(simd.extract(before[role].inverse_mass, lane));
					for k: int = 0; k < 3; k+=1
					{
						angular[k]+=delta*torque[k];
					}
				}
				got_l: [3]f64=contact_oracle_read(bodies[role].linear_velocity, lane);
				got_a: [3]f64=contact_oracle_read(bodies[role].angular_velocity, lane);
				for k: int = 0; k < 3; k+=1
				{
					testing.expect(t, math.abs(linear[k]-got_l[k])<2e-5*(1+math.abs(linear[k])));
					testing.expect(t, math.abs(angular[k]-got_a[k])<2e-5*(1+math.abs(angular[k])));
				}
			}
		}
	}
}
@(test)
convex_normal_solver_matches_independent_double_oracle :: proc(t: ^testing.T)
{
	contact_normal_double_case(t, 1, 2, .Default);
	contact_normal_double_case(t, 1, 3, .Default);
	contact_normal_double_case(t, 1, 4, .Default);
	contact_normal_double_case(t, 2, 2, .Default);
	contact_normal_double_case(t, 2, 3, .Default);
	contact_normal_double_case(t, 2, 4, .Default);
	contact_normal_double_case(t, 1, 2, .Restitution);
	contact_normal_double_case(t, 1, 3, .Restitution);
	contact_normal_double_case(t, 1, 4, .Restitution);
	contact_normal_double_case(t, 2, 2, .Restitution);
	contact_normal_double_case(t, 2, 3, .Restitution);
	contact_normal_double_case(t, 2, 4, .Restitution);
}
contact_normal_result_matches_body_wrench :: proc(t: ^testing.T, record: ^physics.Constraint_Type_Record, result: ^Kernel_Run_Result, lane: int)
{
	count: int=int(record.impulse_bundle_size)/size_of(util.F32x8)-3;
	contacts: [^]physics.Convex_Contact_Wide=([^]physics.Convex_Contact_Wide)(&result.prestep.bytes[0]);
	cursor: uintptr=uintptr(&result.prestep.bytes[0])+uintptr(count*size_of(physics.Convex_Contact_Wide));
	offset: util.Vector3_Wide;
	if record.body_count==2
	{
		offset=(^util.Vector3_Wide)(rawptr(cursor))^;
		cursor+=uintptr(size_of(util.Vector3_Wide));
	}
	normal_wide: util.Vector3_Wide=(^util.Vector3_Wide)(rawptr(cursor))^;
	tangent_x, tangent_y: util.Vector3_Wide;
	tangent_x, tangent_y=physics.constraint_kernel_build_orthonormal_basis(normal_wide);
	normal: [3]f64=contact_oracle_read(normal_wide, lane);
	tx: [3]f64=contact_oracle_read(tangent_x, lane);
	ty: [3]f64=contact_oracle_read(tangent_y, lane);
	offset_b: [3]f64=contact_oracle_read(offset, lane);
	old: [^]util.F32x8=([^]util.F32x8)(&result.warmstarted_impulses.bytes[0]);
	next: [^]util.F32x8=([^]util.F32x8)(&result.impulses.bytes[0]);
	delta_x: f64=f64(simd.extract(next[0], lane))-f64(simd.extract(old[0], lane));
	delta_y: f64=f64(simd.extract(next[1], lane))-f64(simd.extract(old[1], lane));
	delta_twist: f64=f64(simd.extract(next[count+2], lane))-f64(simd.extract(old[count+2], lane));
	center: [3]f64;
	selected_count: int;
	for i: int=0;i<count;i+=1
	{
		if simd.extract(contacts[i].depth, lane)>=0
		{
			p: [3]f64=contact_oracle_read(contacts[i].offset_a, lane);
			center+=p;
			selected_count+=1;
		}
	}
	if selected_count==0
	{
		for i: int=0;i<count;i+=1
		{
			center+=contact_oracle_read(contacts[i].offset_a, lane);
		}
		selected_count=count;
	}
	center/=f64(selected_count);
	friction: [3]f64=tx*delta_x+ty*delta_y;
	linear: [3]f64=friction;
	angular_a: [3]f64=contact_oracle_cross(center, friction)+normal*delta_twist;
	for i: int=0;i<count;i+=1
	{
		delta: f64=f64(simd.extract(next[i+2], lane))-f64(simd.extract(old[i+2], lane));
		p: [3]f64=contact_oracle_read(contacts[i].offset_a, lane);
		linear+=normal*delta;
		angular_a+=contact_oracle_cross(p, normal)*delta;
	}
	for role: int=0;role<int(record.body_count);role+=1
	{
		before: ^physics.Constraint_Kernel_Body_Wide=&result.warmstarted_bodies[role];
		after: ^physics.Constraint_Kernel_Body_Wide=&result.bodies[role];
		torque: [3]f64=angular_a;
		sign: f64=1;
		if role==1
		{
			torque=contact_oracle_cross(offset_b, linear)-angular_a;
			sign=-1;
		}
		expected_linear: [3]f64=contact_oracle_read(before.linear_velocity, lane)+linear*(sign*f64(simd.extract(before.inverse_mass, lane)));
		expected_angular: [3]f64=contact_oracle_read(before.angular_velocity, lane)+contact_oracle_inertia(torque, before.inverse_inertia, lane);
		actual_linear: [3]f64=contact_oracle_read(after.linear_velocity, lane);
		actual_angular: [3]f64=contact_oracle_read(after.angular_velocity, lane);
		for k: int=0;k<3;k+=1
		{
			testing.expect(t, math.abs(expected_linear[k]-actual_linear[k])<2e-5*(1+math.abs(expected_linear[k])));
			testing.expect(t, math.abs(expected_angular[k]-actual_angular[k])<2e-5*(1+math.abs(expected_angular[k])));
		}
	}
}

contact_singular_manifold_case :: proc(t: ^testing.T, $body_count, $contact_count: int)
{
	for ownership: int=0;ownership<256;ownership+=1
	{
		prestep: [40]util.F32x8;
		normal_index: int=contact_count*4+(body_count-1)*3;
		prestep[normal_index+1]=util.F32x8(1);
		material: ^physics.Contact_Material_Wide=(^physics.Contact_Material_Wide)(&prestep[normal_index+3]);
		material.spring_settings={angular_frequency=util.F32x8(1e30)};
		material.maximum_recovery_velocity=util.F32x8(2);
		contacts: [^]physics.Convex_Contact_Wide=([^]physics.Convex_Contact_Wide)(&prestep[0]);
		for point: int=0;point<contact_count;point+=1
		{
			contacts[point].depth=util.F32x8(0.01);
		}
		active: util.I32x8;
		for lane: int=0;lane<8;lane+=1
		{
			if ownership & (1 << uint(lane))!=0
			{
				active=simd.replace(active, lane, -1);
			}
		}
		actual, expected: [2]physics.Constraint_Contact_Body_Wide;
		actual[0].inverse_mass=util.F32x8(1);
		actual[0].linear_velocity.y=util.F32x8(-2);
		when body_count==2
		{
			actual[1].inverse_mass=util.F32x8(1);
		}
		expected=actual;
		impulses, reference: [12]util.F32x8;
		normal: util.Vector3_Wide={y=util.F32x8(1)};
		for point: int=0;point<contact_count;point+=1
		{
			physics.constraint_contact_penetration_solve(&expected[0], normal, {}, {}, util.F32x8(0.01), util.F32x8(60), util.F32x8(1), util.F32x8(2), util.F32x8(0), 1.0/60, 60, &reference[point+2], active, body_count);
		}
		physics.constraint_contact_convex_kernel(&prestep[0], &actual[0], 1.0/60, 60, &impulses[0], active, .Solve, body_count, contact_count);
		for role: int=0;role<body_count;role+=1
		{
			for lane: int=0;lane<8;lane+=1
			{
				a: [3]f64=contact_oracle_read(actual[role].linear_velocity, lane);
				b: [3]f64=contact_oracle_read(expected[role].linear_velocity, lane);
				for k: int=0;k<3;k+=1
				{
					testing.expect(t, math.abs(a[k]-b[k])<2e-5);
				}
				testing.expect_value(t, actual[role].angular_velocity, expected[role].angular_velocity);
			}
		}
		for lane: int=0;lane<8;lane+=1
		{
			sum, reference_sum: f64;
			for point: int=0;point<contact_count;point+=1
			{
				sum+=f64(simd.extract(impulses[point+2], lane));
				reference_sum+=f64(simd.extract(reference[point+2], lane));
				testing.expect(t, simd.extract(impulses[point+2], lane)>=0);
				testing.expect_value(t, simd.extract(impulses[point+2], lane), simd.extract(reference[point+2], lane));
			}
			testing.expect(t, math.abs(sum-reference_sum)<2e-5);
		}
	}
}
@(test)
convex_singular_manifold_preserves_parent_fallback_and_body_wrench :: proc(t: ^testing.T)
{
	contact_singular_manifold_case(t, 1, 2);
	contact_singular_manifold_case(t, 1, 3);
	contact_singular_manifold_case(t, 1, 4);
	contact_singular_manifold_case(t, 2, 2);
	contact_singular_manifold_case(t, 2, 3);
	contact_singular_manifold_case(t, 2, 4);
}

normal_aggregate_range_case :: proc(t: ^testing.T, $body_count: int, $pair_mode: physics.Contact_Normal_Pair_Mode)
{
	CONTACTS :: 2;
	// exact powers of two overflow the unscaled aggregate while each ordered row stays finite
	mass: f32 = 5.421010862427522e-20; // 2^-64
	lambda: f32 = 1.7014118346046923e38; // 2^127
	initial_speed: f32 = 18446744073709551616.0; // 2^64
	actual: [4]physics.Constraint_Kernel_Body_Wide;
	contacts: [CONTACTS]physics.Convex_Contact_Wide;
	impulses, reference_impulses: [CONTACTS]util.F32x8;
	normal: util.Vector3_Wide = {y = util.F32x8(1)};
	for role: int = 0; role < body_count; role += 1
	{
		actual[role].inverse_mass = util.F32x8(mass / f32(body_count));
		actual[role].inverse_inertia.zz = util.F32x8(mass / f32(body_count));
		actual[role].linear_velocity.y = util.F32x8(initial_speed / f32(body_count));
	}
	when body_count == 2
	{
		actual[1].linear_velocity.y = simd.neg(actual[1].linear_velocity.y);
	}
	for point: int = 0; point < CONTACTS; point += 1
	{
		impulses[point] = util.F32x8(lambda);
		contacts[point].offset_a.x = util.F32x8(f32(1 - point*2));
	}
	reference: [4]physics.Constraint_Kernel_Body_Wide = actual;
	reference_impulses = impulses;
	active: util.I32x8 = {-1, 0, -1, -1, 0, -1, 0, -1};
	for point: int = 0; point < CONTACTS; point += 1
	{
		physics.constraint_contact_penetration_solve(&reference[0], normal, contacts[point].offset_a, contacts[point].offset_a,
			util.F32x8(0), util.F32x8(0), util.F32x8(1), util.F32x8(0), util.F32x8(0),
			0, 1, &reference_impulses[point], active, body_count);
	}
	physics.contact_normal_pair_solve_fast(&actual[0], normal, {}, contacts[0], contacts[1],
		util.F32x8(0), util.F32x8(1), util.F32x8(0), util.F32x8(0), 1, &impulses[0], &impulses[1], active, body_count, pair_mode);
	for role: int = 0; role < body_count; role += 1
	{
		for lane: int = 0; lane < 8; lane += 1
		{
			expected_linear: [3]f64 = contact_oracle_read(reference[role].linear_velocity, lane);
			expected_angular: [3]f64 = contact_oracle_read(reference[role].angular_velocity, lane);
			for axis: int = 0; axis < 3; axis += 1
			{
				testing.expect(t, math.abs(expected_linear[axis]) <= 3.4028234663852886e38);
				testing.expect(t, math.abs(expected_angular[axis]) <= 3.4028234663852886e38);
			}
		}
		testing.expect_value(t, actual[role].linear_velocity, reference[role].linear_velocity);
		testing.expect_value(t, actual[role].angular_velocity, reference[role].angular_velocity);
	}
	for point: int = 0; point < CONTACTS; point += 1
	{
		testing.expect_value(t, impulses[point], reference_impulses[point]);
	}
}
@(test)
normal_aggregate_finite_range_preserves_ordered_parent :: proc(t: ^testing.T)
{
	normal_aggregate_range_case(t, 1, .Serial);
	normal_aggregate_range_case(t, 2, .Serial);
	normal_aggregate_range_case(t, 1, .Parallel);
	normal_aggregate_range_case(t, 2, .Parallel);
}

