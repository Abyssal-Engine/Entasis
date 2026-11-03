package solver_kernel_tests
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
contact_warmstart_range_case :: proc(t: ^testing.T, $body_count: int)
{
	for ownership: int=0;ownership<256;ownership+=1
	{
		prestep: [40]util.F32x8;
		contacts: [^]physics.Convex_Contact_Wide=([^]physics.Convex_Contact_Wide)(&prestep[0]);
		normal_index: int=16+(body_count-1)*3;
		prestep[normal_index+1]=util.F32x8(1);
		impulse: [12]util.F32x8;
		active: util.I32x8;
		actual, expected: [2]physics.Constraint_Contact_Body_Wide;
		normal: util.Vector3_Wide={y=util.F32x8(1)};
		tangent_x, tangent_y: util.Vector3_Wide;
		tangent_x, tangent_y=physics.constraint_kernel_build_orthonormal_basis(normal);
		for lane: int=0;lane<8;lane+=1
		{
			if ownership & (1 << uint(lane))!=0
			{
				active=simd.replace(active, lane, -1);
			}
			mass: f32=transmute(f32)u32(0x1f800000);
			magnitude: f32=transmute(f32)u32(0x7e800000);
			when body_count==2
			{
				mass=1;
				magnitude=transmute(f32)u32(0x7d800000);
				actual[1].inverse_mass=simd.replace(actual[1].inverse_mass, lane, 4);
				actual[1].linear_velocity.y=simd.replace(actual[1].linear_velocity.y, lane, transmute(f32)u32(0x7f000000));
			}
			actual[0].inverse_mass=simd.replace(actual[0].inverse_mass, lane, mass);
			for point: int=0;point<4;point+=1
			{
				impulse[point+2]=simd.replace(impulse[point+2], lane, magnitude);
			}
		}
		expected=actual;
		proposed: [2]physics.Constraint_Contact_Body_Wide=actual;
		application: physics.Contact_Warmstart_Application=physics.contact_convex_warmstart_fast(
			&proposed[0], contacts, normal, {}, tangent_x, tangent_y, {}, {}, &impulse[2], util.F32x8(0), active, body_count, 4);
		if ownership==0
		{
			testing.expect_value(t, application, physics.Contact_Warmstart_Application.Applied);
		}
		else
		{
			testing.expect_value(t, application, physics.Contact_Warmstart_Application.Sequential_Required);
		}
		for role: int=0;role<body_count;role+=1
		{
			testing.expect_value(t, proposed[role].linear_velocity, actual[role].linear_velocity);
			testing.expect_value(t, proposed[role].angular_velocity, actual[role].angular_velocity);
		}
		physics.constraint_contact_tangent_apply(&expected[0], tangent_x, tangent_y, {}, {}, {}, active, body_count);
		for point: int=0;point<4;point+=1
		{
			physics.constraint_contact_penetration_apply(&expected[0], normal, {}, {}, impulse[point+2], active, body_count);
		}
		physics.constraint_contact_twist_apply(&expected[0], normal, util.F32x8(0), active, body_count);
		physics.constraint_contact_convex_kernel(&prestep[0], &actual[0], 1.0/60, 60, &impulse[0], active, .Warmstart, body_count, 4);
		for role: int=0;role<body_count;role+=1
		{
			for lane: int=0;lane<8;lane+=1
			{
				a: [6]f32={simd.extract(actual[role].linear_velocity.x, lane), simd.extract(actual[role].linear_velocity.y, lane), simd.extract(actual[role].linear_velocity.z, lane), simd.extract(actual[role].angular_velocity.x, lane), simd.extract(actual[role].angular_velocity.y, lane), simd.extract(actual[role].angular_velocity.z, lane)};
				b: [6]f32={simd.extract(expected[role].linear_velocity.x, lane), simd.extract(expected[role].linear_velocity.y, lane), simd.extract(expected[role].linear_velocity.z, lane), simd.extract(expected[role].angular_velocity.x, lane), simd.extract(expected[role].angular_velocity.y, lane), simd.extract(expected[role].angular_velocity.z, lane)};
				for k: int=0;k<6;k+=1
				{
					testing.expect_value(t, transmute(u32)a[k], transmute(u32)b[k]);
				}
			}
		}
	}
}
@(test)
aggregated_warmstart_falls_back_before_either_body_commits :: proc(t: ^testing.T)
{
	contact_warmstart_range_case(t, 1);
	contact_warmstart_range_case(t, 2);
}
contact_warmstart_wrench_case :: proc(t: ^testing.T, registry: ^physics.Constraint_Type_Registry, description: ^$T)
{
	for count: int=1;count<=8;count+=1
	{
		result: Kernel_Run_Result;
		record: ^physics.Constraint_Type_Record=kernel_initialize(t, registry, description, count, &result);
		record.prestep_warmstart_solve(&result.prestep.bytes[0], &result.bodies, 1.0/64, 64, &result.impulses.bytes[0], lane_mask(count), .Prestep);
		result.warmstarted_bodies=result.bodies;
		record.prestep_warmstart_solve(&result.prestep.bytes[0], &result.bodies, 1.0/64, 64, &result.impulses.bytes[0], lane_mask(count), .Warmstart);
		for lane: int=0;lane<count;lane+=1
		{
			contact_normal_result_matches_body_wrench(t, record, &result, lane);
		}
	}
}
@(test)
aggregated_warmstart_matches_independent_double_wrench :: proc(t: ^testing.T)
{
	registry: physics.Constraint_Type_Registry=registry_fixture(t);
	c0, c1, c2, c3: physics.Constraint_Contact_Data;
	c0=valid_contact(0);
	c1=valid_contact(1);
	c2=valid_contact(2);
	c3=valid_contact(3);
	material: physics.Contact_Material_Properties=valid_material();
	a1: physics.Contact_1_One_Body={c0, unit_y(), material};
	a2: physics.Contact_2_One_Body={c0, c1, unit_y(), material};
	a3: physics.Contact_3_One_Body={c0, c1, c2, unit_y(), material};
	a4: physics.Contact_4_One_Body={c0, c1, c2, c3, unit_y(), material};
	b1: physics.Contact_1={c0, {0.5, 0.25, 0.75}, unit_y(), material};
	b2: physics.Contact_2={c0, c1, {0.5, 0.25, 0.75}, unit_y(), material};
	b3: physics.Contact_3={c0, c1, c2, {0.5, 0.25, 0.75}, unit_y(), material};
	b4: physics.Contact_4={c0, c1, c2, c3, {0.5, 0.25, 0.75}, unit_y(), material};
	contact_warmstart_wrench_case(t, &registry, &a1);
	contact_warmstart_wrench_case(t, &registry, &a2);
	contact_warmstart_wrench_case(t, &registry, &a3);
	contact_warmstart_wrench_case(t, &registry, &a4);
	contact_warmstart_wrench_case(t, &registry, &b1);
	contact_warmstart_wrench_case(t, &registry, &b2);
	contact_warmstart_wrench_case(t, &registry, &b3);
	contact_warmstart_wrench_case(t, &registry, &b4);
}


contact_response_parent_normal :: #force_inline proc "contextless" (
bodies: [^]$Body, normal, offset_a, offset_b: util.Vector3_Wide,
depth, position_error_to_velocity, effective_mass_scale, maximum_recovery_velocity, softness: util.F32x8,
dt, inverse_dt: f32, accumulated: ^util.F32x8, active_mask: util.I32x8, $body_count: int,
)
{
	_ = dt;
	angular_a: util.Vector3_Wide = util.vector3_wide_cross(offset_a, normal);
	inverse_effective_mass: util.F32x8 = simd.add(
	bodies[0].inverse_mass,
	util.symmetric3x3_wide_vector_sandwich(angular_a, bodies[0].inverse_inertia)
	);
	csv: util.F32x8 = simd.add(
	util.vector3_wide_dot(bodies[0].linear_velocity, normal),
	util.vector3_wide_dot(bodies[0].angular_velocity, angular_a)
	);
	when body_count == 2
	{
		angular_b: util.Vector3_Wide = util.vector3_wide_cross(normal, offset_b);
		inverse_effective_mass = simd.add(
		inverse_effective_mass,
		simd.add(
		bodies[1].inverse_mass,
		util.symmetric3x3_wide_vector_sandwich(angular_b, bodies[1].inverse_inertia)
		)
		);
		csv = simd.add(
		simd.sub(csv, util.vector3_wide_dot(bodies[1].linear_velocity, normal)),
		util.vector3_wide_dot(bodies[1].angular_velocity, angular_b)
		);
	}
	effective_mass: util.F32x8 = simd.div(effective_mass_scale, inverse_effective_mass);
	bias: util.F32x8 = simd.min(
	simd.mul(depth, util.F32x8(inverse_dt)),
	simd.min(simd.mul(depth, position_error_to_velocity), maximum_recovery_velocity)
	);
	negated_csi: util.F32x8 = simd.add(simd.mul(accumulated^, softness), simd.mul(simd.sub(csv, bias), effective_mass));
	previous: util.F32x8 = accumulated^;
	accumulated^ = simd.max(util.F32x8(0), simd.sub(accumulated^, negated_csi));
	accumulated^ = physics.constraint_kernel_mask_scalar(accumulated^, active_mask);
	csi: util.F32x8 = physics.constraint_kernel_mask_scalar(simd.sub(accumulated^, previous), active_mask);
	physics.constraint_contact_penetration_apply(bodies, normal, offset_a, offset_b, csi, active_mask, body_count);
}

contact_response_parent_tangent :: #force_inline proc "contextless" (
bodies: [^]$Body, tangent_x, tangent_y, offset_a, offset_b: util.Vector3_Wide,
maximum_impulse: util.F32x8, accumulated: ^util.Vector2_Wide, active_mask: util.I32x8, $body_count: int,
)
{
	linear_jacobian: util.Matrix2x3_Wide = util.Matrix2x3_Wide{tangent_x, tangent_y};
	angular_a: util.Matrix2x3_Wide = util.Matrix2x3_Wide{
		util.vector3_wide_cross(offset_a, tangent_x),
		util.vector3_wide_cross(offset_a, tangent_y),
	};
	inverse_effective_mass: util.Symmetric2x2_Wide = util.symmetric2x2_wide_add(
	util.symmetric2x2_wide_sandwich_scale(linear_jacobian, bodies[0].inverse_mass),
	util.symmetric3x3_wide_matrix_sandwich(angular_a, bodies[0].inverse_inertia),
	);
	csv_linear: util.Vector2_Wide = util.vector2_wide_scale(
	util.matrix2x3_wide_transform_by_transpose(bodies[0].linear_velocity, linear_jacobian),
	-1
	);
	csv_angular: util.Vector2_Wide = util.vector2_wide_scale(
	util.matrix2x3_wide_transform_by_transpose(bodies[0].angular_velocity, angular_a),
	-1
	);
	when body_count == 2
	{
		angular_b: util.Matrix2x3_Wide = util.Matrix2x3_Wide{
			util.vector3_wide_cross(tangent_x, offset_b),
			util.vector3_wide_cross(tangent_y, offset_b),
		};
		inverse_effective_mass = util.symmetric2x2_wide_add(inverse_effective_mass, util.symmetric2x2_wide_add(
		util.symmetric2x2_wide_sandwich_scale(linear_jacobian, bodies[1].inverse_mass),
		util.symmetric3x3_wide_matrix_sandwich(angular_b, bodies[1].inverse_inertia),
		));
		csv_linear = util.vector2_wide_add(
		csv_linear,
		util.matrix2x3_wide_transform_by_transpose(bodies[1].linear_velocity, linear_jacobian)
		);
		csv_angular = util.vector2_wide_subtract(
		csv_angular,
		util.matrix2x3_wide_transform_by_transpose(bodies[1].angular_velocity, angular_b)
		);
	}
	csi: util.Vector2_Wide = util.symmetric2x2_wide_transform(
	util.vector2_wide_add(csv_linear, csv_angular),
	util.symmetric2x2_wide_invert(inverse_effective_mass)
	);
	previous: util.Vector2_Wide = accumulated^;
	accumulated^ = util.vector2_wide_add(accumulated^, csi);
	magnitude: util.F32x8 = util.vector2_wide_length(accumulated^);
	scale: util.F32x8 = simd.min(util.F32x8(1), simd.div(maximum_impulse, simd.max(util.F32x8(1e-16), magnitude)));
	accumulated^ = physics.constraint_kernel_mask_vector2(util.vector2_wide_scale(accumulated^, scale), active_mask);
	csi = physics.constraint_kernel_mask_vector2(util.vector2_wide_subtract(accumulated^, previous), active_mask);
	physics.constraint_contact_tangent_apply(bodies, tangent_x, tangent_y, offset_a, offset_b, csi, active_mask, body_count);
}

contact_response_parent_twist :: #force_inline proc "contextless" (
bodies: [^]$Body, normal: util.Vector3_Wide, maximum_impulse: util.F32x8,
accumulated: ^util.F32x8, active_mask: util.I32x8, $body_count: int,
)
{
	inverse_effective_mass: util.F32x8 = util.symmetric3x3_wide_vector_sandwich(normal, bodies[0].inverse_inertia);
	csv: util.F32x8 = util.vector3_wide_dot(bodies[0].angular_velocity, normal);
	when body_count == 2
	{
		inverse_effective_mass = simd.add(
		inverse_effective_mass,
		util.symmetric3x3_wide_vector_sandwich(normal, bodies[1].inverse_inertia)
		);
		csv = simd.sub(csv, util.vector3_wide_dot(bodies[1].angular_velocity, normal));
	}
	effective_mass: util.F32x8 = util.wide_select_f32(
	transmute(util.I32x8)simd.lanes_eq(inverse_effective_mass, util.F32x8(0)),
	util.F32x8(0),
	simd.div(util.F32x8(1), inverse_effective_mass)
	);
	previous: util.F32x8 = accumulated^;
	accumulated^ = simd.max(
	simd.neg(maximum_impulse),
	simd.min(maximum_impulse, simd.sub(accumulated^, simd.mul(csv, effective_mass)))
	);
	accumulated^ = physics.constraint_kernel_mask_scalar(accumulated^, active_mask);
	csi: util.F32x8 = physics.constraint_kernel_mask_scalar(simd.sub(accumulated^, previous), active_mask);
	physics.constraint_contact_twist_apply(bodies, normal, csi, active_mask, body_count);
}

contact_response_parent_solve :: proc "contextless" (
prestep_raw: rawptr, bodies: [^]$Body, dt, inverse_dt: f32,
impulses_raw: rawptr, active_mask: util.I32x8, $body_count, $contact_count: int,
)
{

	contacts: [^]physics.Convex_Contact_Wide = ([^]physics.Convex_Contact_Wide)(prestep_raw);
	cursor: uintptr = uintptr(prestep_raw) + uintptr(contact_count * size_of(physics.Convex_Contact_Wide));
	offset_b: util.Vector3_Wide = util.Vector3_Wide{};
	when body_count == 2
	{
		offset_b = (^util.Vector3_Wide)(rawptr(cursor))^;
		cursor += uintptr(size_of(util.Vector3_Wide));
	}
	normal: util.Vector3_Wide = (^util.Vector3_Wide)(rawptr(cursor))^;
	cursor += uintptr(size_of(util.Vector3_Wide));
	material: ^physics.Contact_Material_Wide = (^physics.Contact_Material_Wide)(rawptr(cursor));
	tangent: ^util.Vector2_Wide = (^util.Vector2_Wide)(impulses_raw);
	penetrations: [^]util.F32x8 = ([^]util.F32x8)(rawptr(uintptr(impulses_raw) + uintptr(2 * size_of(util.F32x8))));
	twist: ^util.F32x8 = (^util.F32x8)(rawptr(uintptr(impulses_raw) + uintptr((2 + contact_count) * size_of(util.F32x8))));
	tangent_x, tangent_y: util.Vector3_Wide;
	tangent_x, tangent_y = physics.constraint_kernel_build_orthonormal_basis(normal);
	friction_center_a: util.Vector3_Wide = physics.constraint_contact_friction_center(contacts, contact_count);
	friction_center_b: util.Vector3_Wide = util.vector3_wide_subtract(friction_center_a, offset_b);

	position_error_to_velocity, effective_mass_scale, softness: util.F32x8 = physics.spring_settings_wide_compute(
	material.spring_settings,
	dt
	);
	for contact_index: int = 0; contact_index < contact_count; contact_index += 1
	{
		contact_offset_b: util.Vector3_Wide = util.vector3_wide_subtract(contacts[contact_index].offset_a, offset_b);
		contact_response_parent_normal(
		bodies, normal, contacts[contact_index].offset_a, contact_offset_b, contacts[contact_index].depth,
		position_error_to_velocity, effective_mass_scale, material.maximum_recovery_velocity, softness,
		dt, inverse_dt, &penetrations[contact_index], active_mask, body_count,
		);

	}
	total_penetration: util.F32x8 = util.F32x8(0);
	maximum_twist_radius_impulse: util.F32x8 = util.F32x8(0);
	for contact_index: int = 0; contact_index < contact_count; contact_index += 1
	{
		total_penetration = simd.add(total_penetration, penetrations[contact_index]);
		when contact_count == 1
		{
			maximum_twist_radius_impulse = simd.mul(
			penetrations[contact_index],
			simd.max(util.F32x8(0), contacts[contact_index].depth)
			);
		}
		else
		{
			radius: util.F32x8 = util.vector3_wide_length(util.vector3_wide_subtract(
			friction_center_a,
			contacts[contact_index].offset_a
			));
			maximum_twist_radius_impulse = simd.add(
			maximum_twist_radius_impulse,
			simd.mul(penetrations[contact_index], radius)
			);
		}
	}
	contact_response_parent_tangent(
	bodies,
	tangent_x,
	tangent_y,
	friction_center_a,
	friction_center_b,
	simd.mul(material.friction_coefficient, total_penetration),
	tangent,
	active_mask,
	body_count
	);
	contact_response_parent_twist(
	bodies,
	normal,
	simd.mul(material.friction_coefficient, maximum_twist_radius_impulse),
	twist,
	active_mask,
	body_count
	);
}

contact_response_parent_nonconvex_solve :: proc "contextless" (
    prestep_raw: rawptr, bodies: [^]physics.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
    impulses_raw: rawptr, active: util.I32x8, $body_count, $contact_count: int,
)
{
    material: ^physics.Contact_Material_Wide = (^physics.Contact_Material_Wide)(prestep_raw);
    cursor: uintptr = uintptr(prestep_raw)+uintptr(size_of(physics.Contact_Material_Wide));
    offset_b: util.Vector3_Wide;
    when body_count == 2
    {
        offset_b = (^util.Vector3_Wide)(rawptr(cursor))^;
        cursor += uintptr(size_of(util.Vector3_Wide));
    }
    contacts: [^]physics.Nonconvex_Contact_Wide = ([^]physics.Nonconvex_Contact_Wide)(rawptr(cursor));
    impulses: [^]physics.Nonconvex_Contact_Accumulated_Impulses = ([^]physics.Nonconvex_Contact_Accumulated_Impulses)(impulses_raw);
    position, mass, softness: util.F32x8 = physics.spring_settings_wide_compute(material.spring_settings, dt);
    for contact: int = 0; contact < contact_count; contact += 1
    {
        ob: util.Vector3_Wide = util.vector3_wide_subtract(contacts[contact].offset, offset_b);
        contact_response_parent_normal(bodies, contacts[contact].normal, contacts[contact].offset, ob,
            contacts[contact].depth, position, mass, material.maximum_recovery_velocity, softness,
            dt, inverse_dt, &impulses[contact].penetration, active, body_count);
        tx, ty: util.Vector3_Wide = physics.constraint_kernel_build_orthonormal_basis(contacts[contact].normal);
        contact_response_parent_tangent(bodies, tx, ty, contacts[contact].offset, ob,
            simd.mul(material.friction_coefficient, impulses[contact].penetration),
            &impulses[contact].tangent, active, body_count);
    }
}

Contact_Response_Range_Mode :: enum u8
{
    Normal,
    Tangent,
    Twist,
}

contact_response_range_mask :: proc "contextless" (bits: u32) -> util.I32x8
{
    result: util.I32x8;
    for lane: int = 0; lane < 8; lane += 1
    {
        if (bits & (1 << uint(lane))) != 0
        {
            result = simd.replace(result, lane, -1);
        }
    }
    return result;
}

contact_response_range_case :: proc(t: ^testing.T, $Description: typeid, $body_count, $contact_count: int,
    mode: Contact_Response_Range_Mode)
{
    description: Description;
    normal: util.Vector3 = {0, 1, 0};
    offset_a: util.Vector3;
    if mode == .Normal
    {
        when body_count == 1
        {
            offset_a = {2, 0, 0};
        }
    }
    else if mode == .Tangent
    {
        small: f32 = transmute(f32)u32(0x26000000);
        offset_a = {small, 0, -small};
    }
    else
    {
        component: f32 = transmute(f32)u32(0x3f3504f3);
        normal = {component, component, 0};
    }
    when contact_count == 1
    {
        description.normal = normal;
        description.contact_0.offset_a = offset_a;
        description.material = {friction_coefficient=1, spring_settings={64, 2}, maximum_recovery_velocity=2};
        when body_count == 2
        {
            if mode == .Normal
            {
                description.offset_b = {-2, 0, 0};
            }
        }
    }
    else
    {
        description.common.friction_coefficient = 1;
        description.common.spring_settings = {64, 2};
        description.common.maximum_recovery_velocity = 2;
        when body_count == 2
        {
            if mode == .Normal
            {
                description.common.offset_b = {-2, 0, 0};
            }
        }
        description.contact_0 = {offset_a=offset_a, normal=normal};
        description.contact_1 = description.contact_0;
        when contact_count >= 3
        {
            description.contact_2 = description.contact_0;
        }
        when contact_count == 4
        {
            description.contact_3 = description.contact_0;
        }
    }
    testing.expect_value(t, physics.constraint_description_validate(&description), physics.Physics_Status.Ok);
    admitted: [body_count]physics.Body_Description;
    for body: int = 0; body < body_count; body += 1
    {
        admitted[body].pose.orientation.w = 1;
        admitted[body].local_inertia = {inverse_mass=1.0/f32(body_count), inverse_inertia_tensor={xx=1, yy=1, zz=1}};
        if body == body_count-1
        {
            if mode == .Normal
            {
                admitted[body].local_inertia.inverse_inertia_tensor.zz = transmute(f32)u32(0x7f000000);
            }
            else if mode == .Tangent
            {
                admitted[body].local_inertia.inverse_inertia_tensor.yy = transmute(f32)u32(0x7d800000);
            }
            else
            {
                admitted[body].local_inertia.inverse_inertia_tensor = {xx=max(f32), yx=transmute(f32)u32(0x7f000000), yy=max(f32), zz=1};
            }
        }
        if mode == .Tangent && body == 0
        {
            admitted[body].velocity.linear = {-transmute(f32)u32(0x5d800000), -transmute(f32)u32(0x5e800000), transmute(f32)u32(0x5d800000)};
        }
        testing.expect_value(t, physics.body_description_validate(&admitted[body]), physics.Physics_Status.Ok);
    }
    registry: physics.Constraint_Type_Registry = registry_fixture(t);
    for bits: u32 = 0; bits < 256; bits += 1
    {
        actual: Kernel_Run_Result;
        record: ^physics.Constraint_Type_Record = kernel_initialize(t, &registry, &description, 8, &actual);
        actual.bodies = {};
        actual.impulses = {};
        for body: int = 0; body < body_count; body += 1
        {
            inertia: util.Symmetric3x3 = admitted[body].local_inertia.inverse_inertia_tensor;
            actual.bodies[body].orientation.w = util.F32x8(1);
            actual.bodies[body].inverse_mass = util.F32x8(admitted[body].local_inertia.inverse_mass);
            actual.bodies[body].inverse_inertia = {util.F32x8(inertia.xx), util.F32x8(inertia.yx), util.F32x8(inertia.yy), util.F32x8(inertia.zx), util.F32x8(inertia.zy), util.F32x8(inertia.zz)};
            actual.bodies[body].linear_velocity = {util.F32x8(admitted[body].velocity.linear.x), util.F32x8(admitted[body].velocity.linear.y), util.F32x8(admitted[body].velocity.linear.z)};
            for lane: int = 0; lane < 8; lane += 1
            {
                if lane%2 != 0 && body == body_count-1
                {
                    actual.bodies[body].inverse_inertia.xx = simd.replace(actual.bodies[body].inverse_inertia.xx, lane, 1);
                    actual.bodies[body].inverse_inertia.yx = simd.replace(actual.bodies[body].inverse_inertia.yx, lane, 0);
                    actual.bodies[body].inverse_inertia.yy = simd.replace(actual.bodies[body].inverse_inertia.yy, lane, 1);
                    actual.bodies[body].inverse_inertia.zx = simd.replace(actual.bodies[body].inverse_inertia.zx, lane, 0);
                    actual.bodies[body].inverse_inertia.zy = simd.replace(actual.bodies[body].inverse_inertia.zy, lane, 0);
                    actual.bodies[body].inverse_inertia.zz = simd.replace(actual.bodies[body].inverse_inertia.zz, lane, 1);
                }
                if (bits & (1 << uint(lane))) == 0
                {
                    actual.bodies[body].linear_velocity.x = simd.replace(actual.bodies[body].linear_velocity.x, lane, transmute(f32)u32(0x7fc12345));
                    actual.bodies[body].angular_velocity.z = simd.replace(actual.bodies[body].angular_velocity.z, lane, transmute(f32)u32(0x7fc54321));
                }
            }
        }
        mask: util.I32x8 = contact_response_range_mask(bits);
        record.prestep_warmstart_solve(&actual.prestep.bytes[0], &actual.bodies, 1.0/64, 64,
            &actual.impulses.bytes[0], mask, .Prestep);
        record.prestep_warmstart_solve(&actual.prestep.bytes[0], &actual.bodies, 1.0/64, 64,
            &actual.impulses.bytes[0], mask, .Warmstart);
        expected: Kernel_Run_Result = actual;
        when contact_count == 1
        {
            contact_response_parent_solve(&expected.prestep.bytes[0], &expected.bodies[0], 1.0/64, 64,
                &expected.impulses.bytes[0], mask, body_count, contact_count);
        }
        else
        {
            contact_response_parent_nonconvex_solve(&expected.prestep.bytes[0], &expected.bodies[0], 1.0/64, 64,
                &expected.impulses.bytes[0], mask, body_count, contact_count);
        }
        for body: int = 0; body < body_count; body += 1
        {
            for lane: int = 0; lane < 8; lane += 1
            {
                if (bits & (1 << uint(lane))) == 0
                {
                    continue;
                }
                parent: [6]f32 = {simd.extract(expected.bodies[body].linear_velocity.x, lane), simd.extract(expected.bodies[body].linear_velocity.y, lane), simd.extract(expected.bodies[body].linear_velocity.z, lane), simd.extract(expected.bodies[body].angular_velocity.x, lane), simd.extract(expected.bodies[body].angular_velocity.y, lane), simd.extract(expected.bodies[body].angular_velocity.z, lane)};
                for value in parent
                {
                    if !testing.expect(t, abs(value) <= max(f32))
                    {
                        return;
                    }
                }
            }
        }
        record.prestep_warmstart_solve(&actual.prestep.bytes[0], &actual.bodies, 1.0/64, 64,
            &actual.impulses.bytes[0], mask, .Solve);
        for body: int = 0; body < body_count; body += 1
        {
            for lane: int = 0; lane < 8; lane += 1
            {
                a: [6]f32 = {simd.extract(actual.bodies[body].linear_velocity.x, lane), simd.extract(actual.bodies[body].linear_velocity.y, lane), simd.extract(actual.bodies[body].linear_velocity.z, lane), simd.extract(actual.bodies[body].angular_velocity.x, lane), simd.extract(actual.bodies[body].angular_velocity.y, lane), simd.extract(actual.bodies[body].angular_velocity.z, lane)};
                e: [6]f32 = {simd.extract(expected.bodies[body].linear_velocity.x, lane), simd.extract(expected.bodies[body].linear_velocity.y, lane), simd.extract(expected.bodies[body].linear_velocity.z, lane), simd.extract(expected.bodies[body].angular_velocity.x, lane), simd.extract(expected.bodies[body].angular_velocity.y, lane), simd.extract(expected.bodies[body].angular_velocity.z, lane)};
                for axis: int = 0; axis < 6; axis += 1
                {
                    if !testing.expect_value(t, transmute(u32)a[axis], transmute(u32)e[axis])
                    {
                        return;
                    }
                }
            }
        }
        if !testing.expect_value(t, bytes_match(&actual.impulses, &expected.impulses, int(record.impulse_bundle_size)), physics.Reference_State.Present)
        {
            return;
        }
    }
}

@(test)
response_reuse_normal_pretransform_range_preserves_admitted_parent :: proc(t: ^testing.T)
{
    contact_response_range_case(t, physics.Contact_1_One_Body, 1, 1, .Normal);
    contact_response_range_case(t, physics.Contact_1, 2, 1, .Normal);
}

@(test)
response_reuse_tangent_pretransform_range_preserves_admitted_parent :: proc(t: ^testing.T)
{
    contact_response_range_case(t, physics.Contact_1_One_Body, 1, 1, .Tangent);
    contact_response_range_case(t, physics.Contact_1, 2, 1, .Tangent);
}

@(test)
response_reuse_twist_pretransform_range_preserves_admitted_parent :: proc(t: ^testing.T)
{
    contact_response_range_case(t, physics.Contact_1_One_Body, 1, 1, .Twist);
    contact_response_range_case(t, physics.Contact_1, 2, 1, .Twist);
}

@(test)
response_reuse_nonconvex_pretransform_range_preserves_admitted_parent :: proc(t: ^testing.T)
{
    for mode in ([2]Contact_Response_Range_Mode{.Normal, .Tangent})
    {
        contact_response_range_case(t, physics.Contact_2_Nonconvex_One_Body, 1, 2, mode);
        contact_response_range_case(t, physics.Contact_3_Nonconvex_One_Body, 1, 3, mode);
        contact_response_range_case(t, physics.Contact_4_Nonconvex_One_Body, 1, 4, mode);
        contact_response_range_case(t, physics.Contact_2_Nonconvex, 2, 2, mode);
        contact_response_range_case(t, physics.Contact_3_Nonconvex, 2, 3, mode);
        contact_response_range_case(t, physics.Contact_4_Nonconvex, 2, 4, mode);
    }
}
