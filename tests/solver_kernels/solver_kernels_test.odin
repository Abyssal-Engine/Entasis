package solver_kernel_tests

import "core:math"
import "core:mem"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

KERNEL_PRESTEP_STORAGE_BYTES :: 2048;
KERNEL_IMPULSE_STORAGE_BYTES :: 512;
Kernel_Prestep_Storage :: struct #align(32)
{
	bytes: [KERNEL_PRESTEP_STORAGE_BYTES]u8,
}

Kernel_Impulse_Storage :: struct #align(32)
{
	bytes: [KERNEL_IMPULSE_STORAGE_BYTES]u8,
}

Kernel_Run_Result :: struct
{
	prestep:           Kernel_Prestep_Storage,
	warmstarted_impulses: Kernel_Impulse_Storage,
	impulses:          Kernel_Impulse_Storage,
	warmstarted_bodies: [4]physics.Constraint_Kernel_Body_Wide,
	bodies:            [4]physics.Constraint_Kernel_Body_Wide,
}

Kernel_Source_Oracle :: struct
{
	source:             string,
	root_first_line:    i32,
	root_last_line:     i32,
	prestep:           [3]f32,
	warmstart_velocity: [3]f32,
	solve_velocity:    [3]f32,
	solve_impulse:     [3]f32,
}

// expected values were calculated from the corresponding BEPUphysics2 kernel formulas.
// keep them independent from the Odin implementation under test
KERNEL_SOURCE_ORACLES := [physics.CONSTRAINT_TYPE_ID_CAPACITY]Kernel_Source_Oracle{
	{
		"BepuPhysics/Constraints/Contact/ContactConvexTypes.cs:Contact1OneBodyFunctions",
		292,
		333,
		{173.149994, 173.149994, 173.149994},
		{0.246812493, 0.60362494, 0.960437477},
		{0.482626736, 0.708396554, 0.934166491},
		{0.381209671, 0.40096572, 0.420721799},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactConvexTypes.cs:Contact2OneBodyFunctions",
		441,
		488,
		{254.25, 254.25, 254.25},
		{0.307593763, 0.72518754, 1.14278126},
		{1.55631685, 1.652518604, 1.874966568},
		{1.3176235, 1.214556629, 1.212486718},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactConvexTypes.cs:Contact3OneBodyFunctions",
		602,
		653,
		{345, 345, 345},
		{0.394375026, 0.898750007, 1.40312493},
		{2.93974972, 3.041505873, 3.143262558},
		{2.50446749, 2.367880696, 2.231294287},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactConvexTypes.cs:Contact4OneBodyFunctions",
		773,
		828,
		{449.799988, 449.799988, 449.799988},
		{0.513156295, 1.13631248, 1.75946867},
		{4.37176991, 4.46572399, 4.559677345},
		{3.72214031, 3.54941845, 3.376696850},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactConvexTypes.cs:Contact1Functions",
		941,
		984,
		{238.5, 238.5, 238.5},
		{0.952074885, 1.66414976, 2.37622476},
		{0.281458706, 1.08610773, 1.92115951},
		{0.210232943, 0.194573283, 0.178990424},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactConvexTypes.cs:Contact2Functions",
		1103,
		1152,
		{325.600006, 325.600006, 325.600006},
		{0.854325056, 1.46864986, 2.08297539},
		{-0.639803648, 0.167347491, 0.974499345},
		{0.688773155, 0.665949583, 0.64312607},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactConvexTypes.cs:Contact3Functions",
		1277,
		1330,
		{422.350006, 422.350006, 422.350006},
		{0.725624919, 1.21124959, 1.6968751},
		{-1.87524211, -1.05720377, -0.239165902},
		{1.41645098, 1.38303554, 1.34962022},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactConvexTypes.cs:Contact4Functions",
		1461,
		1518,
		{533.150024, 533.150024, 533.150024},
		{0.561025083, 0.882049799, 1.20307505},
		{-3.21430159, -2.3720293, -1.52975702},
		{2.27233434, 2.22450089, 2.17666745},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactNonconvexCommon.cs:ContactNonconvexOneBodyFunctions<Contact2NonconvexOneBodyPrestepData,Contact2NonconvexAccumulatedImpulses>",
		171,
		242,
		{84.3499985, 84.3499985, 84.3499985},
		{0.246687487, 0.603374958, 0.960062444},
		{0.645713151, 0.897336483, 1.14896059},
		{1.66938114, 1.73692322, 1.80446565},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactNonconvexCommon.cs:ContactNonconvexOneBodyFunctions<Contact3NonconvexOneBodyPrestepData,Contact3NonconvexAccumulatedImpulses>",
		171,
		242,
		{140.799988, 140.799988, 140.799988},
		{0.386062503, 0.88212502, 1.37818742},
		{0.27999258, 0.512960672, 0.752284408},
		{3.7512126, 3.98171329, 4.17182064},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactNonconvexCommon.cs:ContactNonconvexOneBodyFunctions<Contact4NonconvexOneBodyPrestepData,Contact4NonconvexAccumulatedImpulses>",
		171,
		242,
		{229.100006, 229.100006, 229.100006},
		{0.666437507, 1.44287491, 2.21931219},
		{2.48896837, 2.61161947, 2.71281743},
		{9.53945255, 9.45845795, 9.32607841},
	},
	{}, {}, {}, {},
	{
		"BepuPhysics/Constraints/Contact/ContactNonconvexCommon.cs:ContactNonconvexTwoBodyFunctions<Contact2NonconvexPrestepData,Contact2NonconvexAccumulatedImpulses>",
		243,
		300,
		{103.949989, 103.949989, 103.949989},
		{1.04537487, 1.85074997, 2.65612507},
		{0.543745279, 1.37459791, 2.19565916},
		{0.754507422, 0.697908282, 0.637392879},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactNonconvexCommon.cs:ContactNonconvexTwoBodyFunctions<Contact3NonconvexPrestepData,Contact3NonconvexAccumulatedImpulses>",
		243,
		300,
		{168.049988, 168.049988, 168.049988},
		{0.678337514, 1.1166749, 1.55501258},
		{-0.75489974, 0.212281525, 1.16504669},
		{2.05742741, 2.23733759, 2.24634314},
	},
	{
		"BepuPhysics/Constraints/Contact/ContactNonconvexCommon.cs:ContactNonconvexTwoBodyFunctions<Contact4NonconvexPrestepData,Contact4NonconvexAccumulatedImpulses>",
		243,
		300,
		{265.649994, 265.649994, 265.649994},
		{0.498193622, 0.756387353, 1.01458073},
		{-1.75503063, -0.709426284, 0.338141561},
		{5.85731125, 5.73361588, 5.60117197},
	},
	{}, {}, {}, {},
	{
		"BepuPhysics/Constraints/BallSocket.cs:BallSocketFunctions",
		66,
		96,
		{108.599998, 108.599998, 108.599998},
		{0.967124939, 1.69424999, 2.42137504},
		{-15.638052, -14.8413401, -14.0446215},
		{4.59881449, 4.6065526, 4.61429024},
	},
	{
		"BepuPhysics/Constraints/AngularHinge.cs:AngularHingeFunctions",
		71,
		224,
		{105.5, 105.5, 105.5},
		{1.09949994, 1.95899999, 2.81850004},
		{-8.4217186, -7.59171486, -6.76171589},
		{-1.86112094, -1.86112082, -1.86112082},
	},
	{
		"BepuPhysics/Constraints/AngularSwivelHinge.cs:AngularSwivelHingeFunctions",
		71,
		149,
		{105.5, 105.5, 105.5},
		{1.04449987, 1.84899998, 2.65349984},
		{1.05772293, 1.88772297, 2.71772289},
		{0.00120363128, 0.00120363105, 0.00120363105},
	},
	{
		"BepuPhysics/Constraints/SwingLimit.cs:SwingLimitFunctions",
		92,
		170,
		{116, 116, 116},
		{1.04449987, 1.84899998, 2.65349984},
		{1.06999993, 1.89999986, 2.73000002},
		{0, 0, 0},
	},
	{
		"BepuPhysics/Constraints/TwistServo.cs:TwistServoFunctions",
		86,
		223,
		{14464.5, 14464.5, 14464.5},
		{1.04449987, 1.84899998, 2.65349984},
		{4.07512283, 4.90512276, 5.73512268},
		{-0.294619828, -0.294619858, -0.294619828},
	},
	{
		"BepuPhysics/Constraints/TwistLimit.cs:TwistLimitFunctions",
		86,
		138,
		{168.5, 168.5, 168.5},
		{1.04449987, 1.84899998, 2.65349984},
		{1.06999993, 1.89999986, 2.73000002},
		{0, 0, 0},
	},
	{
		"BepuPhysics/Constraints/TwistMotor.cs:TwistMotorFunctions",
		77,
		127,
		{6200, 6200, 6200},
		{1.04242289, 1.84484577, 2.6472683},
		{-1.13362038, -0.303620696, 0.526379526},
		{0.199768573, 0.199768573, 0.199768573},
	},
	{
		"BepuPhysics/Constraints/AngularServo.cs:AngularServoFunctions",
		69,
		139,
		{9254.5, 9254.5, 9254.5},
		{0.932999969, 1.62599981, 2.31900024},
		{1.09088969, 1.9208895, 2.75088978},
		{-0.00112721371, -0.0011272179, -0.00112721743},
	},
	{
		"BepuPhysics/Constraints/AngularMotor.cs:AngularMotorFunctions",
		61,
		95,
		{3114, 3114, 3114},
		{0.932999969, 1.62599981, 2.31900024},
		{-3.62897778, -2.79897833, -1.96897817},
		{1.16984439, 1.16984439, 1.16984439},
	},
	{
		"BepuPhysics/Constraints/Weld.cs:WeldFunctions",
		83,
		221,
		{121.900002, 121.900002, 121.900002},
		{0.677124977, 1.11424983, 1.55137503},
		{-7.19396925, -6.39423656, -5.59449959},
		{5.25963593, 5.28021908, 5.30080223},
	},
	{
		"BepuPhysics/Constraints/VolumeConstraint.cs:VolumeConstraintFunctions",
		76,
		187,
		{31.5, 31.5, 31.5},
		{7.84374714, 10.727354, 13.6108303},
		{6.00354767, 8.83689117, 11.6706324},
		{-0.191226184, -0.194607809, -0.197940335},
	},
	{
		"BepuPhysics/Constraints/DistanceServo.cs:DistanceServoFunctions",
		107,
		227,
		{10383.0996, 10383.0996, 10383.0996},
		{1.04191828, 1.84381557, 2.64569283},
		{3.08578134, 3.88803577, 4.69013786},
		{-0.179456547, -0.176920995, -0.174374864},
	},
	{
		"BepuPhysics/Constraints/DistanceLimit.cs:DistanceLimitFunctions",
		103,
		182,
		{156.100006, 156.100006, 156.100006},
		{1.04191828, 1.84381557, 2.64569283},
		{1.07000005, 1.89999986, 2.73000002},
		{0, 0, 0},
	},
	{
		"BepuPhysics/Constraints/CenterDistanceConstraint.cs:CenterDistanceConstraintFunctions",
		69,
		134,
		{31.5, 31.5, 31.5},
		{1.04348087, 1.84693575, 2.65039086},
		{7.73448563, 8.53844547, 9.34220791},
		{-0.628073454, -0.625511348, -0.622937918},
	},
	{
		"BepuPhysics/Constraints/AreaConstraint.cs:AreaConstraintFunctions",
		76,
		198,
		{31.5, 31.5, 31.5},
		{3.42190719, 5.13362312, 6.84514046},
		{6.37024498, 7.99686575, 9.62456512},
		{0.438451231, 0.430785269, 0.423244238},
	},
	{
		"BepuPhysics/Constraints/PointOnLineServo.cs:PointOnLineServoFunctions",
		82,
		194,
		{12455.0996, 12455.0996, 12455.0996},
		{1.15085006, 2.06170011, 2.97254992},
		{-0.96342802, -0.148582697, 0.666263938},
		{-0.103468418, -0.105045587, -0.106622726},
	},
	{
		"BepuPhysics/Constraints/LinearAxisServo.cs:LinearAxisServoFunctions",
		89,
		249,
		{13502.0996, 13502.0996, 13502.0996},
		{1.04013121, 1.84003758, 2.63971877},
		{-0.886288881, -0.060213089, 0.76581955},
		{0.163740411, 0.163453251, 0.163171798},
	},
	{
		"BepuPhysics/Constraints/LinearAxisMotor.cs:LinearAxisMotorFunctions",
		82,
		111,
		{8526.09961, 8526.09961, 8526.09961},
		{1.04013121, 1.84003758, 2.63971877},
		{1.96110821, 2.78942466, 3.61765337},
		{-0.0745853186, -0.0741650388, -0.0737406909},
	},
	{
		"BepuPhysics/Constraints/LinearAxisLimit.cs:LinearAxisLimitFunctions",
		90,
		152,
		{202.600006, 202.600006, 202.600006},
		{1.04013121, 1.84003758, 2.63971877},
		{1.06999993, 1.89999986, 2.73000002},
		{0, 0, 0},
	},
	{
		"BepuPhysics/Constraints/AngularAxisMotor.cs:AngularAxisMotorFunctions",
		69,
		107,
		{3879, 3879, 3879},
		{1.05250001, 1.86499977, 2.67750001},
		{-0.48555544, 0.344443858, 1.17444396},
		{0.222222224, 0.222222224, 0.222222224},
	},
	{
		"BepuPhysics/Constraints/OneBodyAngularServo.cs:OneBodyAngularServoFunctions",
		69,
		110,
		{9254.5, 9254.5, 9254.5},
		{0.263750017, 0.637499988, 1.01125002},
		{0.141318709, 0.352837324, 0.564355969},
		{-0.00746251969, -0.0248750579, -0.042287603},
	},
	{
		"BepuPhysics/Constraints/OneBodyAngularMotor.cs:OneBodyAngularMotorFunctions",
		61,
		94,
		{3114, 3114, 3114},
		{0.263750017, 0.637499988, 1.01125002},
		{7.76095295, 7.97476244, 8.18857193},
		{2.42142868, 2.40476179, 2.38809538},
	},
	{
		"BepuPhysics/Constraints/OneBodyLinearServo.cs:OneBodyLinearServoFunctions",
		77,
		147,
		{11340, 11340, 11340},
		{0.189999998, 0.49000001, 0.789999962},
		{-5.30190182, -5.07235813, -4.84281349},
		{-4.61013794, -4.64091587, -4.67169523},
	},
	{
		"BepuPhysics/Constraints/OneBodyLinearMotor.cs:OneBodyLinearMotorFunctions",
		67,
		101,
		{5425.5, 5425.5, 5425.5},
		{0.189999998, 0.49000001, 0.789999962},
		{1.93181574, 2.1818614, 2.43190718},
		{1.40476155, 1.39285696, 1.38095224},
	},
	{
		"BepuPhysics/Constraints/SwivelHinge.cs:SwivelHingeFunctions",
		83,
		215,
		{212.100006, 212.100006, 212.100006},
		{0.865124941, 1.49025011, 2.11537504},
		{-16.9646683, -16.159132, -15.3535957},
		{5.10100985, 5.10540915, 5.10980892},
	},
	{
		"BepuPhysics/Constraints/Hinge.cs:HingeFunctions",
		89,
		223,
		{212.100006, 212.100006, 212.100006},
		{1.00862503, 1.77725005, 2.54587483},
		{-24.9738274, -24.1668205, -23.3598118},
		{-4.69907808, -4.68813038, -4.67718315},
	},
	{}, {}, {}, {},
	{
		"BepuPhysics/Constraints/BallSocketMotor.cs:BallSocketMotorFunctions",
		68,
		98,
		{5445.2002, 5445.2002, 5445.2002},
		{0.970156193, 1.7003876, 2.43069386},
		{5.16416025, 5.97938156, 6.79455376},
		{-1.21213031, -1.2089721, -1.20579994},
	},
	{
		"BepuPhysics/Constraints/BallSocketServo.cs:BallSocketServoFunctions",
		75,
		108,
		{11338.5996, 11338.5996, 11338.5996},
		{0.967124939, 1.69424999, 2.42137504},
		{-15.6380558, -14.8413401, -14.0446205},
		{4.59881496, 4.6065526, 4.61429071},
	},
	{
		"BepuPhysics/Constraints/AngularAxisGearMotor.cs:AngularAxisGearMotorFunctions; Entasis incremental-impulse correction",
		70,
		115,
		{3879, 3879, 3879},
		{1.0625, 1.88499975, 2.70749998},
		{1.07, 1.9, 2.73},
		{0, 0, 0},
	},
	{
		"BepuPhysics/Constraints/CenterDistanceLimit.cs:CenterDistanceLimitFunctions",
		78,
		133,
		{48, 48, 48},
		{1.04347253, 1.84693575, 2.65039086},
		{1.07000005, 1.89999986, 2.73000002},
		{0, 0, 0},
	},
	{}, {}, {}, {}, {}, {}, {}, {},
};

valid_spring :: proc() -> physics.Spring_Settings
{
	return {angular_frequency=12.5, twice_damping_ratio=1.5};
}

valid_servo :: proc() -> physics.Servo_Settings
{
	return {maximum_speed=25, base_speed=0.5, maximum_force=1000};
}

valid_motor :: proc() -> physics.Motor_Settings
{
	return {maximum_force=750, damping=20};
}

unit_x :: proc() -> util.Vector3
{
	return {1, 0, 0};
}

unit_y :: proc() -> util.Vector3
{
	return {0, 1, 0};
}

unit_z :: proc() -> util.Vector3
{
	return {0, 0, 1};
}

valid_material :: proc() -> physics.Contact_Material_Properties
{
	return {friction_coefficient=0.7, spring_settings=valid_spring(), maximum_recovery_velocity=3};
}

valid_contact :: proc(index: int) -> physics.Constraint_Contact_Data
{
	return {
		offset_a={0.2 * f32(index + 1), 0.1 * f32(index), 0.15 * f32(index + 1)},
		penetration_depth=0.1 * f32(index + 1),
	};
}

valid_nonconvex_contact :: proc(index: int) -> physics.Nonconvex_Constraint_Contact_Data
{
	normals := [4]util.Vector3{unit_y(), unit_x(), unit_z(), unit_y()};
	return {
		offset_a={0.2 * f32(index + 1), 0.1 * f32(index), 0.15 * f32(index + 1)},
		normal=normals[index],
		penetration_depth=0.1 * f32(index + 1),
	};
}

lane_mask :: proc "contextless" (lane_count: int) -> util.I32x8
{
	result := util.I32x8(0);
	for lane in 0 ..< lane_count
	{
		result = simd.replace(result, lane, -1);
	}
	return result;
}

kernel_bodies :: proc "contextless" () -> [4]physics.Constraint_Kernel_Body_Wide
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide;
	base_positions := [4]util.Vector3{{0, 0, 0}, {1, 0.75, 0.25}, {0.25, 1.5, 0.5}, {0.5, 0.25, 2}};
	for body_index in 0 ..< 4
	{
		bodies[body_index].inverse_mass = util.F32x8(1 + 0.1 * f32(body_index));
		bodies[body_index].inverse_inertia.xx = util.F32x8(1 + 0.1 * f32(body_index));
		bodies[body_index].inverse_inertia.yy = util.F32x8(1.25 + 0.1 * f32(body_index));
		bodies[body_index].inverse_inertia.zz = util.F32x8(1.5 + 0.1 * f32(body_index));
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			position := base_positions[body_index];
			position.x += 0.01 * f32(lane);
			position.y += 0.005 * f32(lane * body_index);
			util.vector3_wide_write_slot(&bodies[body_index].position, lane, position);
			util.quaternion_wide_write_slot(&bodies[body_index].orientation, lane, util.quaternion_identity());
			util.vector3_wide_write_slot(&bodies[body_index].linear_velocity, lane, {
					0.05 * f32(body_index + 1), -0.02 * f32(lane + 1), 0.03 * f32(body_index + lane + 1),
			});
			util.vector3_wide_write_slot(&bodies[body_index].angular_velocity, lane, {
					-0.04 * f32(body_index + 1), 0.025 * f32(lane + 1), 0.015 * f32(body_index + lane + 1),
			});
		}
	}
	return bodies;
}

bytes_match :: proc "contextless" (a, b: rawptr, count: int) -> physics.Reference_State
{
	left := ([^]u8)(a);
	right := ([^]u8)(b);
	for index in 0 ..< count
	{
		if left[index] != right[index]
		{
			return .Missing;
		}
	}
	return .Present;
}

f32_is_finite :: proc "contextless" (value: f32) -> physics.Reference_State
{
	if transmute(u32)value & 0x7f80_0000 != 0x7f80_0000
	{
		return .Present;
	}
	return .Missing;
}

body_velocity_is_finite :: proc "contextless" (
	body: ^physics.Constraint_Kernel_Body_Wide, lane: int,
) -> physics.Reference_State
{
	linear := util.vector3_wide_read_slot(body.linear_velocity, lane);
	angular := util.vector3_wide_read_slot(body.angular_velocity, lane);
	if f32_is_finite(linear.x) == .Present &&
	f32_is_finite(linear.y) == .Present &&
	f32_is_finite(linear.z) == .Present &&
	f32_is_finite(angular.x) == .Present &&
	f32_is_finite(angular.y) == .Present &&
	f32_is_finite(angular.z) == .Present
	{
		return .Present;
	}
	return .Missing;
}

kernel_initialize :: proc(
	t: ^testing.T, registry: ^physics.Constraint_Type_Registry, description: ^$T, lane_count: int,
	result: ^Kernel_Run_Result,
) -> ^physics.Constraint_Type_Record
{
	type_id := physics.constraint_description_type_id(T);
	record, status := physics.constraint_type_registry_lookup(registry, type_id);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, record.description_size, i32(size_of(T)));
	testing.expect(t, int(record.prestep_bundle_size) <= KERNEL_PRESTEP_STORAGE_BYTES);
	testing.expect(t, int(record.impulse_bundle_size) <= KERNEL_IMPULSE_STORAGE_BYTES);
	result^ = {};
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		record.apply_description(
			description, &result.prestep.bytes[0], int(type_id), size_of(T), int(record.prestep_bundle_size), lane,
		);
		readback: T;
		testing.expect_value(t, record.build_description(
				&result.prestep.bytes[0], &readback, int(type_id), size_of(T), int(record.prestep_bundle_size), lane,
			), physics.Physics_Status.Ok);
		testing.expect_value(t, bytes_match(description, &readback, size_of(T)), physics.Reference_State.Present);
	}
	impulse_fields := int(record.impulse_bundle_size) / size_of(util.F32x8);
	impulse_vectors := ([^]util.F32x8)(&result.impulses.bytes[0]);
	for field in 0 ..< impulse_fields
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			impulse_vectors[field] = simd.replace(impulse_vectors[field], lane, 0.0025 * f32((field + 1) * (lane + 1)));
		}
	}
	result.bodies = kernel_bodies();
	return record;
}

kernel_run :: proc(
	t: ^testing.T, registry: ^physics.Constraint_Type_Registry, description: ^$T, lane_count: int,
) -> Kernel_Run_Result
{
	result: Kernel_Run_Result;
	record := kernel_initialize(t, registry, description, lane_count, &result);
	mask := lane_mask(lane_count);
	record.prestep_warmstart_solve(
		&result.prestep.bytes[0],
		&result.bodies,
		1.0 / 64.0,
		64,
		&result.impulses.bytes[0],
		mask,
		.Prestep
	);
	record.prestep_warmstart_solve(
		&result.prestep.bytes[0],
		&result.bodies,
		1.0 / 64.0,
		64,
		&result.impulses.bytes[0],
		mask,
		.Warmstart
	);
	result.warmstarted_bodies = result.bodies;
	result.warmstarted_impulses = result.impulses;
	record.prestep_warmstart_solve(
		&result.prestep.bytes[0],
		&result.bodies,
		1.0 / 64.0,
		64,
		&result.impulses.bytes[0],
		mask,
		.Solve
	);
	return result;
}

kernel_results_match :: proc "contextless" (
	a, b: ^Kernel_Run_Result, record: ^physics.Constraint_Type_Record,
) -> physics.Reference_State
{
	if bytes_match(&a.prestep.bytes[0], &b.prestep.bytes[0], int(record.prestep_bundle_size)) == .Missing
	{
		return .Missing;
	}
	if bytes_match(&a.impulses.bytes[0], &b.impulses.bytes[0], int(record.impulse_bundle_size)) == .Missing
	{
		return .Missing;
	}
	if bytes_match(&a.warmstarted_bodies, &b.warmstarted_bodies, size_of(a.warmstarted_bodies)) == .Missing
	{
		return .Missing;
	}
	return bytes_match(&a.bodies, &b.bodies, size_of(a.bodies));
}

numeric_bundle_signature :: proc "contextless" (data: rawptr, size, lane: int) -> f32
{
	vectors := ([^]util.F32x8)(data);
	result: f32;
	for field in 0 ..< size / size_of(util.F32x8)
	{
		result += simd.extract(vectors[field], lane) * f32(field + 1);
	}
	return result;
}

velocity_signature :: proc "contextless" (bodies: ^[4]physics.Constraint_Kernel_Body_Wide, body_count, lane: int) -> f32
{
	result: f32;
	weight: f32 = 1;
	for body_index in 0 ..< body_count
	{
		linear := util.vector3_wide_read_slot(bodies[body_index].linear_velocity, lane);
		angular := util.vector3_wide_read_slot(bodies[body_index].angular_velocity, lane);
		values := [6]f32{linear.x, linear.y, linear.z, angular.x, angular.y, angular.z};
		for value in values
		{
			result += value * weight;
			weight += 1;
		}
	}
	return result;
}

source_oracle_tolerance :: proc "contextless" (type_id: i32, expected, absolute_tolerance: f32) -> f32
{
	switch type_id
	{
		case physics.VOLUME_CONSTRAINT_TYPE_ID,
		physics.CENTER_DISTANCE_CONSTRAINT_TYPE_ID,
		physics.AREA_CONSTRAINT_TYPE_ID,
		physics.CENTER_DISTANCE_LIMIT_TYPE_ID:
		// these kernels retain the approximate reciprocal operations used by the reference formulas.
		// hardware approximation results may vary slightly across CPU and compiler targets
		return max(absolute_tolerance, math.abs(expected) * 0.0005);
	}
	return absolute_tolerance;
}

expect_source_value :: proc(
	t: ^testing.T, type_id: i32, lane: int, oracle: ^Kernel_Source_Oracle,
	operation: string, actual, expected, absolute_tolerance: f32,
)
{
	tolerance := source_oracle_tolerance(type_id, expected, absolute_tolerance);
	testing.expectf(
		t, math.abs(actual - expected) <= tolerance,
		"type %d lane %d %s signature from %s:%d-%d expected %.9g, got %.9g (tolerance %.9g)",
		type_id, lane, operation, oracle.source, oracle.root_first_line, oracle.root_last_line,
		expected, actual, tolerance,
	);
}

verify_source_oracle :: proc(
	t: ^testing.T, type_id: i32, record: ^physics.Constraint_Type_Record, result: ^Kernel_Run_Result,
)
{
	oracle := &KERNEL_SOURCE_ORACLES[type_id];
	testing.expectf(t, len(oracle.source) > 0, "type %d must name its reference function", type_id);
	testing.expectf(
		t, oracle.root_first_line > 0 && oracle.root_last_line >= oracle.root_first_line,
		"type %d must provide an ordered reference span", type_id,
	);
	for lane in 0 ..< 3
	{
		prestep := numeric_bundle_signature(&result.prestep.bytes[0], int(record.prestep_bundle_size), lane);
		warmstart := velocity_signature(&result.warmstarted_bodies, int(record.body_count), lane);
		solve := velocity_signature(&result.bodies, int(record.body_count), lane);
		impulse := numeric_bundle_signature(&result.impulses.bytes[0], int(record.impulse_bundle_size), lane);
		expect_source_value(t, type_id, lane, oracle, "prestep", prestep, oracle.prestep[lane], 0.02);
		expect_source_value(
			t,
			type_id,
			lane,
			oracle,
			"warmstart velocity",
			warmstart,
			oracle.warmstart_velocity[lane],
			0.0002
		);
		expect_source_value(t, type_id, lane, oracle, "solve velocity", solve, oracle.solve_velocity[lane], 0.0002);
		expect_source_value(t, type_id, lane, oracle, "solve impulse", impulse, oracle.solve_impulse[lane], 0.0002);
	}
}

velocity_changed :: proc "contextless" (
	before, after: ^[4]physics.Constraint_Kernel_Body_Wide, body_count, lane: int,
) -> physics.Reference_State
{
	for body_index in 0 ..< body_count
	{
		if util.vector3_wide_read_slot(
			before[body_index].linear_velocity,
			lane
		) != util.vector3_wide_read_slot(after[body_index].linear_velocity, lane) ||
		util.vector3_wide_read_slot(
			before[body_index].angular_velocity,
			lane
		) != util.vector3_wide_read_slot(after[body_index].angular_velocity, lane)
		{
			return .Present;
		}
	}
	return .Missing;
}

verify_kernel_result :: proc(
	t: ^testing.T, record: ^physics.Constraint_Type_Record, result: ^Kernel_Run_Result, lane_count: int,
)
{
	baseline := kernel_bodies();
	impulse_vectors := ([^]util.F32x8)(&result.impulses.bytes[0]);
	warmstarted_impulse_vectors := ([^]util.F32x8)(&result.warmstarted_impulses.bytes[0]);
	impulse_fields := int(record.impulse_bundle_size) / size_of(util.F32x8);
	for field in 0 ..< impulse_fields
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			value := simd.extract(impulse_vectors[field], lane);
			testing.expect_value(t, f32_is_finite(value), physics.Reference_State.Present);
			if lane >= lane_count
			{
				testing.expect_value(t, value, f32(0));
			}
		}
	}
	for lane in 0 ..< lane_count
	{
		testing.expect_value(
			t,
			velocity_changed(&baseline, &result.warmstarted_bodies, int(record.body_count), lane),
			physics.Reference_State.Present
		);
		solve_changed := velocity_changed(&result.warmstarted_bodies, &result.bodies, int(record.body_count), lane);
		for field in 0 ..< impulse_fields
		{
			if simd.extract(warmstarted_impulse_vectors[field], lane) != simd.extract(impulse_vectors[field], lane)
			{
				solve_changed = .Present;
				break;
			}
		}
		testing.expectf(
			t, solve_changed == .Present,
			"type %d lane %d solve must produce an observable velocity or impulse result", record.type_id, lane,
		);
		for body_index in 0 ..< int(record.body_count)
		{
			testing.expect_value(
				t,
				body_velocity_is_finite(&result.warmstarted_bodies[body_index], lane),
				physics.Reference_State.Present
			);
			testing.expect_value(
				t,
				body_velocity_is_finite(&result.bodies[body_index], lane),
				physics.Reference_State.Present
			);
		}
	}
	for lane in lane_count ..< util.PRODUCTION_LANE_COUNT
	{
		for body_index in 0 ..< int(record.body_count)
		{
			testing.expect_value(
				t,
				util.vector3_wide_read_slot(result.bodies[body_index].linear_velocity, lane),
				util.vector3_wide_read_slot(baseline[body_index].linear_velocity, lane)
			);
			testing.expect_value(
				t,
				util.vector3_wide_read_slot(result.bodies[body_index].angular_velocity, lane),
				util.vector3_wide_read_slot(baseline[body_index].angular_velocity, lane)
			);
		}
	}
}

exercise_description :: proc(t: ^testing.T, registry: ^physics.Constraint_Type_Registry, description: ^$T)
{
	type_id := physics.constraint_description_type_id(T);
	record, status := physics.constraint_type_registry_lookup(registry, type_id);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	full0 := kernel_run(t, registry, description, util.PRODUCTION_LANE_COUNT);
	verify_source_oracle(t, type_id, record, &full0);
	full1 := kernel_run(t, registry, description, util.PRODUCTION_LANE_COUNT);
	testing.expect_value(t, kernel_results_match(&full0, &full1, record), physics.Reference_State.Present);
	verify_kernel_result(t, record, &full0, util.PRODUCTION_LANE_COUNT);
	tail0 := kernel_run(t, registry, description, 3);
	verify_source_oracle(t, type_id, record, &tail0);
	tail1 := kernel_run(t, registry, description, 3);
	testing.expect_value(t, kernel_results_match(&tail0, &tail1, record), physics.Reference_State.Present);
	verify_kernel_result(t, record, &tail0, 3);
}

registry_fixture :: proc(t: ^testing.T) -> physics.Constraint_Type_Registry
{
	registry: physics.Constraint_Type_Registry;
	testing.expect_value(t, physics.constraint_type_registry_initialize(&registry), physics.Physics_Status.Ok);
	testing.expect_value(t, registry.registered_count, i32(physics.BUILT_IN_CONSTRAINT_TYPE_COUNT));
	return registry;
}

expected_constraint_access :: proc "contextless" (
	type_id, body_index: int, phase: physics.Constraint_Kernel_Phase,
) -> physics.Body_Access_Mask
{
	switch type_id
	{
		case 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 16, 17:
		return physics.BODY_ACCESS_NO_POSE;
		case physics.BALL_SOCKET_TYPE_ID, physics.BALL_SOCKET_SERVO_TYPE_ID:
		if phase == .Warmstart
		{
			return physics.BODY_ACCESS_NO_POSITION;
		}
		return physics.BODY_ACCESS_ALL;
		case physics.ANGULAR_HINGE_TYPE_ID, physics.ANGULAR_AXIS_MOTOR_TYPE_ID:
		if phase == .Warmstart && body_index == 1
		{
			return physics.BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
		}
		return physics.BODY_ACCESS_ONLY_ANGULAR;
		case physics.ANGULAR_SWIVEL_HINGE_TYPE_ID, physics.SWING_LIMIT_TYPE_ID,
		physics.TWIST_SERVO_TYPE_ID, physics.TWIST_LIMIT_TYPE_ID, physics.TWIST_MOTOR_TYPE_ID:
		return physics.BODY_ACCESS_ONLY_ANGULAR;
		case physics.ANGULAR_SERVO_TYPE_ID:
		if phase == .Warmstart
		{
			return physics.BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
		}
		return physics.BODY_ACCESS_ONLY_ANGULAR;
		case physics.ANGULAR_MOTOR_TYPE_ID:
		if phase == .Warmstart || body_index == 1
		{
			return physics.BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
		}
		return physics.BODY_ACCESS_ONLY_ANGULAR;
		case physics.WELD_TYPE_ID:
		if phase == .Solve
		{
			return physics.BODY_ACCESS_ALL;
		}
		if body_index == 0
		{
			return physics.BODY_ACCESS_NO_POSITION;
		}
		return physics.BODY_ACCESS_NO_POSE;
		case physics.VOLUME_CONSTRAINT_TYPE_ID, physics.CENTER_DISTANCE_CONSTRAINT_TYPE_ID,
		physics.AREA_CONSTRAINT_TYPE_ID, physics.CENTER_DISTANCE_LIMIT_TYPE_ID:
		return physics.BODY_ACCESS_ONLY_LINEAR;
		case physics.DISTANCE_SERVO_TYPE_ID, physics.DISTANCE_LIMIT_TYPE_ID,
		physics.POINT_ON_LINE_SERVO_TYPE_ID, physics.LINEAR_AXIS_SERVO_TYPE_ID,
		physics.LINEAR_AXIS_MOTOR_TYPE_ID, physics.LINEAR_AXIS_LIMIT_TYPE_ID,
		physics.ONE_BODY_LINEAR_SERVO_TYPE_ID:
		return physics.BODY_ACCESS_ALL;
		case physics.ONE_BODY_ANGULAR_SERVO_TYPE_ID:
		return physics.BODY_ACCESS_ONLY_ANGULAR;
		case physics.ONE_BODY_ANGULAR_MOTOR_TYPE_ID:
		if phase == .Warmstart
		{
			return physics.BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
		}
		return physics.BODY_ACCESS_ONLY_ANGULAR;
		case physics.ONE_BODY_LINEAR_MOTOR_TYPE_ID:
		return physics.BODY_ACCESS_NO_POSITION;
		case physics.SWIVEL_HINGE_TYPE_ID, physics.HINGE_TYPE_ID:
		if phase == .Warmstart
		{
			return physics.BODY_ACCESS_NO_POSITION;
		}
		return physics.BODY_ACCESS_ALL;
		case physics.BALL_SOCKET_MOTOR_TYPE_ID:
		if phase == .Solve || body_index == 1
		{
			return physics.BODY_ACCESS_ALL;
		}
		return physics.BODY_ACCESS_NO_ORIENTATION;
		case physics.ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID:
		if body_index == 0
		{
			return physics.BODY_ACCESS_ONLY_ANGULAR;
		}
		return physics.BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
		case:
		return {};
	}
}

@(test)
one_body_noncontact_registry_executes_full_and_tail_bundles :: proc(t: ^testing.T)
{
	registry := registry_fixture(t);
	spring, servo, motor := valid_spring(), valid_servo(), valid_motor();
	q := util.quaternion_identity();
	d0 := physics.One_Body_Angular_Servo{q, spring, servo};
	exercise_description(t, &registry, &d0);
	d1 := physics.One_Body_Angular_Motor{{1, 2, 3}, motor};
	exercise_description(t, &registry, &d1);
	d2 := physics.One_Body_Linear_Servo{{0.25, 0.5, 0.75}, {2, 1, -1}, spring, servo};
	exercise_description(t, &registry, &d2);
	d3 := physics.One_Body_Linear_Motor{{0.25, 0.5, 0.75}, {1, -2, 3}, motor};
	exercise_description(t, &registry, &d3);
}

@(test)
two_body_noncontact_registry_executes_full_and_tail_bundles :: proc(t: ^testing.T)
{
	registry := registry_fixture(t);
	spring, servo, motor := valid_spring(), valid_servo(), valid_motor();
	q := util.quaternion_identity();
	d00 := physics.Ball_Socket{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, spring};
	exercise_description(t, &registry, &d00);
	d01 := physics.Angular_Hinge{unit_x(), unit_y(), spring};
	exercise_description(t, &registry, &d01);
	d02 := physics.Angular_Swivel_Hinge{unit_x(), unit_y(), spring};
	exercise_description(t, &registry, &d02);
	d03 := physics.Swing_Limit{unit_x(), unit_y(), -0.5, spring};
	exercise_description(t, &registry, &d03);
	d04 := physics.Twist_Servo{q, q, 0.5, spring, servo};
	exercise_description(t, &registry, &d04);
	d05 := physics.Twist_Limit{q, q, -1, 1, spring};
	exercise_description(t, &registry, &d05);
	d06 := physics.Twist_Motor{unit_x(), unit_y(), 2, motor};
	exercise_description(t, &registry, &d06);
	d07 := physics.Angular_Servo{q, spring, servo};
	exercise_description(t, &registry, &d07);
	d08 := physics.Angular_Motor{{1, 2, 3}, motor};
	exercise_description(t, &registry, &d08);
	d09 := physics.Weld{{0.1, 0.2, 0.3}, q, spring};
	exercise_description(t, &registry, &d09);
	d10 := physics.Distance_Servo{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, 2, servo, spring};
	exercise_description(t, &registry, &d10);
	d11 := physics.Distance_Limit{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, 0.5, 2, spring};
	exercise_description(t, &registry, &d11);
	d12 := physics.Center_Distance_Constraint{2, spring};
	exercise_description(t, &registry, &d12);
	d13 := physics.Point_On_Line_Servo{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, unit_x(), servo, spring};
	exercise_description(t, &registry, &d13);
	d14 := physics.Linear_Axis_Servo{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, unit_x(), 0.75, servo, spring};
	exercise_description(t, &registry, &d14);
	d15 := physics.Linear_Axis_Motor{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, unit_x(), 2, motor};
	exercise_description(t, &registry, &d15);
	d16 := physics.Linear_Axis_Limit{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, unit_x(), -0.5, 2, spring};
	exercise_description(t, &registry, &d16);
	d17 := physics.Angular_Axis_Motor{unit_x(), 2, motor};
	exercise_description(t, &registry, &d17);
	d18 := physics.Swivel_Hinge{{0.1, 0.2, 0.3}, unit_x(), {0.4, 0.5, 0.6}, unit_y(), spring};
	exercise_description(t, &registry, &d18);
	d19 := physics.Hinge{{0.1, 0.2, 0.3}, unit_x(), {0.4, 0.5, 0.6}, unit_y(), spring};
	exercise_description(t, &registry, &d19);
	d20 := physics.Ball_Socket_Motor{{0.4, 0.5, 0.6}, {1, 2, 3}, motor};
	exercise_description(t, &registry, &d20);
	d21 := physics.Ball_Socket_Servo{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, spring, servo};
	exercise_description(t, &registry, &d21);
	d22 := physics.Angular_Axis_Gear_Motor{unit_x(), 2, motor};
	exercise_description(t, &registry, &d22);
	d23 := physics.Center_Distance_Limit{0.5, 2, spring};
	exercise_description(t, &registry, &d23);
}

@(test)
three_and_four_body_noncontact_registry_executes_full_and_tail_bundles :: proc(t: ^testing.T)
{
	registry := registry_fixture(t);
	area := physics.Area_Constraint{2, valid_spring()};
	exercise_description(t, &registry, &area);
	volume := physics.Volume_Constraint{2, valid_spring()};
	exercise_description(t, &registry, &volume);
}

@(test)
convex_contact_registry_executes_full_and_tail_bundles :: proc(t: ^testing.T)
{
	registry := registry_fixture(t);
	c0, c1, c2, c3 := valid_contact(0), valid_contact(1), valid_contact(2), valid_contact(3);
	material := valid_material();
	d0 := physics.Contact_1_One_Body{c0, unit_y(), material};
	exercise_description(t, &registry, &d0);
	d1 := physics.Contact_2_One_Body{c0, c1, unit_y(), material};
	exercise_description(t, &registry, &d1);
	d2 := physics.Contact_3_One_Body{c0, c1, c2, unit_y(), material};
	exercise_description(t, &registry, &d2);
	d3 := physics.Contact_4_One_Body{c0, c1, c2, c3, unit_y(), material};
	exercise_description(t, &registry, &d3);
	d4 := physics.Contact_1{c0, {0.5, 0.25, 0.75}, unit_y(), material};
	exercise_description(t, &registry, &d4);
	d5 := physics.Contact_2{c0, c1, {0.5, 0.25, 0.75}, unit_y(), material};
	exercise_description(t, &registry, &d5);
	d6 := physics.Contact_3{c0, c1, c2, {0.5, 0.25, 0.75}, unit_y(), material};
	exercise_description(t, &registry, &d6);
	d7 := physics.Contact_4{c0, c1, c2, c3, {0.5, 0.25, 0.75}, unit_y(), material};
	exercise_description(t, &registry, &d7);
}

@(test)
nonconvex_contact_registry_executes_full_and_tail_bundles :: proc(t: ^testing.T)
{
	registry := registry_fixture(t);
	n0, n1, n2, n3 := valid_nonconvex_contact(0), valid_nonconvex_contact(1), valid_nonconvex_contact(2), valid_nonconvex_contact(3);
	material := valid_material();
	one := physics.Nonconvex_One_Body_Properties{
		material.friction_coefficient,
		material.spring_settings,
		material.maximum_recovery_velocity,
	};
	two := physics.Nonconvex_Two_Body_Properties{
		{0.5, 0.25, 0.75},
		material.friction_coefficient,
		material.spring_settings,
		material.maximum_recovery_velocity,
	};
	d0 := physics.Contact_2_Nonconvex_One_Body{one, n0, n1};
	exercise_description(t, &registry, &d0);
	d1 := physics.Contact_3_Nonconvex_One_Body{one, n0, n1, n2};
	exercise_description(t, &registry, &d1);
	d2 := physics.Contact_4_Nonconvex_One_Body{one, n0, n1, n2, n3};
	exercise_description(t, &registry, &d2);
	d3 := physics.Contact_2_Nonconvex{two, n0, n1};
	exercise_description(t, &registry, &d3);
	d4 := physics.Contact_3_Nonconvex{two, n0, n1, n2};
	exercise_description(t, &registry, &d4);
	d5 := physics.Contact_4_Nonconvex{two, n0, n1, n2, n3};
	exercise_description(t, &registry, &d5);
}

@(test)
warmstart_solve_impulses_access_masks_and_allocations_remain_stable :: proc(t: ^testing.T)
{
	registry := registry_fixture(t);
	for type_id in 0 ..< physics.CONSTRAINT_TYPE_ID_CAPACITY
	{
		record := &registry.records[type_id];
		if record.registration == .Missing
		{
			continue;
		}
		for body_index in 0 ..< int(record.body_count)
		{
			testing.expect_value(
				t,
				record.initial_access[body_index],
				expected_constraint_access(type_id, body_index, .Warmstart)
			);
			testing.expect_value(
				t,
				record.solve_access[body_index],
				expected_constraint_access(type_id, body_index, .Solve)
			);
		}
	}
	c0, c1, c2, c3 := valid_contact(0), valid_contact(1), valid_contact(2), valid_contact(3);
	description := physics.Contact_4{c0, c1, c2, c3, {0.5, 0.25, 0.75}, unit_y(), valid_material()};
	result: Kernel_Run_Result;
	record := kernel_initialize(t, &registry, &description, util.PRODUCTION_LANE_COUNT, &result);
	mask := lane_mask(util.PRODUCTION_LANE_COUNT);
	record.prestep_warmstart_solve(
		&result.prestep.bytes[0],
		&result.bodies,
		1.0 / 60.0,
		60,
		&result.impulses.bytes[0],
		mask,
		.Prestep
	);
	record.prestep_warmstart_solve(
		&result.prestep.bytes[0],
		&result.bodies,
		1.0 / 60.0,
		60,
		&result.impulses.bytes[0],
		mask,
		.Warmstart
	);
	before_eight := ([^]util.F32x8)(&result.impulses.bytes[0])[2];
	for _ in 0 ..< 8
	{
		record.prestep_warmstart_solve(
			&result.prestep.bytes[0],
			&result.bodies,
			1.0 / 60.0,
			60,
			&result.impulses.bytes[0],
			mask,
			.Solve
		);
	}
	after_eight := ([^]util.F32x8)(&result.impulses.bytes[0])[2];
	testing.expect(t, simd.extract(before_eight, 0) > 0);
	testing.expect(t, simd.extract(after_eight, 0) >= 0);
	tracker := (^mem.Tracking_Allocator)(context.allocator.data);
	before_allocations := tracker.total_allocation_count;
	for _ in 0 ..< 1024
	{
		record.prestep_warmstart_solve(
			&result.prestep.bytes[0],
			&result.bodies,
			1.0 / 60.0,
			60,
			&result.impulses.bytes[0],
			mask,
			.Solve
		);
	}
	testing.expect_value(t, tracker.total_allocation_count, before_allocations);
	verify_kernel_result(t, record, &result, util.PRODUCTION_LANE_COUNT);
}
#assert(size_of(physics.Contact_4_Nonconvex_Prestep) <= KERNEL_PRESTEP_STORAGE_BYTES);
#assert(size_of(physics.Contact_4_Accumulated_Impulses) <= KERNEL_IMPULSE_STORAGE_BYTES);
@(test)
restitution_capture_masks_match_scalar_oracle :: proc (t: ^testing.T)
{
	// these 4,096 lanes include exact threshold/tangency boundaries, separated
	// contacts, disabled coefficients and incomplete bundles. no timing is involved
	for sample in 0 ..< 512
	{
		velocity, depth, coefficient, threshold: util.F32x8;
		eligible: util.I32x8;
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			index: int = sample * util.PRODUCTION_LANE_COUNT + lane;
			velocity = simd.replace(velocity, lane, f32(index % 17 - 8) * 0.5);
			depth = simd.replace(depth, lane, f32(index % 3 - 1) * 0.125);
			coefficient = simd.replace(coefficient, lane, f32(index % 5) * 0.25);
			threshold = simd.replace(threshold, lane, f32(index % 4) * 0.5);
			if index % 7 != 0
			{
				eligible = simd.replace(eligible, lane, -1);
			}
		}
		target: physics.Restitution_Target_Wide = physics.restitution_capture_target(
			velocity, depth, coefficient, threshold, eligible,
		);
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			v: f32 = simd.extract(velocity, lane);
			e: f32 = simd.extract(coefficient, lane);
			expected_mask: i32 = 0;
			expected_target: f32 = 0;
			if simd.extract(eligible, lane) != 0 && e > 0 &&
			simd.extract(depth, lane) >= 0 && v < -simd.extract(threshold, lane)
			{
				expected_mask = -1;
				expected_target = -e * v;
			}
			testing.expect_value(t, simd.extract(target.active_mask, lane), expected_mask);
			testing.expect_value(t, simd.extract(target.velocity, lane), expected_target);
		}
	}
}

@(test)
restitution_capture_contact_velocity_and_reversed_pair :: proc (t: ^testing.T)
{
	bodies: [2]physics.Constraint_Contact_Body_Wide;
	bodies[0].linear_velocity.y = util.F32x8(-3);
	bodies[0].angular_velocity.z = util.F32x8(-2);
	bodies[1].linear_velocity.y = util.F32x8(1);
	bodies[1].angular_velocity.z = util.F32x8(3);
	normal: util.Vector3_Wide = {y=util.F32x8(1)};
	offset_a: util.Vector3_Wide = {x=util.F32x8(2)};
	offset_b: util.Vector3_Wide = {x=util.F32x8(1)};
	// dot(vA + wA x rA - vB - wB x rB, n) = -3 -4 -1 -3 = -11
	target: physics.Restitution_Target_Wide = physics.restitution_capture_normal(
		&bodies[0], normal, offset_a, offset_b, util.F32x8(0),
		util.F32x8(0.5), util.F32x8(1), util.I32x8(-1), 2,
	);
	testing.expect_value(t, target.velocity, util.F32x8(5.5));
	testing.expect_value(t, target.active_mask, util.I32x8(-1));
	reversed: [2]physics.Constraint_Contact_Body_Wide = {bodies[1], bodies[0]};
	other: physics.Restitution_Target_Wide = physics.restitution_capture_normal(
		&reversed[0], util.vector3_wide_negate(normal), offset_b, offset_a,
		util.F32x8(0), util.F32x8(0.5), util.F32x8(1), util.I32x8(-1), 2,
	);
	testing.expect_value(t, other, target);
	one: physics.Restitution_Target_Wide = physics.restitution_capture_normal(
		&bodies[0], normal, offset_a, {}, util.F32x8(0),
		util.F32x8(0.5), util.F32x8(1), util.I32x8(-1), 1,
	);
	testing.expect_value(t, one.velocity, util.F32x8(3.5));
}

restitution_test_normal_impacts :: proc (t: ^testing.T, $body_count: int)
{
	for coefficient in ([3]f32{0, 0.5, 1})
	{
		for lane_count in ([3]int{1, 5, 8})
		{
			bodies: [2]physics.Constraint_Contact_Body_Wide;
			bodies[0].inverse_mass = util.F32x8(1);
			bodies[0].linear_velocity.y = util.F32x8(-4);
			when body_count == 2
			{
				bodies[1].inverse_mass = util.F32x8(0.5);
				bodies[1].linear_velocity.y = util.F32x8(2);
			}
			normal: util.Vector3_Wide = {y=util.F32x8(1)};
			mask: util.I32x8 = lane_mask(lane_count);
			target: physics.Restitution_Target_Wide = physics.restitution_capture_normal(
				&bodies[0], normal, {}, {}, util.F32x8(0),
				util.F32x8(coefficient), util.F32x8(1), mask, body_count,
			);
			mass, bias, softness: util.F32x8;
			// a deliberately soft normal must not attenuate an enabled impact.
			// e=0 follows the ordinary rigid-contact control used by this oracle
			scale: util.F32x8 = util.F32x8(0.5);
			soft: util.F32x8 = util.F32x8(0.5);
			if coefficient == 0
			{
				scale = util.F32x8(1);
				soft = util.F32x8(0);
			}
			mass, bias, softness = physics.restitution_prepare_normal(
				&bodies[0], normal, {}, {}, util.F32x8(0), util.F32x8(10),
				scale, util.F32x8(2), soft, 60, &target, body_count,
			);
			impulse: util.F32x8 = util.wide_select_f32(mask, util.F32x8(1), util.F32x8(0));
			// capture precedes this retained warmstart impulse. recomputing the
			// target after warmstart would violate the analytic result below
			physics.constraint_contact_penetration_apply(
				&bodies[0], normal, {}, {}, impulse, mask, body_count,
			);
			for _ in 0 ..< 16
			{
				physics.constraint_contact_penetration_solve_cached(
					&bodies[0], normal, {}, {}, softness, &impulse, mask, body_count, &mass, &bias,
				);
				for lane in 0 ..< util.PRODUCTION_LANE_COUNT
				{
					if lane < lane_count
					{
						velocity: f32 = simd.extract(bodies[0].linear_velocity.y, lane);
						expected: f32 = coefficient * 4;
						when body_count == 2
						{
							velocity -= simd.extract(bodies[1].linear_velocity.y, lane);
							expected = coefficient * 6;
						}
						testing.expect(t, abs(velocity - expected) < 0.00001);
					}
					else
					{
						testing.expect_value(t, simd.extract(bodies[0].linear_velocity.y, lane), f32(-4));
						testing.expect_value(t, simd.extract(impulse, lane), f32(0));
					}
				}
			}
		}
	}
}

@(test)
restitution_normal_response_preserves_impact_ratio :: proc (t: ^testing.T)
{
	restitution_test_normal_impacts(t, 1);
	restitution_test_normal_impacts(t, 2);
}

@(test)
restitution_zero_target_preserves_compliance_and_speculation :: proc (t: ^testing.T)
{
	bodies: [2]physics.Constraint_Contact_Body_Wide;
	bodies[0].inverse_mass = util.F32x8(1);
	bodies[0].linear_velocity.y = util.F32x8(-4);
	bodies[1].inverse_mass = util.F32x8(0.5);
	normal: util.Vector3_Wide = {y=util.F32x8(1)};
	depth: util.F32x8 = {-0.5, -0.125, 0, 0.1, 0.5, 0, -0.25, 1};
	target: physics.Restitution_Target_Wide;
	mass, bias, softness: util.F32x8;
	mass, bias, softness = physics.restitution_prepare_normal(
		&bodies[0], normal, {}, {}, depth, util.F32x8(10), util.F32x8(0.5),
		util.F32x8(2), util.F32x8(0.5), 60, &target, 2,
	);
	old_mass, old_bias: util.F32x8;
	old_mass, old_bias = physics.constraint_contact_penetration_prepare_solve(
		&bodies[0], {}, {}, depth, util.F32x8(10), util.F32x8(0.5), util.F32x8(2), 60, 2,
	);
	testing.expect_value(t, mass, old_mass);
	testing.expect_value(t, bias, old_bias);
	testing.expect_value(t, softness, util.F32x8(0.5));
	testing.expect(t, simd.extract(bias, 0) < 0);
	// penetration recovery wins over a smaller impact target, rather than adding
	target.velocity = util.F32x8(0.5);
	target.active_mask = util.I32x8(-1);
	mass, bias, softness = physics.restitution_prepare_normal(
		&bodies[0], normal, {}, {}, util.F32x8(1), util.F32x8(10), util.F32x8(0.5),
		util.F32x8(2), util.F32x8(0.5), 60, &target, 2,
	);
	testing.expect_value(t, bias, util.F32x8(2));
	testing.expect_value(t, softness, util.F32x8(0));
}

restitution_test_selected_contact_kernel :: proc (
	t: ^testing.T, $body_count, $contact_count: int, $kind: physics.Contact_Constraint_Kind,
)
{
	for coefficient in ([2]f32{0.5, 1})
	{
		for count in ([3]int{1, 5, 8})
		{
			bodies: [2]physics.Constraint_Kernel_Body_Wide;
			bodies[0].inverse_mass = util.F32x8(1);
			bodies[0].inverse_inertia.xx = util.F32x8(1);
			bodies[0].inverse_inertia.yy = util.F32x8(1);
			bodies[0].inverse_inertia.zz = util.F32x8(1);
			bodies[0].linear_velocity.y = util.F32x8(-4);
			when body_count == 2
			{
				bodies[1].inverse_mass = util.F32x8(1);
				bodies[1].inverse_inertia = bodies[0].inverse_inertia;
			}
			prestep: [40]util.F32x8;
			impulses: [12]util.F32x8;
			targets: [4]physics.Restitution_Target_Wide;
			mask: util.I32x8 = lane_mask(count);
			normal: util.Vector3_Wide = {y=util.F32x8(1)};
			material: ^physics.Contact_Material_Wide;
			when kind == .Convex
			{
				normal_index: int = contact_count * 4 + (body_count - 1) * 3;
				prestep[normal_index + 1] = util.F32x8(1);
				material = (^physics.Contact_Material_Wide)(&prestep[normal_index + 3]);
			}
			else
			{
				material = (^physics.Contact_Material_Wide)(&prestep[0]);
				for index in 0 ..< contact_count
				{
					prestep[4 + (body_count - 1) * 3 + index * 7 + 5] = util.F32x8(1);
				}
			}
			material.spring_settings = {angular_frequency=util.F32x8(30), twice_damping_ratio=util.F32x8(2)};
			for index in 0 ..< contact_count
			{
				targets[index] = physics.restitution_capture_normal(
					&bodies[0], normal, {}, {}, util.F32x8(0), util.F32x8(coefficient),
					util.F32x8(1), mask, body_count,
				);
			}
			for _ in 0 ..< 8
			{
				when kind == .Convex
				{
					physics.constraint_contact_convex_kernel_response(
						&prestep[0], &bodies[0], 1.0/60.0, 60, &impulses[0], mask, .Solve,
						body_count, contact_count, &targets[0], .Restitution,
					);
				}
				else
				{
					physics.constraint_contact_nonconvex_kernel_response(
						&prestep[0], &bodies[0], 1.0/60.0, 60, &impulses[0], mask, .Solve,
						body_count, contact_count, &targets[0], .Restitution,
					);
				}
				for lane in 0 ..< count
				{
					velocity: f32 = simd.extract(bodies[0].linear_velocity.y, lane);
					when body_count == 2
					{
						velocity -= simd.extract(bodies[1].linear_velocity.y, lane);
					}
					testing.expect(t, abs(velocity - 4 * coefficient) < 0.0001);
				}
			}
		}
	}
}

@(test)
restitution_selected_contact_families :: proc (t: ^testing.T)
{
	restitution_test_selected_contact_kernel(t, 1, 1, .Convex);
	restitution_test_selected_contact_kernel(t, 1, 2, .Convex);
	restitution_test_selected_contact_kernel(t, 1, 3, .Convex);
	restitution_test_selected_contact_kernel(t, 1, 4, .Convex);
	restitution_test_selected_contact_kernel(t, 2, 1, .Convex);
	restitution_test_selected_contact_kernel(t, 2, 2, .Convex);
	restitution_test_selected_contact_kernel(t, 2, 3, .Convex);
	restitution_test_selected_contact_kernel(t, 2, 4, .Convex);
	restitution_test_selected_contact_kernel(t, 1, 2, .Nonconvex);
	restitution_test_selected_contact_kernel(t, 1, 3, .Nonconvex);
	restitution_test_selected_contact_kernel(t, 1, 4, .Nonconvex);
	restitution_test_selected_contact_kernel(t, 2, 2, .Nonconvex);
	restitution_test_selected_contact_kernel(t, 2, 3, .Nonconvex);
	restitution_test_selected_contact_kernel(t, 2, 4, .Nonconvex);
}

restitution_test_cached_contact_response :: proc (
	t: ^testing.T, $body_count, $contact_count: int,
)
{
	for coefficient in ([3]f32{0, 0.5, 1})
	{
		for count in ([3]int{1, 5, 8})
		{
			for target_mode in 0 ..< 3
			{
				bodies: [2]physics.Constraint_Kernel_Body_Wide;
				bodies[0].inverse_mass = util.F32x8(1);
				bodies[0].inverse_inertia.xx = util.F32x8(1);
				bodies[0].inverse_inertia.yy = util.F32x8(1);
				bodies[0].inverse_inertia.zz = util.F32x8(1);
				bodies[0].linear_velocity = {x=util.F32x8(0.75), y=util.F32x8(-4)};
				bodies[0].angular_velocity.z = util.F32x8(0.2);
				when body_count == 2
				{
					bodies[1].inverse_mass = util.F32x8(0.5);
					bodies[1].inverse_inertia = bodies[0].inverse_inertia;
					bodies[1].linear_velocity.y = util.F32x8(0.5);
				}
				prestep: [40]util.F32x8;
				contacts: [^]physics.Convex_Contact_Wide = ([^]physics.Convex_Contact_Wide)(&prestep[0]);
				normal_index: int = contact_count*4 + (body_count-1)*3;
				prestep[normal_index+1] = util.F32x8(1);
				material: ^physics.Contact_Material_Wide = (^physics.Contact_Material_Wide)(&prestep[normal_index+3]);
				material.spring_settings = {angular_frequency=util.F32x8(30), twice_damping_ratio=util.F32x8(2)};
				material.maximum_recovery_velocity = util.F32x8(2);
				material.friction_coefficient = util.F32x8(0.3);
				mask: util.I32x8 = lane_mask(count);
				targets: [4]physics.Restitution_Target_Wide;
				for contact_index in 0 ..< contact_count
				{
					contacts[contact_index].offset_a.x = util.F32x8(f32(contact_index)*0.15);
					contacts[contact_index].depth = util.F32x8(0.01);
					eligible: util.I32x8 = mask;
					for lane in 0 ..< count
					{
						if target_mode == 1 && (lane+contact_index)%2 == 0
						{
							eligible = simd.replace(eligible, lane, 0);
						}
						if target_mode == 2 && (lane+contact_index)%2 == 0
						{
							contacts[contact_index].depth = simd.replace(contacts[contact_index].depth, lane, -0.02);
						}
					}
					targets[contact_index] = physics.restitution_capture_normal(
						&bodies[0], {y=util.F32x8(1)}, contacts[contact_index].offset_a,
						contacts[contact_index].offset_a, contacts[contact_index].depth,
						util.F32x8(coefficient), util.F32x8(1), eligible, body_count,
					);
				}
				cached_bodies: [2]physics.Constraint_Kernel_Body_Wide = bodies;
				impulses, cached_impulses: [12]util.F32x8;
				// a retained normal impulse exercises capture-before-warmstart
				impulses[2] = util.wide_select_f32(mask, util.F32x8(0.25), util.F32x8(0));
				cached_impulses = impulses;
				when contact_count == 4
				{
					data: physics.Contact_4_Solve_Data;
					physics.constraint_contact_convex_warmstart_cached(&prestep[0], &cached_bodies[0], &cached_impulses[0], mask, body_count, &data);
					physics.constraint_contact_convex_prepare_solve_response(&prestep[0], &cached_bodies[0], 1.0/60, 60, body_count, contact_count, &data, &targets[0], .Restitution);
					physics.constraint_contact_convex_kernel(&prestep[0], &bodies[0], 1.0/60, 60, &impulses[0], mask, .Warmstart, body_count, contact_count);
					for _ in 0 ..< 8
					{
						physics.constraint_contact_convex_solve_cached_response(&prestep[0], &cached_bodies[0], &cached_impulses[0], mask, body_count, contact_count, &data, 60, &targets[0], .Restitution);
						physics.constraint_contact_convex_kernel_response(&prestep[0], &bodies[0], 1.0/60, 60, &impulses[0], mask, .Solve, body_count, contact_count, &targets[0], .Restitution);
						restitution_expect_cached_state(t, &bodies, &cached_bodies, &impulses, &cached_impulses, body_count, contact_count+3);
					}
				}
				else
				{
					data: physics.Contact_Convex_Solve_Data(contact_count);
					physics.constraint_contact_convex_kernel(&prestep[0], &bodies[0], 1.0/60, 60, &impulses[0], mask, .Warmstart, body_count, contact_count);
					physics.constraint_contact_convex_kernel(&prestep[0], &cached_bodies[0], 1.0/60, 60, &cached_impulses[0], mask, .Warmstart, body_count, contact_count);
					physics.constraint_contact_convex_prepare_solve_response(&prestep[0], &cached_bodies[0], 1.0/60, 60, body_count, contact_count, &data, &targets[0], .Restitution);
					for _ in 0 ..< 8
					{
						physics.constraint_contact_convex_solve_cached_response(&prestep[0], &cached_bodies[0], &cached_impulses[0], mask, body_count, contact_count, &data, 60, &targets[0], .Restitution);
						physics.constraint_contact_convex_kernel_response(&prestep[0], &bodies[0], 1.0/60, 60, &impulses[0], mask, .Solve, body_count, contact_count, &targets[0], .Restitution);
						restitution_expect_cached_state(t, &bodies, &cached_bodies, &impulses, &cached_impulses, body_count, contact_count+3);
					}
				}
			}
		}
	}
}

restitution_expect_cached_state :: proc (
	t: ^testing.T, expected, actual: ^[2]physics.Constraint_Kernel_Body_Wide,
	expected_impulses, actual_impulses: ^[12]util.F32x8, body_count, impulse_count: int,
)
{
	for body_index in 0 ..< body_count
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			a: ^physics.Constraint_Kernel_Body_Wide = &expected[body_index];
			b: ^physics.Constraint_Kernel_Body_Wide = &actual[body_index];
			for difference in ([6]util.F32x8{
					a.linear_velocity.x-b.linear_velocity.x, a.linear_velocity.y-b.linear_velocity.y,
					a.linear_velocity.z-b.linear_velocity.z, a.angular_velocity.x-b.angular_velocity.x,
					a.angular_velocity.y-b.angular_velocity.y, a.angular_velocity.z-b.angular_velocity.z,
			})
			{
				testing.expect(t, abs(simd.extract(difference, lane)) < 0.0001);
			}
		}
	}
	for index in 0 ..< impulse_count
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			testing.expect(t, abs(simd.extract(expected_impulses[index]-actual_impulses[index], lane)) < 0.0001);
		}
	}
}

@(test)
restitution_cached_convex_families_match_uncached :: proc (t: ^testing.T)
{
	restitution_test_cached_contact_response(t, 1, 1);
	restitution_test_cached_contact_response(t, 1, 2);
	restitution_test_cached_contact_response(t, 1, 3);
	restitution_test_cached_contact_response(t, 1, 4);
	restitution_test_cached_contact_response(t, 2, 1);
	restitution_test_cached_contact_response(t, 2, 2);
	restitution_test_cached_contact_response(t, 2, 3);
	restitution_test_cached_contact_response(t, 2, 4);
}

restitution_test_cached_work_range :: proc (t: ^testing.T, $body_count, $contact_count: int)
{
	BODY_COUNT :: 48;
	PRESTEP_VECTORS :: contact_count*4+(body_count-1)*3+7;
	IMPULSE_VECTORS :: contact_count+3;
	state_storage: struct #align(64)
	{
		values: [BODY_COUNT]physics.Body_Dynamics,
	};
	states: ^[BODY_COUNT]physics.Body_Dynamics = &state_storage.values;
	for index in 0 ..< BODY_COUNT
	{
		states[index].inertia.world.inverse_mass = 1;
		states[index].motion.pose.orientation.w = 1;
		if index < 24
		{
			states[index].motion.velocity.linear.y = -4;
		}
	}
	sets: [1]physics.Body_Set;
	sets[0].dynamics_state = {memory=&states[0], length=BODY_COUNT};
	sets[0].count = BODY_COUNT;
	bodies: physics.Bodies = {sets={memory=&sets[0], length=1}};
	solver: physics.Solver = {bodies=&bodies};
	job: physics.Solver_Substep_Job = {solver=&solver, dt=1.0/60, inverse_dt=60, phase=.First};
	prestep: [3*PRESTEP_VECTORS]util.F32x8;
	impulses: [3*IMPULSE_VECTORS]util.F32x8;
	references: [3*body_count]util.I32x8;
	for bundle in 0 ..< 3
	{
		for body in 0 ..< body_count
		{
			for lane in 0 ..< 8
			{
				references[bundle*body_count+body] = simd.replace(references[bundle*body_count+body], lane, i32(body*24+bundle*8+lane));
			}
		}
		normal_index: int = bundle*PRESTEP_VECTORS+contact_count*4+(body_count-1)*3;
		prestep[normal_index+1] = util.F32x8(1);
		material: ^physics.Contact_Material_Wide = (^physics.Contact_Material_Wide)(&prestep[normal_index+3]);
		material.spring_settings = {angular_frequency=util.F32x8(30), twice_damping_ratio=util.F32x8(2)};
	}
	type_batch: physics.Type_Batch = {
		body_count=i32(body_count), count=17,
		prestep_bundle_size=i32(PRESTEP_VECTORS*size_of(util.F32x8)),
		impulse_bundle_size=i32(IMPULSE_VECTORS*size_of(util.F32x8)),
		body_references={memory=([^]u8)(&references[0]), length=size_of(references)},
		prestep_data={memory=([^]u8)(&prestep[0]), length=size_of(prestep)},
		accumulated_impulses={memory=([^]u8)(&impulses[0]), length=size_of(impulses)},
	};
	// a nonzero start proves targets are relative to this work range, not the
	// complete type batch. bundle zero and inactive tail bodies must stay untouched
	block: physics.Solver_Work_Block = {start_bundle=1, end_bundle=3};
	targets: [2*contact_count]physics.Restitution_Target_Wide;
	for bundle in 0 ..< 2
	{
		mask: util.I32x8 = lane_mask(8 if bundle == 0 else 1);
		for contact in 0 ..< contact_count
		{
			targets[bundle*contact_count+contact] = {velocity=util.F32x8(2 if bundle == 0 else 4), active_mask=mask};
		}
	}
	when contact_count == 4
	{
		data: [2]physics.Contact_4_Solve_Data;
		testing.expect_value(t, physics.solver_execute_convex_contact_work_block_stage_cached_response(
				&job, &type_batch, &block, 0, .Warmstart, body_count, contact_count, .Never, &data[0], &targets[0], .Restitution), physics.Physics_Status.Ok);
		for _ in 0 ..< 8
		{
			testing.expect_value(t, physics.solver_execute_convex_contact_work_block_stage_cached_response(
					&job, &type_batch, &block, 0, .Solve, body_count, contact_count, .Never, &data[0], &targets[0], .Restitution), physics.Physics_Status.Ok);
		}
	}
	else
	{
		data: [2]physics.Contact_Convex_Solve_Data(contact_count);
		testing.expect_value(t, physics.solver_execute_convex_contact_work_block_stage_cached_response(
				&job, &type_batch, &block, 0, .Warmstart, body_count, contact_count, .Never, &data[0], &targets[0], .Restitution), physics.Physics_Status.Ok);
		for _ in 0 ..< 8
		{
			testing.expect_value(t, physics.solver_execute_convex_contact_work_block_stage_cached_response(
					&job, &type_batch, &block, 0, .Solve, body_count, contact_count, .Never, &data[0], &targets[0], .Restitution), physics.Physics_Status.Ok);
		}
	}
	for index in 0 ..< 24
	{
		velocity: f32 = states[index].motion.velocity.linear.y;
		when body_count == 2
		{
			velocity -= states[index+24].motion.velocity.linear.y;
		}
		if index >= 8 && index < 17
		{
			expected: f32 = 2 if index < 16 else 4;
			testing.expect(t, abs(velocity-expected) < 0.0001);
		}
		else
		{
			testing.expect_value(t, velocity, f32(-4));
		}
	}
}

@(test)
restitution_cached_work_ranges_and_tail_preservation :: proc (t: ^testing.T)
{
	restitution_test_cached_work_range(t, 1, 1);
	restitution_test_cached_work_range(t, 1, 2);
	restitution_test_cached_work_range(t, 1, 3);
	restitution_test_cached_work_range(t, 1, 4);
	restitution_test_cached_work_range(t, 2, 1);
	restitution_test_cached_work_range(t, 2, 2);
	restitution_test_cached_work_range(t, 2, 3);
	restitution_test_cached_work_range(t, 2, 4);
}

// compare the physical J^T lambda with the original registered warmstart's
// velocity delta under each body's actual anisotropic inverse inertia. all
// masks, orientations, lever arms and original scalar impulse lanes are kept
reaction_exercise_description :: proc (
	t: ^testing.T, registry: ^physics.Constraint_Type_Registry, description: ^$T,
)
{
	for lane_count in ([3]int{1, 5, 8})
	{
		source: Kernel_Run_Result;
		record: ^physics.Constraint_Type_Record = kernel_initialize(t, registry, description, lane_count, &source);
		testing.expect_value(t, physics.constraint_builtin_reaction_support(record.type_id), physics.Reference_State.Present);
		for body in 0 ..< int(record.body_count)
		{
			source.bodies[body].inverse_inertia.yx = util.F32x8(0.125);
			source.bodies[body].inverse_inertia.zx = util.F32x8(-0.0625);
			source.bodies[body].inverse_inertia.zy = util.F32x8(0.1);
			for lane in 0 ..< 8
			{
				util.quaternion_wide_write_slot(&source.bodies[body].orientation, lane,
					util.quaternion_from_axis_angle({0, 0, 1}, 0.13*f32(body+lane+1)));
			}
		}
		physical: [4]physics.Constraint_Kernel_Body_Wide = source.bodies;
		original: [4]physics.Constraint_Kernel_Body_Wide = source.bodies;
		for index in 0 ..< int(record.body_count)
		{
			physical[index].linear_velocity = {};
			physical[index].angular_velocity = {};
			physical[index].inverse_mass = util.F32x8(1);
			physical[index].inverse_inertia = {xx=util.F32x8(1), yy=util.F32x8(1), zz=util.F32x8(1)};
		}
		saved_prestep: Kernel_Prestep_Storage = source.prestep;
		saved_impulses: Kernel_Impulse_Storage = source.impulses;
		mask: util.I32x8 = lane_mask(lane_count);
		physics.constraint_builtin_reaction_kernel(record.type_id, &source.prestep, &source.impulses, &physical, mask);
		record.prestep_warmstart_solve(&source.prestep, &original, 1.0/64, 64, &source.impulses, mask, .Warmstart);
		testing.expect_value(t, bytes_match(&saved_prestep, &source.prestep, int(record.prestep_bundle_size)), physics.Reference_State.Present);
		testing.expect_value(t, bytes_match(&saved_impulses, &source.impulses, int(record.impulse_bundle_size)), physics.Reference_State.Present);
		for body in 0 ..< int(record.body_count)
		{
			expected_linear: util.Vector3_Wide = util.vector3_wide_scale(physical[body].linear_velocity, source.bodies[body].inverse_mass);
			expected_angular: util.Vector3_Wide = util.symmetric3x3_wide_transform(physical[body].angular_velocity, source.bodies[body].inverse_inertia);
			actual_linear: util.Vector3_Wide = util.vector3_wide_subtract(original[body].linear_velocity, source.bodies[body].linear_velocity);
			actual_angular: util.Vector3_Wide = util.vector3_wide_subtract(original[body].angular_velocity, source.bodies[body].angular_velocity);
			for lane in 0 ..< 8
			{
				e_linear: util.Vector3 = util.vector3_wide_read_slot(expected_linear, lane);
				e_angular: util.Vector3 = util.vector3_wide_read_slot(expected_angular, lane);
				a_linear: util.Vector3 = util.vector3_wide_read_slot(actual_linear, lane);
				a_angular: util.Vector3 = util.vector3_wide_read_slot(actual_angular, lane);
				testing.expect(t, abs(e_linear.x-a_linear.x) < 0.00002 && abs(e_linear.y-a_linear.y) < 0.00002 && abs(e_linear.z-a_linear.z) < 0.00002);
				testing.expect(t, abs(e_angular.x-a_angular.x) < 0.00002 && abs(e_angular.y-a_angular.y) < 0.00002 && abs(e_angular.z-a_angular.z) < 0.00002);
				if lane >= lane_count
				{
					testing.expect_value(t, util.vector3_wide_read_slot(physical[body].linear_velocity, lane), util.Vector3{});
					testing.expect_value(t, util.vector3_wide_read_slot(physical[body].angular_velocity, lane), util.Vector3{});
				}
			}
		}
	}
}

@(test)
one_body_noncontact_reaction_matches_original_jacobians :: proc(t: ^testing.T)
{
	registry: physics.Constraint_Type_Registry = registry_fixture(t);
	spring: physics.Spring_Settings = valid_spring();
	servo: physics.Servo_Settings = valid_servo();
	motor: physics.Motor_Settings = valid_motor();
	q: util.Quaternion = util.quaternion_identity();
	d0: physics.One_Body_Angular_Servo = physics.One_Body_Angular_Servo{q, spring, servo};
	reaction_exercise_description(t, &registry, &d0);
	d1: physics.One_Body_Angular_Motor = physics.One_Body_Angular_Motor{{1, 2, 3}, motor};
	reaction_exercise_description(t, &registry, &d1);
	d2: physics.One_Body_Linear_Servo = physics.One_Body_Linear_Servo{{0.25, 0.5, 0.75}, {2, 1, -1}, spring, servo};
	reaction_exercise_description(t, &registry, &d2);
	d3: physics.One_Body_Linear_Motor = physics.One_Body_Linear_Motor{{0.25, 0.5, 0.75}, {1, -2, 3}, motor};
	reaction_exercise_description(t, &registry, &d3);
}

@(test)
two_body_noncontact_reaction_matches_original_jacobians :: proc(t: ^testing.T)
{
	registry: physics.Constraint_Type_Registry = registry_fixture(t);
	spring: physics.Spring_Settings = valid_spring();
	servo: physics.Servo_Settings = valid_servo();
	motor: physics.Motor_Settings = valid_motor();
	q: util.Quaternion = util.quaternion_identity();
	d00: physics.Ball_Socket = physics.Ball_Socket{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, spring};
	reaction_exercise_description(t, &registry, &d00);
	d01: physics.Angular_Hinge = physics.Angular_Hinge{unit_x(), unit_y(), spring};
	reaction_exercise_description(t, &registry, &d01);
	d02: physics.Angular_Swivel_Hinge = physics.Angular_Swivel_Hinge{unit_x(), unit_y(), spring};
	reaction_exercise_description(t, &registry, &d02);
	d03: physics.Swing_Limit = physics.Swing_Limit{unit_x(), unit_y(), -0.5, spring};
	reaction_exercise_description(t, &registry, &d03);
	d04: physics.Twist_Servo = physics.Twist_Servo{q, q, 0.5, spring, servo};
	reaction_exercise_description(t, &registry, &d04);
	d05: physics.Twist_Limit = physics.Twist_Limit{q, q, -1, 1, spring};
	reaction_exercise_description(t, &registry, &d05);
	d06: physics.Twist_Motor = physics.Twist_Motor{unit_x(), unit_y(), 2, motor};
	reaction_exercise_description(t, &registry, &d06);
	d07: physics.Angular_Servo = physics.Angular_Servo{q, spring, servo};
	reaction_exercise_description(t, &registry, &d07);
	d08: physics.Angular_Motor = physics.Angular_Motor{{1, 2, 3}, motor};
	reaction_exercise_description(t, &registry, &d08);
	d09: physics.Weld = physics.Weld{{0.1, 0.2, 0.3}, q, spring};
	reaction_exercise_description(t, &registry, &d09);
	d10: physics.Distance_Servo = physics.Distance_Servo{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, 2, servo, spring};
	reaction_exercise_description(t, &registry, &d10);
	d11: physics.Distance_Limit = physics.Distance_Limit{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, 0.5, 2, spring};
	reaction_exercise_description(t, &registry, &d11);
	d12: physics.Center_Distance_Constraint = physics.Center_Distance_Constraint{2, spring};
	reaction_exercise_description(t, &registry, &d12);
	d13: physics.Point_On_Line_Servo = physics.Point_On_Line_Servo{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, unit_x(), servo, spring};
	reaction_exercise_description(t, &registry, &d13);
	d14: physics.Linear_Axis_Servo = physics.Linear_Axis_Servo{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, unit_x(), 0.75, servo, spring};
	reaction_exercise_description(t, &registry, &d14);
	d15: physics.Linear_Axis_Motor = physics.Linear_Axis_Motor{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, unit_x(), 2, motor};
	reaction_exercise_description(t, &registry, &d15);
	d16: physics.Linear_Axis_Limit = physics.Linear_Axis_Limit{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, unit_x(), -0.5, 2, spring};
	reaction_exercise_description(t, &registry, &d16);
	d17: physics.Angular_Axis_Motor = physics.Angular_Axis_Motor{unit_x(), 2, motor};
	reaction_exercise_description(t, &registry, &d17);
	d18: physics.Swivel_Hinge = physics.Swivel_Hinge{{0.1, 0.2, 0.3}, unit_x(), {0.4, 0.5, 0.6}, unit_y(), spring};
	reaction_exercise_description(t, &registry, &d18);
	d19: physics.Hinge = physics.Hinge{{0.1, 0.2, 0.3}, unit_x(), {0.4, 0.5, 0.6}, unit_y(), spring};
	reaction_exercise_description(t, &registry, &d19);
	d20: physics.Ball_Socket_Motor = physics.Ball_Socket_Motor{{0.4, 0.5, 0.6}, {1, 2, 3}, motor};
	reaction_exercise_description(t, &registry, &d20);
	d21: physics.Ball_Socket_Servo = physics.Ball_Socket_Servo{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, spring, servo};
	reaction_exercise_description(t, &registry, &d21);
	d22: physics.Angular_Axis_Gear_Motor = physics.Angular_Axis_Gear_Motor{unit_x(), 2, motor};
	reaction_exercise_description(t, &registry, &d22);
	d23: physics.Center_Distance_Limit = physics.Center_Distance_Limit{0.5, 2, spring};
	reaction_exercise_description(t, &registry, &d23);
}

@(test)
three_and_four_body_noncontact_reaction_matches_original_jacobians :: proc(t: ^testing.T)
{
	registry: physics.Constraint_Type_Registry = registry_fixture(t);
	area: physics.Area_Constraint = physics.Area_Constraint{2, valid_spring()};
	reaction_exercise_description(t, &registry, &area);
	volume: physics.Volume_Constraint = physics.Volume_Constraint{2, valid_spring()};
	reaction_exercise_description(t, &registry, &volume);
}

reaction_unit_bodies :: proc "contextless" () -> [4]physics.Constraint_Kernel_Body_Wide
{
	result: [4]physics.Constraint_Kernel_Body_Wide;
	for &body in result
	{
		body.orientation = util.quaternion_wide_broadcast(util.quaternion_identity());
		body.inverse_mass = util.F32x8(1);
		body.inverse_inertia = {xx=util.F32x8(1), yy=util.F32x8(1), zz=util.F32x8(1)};
	}
	return result;
}

reaction_expect_vector :: proc(t: ^testing.T, actual, expected: util.Vector3, tolerance: f32 = 0.00005)
{
	if abs(actual.x-expected.x) >= tolerance || abs(actual.y-expected.y) >= tolerance || abs(actual.z-expected.z) >= tolerance
	{
		testing.expect_value(t, actual, expected);
	}
}

@(test)
reaction_analytic_off_center_and_rotated_anchors :: proc(t: ^testing.T)
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide = reaction_unit_bodies();
	bodies[0].orientation = util.quaternion_wide_broadcast({z=0.7071067811865475, w=0.7071067811865475});
	prestep: physics.One_Body_Linear_Motor_Prestep = {local_offset=util.vector3_wide_broadcast({1, 0, 0})};
	impulse: util.Vector3_Wide = util.vector3_wide_broadcast({0, 0, 2});
	physics.constraint_builtin_reaction_kernel(physics.ONE_BODY_LINEAR_MOTOR_TYPE_ID, &prestep, &impulse, &bodies, lane_mask(1));
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].linear_velocity, 0), {0, 0, 2});
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].angular_velocity, 0), {2, 0, 0});
	bodies = reaction_unit_bodies();
	ball: physics.Ball_Socket_Prestep = {local_offset_a=util.vector3_wide_broadcast({1, 0, 0}), local_offset_b=util.vector3_wide_broadcast({0, 1, 0})};
	impulse = util.vector3_wide_broadcast({0, 2, 3});
	physics.constraint_builtin_reaction_kernel(physics.BALL_SOCKET_TYPE_ID, &ball, &impulse, &bodies, lane_mask(1));
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].linear_velocity, 0), {0, 2, 3});
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[1].linear_velocity, 0), {0, -2, -3});
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].angular_velocity, 0), {0, -3, 2});
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[1].angular_velocity, 0), {-3, 0, 0});
}

@(test)
reaction_area_volume_use_normalized_multibody_jacobians :: proc(t: ^testing.T)
{
	// avx rsqrt normalization intentionally retains the original approximate
	// Jacobian scaling. the independent exact formula uses its error bound
	bodies: [4]physics.Constraint_Kernel_Body_Wide = reaction_unit_bodies();
	bodies[1].position = util.vector3_wide_broadcast({2, 0, 0});
	bodies[2].position = util.vector3_wide_broadcast({0, 3, 0});
	area: physics.Area_Constraint_Prestep;
	impulse: util.F32x8 = util.F32x8(2);
	physics.constraint_builtin_reaction_kernel(physics.AREA_CONSTRAINT_TYPE_ID, &area, &impulse, &bodies, lane_mask(1));
	scale: f32 = f32(2/math.sqrt(26.0));
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].linear_velocity, 0), {-3*scale, -2*scale, 0}, 0.0005);
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[1].linear_velocity, 0), {3*scale, 0, 0}, 0.0005);
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[2].linear_velocity, 0), {0, 2*scale, 0}, 0.0005);
	bodies = reaction_unit_bodies();
	bodies[1].position = util.vector3_wide_broadcast({2, 0, 0});
	bodies[2].position = util.vector3_wide_broadcast({0, 3, 0});
	bodies[3].position = util.vector3_wide_broadcast({0, 0, 4});
	volume: physics.Volume_Constraint_Prestep;
	physics.constraint_builtin_reaction_kernel(physics.VOLUME_CONSTRAINT_TYPE_ID, &volume, &impulse, &bodies, lane_mask(1));
	scale = f32(2/math.sqrt(488.0));
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].linear_velocity, 0), {-12*scale, -8*scale, -6*scale}, 0.0005);
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[1].linear_velocity, 0), {12*scale, 0, 0}, 0.0005);
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[2].linear_velocity, 0), {0, 8*scale, 0}, 0.0005);
	reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[3].linear_velocity, 0), {0, 0, 6*scale}, 0.0005);
}

@(test)
reaction_axis_lock_all_selections_and_rotated_reference :: proc(t: ^testing.T)
{
	for axes in 1 ..< 64
	{
		bodies: [4]physics.Constraint_Kernel_Body_Wide = reaction_unit_bodies();
		prestep: physics.Body_Axis_Lock_Prestep;
		prestep.orientation = util.quaternion_wide_broadcast({z=0.7071067811865475, w=0.7071067811865475});
		impulses: physics.Body_Axis_Lock_Impulses;
		expected_linear, expected_angular: util.Vector3;
		for axis in 0 ..< 3
		{
			value: f32 = f32(axis+1);
			if axes & (1<<u32(axis)) != 0
			{
				prestep.linear[axis] = util.F32x8(1);
				switch axis
				{
					case 0:
					expected_linear.x=value;
					case 1:
					expected_linear.y=value;
					case 2:
					expected_linear.z=value;
				}
			}
			if axes & (1<<u32(axis+3)) != 0
			{
				prestep.angular[axis] = util.F32x8(1);
				switch axis
				{
					case 0:
					expected_angular.y=value;
					case 1:
					expected_angular.x=-value;
					case 2:
					expected_angular.z=value;
				}
			}
			impulses.linear[axis] = util.F32x8(value);
			impulses.angular[axis] = util.F32x8(value);
		}
		physics.constraint_builtin_reaction_kernel(physics.BODY_AXIS_LOCK_TYPE_ID, &prestep, &impulses, &bodies, lane_mask(5));
		for lane in 0 ..< 8
		{
			linear, angular: util.Vector3 = expected_linear, expected_angular;
			if lane>=5
			{
				linear={};
				angular={};
			}
			reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].linear_velocity, lane), linear);
			reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].angular_velocity, lane), angular);
		}
	}
}

@(test)
gear_solver_applies_only_incremental_impulses :: proc(t: ^testing.T)
{
	for ratio in ([2]f32{1, 2})
	{
		bodies: [4]physics.Constraint_Kernel_Body_Wide;
		for i in 0..<2
		{
			bodies[i].orientation.w=util.F32x8(1);
			bodies[i].inverse_mass=util.F32x8(1);
			bodies[i].inverse_inertia={xx=util.F32x8(1), yy=util.F32x8(1), zz=util.F32x8(1)};
		}
		bodies[0].angular_velocity.x=util.F32x8(1);
		bodies[1].angular_velocity.x=util.F32x8(-1);
		prestep:physics.Angular_Axis_Gear_Motor_Prestep={local_axis_a={x=util.F32x8(1)}, velocity_scale=util.F32x8(ratio), settings={maximum_force=util.F32x8(1000), damping=util.F32x8(1e30)}};
		impulses:util.F32x8;
		expected_impulse:f32=(-1-ratio)/(1+ratio*ratio);
		for iteration in 0..<8
		{
			physics.angular_axis_gear_motor_kernel(&prestep, &bodies, 0.25, 4, &impulses, util.I32x8(-1), .Solve);
			testing.expectf(t, abs(simd.extract(bodies[0].angular_velocity.x, 0)-(1+ratio*expected_impulse))<1e-5, "ratio=%v iteration=%v A=%v", ratio, iteration, simd.extract(bodies[0].angular_velocity.x, 0));
			testing.expectf(t, abs(simd.extract(bodies[1].angular_velocity.x, 0)-(-1-expected_impulse))<1e-5, "ratio=%v iteration=%v B=%v", ratio, iteration, simd.extract(bodies[1].angular_velocity.x, 0));
			testing.expectf(t, abs(simd.extract(impulses, 0)-expected_impulse)<1e-5, "ratio=%v iteration=%v lambda=%v", ratio, iteration, simd.extract(impulses, 0));
		}
	}
}

@(test)
gear_zero_force_removes_previous_warmstart_reaction :: proc(t: ^testing.T)
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide = reaction_unit_bodies();
	prestep: physics.Angular_Axis_Gear_Motor_Prestep = {
		local_axis_a={x=util.F32x8(1)}, velocity_scale=util.F32x8(2),
		settings={maximum_force={}, damping=util.F32x8(1e30)},
	};
	impulses: util.F32x8 = util.F32x8(0.25);
	physics.angular_axis_gear_motor_kernel(&prestep, &bodies, 0.25, 4, &impulses, lane_mask(5), .Warmstart);
	physics.angular_axis_gear_motor_kernel(&prestep, &bodies, 0.25, 4, &impulses, lane_mask(5), .Solve);
	for lane in 0 ..< 5
	{
		testing.expect_value(t, simd.extract(impulses, lane), f32(0));
		reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[0].angular_velocity, lane), {});
		reaction_expect_vector(t, util.vector3_wide_read_slot(bodies[1].angular_velocity, lane), {});
	}
}
