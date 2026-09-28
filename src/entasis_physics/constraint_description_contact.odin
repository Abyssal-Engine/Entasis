// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

CONTACT_1_ONE_BODY_TYPE_ID :: 0;
CONTACT_2_ONE_BODY_TYPE_ID :: 1;
CONTACT_3_ONE_BODY_TYPE_ID :: 2;
CONTACT_4_ONE_BODY_TYPE_ID :: 3;
CONTACT_1_TYPE_ID :: 4;
CONTACT_2_TYPE_ID :: 5;
CONTACT_3_TYPE_ID :: 6;
CONTACT_4_TYPE_ID :: 7;
CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID :: 8;
CONTACT_3_NONCONVEX_ONE_BODY_TYPE_ID :: 9;
CONTACT_4_NONCONVEX_ONE_BODY_TYPE_ID :: 10;
CONTACT_2_NONCONVEX_TYPE_ID :: 15;
CONTACT_3_NONCONVEX_TYPE_ID :: 16;
CONTACT_4_NONCONVEX_TYPE_ID :: 17;

Constraint_Contact_Data :: struct
{
	offset_a:          util.Vector3,
	penetration_depth: f32,
}

Nonconvex_Constraint_Contact_Data :: struct
{
	offset_a:          util.Vector3,
	normal:            util.Vector3,
	penetration_depth: f32,
}

Contact_Material_Properties :: struct
{
	friction_coefficient:     f32,
	spring_settings:          Spring_Settings,
	maximum_recovery_velocity: f32,
}

Nonconvex_Two_Body_Properties :: struct
{
	offset_b:                  util.Vector3,
	friction_coefficient:      f32,
	spring_settings:           Spring_Settings,
	maximum_recovery_velocity: f32,
}

Nonconvex_One_Body_Properties :: struct
{
	friction_coefficient:      f32,
	spring_settings:           Spring_Settings,
	maximum_recovery_velocity: f32,
}

Contact_1_One_Body :: struct
{
	contact_0: Constraint_Contact_Data,
	normal: util.Vector3,
	material: Contact_Material_Properties,
}

Contact_2_One_Body :: struct
{
	contact_0, contact_1: Constraint_Contact_Data,
	normal: util.Vector3,
	material: Contact_Material_Properties,
}

Contact_3_One_Body :: struct
{
	contact_0, contact_1, contact_2: Constraint_Contact_Data,
	normal: util.Vector3,
	material: Contact_Material_Properties,
}

Contact_4_One_Body :: struct
{
	contact_0, contact_1, contact_2, contact_3: Constraint_Contact_Data,
	normal: util.Vector3,
	material: Contact_Material_Properties,
}

Contact_1 :: struct
{
	contact_0: Constraint_Contact_Data,
	offset_b, normal: util.Vector3,
	material: Contact_Material_Properties,
}

Contact_2 :: struct
{
	contact_0, contact_1: Constraint_Contact_Data,
	offset_b, normal: util.Vector3,
	material: Contact_Material_Properties,
}

Contact_3 :: struct
{
	contact_0, contact_1, contact_2: Constraint_Contact_Data,
	offset_b, normal: util.Vector3,
	material: Contact_Material_Properties,
}

Contact_4 :: struct
{
	contact_0, contact_1, contact_2, contact_3: Constraint_Contact_Data,
	offset_b, normal: util.Vector3,
	material: Contact_Material_Properties,
}

Contact_2_Nonconvex :: struct
{
	common: Nonconvex_Two_Body_Properties,
	contact_0, contact_1: Nonconvex_Constraint_Contact_Data,
}

Contact_3_Nonconvex :: struct
{
	common: Nonconvex_Two_Body_Properties,
	contact_0, contact_1, contact_2: Nonconvex_Constraint_Contact_Data,
}

Contact_4_Nonconvex :: struct
{
	common: Nonconvex_Two_Body_Properties,
	contact_0, contact_1, contact_2, contact_3: Nonconvex_Constraint_Contact_Data,
}

Contact_2_Nonconvex_One_Body :: struct
{
	common: Nonconvex_One_Body_Properties,
	contact_0, contact_1: Nonconvex_Constraint_Contact_Data,
}

Contact_3_Nonconvex_One_Body :: struct
{
	common: Nonconvex_One_Body_Properties,
	contact_0, contact_1, contact_2: Nonconvex_Constraint_Contact_Data,
}

Contact_4_Nonconvex_One_Body :: struct
{
	common: Nonconvex_One_Body_Properties,
	contact_0, contact_1, contact_2, contact_3: Nonconvex_Constraint_Contact_Data,
}
