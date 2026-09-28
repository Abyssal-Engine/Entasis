#include "entasis.h"
#include "test_support.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static entasis_quaternion_t identity_orientation(void)
{
    const entasis_quaternion_t value = {0.0f, 0.0f, 0.0f, 1.0f};
    return value;
}

static int nearly_equal(float a, float b, float epsilon)
{
    return fabsf(a - b) <= epsilon;
}

static entasis_rigid_pose_t pose_at(float x, float y, float z)
{
    return entasis_pose((entasis_vector3_t){x, y, z}, identity_orientation());
}

static entasis_body_velocity_t zero_velocity(void)
{
    return entasis_velocity(
        (entasis_vector3_t){0.0f, 0.0f, 0.0f},
        (entasis_vector3_t){0.0f, 0.0f, 0.0f});
}

static int test_constructor_variants(void)
{
    const entasis_box_t box = entasis_box_half_extents(1.0f, 2.0f, 3.0f);
    const entasis_capsule_t capsule = entasis_capsule_half_length(0.5f, 2.0f);
    const entasis_cylinder_t cylinder = entasis_cylinder_half_length(0.75f, 3.0f);
    ENTASIS_TEST_CHECK(nearly_equal(box.half_width, 1.0f, 1.0e-6f));
    ENTASIS_TEST_CHECK(nearly_equal(box.half_height, 2.0f, 1.0e-6f));
    ENTASIS_TEST_CHECK(nearly_equal(box.half_length, 3.0f, 1.0e-6f));
    ENTASIS_TEST_CHECK(nearly_equal(capsule.half_length, 2.0f, 1.0e-6f));
    ENTASIS_TEST_CHECK(nearly_equal(cylinder.half_length, 3.0f, 1.0e-6f));

    const entasis_continuous_detection_t passive = entasis_ccd_passive();
    const entasis_continuous_detection_t continuous = entasis_ccd_continuous(0.01f, 0.02f);
    ENTASIS_TEST_CHECK(passive.mode == ENTASIS_CONTINUOUS_DETECTION_PASSIVE);
    ENTASIS_TEST_CHECK(continuous.mode == ENTASIS_CONTINUOUS_DETECTION_CONTINUOUS);
    ENTASIS_TEST_CHECK(nearly_equal(continuous.minimum_sweep_timestep, 0.01f, 1.0e-6f));
    ENTASIS_TEST_CHECK(nearly_equal(continuous.sweep_convergence_threshold, 0.02f, 1.0e-6f));

    const entasis_collision_filter_t default_filter = entasis_collision_filter_default();
    const entasis_collision_properties_t default_properties = entasis_collision_properties_default();
    const entasis_collision_filter_t roundtrip_filter = entasis_collision_properties_filter(default_properties);
    ENTASIS_TEST_CHECK(default_filter.layer == roundtrip_filter.layer);
    ENTASIS_TEST_CHECK(default_filter.mask == roundtrip_filter.mask);
    ENTASIS_TEST_CHECK(entasis_collision_layer_is_valid(default_filter.layer) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_layer_mask_all() == UINT64_MAX);

    const entasis_material_t material = entasis_material_default();
    const entasis_contact_material_t contact = entasis_material_to_contact(material);
    ENTASIS_TEST_CHECK(nearly_equal(contact.friction_coefficient, material.friction, 1.0e-6f));

    const entasis_layer_material_policy_t policy = entasis_layer_material_policy(NULL, NULL, (entasis_material_table_t){0});
    const entasis_narrow_policy_t narrow = entasis_narrow_policy_layers_materials(&policy);
    ENTASIS_TEST_CHECK(narrow.struct_size == sizeof(entasis_narrow_policy_t));
    ENTASIS_TEST_CHECK(narrow.struct_version == UINT32_C(1));
    ENTASIS_TEST_CHECK(narrow.kind == ENTASIS_NARROW_POLICY_LAYERS_MATERIALS);
    ENTASIS_TEST_CHECK(narrow.layer_material_policy == &policy);
    return 0;
}

static int test_body_and_static_mutators(void)
{
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere = entasis_sphere(0.5f);
    const entasis_box_t box = entasis_box(1.0f, 2.0f, 3.0f);
    entasis_shape_handle_t sphere_shape = entasis_shape_handle_invalid();
    entasis_shape_handle_t box_shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &sphere_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_BOX, &box, &box_shape, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_inertia_t sphere_inertia = {0};
    entasis_body_inertia_t box_inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(
                           ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1.0f, &sphere_inertia) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(
                           ENTASIS_SHAPE_TYPE_BOX, &box, 2.0f, &box_inertia) == ENTASIS_STATUS_OK);

    const entasis_activity_description_t activity = entasis_body_activity(0.01f, UINT8_C(16));
    entasis_body_description_t dynamic_description = entasis_body_dynamic(
        sphere_shape, sphere_inertia, pose_at(0.0f, 2.0f, 0.0f), zero_velocity(), activity);
    const entasis_body_description_t kinematic_description = entasis_body_kinematic(
        box_shape, pose_at(2.0f, 2.0f, 0.0f), zero_velocity(), activity);
    const entasis_body_description_t shapeless_description = entasis_body_shapeless(
        sphere_inertia, pose_at(-2.0f, 2.0f, 0.0f), zero_velocity(), activity);

    entasis_body_handle_t dynamic_body = entasis_body_handle_invalid();
    entasis_body_handle_t kinematic_body = entasis_body_handle_invalid();
    entasis_body_handle_t shapeless_body = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &dynamic_description, &dynamic_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &kinematic_description, &kinematic_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &shapeless_description, &shapeless_body, &diagnostic) == ENTASIS_STATUS_OK);

    dynamic_description.pose = pose_at(0.0f, 3.0f, 0.0f);
    ENTASIS_TEST_CHECK(entasis_body_apply(
                           &world, dynamic_body, &dynamic_description, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_rigid_pose_t body_pose = pose_at(0.0f, 4.0f, 0.0f);
    ENTASIS_TEST_CHECK(entasis_body_set_pose(
                           &world, dynamic_body, &body_pose, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_body_velocity_t updated_velocity = entasis_velocity(
        (entasis_vector3_t){1.0f, 2.0f, 3.0f},
        (entasis_vector3_t){0.0f, 1.0f, 0.0f});
    const entasis_activity_description_t default_activity = entasis_body_activity_default();
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(
                           &world, dynamic_body, &updated_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_inertia(
                           &world, dynamic_body, &box_inertia, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_activity(
                           &world, dynamic_body, &default_activity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_shape(
                           &world, dynamic_body, box_shape, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_collidable_description_t collidable = entasis_collidable(
        sphere_shape, entasis_ccd_continuous(0.001f, 0.002f), 0.0f, 2.0f);
    ENTASIS_TEST_CHECK(entasis_body_set_collidable(
                           &world, dynamic_body, &collidable, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_shape(
                           &world, shapeless_body, sphere_shape, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_state_t state = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(&world, dynamic_body, &state, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.collidable.continuity.mode == ENTASIS_CONTINUOUS_DETECTION_CONTINUOUS);
    ENTASIS_TEST_CHECK(nearly_equal(state.velocity.linear.x, 1.0f, 1.0e-6f));
    ENTASIS_TEST_CHECK(nearly_equal(state.pose.position.y, 4.0f, 1.0e-6f));

    entasis_static_description_t static_description = entasis_static_body(
        box_shape, pose_at(0.0f, -1.0f, 0.0f), entasis_ccd_discrete());
    entasis_static_handle_t static_handle = entasis_static_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_static_add(
                           &world, &static_description, ENTASIS_AWAKENING_NONE, &static_handle, &diagnostic) == ENTASIS_STATUS_OK);
    static_description.pose = pose_at(0.0f, -2.0f, 0.0f);
    ENTASIS_TEST_CHECK(entasis_static_apply(
                           &world, static_handle, &static_description, ENTASIS_AWAKENING_NONE, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_rigid_pose_t static_pose = pose_at(0.0f, -3.0f, 0.0f);
    ENTASIS_TEST_CHECK(entasis_static_set_pose(
                           &world, static_handle, &static_pose, ENTASIS_AWAKENING_NONE, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_set_shape(
                           &world, static_handle, sphere_shape, ENTASIS_AWAKENING_NONE, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_continuous_detection_t passive_continuity = entasis_ccd_passive();
    ENTASIS_TEST_CHECK(entasis_static_set_continuity(
                           &world, static_handle, &passive_continuity, ENTASIS_AWAKENING_NONE, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_static_state_t static_state = {0};
    ENTASIS_TEST_CHECK(entasis_static_get(
                           &world, static_handle, &static_state, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(static_state.continuity.mode == ENTASIS_CONTINUOUS_DETECTION_PASSIVE);
    ENTASIS_TEST_CHECK(nearly_equal(static_state.pose.position.y, -3.0f, 1.0e-6f));

    ENTASIS_TEST_CHECK(entasis_static_remove(
                           &world, static_handle, ENTASIS_AWAKENING_NONE, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_remove(&world, dynamic_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_remove(&world, kinematic_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_remove(&world, shapeless_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, sphere_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, box_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static void fill_children(
    entasis_compound_child_t children[2],
    entasis_shape_handle_t sphere,
    entasis_shape_handle_t box)
{
    children[0] = entasis_compound_child(sphere, pose_at(-1.0f, 0.0f, 0.0f));
    children[1] = entasis_compound_child(box, pose_at(1.0f, 0.0f, 0.0f));
}

static int test_compound_helpers_and_big_compounds(void)
{
    entasis_world_description_t description = entasis_world_description_default();
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere = entasis_sphere(0.5f);
    const entasis_box_t box = entasis_box(1.0f, 1.0f, 1.0f);
    entasis_shape_handle_t sphere_shape = entasis_shape_handle_invalid();
    entasis_shape_handle_t box_shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &sphere_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_BOX, &box, &box_shape, &diagnostic) == ENTASIS_STATUS_OK);

    const float masses[2] = {1.0f, 2.0f};
    entasis_compound_child_t children[2];
    fill_children(children, sphere_shape, box_shape);
    entasis_compound_builder_t builder = entasis_compound_builder(children, masses, UINT64_C(2));
    entasis_vector3_t center = {0};
    float inverse_mass = 0.0f;
    ENTASIS_TEST_CHECK(entasis_compound_center_of_mass(
                           builder, &center, &inverse_mass) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearly_equal(inverse_mass, 1.0f / 3.0f, 1.0e-5f));

    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_compound_inertia_weighted(
                           &world, builder, &inertia, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearly_equal(inertia.inverse_mass, 1.0f / 3.0f, 1.0e-5f));

    entasis_compound_child_t recenter_children[2];
    fill_children(recenter_children, sphere_shape, box_shape);
    entasis_vector3_t recentered = {0};
    ENTASIS_TEST_CHECK(entasis_compound_inertia_weighted_recenter(
                           &world,
                           entasis_compound_builder(recenter_children, masses, UINT64_C(2)),
                           &inertia,
                           &recentered,
                           &diagnostic) == ENTASIS_STATUS_OK);

    entasis_compound_child_t compound_children[2];
    fill_children(compound_children, sphere_shape, box_shape);
    entasis_compound_build_result_t compound = {0};
    ENTASIS_TEST_CHECK(entasis_compound_build_dynamic(
                           &world,
                           entasis_compound_builder(compound_children, masses, UINT64_C(2)),
                           ENTASIS_TRUE,
                           &compound,
                           &diagnostic) == ENTASIS_STATUS_OK);

    entasis_compound_child_t big_build_children[2];
    fill_children(big_build_children, sphere_shape, box_shape);
    entasis_compound_build_result_t big_built = {0};
    ENTASIS_TEST_CHECK(entasis_big_compound_build_dynamic(
                           &world,
                           entasis_compound_builder(big_build_children, masses, UINT64_C(2)),
                           ENTASIS_FALSE,
                           &big_built,
                           &diagnostic) == ENTASIS_STATUS_OK);

    entasis_compound_child_t imported_children[2];
    fill_children(imported_children, sphere_shape, box_shape);
    entasis_shape_handle_t imported_big = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_import_big_compound(
                           &world, imported_children, UINT64_C(2), &imported_big, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_shape_info_t info = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, compound.shape, &info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.type_id == ENTASIS_SHAPE_TYPE_COMPOUND);
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, big_built.shape, &info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.type_id == ENTASIS_SHAPE_TYPE_BIG_COMPOUND);
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, imported_big, &info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.type_id == ENTASIS_SHAPE_TYPE_BIG_COMPOUND);

    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, compound.shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, big_built.shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, imported_big, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, sphere_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, box_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_constructor_variants() == 0);
    ENTASIS_TEST_CHECK(test_body_and_static_mutators() == 0);
    ENTASIS_TEST_CHECK(test_compound_helpers_and_big_compounds() == 0);
    return 0;
}
