#include "entasis.h"
#include "test_support.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define BODY_COUNT UINT64_C(1024)
#define STATIC_COUNT UINT64_C(512)

static int nearf(float a, float b)
{
    return fabsf(a - b) <= 1.0e-5f;
}

static entasis_quaternion_t identity_orientation(void)
{
    return (entasis_quaternion_t){0.0f, 0.0f, 0.0f, 1.0f};
}

static entasis_rigid_pose_t pose_at(float x, float y, float z)
{
    return entasis_pose((entasis_vector3_t){x, y, z}, identity_orientation());
}

static entasis_world_description_t scene_world_description(void)
{
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    description.capacity.bodies = 2048;
    description.capacity.statics = 1024;
    description.capacity.shapes_per_type = 64;
    description.capacity.broad_phase_candidates = 4096;
    description.capacity.pairs = 4096;
    description.capacity.collision_child_pairs = 4096;
    return description;
}

static int test_factories_and_validation(void)
{
    const entasis_sphere_t sphere = entasis_sphere(2.0f);
    const entasis_box_t box = entasis_box(2.0f, 4.0f, 6.0f);
    const entasis_box_t half_box = entasis_box_half_extents(1.0f, 2.0f, 3.0f);
    const entasis_capsule_t capsule = entasis_capsule(0.5f, 4.0f);
    const entasis_capsule_t half_capsule = entasis_capsule_half_length(0.5f, 2.0f);
    const entasis_cylinder_t cylinder = entasis_cylinder(0.75f, 6.0f);
    const entasis_cylinder_t half_cylinder = entasis_cylinder_half_length(0.75f, 3.0f);
    const entasis_triangle_t triangle = entasis_triangle(
        (entasis_vector3_t){0.0f, 0.0f, 0.0f},
        (entasis_vector3_t){1.0f, 0.0f, 0.0f},
        (entasis_vector3_t){0.0f, 0.0f, 1.0f});

    ENTASIS_TEST_CHECK(nearf(sphere.radius, 2.0f));
    ENTASIS_TEST_CHECK(nearf(box.half_width, 1.0f));
    ENTASIS_TEST_CHECK(nearf(box.half_height, 2.0f));
    ENTASIS_TEST_CHECK(nearf(box.half_length, 3.0f));
    ENTASIS_TEST_CHECK(memcmp(&box, &half_box, sizeof(box)) == 0);
    ENTASIS_TEST_CHECK(nearf(capsule.half_length, 2.0f));
    ENTASIS_TEST_CHECK(memcmp(&capsule, &half_capsule, sizeof(capsule)) == 0);
    ENTASIS_TEST_CHECK(nearf(cylinder.half_length, 3.0f));
    ENTASIS_TEST_CHECK(memcmp(&cylinder, &half_cylinder, sizeof(cylinder)) == 0);
    ENTASIS_TEST_CHECK(nearf(triangle.b.x, 1.0f));

    ENTASIS_TEST_CHECK(entasis_shape_type_id(ENTASIS_SHAPE_TYPE_SPHERE) == ENTASIS_SHAPE_TYPE_SPHERE);
    ENTASIS_TEST_CHECK(entasis_shape_validate(ENTASIS_SHAPE_TYPE_SPHERE, &sphere) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_validate(ENTASIS_SHAPE_TYPE_BOX, &box) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_validate(ENTASIS_SHAPE_TYPE_CAPSULE, &capsule) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_validate(ENTASIS_SHAPE_TYPE_CYLINDER, &cylinder) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_validate(ENTASIS_SHAPE_TYPE_TRIANGLE, &triangle) == ENTASIS_STATUS_OK);

    const entasis_sphere_t invalid_sphere = entasis_sphere(0.0f);
    ENTASIS_TEST_CHECK(entasis_shape_validate(ENTASIS_SHAPE_TYPE_SPHERE, &invalid_sphere) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(entasis_shape_validate(ENTASIS_SHAPE_TYPE_SPHERE, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_shape_validate(ENTASIS_SHAPE_TYPE_INVALID, &sphere) == ENTASIS_STATUS_INVALID_ARGUMENT);

    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_BOX, &box, 3.0f, &inertia) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(inertia.inverse_mass > 0.0f);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_BOX, &box, 0.0f, &inertia) == ENTASIS_STATUS_INVALID_ARGUMENT);

    const entasis_continuous_detection_t discrete = entasis_ccd_discrete();
    const entasis_continuous_detection_t passive = entasis_ccd_passive();
    const entasis_continuous_detection_t continuous = entasis_ccd_continuous(0.01f, 0.001f);
    ENTASIS_TEST_CHECK(discrete.mode == ENTASIS_CONTINUOUS_DETECTION_DISCRETE);
    ENTASIS_TEST_CHECK(passive.mode == ENTASIS_CONTINUOUS_DETECTION_PASSIVE);
    ENTASIS_TEST_CHECK(continuous.mode == ENTASIS_CONTINUOUS_DETECTION_CONTINUOUS);
    return 0;
}

static int test_materials_filters_and_properties(void)
{
    entasis_collision_layer_t layer_a = 0;
    entasis_collision_layer_t layer_b = 0;
    ENTASIS_TEST_CHECK(entasis_collision_layer(2u, &layer_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_layer(9u, &layer_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_layer(64u, &layer_a) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_collision_layer(2u, &layer_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_layer_is_valid(layer_a) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_collision_layer_index(layer_b) == 9u);

    entasis_layer_mask_t mask = entasis_layer_mask_none();
    mask = entasis_layer_mask_add(mask, layer_b);
    ENTASIS_TEST_CHECK(entasis_layer_mask_contains(mask, layer_b) == ENTASIS_TRUE);
    mask = entasis_layer_mask_remove(mask, layer_b);
    ENTASIS_TEST_CHECK(entasis_layer_mask_contains(mask, layer_b) == ENTASIS_FALSE);
    ENTASIS_TEST_CHECK(entasis_layer_mask_all() == ENTASIS_LAYER_MASK_ALL);

    entasis_layer_matrix_t matrix = entasis_layer_matrix_none();
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allows(&matrix, layer_a, layer_b) == ENTASIS_FALSE);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allow(&matrix, layer_a, layer_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allows(&matrix, layer_a, layer_b) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allows(&matrix, layer_b, layer_a) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_deny(&matrix, layer_b, layer_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allows(&matrix, layer_a, layer_b) == ENTASIS_FALSE);

    const entasis_collision_filter_t filter_a = entasis_collision_filter(layer_a, entasis_layer_mask(layer_b));
    const entasis_collision_filter_t filter_b = entasis_collision_filter(layer_b, entasis_layer_mask(layer_a));
    const entasis_layer_matrix_t all_matrix = entasis_layer_matrix_all();
    ENTASIS_TEST_CHECK(entasis_collision_filter_allows(filter_a, filter_b, &all_matrix) == ENTASIS_TRUE);

    entasis_material_id_t id0 = entasis_material_id_invalid();
    entasis_material_id_t id1 = entasis_material_id_invalid();
    ENTASIS_TEST_CHECK(entasis_material_id(0u, &id0) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_material_id(1u, &id1) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_material_id_is_valid(id0) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_material_id_is_valid(entasis_material_id_invalid()) == ENTASIS_FALSE);

    const entasis_material_t materials[2] = {
        entasis_material(0.5f, 2.0f, entasis_spring_settings(10.0f, 0.5f)),
        entasis_material(0.25f, 4.0f, entasis_spring_settings(30.0f, 1.0f))};
    const entasis_material_t combined = entasis_material_combine_default(materials[0], materials[1]);
    ENTASIS_TEST_CHECK(nearf(combined.friction, 0.125f));
    ENTASIS_TEST_CHECK(nearf(combined.maximum_recovery_velocity, 4.0f));
    ENTASIS_TEST_CHECK(entasis_material_validate(combined) == ENTASIS_STATUS_OK);
    const entasis_material_table_t material_table = entasis_material_table(materials, UINT64_C(2));
    entasis_material_t loaded = {0};
    ENTASIS_TEST_CHECK(entasis_material_table_get(material_table, id1, &loaded) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(loaded.friction, materials[1].friction));
    ENTASIS_TEST_CHECK(entasis_material_table_get(material_table, entasis_material_id_invalid(), &loaded) == ENTASIS_STATUS_NOT_FOUND);

    entasis_collision_property_table_t properties = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_collision_property_init(&properties, UINT64_C(256), UINT64_C(128), NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_ensure_capacity(&properties, UINT64_C(512), UINT64_C(256)) == ENTASIS_STATUS_OK);

    const entasis_body_handle_t body = {17};
    const entasis_static_handle_t static_body = {23};
    const entasis_collision_properties_t body_properties = entasis_collision_properties(filter_a, id0);
    const entasis_collision_properties_t static_properties = entasis_collision_properties(filter_b, id1);
    ENTASIS_TEST_CHECK(entasis_collision_properties_validate(body_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_set_body(&properties, body, body_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_set_static(&properties, static_body, static_properties) == ENTASIS_STATUS_OK);

    entasis_collision_properties_t loaded_properties = {0};
    ENTASIS_TEST_CHECK(entasis_collision_property_get_body(&properties, body, &loaded_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(loaded_properties.material_id == id0);
    ENTASIS_TEST_CHECK(entasis_collision_property_get_static(&properties, static_body, &loaded_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(loaded_properties.material_id == id1);

    const entasis_collidable_reference_t references[2] = {
        {(uint32_t)body.value},
        {(uint32_t)static_body.value | UINT32_C(0x80000000)}};
    const entasis_collision_properties_t values[2] = {body_properties, static_properties};
    uint64_t completed = 0;
    ENTASIS_TEST_CHECK(entasis_collision_property_set_batch(&properties, references, values, UINT64_C(2), &completed) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == UINT64_C(2));
    ENTASIS_TEST_CHECK(entasis_collision_property_remove_batch(&properties, references, UINT64_C(2), &completed) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == UINT64_C(2));
    ENTASIS_TEST_CHECK(entasis_collision_property_get_body(&properties, body, &loaded_properties) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_collision_property_clear(&properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_destroy(&properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(properties.opaque == NULL);
    ENTASIS_TEST_CHECK(entasis_collision_property_destroy(&properties) == ENTASIS_STATUS_DISPOSED);
    return 0;
}

static int test_shapes_bodies_statics_and_bulk(void)
{
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = scene_world_description();
    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere = entasis_sphere(0.5f);
    const entasis_box_t box = entasis_box(2.0f, 1.0f, 2.0f);
    const entasis_capsule_t capsule = entasis_capsule(0.25f, 1.0f);
    const entasis_cylinder_t cylinder = entasis_cylinder(0.3f, 1.2f);
    const entasis_triangle_t triangle = entasis_triangle(
        (entasis_vector3_t){-1.0f, 0.0f, -1.0f},
        (entasis_vector3_t){1.0f, 0.0f, -1.0f},
        (entasis_vector3_t){0.0f, 0.0f, 1.0f});
    const entasis_shape_type_id_t type_ids[5] = {
        ENTASIS_SHAPE_TYPE_SPHERE,
        ENTASIS_SHAPE_TYPE_BOX,
        ENTASIS_SHAPE_TYPE_CAPSULE,
        ENTASIS_SHAPE_TYPE_CYLINDER,
        ENTASIS_SHAPE_TYPE_TRIANGLE};
    const void *shape_values[5] = {&sphere, &box, &capsule, &cylinder, &triangle};
    entasis_shape_handle_t shapes[5] = {0};
    uint64_t completed = 0;
    ENTASIS_TEST_CHECK(entasis_shape_add_batch(&world, type_ids, shape_values, UINT64_C(5), shapes, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == UINT64_C(5));
    for (uint64_t i = 0; i < UINT64_C(5); ++i)
    {
        ENTASIS_TEST_CHECK(entasis_shape_handle_is_valid(shapes[i]) == ENTASIS_TRUE);
        entasis_shape_info_t info = {0};
        ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, shapes[i], &info, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(info.type_id == type_ids[i]);
    }

    entasis_shape_bounds_t bounds = {0};
    ENTASIS_TEST_CHECK(entasis_shape_bounds(&world, shapes[0], identity_orientation(), &bounds, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(bounds.min.x, -0.5f));
    ENTASIS_TEST_CHECK(nearf(bounds.max.x, 0.5f));
    entasis_shape_ray_hit_t hit = {0};
    const entasis_ray_t ray = {
        {-2.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        4.0f};
    ENTASIS_TEST_CHECK(entasis_shape_ray(&world, shapes[0], pose_at(0.0f, 0.0f, 0.0f), ray, &hit, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(hit.hit == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(hit.t > 1.0f && hit.t < 2.0f);

    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1.0f, &inertia) == ENTASIS_STATUS_OK);
    entasis_body_description_t body_descriptions[BODY_COUNT];
    entasis_body_handle_t bodies[BODY_COUNT];
    entasis_body_state_t body_states[BODY_COUNT];
    entasis_rigid_pose_t poses[BODY_COUNT];
    entasis_body_velocity_t velocities[BODY_COUNT];
    entasis_body_inertia_t inertias[BODY_COUNT];
    entasis_activity_description_t activities[BODY_COUNT];
    entasis_collidable_description_t collidables[BODY_COUNT];
    entasis_shape_handle_t body_shapes[BODY_COUNT];
    for (uint64_t i = 0; i < BODY_COUNT; ++i)
    {
        poses[i] = pose_at((float)i * 2.0f, 2.0f, 0.0f);
        velocities[i] = entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f});
        inertias[i] = inertia;
        activities[i] = entasis_body_activity(-1.0f, UINT8_C(255));
        collidables[i] = entasis_collidable(shapes[0], entasis_ccd_discrete(), 0.0f, 1.0f);
        body_shapes[i] = shapes[0];
        body_descriptions[i] = entasis_body_dynamic(shapes[0], inertia, poses[i], velocities[i], activities[i]);
    }
    ENTASIS_TEST_CHECK(entasis_body_add_batch(&world, body_descriptions, BODY_COUNT, bodies, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == BODY_COUNT);
    ENTASIS_TEST_CHECK(entasis_body_get_batch(&world, bodies, BODY_COUNT, body_states, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == BODY_COUNT);
    ENTASIS_TEST_CHECK(nearf(body_states[17].pose.position.x, 34.0f));

    for (uint64_t i = 0; i < BODY_COUNT; ++i)
    {
        poses[i].position.y = 3.0f;
        velocities[i].linear.x = 1.0f + (float)i;
    }
    ENTASIS_TEST_CHECK(entasis_body_set_pose_batch(&world, bodies, poses, BODY_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == BODY_COUNT);
    ENTASIS_TEST_CHECK(entasis_body_set_velocity_batch(&world, bodies, velocities, BODY_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_inertia_batch(&world, bodies, inertias, BODY_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_activity_batch(&world, bodies, activities, BODY_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_collidable_batch(&world, bodies, collidables, BODY_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_shape_batch(&world, bodies, body_shapes, BODY_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_apply_batch(&world, bodies, body_descriptions, BODY_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get(&world, bodies[17], &body_states[17], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(body_states[17].pose.position.x, 34.0f));

    entasis_static_description_t static_descriptions[STATIC_COUNT];
    entasis_static_handle_t statics[STATIC_COUNT];
    entasis_static_state_t static_states[STATIC_COUNT];
    entasis_rigid_pose_t static_poses[STATIC_COUNT];
    for (uint64_t i = 0; i < STATIC_COUNT; ++i)
    {
        static_poses[i] = pose_at((float)i * 3.0f, -1.0f, 0.0f);
        static_descriptions[i] = entasis_static_body(shapes[1], static_poses[i], entasis_ccd_discrete());
    }
    ENTASIS_TEST_CHECK(entasis_static_add_batch(&world, static_descriptions, STATIC_COUNT, ENTASIS_AWAKENING_NONE, statics, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == STATIC_COUNT);
    ENTASIS_TEST_CHECK(entasis_static_get_batch(&world, statics, STATIC_COUNT, static_states, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(static_states[11].pose.position.x, 33.0f));
    for (uint64_t i = 0; i < STATIC_COUNT; ++i)
        static_poses[i].position.z = 5.0f;
    ENTASIS_TEST_CHECK(entasis_static_set_pose_batch(&world, statics, static_poses, STATIC_COUNT, ENTASIS_AWAKENING_NONE, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_set_continuity(&world, statics[0], &static_descriptions[0].continuity, ENTASIS_AWAKENING_NONE, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_set_shape(&world, statics[0], shapes[1], ENTASIS_AWAKENING_NONE, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_apply_batch(&world, statics, static_descriptions, STATIC_COUNT, ENTASIS_AWAKENING_NONE, &completed, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.active_bodies + stats.sleeping_bodies == (int64_t)BODY_COUNT);
    ENTASIS_TEST_CHECK(stats.statics == (int64_t)STATIC_COUNT);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, shapes[0], &diagnostic) == ENTASIS_STATUS_SHAPE_IN_USE);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, shapes[1], &diagnostic) == ENTASIS_STATUS_SHAPE_IN_USE);

    ENTASIS_TEST_CHECK(entasis_body_remove_batch(&world, bodies, BODY_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == BODY_COUNT);
    ENTASIS_TEST_CHECK(entasis_body_get(&world, bodies[0], &body_states[0], &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_static_remove_batch(&world, statics, STATIC_COUNT, ENTASIS_AWAKENING_NONE, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == STATIC_COUNT);
    ENTASIS_TEST_CHECK(entasis_static_get(&world, statics[0], &static_states[0], &diagnostic) == ENTASIS_STATUS_NOT_FOUND);

    ENTASIS_TEST_CHECK(entasis_shape_remove_batch(&world, shapes, UINT64_C(5), &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == UINT64_C(5));
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, shapes[0], &(entasis_shape_info_t){0}, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_compound_ownership(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = scene_world_description();
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere = entasis_sphere(0.5f);
    const entasis_box_t box = entasis_box(1.0f, 1.0f, 1.0f);
    entasis_shape_handle_t child_shapes[2] = {0};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &child_shapes[0], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_BOX, &box, &child_shapes[1], &diagnostic) == ENTASIS_STATUS_OK);

    entasis_compound_child_t children[2] = {
        entasis_compound_child(child_shapes[0], pose_at(-1.0f, 0.0f, 0.0f)),
        entasis_compound_child(child_shapes[1], pose_at(1.0f, 0.0f, 0.0f))};
    const float masses[2] = {1.0f, 2.0f};
    const entasis_compound_builder_t builder = entasis_compound_builder(children, (float *)masses, UINT64_C(2));
    entasis_vector3_t center = {0};
    float inverse_mass = 0.0f;
    ENTASIS_TEST_CHECK(entasis_compound_center_of_mass(builder, &center, &inverse_mass) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(inverse_mass > 0.0f);
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_compound_inertia_weighted(&world, builder, &inertia, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_compound_inertia_weighted_recenter(&world, builder, &inertia, &center, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_shape_handle_t compound = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&world, children, UINT64_C(2), &compound, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, child_shapes[0], &diagnostic) == ENTASIS_STATUS_SHAPE_IN_USE);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, compound, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, child_shapes[0], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, child_shapes[1], &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &child_shapes[0], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_BOX, &box, &child_shapes[1], &diagnostic) == ENTASIS_STATUS_OK);
    children[0] = entasis_compound_child(child_shapes[0], pose_at(-1.0f, 0.0f, 0.0f));
    children[1] = entasis_compound_child(child_shapes[1], pose_at(1.0f, 0.0f, 0.0f));
    entasis_shape_handle_t big = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_import_big_compound(&world, children, UINT64_C(2), &big, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t scratch[3] = {0};
    uint64_t removed = 0;
    uint64_t required = 0;
    ENTASIS_TEST_CHECK(entasis_shape_remove_recursive(&world, big, scratch, UINT64_C(3), &removed, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(removed == UINT64_C(3));
    ENTASIS_TEST_CHECK(required == UINT64_C(3));

    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_layer_material_world_ownership(void)
{
    entasis_collision_property_table_t properties = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_collision_property_init(&properties, UINT64_C(16), UINT64_C(16), NULL, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_layer_matrix_t matrix = entasis_layer_matrix_all();
    const entasis_material_t materials[1] = {entasis_material_default()};
    const entasis_material_table_t table = entasis_material_table(materials, UINT64_C(1));
    const entasis_layer_material_policy_t policy = entasis_layer_material_policy(&properties, &matrix, table);
    const entasis_narrow_policy_t narrow = entasis_narrow_policy_layers_materials(&policy);
    entasis_world_description_t description = scene_world_description();
    ENTASIS_TEST_CHECK(entasis_world_description_set_narrow_policy(&description, &narrow) == ENTASIS_STATUS_OK);

    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_destroy(&properties) == ENTASIS_STATUS_SHAPE_IN_USE);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_destroy(&properties) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_factories_and_validation() == 0);
    ENTASIS_TEST_CHECK(test_materials_filters_and_properties() == 0);
    ENTASIS_TEST_CHECK(test_shapes_bodies_statics_and_bulk() == 0);
    ENTASIS_TEST_CHECK(test_compound_ownership() == 0);
    ENTASIS_TEST_CHECK(test_layer_material_world_ownership() == 0);
    return 0;
}
