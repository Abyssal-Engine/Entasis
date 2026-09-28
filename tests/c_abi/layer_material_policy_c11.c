#include "entasis.h"
#include "test_support.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

typedef struct policy_callback_state_t
{
    uint64_t pair_calls;
    uint64_t child_calls;
    uint64_t material_calls;
} policy_callback_state_t;

static entasis_quaternion_t identity_orientation(void)
{
    const entasis_quaternion_t value = {0.0f, 0.0f, 0.0f, 1.0f};
    return value;
}

static int nearly_equal(float a, float b, float epsilon)
{
    return fabsf(a - b) <= epsilon;
}

static entasis_collidable_reference_t dynamic_reference(entasis_body_handle_t handle)
{
    entasis_collidable_reference_t reference = {(uint32_t)handle.value};
    return reference;
}

static entasis_collidable_reference_t static_reference(entasis_static_handle_t handle)
{
    entasis_collidable_reference_t reference = {
        (UINT32_C(2) << UINT32_C(30)) | (uint32_t)handle.value};
    return reference;
}

static entasis_bool_t ENTASIS_CALL count_pair(
    void *user_context,
    entasis_collidable_reference_t a,
    entasis_collidable_reference_t b)
{
    policy_callback_state_t *state = (policy_callback_state_t *)user_context;
    (void)a;
    (void)b;
    if (state == NULL)
        return ENTASIS_FALSE;
    state->pair_calls += UINT64_C(1);
    return ENTASIS_TRUE;
}

static entasis_bool_t ENTASIS_CALL count_child(
    void *user_context,
    entasis_collidable_reference_t a,
    entasis_collidable_reference_t b,
    int32_t child_a,
    int32_t child_b)
{
    policy_callback_state_t *state = (policy_callback_state_t *)user_context;
    (void)a;
    (void)b;
    (void)child_a;
    (void)child_b;
    if (state == NULL)
        return ENTASIS_FALSE;
    state->child_calls += UINT64_C(1);
    return ENTASIS_TRUE;
}

static entasis_bool_t ENTASIS_CALL count_material(
    void *user_context,
    entasis_material_id_t a_id,
    const entasis_material_t *a,
    entasis_material_id_t b_id,
    const entasis_material_t *b,
    entasis_material_t *out_material)
{
    policy_callback_state_t *state = (policy_callback_state_t *)user_context;
    if (state == NULL || a == NULL || b == NULL || out_material == NULL)
    {
        return ENTASIS_FALSE;
    }
    ENTASIS_TEST_CHECK(entasis_material_id_is_valid(a_id) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_material_id_is_valid(b_id) == ENTASIS_TRUE);
    state->material_calls += UINT64_C(1);
    *out_material = entasis_material_combine_default(*a, *b);
    return ENTASIS_TRUE;
}

static int test_materials_layers_and_dense_properties(void)
{
    const entasis_spring_settings_t soft_spring = entasis_spring_settings(10.0f, 0.5f);
    const entasis_spring_settings_t stiff_spring = entasis_spring_settings(30.0f, 1.0f);
    const entasis_material_t soft = entasis_material(0.5f, 2.0f, soft_spring);
    const entasis_material_t stiff = entasis_material(0.25f, 4.0f, stiff_spring);
    ENTASIS_TEST_CHECK(entasis_material_validate(soft) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_material_validate((entasis_material_t){-1.0f, 1.0f, stiff_spring}) == ENTASIS_STATUS_INVALID_DESCRIPTION);

    const entasis_material_t combined_ab = entasis_material_combine_default(soft, stiff);
    const entasis_material_t combined_ba = entasis_material_combine_default(stiff, soft);
    ENTASIS_TEST_CHECK(nearly_equal(combined_ab.friction, 0.125f, 1.0e-6f));
    ENTASIS_TEST_CHECK(nearly_equal(combined_ab.friction, combined_ba.friction, 1.0e-6f));
    ENTASIS_TEST_CHECK(nearly_equal(combined_ab.maximum_recovery_velocity, 4.0f, 1.0e-6f));

    entasis_material_id_t material_a = entasis_material_id_invalid();
    entasis_material_id_t material_b = entasis_material_id_invalid();
    ENTASIS_TEST_CHECK(entasis_material_id(0, &material_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_material_id(1, &material_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_material_id(UINT32_MAX, &material_a) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_material_id(0, &material_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_material_id_is_valid(material_a) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_material_id_is_valid(entasis_material_id_invalid()) == ENTASIS_FALSE);

    const entasis_material_t materials[2] = {soft, stiff};
    const entasis_material_table_t material_table = entasis_material_table(materials, UINT64_C(2));
    entasis_material_t loaded = {0};
    ENTASIS_TEST_CHECK(entasis_material_table_get(material_table, material_b, &loaded) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearly_equal(loaded.friction, stiff.friction, 1.0e-6f));
    ENTASIS_TEST_CHECK(entasis_material_table_get(material_table, entasis_material_id_invalid(), &loaded) == ENTASIS_STATUS_NOT_FOUND);

    entasis_collision_layer_t layer_a = UINT8_C(0);
    entasis_collision_layer_t layer_b = UINT8_C(0);
    ENTASIS_TEST_CHECK(entasis_collision_layer(3, &layer_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_layer(9, &layer_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_layer(64, &layer_b) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_collision_layer(9, &layer_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_layer_index(layer_a) == UINT32_C(3));

    entasis_layer_mask_t mask = entasis_layer_mask_none();
    mask = entasis_layer_mask_add(mask, layer_b);
    ENTASIS_TEST_CHECK(entasis_layer_mask_contains(mask, layer_b) == ENTASIS_TRUE);
    mask = entasis_layer_mask_remove(mask, layer_b);
    ENTASIS_TEST_CHECK(entasis_layer_mask_contains(mask, layer_b) == ENTASIS_FALSE);

    entasis_layer_matrix_t layer_matrix;
    layer_matrix = entasis_layer_matrix_none();
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allows(&layer_matrix, layer_a, layer_b) == ENTASIS_FALSE);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allow(&layer_matrix, layer_a, layer_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allows(&layer_matrix, layer_b, layer_a) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_deny(&layer_matrix, layer_b, layer_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_allows(&layer_matrix, layer_a, layer_b) == ENTASIS_FALSE);
    ENTASIS_TEST_CHECK(entasis_layer_matrix_set(&layer_matrix, layer_a, layer_b, UINT8_C(2)) == ENTASIS_STATUS_INVALID_ARGUMENT);

    const entasis_collision_filter_t filter_a = entasis_collision_filter(layer_a, entasis_layer_mask(layer_b));
    const entasis_collision_filter_t filter_b = entasis_collision_filter(layer_b, entasis_layer_mask(layer_a));
    layer_matrix = entasis_layer_matrix_all();
    ENTASIS_TEST_CHECK(entasis_collision_filter_allows(filter_a, filter_b, &layer_matrix) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_collision_filter_allows(
                           filter_a,
                           entasis_collision_filter(layer_b, entasis_layer_mask_none()),
                           &layer_matrix) == ENTASIS_FALSE);

    const entasis_collision_properties_t properties_a = entasis_collision_properties(filter_a, material_a);
    const entasis_collision_properties_t properties_b = entasis_collision_properties(filter_b, material_b);
    ENTASIS_TEST_CHECK(entasis_collision_properties_validate(properties_a) == ENTASIS_STATUS_OK);
    entasis_collision_properties_t invalid_properties = properties_a;
    invalid_properties.layer = UINT8_C(64);
    ENTASIS_TEST_CHECK(entasis_collision_properties_validate(invalid_properties) == ENTASIS_STATUS_INVALID_DESCRIPTION);

    entasis_collision_property_table_t table = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_collision_property_init(&table, 1u, 1u, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_ensure_capacity(&table, 16u, 16u) == ENTASIS_STATUS_OK);

    const entasis_body_handle_t body = {3};
    const entasis_static_handle_t static_handle = {4};
    ENTASIS_TEST_CHECK(entasis_collision_property_set_body(&table, body, properties_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_set_static(&table, static_handle, properties_b) == ENTASIS_STATUS_OK);
    entasis_collision_properties_t loaded_properties = {0};
    ENTASIS_TEST_CHECK(entasis_collision_property_get_body(&table, body, &loaded_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(loaded_properties.material_id == material_a);
    ENTASIS_TEST_CHECK(entasis_collision_property_get_static(&table, static_handle, &loaded_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(loaded_properties.material_id == material_b);

    const entasis_collidable_reference_t references[2] = {
        dynamic_reference(body), static_reference(static_handle)};
    ENTASIS_TEST_CHECK(entasis_collision_property_set(
                           &table, references[0], properties_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_get(
                           &table, references[0], &loaded_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(loaded_properties.material_id == material_b);
    ENTASIS_TEST_CHECK(entasis_collision_property_remove(
                           &table, references[0]) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_get_body(
                           &table, body, &loaded_properties) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_collision_property_set_body(
                           &table, body, properties_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_remove_static(
                           &table, static_handle) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_get_static(
                           &table, static_handle, &loaded_properties) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_collision_property_set_static(
                           &table, static_handle, properties_b) == ENTASIS_STATUS_OK);
    const entasis_collision_properties_t batch_values[2] = {properties_b, properties_a};
    uint64_t processed = 0;
    ENTASIS_TEST_CHECK(entasis_collision_property_set_batch(
                           &table, references, batch_values, UINT64_C(2), &processed) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(processed == UINT64_C(2));
    ENTASIS_TEST_CHECK(entasis_collision_property_get(
                           &table, references[0], &loaded_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(loaded_properties.material_id == material_b);
    processed = 0;
    ENTASIS_TEST_CHECK(entasis_collision_property_remove_batch(
                           &table, references, UINT64_C(2), &processed) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(processed == UINT64_C(2));
    ENTASIS_TEST_CHECK(entasis_collision_property_get_body(
                           &table, body, &loaded_properties) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_collision_property_set_body(
                           &table, body, invalid_properties) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(entasis_collision_property_clear(&table) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_destroy(&table) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_destroy(&table) == ENTASIS_STATUS_DISPOSED);
    return 0;
}

static int test_live_layer_material_policy(void)
{
    entasis_diagnostic_t diagnostic = {0};
    entasis_collision_property_table_t properties = {0};
    ENTASIS_TEST_CHECK(entasis_collision_property_init(
                           &properties, UINT64_C(16), UINT64_C(4), NULL, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_collision_layer_t layer_a = UINT8_C(0);
    entasis_collision_layer_t layer_b = UINT8_C(0);
    entasis_material_id_t material_a = entasis_material_id_invalid();
    entasis_material_id_t material_b = entasis_material_id_invalid();
    ENTASIS_TEST_CHECK(entasis_collision_layer(3, &layer_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_layer(4, &layer_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_material_id(0, &material_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_material_id(1, &material_b) == ENTASIS_STATUS_OK);

    const entasis_material_t materials[2] = {
        entasis_material(0.5f, 2.0f, entasis_spring_settings(10.0f, 0.5f)),
        entasis_material(0.25f, 4.0f, entasis_spring_settings(30.0f, 1.0f))};
    const entasis_material_table_t material_table = entasis_material_table(materials, UINT64_C(2));
    entasis_layer_matrix_t layer_matrix;
    layer_matrix = entasis_layer_matrix_all();
    ENTASIS_TEST_CHECK(entasis_layer_matrix_deny(&layer_matrix, layer_a, layer_b) == ENTASIS_STATUS_OK);

    policy_callback_state_t callbacks = {0};
    entasis_layer_material_policy_t policy = entasis_layer_material_policy(
        &properties, &layer_matrix, material_table);
    policy.pair_filter = count_pair;
    policy.child_filter = count_child;
    policy.material_combine = count_material;
    policy.user_context = &callbacks;
    const entasis_narrow_policy_t narrow = entasis_narrow_policy_layers_materials(&policy);

    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    ENTASIS_TEST_CHECK(entasis_world_description_set_narrow_policy(
                           &description, &narrow) == ENTASIS_STATUS_OK);

    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_destroy(&properties) == ENTASIS_STATUS_SHAPE_IN_USE);

    const entasis_sphere_t sphere = entasis_sphere(1.0f);
    entasis_shape_handle_t child_shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &child_shape, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_compound_child_t compound_children[1] = {
        entasis_compound_child(
            child_shape,
            entasis_pose((entasis_vector3_t){0.0f, 0.0f, 0.0f}, identity_orientation()))};
    const float masses[1] = {1.0f};
    entasis_compound_build_result_t compound = {0};
    ENTASIS_TEST_CHECK(entasis_compound_build_dynamic(
                           &world,
                           entasis_compound_builder(compound_children, masses, UINT64_C(1)),
                           ENTASIS_FALSE,
                           &compound,
                           &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_inertia_t sphere_inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(
                           ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1.0f, &sphere_inertia) == ENTASIS_STATUS_OK);
    const entasis_body_description_t body_a_description = entasis_body_dynamic(
        compound.shape,
        compound.inertia,
        entasis_pose((entasis_vector3_t){0.0f, 0.0f, 0.0f}, identity_orientation()),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    const entasis_body_description_t body_b_description = entasis_body_dynamic(
        child_shape,
        sphere_inertia,
        entasis_pose((entasis_vector3_t){1.5f, 0.0f, 0.0f}, identity_orientation()),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    entasis_body_handle_t body_a = entasis_body_handle_invalid();
    entasis_body_handle_t body_b = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_a_description, &body_a, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_b_description, &body_b, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_collision_properties_t body_a_properties = entasis_collision_properties(
        entasis_collision_filter(layer_a, entasis_layer_mask(layer_b)), material_a);
    const entasis_collision_properties_t body_b_properties = entasis_collision_properties(
        entasis_collision_filter(layer_b, entasis_layer_mask(layer_a)), material_b);
    ENTASIS_TEST_CHECK(entasis_collision_property_set_body(
                           &properties, body_a, body_a_properties) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_set_body(
                           &properties, body_b, body_b_properties) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(callbacks.pair_calls == UINT64_C(0));
    ENTASIS_TEST_CHECK(callbacks.child_calls == UINT64_C(0));
    ENTASIS_TEST_CHECK(callbacks.material_calls == UINT64_C(0));

    ENTASIS_TEST_CHECK(entasis_layer_matrix_allow(&layer_matrix, layer_a, layer_b) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(callbacks.pair_calls > UINT64_C(0));
    ENTASIS_TEST_CHECK(callbacks.child_calls > UINT64_C(0));
    ENTASIS_TEST_CHECK(callbacks.material_calls > UINT64_C(0));

    ENTASIS_TEST_CHECK(entasis_body_remove(&world, body_a, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_remove(&world, body_b, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_remove_body(&properties, body_a) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_remove_body(&properties, body_b) == ENTASIS_STATUS_OK);

    entasis_shape_handle_t scratch[2] = {0};
    uint64_t removed = 0;
    uint64_t required = 0;
    ENTASIS_TEST_CHECK(entasis_shape_remove_recursive(
                           &world, compound.shape, scratch, UINT64_C(2), &removed, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(removed == UINT64_C(2));
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_property_destroy(&properties) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_invalid_policy_descriptions(void)
{
    entasis_diagnostic_t diagnostic = {0};
    entasis_material_t invalid_material = entasis_material(
        -1.0f, 1.0f, entasis_spring_settings(30.0f, 1.0f));
    const entasis_material_table_t invalid_table = entasis_material_table(
        &invalid_material, UINT64_C(1));
    entasis_layer_material_policy_t policy = entasis_layer_material_policy(
        NULL, NULL, invalid_table);
    entasis_narrow_policy_t narrow = entasis_narrow_policy_layers_materials(&policy);
    entasis_world_description_t description = entasis_world_description_default();
    ENTASIS_TEST_CHECK(entasis_world_description_set_narrow_policy(
                           &description, &narrow) == ENTASIS_STATUS_OK);

    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(
                           &world, &description, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);

    entasis_collision_property_table_t disposed = {0};
    const entasis_material_t valid_material = entasis_material_default();
    policy = entasis_layer_material_policy(
        &disposed,
        NULL,
        entasis_material_table(&valid_material, UINT64_C(1)));
    narrow = entasis_narrow_policy_layers_materials(&policy);
    description = entasis_world_description_default();
    ENTASIS_TEST_CHECK(entasis_world_description_set_narrow_policy(
                           &description, &narrow) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_init(
                           &world, &description, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    return 0;
}

typedef struct restitution_callback_state_t
{
    entasis_world_t *world;
    unsigned calls, errors;
    int invalid;
} restitution_callback_state_t;

static void ENTASIS_CALL combine_restitution(
    void *raw, entasis_collidable_reference_t a, entasis_collidable_reference_t b,
    const entasis_restitution_settings_t *sa, const entasis_restitution_settings_t *sb,
    entasis_restitution_settings_t *output)
{
    restitution_callback_state_t *state = (restitution_callback_state_t *)raw;
    (void)b;
    (void)sb;
    ++state->calls;
    if (entasis_restitution_set(state->world, a, sa, NULL) != ENTASIS_STATUS_INVALID_ARGUMENT)
        ++state->errors;
    if (state->invalid)
        output->coefficient = -1;
}

static int test_restitution_public_api(void)
{
    for (unsigned scope = 0; scope < 2; ++scope)
    {
        entasis_world_t world = {0};
        restitution_callback_state_t callback = {&world, 0, 0, 0};
        entasis_world_description_t description = entasis_world_description_default();
        description.gravity = (entasis_vector3_t){0, 0, 0};
        description.damping.linear = description.damping.angular = 0;
        description.threading.worker_count = 2;
        entasis_world_extensions_t extensions = entasis_world_extensions_default();
        extensions.allocation_scope = scope;
        ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &description, &extensions, NULL) == ENTASIS_STATUS_OK);
        const entasis_sphere_t sphere = entasis_sphere(1);
        entasis_shape_handle_t shape = {0};
        entasis_body_inertia_t inertia = {0};
        ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1, &inertia) == ENTASIS_STATUS_OK);
        const entasis_static_description_t ground_description = entasis_static_body(shape,
                                                                                    entasis_pose((entasis_vector3_t){0, 0, 0}, identity_orientation()), entasis_ccd_discrete());
        entasis_static_handle_t ground = {0};
        entasis_body_handle_t body = {0};
        ENTASIS_TEST_CHECK(entasis_static_add(&world, &ground_description, ENTASIS_AWAKENING_NONE, &ground, NULL) == ENTASIS_STATUS_OK);
        const entasis_body_description_t body_description = entasis_body_dynamic(shape, inertia,
                                                                                 entasis_pose((entasis_vector3_t){0, 2, 0}, identity_orientation()),
                                                                                 entasis_velocity((entasis_vector3_t){0, -10, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
        ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_description, &body, NULL) == ENTASIS_STATUS_OK);
        entasis_restitution_configuration_t configuration = entasis_restitution_configuration_default();
        ENTASIS_TEST_CHECK(configuration.fallback.coefficient == 0 && configuration.fallback.threshold == 1);
        configuration.fallback.coefficient = 0.5f;
        configuration.combine = combine_restitution;
        configuration.user_context = &callback;
        ENTASIS_TEST_CHECK(entasis_world_enable_restitution(&world, &configuration, NULL) == ENTASIS_STATUS_OK);
        configuration.combine = NULL;
        configuration.fallback.coefficient = 1;
        entasis_restitution_settings_t settings = {0};
        const entasis_collidable_reference_t reference = dynamic_reference(body);
        ENTASIS_TEST_CHECK(entasis_restitution_get(&world, reference, &settings, NULL) == ENTASIS_STATUS_OK && settings.coefficient == 0.5f);
        settings.coefficient = 2;
        ENTASIS_TEST_CHECK(entasis_restitution_set(&world, reference, &settings, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        settings.coefficient = 0.75f;
        ENTASIS_TEST_CHECK(entasis_restitution_set(&world, reference, &settings, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_restitution_remove(&world, reference, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_restitution_remove(&world, reference, NULL) == ENTASIS_STATUS_NOT_FOUND);
        ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_restitution_get(&world, reference, &settings, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_restitution_set(&world, reference, &settings, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_disable_restitution(&world, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_end_read(&world, NULL) == ENTASIS_STATUS_OK);
        callback.invalid = 1;
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        callback.invalid = 0;
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, NULL) == ENTASIS_STATUS_OK);
        entasis_body_state_t result = {0};
        ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &result, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(nearly_equal(result.velocity.linear.y, 5, 0.0002f));
        ENTASIS_TEST_CHECK(callback.calls >= 2 && callback.errors == 0);
        ENTASIS_TEST_CHECK(entasis_restitution_reserve(&world, 16, 128, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_disable_restitution(&world, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
    }
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_materials_layers_and_dense_properties() == 0);
    ENTASIS_TEST_CHECK(test_live_layer_material_policy() == 0);
    ENTASIS_TEST_CHECK(test_invalid_policy_descriptions() == 0);
    ENTASIS_TEST_CHECK(test_restitution_public_api() == 0);
    return 0;
}
