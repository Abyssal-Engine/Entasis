#include "entasis.h"
#include "test_support.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct allocation_counter_t
{
    uint64_t requests;
    uint64_t live;
} allocation_counter_t;

static size_t normalized_alignment(uint64_t requested)
{
    size_t alignment = (size_t)requested;
    if (alignment < sizeof(void *))
        alignment = sizeof(void *);
    return alignment;
}

static size_t rounded_size(uint64_t requested, size_t alignment)
{
    size_t size = requested == 0u ? 1u : (size_t)requested;
    const size_t remainder = size % alignment;
    if (remainder != 0u)
        size += alignment - remainder;
    return size;
}

static void *ENTASIS_CALL counted_allocate(void *context, uint64_t size, uint64_t alignment)
{
    allocation_counter_t *counter = (allocation_counter_t *)context;
    if (counter == NULL)
        return NULL;
    counter->requests += UINT64_C(1);
    void *memory = ENTASIS_TEST_ALIGNED_ALLOC(normalized_alignment(alignment), rounded_size(size, normalized_alignment(alignment)));
    if (memory != NULL)
        counter->live += UINT64_C(1);
    return memory;
}

static void *ENTASIS_CALL counted_reallocate(
    void *context,
    void *memory,
    uint64_t old_size,
    uint64_t new_size,
    uint64_t alignment)
{
    allocation_counter_t *counter = (allocation_counter_t *)context;
    if (counter == NULL)
        return NULL;
    counter->requests += UINT64_C(1);
    const size_t normalized = normalized_alignment(alignment);
    void *replacement = ENTASIS_TEST_ALIGNED_ALLOC(normalized, rounded_size(new_size, normalized));
    if (replacement == NULL)
        return NULL;
    if (memory != NULL)
    {
        const size_t copy_size = (size_t)(old_size < new_size ? old_size : new_size);
        if (copy_size > 0u)
            (void)memcpy(replacement, memory, copy_size);
        ENTASIS_TEST_ALIGNED_FREE(memory);
    }
    else
    {
        counter->live += UINT64_C(1);
    }
    return replacement;
}

static void ENTASIS_CALL counted_deallocate(
    void *context,
    void *memory,
    uint64_t size,
    uint64_t alignment)
{
    allocation_counter_t *counter = (allocation_counter_t *)context;
    (void)size;
    (void)alignment;
    if (counter == NULL || memory == NULL)
        return;
    ENTASIS_TEST_ALIGNED_FREE(memory);
    if (counter->live > 0u)
        counter->live -= UINT64_C(1);
}

static entasis_allocator_t counted_allocator(allocation_counter_t *counter)
{
    entasis_allocator_t allocator = {
        (uint32_t)sizeof(entasis_allocator_t),
        UINT32_C(1),
        counter,
        counted_allocate,
        counted_reallocate,
        counted_deallocate};
    return allocator;
}

static int nearf(float a, float b, float epsilon)
{
    return fabsf(a - b) <= epsilon;
}

static entasis_quaternion_t identity_orientation(void)
{
    return (entasis_quaternion_t){0.0f, 0.0f, 0.0f, 1.0f};
}

static entasis_rigid_pose_t pose_at(float x, float y, float z)
{
    return entasis_pose((entasis_vector3_t){x, y, z}, identity_orientation());
}

static entasis_world_description_t dynamics_world_description(void)
{
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    description.threading.worker_count = UINT32_C(1);
    description.capacity.bodies = INT32_C(256);
    description.capacity.statics = INT32_C(128);
    description.capacity.shapes_per_type = INT32_C(64);
    description.capacity.constraints = INT32_C(512);
    description.capacity.initial_constraints_per_type_batch = INT32_C(32);
    description.capacity.minimum_constraints_per_body = INT32_C(16);
    description.capacity.broad_phase_candidates = INT32_C(2048);
    description.capacity.pairs = INT32_C(2048);
    description.capacity.collision_child_pairs = INT32_C(2048);
    return description;
}

static entasis_body_inertia_t unit_inertia(void)
{
    entasis_body_inertia_t inertia = {0};
    inertia.inverse_inertia_tensor.xx = 1.0f;
    inertia.inverse_inertia_tensor.yy = 1.0f;
    inertia.inverse_inertia_tensor.zz = 1.0f;
    inertia.inverse_mass = 1.0f;
    return inertia;
}

static entasis_body_description_t shapeless_body(float x, uint8_t quiet_steps)
{
    return entasis_body_shapeless(
        unit_inertia(),
        pose_at(x, 0.0f, 0.0f),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(0.01f, quiet_steps));
}

static int test_impulses_bounds_and_factories(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = dynamics_world_description();
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere_value = entasis_sphere(0.5f);
    entasis_shape_handle_t sphere = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, &sphere, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(
                           ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, 1.0f, &inertia) == ENTASIS_STATUS_OK);

    const entasis_body_description_t body_description = entasis_body_dynamic(
        sphere,
        inertia,
        pose_at(0.0f, 0.0f, 0.0f),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    entasis_body_handle_t body = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_description, &body, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_static_description_t static_description = entasis_static_body(sphere, pose_at(4.0f, 0.0f, 0.0f), entasis_ccd_discrete());
    entasis_static_handle_t static_body = entasis_static_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_static_add(
                           &world, &static_description, ENTASIS_AWAKENING_NONE, &static_body, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_bounding_box_t body_bounds = {0};
    entasis_bounding_box_t static_bounds = {0};
    ENTASIS_TEST_CHECK(entasis_body_bounds(&world, body, &body_bounds, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_bounds(&world, static_body, &static_bounds, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(body_bounds.min.x, -0.5f, 1.0e-5f));
    ENTASIS_TEST_CHECK(nearf(body_bounds.max.x, 0.5f, 1.0e-5f));
    ENTASIS_TEST_CHECK(nearf(static_bounds.min.x, 3.5f, 1.0e-5f));
    ENTASIS_TEST_CHECK(nearf(static_bounds.max.x, 4.5f, 1.0e-5f));
    ENTASIS_TEST_CHECK(entasis_body_update_bounds(&world, body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_update_bounds(&world, static_body, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_body_apply_linear_impulse(
                           &world, body, (entasis_vector3_t){1.0f, 2.0f, 3.0f}, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_apply_angular_impulse(
                           &world, body, (entasis_vector3_t){0.0f, 1.0f, 0.0f}, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_apply_impulse(
                           &world,
                           body,
                           (entasis_vector3_t){1.0f, 0.0f, 0.0f},
                           (entasis_vector3_t){0.0f, 1.0f, 0.0f},
                           &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_state_t state = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &state, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.velocity.linear.x > 1.0f);
    ENTASIS_TEST_CHECK(state.velocity.linear.y > 0.0f);
    ENTASIS_TEST_CHECK(state.velocity.angular.y > 0.0f);
    ENTASIS_TEST_CHECK(fabsf(state.velocity.angular.z) > 0.0f);

    entasis_vector3_t offset_velocity = {0};
    ENTASIS_TEST_CHECK(entasis_body_velocity_at_offset(
                           &world, body, (entasis_vector3_t){0.0f, 1.0f, 0.0f}, &offset_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(isfinite(offset_velocity.x));
    ENTASIS_TEST_CHECK(isfinite(offset_velocity.y));
    ENTASIS_TEST_CHECK(isfinite(offset_velocity.z));

    const entasis_motor_settings_t softness = entasis_motor_settings(20.0f, 0.5f);
    ENTASIS_TEST_CHECK(nearf(softness.maximum_force, 20.0f, 0.0f));
    ENTASIS_TEST_CHECK(softness.damping > 0.0f);

    const entasis_solve_description_t solve = entasis_solve_description_substeps(INT32_C(4), INT32_C(6), INT32_C(32));
    ENTASIS_TEST_CHECK(solve.substeps == INT32_C(4));
    ENTASIS_TEST_CHECK(solve.velocity_iterations == INT32_C(6));
    ENTASIS_TEST_CHECK(solve.fallback_batch_threshold == INT32_C(32));

    ENTASIS_TEST_CHECK(entasis_body_remove(&world, body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_bounds(&world, body, &body_bounds, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_static_remove(
                           &world, static_body, ENTASIS_AWAKENING_NONE, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, sphere, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_sleeping_and_fixed_step(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = dynamics_world_description();
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_description_t body_description = shapeless_body(0.0f, UINT8_C(1));
    body_description.velocity.linear.x = 1.0f;
    entasis_body_handle_t moving = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_description, &moving, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_fixed_stepper_t stepper = entasis_fixed_stepper(0.1f, UINT8_C(4));
    uint32_t steps = UINT32_C(99);
    float alpha = -1.0f;
    ENTASIS_TEST_CHECK(entasis_fixed_stepper_update(
                           &stepper, &world, 0.25f, &steps, &alpha, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(steps == UINT32_C(2));
    ENTASIS_TEST_CHECK(nearf(alpha, 0.5f, 1.0e-5f));

    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.step_index == UINT64_C(2));
    ENTASIS_TEST_CHECK(entasis_fixed_stepper_reset(&stepper) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(stepper.accumulator, 0.0f, 0.0f));

    entasis_fixed_stepper_t invalid = entasis_fixed_stepper(0.0f, UINT8_C(4));
    ENTASIS_TEST_CHECK(entasis_fixed_stepper_update(
                           &invalid, &world, 0.1f, &steps, &alpha, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(entasis_fixed_stepper_update(
                           &stepper, &world, -0.1f, &steps, &alpha, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);

    entasis_body_handle_t sleeper = entasis_body_handle_invalid();
    const entasis_body_description_t sleeper_description = shapeless_body(2.0f, UINT8_C(1));
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &sleeper_description, &sleeper, &diagnostic) == ENTASIS_STATUS_OK);
    for (int index = 0; index < 8; ++index)
    {
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    }

    entasis_body_activation_state_t activation = ENTASIS_BODY_ACTIVATION_MISSING;
    entasis_bool_t sleeping = ENTASIS_FALSE;
    entasis_bool_t active = ENTASIS_FALSE;
    ENTASIS_TEST_CHECK(entasis_body_activation_state(&world, sleeper, &activation, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_is_sleeping(&world, sleeper, &sleeping, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(activation == ENTASIS_BODY_ACTIVATION_SLEEPING);
    ENTASIS_TEST_CHECK(sleeping == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_body_awaken(&world, sleeper, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_is_active(&world, sleeper, &active, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(active == ENTASIS_TRUE);

    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_commands_and_allocation_contract(void)
{
    allocation_counter_t counter = {0};
    entasis_world_description_t description = dynamics_world_description();
    description.allocator = counted_allocator(&counter);
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere_value = entasis_sphere(0.5f);
    entasis_shape_handle_t sphere = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, &sphere, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(
                           ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, 1.0f, &inertia) == ENTASIS_STATUS_OK);

    const entasis_body_description_t body_a = entasis_body_dynamic(
        sphere, inertia, pose_at(0.0f, 0.0f, 0.0f),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    entasis_body_description_t body_b = body_a;
    body_b.pose.position.x = 2.0f;
    const entasis_static_description_t static_a = entasis_static_body(sphere, pose_at(5.0f, 0.0f, 0.0f), entasis_ccd_discrete());
    entasis_static_description_t static_b = static_a;
    static_b.pose.position.x = 7.0f;

    entasis_body_handle_t existing_body = entasis_body_handle_invalid();
    entasis_static_handle_t existing_static = entasis_static_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_a, &existing_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_add(
                           &world, &static_a, ENTASIS_AWAKENING_NONE, &existing_static, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_command_t commands[6] = {
        entasis_command_body_add(body_b, INT32_C(0)),
        entasis_command_static_add(static_b, INT32_C(0), ENTASIS_AWAKENING_NONE),
        entasis_command_body_apply(existing_body, body_b),
        entasis_command_static_apply(existing_static, static_b, ENTASIS_AWAKENING_NONE),
        entasis_command_body_remove(existing_body),
        entasis_command_static_remove(existing_static, ENTASIS_AWAKENING_NONE)};
    entasis_status_t statuses[6] = {
        ENTASIS_STATUS_DISPOSED, ENTASIS_STATUS_DISPOSED, ENTASIS_STATUS_DISPOSED,
        ENTASIS_STATUS_DISPOSED, ENTASIS_STATUS_DISPOSED, ENTASIS_STATUS_DISPOSED};
    entasis_body_handle_t body_results[1] = {entasis_body_handle_invalid()};
    entasis_static_handle_t static_results[1] = {entasis_static_handle_invalid()};
    entasis_command_buffer_t buffer = entasis_command_buffer(
        commands,
        UINT64_C(6),
        statuses,
        UINT64_C(6),
        body_results,
        UINT64_C(1),
        static_results,
        UINT64_C(1));

    const uint64_t requests_before = counter.requests;
    uint64_t processed = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_world_apply_commands(
                           &world, &buffer, &processed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(processed == UINT64_C(6));
    ENTASIS_TEST_CHECK(counter.requests == requests_before);
    for (size_t index = 0; index < sizeof(statuses) / sizeof(statuses[0]); ++index)
    {
        ENTASIS_TEST_CHECK(statuses[index] == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(entasis_body_handle_is_valid(body_results[0]) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_static_handle_is_valid(static_results[0]) == ENTASIS_TRUE);

    entasis_body_state_t untouched_before = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(
                           &world, body_results[0], &untouched_before, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_description_t changed = untouched_before;
    changed.pose.position.x = 20.0f;
    entasis_body_description_t skipped = untouched_before;
    skipped.pose.position.x = 30.0f;
    entasis_command_t prefix_commands[3] = {
        entasis_command_body_apply(body_results[0], changed),
        entasis_command_body_remove(entasis_body_handle_invalid()),
        entasis_command_body_apply(body_results[0], skipped)};
    entasis_status_t prefix_statuses[3] = {
        ENTASIS_STATUS_DISPOSED, ENTASIS_STATUS_DISPOSED, ENTASIS_STATUS_DISPOSED};
    entasis_command_buffer_t prefix_buffer = entasis_command_buffer(
        prefix_commands, UINT64_C(3), prefix_statuses, UINT64_C(3), NULL, UINT64_C(0), NULL, UINT64_C(0));
    ENTASIS_TEST_CHECK(entasis_world_apply_commands(
                           &world, &prefix_buffer, &processed, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(processed == UINT64_C(1));
    ENTASIS_TEST_CHECK(prefix_statuses[0] == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(prefix_statuses[1] == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(prefix_statuses[2] == ENTASIS_STATUS_DISPOSED);
    entasis_body_state_t prefix_state = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(
                           &world, body_results[0], &prefix_state, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(prefix_state.pose.position.x, 20.0f, 0.0f));

    entasis_command_buffer_t short_status_buffer = entasis_command_buffer(
        prefix_commands, UINT64_C(3), prefix_statuses, UINT64_C(2), NULL, UINT64_C(0), NULL, UINT64_C(0));
    ENTASIS_TEST_CHECK(entasis_world_apply_commands(
                           &world, &short_status_buffer, &processed, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(processed == UINT64_C(0));

    entasis_command_t duplicate_results[2] = {
        entasis_command_body_add(body_a, INT32_C(0)),
        entasis_command_body_add(body_b, INT32_C(0))};
    entasis_body_handle_t duplicate_output[1] = {entasis_body_handle_invalid()};
    entasis_command_buffer_t duplicate_buffer = entasis_command_buffer(
        duplicate_results, UINT64_C(2), NULL, UINT64_C(0), duplicate_output, UINT64_C(1), NULL, UINT64_C(0));
    ENTASIS_TEST_CHECK(entasis_world_apply_commands(
                           &world, &duplicate_buffer, &processed, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(processed == UINT64_C(0));
    ENTASIS_TEST_CHECK(duplicate_output[0].value == -1);

    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(counter.live == UINT64_C(0));
    return 0;
}

static int test_solver_contact_inspection_and_impulses(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = dynamics_world_description();
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere_value = entasis_sphere(1.0f);
    entasis_shape_handle_t sphere = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, &sphere, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(
                           ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, 1.0f, &inertia) == ENTASIS_STATUS_OK);

    const entasis_body_description_t body_description = entasis_body_dynamic(
        sphere, inertia, pose_at(0.0f, 0.0f, 0.0f),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    entasis_body_handle_t body = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_description, &body, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_static_description_t static_description = entasis_static_body(sphere, pose_at(1.5f, 0.0f, 0.0f), entasis_ccd_discrete());
    entasis_static_handle_t static_body = entasis_static_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_static_add(
                           &world, &static_description, ENTASIS_AWAKENING_NONE, &static_body, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_constraint_info_t infos[16] = {0};
    uint64_t written = UINT64_C(0);
    uint64_t required = UINT64_C(0);
    const entasis_status_t enumerate_status = entasis_constraint_enumerate(
        &world, infos, UINT64_C(16), &written, &required, &diagnostic);
    ENTASIS_TEST_CHECK(enumerate_status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written >= UINT64_C(1));
    ENTASIS_TEST_CHECK(required == written);

    entasis_constraint_handle_t contact = entasis_constraint_handle_invalid();
    for (uint64_t index = 0; index < written; ++index)
    {
        entasis_solver_contact_data_t candidate = {0};
        if (entasis_solver_contact_data(&world, infos[index].handle, &candidate, &diagnostic) == ENTASIS_STATUS_OK)
        {
            contact = infos[index].handle;
            break;
        }
    }
    ENTASIS_TEST_CHECK(entasis_constraint_handle_is_valid(contact) == ENTASIS_TRUE);

    entasis_solver_contact_data_t contact_data = {0};
    ENTASIS_TEST_CHECK(entasis_solver_contact_data(
                           &world, contact, &contact_data, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(contact_data.constraint.value == contact.value);
    ENTASIS_TEST_CHECK(contact_data.contact_count >= UINT8_C(1));
    ENTASIS_TEST_CHECK(contact_data.contact_count <= ENTASIS_MAXIMUM_SOLVER_CONTACT_COUNT);
    ENTASIS_TEST_CHECK(contact_data.impulse_count >= UINT8_C(1));
    ENTASIS_TEST_CHECK(contact_data.impulse_count <= ENTASIS_MAXIMUM_SOLVER_IMPULSE_COUNT);

    written = UINT64_C(99);
    required = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulses(
                           &world, contact, NULL, UINT64_C(0), &written, &required, &diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(written == UINT64_C(0));
    ENTASIS_TEST_CHECK(required == contact_data.impulse_count);

    float impulses[ENTASIS_MAXIMUM_SOLVER_IMPULSE_COUNT] = {0};
    ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulses(
                           &world,
                           contact,
                           impulses,
                           ENTASIS_MAXIMUM_SOLVER_IMPULSE_COUNT,
                           &written,
                           &required,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == required);

    float magnitude = -1.0f;
    float magnitude_squared = -1.0f;
    ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulse_magnitude(
                           &world, contact, &magnitude, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulse_magnitude_squared(
                           &world, contact, &magnitude_squared, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(magnitude >= 0.0f);
    ENTASIS_TEST_CHECK(nearf(magnitude * magnitude, magnitude_squared, 1.0e-3f));

    ENTASIS_TEST_CHECK(entasis_world_scale_active_accumulated_impulses(
                           &world, 0.5f, &diagnostic) == ENTASIS_STATUS_OK);
    float scaled = -1.0f;
    ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulse_magnitude(
                           &world, contact, &scaled, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(scaled, magnitude * 0.5f, 1.0e-3f));
    ENTASIS_TEST_CHECK(entasis_world_scale_accumulated_impulses(
                           &world, 1.0f, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_impulses_bounds_and_factories() == 0);
    ENTASIS_TEST_CHECK(test_sleeping_and_fixed_step() == 0);
    ENTASIS_TEST_CHECK(test_commands_and_allocation_contract() == 0);
    ENTASIS_TEST_CHECK(test_solver_contact_inspection_and_impulses() == 0);
    return 0;
}
