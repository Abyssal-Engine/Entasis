#include "entasis.h"
#include "test_support.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t float_bits(float value)
{
    uint32_t bits = 0;
    (void)memcpy(&bits, &value, sizeof(bits));
    return bits;
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

static entasis_body_description_t body_at(float x)
{
    return entasis_body_shapeless(
        unit_inertia(),
        entasis_pose(
            (entasis_vector3_t){x, 0.0f, 0.0f},
            (entasis_quaternion_t){0.0f, 0.0f, 0.0f, 1.0f}),
        entasis_velocity(
            (entasis_vector3_t){0.0f, 0.0f, 0.0f},
            (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(0.01f, UINT8_C(255)));
}

int main(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    description.threading.worker_count = UINT32_C(1);
    description.capacity.bodies = INT32_C(32);
    description.capacity.constraints = INT32_C(32);
    description.capacity.initial_constraints_per_type_batch = INT32_C(8);
    description.capacity.minimum_constraints_per_body = INT32_C(8);
    description.capacity.broad_phase_candidates = INT32_C(64);
    description.capacity.pairs = INT32_C(64);
    description.capacity.collision_child_pairs = INT32_C(64);

    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_handle_t bodies[2] = {
        entasis_body_handle_invalid(), entasis_body_handle_invalid()};
    entasis_body_description_t body_a = body_at(0.0f);
    entasis_body_description_t body_b = body_at(2.0f);
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_a, &bodies[0], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_b, &bodies[1], &diagnostic) == ENTASIS_STATUS_OK);

    entasis_ball_socket_t ball_socket = {
        {0.5f, 0.0f, 0.0f},
        {-0.5f, 0.0f, 0.0f},
        entasis_spring_settings(30.0f, 1.0f)};
    entasis_constraint_handle_t constraint = entasis_constraint_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_constraint_add(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET,
                           bodies,
                           UINT32_C(2),
                           &ball_socket,
                           (uint32_t)sizeof(ball_socket),
                           &constraint,
                           &diagnostic) == ENTASIS_STATUS_OK);

    ball_socket.local_offset_a.x = 0.25f;
    ball_socket.local_offset_b.x = -0.25f;
    ENTASIS_TEST_CHECK(entasis_constraint_apply(
                           &world,
                           constraint,
                           ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET,
                           &ball_socket,
                           (uint32_t)sizeof(ball_socket),
                           &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_body_apply_linear_impulse(
                           &world, bodies[0], (entasis_vector3_t){2.0f, 0.5f, 0.0f}, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_apply_angular_impulse(
                           &world, bodies[1], (entasis_vector3_t){0.0f, 0.0f, 0.25f}, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_apply_impulse(
                           &world,
                           bodies[0],
                           (entasis_vector3_t){0.0f, 0.0f, 1.0f},
                           (entasis_vector3_t){0.0f, 1.0f, 0.0f},
                           &diagnostic) == ENTASIS_STATUS_OK);

    entasis_fixed_stepper_t stepper = entasis_fixed_stepper(1.0f / 60.0f, UINT8_C(4));
    uint32_t steps = 0;
    float alpha = 0.0f;
    ENTASIS_TEST_CHECK(entasis_fixed_stepper_update(
                           &stepper, &world, 1.0f / 30.0f, &steps, &alpha, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_state_t state_a = {0};
    entasis_body_state_t state_b = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(&world, bodies[0], &state_a, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get(&world, bodies[1], &state_b, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_constraint_info_t info = {0};
    ENTASIS_TEST_CHECK(entasis_constraint_inspect(&world, constraint, &info, &diagnostic) == ENTASIS_STATUS_OK);
    float impulses[32] = {0};
    uint64_t impulse_written = 0;
    uint64_t impulse_required = 0;
    ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulses(
                           &world,
                           constraint,
                           impulses,
                           UINT64_C(32),
                           &impulse_written,
                           &impulse_required,
                           &diagnostic) == ENTASIS_STATUS_OK);
    float magnitude = 0.0f;
    ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulse_magnitude(
                           &world, constraint, &magnitude, &diagnostic) == ENTASIS_STATUS_OK);

    (void)printf(
        "solve steps=%" PRIu32 " alpha=%" PRIu32 " constraint=%" PRId32
        " type=%" PRId32 " bodies=%" PRIu8 " impulses=%" PRIu64 "/%" PRIu64
        " magnitude=%" PRIu32 " ax=%" PRIu32 " ay=%" PRIu32 " az=%" PRIu32
        " avx=%" PRIu32 " avy=%" PRIu32 " avz=%" PRIu32
        " bx=%" PRIu32 " by=%" PRIu32 " bz=%" PRIu32
        " bvx=%" PRIu32 " bvy=%" PRIu32 " bvz=%" PRIu32 "\n",
        steps,
        float_bits(alpha),
        constraint.value,
        info.type_id,
        info.body_count,
        impulse_written,
        impulse_required,
        float_bits(magnitude),
        float_bits(state_a.pose.position.x),
        float_bits(state_a.pose.position.y),
        float_bits(state_a.pose.position.z),
        float_bits(state_a.velocity.linear.x),
        float_bits(state_a.velocity.linear.y),
        float_bits(state_a.velocity.linear.z),
        float_bits(state_b.pose.position.x),
        float_bits(state_b.pose.position.y),
        float_bits(state_b.pose.position.z),
        float_bits(state_b.velocity.linear.x),
        float_bits(state_b.velocity.linear.y),
        float_bits(state_b.velocity.linear.z));

    entasis_body_description_t applied_a = body_at(4.0f);
    applied_a.velocity.linear = (entasis_vector3_t){3.0f, 2.0f, 1.0f};
    const entasis_body_description_t added_c = body_at(-2.0f);
    entasis_command_t commands[3] = {
        entasis_command_body_apply(bodies[0], applied_a),
        entasis_command_body_add(added_c, INT32_C(0)),
        entasis_command_body_remove(bodies[1])};
    entasis_status_t statuses[3] = {
        ENTASIS_STATUS_INVALID_ARGUMENT,
        ENTASIS_STATUS_INVALID_ARGUMENT,
        ENTASIS_STATUS_INVALID_ARGUMENT};
    entasis_body_handle_t body_results[1] = {entasis_body_handle_invalid()};
    const entasis_command_buffer_t command_buffer = entasis_command_buffer(
        commands,
        UINT64_C(3),
        statuses,
        UINT64_C(3),
        body_results,
        UINT64_C(1),
        NULL,
        UINT64_C(0));
    uint64_t processed = 0;
    ENTASIS_TEST_CHECK(entasis_world_apply_commands(
                           &world, &command_buffer, &processed, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_state_t command_a = {0};
    entasis_body_state_t command_c = {0};
    const entasis_status_t removed_status = entasis_body_get(
        &world, bodies[1], &state_b, &diagnostic);
    ENTASIS_TEST_CHECK(entasis_body_get(
                           &world, bodies[0], &command_a, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get(
                           &world, body_results[0], &command_c, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);

    (void)printf(
        "commands processed=%" PRIu64 " statuses=%" PRId32 ",%" PRId32 ",%" PRId32
        " result=%" PRId32 " removed=%" PRId32 " active=%" PRId64
        " constraints=%" PRId64 " ax=%" PRIu32 " avx=%" PRIu32
        " cx=%" PRIu32 " cvx=%" PRIu32 "\n",
        processed,
        statuses[0],
        statuses[1],
        statuses[2],
        body_results[0].value,
        removed_status,
        stats.active_bodies,
        stats.active_constraints,
        float_bits(command_a.pose.position.x),
        float_bits(command_a.velocity.linear.x),
        float_bits(command_c.pose.position.x),
        float_bits(command_c.velocity.linear.x));

    ENTASIS_TEST_CHECK(entasis_constraint_remove(&world, constraint, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_body_remove(&world, bodies[0], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_remove(&world, body_results[0], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}
