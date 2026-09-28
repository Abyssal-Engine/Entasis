#include "entasis.h"
#include "test_support.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct constraint_case_t
{
    entasis_constraint_type_id_t type_id;
    uint32_t body_count;
    const void *description;
    uint32_t description_size;
} constraint_case_t;

#if defined(_WIN32)
typedef __declspec(align(16)) struct constraint_storage_t
{
    unsigned char bytes[128];
} constraint_storage_t;
#else
typedef union constraint_storage_t
{
    max_align_t alignment;
    unsigned char bytes[128];
} constraint_storage_t;
#endif

static entasis_quaternion_t identity_orientation(void)
{
    return (entasis_quaternion_t){0.0f, 0.0f, 0.0f, 1.0f};
}

static entasis_world_description_t constraint_world_description(void)
{
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    description.threading.worker_count = UINT32_C(1);
    description.capacity.bodies = INT32_C(128);
    description.capacity.constraints = INT32_C(512);
    description.capacity.initial_constraints_per_type_batch = INT32_C(16);
    description.capacity.minimum_constraints_per_body = INT32_C(16);
    description.capacity.broad_phase_candidates = INT32_C(512);
    description.capacity.pairs = INT32_C(512);
    description.capacity.collision_child_pairs = INT32_C(512);
    return description;
}

static entasis_body_description_t constraint_body(float x)
{
    entasis_body_inertia_t inertia = {0};
    inertia.inverse_inertia_tensor.xx = 1.0f;
    inertia.inverse_inertia_tensor.yy = 1.0f;
    inertia.inverse_inertia_tensor.zz = 1.0f;
    inertia.inverse_mass = 1.0f;
    return entasis_body_shapeless(
        inertia,
        entasis_pose((entasis_vector3_t){x, 0.0f, 0.0f}, identity_orientation()),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(0.01f, UINT8_C(255)));
}

static int run_constraint_case(
    entasis_world_t *world,
    const entasis_body_handle_t bodies[4],
    const constraint_case_t *test_case,
    entasis_diagnostic_t *diagnostic)
{
    entasis_constraint_handle_t handle = entasis_constraint_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_constraint_type_id(test_case->type_id) == test_case->type_id);
    ENTASIS_TEST_CHECK(entasis_constraint_body_count(test_case->type_id) == test_case->body_count);

    ENTASIS_TEST_CHECK(entasis_constraint_add(
                           world,
                           test_case->type_id,
                           bodies,
                           test_case->body_count,
                           test_case->description,
                           test_case->description_size,
                           &handle,
                           diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_constraint_handle_is_valid(handle) == ENTASIS_TRUE);

    entasis_constraint_info_t info = {0};
    ENTASIS_TEST_CHECK(entasis_constraint_inspect(world, handle, &info, diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.handle.value == handle.value);
    ENTASIS_TEST_CHECK(info.type_id == test_case->type_id);
    ENTASIS_TEST_CHECK(info.body_count == test_case->body_count);
    ENTASIS_TEST_CHECK(info.state == ENTASIS_CONSTRAINT_STATE_ACTIVE);
    for (uint32_t index = 0; index < test_case->body_count; ++index)
    {
        ENTASIS_TEST_CHECK(info.bodies[index].value == bodies[index].value);
    }

    constraint_storage_t readback = {0};
    ENTASIS_TEST_CHECK(entasis_constraint_get(
                           world,
                           handle,
                           test_case->type_id,
                           readback.bytes,
                           test_case->description_size,
                           diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(memcmp(readback.bytes, test_case->description, test_case->description_size) == 0);

    if (test_case->type_id == ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET)
    {
        entasis_angular_hinge_t wrong_type_readback = {0};
        ENTASIS_TEST_CHECK(sizeof(wrong_type_readback) == test_case->description_size);
        ENTASIS_TEST_CHECK(entasis_constraint_get(
                               world,
                               handle,
                               ENTASIS_CONSTRAINT_TYPE_ANGULAR_HINGE,
                               &wrong_type_readback,
                               (uint32_t)sizeof(wrong_type_readback),
                               diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_constraint_apply(
                               world,
                               handle,
                               ENTASIS_CONSTRAINT_TYPE_ANGULAR_HINGE,
                               &wrong_type_readback,
                               (uint32_t)sizeof(wrong_type_readback),
                               diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    }

    ENTASIS_TEST_CHECK(entasis_constraint_apply(
                           world,
                           handle,
                           test_case->type_id,
                           test_case->description,
                           test_case->description_size,
                           diagnostic) == ENTASIS_STATUS_OK);

    uint64_t written = UINT64_C(99);
    uint64_t required = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_constraint_connected_bodies(
                           world, handle, NULL, UINT64_C(0), &written, &required, diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(written == UINT64_C(0));
    ENTASIS_TEST_CHECK(required == test_case->body_count);

    entasis_body_handle_t connected[4] = {{0}};
    ENTASIS_TEST_CHECK(entasis_constraint_connected_bodies(
                           world, handle, connected, UINT64_C(4), &written, &required, diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == test_case->body_count);
    ENTASIS_TEST_CHECK(required == test_case->body_count);
    for (uint32_t index = 0; index < test_case->body_count; ++index)
    {
        ENTASIS_TEST_CHECK(connected[index].value == bodies[index].value);
    }

    ENTASIS_TEST_CHECK(entasis_constraint_remove(world, handle, diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_constraint_remove(world, handle, diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    return 0;
}

static int test_all_constraint_families(void)
{
    const entasis_vector3_t zero = {0.0f, 0.0f, 0.0f};
    const entasis_vector3_t axis_x = {1.0f, 0.0f, 0.0f};
    const entasis_vector3_t axis_y = {0.0f, 1.0f, 0.0f};
    const entasis_quaternion_t identity = identity_orientation();
    const entasis_spring_settings_t spring = entasis_spring_settings(30.0f, 1.0f);
    const entasis_servo_settings_t servo = entasis_servo_settings(10.0f, 0.1f, 100.0f);
    const entasis_motor_settings_t motor = entasis_motor_settings(100.0f, 1.0f);

    const entasis_angular_axis_gear_motor_t angular_axis_gear_motor = {axis_x, 1.0f, motor};
    const entasis_angular_axis_motor_t angular_axis_motor = {axis_x, 0.0f, motor};
    const entasis_angular_hinge_t angular_hinge = {axis_x, axis_x, spring};
    const entasis_angular_motor_t angular_motor = {zero, motor};
    const entasis_angular_servo_t angular_servo = {identity, spring, servo};
    const entasis_angular_swivel_hinge_t angular_swivel_hinge = {axis_x, axis_y, spring};
    const entasis_area_constraint_t area = {1.0f, spring};
    const entasis_ball_socket_t ball_socket = {zero, zero, spring};
    const entasis_ball_socket_motor_t ball_socket_motor = {zero, zero, motor};
    const entasis_ball_socket_servo_t ball_socket_servo = {zero, zero, spring, servo};
    const entasis_center_distance_constraint_t center_distance = {1.0f, spring};
    const entasis_center_distance_limit_t center_distance_limit = {0.0f, 2.0f, spring};
    const entasis_distance_limit_t distance_limit = {zero, zero, 0.0f, 2.0f, spring};
    const entasis_distance_servo_t distance_servo = {zero, zero, 1.0f, servo, spring};
    const entasis_hinge_t hinge = {zero, axis_x, zero, axis_x, spring};
    const entasis_linear_axis_limit_t linear_axis_limit = {zero, zero, axis_x, -1.0f, 1.0f, spring};
    const entasis_linear_axis_motor_t linear_axis_motor = {zero, zero, axis_x, 0.0f, motor};
    const entasis_linear_axis_servo_t linear_axis_servo = {zero, zero, axis_x, 0.0f, servo, spring};
    const entasis_one_body_angular_motor_t one_body_angular_motor = {zero, motor};
    const entasis_one_body_angular_servo_t one_body_angular_servo = {identity, spring, servo};
    const entasis_one_body_linear_motor_t one_body_linear_motor = {zero, zero, motor};
    const entasis_one_body_linear_servo_t one_body_linear_servo = {zero, zero, spring, servo};
    const entasis_point_on_line_servo_t point_on_line_servo = {zero, zero, axis_x, servo, spring};
    const entasis_swing_limit_t swing_limit = {axis_x, axis_x, 0.5f, spring};
    const entasis_swivel_hinge_t swivel_hinge = {zero, axis_x, zero, axis_x, spring};
    const entasis_twist_limit_t twist_limit = {identity, identity, -1.0f, 1.0f, spring};
    const entasis_twist_motor_t twist_motor = {axis_x, axis_x, 0.0f, motor};
    const entasis_twist_servo_t twist_servo = {identity, identity, 0.0f, spring, servo};
    const entasis_volume_constraint_t volume = {1.0f, spring};
    const entasis_weld_t weld = {zero, identity, spring};

#define CONSTRAINT_CASE(type_constant, body_arity, value)                                                        \
    {                                                                                                            \
        (entasis_constraint_type_id_t)(type_constant), (uint32_t)(body_arity), &(value), (uint32_t)sizeof(value) \
    }
    const constraint_case_t cases[] = {
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ANGULAR_AXIS_GEAR_MOTOR, 2, angular_axis_gear_motor),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ANGULAR_AXIS_MOTOR, 2, angular_axis_motor),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ANGULAR_HINGE, 2, angular_hinge),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ANGULAR_MOTOR, 2, angular_motor),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ANGULAR_SERVO, 2, angular_servo),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ANGULAR_SWIVEL_HINGE, 2, angular_swivel_hinge),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_AREA_CONSTRAINT, 3, area),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET, 2, ball_socket),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET_MOTOR, 2, ball_socket_motor),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET_SERVO, 2, ball_socket_servo),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_CENTER_DISTANCE_CONSTRAINT, 2, center_distance),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_CENTER_DISTANCE_LIMIT, 2, center_distance_limit),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_DISTANCE_LIMIT, 2, distance_limit),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_DISTANCE_SERVO, 2, distance_servo),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_HINGE, 2, hinge),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_LINEAR_AXIS_LIMIT, 2, linear_axis_limit),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_LINEAR_AXIS_MOTOR, 2, linear_axis_motor),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_LINEAR_AXIS_SERVO, 2, linear_axis_servo),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ONE_BODY_ANGULAR_MOTOR, 1, one_body_angular_motor),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ONE_BODY_ANGULAR_SERVO, 1, one_body_angular_servo),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ONE_BODY_LINEAR_MOTOR, 1, one_body_linear_motor),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_ONE_BODY_LINEAR_SERVO, 1, one_body_linear_servo),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_POINT_ON_LINE_SERVO, 2, point_on_line_servo),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_SWING_LIMIT, 2, swing_limit),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_SWIVEL_HINGE, 2, swivel_hinge),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_TWIST_LIMIT, 2, twist_limit),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_TWIST_MOTOR, 2, twist_motor),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_TWIST_SERVO, 2, twist_servo),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_VOLUME_CONSTRAINT, 4, volume),
        CONSTRAINT_CASE(ENTASIS_CONSTRAINT_TYPE_WELD, 2, weld)};
#undef CONSTRAINT_CASE

    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = constraint_world_description();
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_handle_t bodies[4] = {{0}};
    for (uint32_t index = 0; index < UINT32_C(4); ++index)
    {
        const entasis_body_description_t body_description = constraint_body((float)index);
        ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_description, &bodies[index], &diagnostic) == ENTASIS_STATUS_OK);
    }

    for (size_t index = 0; index < sizeof(cases) / sizeof(cases[0]); ++index)
    {
        ENTASIS_TEST_CHECK(run_constraint_case(&world, bodies, &cases[index], &diagnostic) == 0);
    }

    entasis_constraint_handle_t invalid = {INT32_C(123)};
    ENTASIS_TEST_CHECK(entasis_constraint_add(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET,
                           bodies,
                           UINT32_C(2),
                           &ball_socket,
                           (uint32_t)sizeof(ball_socket) - UINT32_C(1),
                           &invalid,
                           &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(invalid.value == -1);

    constraint_storage_t misaligned = {0};
    (void)memcpy(&misaligned.bytes[1], &ball_socket, sizeof(ball_socket));
    ENTASIS_TEST_CHECK(entasis_constraint_add(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET,
                           bodies,
                           UINT32_C(2),
                           &misaligned.bytes[1],
                           (uint32_t)sizeof(ball_socket),
                           &invalid,
                           &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);

    uint64_t count = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_constraint_count(&world, &count, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(count == UINT64_C(0));
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_constraint_batches_and_prefix_failures(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = constraint_world_description();
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_handle_t bodies[8] = {{0}};
    for (uint32_t index = 0; index < UINT32_C(8); ++index)
    {
        const entasis_body_description_t body_description = constraint_body((float)index);
        ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_description, &bodies[index], &diagnostic) == ENTASIS_STATUS_OK);
    }

    const entasis_spring_settings_t spring = entasis_spring_settings(20.0f, 1.0f);
    const entasis_motor_settings_t motor = entasis_motor_settings(50.0f, 1.0f);
    entasis_one_body_linear_motor_t one_body[3] = {
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, motor},
        {{0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, motor},
        {{0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, motor}};
    entasis_constraint_handle_t handles[3] = {
        entasis_constraint_handle_invalid(),
        entasis_constraint_handle_invalid(),
        entasis_constraint_handle_invalid()};
    uint64_t completed = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_constraint_add_batch(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_ONE_BODY_LINEAR_MOTOR,
                           bodies,
                           UINT64_C(3),
                           one_body,
                           (uint32_t)sizeof(one_body[0]),
                           UINT64_C(3),
                           handles,
                           &completed,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == UINT64_C(3));

    ENTASIS_TEST_CHECK(entasis_constraint_add_batch(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_ONE_BODY_LINEAR_MOTOR,
                           bodies,
                           UINT64_C(3),
                           one_body,
                           (uint32_t)sizeof(one_body[0]) - UINT32_C(1),
                           UINT64_C(3),
                           handles,
                           &completed,
                           &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(completed == UINT64_C(0));

    entasis_constraint_handle_t apply_handles[3] = {
        handles[0], entasis_constraint_handle_invalid(), handles[2]};
    completed = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_constraint_apply_batch(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_ONE_BODY_LINEAR_MOTOR,
                           apply_handles,
                           one_body,
                           (uint32_t)sizeof(one_body[0]),
                           UINT64_C(3),
                           &completed,
                           &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(completed == UINT64_C(1));

    completed = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_constraint_remove_batch(
                           &world, apply_handles, UINT64_C(3), &completed, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(completed == UINT64_C(1));
    ENTASIS_TEST_CHECK(entasis_constraint_remove(&world, handles[1], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_constraint_remove(&world, handles[2], &diagnostic) == ENTASIS_STATUS_OK);

    entasis_ball_socket_t ball_sockets[2] = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, spring},
        {{0.25f, 0.0f, 0.0f}, {-0.25f, 0.0f, 0.0f}, spring}};
    const entasis_body_handle_t two_body_pairs[4] = {bodies[0], bodies[1], bodies[2], bodies[3]};
    entasis_constraint_handle_t two_handles[2] = {{-1}, {-1}};
    ENTASIS_TEST_CHECK(entasis_constraint_add_batch(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET,
                           two_body_pairs,
                           UINT64_C(4),
                           ball_sockets,
                           (uint32_t)sizeof(ball_sockets[0]),
                           UINT64_C(2),
                           two_handles,
                           &completed,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == UINT64_C(2));

    entasis_area_constraint_t areas[2] = {{1.0f, spring}, {2.0f, spring}};
    const entasis_body_handle_t three_body_groups[6] = {
        bodies[0], bodies[1], bodies[2], bodies[3], bodies[4], bodies[5]};
    entasis_constraint_handle_t three_handles[2] = {{-1}, {-1}};
    ENTASIS_TEST_CHECK(entasis_constraint_add_batch(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_AREA_CONSTRAINT,
                           three_body_groups,
                           UINT64_C(6),
                           areas,
                           (uint32_t)sizeof(areas[0]),
                           UINT64_C(2),
                           three_handles,
                           &completed,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == UINT64_C(2));

    entasis_volume_constraint_t volumes[2] = {{1.0f, spring}, {2.0f, spring}};
    const entasis_body_handle_t four_body_groups[8] = {
        bodies[0], bodies[1], bodies[2], bodies[3],
        bodies[4], bodies[5], bodies[6], bodies[7]};
    entasis_constraint_handle_t four_handles[2] = {{-1}, {-1}};
    ENTASIS_TEST_CHECK(entasis_constraint_add_batch(
                           &world,
                           ENTASIS_CONSTRAINT_TYPE_VOLUME_CONSTRAINT,
                           four_body_groups,
                           UINT64_C(8),
                           volumes,
                           (uint32_t)sizeof(volumes[0]),
                           UINT64_C(2),
                           four_handles,
                           &completed,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == UINT64_C(2));

    uint64_t count = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_constraint_count(&world, &count, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(count == UINT64_C(6));

    entasis_constraint_info_t infos[4] = {0};
    uint64_t written = UINT64_C(0);
    uint64_t required = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_constraint_enumerate(
                           &world, infos, UINT64_C(4), &written, &required, &diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(written == UINT64_C(4));
    ENTASIS_TEST_CHECK(required == UINT64_C(6));

    ENTASIS_TEST_CHECK(entasis_constraint_remove_batch(
                           &world, two_handles, UINT64_C(2), &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_constraint_remove_batch(
                           &world, three_handles, UINT64_C(2), &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_constraint_remove_batch(
                           &world, four_handles, UINT64_C(2), &completed, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_all_constraint_families() == 0);
    ENTASIS_TEST_CHECK(test_constraint_batches_and_prefix_failures() == 0);
    return 0;
}
