#ifndef ENTASIS_GLOBAL_PRECISION_FIXTURE_H
#define ENTASIS_GLOBAL_PRECISION_FIXTURE_H

#include "entasis.h"
#include "test_support.h"
#include <math.h>
#include <string.h>

#if !ENTASIS_PRECISION_MIXED64
#error "global precision fixture requires the mixed64 library and headers"
#endif

struct global_precision_callbacks
{
    unsigned visits;
    unsigned disposed;
    double first_position;
};

entasis_status_t ENTASIS_CALL global_precision_initialize(void *user, entasis_world_t world)
{
    (void)user;
    (void)world;
    return ENTASIS_STATUS_OK;
}

entasis_status_t ENTASIS_CALL global_precision_prepare(void *user, float dt)
{
    (void)user;
    return (entasis_status_t)(dt > 0.0f ? ENTASIS_STATUS_OK : ENTASIS_STATUS_INVALID_ARGUMENT);
}

void ENTASIS_CALL global_precision_integrate(void *user, const entasis_global_pose_integration_view_t *view)
{
    struct global_precision_callbacks *state = (struct global_precision_callbacks *)user;
    const int32_t *mask = (const int32_t *)view->active_mask;
    unsigned lane;
    for (lane = 0; lane < 8; ++lane)
    {
        if (mask[lane] != 0)
        {
            if (state->visits == 0)
            {
                state->first_position = view->position[lane].x;
            }
            state->visits += 1;
        }
    }
}

void ENTASIS_CALL global_precision_dispose(void *user)
{
    struct global_precision_callbacks *state = (struct global_precision_callbacks *)user;
    state->disposed += 1;
}

int global_precision_fixture(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    entasis_diagnostic_t diagnostic;
    entasis_world_t world;
    memset(&world, 0, sizeof(world));
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity.x = 0.0f;
    description.gravity.y = 0.0f;
    description.gravity.z = 0.0f;
    description.threading.worker_count = 1;
    struct global_precision_callbacks callback_state;
    memset(&callback_state, 0, sizeof(callback_state));
    entasis_global_velocity_callbacks_t callbacks;
    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.struct_size = (uint32_t)sizeof(callbacks);
    callbacks.struct_version = UINT32_C(1);
    callbacks.integrate_kinematic_velocity = ENTASIS_TRUE;
    callbacks.user_context = &callback_state;
    callbacks.initialize = global_precision_initialize;
    callbacks.prepare = global_precision_prepare;
    callbacks.integrate = global_precision_integrate;
    callbacks.dispose = global_precision_dispose;
    ENTASIS_TEST_CHECK(entasis_world_init_global(&world, &description, NULL, NULL, &callbacks, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(0.5f);
    entasis_shape_handle_t shape;
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_vector3_t zero = {0.0f, 0.0f, 0.0f};
    const entasis_quaternion_t identity = {0.0f, 0.0f, 0.0f, 1.0f};
    const entasis_body_velocity_t velocity = entasis_velocity(zero, zero);
    entasis_body_description_t bodies[2];
    bodies[0] = entasis_body_kinematic(shape, entasis_pose(zero, identity), velocity, entasis_body_activity(-1.0f, 255));
    bodies[1] = bodies[0];
    const entasis_global_position_t positions[2] = {
        {1000000000.002, 3.0, 0.0}, {1000000000.004, 3.0, 0.0}};
    entasis_body_handle_t handles[2];
    uint64_t completed;
    ENTASIS_TEST_CHECK(entasis_body_add_batch_global(&world, bodies, 2, positions, 2, handles, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == 2);
    entasis_global_pose_t pose;
    ENTASIS_TEST_CHECK(entasis_body_get_global_pose(&world, handles[0], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pose.position.x == positions[0].x);
    const double first_position = pose.position.x;
    ENTASIS_TEST_CHECK(entasis_body_get_global_pose(&world, handles[1], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pose.position.x == positions[1].x && positions[0].x != positions[1].x);
    const double observed_gap = pose.position.x - first_position;
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(&world, handles[0], &velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get_global_pose(&world, handles[0], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pose.position.x == positions[0].x);

    entasis_static_description_t statics[2];
    statics[0] = entasis_static_body(shape, entasis_pose(zero, identity), entasis_ccd_passive());
    statics[1] = statics[0];
    const entasis_global_position_t static_positions[2] = {
        {1000000000.002, -3.0, 0.0}, {1000000032.004, -3.0, 0.0}};
    entasis_static_handle_t fixed[2];
    ENTASIS_TEST_CHECK(entasis_static_add_batch_global(&world, statics, 2, static_positions, 2, ENTASIS_AWAKENING_NONE, fixed, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == 2);
    entasis_static_view_t static_view;
    ENTASIS_TEST_CHECK(entasis_static_view(&world, &static_view, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(static_view.count == 2 && sizeof(static_view.records[0]) == 52);
    ENTASIS_TEST_CHECK(static_view.records[0].pose.position.x == (float)static_positions[0].x);
    ENTASIS_TEST_CHECK(static_view.records[1].pose.position.x == (float)static_positions[1].x);
    entasis_bounding_box_t bounds;
    ENTASIS_TEST_CHECK(entasis_body_bounds(&world, handles[0], &bounds, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK((double)bounds.min.x <= positions[0].x - 0.5 && (double)bounds.max.x >= positions[0].x + 0.5);
    ENTASIS_TEST_CHECK(entasis_static_bounds(&world, fixed[0], &bounds, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK((double)bounds.min.x <= static_positions[0].x - 0.5 && (double)bounds.max.x >= static_positions[0].x + 0.5);

    entasis_query_context_t query;
    memset(&query, 0, sizeof(query));
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &world, NULL, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    int32_t stack[128];
    const entasis_world_precision_segment_t segment = {
        {positions[0].x - 2.0, 3.0, 0.0}, {positions[0].x + 2.0, 3.0, 0.0}};
    entasis_global_ray_hit_t ray_hit;
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_global_closest(&query, segment, NULL, &ray_hit, stack, 128, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(fabs(ray_hit.location.x - (positions[0].x - 0.5)) < 0.000001);
    entasis_global_pose_t a = {positions[0], identity};
    entasis_global_pose_t b = {{positions[0].x + 2.002, 3.0, 0.0}, identity};
    entasis_global_shape_distance_result_t distance;
    ENTASIS_TEST_CHECK(entasis_query_context_shape_distance_global(&query, shape, a, shape, b, NULL, &distance, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(fabs((double)distance.geometry.distance - 1.002) < 0.001);
    ENTASIS_TEST_CHECK(fabs(distance.geometry.point_a.x - (a.position.x + 0.5)) < 0.001);
    ENTASIS_TEST_CHECK(fabs(distance.geometry.point_b.x - (b.position.x - 0.5)) < 0.001);
    printf("GLOBAL_PRECISION_VALUES body=%.9f gap=%.9f hit=%.9f a=%.9f b=%.9f distance=%.9f\n", first_position, observed_gap, ray_hit.location.x, distance.geometry.point_a.x, distance.geometry.point_b.x, (double)distance.geometry.distance);

    entasis_global_query_t inputs[2];
    inputs[0] = entasis_query_ray_global_closest(segment, NULL);
    inputs[1] = entasis_query_ray_global_any(segment, NULL);
    entasis_global_query_result_t results[2];
    entasis_global_query_scratch_t scratch;
    memset(&scratch, 0, sizeof(scratch));
    scratch.traversal_stack = stack;
    scratch.stack_capacity = 128;
    ENTASIS_TEST_CHECK(entasis_query_context_query_batch_global(&query, inputs, 2, results, 2, &scratch, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[0].hit == ENTASIS_TRUE && results[1].hit == ENTASIS_TRUE);
    memcpy(&ray_hit, &results[0].payload[0], sizeof(ray_hit));
    ENTASIS_TEST_CHECK(fabs(ray_hit.location.x - (positions[0].x - 0.5)) < 0.000001);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(callback_state.visits > 0 && callback_state.first_position == positions[0].x);
    ENTASIS_TEST_CHECK(entasis_world_enable_body_control(&world, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_global_pose_t target = {{positions[0].x + 0.002, 3.0, 0.0}, identity};
    ENTASIS_TEST_CHECK(entasis_body_set_global_kinematic_target(&world, handles[0], &target, ENTASIS_BODY_CONTROL_WAKE, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get_global_kinematic_target(&world, handles[0], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pose.position.x == target.position.x);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get_global_pose(&world, handles[0], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(fabs(pose.position.x - target.position.x) < 0.001);

    entasis_global_state_selection_t selection;
    entasis_global_state_snapshot_t snapshot;
    memset(&selection, 0, sizeof(selection));
    memset(&snapshot, 0, sizeof(snapshot));
    entasis_global_body_state_t body_states[2];
    entasis_global_static_state_t static_states[2];
    selection.bodies = handles;
    selection.bodies_count = 2;
    selection.statics = fixed;
    selection.statics_count = 2;
    snapshot.bodies = body_states;
    snapshot.bodies_capacity = 2;
    snapshot.statics = static_states;
    snapshot.statics_capacity = 2;
    entasis_global_state_counts_t counts;
    ENTASIS_TEST_CHECK(entasis_world_export_global_states(&world, &selection, &snapshot, &counts, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(counts.bodies == 2 && counts.statics == 2);
    ENTASIS_TEST_CHECK(static_states[1].pose.position.x == static_positions[1].x);
    body_states[0].pose.position.x += 0.002;
    body_states[0].velocity = velocity;
    ENTASIS_TEST_CHECK(entasis_world_apply_global_states(&world, &selection, &snapshot, ENTASIS_AWAKENING_NONE, ENTASIS_BODY_CONTROL_WAKE, &counts, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get_global_pose(&world, handles[0], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pose.position.x == body_states[0].pose.position.x);

    entasis_active_body_view_t active;
    ENTASIS_TEST_CHECK(entasis_active_body_view(&world, &active, &diagnostic) == ENTASIS_STATUS_OK);
    uint64_t row;
    for (row = 0; row < active.count; ++row)
    {
        if (active.handles[row].value == handles[0].value)
        {
            active.dynamics[row].motion.pose.position.x = 64.0f;
        }
    }
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get_global_pose(&world, handles[0], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pose.position.x == 64.0);
    ENTASIS_TEST_CHECK(entasis_body_get_global_pose(&world, handles[1], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pose.position.x == positions[1].x);

    entasis_command_t commands[3];
    bodies[0].pose.position.x = NAN;
    commands[0] = entasis_command_body_add(bodies[0], 0);
    commands[1] = entasis_command_body_remove(handles[1]);
    commands[2] = entasis_command_body_add(bodies[0], 1);
    entasis_status_t statuses[3] = {ENTASIS_STATUS_DISPOSED, ENTASIS_STATUS_DISPOSED, ENTASIS_STATUS_DISPOSED};
    entasis_body_handle_t command_results[2];
    command_results[0].value = 777;
    command_results[1].value = 778;
    const entasis_global_position_t command_positions[3] = {positions[0], {NAN, 0.0, 0.0}, {INFINITY, 0.0, 0.0}};
    entasis_command_buffer_t buffer = entasis_command_buffer(commands, 3, statuses, 3, command_results, 2, NULL, 0);
    uint64_t processed;
    ENTASIS_TEST_CHECK(entasis_world_apply_commands_global(&world, &buffer, command_positions, 2, &processed, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(command_results[0].value == 777 && statuses[0] == ENTASIS_STATUS_DISPOSED);
    ENTASIS_TEST_CHECK(entasis_world_apply_commands_global(&world, &buffer, command_positions, 3, &processed, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(processed == 2 && statuses[0] == ENTASIS_STATUS_OK && statuses[1] == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(statuses[2] == ENTASIS_STATUS_INVALID_ARGUMENT && command_results[1].value == -1);
    ENTASIS_TEST_CHECK(entasis_body_get_global_pose(&world, command_results[0], &pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pose.position.x == positions[0].x);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(callback_state.disposed == 1);
    puts("GLOBAL_PRECISION_OK");
    return 0;
}

#endif
