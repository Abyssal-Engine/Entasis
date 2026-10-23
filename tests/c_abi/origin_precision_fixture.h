#ifndef ENTASIS_ORIGIN_PRECISION_FIXTURE_H
#define ENTASIS_ORIGIN_PRECISION_FIXTURE_H

#include "entasis.h"
#include "test_support.h"
#include <math.h>
#include <string.h>

int origin_precision_fixture(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    entasis_diagnostic_t diagnostic;
    const entasis_vector3_t zero = {0.0f, 0.0f, 0.0f};
    const entasis_global_position_t origin = entasis_global_position(1000000000.0, -1000000000.0, 500000000.0);
    entasis_origin_frame_t frame;
    ENTASIS_TEST_CHECK(entasis_origin_frame_at(&origin, &frame, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_global_position_t application = entasis_global_position(origin.x + 0.002, origin.y + 2.0, origin.z);
    entasis_vector3_t local;
    ENTASIS_TEST_CHECK(entasis_origin_frame_to_local(&frame, &application, &local, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(fabs((double)local.x - 0.002) < 0.000001);
    entasis_global_position_t reconstructed;
    ENTASIS_TEST_CHECK(entasis_origin_frame_to_global(&frame, &local, &reconstructed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(fabs(reconstructed.x - application.x) < 0.000001);
    entasis_vector3_t difference;
    ENTASIS_TEST_CHECK(entasis_global_position_delta(&origin, &application, &difference, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(difference.x == local.x && difference.y == local.y && difference.z == local.z);

    entasis_world_t world;
    memset(&world, 0, sizeof(world));
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity.x = 0.0f;
    description.gravity.y = 0.0f;
    description.gravity.z = 0.0f;
    description.threading.worker_count = 1;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(0.5f);
    entasis_shape_handle_t shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_quaternion_t identity = {0.0f, 0.0f, 0.0f, 1.0f};
    const entasis_body_velocity_t velocity = entasis_velocity(zero, zero);
    const entasis_body_description_t body = entasis_body_kinematic(shape, entasis_pose(local, identity), velocity, entasis_body_activity(-1.0f, 255));
    entasis_body_handle_t handle = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body, &handle, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_query_context_t query;
    memset(&query, 0, sizeof(query));
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &world, NULL, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_active_body_view_t view;
    ENTASIS_TEST_CHECK(entasis_active_body_view(&world, &view, &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_vector3_t delta = {256.0f, 0.0f, 0.0f};
    const entasis_vector3_t invalid = {NAN, 0.0f, 0.0f};
    ENTASIS_TEST_CHECK(entasis_world_shift_origin(&world, &frame, &invalid, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(frame.origin.x == origin.x);
    ENTASIS_TEST_CHECK(entasis_body_view_valid(&world, &view) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_world_shift_origin(&world, &frame, &zero, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(frame.origin.x == origin.x);
    ENTASIS_TEST_CHECK(entasis_body_view_valid(&world, &view) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(entasis_world_shift_origin(&world, &frame, &delta, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(frame.origin.x == origin.x + 256.0);
    ENTASIS_TEST_CHECK(entasis_body_view_valid(&world, &view) == ENTASIS_FALSE);
    entasis_body_state_t after;
    ENTASIS_TEST_CHECK(entasis_body_get(&world, handle, &after, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_origin_frame_to_global(&frame, &after.pose.position, &reconstructed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(fabs(reconstructed.x - application.x) < 0.001);
    ENTASIS_TEST_CHECK(fabs(reconstructed.y - application.y) < 0.001);
    const entasis_vector3_t ray_origin = {after.pose.position.x - 2.0f, after.pose.position.y, after.pose.position.z};
    const entasis_vector3_t ray_direction = {1.0f, 0.0f, 0.0f};
    const entasis_ray_t ray = entasis_ray(ray_origin, ray_direction, 4.0f);
    entasis_ray_hit_t hit;
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&query, ray, NULL, &hit, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_handle_t hit_handle = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_collidable_body_handle(hit.collidable, &hit_handle, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(hit_handle.value == handle.value);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    printf("ORIGIN_PRECISION_VALUES x=%.9f y=%.9f origin=%.9f handle=%d\n", reconstructed.x, reconstructed.y, frame.origin.x, hit_handle.value);
    puts("ORIGIN_PRECISION_OK");
    return 0;
}

#endif
