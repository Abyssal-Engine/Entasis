#include "entasis.h"
#include "test_support.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#define BODY_COUNT UINT64_C(64)
#define STATIC_COUNT UINT64_C(32)

static entasis_rigid_pose_t pose_at(float x, float y, float z)
{
    const entasis_quaternion_t identity = {0.0f, 0.0f, 0.0f, 1.0f};
    return entasis_pose((entasis_vector3_t){x, y, z}, identity);
}

int main(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    description.capacity.bodies = 128;
    description.capacity.statics = 64;
    description.capacity.shapes_per_type = 16;
    description.capacity.broad_phase_candidates = 512;
    description.capacity.pairs = 512;
    description.capacity.collision_child_pairs = 512;

    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere_value = entasis_sphere(0.5f);
    const entasis_box_t box_value = entasis_box(1.0f, 1.0f, 1.0f);
    entasis_shape_handle_t sphere = entasis_shape_handle_invalid();
    entasis_shape_handle_t box = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, &sphere, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_BOX, &box_value, &box, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, 1.0f, &inertia) == ENTASIS_STATUS_OK);

    entasis_body_description_t body_descriptions[BODY_COUNT];
    entasis_body_handle_t body_handles[BODY_COUNT];
    entasis_body_state_t body_states[BODY_COUNT];
    for (uint64_t index = 0; index < BODY_COUNT; ++index)
    {
        body_descriptions[index] = entasis_body_dynamic(
            sphere,
            inertia,
            pose_at((float)index, 1.0f, 0.0f),
            entasis_velocity((entasis_vector3_t){(float)(index + UINT64_C(1)), 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
            entasis_body_activity(-1.0f, UINT8_C(255)));
    }
    uint64_t body_written = 0;
    ENTASIS_TEST_CHECK(entasis_body_add_batch(&world, body_descriptions, BODY_COUNT, body_handles, &body_written, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(body_written == BODY_COUNT);

    entasis_static_description_t static_descriptions[STATIC_COUNT];
    entasis_static_handle_t static_handles[STATIC_COUNT];
    entasis_static_state_t static_states[STATIC_COUNT];
    for (uint64_t index = 0; index < STATIC_COUNT; ++index)
    {
        static_descriptions[index] = entasis_static_body(box, pose_at((float)index, -1.0f, 0.0f), entasis_ccd_discrete());
    }
    uint64_t static_written = 0;
    ENTASIS_TEST_CHECK(entasis_static_add_batch(
                           &world,
                           static_descriptions,
                           STATIC_COUNT,
                           ENTASIS_AWAKENING_NONE,
                           static_handles,
                           &static_written,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(static_written == STATIC_COUNT);

    for (uint64_t index = 0; index < BODY_COUNT; ++index)
    {
        body_descriptions[index].pose.position = (entasis_vector3_t){
            (float)(index * UINT64_C(2)),
            (float)(index % UINT64_C(7)),
            -(float)index};
        body_descriptions[index].velocity.linear = (entasis_vector3_t){
            (float)(UINT64_C(100) + index),
            (float)(index * UINT64_C(3)),
            0.0f};
    }
    uint64_t body_applied = 0;
    ENTASIS_TEST_CHECK(entasis_body_apply_batch(
                           &world,
                           body_handles,
                           body_descriptions,
                           BODY_COUNT,
                           &body_applied,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(body_applied == BODY_COUNT);

    for (uint64_t index = 0; index < STATIC_COUNT; ++index)
    {
        static_descriptions[index].pose.position = (entasis_vector3_t){
            (float)(index * UINT64_C(3)), -2.0f, (float)index};
    }
    uint64_t static_applied = 0;
    ENTASIS_TEST_CHECK(entasis_static_apply_batch(
                           &world,
                           static_handles,
                           static_descriptions,
                           STATIC_COUNT,
                           ENTASIS_AWAKENING_NONE,
                           &static_applied,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(static_applied == STATIC_COUNT);

    uint64_t body_read = 0;
    ENTASIS_TEST_CHECK(entasis_body_get_batch(
                           &world, body_handles, BODY_COUNT, body_states, &body_read, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(body_read == BODY_COUNT);
    int64_t body_position_sum = 0;
    int64_t body_velocity_sum = 0;
    for (uint64_t index = 0; index < BODY_COUNT; ++index)
    {
        body_position_sum += (int64_t)body_states[index].pose.position.x;
        body_position_sum += (int64_t)body_states[index].pose.position.y;
        body_position_sum += (int64_t)body_states[index].pose.position.z;
        body_velocity_sum += (int64_t)body_states[index].velocity.linear.x;
        body_velocity_sum += (int64_t)body_states[index].velocity.linear.y;
    }

    uint64_t static_read = 0;
    ENTASIS_TEST_CHECK(entasis_static_get_batch(
                           &world, static_handles, STATIC_COUNT, static_states, &static_read, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(static_read == STATIC_COUNT);
    int64_t static_position_sum = 0;
    for (uint64_t index = 0; index < STATIC_COUNT; ++index)
    {
        static_position_sum += (int64_t)static_states[index].pose.position.x;
        static_position_sum += (int64_t)static_states[index].pose.position.y;
        static_position_sum += (int64_t)static_states[index].pose.position.z;
    }

    entasis_shape_info_t sphere_info = {0};
    entasis_shape_info_t box_info = {0};
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, sphere, &sphere_info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, box, &box_info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    (void)printf(
        "bodies=%" PRIu64 " statics=%" PRIu64 " body_position_sum=%" PRId64
        " body_velocity_sum=%" PRId64 " static_position_sum=%" PRId64
        " sphere_refs=%" PRIu64 " box_refs=%" PRIu64 " active=%" PRId64
        " sleeping=%" PRId64 " registered_shapes=%" PRId64
        " first_body=%" PRId32 " last_body=%" PRId32
        " first_static=%" PRId32 " last_static=%" PRId32 "\n",
        body_written,
        static_written,
        body_position_sum,
        body_velocity_sum,
        static_position_sum,
        sphere_info.reference_count,
        box_info.reference_count,
        stats.active_bodies,
        stats.sleeping_bodies,
        stats.registered_shapes,
        body_handles[0].value,
        body_handles[BODY_COUNT - UINT64_C(1)].value,
        static_handles[0].value,
        static_handles[STATIC_COUNT - UINT64_C(1)].value);

    uint64_t body_removed = 0;
    uint64_t static_removed = 0;
    ENTASIS_TEST_CHECK(entasis_body_remove_batch(&world, body_handles, BODY_COUNT, &body_removed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_remove_batch(
                           &world,
                           static_handles,
                           STATIC_COUNT,
                           ENTASIS_AWAKENING_NONE,
                           &static_removed,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, sphere, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, box, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_world_stats_t cleared = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &cleared, &diagnostic) == ENTASIS_STATUS_OK);
    (void)printf(
        "removed_bodies=%" PRIu64 " removed_statics=%" PRIu64 " active=%" PRId64
        " sleeping=%" PRId64 " statics=%" PRId64 " registered_shapes=%" PRId64 "\n",
        body_removed,
        static_removed,
        cleared.active_bodies,
        cleared.sleeping_bodies,
        cleared.statics,
        cleared.registered_shapes);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}
