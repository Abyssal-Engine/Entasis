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

static entasis_quaternion_t identity_orientation(void)
{
    return (entasis_quaternion_t){0.0f, 0.0f, 0.0f, 1.0f};
}

static entasis_rigid_pose_t pose_at(float x, float y, float z)
{
    return entasis_pose((entasis_vector3_t){x, y, z}, identity_orientation());
}

static entasis_world_description_t world_description(void)
{
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    description.profiling = ENTASIS_TRUE;
    description.threading.worker_count = UINT32_C(1);
    description.capacity.bodies = INT32_C(32);
    description.capacity.statics = INT32_C(32);
    description.capacity.shapes_per_type = INT32_C(16);
    description.capacity.constraints = INT32_C(64);
    description.capacity.broad_phase_candidates = INT32_C(256);
    description.capacity.pairs = INT32_C(256);
    description.capacity.collision_child_pairs = INT32_C(256);
    return description;
}

typedef struct query_order_t
{
    entasis_query_result_t *results;
    int calls;
    int failures;
} query_order_t;

static entasis_bool_t ENTASIS_CALL query_order_allow(void *user, entasis_collidable_reference_t collidable)
{
    query_order_t *state = (query_order_t *)user;
    ++state->calls;
    if (state->results[9].count != 1)
    {
        ++state->failures;
    }
    return entasis_collidable_mobility(collidable) == ENTASIS_BODY_MOBILITY_STATIC ? ENTASIS_TRUE : ENTASIS_FALSE;
}

#if defined(ENTASIS_TEST_QUERY_CONTEXT)
#define QUERY_TARGET &query_context
#define QUERY_RAY_CAST_ANY entasis_query_context_ray_cast_any
#define QUERY_RAY_CAST_CLOSEST entasis_query_context_ray_cast_closest
#define QUERY_RAY_CAST_ALL entasis_query_context_ray_cast_all
#define QUERY_QUERY_BATCH entasis_query_context_query_batch
#define QUERY_COLLISION_QUERY entasis_query_context_collision_query
#define DISTANCE_RESERVE entasis_query_context_distance_query_reserve
#define DISTANCE_BATCH entasis_query_context_distance_query_batch
#define OVERLAP_ANY entasis_query_context_overlap_any
#else
#define QUERY_TARGET &world
#define QUERY_RAY_CAST_ANY entasis_ray_cast_any
#define QUERY_RAY_CAST_CLOSEST entasis_ray_cast_closest
#define QUERY_RAY_CAST_ALL entasis_ray_cast_all
#define QUERY_QUERY_BATCH entasis_query_batch
#define QUERY_COLLISION_QUERY entasis_collision_query
#define DISTANCE_RESERVE entasis_distance_query_reserve
#define DISTANCE_BATCH entasis_distance_query_batch
#define OVERLAP_ANY entasis_overlap_any
#endif

int main(int argc, char **argv)
{
    if (argc > 2 || (argc == 2 && strcmp(argv[1], "--queries-only") != 0))
    {
        return 2;
    }
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_t world = {0};
    entasis_world_description_t description = world_description();
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_sphere_t sphere_value = entasis_sphere(1.0f);
    const entasis_box_t box_value = entasis_box(2.0f, 2.0f, 2.0f);
    entasis_shape_handle_t sphere_shape = {0};
    entasis_shape_handle_t box_shape = {0};
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, &sphere_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &world, ENTASIS_SHAPE_TYPE_BOX, &box_value, &box_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(
                           ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, 1.0f, &inertia) == ENTASIS_STATUS_OK);

    const entasis_body_description_t dynamic_description = entasis_body_dynamic(
        sphere_shape,
        inertia,
        pose_at(0.0f, 0.0f, 0.0f),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    entasis_body_handle_t dynamic_body = {0};
    ENTASIS_TEST_CHECK(entasis_body_add(
                           &world, &dynamic_description, &dynamic_body, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_body_description_t kinematic_description = entasis_body_kinematic(
        sphere_shape,
        pose_at(8.0f, 0.0f, 0.0f),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    entasis_body_handle_t kinematic_body = {0};
    ENTASIS_TEST_CHECK(entasis_body_add(
                           &world, &kinematic_description, &kinematic_body, &diagnostic) == ENTASIS_STATUS_OK);
    (void)kinematic_body;

    const entasis_static_description_t static_description = entasis_static_body(
        box_shape, pose_at(5.0f, 0.0f, 0.0f), entasis_ccd_discrete());
    entasis_static_handle_t static_body = {0};
    ENTASIS_TEST_CHECK(entasis_static_add(
                           &world, &static_description, ENTASIS_AWAKENING_NONE, &static_body, &diagnostic) == ENTASIS_STATUS_OK);

#if defined(ENTASIS_TEST_QUERY_CONTEXT)
    entasis_query_context_t query_context = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query_context, &world, NULL, NULL, &diagnostic) == ENTASIS_STATUS_OK);
#endif

    const entasis_ray_t ray = entasis_ray(
        (entasis_vector3_t){-10.0f, 0.0f, 0.0f},
        (entasis_vector3_t){1.0f, 0.0f, 0.0f},
        30.0f);
    entasis_bool_t any_hit = ENTASIS_FALSE;
    entasis_ray_hit_t closest = {0};
    entasis_ray_hit_t hits[8] = {0};
    uint64_t written = 0;
    uint64_t required = 0;
    ENTASIS_TEST_CHECK(QUERY_RAY_CAST_ANY(QUERY_TARGET, ray, NULL, &any_hit, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(QUERY_RAY_CAST_CLOSEST(QUERY_TARGET, ray, NULL, &closest, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(QUERY_RAY_CAST_ALL(QUERY_TARGET, ray, NULL, hits, UINT64_C(8), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    uint32_t mobility_mask = UINT32_C(0);
    for (uint64_t index = 0; index < written; ++index)
    {
        mobility_mask |= UINT32_C(1) << entasis_collidable_mobility(hits[index].collidable);
    }

    const entasis_bounding_box_t bounds = {
        .min = {4.5f, -1.0f, -1.0f},
        .reserved_min = 0.0f,
        .max = {5.5f, 1.0f, 1.0f},
        .reserved_max = 0.0f};
    entasis_query_t queries[4] = {
        entasis_query_ray_any(ray, NULL),
        entasis_query_ray_closest(ray, NULL),
        entasis_query_ray_all(ray, entasis_query_output(INT32_C(0), INT32_C(8)), NULL),
        entasis_query_volume_all(bounds, entasis_query_output(INT32_C(0), INT32_C(8)), NULL)};
    entasis_query_result_t results[4] = {0};
    entasis_ray_hit_t batch_ray_hits[8] = {0};
    entasis_volume_hit_t batch_volume_hits[8] = {0};
    entasis_query_scratch_t scratch = entasis_query_scratch(
        batch_ray_hits, UINT64_C(8),
        NULL, UINT64_C(0),
        batch_volume_hits, UINT64_C(8));
    ENTASIS_TEST_CHECK(QUERY_QUERY_BATCH(QUERY_TARGET, queries, UINT64_C(4), results, UINT64_C(4), &scratch, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_query_t packet[23];
    entasis_query_result_t packet_results[23] = {0};
    for (int i = 0; i < 23; ++i)
    {
        packet[i] = entasis_query_ray_closest(ray, NULL);
    }
    ENTASIS_TEST_CHECK(QUERY_QUERY_BATCH(QUERY_TARGET, packet, 23, packet_results, 23, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    for (int i = 0; i < 23; ++i)
    {
        entasis_ray_hit_t actual;
        (void)memcpy(&actual, packet_results[i].payload, sizeof(actual));
        ENTASIS_TEST_CHECK(actual.t == closest.t && actual.collidable.packed == closest.collidable.packed && packet_results[i].count == 1);
    }
    query_order_t order = {packet_results, 0, 0};
    entasis_query_filter_t order_filter = {0};
    order_filter.allow = query_order_allow;
    order_filter.user_context = &order;
    packet[10] = entasis_query_ray_closest(ray, &order_filter);
    packet[19] = entasis_query_ray_closest(entasis_ray((entasis_vector3_t){0}, (entasis_vector3_t){0}, 30), NULL);
    packet[22] = entasis_query_ray_all(ray, entasis_query_output(0, 8), NULL);
    (void)memset(packet_results, 0, sizeof(packet_results));
    ENTASIS_TEST_CHECK(QUERY_QUERY_BATCH(QUERY_TARGET, packet, 23, packet_results, 23, &scratch, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    for (int i = 0; i < 22; ++i)
    {
        entasis_ray_hit_t actual;
        (void)memcpy(&actual, packet_results[i].payload, sizeof(actual));
        if (i == 10)
        {
            ENTASIS_TEST_CHECK(actual.t == 14);
        }
        else if (i == 19)
        {
            ENTASIS_TEST_CHECK(packet_results[i].count == 0);
        }
        else
        {
            ENTASIS_TEST_CHECK(actual.t == closest.t && actual.collidable.packed == closest.collidable.packed);
        }
    }
    ENTASIS_TEST_CHECK(order.calls == 3 && order.failures == 0 && packet_results[22].count == 3);
    (void)printf("packet count=23 callbacks=%d invalid=%d tail=%" PRId32 "\n", order.calls, (int)packet_results[19].status, packet_results[21].count);

    entasis_contact_manifold_t manifold = {0};
    const entasis_status_t collision_status = QUERY_COLLISION_QUERY(QUERY_TARGET,
                                                                    sphere_shape,
                                                                    pose_at(0.0f, 0.0f, 0.0f),
                                                                    sphere_shape,
                                                                    pose_at(1.5f, 0.0f, 0.0f),
                                                                    0.0f,
                                                                    &manifold,
                                                                    &diagnostic);
    ENTASIS_TEST_CHECK(collision_status == ENTASIS_STATUS_OK);

    (void)printf(
        "query any=%d closest=%" PRIu32 " closest_mobility=%u all=%" PRIu64 " mask=%" PRIu32
        " batch=%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 " collision=%d\n",
        any_hit == ENTASIS_TRUE ? 1 : 0,
        float_bits(closest.t),
        (unsigned)entasis_collidable_mobility(closest.collidable),
        written,
        mobility_mask,
        results[0].count,
        results[1].count,
        results[2].count,
        results[3].count,
        collision_status == ENTASIS_STATUS_OK ? 1 : 0);

    ENTASIS_TEST_CHECK(DISTANCE_RESERVE(QUERY_TARGET, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_distance_query_t distance_queries[5] = {0};
    entasis_distance_query_result_t distance_results[5] = {0};
    for (unsigned i = 0; i < 5u; ++i)
    {
        distance_queries[i].kind = (entasis_distance_query_kind_t)(i < 4u ? i : 255u);
        distance_queries[i].shape_a = sphere_shape;
        distance_queries[i].shape_b = sphere_shape;
        distance_queries[i].pose_a = pose_at(0, 0, 0);
        distance_queries[i].pose_b = pose_at(i == 1u ? 4.0f : 1.0f, 0, 0);
        distance_queries[i].point = (entasis_vector3_t){4, 0, 0};
        distance_queries[i].settings = entasis_distance_query_settings_default();
    }
    ENTASIS_TEST_CHECK(DISTANCE_BATCH(QUERY_TARGET, distance_queries, 5, distance_results, 5, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    for (unsigned i = 0; i < 5u; ++i)
    {
        const entasis_distance_query_result_t *value = &distance_results[i];
        (void)printf("distance item=%u status=%u state=%u distance=%u depth=%u pa=%u,%u,%u pb=%u,%u,%u normal=%u,%u,%u children=%d,%d correction=%u,%u,%u,%u\n",
                     i, (unsigned)value->status, (unsigned)value->shape.geometry.state,
                     float_bits(value->shape.geometry.distance), float_bits(value->shape.geometry.depth),
                     float_bits(value->shape.geometry.point_a.x), float_bits(value->shape.geometry.point_a.y), float_bits(value->shape.geometry.point_a.z),
                     float_bits(value->shape.geometry.point_b.x), float_bits(value->shape.geometry.point_b.y), float_bits(value->shape.geometry.point_b.z),
                     float_bits(value->shape.geometry.normal.x), float_bits(value->shape.geometry.normal.y), float_bits(value->shape.geometry.normal.z),
                     value->shape.child_a, value->shape.child_b, (unsigned)value->correction.state,
                     float_bits(value->correction.translation.x), float_bits(value->correction.translation.y), float_bits(value->correction.translation.z));
    }
    entasis_overlap_state_t geometric_any = ENTASIS_OVERLAP_SEPARATED;
    ENTASIS_TEST_CHECK(OVERLAP_ANY(QUERY_TARGET, sphere_shape, pose_at(0, 0, 0), NULL, &geometric_any, &diagnostic) == ENTASIS_STATUS_OK);
    (void)printf("geometric_any state=%u\n", (unsigned)geometric_any);

    if (argc == 2)
    {
#if defined(ENTASIS_TEST_QUERY_CONTEXT)
        ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query_context, &diagnostic) == ENTASIS_STATUS_OK);
#endif
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
        return 0;
    }

    entasis_active_body_view_t body_view = {0};
    entasis_static_view_t static_view = {0};
    entasis_active_body_row_t body_row = {0};
    entasis_static_row_t static_row = {0};
    ENTASIS_TEST_CHECK(entasis_active_body_view(&world, &body_view, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_view(&world, &static_view, &diagnostic) == ENTASIS_STATUS_OK);
    (void)printf(
        "views bodies=%" PRIu64 " statics=%" PRIu64 " body_valid=%d static_valid=%d rows=%d,%d\n",
        body_view.count,
        static_view.count,
        entasis_body_view_valid(&world, &body_view) == ENTASIS_TRUE ? 1 : 0,
        entasis_static_view_valid(&world, &static_view) == ENTASIS_TRUE ? 1 : 0,
        entasis_active_body_row(&body_view, UINT64_C(0), &body_row) == ENTASIS_TRUE ? 1 : 0,
        entasis_static_view_row(&static_view, UINT64_C(0), &static_row) == ENTASIS_TRUE ? 1 : 0);

    entasis_body_property_table_t properties = {0};
    ENTASIS_TEST_CHECK(entasis_body_property_init(
                           &properties, (uint64_t)sizeof(uint64_t), (uint64_t)_Alignof(uint64_t), UINT64_C(8), NULL, &diagnostic) == ENTASIS_STATUS_OK);
    const uint64_t property_input = UINT64_C(777);
    const void *property_pointer = NULL;
    entasis_body_property_key_t property_key = {0};
    ENTASIS_TEST_CHECK(entasis_body_property_set(
                           &properties, dynamic_body, &property_input, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_property_get(
                           &properties, dynamic_body, &property_pointer, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_property_key(
                           &properties, dynamic_body, &property_key, &diagnostic) == ENTASIS_STATUS_OK);
    const uint64_t property_value = *(const uint64_t *)property_pointer;
    ENTASIS_TEST_CHECK(entasis_body_property_remove(
                           &properties, dynamic_body, &diagnostic) == ENTASIS_STATUS_OK);
    property_pointer = NULL;
    const entasis_status_t stale_status = entasis_body_property_get_key(
        &properties, property_key, &property_pointer, &diagnostic);
    (void)printf(
        "property value=%" PRIu64 " stale=%d capacity=%d\n",
        property_value,
        stale_status == ENTASIS_STATUS_NOT_FOUND ? 1 : 0,
        entasis_body_property_capacity(&properties) >= UINT64_C(8) ? 1 : 0);
    ENTASIS_TEST_CHECK(entasis_body_property_destroy(&properties, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_contact_user_table_t users = {0};
    entasis_contact_tracker_t tracker = {0};
    ENTASIS_TEST_CHECK(entasis_contact_user_table_init(
                           &users, UINT64_C(8), UINT64_C(8), NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_set_body(
                           &users, dynamic_body, UINT64_C(111), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_set_static(
                           &users, static_body, UINT64_C(222), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_init(
                           &tracker, UINT64_C(8), NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_bind(
                           &tracker, &world, &users, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_body_velocity_t zero_velocity = entasis_velocity(
        (entasis_vector3_t){0.0f, 0.0f, 0.0f},
        (entasis_vector3_t){0.0f, 0.0f, 0.0f});
    const entasis_rigid_pose_t contact_pose = pose_at(3.5f, 0.0f, 0.0f);
    entasis_contact_event_t events[4] = {0};
    uint64_t begin_count = 0;
    uint64_t persist_count = 0;
    uint64_t end_count = 0;
    uint64_t event_required = 0;

    ENTASIS_TEST_CHECK(entasis_body_set_pose(&world, dynamic_body, &contact_pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(&world, dynamic_body, &zero_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_events_drain(
                           &tracker, events, UINT64_C(4), &begin_count, &event_required, &diagnostic) == ENTASIS_STATUS_OK);
    const unsigned begin_kind = events[0].kind;
    const int begin_users =
        (events[0].user_a == UINT64_C(111) || events[0].user_b == UINT64_C(111)) &&
        (events[0].user_a == UINT64_C(222) || events[0].user_b == UINT64_C(222));

    ENTASIS_TEST_CHECK(entasis_body_set_pose(&world, dynamic_body, &contact_pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(&world, dynamic_body, &zero_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_events_drain(
                           &tracker, events, UINT64_C(4), &persist_count, &event_required, &diagnostic) == ENTASIS_STATUS_OK);
    const unsigned persist_kind = events[0].kind;

    const entasis_rigid_pose_t separated_pose = pose_at(-10.0f, 0.0f, 0.0f);
    ENTASIS_TEST_CHECK(entasis_body_set_pose(&world, dynamic_body, &separated_pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(&world, dynamic_body, &zero_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_events_drain(
                           &tracker, events, UINT64_C(4), &end_count, &event_required, &diagnostic) == ENTASIS_STATUS_OK);
    const unsigned end_kind = events[0].kind;
    (void)printf(
        "events counts=%" PRIu64 ",%" PRIu64 ",%" PRIu64 " kinds=%u,%u,%u users=%d\n",
        begin_count,
        persist_count,
        end_count,
        begin_kind,
        persist_kind,
        end_kind,
        begin_users ? 1 : 0);

    entasis_bool_t profile_enabled = ENTASIS_FALSE;
    entasis_profile_snapshot_t profile = {0};
    entasis_world_stats_t stats = {0};
    entasis_world_solver_stats_t solver_stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_profile_enabled(
                           &world, &profile_enabled, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_profile_snapshot(
                           &world, &profile, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_solver_stats(
                           &world, &solver_stats, &diagnostic) == ENTASIS_STATUS_OK);
    (void)printf(
        "profile enabled=%d step=%" PRIu64 " trace=%d active=%" PRId64 " statics=%" PRId64
        " batches_nonnegative=%d\n",
        profile_enabled == ENTASIS_TRUE ? 1 : 0,
        profile.step_index,
        profile.trace_count > UINT64_C(0) ? 1 : 0,
        stats.active_bodies,
        stats.statics,
        solver_stats.active_batches >= INT64_C(0) ? 1 : 0);

    ENTASIS_TEST_CHECK(entasis_contact_tracker_unbind(&tracker, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_destroy(&tracker, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_table_destroy(&users, &diagnostic) == ENTASIS_STATUS_OK);
#if defined(ENTASIS_TEST_QUERY_CONTEXT)
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query_context, &diagnostic) == ENTASIS_STATUS_OK);
#endif
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}
