#include "entasis.h"
#include "test_support.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
typedef DWORD query_thread_id_t;
#define QUERY_CURRENT_THREAD_ID() GetCurrentThreadId()
#define QUERY_THREAD_IDS_EQUAL(left, right) ((left) == (right))
#else
#include <pthread.h>
typedef pthread_t query_thread_id_t;
#define QUERY_CURRENT_THREAD_ID() pthread_self()
#define QUERY_THREAD_IDS_EQUAL(left, right) pthread_equal((left), (right))
#endif

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

static entasis_world_description_t query_world_description(void)
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

typedef struct query_scene_t
{
    entasis_world_t world;
    entasis_shape_handle_t sphere;
    entasis_shape_handle_t box;
    entasis_body_handle_t dynamic_body;
    entasis_body_handle_t kinematic_body;
    entasis_static_handle_t static_body;
} query_scene_t;

static int query_scene_init(query_scene_t *scene)
{
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = query_world_description();
    const entasis_sphere_t sphere_value = entasis_sphere(1.0f);
    const entasis_box_t box_value = entasis_box(2.0f, 2.0f, 2.0f);
    entasis_body_inertia_t inertia = {0};

    (void)memset(scene, 0, sizeof(*scene));
    ENTASIS_TEST_CHECK(entasis_world_init(&scene->world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &scene->world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, &scene->sphere, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(
                           &scene->world, ENTASIS_SHAPE_TYPE_BOX, &box_value, &scene->box, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(
                           ENTASIS_SHAPE_TYPE_SPHERE, &sphere_value, 1.0f, &inertia) == ENTASIS_STATUS_OK);

    const entasis_body_description_t dynamic_description = entasis_body_dynamic(
        scene->sphere,
        inertia,
        pose_at(0.0f, 0.0f, 0.0f),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    ENTASIS_TEST_CHECK(entasis_body_add(
                           &scene->world, &dynamic_description, &scene->dynamic_body, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_body_description_t kinematic_description = entasis_body_kinematic(
        scene->sphere,
        pose_at(8.0f, 0.0f, 0.0f),
        entasis_velocity((entasis_vector3_t){0.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f}),
        entasis_body_activity(-1.0f, UINT8_C(255)));
    ENTASIS_TEST_CHECK(entasis_body_add(
                           &scene->world, &kinematic_description, &scene->kinematic_body, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_static_description_t static_description = entasis_static_body(
        scene->box, pose_at(5.0f, 0.0f, 0.0f), entasis_ccd_discrete());
    ENTASIS_TEST_CHECK(entasis_static_add(
                           &scene->world, &static_description, ENTASIS_AWAKENING_NONE, &scene->static_body, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static void query_scene_destroy(query_scene_t *scene)
{
    entasis_diagnostic_t diagnostic = {0};
    (void)entasis_world_destroy(&scene->world, &diagnostic);
}

static entasis_bool_t ENTASIS_CALL allow_static_only(
    void *user_context,
    entasis_collidable_reference_t collidable)
{
    uint64_t *call_count = (uint64_t *)user_context;
    if (call_count != NULL)
        *call_count += UINT64_C(1);
    return entasis_collidable_mobility(collidable) == ENTASIS_BODY_MOBILITY_STATIC
               ? ENTASIS_TRUE
               : ENTASIS_FALSE;
}

static entasis_bool_t ENTASIS_CALL reject_all_children(
    void *user_context,
    entasis_collidable_reference_t collidable,
    int32_t child_index)
{
    uint64_t *call_count = (uint64_t *)user_context;
    (void)collidable;
    (void)child_index;
    if (call_count != NULL)
        *call_count += UINT64_C(1);
    return ENTASIS_FALSE;
}

typedef struct query_callback_context_t
{
    entasis_world_t *world;
    query_thread_id_t caller_thread;
    uint64_t call_count;
    entasis_bool_t wrong_thread;
    entasis_status_t reentry_status;
} query_callback_context_t;

static entasis_bool_t ENTASIS_CALL verify_query_callback_contract(
    void *user_context,
    entasis_collidable_reference_t collidable)
{
    query_callback_context_t *context = (query_callback_context_t *)user_context;
    entasis_world_stats_t stats = {0};
    entasis_diagnostic_t diagnostic = {0};
    (void)collidable;
    if (context == NULL)
        return ENTASIS_FALSE;
    context->call_count += UINT64_C(1);
    if (!QUERY_THREAD_IDS_EQUAL(context->caller_thread, QUERY_CURRENT_THREAD_ID()))
    {
        context->wrong_thread = ENTASIS_TRUE;
    }
    context->reentry_status = entasis_world_stats(context->world, &stats, &diagnostic);
    return ENTASIS_TRUE;
}

static int test_scalar_queries_and_filters(void)
{
    query_scene_t scene;
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(query_scene_init(&scene) == 0);

    const entasis_ray_t ray = entasis_ray(
        (entasis_vector3_t){-10.0f, 0.0f, 0.0f},
        (entasis_vector3_t){1.0f, 0.0f, 0.0f},
        30.0f);
    entasis_bool_t any_hit = ENTASIS_FALSE;
    ENTASIS_TEST_CHECK(entasis_ray_cast_any(
                           &scene.world, ray, NULL, &any_hit, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(any_hit == ENTASIS_TRUE);

    entasis_ray_hit_t closest = {0};
    ENTASIS_TEST_CHECK(entasis_ray_cast_closest(
                           &scene.world, ray, NULL, &closest, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(nearf(closest.t, 9.0f, 1.0e-5f));
    ENTASIS_TEST_CHECK(entasis_collidable_mobility(closest.collidable) == ENTASIS_BODY_MOBILITY_DYNAMIC);
    entasis_body_handle_t closest_body = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_collidable_body_handle(
                           closest.collidable, &closest_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(closest_body.value == scene.dynamic_body.value);

    entasis_ray_hit_t hits[8] = {0};
    uint64_t written = 0;
    uint64_t required = 0;
    ENTASIS_TEST_CHECK(entasis_ray_cast_all(
                           &scene.world, ray, NULL, hits, UINT64_C(8), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == UINT64_C(3));
    ENTASIS_TEST_CHECK(required == UINT64_C(3));

    uint32_t mobility_mask = UINT32_C(0);
    entasis_collidable_reference_t static_collidable = {0};
    for (uint64_t index = 0; index < written; ++index)
    {
        const entasis_body_mobility_t mobility = entasis_collidable_mobility(hits[index].collidable);
        mobility_mask |= UINT32_C(1) << mobility;
        if (mobility == ENTASIS_BODY_MOBILITY_STATIC)
            static_collidable = hits[index].collidable;
    }
    ENTASIS_TEST_CHECK(mobility_mask == UINT32_C(7));
    entasis_static_handle_t queried_static = entasis_static_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_collidable_static_handle(
                           static_collidable, &queried_static, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(queried_static.value == scene.static_body.value);

    written = UINT64_C(99);
    required = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_ray_cast_all(
                           &scene.world, ray, NULL, hits, UINT64_C(1), &written, &required, &diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(written == UINT64_C(1));
    ENTASIS_TEST_CHECK(required >= UINT64_C(2));

    entasis_query_filter_t filter = entasis_query_filter_mobility(ENTASIS_COLLIDABLE_STATIC);
    ENTASIS_TEST_CHECK(entasis_ray_cast_closest(
                           &scene.world, ray, &filter, &closest, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collidable_mobility(closest.collidable) == ENTASIS_BODY_MOBILITY_STATIC);

    uint64_t callback_count = 0;
    filter = entasis_query_filter_all();
    filter.allow = allow_static_only;
    filter.user_context = &callback_count;
    written = 0;
    required = 0;
    ENTASIS_TEST_CHECK(entasis_ray_cast_all(
                           &scene.world, ray, &filter, hits, UINT64_C(8), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == UINT64_C(1));
    ENTASIS_TEST_CHECK(callback_count >= UINT64_C(3));
    ENTASIS_TEST_CHECK(entasis_collidable_mobility(hits[0].collidable) == ENTASIS_BODY_MOBILITY_STATIC);

    callback_count = 0;
    filter = entasis_query_filter_all();
    filter.allow_child = reject_all_children;
    filter.user_context = &callback_count;
    written = 0;
    required = 0;
    ENTASIS_TEST_CHECK(entasis_ray_cast_all(
                           &scene.world, ray, &filter, hits, UINT64_C(8), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == UINT64_C(0));
    ENTASIS_TEST_CHECK(callback_count >= UINT64_C(3));

    query_callback_context_t callback_contract = {
        &scene.world,
        QUERY_CURRENT_THREAD_ID(),
        UINT64_C(0),
        ENTASIS_FALSE,
        ENTASIS_STATUS_OK};
    filter = entasis_query_filter_all();
    filter.allow = verify_query_callback_contract;
    filter.user_context = &callback_contract;
    written = 0;
    required = 0;
    ENTASIS_TEST_CHECK(entasis_ray_cast_all(
                           &scene.world, ray, &filter, hits, UINT64_C(8), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == UINT64_C(3));
    ENTASIS_TEST_CHECK(callback_contract.call_count >= UINT64_C(3));
    ENTASIS_TEST_CHECK(callback_contract.wrong_thread == ENTASIS_FALSE);
    ENTASIS_TEST_CHECK(callback_contract.reentry_status == ENTASIS_STATUS_INVALID_ARGUMENT);

    const entasis_rigid_pose_t sweep_pose = pose_at(-5.0f, 0.0f, 0.0f);
    const entasis_body_velocity_t sweep_velocity = entasis_velocity(
        (entasis_vector3_t){1.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f});
    entasis_bool_t sweep_any_hit = ENTASIS_FALSE;
    ENTASIS_TEST_CHECK(entasis_sweep_any(
                           &scene.world, scene.sphere, sweep_pose, sweep_velocity, 20.0f,
                           NULL, NULL, &sweep_any_hit, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(sweep_any_hit == ENTASIS_TRUE);

    entasis_sweep_hit_t sweep = {0};
    ENTASIS_TEST_CHECK(entasis_sweep_closest(
                           &scene.world,
                           scene.sphere,
                           sweep_pose,
                           sweep_velocity,
                           20.0f,
                           NULL,
                           NULL,
                           &sweep,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(sweep.sweep.state == ENTASIS_SWEEP_HIT);
    ENTASIS_TEST_CHECK(sweep.sweep.t1 >= 0.0f && sweep.sweep.t1 <= 5.0f);

    entasis_sweep_hit_t sweep_hits[8] = {0};
    written = UINT64_C(0);
    required = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_sweep_all(
                           &scene.world, scene.sphere, sweep_pose, sweep_velocity, 20.0f,
                           NULL, NULL, sweep_hits, UINT64_C(8), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written >= UINT64_C(1));
    ENTASIS_TEST_CHECK(required == written);

    entasis_overlap_hit_t overlaps[8] = {0};
    written = 0;
    required = 0;
    ENTASIS_TEST_CHECK(entasis_overlap_all(
                           &scene.world, scene.sphere, pose_at(0.5f, 0.0f, 0.0f), NULL,
                           overlaps, UINT64_C(8), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written >= UINT64_C(1));
    ENTASIS_TEST_CHECK(required == written);

    entasis_volume_hit_t volumes[8] = {0};
    const entasis_bounding_box_t bounds = {
        {-2.0f, -2.0f, -2.0f}, 0.0f, {2.0f, 2.0f, 2.0f}, 0.0f};
    written = 0;
    required = 0;
    ENTASIS_TEST_CHECK(entasis_volume_all(
                           &scene.world, bounds, NULL, volumes, UINT64_C(8), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == UINT64_C(1));
    ENTASIS_TEST_CHECK(entasis_collidable_mobility(volumes[0].collidable) == ENTASIS_BODY_MOBILITY_DYNAMIC);

    query_scene_destroy(&scene);
    return 0;
}

static int test_query_batches_and_direct_collision(void)
{
    query_scene_t scene;
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(query_scene_init(&scene) == 0);

    const entasis_ray_t ray = entasis_ray(
        (entasis_vector3_t){-10.0f, 0.0f, 0.0f},
        (entasis_vector3_t){1.0f, 0.0f, 0.0f},
        30.0f);
    const entasis_bounding_box_t bounds = {
        {-2.0f, -2.0f, -2.0f}, 0.0f, {2.0f, 2.0f, 2.0f}, 0.0f};
    const entasis_body_velocity_t sweep_velocity = entasis_velocity(
        (entasis_vector3_t){1.0f, 0.0f, 0.0f}, (entasis_vector3_t){0.0f, 0.0f, 0.0f});
    entasis_query_t queries[6] = {
        entasis_query_ray_any(ray, NULL),
        entasis_query_ray_closest(ray, NULL),
        entasis_query_ray_all(ray, entasis_query_output(0, 8), NULL),
        entasis_query_volume_all(bounds, entasis_query_output(0, 8), NULL),
        entasis_query_sweep_closest(
            scene.sphere, pose_at(-5.0f, 0.0f, 0.0f), sweep_velocity, 20.0f, NULL, NULL),
        entasis_query_overlap_all(
            scene.sphere, pose_at(0.5f, 0.0f, 0.0f), entasis_query_output(0, 8), NULL)};
    entasis_query_result_t results[6] = {0};
    entasis_ray_hit_t ray_hits[8] = {0};
    entasis_overlap_hit_t overlap_hits[8] = {0};
    entasis_volume_hit_t volume_hits[8] = {0};
    entasis_query_scratch_t scratch = entasis_query_scratch(
        ray_hits, UINT64_C(8), overlap_hits, UINT64_C(8), volume_hits, UINT64_C(8));
    ENTASIS_TEST_CHECK(entasis_query_batch(
                           &scene.world, queries, UINT64_C(6), results, UINT64_C(6), &scratch, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[0].status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[0].hit == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(results[1].status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[1].hit == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(results[2].status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[2].count == INT32_C(3));
    ENTASIS_TEST_CHECK(results[3].status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[3].count == INT32_C(1));
    ENTASIS_TEST_CHECK(results[4].status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[4].hit == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(results[5].status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[5].count >= INT32_C(1));

    entasis_ray_hit_t batch_closest = {0};
    (void)memcpy(&batch_closest, results[1].payload, sizeof(batch_closest));
    ENTASIS_TEST_CHECK(nearf(batch_closest.t, 9.0f, 1.0e-5f));

    queries[2] = entasis_query_ray_all(ray, entasis_query_output(7, 2), NULL);
    ENTASIS_TEST_CHECK(entasis_query_batch(
                           &scene.world, queries, UINT64_C(6), results, UINT64_C(6), &scratch, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(results[2].status == ENTASIS_STATUS_INVALID_ARGUMENT);

    entasis_contact_manifold_t manifold = {0};
    ENTASIS_TEST_CHECK(entasis_collision_query(
                           &scene.world,
                           scene.sphere,
                           pose_at(0.0f, 0.0f, 0.0f),
                           scene.sphere,
                           pose_at(1.5f, 0.0f, 0.0f),
                           0.0f,
                           &manifold,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(
        manifold.convex.count > 0 || manifold.nonconvex.count > 0);

    const entasis_collision_query_t collision_queries[2] = {
        {scene.sphere, scene.sphere, pose_at(0.0f, 0.0f, 0.0f), pose_at(1.5f, 0.0f, 0.0f), 0.0f},
        {scene.sphere, scene.sphere, pose_at(0.0f, 0.0f, 0.0f), pose_at(10.0f, 0.0f, 0.0f), 0.0f}};
    entasis_collision_query_result_t collision_results[2] = {0};
    ENTASIS_TEST_CHECK(entasis_collision_query_batch(
                           &scene.world, collision_queries, UINT64_C(2), collision_results, UINT64_C(2), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(collision_results[0].status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(collision_results[0].hit == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(collision_results[1].status == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(collision_results[1].hit == ENTASIS_FALSE);

    query_scene_destroy(&scene);
    return 0;
}

static int test_views_profiling_and_contact_events(void)
{
    query_scene_t scene;
    entasis_diagnostic_t diagnostic = {0};
    entasis_contact_user_table_t users = {0};
    entasis_contact_tracker_t tracker = {0};
    ENTASIS_TEST_CHECK(query_scene_init(&scene) == 0);

    entasis_active_body_view_t body_view = {0};
    entasis_static_view_t static_view = {0};
    ENTASIS_TEST_CHECK(entasis_active_body_view(
                           &scene.world, &body_view, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(body_view.count == UINT64_C(2));
    ENTASIS_TEST_CHECK(entasis_body_view_valid(&scene.world, &body_view) == ENTASIS_TRUE);
    entasis_active_body_row_t body_row = {0};
    ENTASIS_TEST_CHECK(entasis_active_body_row(&body_view, UINT64_C(0), &body_row) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(body_row.dynamics != NULL);
    ENTASIS_TEST_CHECK(body_row.collidable != NULL);
    ENTASIS_TEST_CHECK(body_row.activity != NULL);

    ENTASIS_TEST_CHECK(entasis_static_view(
                           &scene.world, &static_view, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(static_view.count == UINT64_C(1));
    ENTASIS_TEST_CHECK(entasis_static_view_valid(&scene.world, &static_view) == ENTASIS_TRUE);
    entasis_static_row_t static_row = {0};
    ENTASIS_TEST_CHECK(entasis_static_view_row(&static_view, UINT64_C(0), &static_row) == ENTASIS_TRUE);
    ENTASIS_TEST_CHECK(static_row.record != NULL);

    const entasis_rigid_pose_t moved = pose_at(0.25f, 0.0f, 0.0f);
    ENTASIS_TEST_CHECK(entasis_body_set_pose(
                           &scene.world, scene.dynamic_body, &moved, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_view_valid(&scene.world, &body_view) == ENTASIS_FALSE);
    ENTASIS_TEST_CHECK(entasis_static_view_valid(&scene.world, &static_view) == ENTASIS_FALSE);

    entasis_bool_t profile_enabled = ENTASIS_FALSE;
    ENTASIS_TEST_CHECK(entasis_world_profile_enabled(
                           &scene.world, &profile_enabled, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(profile_enabled == ENTASIS_TRUE);
    const entasis_string_view_t stage_name = entasis_profile_stage_text(ENTASIS_PROFILE_TIMESTEP);
    ENTASIS_TEST_CHECK(stage_name.data != NULL && stage_name.count > UINT64_C(0));
    const entasis_string_view_t invalid_stage = entasis_profile_stage_text(UINT8_C(255));
    ENTASIS_TEST_CHECK(invalid_stage.data == NULL && invalid_stage.count == UINT64_C(0));

    entasis_world_solver_stats_t solver_stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_solver_stats(
                           &scene.world, &solver_stats, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(solver_stats.active_batches >= INT64_C(0));

    ENTASIS_TEST_CHECK(entasis_contact_user_table_init(
                           &users, UINT64_C(8), UINT64_C(8), NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_table_ensure_capacity(
                           &users, UINT64_C(16), UINT64_C(16), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_set_body(
                           &users, scene.dynamic_body, UINT64_C(111), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_set_static(
                           &users, scene.static_body, UINT64_C(222), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_remove_body(
                           &users, scene.dynamic_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_remove_static(
                           &users, scene.static_body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_table_clear(&users, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_set_body(
                           &users, scene.dynamic_body, UINT64_C(111), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_set_static(
                           &users, scene.static_body, UINT64_C(222), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_init(
                           &tracker, UINT64_C(8), NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_ensure_capacity(
                           &tracker, UINT64_C(16), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_capacity(&tracker) >= UINT64_C(16));
    ENTASIS_TEST_CHECK(entasis_contact_tracker_bind(
                           &tracker, &scene.world, &users, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&scene.world, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);

    const entasis_rigid_pose_t contact_pose = pose_at(3.5f, 0.0f, 0.0f);
    const entasis_body_velocity_t zero_velocity = entasis_velocity(
        (entasis_vector3_t){0.0f, 0.0f, 0.0f},
        (entasis_vector3_t){0.0f, 0.0f, 0.0f});
    ENTASIS_TEST_CHECK(entasis_body_set_pose(
                           &scene.world, scene.dynamic_body, &contact_pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(
                           &scene.world, scene.dynamic_body, &zero_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&scene.world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_contact_event_t events[4] = {0};
    uint64_t written = UINT64_C(99);
    uint64_t required = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_contact_events_drain(
                           &tracker, NULL, UINT64_C(0), &written, &required, &diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(written == UINT64_C(0));
    ENTASIS_TEST_CHECK(required == UINT64_C(1));

    written = 0;
    required = 0;
    ENTASIS_TEST_CHECK(entasis_contact_events_drain(
                           &tracker, events, UINT64_C(4), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == UINT64_C(1));
    ENTASIS_TEST_CHECK(required == UINT64_C(1));
    ENTASIS_TEST_CHECK(events[0].kind == ENTASIS_CONTACT_EVENT_BEGIN);
    ENTASIS_TEST_CHECK(events[0].user_a == UINT64_C(111) || events[0].user_b == UINT64_C(111));
    ENTASIS_TEST_CHECK(events[0].user_a == UINT64_C(222) || events[0].user_b == UINT64_C(222));

    ENTASIS_TEST_CHECK(entasis_body_set_pose(
                           &scene.world, scene.dynamic_body, &contact_pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(
                           &scene.world, scene.dynamic_body, &zero_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&scene.world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    written = 0;
    required = 0;
    ENTASIS_TEST_CHECK(entasis_contact_events_drain(
                           &tracker, events, UINT64_C(4), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == UINT64_C(1));
    ENTASIS_TEST_CHECK(events[0].kind == ENTASIS_CONTACT_EVENT_PERSIST);

    const entasis_rigid_pose_t separated = pose_at(-10.0f, 0.0f, 0.0f);
    ENTASIS_TEST_CHECK(entasis_body_set_pose(
                           &scene.world, scene.dynamic_body, &separated, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(
                           &scene.world, scene.dynamic_body, &zero_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&scene.world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    written = 0;
    required = 0;
    ENTASIS_TEST_CHECK(entasis_contact_events_drain(
                           &tracker, events, UINT64_C(4), &written, &required, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written == UINT64_C(1));
    ENTASIS_TEST_CHECK(events[0].kind == ENTASIS_CONTACT_EVENT_END);

    entasis_profile_snapshot_t profile = {0};
    ENTASIS_TEST_CHECK(entasis_world_profile_snapshot(
                           &scene.world, &profile, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(profile.step_index == UINT64_C(3));
    ENTASIS_TEST_CHECK(profile.trace_count > UINT64_C(0));

    ENTASIS_TEST_CHECK(entasis_body_set_pose(
                           &scene.world, scene.dynamic_body, &contact_pose, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_set_velocity(
                           &scene.world, scene.dynamic_body, &zero_velocity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&scene.world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    uint64_t discarded = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_contact_events_discard(
                           &tracker, &discarded, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(discarded == UINT64_C(1));
    written = UINT64_C(99);
    required = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_contact_events_drain(
                           &tracker, events, UINT64_C(4), &written, &required, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(written == UINT64_C(0) && required == UINT64_C(0));
    ENTASIS_TEST_CHECK(entasis_contact_tracker_clear(&tracker, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_contact_tracker_unbind(&tracker, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_destroy(&tracker, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_user_table_destroy(&users, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct context_reentry_t
{
    entasis_world_t *world;
    entasis_query_context_t *context;
    entasis_ray_t ray;
    unsigned calls;
    unsigned failures;
} context_reentry_t;

static entasis_bool_t ENTASIS_CALL context_reentry_filter(void *raw, entasis_collidable_reference_t collidable)
{
    context_reentry_t *state = (context_reentry_t *)raw;
    entasis_diagnostic_t diagnostic = {0};
    entasis_ray_hit_t hit = {0};
    (void)collidable;
    state->calls += 1u;
    if (entasis_world_clear(state->world, &diagnostic) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->failures += 1u;
    if (entasis_world_step(state->world, 1.0f / 60.0f, &diagnostic) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->failures += 1u;
    if (entasis_query_context_destroy(state->context, &diagnostic) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->failures += 1u;
    if (entasis_query_context_ray_cast_closest(state->context, state->ray, NULL, &hit, &diagnostic) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->failures += 1u;
    return ENTASIS_TRUE;
}

static int test_query_context_access_lifetime_and_preflight(void)
{
    query_scene_t scene;
    ENTASIS_TEST_CHECK(query_scene_init(&scene) == 0);
    entasis_diagnostic_t diagnostic = {0};
    entasis_query_context_t context = {0};
    entasis_query_context_t duplicate = {0};
    entasis_buffer_pool_t pool = {0};
    entasis_query_context_description_t description = entasis_query_context_description_default();
    ENTASIS_TEST_CHECK(entasis_buffer_pool_init(&pool, 4096, 4, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_init(&context, &scene.world, &description, &pool, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_init(&duplicate, &scene.world, &description, &pool, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_clear(&pool, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_destroy(&pool, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    entasis_world_t rejected_world = {0};
    entasis_world_description_t world_description = query_world_description();
    ENTASIS_TEST_CHECK(entasis_world_init_with_pool(&rejected_world, &world_description, &pool, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(rejected_world.opaque == NULL);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&scene.world, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    description.pair_capacity = 0;
    ENTASIS_TEST_CHECK(entasis_query_context_reserve(&context, &description, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    description = entasis_query_context_description_default();
    description.child_capacity *= 2;
    ENTASIS_TEST_CHECK(entasis_query_context_reserve(&context, &description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_ray_t ray = entasis_ray((entasis_vector3_t){-10.0f, 0.0f, 0.0f}, (entasis_vector3_t){1.0f, 0.0f, 0.0f}, 30.0f);
    entasis_ray_hit_t native_hit = {0}, context_hit = {0};
    ENTASIS_TEST_CHECK(entasis_ray_cast_closest(&scene.world, ray, NULL, &native_hit, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&context, ray, NULL, &context_hit, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(context_hit.collidable.packed == native_hit.collidable.packed && context_hit.t == native_hit.t);
    context_reentry_t reentry = {&scene.world, &context, ray, 0u, 0u};
    entasis_query_filter_t filter = entasis_query_filter_all();
    filter.allow = context_reentry_filter;
    filter.user_context = &reentry;
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&context, ray, &filter, &context_hit, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(reentry.calls != 0u && reentry.failures == 0u);

    entasis_contact_tracker_t tracker = {0};
    ENTASIS_TEST_CHECK(entasis_contact_tracker_init(&tracker, 8, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_bind(&tracker, &scene.world, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_step(&scene.world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_clear(&scene.world, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&scene.world, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_query_context_reserve(&context, &description, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&context, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_clear(&tracker, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_unbind(&tracker, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_destroy(&tracker, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&scene.world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_description_t body = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(&scene.world, scene.dynamic_body, &body, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_ray_cast_closest(&scene.world, ray, NULL, &context_hit, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&context, ray, &filter, &context_hit, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(reentry.failures == 0u);

    entasis_query_t queries[2] = {entasis_query_ray_closest(ray, NULL), entasis_query_ray_closest(ray, NULL)};
    entasis_query_result_t results[2] = {0};
    ENTASIS_TEST_CHECK(entasis_query_batch(&scene.world, queries, 2, results, 2, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(results[0].hit == ENTASIS_TRUE && results[1].hit == ENTASIS_TRUE);
    unsigned char before[sizeof(results)];
    (void)memset(results, 0x5a, sizeof(results));
    (void)memcpy(before, results, sizeof(results));
    queries[1] = entasis_query_ray_any(ray, NULL);
    ENTASIS_TEST_CHECK(entasis_query_batch(&scene.world, queries, 2, results, 2, NULL, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(memcmp(before, results, sizeof(results)) == 0);
    queries[1] = entasis_query_ray_closest(ray, &filter);
    ENTASIS_TEST_CHECK(entasis_query_batch(&scene.world, queries, 2, results, 2, NULL, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(memcmp(before, results, sizeof(results)) == 0);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_destroy(&tracker, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_world_clear(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&context, ray, NULL, &context_hit, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&context, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(context.opaque == NULL);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_clear(&pool, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_destroy(&pool, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct context_thread_t
{
    entasis_query_context_t context;
    entasis_world_t *world;
    entasis_shape_handle_t shape;
    unsigned failures;
} context_thread_t;

#if defined(_WIN32)
static DWORD WINAPI context_thread_main(LPVOID raw)
#else
static void *context_thread_main(void *raw)
#endif
{
    context_thread_t *call = (context_thread_t *)raw;
    const entasis_ray_t ray = entasis_ray((entasis_vector3_t){-10.0f, 0.0f, 0.0f}, (entasis_vector3_t){1.0f, 0.0f, 0.0f}, 30.0f);
    for (unsigned repeat = 0; repeat < 64u; ++repeat)
    {
        entasis_diagnostic_t diagnostic = {0};
        entasis_query_t queries[2] = {entasis_query_ray_closest(ray, NULL), entasis_query_ray_closest(ray, NULL)};
        entasis_query_result_t results[2] = {0};
        if (entasis_query_batch(call->world, queries, 2, results, 2, NULL, &diagnostic) != ENTASIS_STATUS_OK || !results[0].hit || !results[1].hit)
            call->failures += 1u;
        if (entasis_query_context_query_batch(&call->context, queries, 2, results, 2, NULL, &diagnostic) != ENTASIS_STATUS_OK || !results[0].hit || !results[1].hit)
            call->failures += 1u;
        entasis_overlap_hit_t overlaps[8] = {0};
        uint64_t written = 0, required = 0;
        if (entasis_query_context_overlap_all(&call->context, call->shape, pose_at(0.5f, 0.0f, 0.0f), NULL, overlaps, 8, &written, &required, &diagnostic) != ENTASIS_STATUS_OK || written != 1)
            call->failures += 1u;
        entasis_shape_distance_result_t distance = {0};
        entasis_overlap_state_t overlap = ENTASIS_OVERLAP_SEPARATED;
        if (entasis_query_context_shape_distance(&call->context, call->shape, pose_at(0, 0, 0), call->shape, pose_at(4, 0, 0), NULL, &distance, &diagnostic) != ENTASIS_STATUS_OK || distance.geometry.distance != 2)
            call->failures += 1u;
        if (entasis_query_context_shape_penetration(&call->context, call->shape, pose_at(0, 0, 0), call->shape, pose_at(1, 0, 0), NULL, &distance, &diagnostic) != ENTASIS_STATUS_OK || distance.geometry.depth != 1)
            call->failures += 1u;
        if (entasis_query_context_overlap_any(&call->context, call->shape, pose_at(0, 0, 0), NULL, &overlap, &diagnostic) != ENTASIS_STATUS_OK || overlap != ENTASIS_OVERLAP_INTERSECTING)
            call->failures += 1u;
        entasis_contact_manifold_t manifold = {0};
        if (entasis_query_context_collision_query(&call->context, call->shape, pose_at(0.0f, 0.0f, 0.0f), call->shape, pose_at(0.5f, 0.0f, 0.0f), 0.0f, &manifold, &diagnostic) != ENTASIS_STATUS_OK)
            call->failures += 1u;
    }
#if defined(_WIN32)
    return 0;
#else
    return NULL;
#endif
}

static int test_query_context_concurrent_readers(void)
{
    query_scene_t scene;
    ENTASIS_TEST_CHECK(query_scene_init(&scene) == 0);
    entasis_diagnostic_t diagnostic = {0};
    context_thread_t calls[4] = {0};
#if defined(_WIN32)
    HANDLE hosts[4] = {0};
#else
    pthread_t hosts[4];
#endif
    for (unsigned i = 0; i < 4u; ++i)
    {
        calls[i].world = &scene.world;
        calls[i].shape = scene.sphere;
        ENTASIS_TEST_CHECK(entasis_query_context_init(&calls[i].context, &scene.world, NULL, NULL, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_query_context_distance_query_reserve(&calls[i].context, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 4u; ++i)
    {
#if defined(_WIN32)
        hosts[i] = CreateThread(NULL, 0, context_thread_main, &calls[i], 0, NULL);
        ENTASIS_TEST_CHECK(hosts[i] != NULL);
#else
        ENTASIS_TEST_CHECK(pthread_create(&hosts[i], NULL, context_thread_main, &calls[i]) == 0);
#endif
    }
#if defined(_WIN32)
    if (WaitForMultipleObjects(4, hosts, TRUE, 30000) != WAIT_OBJECT_0)
    {
        fputs("query context reader join exceeded 30 seconds\n", stderr);
        ExitProcess(1);
    }
#endif
    for (unsigned i = 0; i < 4u; ++i)
    {
#if defined(_WIN32)
        ENTASIS_TEST_CHECK(CloseHandle(hosts[i]) != 0);
#else
        ENTASIS_TEST_CHECK(pthread_join(hosts[i], NULL) == 0);
#endif
        ENTASIS_TEST_CHECK(calls[i].failures == 0u);
    }
    ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 4u; ++i)
        ENTASIS_TEST_CHECK(entasis_query_context_destroy(&calls[i].context, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_query_context_scalar_family_parity(void)
{
    query_scene_t scene;
    ENTASIS_TEST_CHECK(query_scene_init(&scene) == 0);
    entasis_diagnostic_t d = {0};
    entasis_query_context_t context = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_init(&context, &scene.world, NULL, NULL, &d) == ENTASIS_STATUS_OK);
    const entasis_ray_t ray = entasis_ray((entasis_vector3_t){-10, 0, 0}, (entasis_vector3_t){1, 0, 0}, 30);
    const entasis_rigid_pose_t pose = pose_at(-10, 0, 0), overlapping = pose_at(0, 0, 0);
    const entasis_body_velocity_t velocity = entasis_velocity((entasis_vector3_t){1, 0, 0}, (entasis_vector3_t){0, 0, 0});
    const entasis_bounding_box_t bounds = {{-20, -2, -2}, 0, {20, 2, 2}, 0};
    const entasis_query_filter_t filter = entasis_query_filter_mobility(ENTASIS_COLLIDABLE_STATIC);
    const uint64_t capacities[3] = {0, 1, 8};
    for (unsigned filtered = 0; filtered < 2u; ++filtered)
    {
        const entasis_query_filter_t *selected = filtered ? &filter : NULL;
        for (unsigned c = 0; c < 3u; ++c)
        {
            const uint64_t capacity = capacities[c];
            {
                entasis_ray_hit_t left[8] = {0}, right[8] = {0};
                uint64_t lw = 0, lr = 0, rw = 0, rr = 0;
                entasis_ray_hit_t *lp = capacity ? left : NULL, *rp = capacity ? right : NULL;
                const entasis_status_t expected = entasis_ray_cast_all(&scene.world, ray, selected, lp, capacity, &lw, &lr, &d);
                ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &d) == ENTASIS_STATUS_OK);
                const entasis_status_t actual = entasis_query_context_ray_cast_all(&context, ray, selected, rp, capacity, &rw, &rr, &d);
                ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &d) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(expected == actual && lw == rw && lr == rr);
                for (uint64_t h = 0; h < lw; ++h)
                {
                    ENTASIS_TEST_CHECK(left[h].collidable.packed == right[h].collidable.packed);
                    ENTASIS_TEST_CHECK(left[h].t == right[h].t && left[h].child_index == right[h].child_index);
                }
            }
            {
                entasis_sweep_hit_t left[8] = {0}, right[8] = {0};
                uint64_t lw = 0, lr = 0, rw = 0, rr = 0;
                entasis_sweep_hit_t *lp = capacity ? left : NULL, *rp = capacity ? right : NULL;
                const entasis_status_t expected = entasis_sweep_all(&scene.world, scene.sphere, pose, velocity, 30, selected, NULL, lp, capacity, &lw, &lr, &d);
                ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &d) == ENTASIS_STATUS_OK);
                const entasis_status_t actual = entasis_query_context_sweep_all(&context, scene.sphere, pose, velocity, 30, selected, NULL, rp, capacity, &rw, &rr, &d);
                ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &d) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(expected == actual && lw == rw && lr == rr);
                for (uint64_t h = 0; h < lw; ++h)
                {
                    ENTASIS_TEST_CHECK(left[h].collidable.packed == right[h].collidable.packed);
                    ENTASIS_TEST_CHECK(left[h].sweep.t0 == right[h].sweep.t0);
                }
            }
            {
                entasis_overlap_hit_t left[8] = {0}, right[8] = {0};
                uint64_t lw = 0, lr = 0, rw = 0, rr = 0;
                entasis_overlap_hit_t *lp = capacity ? left : NULL, *rp = capacity ? right : NULL;
                const entasis_status_t expected = entasis_overlap_all(&scene.world, scene.sphere, overlapping, selected, lp, capacity, &lw, &lr, &d);
                ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &d) == ENTASIS_STATUS_OK);
                const entasis_status_t actual = entasis_query_context_overlap_all(&context, scene.sphere, overlapping, selected, rp, capacity, &rw, &rr, &d);
                ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &d) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(expected == actual && lw == rw && lr == rr);
                for (uint64_t h = 0; h < lw; ++h)
                {
                    ENTASIS_TEST_CHECK(left[h].collidable.packed == right[h].collidable.packed);
                    ENTASIS_TEST_CHECK(left[h].manifold.kind == right[h].manifold.kind);
                }
            }
            {
                entasis_volume_hit_t left[8] = {0}, right[8] = {0};
                uint64_t lw = 0, lr = 0, rw = 0, rr = 0;
                entasis_volume_hit_t *lp = capacity ? left : NULL, *rp = capacity ? right : NULL;
                const entasis_status_t expected = entasis_volume_all(&scene.world, bounds, selected, lp, capacity, &lw, &lr, &d);
                ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &d) == ENTASIS_STATUS_OK);
                const entasis_status_t actual = entasis_query_context_volume_all(&context, bounds, selected, rp, capacity, &rw, &rr, &d);
                ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &d) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(expected == actual && lw == rw && lr == rr);
                for (uint64_t h = 0; h < lw; ++h)
                {
                    ENTASIS_TEST_CHECK(left[h].collidable.packed == right[h].collidable.packed);
                }
            }
        }
        entasis_bool_t expected = ENTASIS_FALSE, actual = ENTASIS_FALSE;
        const entasis_status_t status = entasis_sweep_any(&scene.world, scene.sphere, pose, velocity, 30, selected, NULL, &expected, &d);
        ENTASIS_TEST_CHECK(entasis_query_context_sweep_any(&context, scene.sphere, pose, velocity, 30, selected, NULL, &actual, &d) == status);
        ENTASIS_TEST_CHECK(expected == actual);
        entasis_sweep_hit_t left = {0}, right = {0};
        const entasis_status_t closest = entasis_sweep_closest(&scene.world, scene.sphere, pose, velocity, 30, selected, NULL, &left, &d);
        ENTASIS_TEST_CHECK(entasis_query_context_sweep_closest(&context, scene.sphere, pose, velocity, 30, selected, NULL, &right, &d) == closest);
        ENTASIS_TEST_CHECK(left.collidable.packed == right.collidable.packed && left.sweep.t0 == right.sweep.t0);
    }
    entasis_collision_query_t pairs[2] = {0};
    for (unsigned k = 0; k < 2u; ++k)
    {
        pairs[k].shape_a = scene.sphere;
        pairs[k].shape_b = scene.sphere;
        pairs[k].pose_a = overlapping;
        pairs[k].pose_b = pose_at(k ? 10.0f : 1.0f, 0, 0);
    }
    entasis_collision_query_result_t left[2] = {0}, right[2] = {0};
    const entasis_status_t expected = entasis_collision_query_batch(&scene.world, pairs, 2, left, 2, &d);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_collision_query_batch(&context, pairs, 2, right, 2, &d) == expected);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &d) == ENTASIS_STATUS_OK);
    for (unsigned k = 0; k < 2u; ++k)
        ENTASIS_TEST_CHECK(left[k].status == right[k].status && left[k].hit == right[k].hit && left[k].manifold.kind == right[k].manifold.kind);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&context, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&scene.world, &d) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct distance_reentry_t
{
    entasis_world_t *world;
    entasis_query_context_t *query;
    entasis_shape_handle_t shape;
    unsigned calls;
    unsigned failures;
} distance_reentry_t;

static entasis_bool_t ENTASIS_CALL distance_reentry(void *raw, entasis_collidable_reference_t target)
{
    distance_reentry_t *state = (distance_reentry_t *)raw;
    entasis_diagnostic_t diagnostic = {0};
    entasis_shape_distance_result_t result = {0};
    entasis_overlap_state_t overlap = ENTASIS_OVERLAP_INTERSECTING;
    (void)target;
    state->calls += 1u;
    if (entasis_shape_distance(state->world, state->shape, pose_at(0, 0, 0), state->shape, pose_at(4, 0, 0), NULL, &result, &diagnostic) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->failures += 1u;
    if (state->query != NULL && entasis_query_context_overlap_any(state->query, state->shape, pose_at(0, 0, 0), NULL, &overlap, &diagnostic) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->failures += 1u;
    if (entasis_distance_query_reserve(state->world, NULL, &diagnostic) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->failures += 1u;
    return ENTASIS_TRUE;
}

static int test_distance_queries(void)
{
    query_scene_t scene;
    ENTASIS_TEST_CHECK(query_scene_init(&scene) == 0);
    entasis_diagnostic_t diagnostic = {0};
    entasis_query_context_t query = {0};
    entasis_distance_query_settings_t settings = entasis_distance_query_settings_default();
    entasis_distance_query_capacity_t capacity = entasis_distance_query_capacity_default();
    ENTASIS_TEST_CHECK(settings.absolute_tolerance > 0 && settings.maximum_iterations > 0);
    ENTASIS_TEST_CHECK(capacity.vertices > 0 && capacity.faces > 0 && capacity.edges > 0);
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &scene.world, NULL, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_shape_distance_result_t result = {0}, other = {0};
    const entasis_rigid_pose_t origin = pose_at(0, 0, 0);
    const entasis_rigid_pose_t separated = pose_at(4, 0, 0);
    const entasis_rigid_pose_t penetrating = pose_at(1, 0, 0);
    ENTASIS_TEST_CHECK(entasis_shape_closest_point(&scene.world, (entasis_vector3_t){3, 0, 0}, scene.sphere, origin, NULL, &result, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(result.geometry.state == ENTASIS_DISTANCE_SEPARATED && nearf(result.geometry.distance, 2, 1e-5f));
    ENTASIS_TEST_CHECK(nearf(result.geometry.point_a.x, 3, 1e-5f) && nearf(result.geometry.point_b.x, 1, 1e-5f));
    ENTASIS_TEST_CHECK(entasis_query_context_shape_closest_point(&query, (entasis_vector3_t){0, 0, 0}, scene.box, origin, &settings, &other, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(other.geometry.distance == 0);
    ENTASIS_TEST_CHECK(entasis_shape_distance(&scene.world, scene.sphere, origin, scene.sphere, separated, NULL, &result, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(result.geometry.state == ENTASIS_DISTANCE_SEPARATED && nearf(result.geometry.distance, 2, 1e-5f));
    ENTASIS_TEST_CHECK(entasis_query_context_shape_distance(&query, scene.sphere, origin, scene.sphere, separated, &settings, &other, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(result.geometry.distance == other.geometry.distance && result.geometry.point_a.x == other.geometry.point_a.x);
    ENTASIS_TEST_CHECK(entasis_shape_penetration(&scene.world, scene.sphere, origin, scene.sphere, penetrating, NULL, &result, &diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(result.geometry.state == ENTASIS_DISTANCE_UNRESOLVED);
    ENTASIS_TEST_CHECK(entasis_query_context_shape_penetration(&query, scene.sphere, origin, scene.sphere, penetrating, NULL, &result, &diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(entasis_distance_query_reserve(&scene.world, &capacity, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_distance_query_reserve(&query, NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_penetration(&scene.world, scene.sphere, origin, scene.sphere, penetrating, NULL, &result, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(result.geometry.state == ENTASIS_DISTANCE_PENETRATING && nearf(result.geometry.depth, 1, 1e-4f));
    ENTASIS_TEST_CHECK(entasis_query_context_shape_penetration(&query, scene.sphere, origin, scene.sphere, penetrating, NULL, &other, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(result.geometry.depth == other.geometry.depth);
    entasis_shape_correction_result_t correction = {0}, correction_other = {0};
    ENTASIS_TEST_CHECK(entasis_shape_depenetrate(&scene.world, scene.sphere, origin, scene.box, penetrating, NULL, &correction, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_shape_depenetrate(&query, scene.sphere, origin, scene.box, penetrating, NULL, &correction_other, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(correction.translation.x == correction_other.translation.x && correction.state == correction_other.state);
    entasis_rigid_pose_t corrected = origin;
    corrected.position = correction.translation;
    ENTASIS_TEST_CHECK(entasis_shape_distance(&scene.world, scene.sphere, corrected, scene.box, penetrating, NULL, &result, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(result.geometry.state != ENTASIS_DISTANCE_PENETRATING);
    entasis_collidable_reference_t reference = {0};
    entasis_shape_handle_t shape = {0};
    entasis_rigid_pose_t resolved = {0};
    ENTASIS_TEST_CHECK(entasis_static_collidable_reference(&scene.world, scene.static_body, &reference, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collidable_shape_pose(&scene.world, reference, &shape, &resolved, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(shape.packed == scene.box.packed && resolved.position.x == 5);
    ENTASIS_TEST_CHECK(entasis_query_context_collidable_shape_pose(&query, reference, &shape, &resolved, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(shape.packed == scene.box.packed && resolved.position.x == 5);
    entasis_distance_query_t queries[5] = {0};
    entasis_distance_query_result_t results[5] = {0}, context_results[5] = {0};
    for (unsigned i = 0; i < 5u; ++i)
    {
        queries[i].kind = (entasis_distance_query_kind_t)(i < 4u ? i : 255u);
        queries[i].shape_a = scene.sphere;
        queries[i].shape_b = scene.sphere;
        queries[i].pose_a = origin;
        queries[i].pose_b = i == 1u ? separated : penetrating;
        queries[i].point = (entasis_vector3_t){4, 0, 0};
        queries[i].settings = settings;
    }
    /* a bad first item must not suppress later valid queries */
    entasis_distance_query_t saved = queries[0];
    queries[0] = queries[4];
    queries[4] = saved;
    ENTASIS_TEST_CHECK(entasis_distance_query_batch(&scene.world, queries, 5, results, 5, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(results[0].status == ENTASIS_STATUS_INVALID_ARGUMENT);
    for (unsigned i = 1; i < 5u; ++i)
        ENTASIS_TEST_CHECK(results[i].status == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_distance_query_batch(&query, queries, 5, context_results, 5, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    for (unsigned i = 0; i < 5u; ++i)
    {
        ENTASIS_TEST_CHECK(results[i].status == context_results[i].status);
        ENTASIS_TEST_CHECK(results[i].shape.geometry.distance == context_results[i].shape.geometry.distance);
        ENTASIS_TEST_CHECK(results[i].shape.geometry.depth == context_results[i].shape.geometry.depth);
        ENTASIS_TEST_CHECK(results[i].correction.translation.x == context_results[i].correction.translation.x);
    }
    results[0].status = UINT8_C(254);
    ENTASIS_TEST_CHECK(entasis_distance_query_batch(&scene.world, queries, 5, results, 4, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(results[0].status == UINT8_C(254));
    ENTASIS_TEST_CHECK(entasis_distance_query_batch(&scene.world, queries, 1, (entasis_distance_query_result_t *)(void *)queries, 1, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_distance_query_batch(&scene.world, NULL, 0, NULL, 0, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_distance_query_batch(&query, NULL, 0, NULL, 0, &diagnostic) == ENTASIS_STATUS_OK);
    settings.maximum_iterations = 0;
    result.geometry.state = ENTASIS_DISTANCE_PENETRATING;
    ENTASIS_TEST_CHECK(entasis_shape_distance(&scene.world, scene.sphere, origin, scene.box, separated, &settings, &result, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(result.geometry.state == ENTASIS_DISTANCE_UNRESOLVED);
    ENTASIS_TEST_CHECK(entasis_shape_distance(&scene.world, scene.sphere, origin, scene.box, separated, NULL, NULL, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    entasis_overlap_state_t overlap = ENTASIS_OVERLAP_SEPARATED;
    distance_reentry_t trace = {&scene.world, &query, scene.sphere, 0, 0};
    entasis_query_filter_t filter = entasis_query_filter_all();
    filter.allow = distance_reentry;
    filter.user_context = &trace;
    ENTASIS_TEST_CHECK(entasis_query_context_overlap_any(&query, scene.sphere, origin, &filter, &overlap, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(overlap == ENTASIS_OVERLAP_INTERSECTING && trace.calls == 1u && trace.failures == 0u);
    filter = entasis_query_filter_mobility(ENTASIS_COLLIDABLE_STATIC);
    ENTASIS_TEST_CHECK(entasis_overlap_any(&scene.world, scene.sphere, origin, &filter, &overlap, &diagnostic) == ENTASIS_STATUS_OK && overlap == ENTASIS_OVERLAP_SEPARATED);
    ENTASIS_TEST_CHECK(entasis_overlap_any(&scene.world, scene.sphere, pose_at(4.5f, 0, 0), &filter, &overlap, &diagnostic) == ENTASIS_STATUS_OK && overlap == ENTASIS_OVERLAP_INTERSECTING);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_distance_query_reserve(&scene.world, NULL, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_query_context_distance_query_reserve(&query, NULL, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_shape_distance(&scene.world, scene.sphere, origin, scene.sphere, separated, NULL, &result, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_query_context_shape_distance(&query, scene.sphere, origin, scene.sphere, separated, NULL, &result, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_overlap_any(&query, scene.sphere, origin, NULL, &overlap, &diagnostic) == ENTASIS_STATUS_OK && overlap == ENTASIS_OVERLAP_INTERSECTING);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&scene.world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_shape_distance(&query, scene.sphere, origin, scene.sphere, separated, NULL, &result, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    query_scene_destroy(&scene);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 2 || (argc == 2 && strcmp(argv[1], "queries") != 0))
    {
        return 2;
    }
    ENTASIS_TEST_CHECK(test_distance_queries() == 0);
    ENTASIS_TEST_CHECK(test_query_context_scalar_family_parity() == 0);
    ENTASIS_TEST_CHECK(test_query_context_access_lifetime_and_preflight() == 0);
    ENTASIS_TEST_CHECK(test_query_context_concurrent_readers() == 0);
    ENTASIS_TEST_CHECK(test_scalar_queries_and_filters() == 0);
    ENTASIS_TEST_CHECK(test_query_batches_and_direct_collision() == 0);
    if (argc == 1)
    {
        ENTASIS_TEST_CHECK(test_views_profiling_and_contact_events() == 0);
    }
    return 0;
}
