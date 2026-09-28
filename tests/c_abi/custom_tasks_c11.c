#include "custom_task_fixture.h"

static int test_task_registration_and_scalar(void)
{
    task_fixture_t f = {0};
    ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0);
    entasis_collision_task_registration_t c = task_collision_registration(&f);
    int32_t id = 123;
    for (unsigned bad = 0; bad < 9; ++bad)
    {
        entasis_collision_task_registration_t r = c;
        switch (bad)
        {
        case 0:
            r.struct_size = 8;
            break;
        case 1:
            r.struct_version = 2;
            break;
        case 2:
            r.test = NULL;
            break;
        case 3:
            r.wide_test = NULL;
            break;
        case 4:
            r.batch_size = 0;
            break;
        case 5:
            r.batch_size = 33;
            break;
        case 6:
            r.pair_type = 5;
            break;
        case 7:
            r.shape_type_a = 127;
            break;
        default:
            r.shape_type_a = ENTASIS_SHAPE_TYPE_SPHERE;
            break;
        }
        ENTASIS_TEST_CHECK(entasis_collision_task_register(&f.world, &r, &id, NULL) != ENTASIS_STATUS_OK && id == -1);
    }
    entasis_sweep_task_registration_t s = task_sweep_registration(&f);
    for (unsigned bad = 0; bad < 6; ++bad)
    {
        entasis_sweep_task_registration_t r = s;
        switch (bad)
        {
        case 0:
            r.struct_size = 8;
            break;
        case 1:
            r.struct_version = 2;
            break;
        case 2:
            r.test = NULL;
            break;
        case 3:
            r.child_test = NULL;
            break;
        case 4:
            r.shape_type_a = 127;
            break;
        default:
            r.shape_type_a = ENTASIS_SHAPE_TYPE_SPHERE;
            break;
        }
        ENTASIS_TEST_CHECK(entasis_sweep_task_register(&f.world, &r, &id, NULL) != ENTASIS_STATUS_OK && id == -1);
    }
    /* output aliases a descriptor field: snapshot must precede the -1 write */
    ENTASIS_TEST_CHECK(entasis_collision_task_register(&f.world, &c, &c.shape_type_a, NULL) == ENTASIS_STATUS_OK && c.shape_type_a >= 0);
    ENTASIS_TEST_CHECK(entasis_sweep_task_register(&f.world, &s, &s.shape_type_a, NULL) == ENTASIS_STATUS_OK && s.shape_type_a >= 0);
    c = task_collision_registration(&f);
    s = task_sweep_registration(&f);
    ENTASIS_TEST_CHECK(entasis_collision_task_register(&f.world, &c, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && id == -1);
    ENTASIS_TEST_CHECK(entasis_sweep_task_register(&f.world, &s, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && id == -1);
    f.task.reentry = 1;
    entasis_contact_manifold_t a = {0}, b = {0};
    ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.custom, pose_at(0, 0, 0), f.sphere, pose_at(0, 1, 0), 0, &a, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(a.convex.count == 1 && a.convex.contacts[0].feature_id == 73 && a.convex.contacts[0].depth == 1);
    ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.sphere, pose_at(0, 1, 0), f.custom, pose_at(0, 0, 0), 0, &b, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(b.convex.count == 1 && b.convex.normal.y == -a.convex.normal.y);
    for (int bad = 1; bad <= 2; ++bad)
    {
        f.task.bad_scalar = bad;
        ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.custom, pose_at(0, 0, 0), f.sphere, pose_at(0, 1, 0), 0, &a, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    }
    f.task.bad_scalar = 0;
    f.task.scalar_status = 255;
    ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.custom, pose_at(0, 0, 0), f.sphere, pose_at(0, 1, 0), 0, &a, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    f.task.scalar_status = 0;
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&f.world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_task_register(&f.world, &c, &id, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT && id == -1);
    ENTASIS_TEST_CHECK(entasis_sweep_task_register(&f.world, &s, &id, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT && id == -1);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&f.world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(task_count(&f.task.errors) == 0);
    ENTASIS_TEST_CHECK(entasis_world_clear(&f.world, NULL) == ENTASIS_STATUS_OK);
    payload_t payload = {0};
    payload.radius = 1;
    const entasis_sphere_t sphere = entasis_sphere(1);
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&f.world, f.type, &payload, sizeof(payload), &f.custom, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(&f.world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &f.sphere, NULL) == ENTASIS_STATUS_OK);
    f.task.custom = f.custom;
    ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.custom, pose_at(0, 0, 0), f.sphere, pose_at(0, 1, 0), 0, &a, NULL) == ENTASIS_STATUS_OK && a.convex.contacts[0].feature_id == 73);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_task_wide_and_compounds(void)
{
    task_fixture_t f = {0};
    ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0 && task_fixture_bind(&f) == 0);
    f.task.reentry = 1;
    entasis_collision_query_t queries[10];
    entasis_collision_query_result_t results[10] = {0};
    for (unsigned i = 0; i < 10; ++i)
        queries[i] = (entasis_collision_query_t){f.custom, f.sphere, pose_at(0, 0, 0), pose_at(0, 1, 0), 0};
    ENTASIS_TEST_CHECK(entasis_collision_query_batch(&f.world, queries, 10, results, 10, NULL) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 10; ++i)
        ENTASIS_TEST_CHECK(results[i].status == ENTASIS_STATUS_OK && results[i].manifold.convex.count == 1 && results[i].manifold.convex.contacts[0].feature_id == 73);
    /* simulation batching is independently required. direct queries may be scalar */
    entasis_body_inertia_t inertia = {0};
    const entasis_sphere_t sphere = entasis_sphere(1);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1, &inertia) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 10; ++i)
    {
        entasis_static_description_t sd = entasis_static_body(f.custom, pose_at((float)i * 5, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t sh = {0};
        ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &sh, NULL) == ENTASIS_STATUS_OK);
        entasis_body_description_t bd = entasis_body_dynamic(f.sphere, inertia, pose_at((float)i * 5, 1, 0), entasis_velocity((entasis_vector3_t){0, 0, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
        entasis_body_handle_t bh = {0};
        ENTASIS_TEST_CHECK(entasis_body_add(&f.world, &bd, &bh, NULL) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(entasis_world_stage_predict_bounds(&f.world, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stage_collision_detection(&f.world, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(task_count(&f.task.full) > 0 && task_count(&f.task.partial) > 0 && task_count(&f.task.wide) < task_count(&f.task.lanes));
    for (int bad = 1; bad <= 5; ++bad)
    {
        f.task.bad_wide = bad;
        ENTASIS_TEST_CHECK(entasis_world_stage_collision_detection(&f.world, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    }
    f.task.bad_wide = 6;
    ENTASIS_TEST_CHECK(entasis_world_stage_collision_detection(&f.world, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    f.task.bad_wide = 0;
    f.task.wide_status = ENTASIS_STATUS_CAPACITY_MISSING;
    ENTASIS_TEST_CHECK(entasis_world_stage_collision_detection(&f.world, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_CAPACITY_MISSING);
    f.task.wide_status = 0;
    ENTASIS_TEST_CHECK(entasis_world_stage_collision_detection(&f.world, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    entasis_compound_child_t children[2] = {entasis_compound_child(f.custom, pose_at(-0.4f, 0, 0)), entasis_compound_child(f.custom, pose_at(0.4f, 0, 0))};
    entasis_shape_handle_t compound = {0};
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&f.world, children, 2, &compound, NULL) == ENTASIS_STATUS_OK);
    entasis_contact_manifold_t manifold = {0};
    ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.sphere, pose_at(0, 1, 0), compound, pose_at(0, 0, 0), 0, &manifold, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(task_count(&f.task.errors) == 0);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct task_filter_t
{
    int child, calls, reject;
    entasis_world_t *world;
} task_filter_t;
static entasis_bool_t ENTASIS_CALL task_query_child(void *ctx, entasis_collidable_reference_t collidable, int32_t child)
{
    (void)collidable;
    task_filter_t *f = (task_filter_t *)ctx;
    f->child = child;
    ++f->calls;
    return f->reject ? ENTASIS_FALSE : ENTASIS_TRUE;
}
static int test_task_sweeps(void)
{
    task_fixture_t f = {0};
    ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0 && task_fixture_bind(&f) == 0);
    f.task.reentry = 1;
    entasis_static_description_t sd = entasis_static_body(f.custom, pose_at(0, 0, 0), entasis_ccd_discrete());
    entasis_static_handle_t handle = {0};
    ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &handle, NULL) == ENTASIS_STATUS_OK);
    entasis_body_velocity_t velocity = entasis_velocity((entasis_vector3_t){1, 0, 0}, (entasis_vector3_t){0, 0, 0});
    entasis_sweep_hit_t hit = {0};
    task_filter_t filter_state = {0};
    entasis_query_filter_t filter = entasis_query_filter_all();
    filter.allow_child = task_query_child;
    filter.user_context = &filter_state;
    ENTASIS_TEST_CHECK(entasis_sweep_closest(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, &filter, NULL, &hit, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(hit.sweep.state == ENTASIS_SWEEP_HIT && fabsf(hit.sweep.t0 - 3) < 0.01f && filter_state.calls > 0 && filter_state.child == 3);
    filter_state.reject = 1;
    entasis_bool_t any = 1;
    ENTASIS_TEST_CHECK(entasis_sweep_any(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, &filter, NULL, &any, NULL) == ENTASIS_STATUS_OK && any == 0);
    filter_state.reject = 0;
    for (int bad = 1; bad <= 3; ++bad)
    {
        f.task.bad_sweep = bad;
        ENTASIS_TEST_CHECK(entasis_sweep_any(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &any, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    }
    f.task.bad_sweep = 0;
    f.task.sweep_status = 255;
    ENTASIS_TEST_CHECK(entasis_sweep_any(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &any, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    f.task.sweep_status = 0;
    ENTASIS_TEST_CHECK(entasis_static_remove(&f.world, handle, ENTASIS_AWAKENING_NONE, NULL) == ENTASIS_STATUS_OK);
    entasis_compound_child_t children[2] = {entasis_compound_child(f.custom, pose_at(-0.4f, 0, 0)), entasis_compound_child(f.custom, pose_at(0.4f, 0, 0))};
    entasis_shape_handle_t compound = {0};
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&f.world, children, 2, &compound, NULL) == ENTASIS_STATUS_OK);
    sd = entasis_static_body(compound, pose_at(0, 0, 0), entasis_ccd_discrete());
    ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &handle, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_sweep_closest(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &hit, NULL) == ENTASIS_STATUS_OK && task_count(&f.task.child) > 0);
    f.task.child_status = ENTASIS_STATUS_CAPACITY_MISSING;
    ENTASIS_TEST_CHECK(entasis_sweep_any(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &any, NULL) == ENTASIS_STATUS_CAPACITY_MISSING);
    f.task.child_status = 0;
    ENTASIS_TEST_CHECK(task_count(&f.task.errors) == 0);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct task_alloc_record_t
{
    void *memory;
    uint64_t size, alignment;
} task_alloc_record_t;
typedef struct task_alloc_state_t
{
    task_alloc_record_t records[512];
    unsigned calls, fail_from, live, errors, task_frees;
    uint64_t bytes;
    entasis_world_t *world;
    int reentry;
    entasis_collision_task_registration_t *collision;
    entasis_sweep_task_registration_t *sweep;
} task_alloc_state_t;
static void *ENTASIS_CALL task_allocate(void *raw, uint64_t size, uint64_t alignment)
{
    task_alloc_state_t *s = (task_alloc_state_t *)raw;
    ++s->calls;
    if (s->collision)
    {
        s->collision->test = NULL;
        s->collision->wide_test = NULL;
        s->collision = NULL;
    }
    if (s->sweep)
    {
        s->sweep->test = NULL;
        s->sweep->child_test = NULL;
        s->sweep = NULL;
    }
    if (s->reentry && entasis_world_destroy(s->world, NULL) != ENTASIS_STATUS_INVALID_ARGUMENT)
        ++s->errors;
    if (s->fail_from && s->calls >= s->fail_from)
        return NULL;
    const uint64_t a = alignment < sizeof(void *) ? sizeof(void *) : alignment, n = (size + a - 1) / a * a;
    void *memory = ENTASIS_TEST_ALIGNED_ALLOC((size_t)a, (size_t)n);
    if (!memory)
        return NULL;
    for (unsigned i = 0; i < 512; ++i)
        if (!s->records[i].memory)
        {
            s->records[i] = (task_alloc_record_t){memory, size, alignment};
            ++s->live;
            s->bytes += size;
            return memory;
        }
    ++s->errors;
    ENTASIS_TEST_ALIGNED_FREE(memory);
    return NULL;
}
static void ENTASIS_CALL task_free(void *raw, void *memory, uint64_t size, uint64_t alignment)
{
    task_alloc_state_t *s = (task_alloc_state_t *)raw;
    for (unsigned i = 0; i < 512; ++i)
        if (s->records[i].memory == memory)
        {
            /* the C task owner is 72 bytes. Legacy native records can free unsized */
            if (s->records[i].size == 72)
            {
                if (size != 72 || alignment != s->records[i].alignment)
                    ++s->errors;
                ++s->task_frees;
            }
            --s->live;
            s->bytes -= s->records[i].size;
            s->records[i].memory = NULL;
            ENTASIS_TEST_ALIGNED_FREE(memory);
            return;
        }
    ++s->errors;
}
static int test_task_allocator_failure_and_snapshot(void)
{
    for (unsigned sweep = 0; sweep < 2; ++sweep)
        for (unsigned failure = 1; failure <= 2; ++failure)
        {
            task_fixture_t f = {0};
            task_alloc_state_t a = {0};
            a.world = &f.world;
            entasis_allocator_t allocator = {0};
            allocator.struct_size = sizeof(allocator);
            allocator.struct_version = 1;
            allocator.user_context = &a;
            allocator.allocate = task_allocate;
            allocator.deallocate = task_free;
            ENTASIS_TEST_CHECK(task_fixture_init(&f, &allocator) == 0);
            entasis_collision_task_registration_t c = task_collision_registration(&f);
            entasis_sweep_task_registration_t s = task_sweep_registration(&f);
            const unsigned live = a.live;
            const uint64_t bytes = a.bytes;
            int32_t id = -1;
            a.reentry = 1;
            a.fail_from = a.calls + failure;
            entasis_status_t status = sweep ? entasis_sweep_task_register(&f.world, &s, &id, NULL) : entasis_collision_task_register(&f.world, &c, &id, NULL);
            ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_CAPACITY_MISSING && id == -1 && a.live == live && a.bytes == bytes && a.errors == 0);
            a.fail_from = 0;
            if (sweep)
                a.sweep = &s;
            else
                a.collision = &c;
            status = sweep ? entasis_sweep_task_register(&f.world, &s, &id, NULL) : entasis_collision_task_register(&f.world, &c, &id, NULL);
            ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK && id >= 0 && a.live == live + 2 && a.bytes == bytes + 104);
            ENTASIS_TEST_CHECK(sweep ? s.test == NULL : c.test == NULL);
            if (sweep)
            {
                entasis_static_handle_t handle = {0};
                const entasis_static_description_t sd = entasis_static_body(f.custom, pose_at(0, 0, 0), entasis_ccd_discrete());
                ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &handle, NULL) == ENTASIS_STATUS_OK);
            }
            a.fail_from = a.calls + 1;
            const unsigned calls = a.calls;
            if (sweep)
            {
                entasis_bool_t hit = 0;
                ENTASIS_TEST_CHECK(entasis_sweep_any(&f.world, f.sphere, pose_at(-5, 0, 0), entasis_velocity((entasis_vector3_t){1, 0, 0}, (entasis_vector3_t){0, 0, 0}), 10, NULL, NULL, &hit, NULL) == ENTASIS_STATUS_OK && hit);
            }
            else
            {
                entasis_contact_manifold_t m = {0};
                ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.custom, pose_at(0, 0, 0), f.sphere, pose_at(0, 1, 0), 0, &m, NULL) == ENTASIS_STATUS_OK && m.convex.contacts[0].feature_id == 73);
            }
            ENTASIS_TEST_CHECK(a.calls == calls && task_count(&f.task.errors) == 0);
            /* callback-free compound registration stays available after the
               application allocator starts failing, without requesting storage */
            entasis_compound_task_registration_t compound = entasis_compound_task_registration_default();
            compound.shape_type_a = f.type;
            compound.shape_type_b = ENTASIS_SHAPE_TYPE_COMPOUND;
            ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &compound, &id, NULL) == ENTASIS_STATUS_OK && id >= 0);
            ENTASIS_TEST_CHECK(a.calls == calls && a.live == live + 2 && a.bytes == bytes + 104);
            ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK && a.live == 0 && a.bytes == 0 && a.errors == 0 && a.task_frees == failure);
        }
    return 0;
}
typedef struct task_query_job_t
{
    entasis_query_context_t query;
    task_fixture_t *fixture;
    int failed;
} task_query_job_t;
static void task_query_work(task_query_job_t *job)
{
    task_fixture_t *f = job->fixture;
    for (unsigned i = 0; i < 200; ++i)
    {
        entasis_contact_manifold_t m = {0};
        entasis_sweep_hit_t hit = {0};
        if (entasis_query_context_collision_query(&job->query, f->custom, pose_at(0, 0, 0), f->sphere, pose_at(0, 1, 0), 0, &m, NULL) != ENTASIS_STATUS_OK || m.convex.count != 1 || m.convex.contacts[0].feature_id != f->task.tag)
            job->failed = 1;
        if (entasis_query_context_sweep_closest(&job->query, f->sphere, pose_at(-5, 0, 0), entasis_velocity((entasis_vector3_t){1, 0, 0}, (entasis_vector3_t){0, 0, 0}), 10, NULL, NULL, &hit, NULL) != ENTASIS_STATUS_OK || fabsf(hit.sweep.t0 - 3) > 0.01f)
            job->failed = 1;
    }
}
#if defined(_WIN32)
static DWORD WINAPI task_query_thread(LPVOID raw)
{
    task_query_work((task_query_job_t *)raw);
    return 0;
}
#else
static void *task_query_thread(void *raw)
{
    task_query_work((task_query_job_t *)raw);
    return NULL;
}
#endif
static int test_task_concurrent_worlds_and_contexts(void)
{
    task_fixture_t f[2] = {0};
    task_query_job_t jobs[4] = {0};
    for (unsigned w = 0; w < 2; ++w)
    {
        ENTASIS_TEST_CHECK(task_fixture_init(&f[w], NULL) == 0 && task_fixture_bind(&f[w]) == 0);
        f[w].task.tag = 111 + (int)w;
        entasis_static_handle_t handle = {0};
        const entasis_static_description_t sd = entasis_static_body(f[w].custom, pose_at(0, 0, 0), entasis_ccd_discrete());
        ENTASIS_TEST_CHECK(entasis_static_add(&f[w].world, &sd, ENTASIS_AWAKENING_NONE, &handle, NULL) == ENTASIS_STATUS_OK);
        for (unsigned i = 0; i < 2; ++i)
        {
            jobs[w * 2 + i].fixture = &f[w];
            ENTASIS_TEST_CHECK(entasis_query_context_init(&jobs[w * 2 + i].query, &f[w].world, NULL, NULL, NULL) == ENTASIS_STATUS_OK);
        }
        f[w].shape.read_only = 1;
        f[w].task.reentry = 1;
        ENTASIS_TEST_CHECK(entasis_world_begin_read(&f[w].world, NULL) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(f[0].type == f[1].type);
#if defined(_WIN32)
    HANDLE threads[4];
    for (unsigned i = 0; i < 4; ++i)
    {
        threads[i] = CreateThread(NULL, 0, task_query_thread, &jobs[i], 0, NULL);
        ENTASIS_TEST_CHECK(threads[i] != NULL);
    }
    ENTASIS_TEST_CHECK(WaitForMultipleObjects(4, threads, TRUE, INFINITE) == WAIT_OBJECT_0);
    for (unsigned i = 0; i < 4; ++i)
        CloseHandle(threads[i]);
#else
    pthread_t threads[4];
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(pthread_create(&threads[i], NULL, task_query_thread, &jobs[i]) == 0);
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(pthread_join(threads[i], NULL) == 0);
#endif
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(jobs[i].failed == 0);
    for (unsigned w = 0; w < 2; ++w)
    {
        ENTASIS_TEST_CHECK(entasis_world_end_read(&f[w].world, NULL) == ENTASIS_STATUS_OK);
        f[w].shape.read_only = 0;
        for (unsigned i = 0; i < 2; ++i)
            ENTASIS_TEST_CHECK(entasis_query_context_destroy(&jobs[w * 2 + i].query, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(task_count(&f[w].task.scalar) == 400 && task_count(&f[w].task.sweep) == 400 && task_count(&f[w].task.errors) == 0);
        ENTASIS_TEST_CHECK(entasis_world_destroy(&f[w].world, NULL) == ENTASIS_STATUS_OK);
    }
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_task_registration_and_scalar() == 0);
    ENTASIS_TEST_CHECK(test_task_wide_and_compounds() == 0);
    ENTASIS_TEST_CHECK(test_task_sweeps() == 0);
    ENTASIS_TEST_CHECK(test_task_allocator_failure_and_snapshot() == 0);
    ENTASIS_TEST_CHECK(test_task_concurrent_worlds_and_contexts() == 0);
    (void)puts("C_ABI_CUSTOM_TASKS_OK groups=5");
    return 0;
}
