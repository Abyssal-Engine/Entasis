#include "compound_task_fixture.h"

typedef struct child_state_t
{
    entasis_world_t *world;
    unsigned calls, mask, errors;
    int reject, limit;
} child_state_t;
static entasis_bool_t ENTASIS_CALL compound_allow_child(void *raw, entasis_collidable_reference_t collidable, int32_t child)
{
    child_state_t *s = (child_state_t *)raw;
    (void)collidable;
    ++s->calls;
    if (child < 0 || child >= s->limit)
        ++s->errors;
    else
        s->mask |= 1u << (unsigned)child;
    entasis_compound_task_registration_t r = entasis_compound_task_registration_default();
    int32_t id = 0;
    if (entasis_collision_task_compound_register(s->world, &r, &id, NULL) != ENTASIS_STATUS_INVALID_ARGUMENT || id != -1)
        ++s->errors;
    return s->reject ? ENTASIS_FALSE : ENTASIS_TRUE;
}

static int test_compound_registration(void)
{
    task_fixture_t f = {0};
    ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0);
    entasis_compound_task_registration_t base = entasis_compound_task_registration_default();
    ENTASIS_TEST_CHECK(base.struct_size == sizeof(base) && base.struct_version == 1 && base.shape_type_a == -1 &&
                       base.shape_type_b == -1 && base.batch_size == 16 && base.kind == ENTASIS_COLLISION_TASK_CONVEX_COMPOUND && base.capabilities == 12);
    base.shape_type_a = f.type;
    base.shape_type_b = ENTASIS_SHAPE_TYPE_COMPOUND;
    int32_t id = 0;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, NULL, &id, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT && id == -1);
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &base, NULL, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    for (unsigned bad = 0; bad < 17; ++bad)
    {
        entasis_compound_task_registration_t r = base;
        switch (bad)
        {
        case 0:
            r.struct_size = 8;
            break;
        case 1:
            r.struct_version = 2;
            break;
        case 2:
            r.batch_size = 0;
            break;
        case 3:
            r.batch_size = 33;
            break;
        case 4:
            r.batch_size = 65552;
            break;
        case 5:
            r.kind = ENTASIS_COLLISION_TASK_CONVEX;
            break;
        case 6:
            r.kind = 257;
            break;
        case 7:
            r.capabilities = 0;
            break;
        case 8:
            r.capabilities |= ENTASIS_COLLISION_TASK_CONVEX_RESULT;
            break;
        case 9:
            r.capabilities |= ENTASIS_COLLISION_TASK_WIDE_RESULT;
            break;
        case 10:
            r.capabilities |= 256;
            break;
        case 11:
            r.shape_type_a = -1;
            break;
        case 12:
            r.shape_type_a = 65545;
            break;
        case 13:
            r.shape_type_b = f.type;
            break;
        case 14:
            r.shape_type_b = ENTASIS_SHAPE_TYPE_SPHERE;
            break;
        case 15:
            r.shape_type_a = ENTASIS_SHAPE_TYPE_COMPOUND;
            break;
        default:
            r.kind = ENTASIS_COLLISION_TASK_COMPOUND_PAIR;
            break;
        }
        ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &r, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && id == -1);
    }
    /* a caller's Compound classification is not an engine-owned representation */
    entasis_custom_shape_registration_t fake = registration(&f.shape);
    fake.batch_type = ENTASIS_SHAPE_BATCH_COMPOUND;
    entasis_shape_type_id_t fake_type = -1;
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&f.world, &fake, &fake_type, NULL) == ENTASIS_STATUS_OK);
    entasis_compound_task_registration_t r = base;
    r.shape_type_b = fake_type;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &r, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    r = base;
    r.shape_type_a = fake_type;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &r, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    for (int32_t a = ENTASIS_SHAPE_TYPE_COMPOUND; a <= ENTASIS_SHAPE_TYPE_MESH; ++a)
        for (int32_t b = ENTASIS_SHAPE_TYPE_COMPOUND; b <= ENTASIS_SHAPE_TYPE_MESH; ++b)
        {
            r = base;
            r.shape_type_a = a;
            r.shape_type_b = b;
            r.kind = ENTASIS_COLLISION_TASK_COMPOUND_PAIR;
            ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &r, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && id == -1);
        }
    r = base;
    r.shape_type_a = ENTASIS_SHAPE_TYPE_SPHERE;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &r, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&f.world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &base, &id, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT && id == -1);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&f.world, NULL) == ENTASIS_STATUS_OK);
    r = base;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &r, &r.shape_type_a, NULL) == ENTASIS_STATUS_OK);
    const int32_t first = r.shape_type_a;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &base, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && id == -1);
    r = base;
    r.shape_type_b = ENTASIS_SHAPE_TYPE_BIG_COMPOUND;
    r.batch_size = 32;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &r, &id, NULL) == ENTASIS_STATUS_OK && id == first + 1);
    const entasis_compound_child_t child = entasis_compound_child(f.sphere, pose_at(0, 0, 0));
    entasis_shape_handle_t target = {0};
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&f.world, &child, 1, &target, NULL) == ENTASIS_STATUS_OK);
    entasis_contact_manifold_t m = {0};
    /* parent registration does not manufacture a missing custom leaf task */
    ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.custom, pose_at(0, 1, 0), target, pose_at(0, 0, 0), 0, &m, NULL) != ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(task_fixture_bind(&f) == 0);
    ENTASIS_TEST_CHECK(entasis_collision_query(&f.world, f.custom, pose_at(0, 1, 0), target, pose_at(0, 0, 0), 0, &m, NULL) == ENTASIS_STATUS_OK && m.nonconvex.count == 1);
    ENTASIS_TEST_CHECK(entasis_world_step(&f.world, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    r = base;
    r.shape_type_b = ENTASIS_SHAPE_TYPE_MESH;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &r, &id, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT && id == -1);
    ENTASIS_TEST_CHECK(entasis_world_clear(&f.world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &base, &id, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && id == -1);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f.world, &base, &id, NULL) == ENTASIS_STATUS_DISPOSED && id == -1);
    return 0;
}

static int test_compound_queries(int triangle_first)
{
    task_fixture_t f = {0};
    entasis_shape_handle_t targets[3] = {0};
    ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0 && task_fixture_bind(&f) == 0 && compound_targets_order(&f, targets, triangle_first) == 0);
    entasis_query_context_t query = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &f.world, NULL, NULL, NULL) == ENTASIS_STATUS_OK);
    f.task.reentry = 1;
    for (unsigned target = 0; target < 3; ++target)
    {
        const entasis_rigid_pose_t a = pose_at(0, target == 2 ? 0.5f : 0, 0), b = pose_at(0, 0, 0);
        entasis_collision_query_t pairs[2] = {0};
        pairs[0].shape_a = f.custom;
        pairs[0].shape_b = targets[target];
        pairs[0].pose_a = a;
        pairs[0].pose_b = b;
        pairs[1].shape_a = targets[target];
        pairs[1].shape_b = f.custom;
        pairs[1].pose_a = b;
        pairs[1].pose_b = a;
        entasis_collision_query_result_t ordinary[2] = {0}, contextual[2] = {0};
        const entasis_status_t batch_status = entasis_collision_query_batch(&f.world, pairs, 2, ordinary, 2, NULL);
        if (batch_status != ENTASIS_STATUS_OK)
            fprintf(stderr, "compound batch target=%u status=%u pair=%u,%u task_errors=%u\n", target, (unsigned)batch_status, (unsigned)ordinary[0].status, (unsigned)ordinary[1].status, task_count(&f.task.errors));
        ENTASIS_TEST_CHECK(batch_status == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_query_context_collision_query_batch(&query, pairs, 2, contextual, 2, NULL) == ENTASIS_STATUS_OK);
        for (unsigned flipped = 0; flipped < 2; ++flipped)
        {
            entasis_contact_manifold_t m = {0};
            const entasis_collision_query_t *pair = &pairs[flipped];
            const entasis_status_t direct_status = entasis_collision_query(&f.world, pair->shape_a, pair->pose_a, pair->shape_b, pair->pose_b, 0, &m, NULL);
            if (direct_status != ENTASIS_STATUS_OK)
                fprintf(stderr, "compound direct target=%u flip=%u status=%u\n", target, flipped, (unsigned)direct_status);
            ENTASIS_TEST_CHECK(direct_status == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(m.kind == ENTASIS_MANIFOLD_NONCONVEX && m.nonconvex.count > 0 && ordinary[flipped].hit && contextual[flipped].hit);
            ENTASIS_TEST_CHECK(memcmp(&m, &ordinary[flipped].manifold, sizeof(m)) == 0 && memcmp(&m, &contextual[flipped].manifold, sizeof(m)) == 0);
        }
        entasis_static_handle_t handle = {0};
        const entasis_static_description_t sd = entasis_static_body(targets[target], b, entasis_ccd_discrete());
        ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &handle, NULL) == ENTASIS_STATUS_OK);
        child_state_t child = {0};
        child.world = &f.world;
        child.limit = target == 2 ? 8 : 2;
        entasis_query_filter_t filter = entasis_query_filter_all();
        filter.allow_child = compound_allow_child;
        filter.user_context = &child;
        for (unsigned mode = 0; mode < 2; ++mode)
            for (unsigned reject = 0; reject < 2; ++reject)
            {
                entasis_overlap_hit_t hits[2] = {0};
                uint64_t written = 0, required = 0;
                child.calls = child.mask = child.errors = 0;
                child.reject = (int)reject;
                const entasis_status_t status = mode ? entasis_query_context_overlap_all(&query, f.custom, a, &filter, hits, 2, &written, &required, NULL) : entasis_overlap_all(&f.world, f.custom, a, &filter, hits, 2, &written, &required, NULL);
                ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK && written == (reject ? 0u : 1u) && required == written && child.calls > 0 && child.errors == 0);
                ENTASIS_TEST_CHECK(child.mask == (target == 2 ? 255u : 3u));
            }
        f.task.wide_status = ENTASIS_STATUS_CAPACITY_MISSING;
        entasis_contact_manifold_t failed = {0};
        ENTASIS_TEST_CHECK(entasis_query_context_collision_query(&query, f.custom, a, targets[target], b, 0, &failed, NULL) == ENTASIS_STATUS_CAPACITY_MISSING);
        f.task.wide_status = 0;
        ENTASIS_TEST_CHECK(entasis_query_context_collision_query(&query, f.custom, a, targets[target], b, 0, &failed, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_static_remove(&f.world, handle, ENTASIS_AWAKENING_NONE, NULL) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(task_count(&f.task.full) > 0 && task_count(&f.task.partial) > 0 && task_count(&f.task.errors) == 0);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct compound_job_t
{
    task_fixture_t *f;
    entasis_shape_handle_t *targets;
    entasis_query_context_t query;
    int failed;
} compound_job_t;
static void compound_work(compound_job_t *job)
{
    for (unsigned iteration = 0; iteration < 100; ++iteration)
        for (unsigned target = 0; target < 3; ++target)
        {
            entasis_contact_manifold_t m = {0};
            if (entasis_query_context_collision_query(&job->query, job->f->custom, pose_at(0, target == 2 ? 0.5f : 0, 0), job->targets[target], pose_at(0, 0, 0), 0, &m, NULL) != ENTASIS_STATUS_OK || m.kind != ENTASIS_MANIFOLD_NONCONVEX || m.nonconvex.count == 0)
                job->failed = 1;
        }
}
#if defined(_WIN32)
static DWORD WINAPI compound_thread(void *raw)
{
    compound_work((compound_job_t *)raw);
    return 0;
}
#else
static void *compound_thread(void *raw)
{
    compound_work((compound_job_t *)raw);
    return NULL;
}
#endif
static int test_compound_concurrent_reads(void)
{
    task_fixture_t f = {0};
    entasis_shape_handle_t targets[3] = {0};
    compound_job_t jobs[2] = {0};
    ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0 && task_fixture_bind(&f) == 0 && compound_targets(&f, targets) == 0);
    f.shape.read_only = 1;
    f.task.reentry = 1;
    for (unsigned i = 0; i < 2; ++i)
    {
        jobs[i].f = &f;
        jobs[i].targets = targets;
        ENTASIS_TEST_CHECK(entasis_query_context_init(&jobs[i].query, &f.world, NULL, NULL, NULL) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&f.world, NULL) == ENTASIS_STATUS_OK);
#if defined(_WIN32)
    HANDLE threads[2];
    for (unsigned i = 0; i < 2; ++i)
    {
        threads[i] = CreateThread(NULL, 0, compound_thread, &jobs[i], 0, NULL);
        ENTASIS_TEST_CHECK(threads[i] != NULL);
    }
    ENTASIS_TEST_CHECK(WaitForMultipleObjects(2, threads, TRUE, INFINITE) == WAIT_OBJECT_0);
    for (unsigned i = 0; i < 2; ++i)
        CloseHandle(threads[i]);
#else
    pthread_t threads[2];
    for (unsigned i = 0; i < 2; ++i)
        ENTASIS_TEST_CHECK(pthread_create(&threads[i], NULL, compound_thread, &jobs[i]) == 0);
    for (unsigned i = 0; i < 2; ++i)
        ENTASIS_TEST_CHECK(pthread_join(threads[i], NULL) == 0);
#endif
    ENTASIS_TEST_CHECK(entasis_world_end_read(&f.world, NULL) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 2; ++i)
    {
        ENTASIS_TEST_CHECK(jobs[i].failed == 0);
        ENTASIS_TEST_CHECK(entasis_query_context_destroy(&jobs[i].query, NULL) == ENTASIS_STATUS_OK);
    }
    f.shape.read_only = 0;
    ENTASIS_TEST_CHECK(task_count(&f.task.errors) == 0);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
int main(void)
{
    ENTASIS_TEST_CHECK(test_compound_registration() == 0);
    ENTASIS_TEST_CHECK(test_compound_queries(0) == 0 && test_compound_queries(1) == 0);
    ENTASIS_TEST_CHECK(test_compound_concurrent_reads() == 0);
    puts("C_ABI_COMPOUND_TASKS_OK groups=4 concurrent_queries=600");
    return 0;
}
