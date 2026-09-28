#include "custom_shape_fixture.h"
#if defined(_WIN32)
#include <windows.h>
#else
#include <pthread.h>
#endif

static int test_registration_instances_and_access(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diag = {0};
    entasis_world_description_t d = description();
    shape_state_t state = {0};
    state.world = &world;
    state.alignment = 128;
    state.delegate_child = 1;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &d, &diag) == ENTASIS_STATUS_OK);
    entasis_custom_shape_registration_t r = registration(&state);
    entasis_shape_type_id_t id = -1, next = -1;
    ENTASIS_TEST_CHECK(entasis_custom_shape_next_type_id(&world, &next, &diag) == ENTASIS_STATUS_OK && next == (entasis_shape_type_id_t)ENTASIS_BUILT_IN_SHAPE_TYPE_COUNT);
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &r, &id, &diag) == ENTASIS_STATUS_OK && id == next);
    r.bounds = NULL;
    r.inertia = NULL;
    r.user_context = NULL; /* registration is copied */
    const entasis_sphere_t sphere = entasis_sphere(2);
    payload_t input = {0};
    input.radius = 2;
    input.tag = 0x1234u;
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &input.child, &diag) == ENTASIS_STATUS_OK);
    /* deliberately unaligned input is copied, not retained or cast by the ABI */
    unsigned char bytes[sizeof(input) + 1];
    memcpy(bytes + 1, &input, sizeof(input));
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, id, bytes + 1, sizeof(input), &state.self, &diag) == ENTASIS_STATUS_OK);
    state.check_access = 1;
    state.check_support = 1;
    state.reentry = 1;
    uint64_t required = 0;
    payload_t output = {0};
    ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, state.self, NULL, 0, &required, &diag) == ENTASIS_STATUS_CAPACITY_MISSING && required == sizeof(input));
    memset(&output, 0x5a, sizeof(output));
    payload_t sentinel = output;
    ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, state.self, &output, sizeof(output) - 1, &required, &diag) == ENTASIS_STATUS_CAPACITY_MISSING && memcmp(&output, &sentinel, sizeof(output)) == 0);
    ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, state.self, &output, sizeof(output), &required, &diag) == ENTASIS_STATUS_OK && memcmp(&input, &output, sizeof(input)) == 0);
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_custom_shape_inertia(&world, state.self, 2, &inertia, &diag) == ENTASIS_STATUS_OK && inertia.inverse_mass == 0.5f);
    entasis_body_description_t body_desc = entasis_body_dynamic(state.self, inertia, pose_at(0, 0, 0), entasis_velocity((entasis_vector3_t){0, 0, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
    entasis_body_handle_t body = {0};
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_desc, &body, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, state.self, &diag) == ENTASIS_STATUS_SHAPE_IN_USE && state.disposals == 0);
    entasis_query_context_t query = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &world, NULL, NULL, &diag) == ENTASIS_STATUS_OK);
    entasis_ray_t ray = entasis_ray((entasis_vector3_t){-5, 0, 0}, (entasis_vector3_t){1, 0, 0}, 10);
    entasis_ray_hit_t hit = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&query, ray, NULL, &hit, &diag) == ENTASIS_STATUS_OK && fabsf(hit.t - 3) < 1e-5f);
    state.fail_ray = ENTASIS_STATUS_INVALID_DESCRIPTION;
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&query, ray, NULL, &hit, &diag) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    state.fail_ray = 0;
    state.bad_hit = 1;
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&query, ray, NULL, &hit, &diag) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    state.bad_hit = 0;
    state.fail_inertia = 255;
    inertia.inverse_mass = 7;
    ENTASIS_TEST_CHECK(entasis_custom_shape_inertia(&world, state.self, 2, &inertia, &diag) == ENTASIS_STATUS_INVALID_ARGUMENT && inertia.inverse_mass == 0);
    state.fail_inertia = 0;
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, state.self, &output, sizeof(output), &required, &diag) == ENTASIS_STATUS_OK);
    r = registration(&state);
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &r, &next, &diag) == ENTASIS_STATUS_INVALID_ARGUMENT && next == ENTASIS_SHAPE_TYPE_INVALID);
    ENTASIS_TEST_CHECK(entasis_custom_shape_inertia(&world, state.self, 1, &inertia, &diag) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&world, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_remove(&world, body, &diag) == ENTASIS_STATUS_OK);
    state.fail_dispose = ENTASIS_STATUS_INVALID_ARGUMENT;
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, state.self, &diag) == ENTASIS_STATUS_INVALID_ARGUMENT && state.disposals == 0);
    ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, state.self, &output, sizeof(output), &required, &diag) == ENTASIS_STATUS_OK);
    state.fail_dispose = 0;
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, state.self, &diag) == ENTASIS_STATUS_OK && state.disposals == 1);
    ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, state.self, &output, sizeof(output), &required, &diag) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(state.errors == 0 && state.bounds > 0 && state.inertia > 0 && state.rays > 0 && state.support >= 2 && state.sweep_support > 0);
    state.check_access = 0;
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, id, &input, sizeof(input), &state.self, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_clear(&world, &diag) == ENTASIS_STATUS_OK && state.disposals == 2);
    state.delegate_child = 0;
    input.child = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, id, &input, sizeof(input), &state.self, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diag) == ENTASIS_STATUS_OK && state.disposals == 3 && state.errors == 0);
    return 0;
}

static int test_validation_and_capacity(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diag = {0};
    entasis_world_description_t d = description();
    shape_state_t state = {0};
    state.world = &world;
    state.alignment = 4;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &d, &diag) == ENTASIS_STATUS_OK);
    entasis_custom_shape_registration_t good = registration(&state), bad;
    entasis_shape_type_id_t id = -1;
    for (unsigned fault = 0; fault < 11; ++fault)
    {
        bad = good;
        switch (fault)
        {
        case 0:
            bad.struct_size = 0;
            break;
        case 1:
            bad.struct_version = 2;
            break;
        case 2:
            bad.size = 0;
            break;
        case 3:
            bad.size = UINT64_MAX;
            break;
        case 4:
            bad.alignment = 256;
            break;
        case 5:
            bad.alignment = 3;
            break;
        case 6:
            bad.batch_type = UINT32_MAX;
            break;
        case 7:
            bad.bounds = NULL;
            break;
        case 8:
            bad.inertia = NULL;
            break;
        case 9:
            bad.ray = NULL;
            break;
        default:
            bad.support = NULL;
            break;
        }
        ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &bad, &id, &diag) == ENTASIS_STATUS_INVALID_DESCRIPTION && id == ENTASIS_SHAPE_TYPE_INVALID);
        ENTASIS_TEST_CHECK(entasis_custom_shape_next_type_id(&world, &id, &diag) == ENTASIS_STATUS_OK && id == (entasis_shape_type_id_t)ENTASIS_BUILT_IN_SHAPE_TYPE_COUNT);
    }
    bad = good;
    bad.expected_type_id = ENTASIS_SHAPE_TYPE_SPHERE;
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &bad, &id, &diag) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &bad, &bad.expected_type_id, &diag) == ENTASIS_STATUS_INVALID_DESCRIPTION && bad.expected_type_id == ENTASIS_SHAPE_TYPE_INVALID);
    entasis_shape_access_t empty = {0};
    entasis_custom_shape_view_t view = {0};
    entasis_shape_bounds_t bounds = {0};
    entasis_body_inertia_t inertia = {0};
    entasis_shape_ray_hit_t hit = {0};
    entasis_vector3_t v = {1, 0, 0}, out = {0};
    entasis_quaternion_t q = {0, 0, 0, 1};
    entasis_rigid_pose_t p = pose_at(0, 0, 0);
    entasis_ray_t ray = entasis_ray(v, v, 5);
    entasis_shape_handle_t invalid = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_access_custom_data(empty, invalid, &view) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_shape_access_bounds(empty, invalid, &q, &bounds) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_shape_access_inertia(empty, invalid, 1, &inertia) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_shape_access_ray(empty, invalid, &p, &ray, &hit) == ENTASIS_STATUS_INVALID_ARGUMENT && hit.hit == 0);
    ENTASIS_TEST_CHECK(entasis_shape_access_support(empty, invalid, &v, &out) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_shape_access_sweep_support(empty, invalid, &v, &out) == ENTASIS_STATUS_INVALID_ARGUMENT);
    good.dispose = NULL;
    good.sweep_support = NULL;
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &good, &id, &diag) == ENTASIS_STATUS_OK);
    payload_t value = {0};
    value.radius = 1;
    entasis_shape_handle_t handle = invalid;
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, id, &value, sizeof(value) - 1, &handle, &diag) == ENTASIS_STATUS_INVALID_ARGUMENT && !entasis_shape_handle_is_valid(handle));
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &value, sizeof(value), &handle, &diag) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, id, &value, sizeof(value), &handle, &diag) == ENTASIS_STATUS_OK);
    state.self = handle;
    state.check_support = 1;
    ENTASIS_TEST_CHECK(entasis_custom_shape_inertia(&world, handle, 1, &inertia, &diag) == ENTASIS_STATUS_OK && state.support == 2 && state.sweep_support == 0 && state.errors == 0);
    ENTASIS_TEST_CHECK(entasis_shape_remove(&world, handle, &diag) == ENTASIS_STATUS_OK && state.disposals == 0);
    for (unsigned i = ENTASIS_BUILT_IN_SHAPE_TYPE_COUNT + 1; i < ENTASIS_MAXIMUM_SHAPE_TYPE_COUNT; ++i)
    {
        good.expected_type_id = (entasis_shape_type_id_t)i;
        ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &good, &id, &diag) == ENTASIS_STATUS_OK && id == (entasis_shape_type_id_t)i);
    }
    ENTASIS_TEST_CHECK(entasis_custom_shape_next_type_id(&world, &id, &diag) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &good, &id, &diag) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &d, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64.0f, &diag) == ENTASIS_STATUS_OK);
    good = registration(&state);
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &good, &id, &diag) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diag) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct alloc_record_t
{
    void *memory;
    uint64_t size, alignment;
} alloc_record_t;
typedef struct alloc_state_t
{
    alloc_record_t records[512];
    unsigned calls, fail_from, live, errors, exact_bridge_frees;
    uint64_t live_bytes, peak_bytes;
    entasis_world_t *world;
    int reentry;
    entasis_custom_shape_registration_t *mutate;
} alloc_state_t;
static void *ENTASIS_CALL allocate_callback(void *raw, uint64_t size, uint64_t alignment)
{
    alloc_state_t *s = (alloc_state_t *)raw;
    ++s->calls;
    if (s->mutate)
    {
        s->mutate->bounds = NULL;
        s->mutate->size = 0;
        s->mutate = NULL;
    }
    if (s->reentry)
    {
        entasis_diagnostic_t d = {0};
        if (entasis_world_destroy(s->world, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
            ++s->errors;
    }
    if (s->fail_from && s->calls >= s->fail_from)
        return NULL;
    const uint64_t a = alignment < sizeof(void *) ? sizeof(void *) : alignment;
    const uint64_t n = (size + a - 1) / a * a;
    void *memory = ENTASIS_TEST_ALIGNED_ALLOC((size_t)a, (size_t)n);
    if (!memory)
        return NULL;
    for (unsigned i = 0; i < 512; ++i)
        if (!s->records[i].memory)
        {
            s->records[i] = (alloc_record_t){memory, size, alignment};
            ++s->live;
            s->live_bytes += size;
            if (s->live_bytes > s->peak_bytes)
                s->peak_bytes = s->live_bytes;
            return memory;
        }
    ++s->errors;
    ENTASIS_TEST_ALIGNED_FREE(memory);
    return NULL;
}
static void ENTASIS_CALL free_callback(void *raw, void *memory, uint64_t size, uint64_t alignment)
{
    alloc_state_t *s = (alloc_state_t *)raw;
    for (unsigned i = 0; i < 512; ++i)
        if (s->records[i].memory == memory)
        {
            /* existing native Legacy owners may free unsized. new C records must not */
            if (s->records[i].size == sizeof(void *) + sizeof(entasis_custom_shape_registration_t))
            {
                if (size != s->records[i].size || alignment != s->records[i].alignment)
                    ++s->errors;
                ++s->exact_bridge_frees;
            }
            s->live_bytes -= s->records[i].size;
            --s->live;
            s->records[i].memory = NULL;
            ENTASIS_TEST_ALIGNED_FREE(memory);
            return;
        }
    ++s->errors;
}
static int test_allocation_failure_and_release(void)
{
    for (unsigned failure = 1; failure <= 2; ++failure)
    {
        entasis_world_t world = {0};
        entasis_diagnostic_t diag = {0};
        alloc_state_t allocations = {0};
        allocations.world = &world;
        entasis_world_description_t d = description();
        d.allocator = (entasis_allocator_t){0};
        d.allocator.struct_size = sizeof(d.allocator);
        d.allocator.struct_version = 1;
        d.allocator.user_context = &allocations;
        d.allocator.allocate = allocate_callback;
        d.allocator.deallocate = free_callback;
        ENTASIS_TEST_CHECK(entasis_world_init(&world, &d, &diag) == ENTASIS_STATUS_OK);
        shape_state_t state = {0};
        state.world = &world;
        state.alignment = 16;
        entasis_custom_shape_registration_t r = registration(&state);
        entasis_shape_type_id_t id = -1;
        const unsigned live = allocations.live;
        const uint64_t bytes = allocations.live_bytes;
        allocations.reentry = 1;
        allocations.fail_from = allocations.calls + failure;
        ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &r, &id, &diag) == ENTASIS_STATUS_CAPACITY_MISSING && id == ENTASIS_SHAPE_TYPE_INVALID);
        ENTASIS_TEST_CHECK(allocations.live == live && allocations.live_bytes == bytes && allocations.errors == 0);
        ENTASIS_TEST_CHECK(entasis_custom_shape_next_type_id(&world, &id, &diag) == ENTASIS_STATUS_OK && id == (entasis_shape_type_id_t)ENTASIS_BUILT_IN_SHAPE_TYPE_COUNT);
        allocations.fail_from = 0;
        allocations.mutate = &r;
        ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &r, &id, &diag) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(allocations.live == live + 2 && allocations.live_bytes == bytes + 160 && r.bounds == NULL && r.size == 0);
        payload_t value = {0};
        value.child = entasis_shape_handle_invalid();
        value.radius = 1;
        ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, id, &value, sizeof(value), &state.self, &diag) == ENTASIS_STATUS_OK);
        entasis_body_inertia_t inertia = {0};
        ENTASIS_TEST_CHECK(entasis_custom_shape_inertia(&world, state.self, 1, &inertia, &diag) == ENTASIS_STATUS_OK && inertia.inverse_mass == 1);
        allocations.fail_from = allocations.calls + 1;
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diag) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(allocations.live == 0 && allocations.live_bytes == 0 && allocations.errors == 0 && allocations.exact_bridge_frees == failure && state.disposals == 1);
    }
    return 0;
}

typedef struct query_job_t
{
    entasis_query_context_t *query;
    entasis_ray_t ray;
    float expected;
    int failed;
} query_job_t;
static void query_work(query_job_t *job)
{
    for (unsigned i = 0; i < 200; ++i)
    {
        entasis_ray_hit_t hit = {0};
        entasis_diagnostic_t d = {0};
        if (entasis_query_context_ray_cast_closest(job->query, job->ray, NULL, &hit, &d) != ENTASIS_STATUS_OK || fabsf(hit.t - job->expected) > 1e-5f)
            job->failed = 1;
    }
}
#if defined(_WIN32)
static DWORD WINAPI query_thread(LPVOID raw)
{
    query_work((query_job_t *)raw);
    return 0;
}
#else
static void *query_thread(void *raw)
{
    query_work((query_job_t *)raw);
    return NULL;
}
#endif
static int test_independent_worlds_and_concurrent_contexts(void)
{
    entasis_world_t worlds[2] = {{0}, {0}};
    entasis_diagnostic_t diag = {0};
    entasis_world_description_t d = description();
    shape_state_t states[2] = {{0}, {0}};
    entasis_query_context_t contexts[2][2] = {{{0}, {0}}, {{0}, {0}}};
    query_job_t jobs[4];
    for (unsigned w = 0; w < 2; ++w)
    {
        states[w].world = &worlds[w];
        states[w].alignment = 32;
        ENTASIS_TEST_CHECK(entasis_world_init(&worlds[w], &d, &diag) == ENTASIS_STATUS_OK);
        entasis_custom_shape_registration_t r = registration(&states[w]);
        entasis_shape_type_id_t id = -1;
        ENTASIS_TEST_CHECK(entasis_custom_shape_register(&worlds[w], &r, &id, &diag) == ENTASIS_STATUS_OK && id == (entasis_shape_type_id_t)ENTASIS_BUILT_IN_SHAPE_TYPE_COUNT);
        payload_t p = {0};
        p.radius = (float)(w + 1);
        p.tag = w;
        ENTASIS_TEST_CHECK(entasis_custom_shape_add(&worlds[w], id, &p, sizeof(p), &states[w].self, &diag) == ENTASIS_STATUS_OK);
        entasis_static_description_t stat = entasis_static_body(states[w].self, pose_at(0, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t handle = {0};
        ENTASIS_TEST_CHECK(entasis_static_add(&worlds[w], &stat, ENTASIS_AWAKENING_NONE, &handle, &diag) == ENTASIS_STATUS_OK);
        for (unsigned q = 0; q < 2; ++q)
        {
            ENTASIS_TEST_CHECK(entasis_query_context_init(&contexts[w][q], &worlds[w], NULL, NULL, &diag) == ENTASIS_STATUS_OK);
            jobs[w * 2 + q] = (query_job_t){&contexts[w][q], entasis_ray((entasis_vector3_t){-5, 0, 0}, (entasis_vector3_t){1, 0, 0}, 10), 5 - p.radius, 0};
        }
        states[w].read_only = 1;
        ENTASIS_TEST_CHECK(entasis_world_begin_read(&worlds[w], &diag) == ENTASIS_STATUS_OK);
    }
#if defined(_WIN32)
    HANDLE threads[4];
    for (unsigned i = 0; i < 4; ++i)
    {
        threads[i] = CreateThread(NULL, 0, query_thread, &jobs[i], 0, NULL);
        ENTASIS_TEST_CHECK(threads[i] != NULL);
    }
    ENTASIS_TEST_CHECK(WaitForMultipleObjects(4, threads, TRUE, INFINITE) == WAIT_OBJECT_0);
    for (unsigned i = 0; i < 4; ++i)
        CloseHandle(threads[i]);
#else
    pthread_t threads[4];
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(pthread_create(&threads[i], NULL, query_thread, &jobs[i]) == 0);
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(pthread_join(threads[i], NULL) == 0);
#endif
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(jobs[i].failed == 0);
    for (unsigned w = 0; w < 2; ++w)
    {
        ENTASIS_TEST_CHECK(entasis_world_end_read(&worlds[w], &diag) == ENTASIS_STATUS_OK);
        states[w].read_only = 0;
        for (unsigned q = 0; q < 2; ++q)
            ENTASIS_TEST_CHECK(entasis_query_context_destroy(&contexts[w][q], &diag) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_destroy(&worlds[w], &diag) == ENTASIS_STATUS_OK && states[w].disposals == 1 && states[w].errors == 0);
    }
    return 0;
}
static int test_compound_child_ownership_and_queries(void)
{
    for (unsigned big = 0; big < 2; ++big)
    {
        entasis_world_t world = {0};
        entasis_diagnostic_t diag = {0};
        entasis_world_description_t d = description();
        shape_state_t state = {0};
        state.world = &world;
        state.alignment = 16;
        ENTASIS_TEST_CHECK(entasis_world_init(&world, &d, &diag) == ENTASIS_STATUS_OK);
        entasis_custom_shape_registration_t r = registration(&state);
        entasis_shape_type_id_t type = -1;
        ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &r, &type, &diag) == ENTASIS_STATUS_OK);
        payload_t value = {0};
        value.child = entasis_shape_handle_invalid();
        value.radius = 0.5f;
        ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, type, &value, sizeof(value), &state.self, &diag) == ENTASIS_STATUS_OK);
        state.check_access = 1;
        const entasis_compound_child_t children[2] = {
            entasis_compound_child(state.self, pose_at(-0.4f, 0, 0)), entasis_compound_child(state.self, pose_at(0.4f, 0, 0))};
        entasis_shape_handle_t compound = entasis_shape_handle_invalid();
        const entasis_status_t imported = big ? entasis_shape_import_big_compound(&world, children, 2, &compound, &diag) : entasis_shape_import_compound(&world, children, 2, &compound, &diag);
        ENTASIS_TEST_CHECK(imported == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_shape_remove(&world, state.self, &diag) == ENTASIS_STATUS_SHAPE_IN_USE);
        entasis_static_description_t stat = entasis_static_body(compound, pose_at(0, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t handle = {0};
        ENTASIS_TEST_CHECK(entasis_static_add(&world, &stat, ENTASIS_AWAKENING_NONE, &handle, &diag) == ENTASIS_STATUS_OK);
        entasis_query_context_t query = {0};
        ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &world, NULL, NULL, &diag) == ENTASIS_STATUS_OK);
        entasis_ray_t ray = entasis_ray((entasis_vector3_t){-5, 0, 0}, (entasis_vector3_t){1, 0, 0}, 10);
        entasis_ray_hit_t hit = {0};
        ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&query, ray, NULL, &hit, &diag) == ENTASIS_STATUS_OK && fabsf(hit.t - 4.1f) < 1e-4f);
        ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, &diag) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(state.errors == 0 && state.rays > 0);
        state.check_access = 0;
        ENTASIS_TEST_CHECK(entasis_world_clear(&world, &diag) == ENTASIS_STATUS_OK && state.disposals == 1);
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diag) == ENTASIS_STATUS_OK && state.disposals == 1);
    }
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_registration_instances_and_access() == 0);
    ENTASIS_TEST_CHECK(test_validation_and_capacity() == 0);
    ENTASIS_TEST_CHECK(test_allocation_failure_and_release() == 0);
    ENTASIS_TEST_CHECK(test_independent_worlds_and_concurrent_contexts() == 0);
    ENTASIS_TEST_CHECK(test_compound_child_ownership_and_queries() == 0);
    puts("C_CUSTOM_SHAPES_OK groups=5 concurrent_queries=800");
    return 0;
}
