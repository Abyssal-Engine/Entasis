#define _POSIX_C_SOURCE 200809L
#include "entasis.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BODY_COUNT 10000
#define CREATE_BODY_COUNT 10000
#define CREATE_STATIC_COUNT 10000
#define QUERY_STATIC_COUNT 1024
#define QUERY_COUNT 256
#define EVENT_PAIR_COUNT 2048
#define STEP_REPEATS 64
#define MUTATION_REPEATS 8
#define QUERY_REPEATS 64

typedef struct result_t
{
    int64_t nanoseconds;
    int64_t operations;
    double checksum;
} result_t;

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(2);
}
static int64_t tick_ns(void)
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC_RAW, &t) != 0)
        fail("clock_gettime");
    return (int64_t)t.tv_sec * INT64_C(1000000000) + t.tv_nsec;
}
static void check(entasis_status_t status, const char *message)
{
    if (status != ENTASIS_STATUS_OK)
        fail(message);
}
static entasis_vector3_t v3(float x, float y, float z)
{
    entasis_vector3_t v = {x, y, z};
    return v;
}
static entasis_body_inertia_t shapeless_inertia(void)
{
    entasis_body_inertia_t v = {0};
    v.inverse_inertia_tensor.xx = 1.0f;
    v.inverse_inertia_tensor.yy = 1.0f;
    v.inverse_inertia_tensor.zz = 1.0f;
    v.inverse_mass = 1.0f;
    return v;
}
static entasis_world_description_t world_description(int bodies, int statics, int pairs)
{
    entasis_world_description_t d = entasis_world_description_default();
    d.gravity = v3(0, 0, 0);
    d.threading.worker_count = 1;
    d.capacity.bodies = bodies > 0 ? bodies : 1;
    d.capacity.statics = statics > 0 ? statics : 1;
    d.capacity.shapes_per_type = 8;
    d.capacity.broad_phase_candidates = bodies + statics > 1024 ? bodies + statics : 1024;
    d.capacity.pairs = pairs > 1024 ? pairs : 1024;
    d.capacity.inactive_pairs = pairs > 1024 ? pairs : 1024;
    return d;
}

static result_t step_10k(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diag = {0};
    entasis_world_description_t d = world_description(BODY_COUNT, 0, 0);
    check(entasis_world_init(&world, &d, &diag), "step world_init");
    entasis_body_description_t *descs = calloc(BODY_COUNT, sizeof(*descs));
    entasis_body_handle_t *handles = calloc(BODY_COUNT, sizeof(*handles));
    if (!descs || !handles)
        fail("step calloc");
    entasis_body_inertia_t inertia = shapeless_inertia();
    for (int i = 0; i < BODY_COUNT; ++i)
        descs[i] = entasis_body_shapeless(inertia, entasis_pose(v3((float)(i % 100), (float)(i / 100), 0), (entasis_quaternion_t){0, 0, 0, 1}), entasis_velocity(v3((float)((i % 7) + 1) * 0.01f, 0, 0), v3(0, 0, 0)), entasis_body_activity(-1.0f, UINT8_C(255)));
    uint64_t completed = 0;
    check(entasis_body_add_batch(&world, descs, BODY_COUNT, handles, &completed, &diag), "step add");
    if (completed != BODY_COUNT)
        fail("step add count");
    for (int i = 0; i < 4; ++i)
        check(entasis_world_step(&world, 1.0f / 60.0f, &diag), "step warmup");
    int64_t start = tick_ns();
    for (int i = 0; i < STEP_REPEATS; ++i)
        check(entasis_world_step(&world, 1.0f / 60.0f, &diag), "step measured");
    int64_t end = tick_ns();
    entasis_body_state_t state = {0};
    check(entasis_body_get(&world, handles[BODY_COUNT - 1], &state, &diag), "step checksum");
    free(descs);
    free(handles);
    check(entasis_world_destroy(&world, &diag), "step destroy");
    return (result_t){end - start, (int64_t)BODY_COUNT * STEP_REPEATS, state.pose.position.x};
}

static result_t create_10k(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diag = {0};
    entasis_world_description_t d = world_description(CREATE_BODY_COUNT, CREATE_STATIC_COUNT, 0);
    check(entasis_world_init(&world, &d, &diag), "create world_init");
    entasis_sphere_t sphere = entasis_sphere(0.25f);
    entasis_shape_handle_t shape = {0};
    check(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, &diag), "create shape");
    entasis_body_description_t *bdesc = calloc(CREATE_BODY_COUNT, sizeof(*bdesc));
    entasis_body_handle_t *bhandles = calloc(CREATE_BODY_COUNT, sizeof(*bhandles));
    entasis_static_description_t *sdesc = calloc(CREATE_STATIC_COUNT, sizeof(*sdesc));
    entasis_static_handle_t *shandles = calloc(CREATE_STATIC_COUNT, sizeof(*shandles));
    if (!bdesc || !bhandles || !sdesc || !shandles)
        fail("create calloc");
    entasis_body_inertia_t inertia = shapeless_inertia();
    entasis_quaternion_t q = {0, 0, 0, 1};
    for (int i = 0; i < CREATE_BODY_COUNT; ++i)
        bdesc[i] = entasis_body_shapeless(inertia, entasis_pose(v3((float)i, 10, 0), q), entasis_velocity(v3(0, 0, 0), v3(0, 0, 0)), entasis_body_activity(-1.0f, UINT8_C(255)));
    for (int i = 0; i < CREATE_STATIC_COUNT; ++i)
        sdesc[i] = entasis_static_body(shape, entasis_pose(v3((float)i * 2, -10, 0), q), entasis_ccd_discrete());
    uint64_t bw = 0, sw = 0;
    int64_t start = tick_ns();
    check(entasis_body_add_batch(&world, bdesc, CREATE_BODY_COUNT, bhandles, &bw, &diag), "create bodies");
    check(entasis_static_add_batch(&world, sdesc, CREATE_STATIC_COUNT, ENTASIS_AWAKENING_NONE, shandles, &sw, &diag), "create statics");
    int64_t end = tick_ns();
    if (bw != CREATE_BODY_COUNT || sw != CREATE_STATIC_COUNT)
        fail("create count");
    double checksum = (double)bhandles[CREATE_BODY_COUNT - 1].value + (double)shandles[CREATE_STATIC_COUNT - 1].value;
    free(bdesc);
    free(bhandles);
    free(sdesc);
    free(shandles);
    check(entasis_world_destroy(&world, &diag), "create destroy");
    return (result_t){end - start, CREATE_BODY_COUNT + CREATE_STATIC_COUNT, checksum};
}

static result_t mutate_10k(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diag = {0};
    entasis_world_description_t d = world_description(BODY_COUNT, 0, 0);
    check(entasis_world_init(&world, &d, &diag), "mutate world_init");
    entasis_body_description_t *base = calloc(BODY_COUNT, sizeof(*base)), *a = calloc(BODY_COUNT, sizeof(*a)), *b = calloc(BODY_COUNT, sizeof(*b));
    entasis_body_handle_t *handles = calloc(BODY_COUNT, sizeof(*handles));
    if (!base || !a || !b || !handles)
    {
        fail("mutate calloc");
    }
    entasis_body_inertia_t inertia = shapeless_inertia();
    entasis_quaternion_t q = {0, 0, 0, 1};
    for (int i = 0; i < BODY_COUNT; ++i)
    {
        base[i] = entasis_body_shapeless(inertia, entasis_pose(v3((float)i, 0, 0), q), entasis_velocity(v3(0, 0, 0), v3(0, 0, 0)), entasis_body_activity(-1.0f, UINT8_C(255)));
        a[i] = base[i];
        b[i] = base[i];
        a[i].pose.position.y = 1;
        b[i].pose.position.y = 2;
    }
    uint64_t done = 0;
    check(entasis_body_add_batch(&world, base, BODY_COUNT, handles, &done, &diag), "mutate add");
    if (done != BODY_COUNT)
        fail("mutate add count");
    int64_t start = tick_ns();
    for (int iteration = 0; iteration < MUTATION_REPEATS; ++iteration)
    {
        done = 0;
        check(entasis_body_apply_batch(&world, handles, (iteration & 1) ? b : a, BODY_COUNT, &done, &diag), "mutate apply");
        if (done != BODY_COUNT)
            fail("mutate count");
    }
    int64_t end = tick_ns();
    entasis_body_state_t state = {0};
    check(entasis_body_get(&world, handles[BODY_COUNT - 1], &state, &diag), "mutate checksum");
    free(base);
    free(a);
    free(b);
    free(handles);
    check(entasis_world_destroy(&world, &diag), "mutate destroy");
    return (result_t){end - start, (int64_t)BODY_COUNT * MUTATION_REPEATS, state.pose.position.y};
}

static result_t query_batch_run(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diag = {0};
    entasis_world_description_t d = world_description(0, QUERY_STATIC_COUNT, 0);
    check(entasis_world_init(&world, &d, &diag), "query world_init");
    entasis_sphere_t sphere = entasis_sphere(0.25f);
    entasis_shape_handle_t shape = {0};
    check(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, &diag), "query shape");
    entasis_static_description_t *descs = calloc(QUERY_STATIC_COUNT, sizeof(*descs));
    entasis_static_handle_t *handles = calloc(QUERY_STATIC_COUNT, sizeof(*handles));
    entasis_query_t *queries = calloc(QUERY_COUNT, sizeof(*queries));
    entasis_query_result_t *results = calloc(QUERY_COUNT, sizeof(*results));
    if (!descs || !handles || !queries || !results)
        fail("query calloc");
    entasis_quaternion_t q = {0, 0, 0, 1};
    for (int i = 0; i < QUERY_STATIC_COUNT; ++i)
    {
        descs[i] = entasis_static_body(
            shape,
            entasis_pose(v3((float)i * 2, 0, 0), q),
            entasis_ccd_discrete());
    }
    uint64_t done = 0;
    check(entasis_static_add_batch(
              &world, descs, QUERY_STATIC_COUNT, ENTASIS_AWAKENING_NONE,
              handles, &done, &diag),
          "query statics");
    if (done != QUERY_STATIC_COUNT)
        fail("query count");
    for (int i = 0; i < QUERY_COUNT; ++i)
        queries[i] = entasis_query_ray_closest(entasis_ray(v3(-1, (float)(i % 8) * 0.01f, 0), v3(1, 0, 0), (float)(QUERY_STATIC_COUNT * 2 + 2)), NULL);
    check(entasis_query_batch(&world, queries, QUERY_COUNT, results, QUERY_COUNT, NULL, &diag), "query warmup");
    int64_t start = tick_ns();
    for (int i = 0; i < QUERY_REPEATS; ++i)
        check(entasis_query_batch(&world, queries, QUERY_COUNT, results, QUERY_COUNT, NULL, &diag), "query measured");
    int64_t end = tick_ns();
    double checksum = 0.0;
    for (int i = 0; i < QUERY_COUNT; ++i)
    {
        if (results[i].hit)
        {
            entasis_ray_hit_t hit = {0};
            memcpy(&hit, results[i].payload, sizeof(hit));
            checksum += (double)hit.t;
        }
    }
    if (checksum <= 0.0)
        fail("query checksum");
    free(descs);
    free(handles);
    free(queries);
    free(results);
    check(entasis_world_destroy(&world, &diag), "query destroy");
    return (result_t){end - start, (int64_t)QUERY_COUNT * QUERY_REPEATS, checksum};
}

static result_t event_drain(void)
{
    entasis_world_t world = {0};
    entasis_diagnostic_t diag = {0};
    entasis_world_description_t d = world_description(EVENT_PAIR_COUNT, EVENT_PAIR_COUNT, EVENT_PAIR_COUNT * 2);
    check(entasis_world_init(&world, &d, &diag), "event world_init");
    entasis_sphere_t sphere = entasis_sphere(0.5f);
    entasis_shape_handle_t shape = {0};
    check(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, &diag), "event shape");
    entasis_body_inertia_t inertia = {0};
    check(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1.0f, &inertia), "event inertia");
    entasis_body_description_t *bdesc = calloc(EVENT_PAIR_COUNT, sizeof(*bdesc));
    entasis_body_handle_t *bhandles = calloc(EVENT_PAIR_COUNT, sizeof(*bhandles));
    entasis_static_description_t *sdesc = calloc(EVENT_PAIR_COUNT, sizeof(*sdesc));
    entasis_static_handle_t *shandles = calloc(EVENT_PAIR_COUNT, sizeof(*shandles));
    entasis_contact_event_t *events = calloc(EVENT_PAIR_COUNT * 2, sizeof(*events));
    if (!bdesc || !bhandles || !sdesc || !shandles || !events)
        fail("event calloc");
    entasis_quaternion_t q = {0, 0, 0, 1};
    for (int i = 0; i < EVENT_PAIR_COUNT; ++i)
    {
        entasis_rigid_pose_t p = entasis_pose(v3((float)i * 3, 0, 0), q);
        bdesc[i] = entasis_body_dynamic(shape, inertia, p, entasis_velocity(v3(0, 0, 0), v3(0, 0, 0)), entasis_body_activity(-1.0f, UINT8_C(255)));
        sdesc[i] = entasis_static_body(shape, p, entasis_ccd_discrete());
    }
    uint64_t bw = 0, sw = 0;
    check(entasis_body_add_batch(&world, bdesc, EVENT_PAIR_COUNT, bhandles, &bw, &diag), "event bodies");
    check(entasis_static_add_batch(&world, sdesc, EVENT_PAIR_COUNT, ENTASIS_AWAKENING_NONE, shandles, &sw, &diag), "event statics");
    if (bw != EVENT_PAIR_COUNT || sw != EVENT_PAIR_COUNT)
        fail("event count");
    entasis_contact_tracker_t tracker = {0};
    check(entasis_contact_tracker_init(&tracker, EVENT_PAIR_COUNT * 2, NULL, &diag), "event tracker init");
    check(entasis_contact_tracker_bind(&tracker, &world, NULL, &diag), "event bind");
    check(entasis_world_step(&world, 1.0f / 60.0f, &diag), "event step");
    uint64_t written = 0, required = 0;
    int64_t start = tick_ns();
    check(entasis_contact_events_drain(&tracker, events, EVENT_PAIR_COUNT * 2, &written, &required, &diag), "event drain");
    int64_t end = tick_ns();
    if (written != required || written < EVENT_PAIR_COUNT)
        fail("event written");
    check(entasis_contact_tracker_destroy(&tracker, &diag), "event tracker destroy");
    free(bdesc);
    free(bhandles);
    free(sdesc);
    free(shandles);
    free(events);
    check(entasis_world_destroy(&world, &diag), "event world destroy");
    return (result_t){end - start, (int64_t)written, (double)required};
}

int main(int argc, char **argv)
{
    if (argc != 2)
        fail("usage: c_abi <scenario>");
    result_t r;
    if (strcmp(argv[1], "step_10k") == 0)
        r = step_10k();
    else if (strcmp(argv[1], "create_10k") == 0)
        r = create_10k();
    else if (strcmp(argv[1], "mutate_10k") == 0)
        r = mutate_10k();
    else if (strcmp(argv[1], "query_batch") == 0)
        r = query_batch_run();
    else if (strcmp(argv[1], "event_drain") == 0)
        r = event_drain();
    else
        fail("unknown scenario");
    printf("%s,%" PRId64 ",%" PRId64 ",%.9f\n", argv[1], r.nanoseconds, r.operations, r.checksum);
    return 0;
}
