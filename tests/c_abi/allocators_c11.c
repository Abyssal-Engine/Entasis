#include "entasis.h"
#include "entasis_cooking.h"
#include "test_support.h"
#include "extensions_fixture.h"
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

/* a test-only map checks exact callback metadata. Entasis stores no allocation map */
typedef struct allocation_t
{
    void *pointer;
    uint64_t size, alignment;
} allocation_t;
typedef struct allocator_state_t
{
    atomic_flag lock;
    allocation_t allocations[4096];
    uint64_t requests, fail_from, live, bytes, peak;
    unsigned errors;
} allocator_state_t;
static void acquire(allocator_state_t *s)
{
    while (atomic_flag_test_and_set_explicit(&s->lock, memory_order_acquire))
    {
    }
}
static void release(allocator_state_t *s)
{
    atomic_flag_clear_explicit(&s->lock, memory_order_release);
}
static void initialize(allocator_state_t *s)
{
    memset(s, 0, sizeof(*s));
    atomic_flag_clear(&s->lock);
}
static void *ENTASIS_CALL allocate(void *raw, uint64_t size, uint64_t alignment)
{
    allocator_state_t *s = (allocator_state_t *)raw;
    acquire(s);
    ++s->requests;
    if (!size || !alignment || (alignment & (alignment - 1)))
    {
        ++s->errors;
        release(s);
        return NULL;
    }
    if (s->fail_from && s->requests >= s->fail_from)
    {
        release(s);
        return NULL;
    }
    unsigned slot = 0;
    while (slot < 4096 && s->allocations[slot].pointer)
        ++slot;
    if (slot == 4096)
    {
        ++s->errors;
        release(s);
        return NULL;
    }
    size_t a = (size_t)alignment;
    if (a < sizeof(void *))
        a = sizeof(void *);
    const size_t rounded = ((size_t)size + a - 1) / a * a;
    void *p = ENTASIS_TEST_ALIGNED_ALLOC(a, rounded);
    if (p)
    {
        memset(p, 0xa5, (size_t)size);
        s->allocations[slot] = (allocation_t){p, size, alignment};
        ++s->live;
        s->bytes += size;
        if (s->bytes > s->peak)
            s->peak = s->bytes;
    }
    release(s);
    return p;
}
static void ENTASIS_CALL deallocate(void *raw, void *pointer, uint64_t size, uint64_t alignment)
{
    allocator_state_t *s = (allocator_state_t *)raw;
    acquire(s);
    unsigned slot = 0;
    while (slot < 4096 && s->allocations[slot].pointer != pointer)
        ++slot;
    if (!pointer || slot == 4096)
    {
        ++s->errors;
        release(s);
        return;
    }
    allocation_t old = s->allocations[slot];
    if (old.size != size || old.alignment != alignment)
        ++s->errors;
    s->bytes -= old.size;
    --s->live;
    s->allocations[slot] = (allocation_t){0};
    ENTASIS_TEST_ALIGNED_FREE(pointer);
    release(s);
}
static void *ENTASIS_CALL reallocate(void *raw, void *pointer, uint64_t old_size, uint64_t size, uint64_t alignment)
{
    void *next = allocate(raw, size, alignment);
    if (next)
    {
        memcpy(next, pointer, (size_t)(old_size < size ? old_size : size));
        deallocate(raw, pointer, old_size, alignment);
    }
    return next;
}
static entasis_allocator_t callbacks(allocator_state_t *s, int resize)
{
    const entasis_allocator_t a = {(uint32_t)sizeof(a), 1, s, allocate, resize ? reallocate : NULL, deallocate};
    return a;
}
static entasis_world_description_t allocation_description(allocator_state_t *s)
{
    entasis_world_description_t d = entasis_world_description_default();
    d.allocator = callbacks(s, 0);
    d.gravity = (entasis_vector3_t){0, 0, 0};
    d.threading.worker_count = 2;
    d.capacity.bodies = 4;
    d.capacity.statics = 4;
    d.capacity.inactive_body_sets = 2;
    d.capacity.shapes_per_type = 2;
    d.capacity.constraints = 16;
    d.capacity.initial_constraints_per_type_batch = 4;
    d.capacity.minimum_constraints_per_body = 4;
    d.capacity.broad_phase_candidates = 16;
    d.capacity.pairs = 16;
    d.capacity.collision_child_pairs = 0;
    d.capacity.inactive_pairs = 8;
    d.capacity.pending_pairs_per_worker = 8;
    return d;
}
static entasis_world_extensions_t extensions(void)
{
    entasis_world_extensions_t e = entasis_world_extensions_default();
    e.allocation_scope = ENTASIS_ALLOCATION_ALL_OWNED;
    return e;
}
static int clean(const allocator_state_t *s)
{
    return s->live == 0 && s->bytes == 0 && s->errors == 0;
}
static int test_invalid(void)
{
    allocator_state_t s;
    initialize(&s);
    entasis_allocator_t a = callbacks(&s, 0);
    entasis_buffer_pool_t p = {0};
    entasis_thread_pool_t t = {0};
    entasis_cooking_context_t c = {0};
    entasis_world_t w = {0};
    entasis_world_description_t d = allocation_description(&s);
    entasis_world_extensions_t e = extensions();
    e.allocation_scope = 2;
    ENTASIS_TEST_CHECK(entasis_buffer_pool_init_extended(&p, 1024, 1, &a, 2, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_thread_pool_init_extended(&t, 2, 1024, &a, 2, NULL) == ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_cooking_context_init_extended(&c, 1024, 1, &a, 2, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&w, &d, &e, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    a.deallocate = NULL;
    ENTASIS_TEST_CHECK(entasis_buffer_pool_init_extended(&p, 1024, 1, &a, 1, NULL) != ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_thread_pool_init_extended(&t, 2, 1024, &a, 1, NULL) != ENTASIS_DISPATCHER_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooking_context_init_extended(&c, 1024, 1, &a, 1, NULL) != ENTASIS_STATUS_OK);
    d.allocator = a;
    e.allocation_scope = 1;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&w, &d, &e, NULL) != ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(s.requests == 0 && clean(&s) && !w.opaque && !p.opaque && !c.opaque && !t.opaque);
    return 0;
}
/* every initialization request can fail permanently, including partial worker creation */
static int test_initialization_failures(void)
{
    for (unsigned kind = 0; kind < 4; ++kind)
    {
        uint64_t requests = 0;
        for (uint64_t failure = 0; failure <= requests; ++failure)
        {
            allocator_state_t s;
            initialize(&s);
            s.fail_from = failure;
            entasis_allocator_t a = callbacks(&s, (int)(failure & 1));
            entasis_buffer_pool_t p = {0};
            entasis_thread_pool_t t = {0};
            entasis_cooking_context_t c = {0};
            entasis_world_t w = {0};
            entasis_world_description_t d = allocation_description(&s);
            d.allocator = a;
            entasis_world_extensions_t e = extensions();
            uint32_t status = 0;
            switch (kind)
            {
            case 0:
                status = entasis_buffer_pool_init_extended(&p, 1024, 1, &a, 1, NULL);
                break;
            case 1:
                status = entasis_thread_pool_init_extended(&t, 3, 1024, &a, 1, NULL);
                break;
            case 2:
                status = entasis_cooking_context_init_extended(&c, 1024, 1, &a, 1, NULL);
                break;
            case 3:
                status = entasis_world_init_extended(&w, &d, &e, NULL);
                break;
            }
            if (failure == 0)
            {
                ENTASIS_TEST_CHECK(status == 0);
                requests = s.requests;
                s.fail_from = s.requests + 1;
                a.allocate = NULL;
                a.deallocate = NULL;
                d.allocator = a;
                switch (kind)
                {
                case 0:
                    ENTASIS_TEST_CHECK(entasis_buffer_pool_clear(&p, NULL) == ENTASIS_STATUS_OK);
                    status = entasis_buffer_pool_destroy(&p, NULL);
                    break;
                case 1:
                    status = entasis_thread_pool_destroy(&t, NULL);
                    break;
                case 2:
                    ENTASIS_TEST_CHECK(entasis_cooking_context_clear(&c, NULL) == ENTASIS_STATUS_OK);
                    status = entasis_cooking_context_destroy(&c, NULL);
                    break;
                case 3:
                    status = entasis_world_destroy(&w, NULL);
                    break;
                }
                ENTASIS_TEST_CHECK(status == 0 && s.requests == requests);
            }
            else
            {
                if (kind == 1)
                    ENTASIS_TEST_CHECK(status == ENTASIS_DISPATCHER_STATUS_CAPACITY_MISSING);
                else
                    ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_CAPACITY_MISSING);
            }
            if (!clean(&s))
                fprintf(stderr, "allocation initialization kind=%u fail=%llu requests=%llu live=%llu errors=%u\n", kind, (unsigned long long)failure, (unsigned long long)s.requests, (unsigned long long)s.live, s.errors);
            ENTASIS_TEST_CHECK(clean(&s) && !p.opaque && !t.opaque && !c.opaque && !w.opaque);
        }
        printf("allocator initialization kind=%u failpoints=%llu\n", kind, (unsigned long long)requests);
    }
    return 0;
}
static entasis_triangle_t triangle(void)
{
    return entasis_triangle((entasis_vector3_t){-1, 0, -1}, (entasis_vector3_t){1, 0, -1}, (entasis_vector3_t){0, 0, 1});
}
/* fault the operation, then keep refusing allocation through every destructor */
static int test_operation_failures(void)
{
    static entasis_triangle_t triangles[8192];
    for (unsigned i = 0; i < 8192; ++i)
    {
        triangles[i] = triangle();
        triangles[i].a.x += (float)i * 2;
        triangles[i].b.x += (float)i * 2;
        triangles[i].c.x += (float)i * 2;
    }
    for (unsigned operation = 0; operation < 4; ++operation)
    {
        uint64_t requests = 0;
        for (uint64_t failure = 0; failure <= requests; ++failure)
        {
            allocator_state_t s, source;
            initialize(&s);
            initialize(&source);
            entasis_allocator_t a = callbacks(&s, 0), ca = callbacks(&source, 1);
            entasis_world_t w = {0};
            entasis_world_description_t d = allocation_description(&s);
            d.threading.worker_count = 1;
            entasis_world_extensions_t e = extensions();
            entasis_cooking_context_t c = {0};
            entasis_cooked_mesh_t m = {0};
            entasis_query_context_t q = {0};
            entasis_shape_handle_t shape = entasis_shape_handle_invalid();
            entasis_query_context_description_t qd = entasis_query_context_description_default();
            qd.pair_capacity = 8;
            qd.child_capacity = 16;
            qd.traversal_maximum_power = 8;
            qd.traversal_slots_per_power = 2;
            ENTASIS_TEST_CHECK(entasis_world_init_extended(&w, &d, &e, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_cooking_context_init_extended(&c, 1024, 1, operation == 0 ? &a : &ca, 1, NULL) == ENTASIS_STATUS_OK);
            if (operation != 0)
                ENTASIS_TEST_CHECK(entasis_cook_mesh(&c, triangles, 8192, (entasis_vector3_t){1, 1, 1}, &m, NULL) == ENTASIS_STATUS_OK);
            const uint64_t before = s.requests;
            s.fail_from = failure ? before + failure : 0;
            entasis_status_t status;
            if (operation == 0)
                status = entasis_cook_mesh(&c, triangles, 8192, (entasis_vector3_t){1, 1, 1}, &m, NULL);
            else if (operation == 1)
                status = entasis_cooked_mesh_import(&w, &m, &shape, NULL);
            else if (operation == 2)
                status = entasis_query_context_init(&q, &w, &qd, NULL, NULL);
            else
            {
                status = ENTASIS_STATUS_OK;
                entasis_sphere_t sphere = entasis_sphere(1);
                for (unsigned i = 0; i < 20000 && status == ENTASIS_STATUS_OK; ++i)
                    status = entasis_shape_add(&w, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL);
            }
            if (failure == 0)
            {
                ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
                requests = s.requests - before;
            }
            else
                ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_CAPACITY_MISSING);
            const uint64_t stop = s.requests;
            s.fail_from = stop + 1;
            source.fail_from = source.requests + 1;
            if (q.opaque)
                ENTASIS_TEST_CHECK(entasis_query_context_destroy(&q, NULL) == ENTASIS_STATUS_OK);
            if (m.opaque)
                ENTASIS_TEST_CHECK(entasis_cooked_mesh_destroy(&m, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_cooking_context_destroy(&c, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK);
            if (!clean(&s))
                fprintf(stderr, "allocation operation=%u fail=%llu requests=%llu live=%llu errors=%u\n", operation, (unsigned long long)failure, (unsigned long long)s.requests, (unsigned long long)s.live, s.errors);
            ENTASIS_TEST_CHECK(s.requests == stop && clean(&s) && clean(&source));
        }
        printf("allocator operation=%u failpoints=%llu\n", operation, (unsigned long long)requests);
    }
    return 0;
}
static int test_borrowed_owners(void)
{
    allocator_state_t root, storage, source;
    initialize(&root);
    initialize(&storage);
    initialize(&source);
    entasis_allocator_t pa = callbacks(&storage, 1), ca = callbacks(&source, 0);
    entasis_buffer_pool_t p = {0};
    entasis_thread_pool_t tp = {0};
    entasis_cooking_context_t c = {0};
    entasis_cooked_mesh_t m = {0};
    ENTASIS_TEST_CHECK(entasis_buffer_pool_init_extended(&p, 1024, 1, &pa, 1, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_thread_pool_init_extended(&tp, 2, 1024, &pa, 1, NULL) == ENTASIS_DISPATCHER_STATUS_OK);
    entasis_dispatcher_interface_t dispatcher = {0};
    ENTASIS_TEST_CHECK(entasis_dispatcher_from_thread_pool(&tp, &dispatcher) == ENTASIS_DISPATCHER_STATUS_OK);
    entasis_world_description_t d = allocation_description(&root);
    d.threading.external_dispatcher = &dispatcher;
    entasis_world_extensions_t e = extensions();
    entasis_world_t w = {0};
    ENTASIS_TEST_CHECK(entasis_world_init_with_pool_extended(&w, &d, &p, &e, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooking_context_init_extended(&c, 1024, 1, &ca, 1, NULL) == ENTASIS_STATUS_OK);
    entasis_triangle_t tri = triangle();
    ENTASIS_TEST_CHECK(entasis_cook_mesh(&c, &tri, 1, (entasis_vector3_t){1, 1, 1}, &m, NULL) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t shape;
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_import(&w, &m, &shape, NULL) == ENTASIS_STATUS_OK);
    source.fail_from = source.requests + 1;
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_destroy(&m, NULL) == ENTASIS_STATUS_OK && entasis_cooking_context_destroy(&c, NULL) == ENTASIS_STATUS_OK && clean(&source));
    entasis_shape_bounds_t bounds;
    ENTASIS_TEST_CHECK(entasis_shape_bounds(&w, shape, (entasis_quaternion_t){0, 0, 0, 1}, &bounds, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(bounds.min.x == -1 && bounds.max.x == 1);
    root.fail_from = root.requests + 1;
    storage.fail_from = storage.requests + 1;
    const uint64_t root_requests = root.requests, storage_requests = storage.requests;
    ENTASIS_TEST_CHECK(entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK && clean(&root));
    ENTASIS_TEST_CHECK(p.opaque && tp.opaque && storage.live > 0);
    ENTASIS_TEST_CHECK(entasis_thread_pool_destroy(&tp, NULL) == ENTASIS_DISPATCHER_STATUS_OK && entasis_buffer_pool_destroy(&p, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(clean(&storage) && root.requests == root_requests && storage.requests == storage_requests);
    return 0;
}
/* reserve is a capacity lower bound: equal/smaller hints must reuse live scratch,
   including while the application's allocator refuses every new request */
static int test_query_reserve_reuse(void)
{
    allocator_state_t s;
    initialize(&s);
    entasis_world_description_t d = allocation_description(&s);
    d.threading.worker_count = 1;
    entasis_world_extensions_t e = extensions();
    entasis_world_t w = {0};
    entasis_query_context_t q = {0};
    entasis_query_context_description_t initial = entasis_query_context_description_default();
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&w, &d, &e, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_init(&q, &w, &initial, NULL, NULL) == ENTASIS_STATUS_OK);
    const uint64_t requests = s.requests, bytes = s.bytes, live = s.live;
    ENTASIS_TEST_CHECK(entasis_query_context_reserve(&q, &initial, NULL) == ENTASIS_STATUS_OK);
    printf("query reserve equal requests=%llu retained_delta=%lld\n",
           (unsigned long long)(s.requests - requests), (long long)s.bytes - (long long)bytes);
    ENTASIS_TEST_CHECK(s.requests == requests && s.bytes == bytes && s.live == live);
    entasis_query_context_description_t smaller = initial;
    smaller.pair_capacity = 1;
    smaller.child_capacity = 1;
    smaller.traversal_maximum_power = 8;
    smaller.traversal_slots_per_power = 1;
    s.fail_from = s.requests + 1;
    for (unsigned i = 0; i < 4; ++i)
    {
        ENTASIS_TEST_CHECK(entasis_query_context_reserve(&q, &smaller, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_query_context_reserve(&q, &initial, NULL) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(s.requests == requests && s.bytes == bytes && s.live == live);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&q, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK && clean(&s));
    return 0;
}

/* replacement is transactional for the bound arrays. fault every real growth
   allocation and keep refusing allocations through a query and destruction */
static int test_query_reserve_growth_failures(void)
{
    for (unsigned dimension = 0; dimension < 4; ++dimension)
    {
        uint64_t requests = 0;
        for (uint64_t failure = 0; failure <= requests; ++failure)
        {
            allocator_state_t s;
            initialize(&s);
            entasis_world_description_t d = allocation_description(&s);
            d.threading.worker_count = 1;
            entasis_world_extensions_t e = extensions();
            entasis_world_t w = {0};
            entasis_query_context_t q = {0};
            entasis_query_context_description_t initial = entasis_query_context_description_default();
            initial.pair_capacity = 4;
            initial.child_capacity = 16;
            initial.traversal_maximum_power = 8;
            initial.traversal_slots_per_power = 2;
            ENTASIS_TEST_CHECK(entasis_world_init_extended(&w, &d, &e, NULL) == ENTASIS_STATUS_OK);
            entasis_shape_handle_t shape;
            const entasis_sphere_t sphere = entasis_sphere(1);
            ENTASIS_TEST_CHECK(entasis_shape_add(&w, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_query_context_init(&q, &w, &initial, NULL, NULL) == ENTASIS_STATUS_OK);
            entasis_query_context_description_t grown = initial;
            if (dimension == 0)
                grown.pair_capacity = 4096;
            if (dimension == 1)
                grown.child_capacity = 512;
            if (dimension == 2)
                grown.child_capacity = 17000; /* split to combined. u16 to u32 */
            if (dimension == 3)
            {
                grown.traversal_maximum_power = 16;
                grown.traversal_slots_per_power = 8;
            }
            const uint64_t before = s.requests;
            s.fail_from = failure ? before + failure : 0;
            const entasis_status_t status = entasis_query_context_reserve(&q, &grown, NULL);
            if (!failure)
            {
                ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
                requests = s.requests - before;
            }
            else
                ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_CAPACITY_MISSING);
            s.fail_from = s.requests + 1;
            const uint64_t after = s.requests;
            ENTASIS_TEST_CHECK(entasis_query_context_reserve(&q, &initial, NULL) == ENTASIS_STATUS_OK);
            entasis_contact_manifold_t manifold = {0};
            const entasis_rigid_pose_t pose = entasis_pose((entasis_vector3_t){0, 0, 0}, (entasis_quaternion_t){0, 0, 0, 1});
            ENTASIS_TEST_CHECK(entasis_query_context_collision_query(&q, shape, pose, shape, pose, 0, &manifold, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(manifold.convex.count > 0);
            ENTASIS_TEST_CHECK(entasis_query_context_destroy(&q, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK && clean(&s) && s.requests == after);
        }
        printf("query reserve growth dimension=%u failpoints=%llu\n", dimension, (unsigned long long)requests);
    }
    return 0;
}

static void ENTASIS_CALL restitution_combine_allocated(void *raw,
                                                       entasis_collidable_reference_t a, entasis_collidable_reference_t b,
                                                       const entasis_restitution_settings_t *sa, const entasis_restitution_settings_t *sb,
                                                       entasis_restitution_settings_t *out)
{
    (void)raw;
    (void)a;
    (void)b;
    (void)sa;
    (void)sb;
    (void)out;
}

static int test_restitution_allocation_failures(void)
{
    for (unsigned direct_destroy = 0; direct_destroy < 2; ++direct_destroy)
    {
        uint64_t requests = 0;
        for (uint64_t failure = 0; failure <= requests; ++failure)
        {
            allocator_state_t s;
            initialize(&s);
            entasis_world_description_t d = allocation_description(&s);
            d.threading.worker_count = 1;
            entasis_world_extensions_t e = extensions();
            entasis_world_t world = {0};
            ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &d, &e, NULL) == ENTASIS_STATUS_OK);
            entasis_restitution_configuration_t config = entasis_restitution_configuration_default();
            config.fallback.coefficient = 0.5f;
            config.combine = restitution_combine_allocated;
            const uint64_t before = s.requests;
            s.fail_from = failure ? before + failure : 0;
            const entasis_status_t status = entasis_world_enable_restitution(&world, &config, NULL);
            if (!failure)
            {
                ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
                requests = s.requests - before;
            }
            else
                ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_CAPACITY_MISSING);
            s.fail_from = s.requests + 1;
            const uint64_t final_requests = s.requests;
            if (!direct_destroy)
                ENTASIS_TEST_CHECK(entasis_world_disable_restitution(&world, NULL) == (failure ? ENTASIS_STATUS_NOT_FOUND : ENTASIS_STATUS_OK));
            ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(clean(&s) && s.requests == final_requests);
        }
        printf("restitution allocation destroy=%u failpoints=%llu\n", direct_destroy, (unsigned long long)requests);
    }
    return 0;
}

static int test_distance_reserve_failures(void)
{
    for (unsigned owner = 0; owner < 2u; ++owner)
    {
        for (unsigned growth = 0; growth < 2u; ++growth)
        {
            uint64_t requests = 0;
            for (uint64_t failure = 0; failure <= requests; ++failure)
            {
                allocator_state_t state;
                initialize(&state);
                entasis_world_t world = {0};
                entasis_query_context_t query = {0};
                entasis_world_description_t description = allocation_description(&state);
                description.threading.worker_count = 1;
                entasis_world_extensions_t extension = extensions();
                ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &description, &extension, NULL) == ENTASIS_STATUS_OK);
                entasis_shape_handle_t sphere;
                const entasis_sphere_t geometry = entasis_sphere(1);
                const entasis_rigid_pose_t pose = entasis_pose((entasis_vector3_t){0, 0, 0}, (entasis_quaternion_t){0, 0, 0, 1});
                const entasis_rigid_pose_t other = entasis_pose((entasis_vector3_t){1, 0, 0}, (entasis_quaternion_t){0, 0, 0, 1});
                ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &geometry, &sphere, NULL) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &world, NULL, NULL, NULL) == ENTASIS_STATUS_OK);
                const entasis_distance_query_capacity_t initial = {16, 32, 48};
                const entasis_distance_query_capacity_t large = {512, 1024, 1536};
                if (growth)
                {
                    const entasis_status_t status = owner ? entasis_query_context_distance_query_reserve(&query, &initial, NULL) : entasis_distance_query_reserve(&world, &initial, NULL);
                    ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
                }
                const uint64_t before = state.requests;
                state.fail_from = failure ? before + failure : 0;
                const entasis_distance_query_capacity_t *capacity = growth ? &large : &initial;
                const entasis_status_t status = owner ? entasis_query_context_distance_query_reserve(&query, capacity, NULL) : entasis_distance_query_reserve(&world, capacity, NULL);
                if (!failure)
                {
                    ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
                    requests = state.requests - before;
                }
                else
                    ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_CAPACITY_MISSING);
                state.fail_from = state.requests + 1;
                const uint64_t after = state.requests;
                entasis_shape_distance_result_t result = {0};
                if (growth || !failure)
                {
                    for (unsigned repeat = 0; repeat < 16u; ++repeat)
                    {
                        const entasis_status_t reserve = owner ? entasis_query_context_distance_query_reserve(&query, &initial, NULL) : entasis_distance_query_reserve(&world, &initial, NULL);
                        ENTASIS_TEST_CHECK(reserve == ENTASIS_STATUS_OK);
                        const entasis_status_t queried = owner ? entasis_query_context_shape_penetration(&query, sphere, pose, sphere, other, NULL, &result, NULL) : entasis_shape_penetration(&world, sphere, pose, sphere, other, NULL, &result, NULL);
                        ENTASIS_TEST_CHECK(queried == ENTASIS_STATUS_OK && result.geometry.depth == 1);
                    }
                }
                else
                {
                    const entasis_status_t queried = owner ? entasis_query_context_shape_penetration(&query, sphere, pose, sphere, other, NULL, &result, NULL) : entasis_shape_penetration(&world, sphere, pose, sphere, other, NULL, &result, NULL);
                    ENTASIS_TEST_CHECK(queried == ENTASIS_STATUS_CAPACITY_MISSING && result.geometry.state == ENTASIS_DISTANCE_UNRESOLVED);
                }
                ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, NULL) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(state.requests == after && clean(&state));
            }
            printf("distance reserve owner=%u growth=%u failpoints=%llu\n", owner, growth, (unsigned long long)requests);
        }
    }
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_distance_reserve_failures() == 0);
    ENTASIS_TEST_CHECK(test_restitution_allocation_failures() == 0);
    ENTASIS_TEST_CHECK(test_query_reserve_reuse() == 0);
    ENTASIS_TEST_CHECK(test_query_reserve_growth_failures() == 0);
    ENTASIS_TEST_CHECK(test_invalid() == 0);
    ENTASIS_TEST_CHECK(test_initialization_failures() == 0);
    ENTASIS_TEST_CHECK(test_operation_failures() == 0);
    ENTASIS_TEST_CHECK(test_borrowed_owners() == 0);
    for (unsigned step = 0; step < 2; ++step)
    {
        allocator_state_t s;
        initialize(&s);
        entasis_allocator_t a = callbacks(&s, (int)step);
        ENTASIS_TEST_CHECK(ef_run(ENTASIS_ALLOCATION_ALL_OWNED, step, &a, 0) == 0 && clean(&s));
    }
    puts("ALL_OWNED_C_OK");
    return 0;
}
