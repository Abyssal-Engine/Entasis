#ifndef ENTASIS_TEST_CUSTOM_TASK_FIXTURE_H
#define ENTASIS_TEST_CUSTOM_TASK_FIXTURE_H
#include "custom_shape_fixture.h"
#if defined(_WIN32)
#include <windows.h>
typedef volatile LONG task_counter_t;
static void task_increment(task_counter_t *p)
{
    (void)InterlockedIncrement(p);
}
static inline unsigned task_count(task_counter_t *p)
{
    return (unsigned)InterlockedCompareExchange(p, 0, 0);
}
#else
#include <pthread.h>
#include <stdatomic.h>
typedef _Atomic unsigned task_counter_t;
static void task_increment(task_counter_t *p)
{
    (void)atomic_fetch_add_explicit(p, 1, memory_order_relaxed);
}
static inline unsigned task_count(task_counter_t *p)
{
    return atomic_load_explicit(p, memory_order_relaxed);
}
#endif

typedef struct task_state_t
{
    entasis_world_t *world;
    entasis_shape_handle_t custom;
    entasis_shape_type_id_t type;
    int tag, reentry, bad_scalar, bad_wide, bad_sweep;
    entasis_status_t scalar_status, wide_status, sweep_status, child_status;
    task_counter_t scalar, wide, lanes, sweep, child, errors, full, partial;
} task_state_t;

static float task_radius(const void *p, entasis_shape_type_id_t type)
{
    return type == ENTASIS_SHAPE_TYPE_SPHERE ? ((const entasis_sphere_t *)p)->radius : ((const payload_t *)p)->radius;
}
static void task_check(task_state_t *s, entasis_task_access_t access)
{
    entasis_shape_access_t shapes = {0};
    entasis_custom_shape_view_t view = {0};
    if (entasis_task_shape_access(access, &shapes) != ENTASIS_STATUS_OK ||
        entasis_shape_access_custom_data(shapes, s->custom, &view) != ENTASIS_STATUS_OK ||
        view.size != sizeof(payload_t))
        task_increment(&s->errors);
    if (s->reentry)
    {
        if (entasis_world_step(s->world, 1.0f / 60.0f, NULL) != ENTASIS_STATUS_INVALID_ARGUMENT ||
            entasis_world_destroy(s->world, NULL) != ENTASIS_STATUS_INVALID_ARGUMENT)
            task_increment(&s->errors);
    }
}
static entasis_status_t ENTASIS_CALL task_scalar(void *ctx, const entasis_collision_task_input_t *input,
                                                 entasis_task_access_t access, entasis_convex_contact_manifold_t *out)
{
    task_state_t *s = (task_state_t *)ctx;
    task_increment(&s->scalar);
    task_check(s, access);
    if (s->scalar_status)
        return s->scalar_status;
    entasis_convex_contact_manifold_t rejected = {0};
    if (entasis_task_collide_convex(access, input, &rejected) != ENTASIS_STATUS_INVALID_ARGUMENT)
        task_increment(&s->errors);
    entasis_collision_task_input_t pair = *input;
    const entasis_sphere_t a = entasis_sphere(task_radius(input->shape_a, input->type_a));
    const entasis_sphere_t b = entasis_sphere(task_radius(input->shape_b, input->type_b));
    pair.shape_a = &a;
    pair.shape_b = &b;
    pair.type_a = pair.type_b = ENTASIS_SHAPE_TYPE_SPHERE;
    const entasis_status_t status = entasis_task_collide_convex(access, &pair, out);
    for (int i = 0; i < out->count; ++i)
        out->contacts[i].feature_id = s->tag;
    if (s->bad_scalar == 1)
        out->count = 5;
    if (s->bad_scalar == 2)
        out->normal.x = NAN;
    return status;
}
static entasis_status_t ENTASIS_CALL task_wide(void *ctx, const entasis_collision_task_wide_input_t *input,
                                               entasis_task_access_t access, entasis_convex_manifold_wide_t *out)
{
    task_state_t *s = (task_state_t *)ctx;
    task_increment(&s->wide);
    task_check(s, access);
    if (s->wide_status)
        return s->wide_status;
    if (input->count == 0 || input->count > 8 || (uintptr_t)out % 32 || (uintptr_t)input->offset_b % 32 ||
        (uintptr_t)input->orientation_a % 32 || (uintptr_t)input->orientation_b % 32 ||
        (uintptr_t)input->speculative_margin % 32)
        return ENTASIS_STATUS_INVALID_DESCRIPTION;
    if (input->count == 8)
        task_increment(&s->full);
    else
        task_increment(&s->partial);
    for (unsigned i = 0; i < input->count; ++i)
    {
        task_increment(&s->lanes);
        const float x = input->offset_b->x.lanes[i], y = input->offset_b->y.lanes[i], z = input->offset_b->z.lanes[i];
        const float distance = sqrtf(x * x + y * y + z * z), a = task_radius(input->shape_a[i], input->type_a), b = task_radius(input->shape_b[i], input->type_b);
        const float depth = a + b - distance;
        if (depth < -input->speculative_margin->lanes[i])
            continue;
        const float nx = distance > 0 ? -x / distance : 0, ny = distance > 0 ? -y / distance : 1, nz = distance > 0 ? -z / distance : 0;
        out->normal.x.lanes[i] = nx;
        out->normal.y.lanes[i] = ny;
        out->normal.z.lanes[i] = nz;
        /* a sparse contact slot exercises mask-based compaction, not just slot 0 */
        entasis_convex_contact_wide_t *contact = &out->contacts[2];
        contact->exists.lanes[i] = -1;
        contact->depth.lanes[i] = depth;
        contact->feature_id.lanes[i] = s->tag;
        contact->offset_a.x.lanes[i] = (x + (b - a) * nx) * 0.5f;
        contact->offset_a.y.lanes[i] = (y + (b - a) * ny) * 0.5f;
        contact->offset_a.z.lanes[i] = (z + (b - a) * nz) * 0.5f;
    }
    if (s->bad_wide == 1)
        out->contacts[0].exists.lanes[0] = 1;
    if (s->bad_wide == 2)
        out->contacts[2].depth.lanes[0] = NAN;
    if (s->bad_wide == 3 && input->count < 8)
        out->contacts[0].exists.lanes[input->count] = -1;
    if (s->bad_wide == 4)
        out->contacts[2].offset_a.z.lanes[0] = INFINITY;
    if (s->bad_wide == 5)
        out->normal.y.lanes[0] = NAN;
    if (s->bad_wide == 6)
        out->contacts[1].depth.lanes[0] = NAN; /* absent contact stays ignored */
    return ENTASIS_STATUS_OK;
}
static entasis_status_t task_sweep_common(task_state_t *s, const entasis_sweep_task_input_t *input,
                                          entasis_task_access_t access, entasis_sweep_task_result_t *out, int child)
{
    task_check(s, access);
    if (child)
    {
        task_increment(&s->child);
        if (s->child_status)
            return s->child_status;
    }
    else
    {
        task_increment(&s->sweep);
        if (s->sweep_status)
            return s->sweep_status;
    }
    entasis_bool_t allow = 0;
    if (entasis_task_allow_child(access, 19, 3, 7, &allow) != ENTASIS_STATUS_OK)
        return ENTASIS_STATUS_INVALID_DESCRIPTION;
    if (!allow)
        return ENTASIS_STATUS_OK;
    entasis_sweep_task_input_t pair = *input;
    const entasis_sphere_t a = entasis_sphere(task_radius(input->shape_a, input->type_a));
    const entasis_sphere_t b = entasis_sphere(task_radius(input->shape_b, input->type_b));
    pair.shape_a = &a;
    pair.shape_b = &b;
    pair.type_a = pair.type_b = ENTASIS_SHAPE_TYPE_SPHERE;
    const entasis_status_t status = entasis_task_sweep_convex(access, &pair, out);
    if (out->hit)
    {
        out->child_a = 3;
        out->child_b = 7;
    }
    if (s->bad_sweep == 1)
        out->hit = 2;
    if (s->bad_sweep == 2)
    {
        out->hit = 1;
        out->t0 = -1;
    }
    if (s->bad_sweep == 3)
    {
        out->hit = 1;
        out->t0 = 0;
        out->t1 = NAN;
    }
    return status;
}
static entasis_status_t ENTASIS_CALL task_sweep(void *ctx, const entasis_sweep_task_input_t *input,
                                                entasis_task_access_t access, entasis_sweep_task_result_t *out)
{
    return task_sweep_common((task_state_t *)ctx, input, access, out, 0);
}
static entasis_status_t ENTASIS_CALL task_child(void *ctx, const entasis_sweep_task_input_t *input,
                                                entasis_task_access_t access, entasis_sweep_task_result_t *out)
{
    return task_sweep_common((task_state_t *)ctx, input, access, out, 1);
}

typedef struct task_fixture_t
{
    entasis_world_t world;
    shape_state_t shape;
    task_state_t task;
    entasis_shape_handle_t custom, sphere;
    entasis_shape_type_id_t type;
} task_fixture_t;
static inline int task_fixture_init(task_fixture_t *f, const entasis_allocator_t *allocator)
{
    entasis_world_description_t d = description();
    d.capacity.collision_child_pairs = 256;
    if (allocator)
        d.allocator = *allocator;
    f->shape.world = &f->world;
    f->shape.alignment = 16;
    f->task.world = &f->world;
    f->task.tag = 73;
    ENTASIS_TEST_CHECK(entasis_world_init(&f->world, &d, NULL) == ENTASIS_STATUS_OK);
    entasis_custom_shape_registration_t r = registration(&f->shape);
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&f->world, &r, &f->type, NULL) == ENTASIS_STATUS_OK);
    payload_t payload = {0};
    payload.radius = 1;
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&f->world, f->type, &payload, sizeof(payload), &f->custom, NULL) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(1);
    ENTASIS_TEST_CHECK(entasis_shape_add(&f->world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &f->sphere, NULL) == ENTASIS_STATUS_OK);
    f->shape.self = f->custom;
    f->task.custom = f->custom;
    f->task.type = f->type;
    return 0;
}
static entasis_collision_task_registration_t task_collision_registration(task_fixture_t *f)
{
    entasis_collision_task_registration_t r = entasis_collision_task_registration_default();
    r.shape_type_a = f->type;
    r.shape_type_b = ENTASIS_SHAPE_TYPE_SPHERE;
    r.user_context = &f->task;
    r.test = task_scalar;
    r.wide_test = task_wide;
    return r;
}
static entasis_sweep_task_registration_t task_sweep_registration(task_fixture_t *f)
{
    entasis_sweep_task_registration_t r = entasis_sweep_task_registration_default();
    r.shape_type_a = f->type;
    r.shape_type_b = ENTASIS_SHAPE_TYPE_SPHERE;
    r.user_context = &f->task;
    r.test = task_sweep;
    r.child_test = task_child;
    return r;
}
static int task_fixture_bind(task_fixture_t *f)
{
    int32_t id = -1;
    entasis_collision_task_registration_t c = task_collision_registration(f);
    entasis_sweep_task_registration_t s = task_sweep_registration(f);
    ENTASIS_TEST_CHECK(entasis_collision_task_register(&f->world, &c, &id, NULL) == ENTASIS_STATUS_OK && id >= 0);
    ENTASIS_TEST_CHECK(entasis_sweep_task_register(&f->world, &s, &id, NULL) == ENTASIS_STATUS_OK && id >= 0);
    c.test = NULL;
    c.wide_test = NULL;
    s.test = NULL;
    s.child_test = NULL; /* bridge must own copies */
    return 0;
}

#endif
