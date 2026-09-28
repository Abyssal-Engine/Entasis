#ifndef ENTASIS_CUSTOM_CONSTRAINTS_FIXTURE_H
#define ENTASIS_CUSTOM_CONSTRAINTS_FIXTURE_H
#include "entasis.h"
#include "generated_layout_asserts.h"
#include "test_support.h"
#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

typedef struct cc_state_t
{
    entasis_world_t *world;
    float bias;
    unsigned arity;
    entasis_status_t validation_error;
    int reentry;
    atomic_int validations, phases[4], full, partial, errors;
} cc_state_t;
static void cc_error(cc_state_t *s, int condition)
{
    if (!condition)
        (void)atomic_fetch_add_explicit(&s->errors, 1, memory_order_relaxed);
}
static entasis_status_t ENTASIS_CALL cc_validate(void *ctx, entasis_constraint_type_id_t id, const void *raw, uint32_t size)
{
    cc_state_t *s = (cc_state_t *)ctx;
    (void)atomic_fetch_add_explicit(&s->validations, 1, memory_order_relaxed);
    cc_error(s, id >= 56 && id < 64 && size == sizeof(float) && ((uintptr_t)raw % 4) == 0);
    if (s->reentry)
        cc_error(s, entasis_world_clear(s->world, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    if (s->validation_error != ENTASIS_STATUS_OK)
        return s->validation_error;
    const float target = *(const float *)raw;
    return isfinite(target) && fabsf(target) <= 10000 ? ENTASIS_STATUS_OK : ENTASIS_STATUS_INVALID_DESCRIPTION;
}
static void ENTASIS_CALL cc_kernel(void *ctx, const entasis_constraint_kernel_view_t *v)
{
    cc_state_t *s = (cc_state_t *)ctx;
    if (v->phase > 3 || v->body_count != s->arity)
    {
        cc_error(s, 0);
        return;
    }
    (void)atomic_fetch_add_explicit(&s->phases[v->phase], 1, memory_order_relaxed);
    cc_error(s, v->prestep_field_count == 1 && v->impulse_field_count == 1 && v->dt > 0 && v->inverse_dt > 0);
    cc_error(s, (uintptr_t)v->prestep % 32 == 0 && (uintptr_t)v->impulses % 32 == 0 && (uintptr_t)v->active_mask % 32 == 0);
    if (s->reentry)
        cc_error(s, entasis_world_clear(s->world, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    unsigned active = 0;
    for (unsigned lane = 0; lane < 8; ++lane)
    {
        cc_error(s, v->active_mask->lanes[lane] == -1 || v->active_mask->lanes[lane] == 0);
        if (v->active_mask->lanes[lane])
            ++active;
    }
    (void)atomic_fetch_add_explicit(active == 8 ? &s->full : &s->partial, 1, memory_order_relaxed);
    for (unsigned body = 0; body < v->body_count; ++body)
    {
        const entasis_constraint_body_view_t *b = &v->bodies[body];
        cc_error(s, b->inverse_mass != NULL && b->linear_velocity != NULL);
        cc_error(s, (uintptr_t)b->inverse_mass % 32 == 0 && (uintptr_t)b->linear_velocity % 32 == 0);
        if (v->phase == ENTASIS_CONSTRAINT_KERNEL_SOLVE)
        {
            cc_error(s, b->position == NULL && b->orientation == NULL && b->inverse_inertia == NULL && b->angular_velocity == NULL);
            for (unsigned lane = 0; lane < 8; ++lane)
                if (v->active_mask->lanes[lane] && b->inverse_mass->lanes[lane] > 0)
                {
                    b->linear_velocity->x.lanes[lane] = v->prestep[0].lanes[lane] + s->bias + (float)body;
                    v->impulses[0].lanes[lane] = v->prestep[0].lanes[lane];
                }
        }
        else
        {
            cc_error(s, b->position != NULL && b->orientation != NULL && b->inverse_inertia != NULL && b->angular_velocity != NULL);
            cc_error(s, (uintptr_t)b->position % 32 == 0 && (uintptr_t)b->orientation % 32 == 0 && (uintptr_t)b->inverse_inertia % 32 == 0 && (uintptr_t)b->angular_velocity % 32 == 0);
        }
    }
}
static inline int cc_init(entasis_world_t *world, cc_state_t *s, unsigned arity, unsigned workers,
                          unsigned substeps, unsigned fallback, const entasis_allocator_t *allocator)
{
    s->world = world;
    s->arity = arity;
    atomic_init(&s->validations, 0);
    atomic_init(&s->full, 0);
    atomic_init(&s->partial, 0);
    atomic_init(&s->errors, 0);
    for (unsigned i = 0; i < 4; ++i)
        atomic_init(&s->phases[i], 0);
    entasis_world_description_t d = entasis_world_description_default();
    d.gravity = (entasis_vector3_t){0, 0, 0};
    d.threading.worker_count = workers;
    d.solve.velocity_iterations = 4;
    d.solve.substeps = (int32_t)substeps;
    d.solve.fallback_batch_threshold = (int32_t)fallback;
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
    if (allocator)
        d.allocator = *allocator;
    ENTASIS_TEST_CHECK(entasis_world_init(world, &d, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
static entasis_custom_constraint_registration_t cc_registration(cc_state_t *s, int id)
{
    entasis_custom_constraint_registration_t r = entasis_custom_constraint_registration_default();
    r.type_id = id;
    r.body_count = s->arity;
    r.description_size = sizeof(float);
    r.prestep_bundle_size = 32;
    r.impulse_bundle_size = 32;
    r.user_context = s;
    r.validate = cc_validate;
    r.kernel = cc_kernel;
    for (unsigned i = 0; i < s->arity; ++i)
    {
        r.initial_access[i] = ENTASIS_BODY_ACCESS_ALL;
        r.solve_access[i] = ENTASIS_BODY_ACCESS_MASS | ENTASIS_BODY_ACCESS_LINEAR_VELOCITY;
    }
    return r;
}
static inline int cc_body(entasis_world_t *w, float x, int sleeping, entasis_body_handle_t *out)
{
    entasis_body_inertia_t inertia = {0};
    inertia.inverse_mass = 1;
    inertia.inverse_inertia_tensor.xx = 1;
    inertia.inverse_inertia_tensor.yy = 1;
    inertia.inverse_inertia_tensor.zz = 1;
    const entasis_body_description_t d = entasis_body_shapeless(inertia,
                                                                entasis_pose((entasis_vector3_t){x, 0, 0}, (entasis_quaternion_t){0, 0, 0, 1}),
                                                                entasis_velocity((entasis_vector3_t){0, 0, 0}, (entasis_vector3_t){0, 0, 0}),
                                                                entasis_body_activity(sleeping ? 10.0f : -1.0f, sleeping ? 1 : 255));
    ENTASIS_TEST_CHECK(entasis_body_add(w, &d, out, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
#endif
