#ifndef ENTASIS_TEST_CUSTOM_SHAPE_FIXTURE_H
#define ENTASIS_TEST_CUSTOM_SHAPE_FIXTURE_H
#include "entasis.h"
#include "test_support.h"
#include "generated_layout_asserts.h"
_Static_assert(ENTASIS_SHAPE_BATCH_CONVEX == 0 && ENTASIS_SHAPE_BATCH_COMPOUND == 1 &&
                   ENTASIS_SHAPE_BATCH_HOMOGENEOUS_COMPOUND == 2,
               "shape batch wire values");
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
typedef struct payload_t
{
    entasis_shape_handle_t child;
    float radius;
    uint32_t tag;
} payload_t;

typedef struct shape_state_t
{
    entasis_world_t *world;
    entasis_shape_handle_t self;
    uint64_t alignment;
    unsigned bounds, inertia, rays, support, sweep_support, disposals, errors;
    int delegate_child, check_access, check_support, read_only, reentry;
    entasis_status_t fail_inertia, fail_ray, fail_dispose;
    int bad_hit;
} shape_state_t;

static entasis_rigid_pose_t pose_at(float x, float y, float z)
{
    const entasis_vector3_t p = {x, y, z};
    const entasis_quaternion_t q = {0, 0, 0, 1};
    return entasis_pose(p, q);
}
static entasis_world_description_t description(void)
{
    entasis_world_description_t d = entasis_world_description_default();
    d.threading.worker_count = 1;
    d.gravity = (entasis_vector3_t){0, 0, 0};
    d.capacity.bodies = 32;
    d.capacity.statics = 16;
    d.capacity.shapes_per_type = 4;
    d.capacity.constraints = 32;
    d.capacity.pairs = 128;
    d.capacity.broad_phase_candidates = 128;
    return d;
}
static void check_callback(shape_state_t *state, const void *raw, entasis_shape_access_t access)
{
    if (state->read_only)
        return;
    if (state->alignment && (uintptr_t)raw % (uintptr_t)state->alignment)
        ++state->errors;
    if (state->check_access)
    {
        entasis_custom_shape_view_t view = {0};
        if (entasis_shape_access_custom_data(access, state->self, &view) != ENTASIS_STATUS_OK ||
            view.data != raw || view.size != sizeof(payload_t) || view.alignment != state->alignment)
            ++state->errors;
        const payload_t *value = (const payload_t *)raw;
        if (entasis_shape_handle_is_valid(value->child))
        {
            if (entasis_shape_access_custom_data(access, value->child, &view) != ENTASIS_STATUS_INVALID_ARGUMENT || view.data != NULL)
                ++state->errors;
            entasis_vector3_t dir = {1, 0, 0}, support = {0};
            entasis_quaternion_t q = {0, 0, 0, 1};
            entasis_shape_bounds_t bounds = {0};
            if (entasis_shape_access_bounds(access, value->child, &q, &bounds) != ENTASIS_STATUS_OK || bounds.maximum_radius != value->radius)
                ++state->errors;
            if (entasis_shape_access_support(access, value->child, &dir, &support) != ENTASIS_STATUS_OK || support.x != value->radius)
                ++state->errors;
            if (entasis_shape_access_sweep_support(access, value->child, &dir, &support) != ENTASIS_STATUS_OK || support.x != value->radius)
                ++state->errors;
        }
    }
    if (state->reentry)
    {
        entasis_diagnostic_t d = {0};
        if (entasis_world_step(state->world, 1.0f / 64.0f, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
            ++state->errors;
        if (entasis_world_destroy(state->world, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
            ++state->errors;
    }
}
static entasis_status_t ENTASIS_CALL bounds_callback(void *ctx, const void *raw, const entasis_quaternion_t *q, entasis_shape_access_t access, entasis_shape_bounds_t *out)
{
    shape_state_t *s = (shape_state_t *)ctx;
    const payload_t *p = (const payload_t *)raw;
    check_callback(s, raw, access);
    if (!s->read_only)
        ++s->bounds;
    if (s->delegate_child)
        return entasis_shape_access_bounds(access, p->child, q, out);
    *out = (entasis_shape_bounds_t){{-p->radius, -p->radius, -p->radius}, {p->radius, p->radius, p->radius}, p->radius, 0};
    return ENTASIS_STATUS_OK;
}
static entasis_status_t ENTASIS_CALL inertia_callback(void *ctx, const void *raw, float mass, entasis_shape_access_t access, entasis_body_inertia_t *out)
{
    shape_state_t *s = (shape_state_t *)ctx;
    const payload_t *p = (const payload_t *)raw;
    check_callback(s, raw, access);
    if (!s->read_only)
        ++s->inertia;
    if (s->fail_inertia)
    {
        out->inverse_mass = 123;
        return s->fail_inertia;
    }
    if (s->check_support)
    {
        const entasis_vector3_t direction = {1, 0, 0};
        entasis_vector3_t support = {0};
        if (entasis_shape_access_support(access, s->self, &direction, &support) != ENTASIS_STATUS_OK || support.x != p->radius)
            ++s->errors;
        if (entasis_shape_access_sweep_support(access, s->self, &direction, &support) != ENTASIS_STATUS_OK || support.x != p->radius)
            ++s->errors;
    }
    if (s->delegate_child)
        return entasis_shape_access_inertia(access, p->child, mass, out);
    const entasis_sphere_t sphere = entasis_sphere(p->radius);
    return entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, mass, out);
}
static entasis_status_t ENTASIS_CALL ray_callback(void *ctx, const void *raw, const entasis_rigid_pose_t *pose, const entasis_ray_t *ray, entasis_shape_access_t access, entasis_shape_ray_hit_t *out)
{
    shape_state_t *s = (shape_state_t *)ctx;
    const payload_t *p = (const payload_t *)raw;
    check_callback(s, raw, access);
    if (!s->read_only)
        ++s->rays;
    if (s->fail_ray)
    {
        out->hit = ENTASIS_TRUE;
        out->t = 9;
        return s->fail_ray;
    }
    if (s->bad_hit)
    {
        out->hit = 2;
        return ENTASIS_STATUS_OK;
    }
    if (s->delegate_child)
        return entasis_shape_access_ray(access, p->child, pose, ray, out);
    const float x = ray->origin.x - pose->position.x, y = ray->origin.y - pose->position.y, z = ray->origin.z - pose->position.z;
    const float a = ray->direction.x * ray->direction.x + ray->direction.y * ray->direction.y + ray->direction.z * ray->direction.z;
    const float b = x * ray->direction.x + y * ray->direction.y + z * ray->direction.z;
    const float c = x * x + y * y + z * z - p->radius * p->radius;
    *out = (entasis_shape_ray_hit_t){0};
    out->child_index = -1;
    if (!(a > 0) || ray->maximum_t < 0)
        return ENTASIS_STATUS_INVALID_ARGUMENT;
    const float discr = b * b - a * c;
    if (discr < 0)
        return ENTASIS_STATUS_OK;
    const float root = (-b - sqrtf(discr)) / a;
    if (c > 0 && root < 0)
        return ENTASIS_STATUS_OK;
    const float t = fmaxf(0, root);
    if (t > ray->maximum_t)
        return ENTASIS_STATUS_OK;
    out->hit = ENTASIS_TRUE;
    out->t = t;
    out->child_index = 0;
    out->normal = (entasis_vector3_t){(x + ray->direction.x * t) / p->radius, (y + ray->direction.y * t) / p->radius, (z + ray->direction.z * t) / p->radius};
    return ENTASIS_STATUS_OK;
}
static entasis_status_t ENTASIS_CALL support_callback(void *ctx, const void *raw, const entasis_vector3_t *d, entasis_shape_access_t access, entasis_vector3_t *out)
{
    shape_state_t *s = (shape_state_t *)ctx;
    const payload_t *p = (const payload_t *)raw;
    check_callback(s, raw, access);
    if (!s->read_only)
        ++s->support;
    if (s->delegate_child)
        return entasis_shape_access_support(access, p->child, d, out);
    const float length = sqrtf(d->x * d->x + d->y * d->y + d->z * d->z);
    if (!(length > 0))
        return ENTASIS_STATUS_INVALID_ARGUMENT;
    const float scale = p->radius / length;
    *out = (entasis_vector3_t){d->x * scale, d->y * scale, d->z * scale};
    return ENTASIS_STATUS_OK;
}
static entasis_status_t ENTASIS_CALL sweep_support_callback(void *ctx, const void *raw, const entasis_vector3_t *d, entasis_shape_access_t access, entasis_vector3_t *out)
{
    shape_state_t *s = (shape_state_t *)ctx;
    if (!s->read_only)
        ++s->sweep_support;
    return support_callback(ctx, raw, d, access, out);
}
static entasis_status_t ENTASIS_CALL dispose_callback(void *ctx, void *raw, entasis_shape_access_t access)
{
    shape_state_t *s = (shape_state_t *)ctx;
    check_callback(s, raw, access);
    if (s->fail_dispose)
        return s->fail_dispose;
    ++s->disposals;
    return ENTASIS_STATUS_OK;
}
static entasis_custom_shape_registration_t registration(shape_state_t *state)
{
    entasis_custom_shape_registration_t r = entasis_custom_shape_registration_default();
    r.size = sizeof(payload_t);
    r.alignment = state->alignment;
    r.user_context = state;
    r.bounds = bounds_callback;
    r.inertia = inertia_callback;
    r.ray = ray_callback;
    r.support = support_callback;
    r.sweep_support = sweep_support_callback;
    r.dispose = dispose_callback;
    return r;
}
#endif
