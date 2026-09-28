#ifndef ENTASIS_TEST_COMPOUND_TASK_FIXTURE_H
#define ENTASIS_TEST_COMPOUND_TASK_FIXTURE_H
#include "custom_task_fixture.h"
#include "entasis_cooking.h"

_Static_assert(sizeof(entasis_compound_task_registration_t) == 28, "compound descriptor size");
_Static_assert(ENTASIS_COLLISION_TASK_CONVEX_COMPOUND == 1 && ENTASIS_COLLISION_TASK_COMPOUND_PAIR == 2,
               "native compound task kinds");
_Static_assert(ENTASIS_COLLISION_TASK_SUBTASK_GENERATOR == 4 && ENTASIS_COLLISION_TASK_CHILD_ORDER == 8 &&
                   ENTASIS_COLLISION_TASK_MESH_REDUCTION == 16,
               "native compound capability bits");

static entasis_status_t ENTASIS_CALL compound_triangle_scalar(void *raw,
                                                              const entasis_collision_task_input_t *input, entasis_task_access_t access,
                                                              entasis_convex_contact_manifold_t *out)
{
    task_state_t *state = (task_state_t *)raw;
    task_increment(&state->scalar);
    task_check(state, access);
    if (state->scalar_status)
        return state->scalar_status;
    entasis_collision_task_input_t pair = *input;
    const int triangle_first = input->type_a == ENTASIS_SHAPE_TYPE_TRIANGLE;
    const payload_t *payload = (const payload_t *)(triangle_first ? input->shape_b : input->shape_a);
    const entasis_sphere_t sphere = entasis_sphere(payload->radius);
    if (triangle_first)
    {
        pair.shape_b = &sphere;
        pair.type_b = ENTASIS_SHAPE_TYPE_SPHERE;
    }
    else
    {
        pair.shape_a = &sphere;
        pair.type_a = ENTASIS_SHAPE_TYPE_SPHERE;
    }
    return entasis_task_collide_convex(access, &pair, out);
}

static entasis_status_t ENTASIS_CALL compound_triangle_wide(void *raw,
                                                            const entasis_collision_task_wide_input_t *input, entasis_task_access_t access,
                                                            entasis_convex_manifold_wide_t *out)
{
    task_state_t *state = (task_state_t *)raw;
    task_increment(&state->wide);
    task_check(state, access);
    if (state->wide_status)
        return state->wide_status;
    if (input->count == 8)
        task_increment(&state->full);
    else
        task_increment(&state->partial);
    /* fixture implementation reuses scalar native triangle geometry. the C
       adapter still invokes this callback once per bundle, not once per lane */
    for (unsigned lane = 0; lane < input->count; ++lane)
    {
        entasis_collision_task_input_t pair = {0};
        pair.shape_a = input->shape_a[lane];
        pair.shape_b = input->shape_b[lane];
        pair.type_a = input->type_a;
        pair.type_b = input->type_b;
        pair.pose_a = pose_at(0, 0, 0);
        pair.pose_b = pose_at(input->offset_b->x.lanes[lane], input->offset_b->y.lanes[lane], input->offset_b->z.lanes[lane]);
        pair.pose_a.orientation = (entasis_quaternion_t){input->orientation_a->x.lanes[lane], input->orientation_a->y.lanes[lane], input->orientation_a->z.lanes[lane], input->orientation_a->w.lanes[lane]};
        pair.pose_b.orientation = (entasis_quaternion_t){input->orientation_b->x.lanes[lane], input->orientation_b->y.lanes[lane], input->orientation_b->z.lanes[lane], input->orientation_b->w.lanes[lane]};
        pair.speculative_margin = input->speculative_margin->lanes[lane];
        entasis_convex_contact_manifold_t result = {0};
        const entasis_status_t status = compound_triangle_scalar(raw, &pair, access, &result);
        if (status != ENTASIS_STATUS_OK)
            return status;
        if (result.count > 1)
            return ENTASIS_STATUS_INVALID_DESCRIPTION;
        if (result.count == 0)
            continue;
        out->normal.x.lanes[lane] = result.normal.x;
        out->normal.y.lanes[lane] = result.normal.y;
        out->normal.z.lanes[lane] = result.normal.z;
        entasis_convex_contact_wide_t *contact = &out->contacts[0];
        contact->exists.lanes[lane] = -1;
        contact->depth.lanes[lane] = result.contacts[0].depth;
        contact->feature_id.lanes[lane] = result.contacts[0].feature_id;
        contact->offset_a.x.lanes[lane] = result.contacts[0].offset.x;
        contact->offset_a.y.lanes[lane] = result.contacts[0].offset.y;
        contact->offset_a.z.lanes[lane] = result.contacts[0].offset.z;
    }
    return ENTASIS_STATUS_OK;
}

static int compound_targets_order(task_fixture_t *f, entasis_shape_handle_t targets[3], int triangle_first)
{
    entasis_collision_task_registration_t triangle = task_collision_registration(f);
    if (triangle_first)
    {
        triangle.shape_type_a = ENTASIS_SHAPE_TYPE_TRIANGLE;
        triangle.shape_type_b = f->type;
    }
    else
        triangle.shape_type_b = ENTASIS_SHAPE_TYPE_TRIANGLE;
    triangle.test = compound_triangle_scalar;
    triangle.wide_test = compound_triangle_wide;
    int32_t id = -1;
    ENTASIS_TEST_CHECK(entasis_collision_task_register(&f->world, &triangle, &id, NULL) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 3; ++i)
    {
        entasis_compound_task_registration_t r = entasis_compound_task_registration_default();
        r.shape_type_a = f->type;
        r.shape_type_b = ENTASIS_SHAPE_TYPE_COMPOUND + (int32_t)i;
        if (i == 2)
            r.capabilities |= ENTASIS_COLLISION_TASK_MESH_REDUCTION;
        ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&f->world, &r, &id, NULL) == ENTASIS_STATUS_OK);
    }
    const entasis_compound_child_t children[2] = {entasis_compound_child(f->sphere, pose_at(-1, 0, 0)), entasis_compound_child(f->sphere, pose_at(1, 0, 0))};
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&f->world, children, 2, &targets[0], NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_import_big_compound(&f->world, children, 2, &targets[1], NULL) == ENTASIS_STATUS_OK);
    entasis_cooking_context_t cooking = {0};
    entasis_cooked_mesh_t mesh = {0};
    /* eight distinct fan triangles exercise a full callback bundle. the
       two-child compounds exercise partial bundles using the same owner */
    const entasis_triangle_t triangles[8] = {
        {{0, 0, 0}, {-4, 0, -4}, {0, 0, -4}}, {{0, 0, 0}, {0, 0, -4}, {4, 0, -4}}, {{0, 0, 0}, {4, 0, -4}, {4, 0, 0}}, {{0, 0, 0}, {4, 0, 0}, {4, 0, 4}}, {{0, 0, 0}, {4, 0, 4}, {0, 0, 4}}, {{0, 0, 0}, {0, 0, 4}, {-4, 0, 4}}, {{0, 0, 0}, {-4, 0, 4}, {-4, 0, 0}}, {{0, 0, 0}, {-4, 0, 0}, {-4, 0, -4}}};
    ENTASIS_TEST_CHECK(entasis_cooking_context_init(&cooking, 131072, 32, NULL, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cook_mesh(&cooking, triangles, 8, (entasis_vector3_t){1, 1, 1}, &mesh, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_import(&f->world, &mesh, &targets[2], NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_destroy(&mesh, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooking_context_destroy(&cooking, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
static int compound_targets(task_fixture_t *f, entasis_shape_handle_t targets[3])
{
    return compound_targets_order(f, targets, 0);
}
#endif
