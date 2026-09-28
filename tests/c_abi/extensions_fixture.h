#ifndef ENTASIS_EXTENSIONS_FIXTURE_H
#define ENTASIS_EXTENSIONS_FIXTURE_H
#include "compound_task_fixture.h"
#include "custom_constraints_fixture.h"

typedef struct ef_state_t
{
    unsigned initialized, disposed, completed, scheduled, velocity, contacts, child_contacts, errors;
    entasis_world_t *world;
} ef_state_t;
static entasis_status_t ENTASIS_CALL ef_initialize(void *raw, entasis_world_t w)
{
    ef_state_t *s = (ef_state_t *)raw;
    ++s->initialized;
    if (entasis_world_clear(&w, NULL) != ENTASIS_STATUS_INVALID_ARGUMENT)
        ++s->errors;
    return ENTASIS_STATUS_OK;
}
static void ENTASIS_CALL ef_dispose(void *raw)
{
    ++((ef_state_t *)raw)->disposed;
}
static entasis_status_t ENTASIS_CALL ef_prepare(void *raw, float dt)
{
    (void)raw;
    return dt > 0 ? ENTASIS_STATUS_OK : ENTASIS_STATUS_INVALID_ARGUMENT;
}
static void ENTASIS_CALL ef_integrate(void *raw, const entasis_pose_integration_view_t *v)
{
    ef_state_t *s = (ef_state_t *)raw;
    ++s->velocity;
    for (unsigned lane = 0; lane < 8; ++lane)
        if (v->active_mask->lanes[lane])
            v->velocity->linear.z.lanes[lane] = 0.25f;
}
static entasis_bool_t ENTASIS_CALL ef_allow(void *raw, int32_t worker, entasis_collidable_reference_t a, entasis_collidable_reference_t b, float *margin)
{
    (void)raw;
    (void)worker;
    (void)a;
    (void)b;
    *margin = 0.125f;
    return ENTASIS_TRUE;
}
static entasis_bool_t ENTASIS_CALL ef_allow_child(void *raw, int32_t worker, entasis_collidable_reference_t a, entasis_collidable_reference_t b, int32_t ca, int32_t cb)
{
    (void)raw;
    (void)worker;
    (void)a;
    (void)b;
    (void)ca;
    (void)cb;
    return ENTASIS_TRUE;
}
static entasis_bool_t ENTASIS_CALL ef_configure(void *raw, int32_t worker, entasis_collidable_reference_t a, entasis_collidable_reference_t b, entasis_contact_manifold_t *m, entasis_contact_material_t *material)
{
    ef_state_t *s = (ef_state_t *)raw;
    (void)worker;
    (void)a;
    (void)b;
    ++s->contacts;
    *material = entasis_contact_material_default();
    material->friction_coefficient = 0;
    material->maximum_recovery_velocity = 0;
    if (m->kind == ENTASIS_MANIFOLD_CONVEX)
        for (int i = 0; i < m->convex.count; ++i)
            m->convex.contacts[i].depth = 0;
    else
        for (int i = 0; i < m->nonconvex.count; ++i)
            m->nonconvex.contacts[i].depth = 0;
    return ENTASIS_TRUE;
}
static entasis_bool_t ENTASIS_CALL ef_configure_child(void *raw, int32_t worker, entasis_collidable_reference_t a, entasis_collidable_reference_t b, int32_t ca, int32_t cb, entasis_convex_contact_manifold_t *m)
{
    ef_state_t *s = (ef_state_t *)raw;
    (void)worker;
    (void)a;
    (void)b;
    (void)ca;
    (void)cb;
    ++s->child_contacts;
    for (int i = 0; i < m->count; ++i)
        m->contacts[i].feature_id = 91;
    return ENTASIS_TRUE;
}
static entasis_status_t ENTASIS_CALL ef_completed(void *raw, entasis_world_t w, entasis_timestep_completion_stage_t stage, float dt)
{
    ef_state_t *s = (ef_state_t *)raw;
    if (stage != s->completed % 4 || dt != 1.0f / 64.0f || entasis_world_stage_sleep(&w, NULL) != ENTASIS_STATUS_INVALID_ARGUMENT)
        ++s->errors;
    ++s->completed;
    return ENTASIS_STATUS_OK;
}
static int32_t ENTASIS_CALL ef_schedule(void *raw, int32_t index)
{
    ef_state_t *s = (ef_state_t *)raw;
    if (index != (int32_t)(s->scheduled % 2))
        ++s->errors;
    ++s->scheduled;
    return 3;
}
static entasis_status_t ENTASIS_CALL ef_step(void *raw, entasis_step_scope_t scope, float dt)
{
    (void)raw;
    entasis_status_t status = entasis_step_scope_sleep(scope, NULL);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_report_completion(scope, ENTASIS_TIMESTEP_SLEPT, dt, NULL);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_predict_bounds(scope, dt, NULL);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_report_completion(scope, ENTASIS_TIMESTEP_BEFORE_COLLISION_DETECTION, dt, NULL);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_collision_detection(scope, dt, NULL);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_report_completion(scope, ENTASIS_TIMESTEP_COLLISIONS_DETECTED, dt, NULL);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_solve(scope, dt, NULL);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_report_completion(scope, ENTASIS_TIMESTEP_CONSTRAINTS_SOLVED, dt, NULL);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_optimize(scope, NULL);
    return status;
}
static uint32_t ef_bits(float value)
{
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}
/* one world combines custom shapes, tasks, constraints, callbacks and allocation */
static int ef_run(unsigned scope, unsigned custom_step, const entasis_allocator_t *allocator, int output)
{
    task_fixture_t f = {0};
    ef_state_t s = {0};
    cc_state_t constraint = {0};
    s.world = &f.world;
    entasis_world_description_t d = description();
    d.solve.substeps = 2;
    d.solve.velocity_iterations = 3;
    d.capacity.collision_child_pairs = 256;
    if (allocator)
        d.allocator = *allocator;
    entasis_velocity_callbacks_t velocity = {0};
    velocity.struct_size = sizeof(velocity);
    velocity.struct_version = 1;
    velocity.user_context = &s;
    velocity.initialize = ef_initialize;
    velocity.prepare = ef_prepare;
    velocity.integrate = ef_integrate;
    velocity.dispose = ef_dispose;
    entasis_contact_callbacks_t contact = {sizeof(contact), 1, &s, ef_initialize, ef_allow, ef_allow_child, ef_configure, ef_configure_child, ef_dispose, NULL};
    entasis_timestep_callbacks_t stages = {sizeof(stages), 1, &s, ef_completed};
    entasis_timestepper_t stepper = {sizeof(stepper), 1, &s, ef_step};
    entasis_substep_scheduler_t scheduler = {sizeof(scheduler), 1, &s, ef_schedule};
    entasis_world_extensions_t ext = entasis_world_extensions_default();
    ext.allocation_scope = scope;
    ext.velocity_callbacks = &velocity;
    ext.contact_callbacks = &contact;
    ext.timestep_callbacks = &stages;
    ext.substep_scheduler = &scheduler;
    if (custom_step)
        ext.timestepper = &stepper;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&f.world, &d, &ext, NULL) == ENTASIS_STATUS_OK);
    velocity.integrate = NULL;
    contact.configure = NULL;
    stages.stage_completed = NULL;
    stepper.step = NULL;
    scheduler.iterations = NULL;
    f.shape.world = &f.world;
    f.shape.alignment = 16;
    f.shape.delegate_child = 1;
    f.task.world = &f.world;
    f.task.tag = 73;
    entasis_sphere_t sphere = entasis_sphere(1);
    ENTASIS_TEST_CHECK(entasis_shape_add(&f.world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &f.sphere, NULL) == ENTASIS_STATUS_OK);
    entasis_custom_shape_registration_t sr = registration(&f.shape);
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&f.world, &sr, &f.type, NULL) == ENTASIS_STATUS_OK);
    payload_t payload = {f.sphere, 1, 17};
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&f.world, f.type, &payload, sizeof(payload), &f.custom, NULL) == ENTASIS_STATUS_OK);
    f.shape.self = f.custom;
    f.task.custom = f.custom;
    f.task.type = f.type;
    ENTASIS_TEST_CHECK(task_fixture_bind(&f) == 0);
    entasis_shape_handle_t compounds[3];
    ENTASIS_TEST_CHECK(compound_targets(&f, compounds) == 0);
    constraint.world = &f.world;
    constraint.arity = 1;
    constraint.reentry = 1;
    atomic_init(&constraint.validations, 0);
    atomic_init(&constraint.full, 0);
    atomic_init(&constraint.partial, 0);
    atomic_init(&constraint.errors, 0);
    for (unsigned i = 0; i < 4; ++i)
        atomic_init(&constraint.phases[i], 0);
    entasis_custom_constraint_registration_t cr = cc_registration(&constraint, 56);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&f.world, &cr, NULL) == ENTASIS_STATUS_OK);
    entasis_body_handle_t bodies[9];
    entasis_constraint_handle_t constraints[9];
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_custom_shape_inertia(&f.world, f.custom, 1, &inertia, NULL) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 9; ++i)
    {
        entasis_body_description_t bd = entasis_body_dynamic(f.custom, inertia, pose_at((float)i * 4, 1.5f, 0),
                                                             entasis_velocity((entasis_vector3_t){0, 0, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
        ENTASIS_TEST_CHECK(entasis_body_add(&f.world, &bd, &bodies[i], NULL) == ENTASIS_STATUS_OK);
        entasis_static_description_t sd = entasis_static_body(i == 0 ? compounds[0] : f.sphere, pose_at((float)i * 4, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t sh;
        ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &sh, NULL) == ENTASIS_STATUS_OK);
        float target = (float)(i + 1) * 0.125f;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_add(&f.world, 56, &bodies[i], 1, &target, 4, &constraints[i], NULL) == ENTASIS_STATUS_OK);
    }
    entasis_static_description_t sd = entasis_static_body(f.custom, pose_at(50, 0, 0), entasis_ccd_discrete());
    entasis_static_handle_t sh;
    ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &sh, NULL) == ENTASIS_STATUS_OK);
    const entasis_compound_child_t children[2] = {entasis_compound_child(f.custom, pose_at(-0.5f, 0, 0)), entasis_compound_child(f.custom, pose_at(0.5f, 0, 0))};
    entasis_shape_handle_t child_compound;
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&f.world, children, 2, &child_compound, NULL) == ENTASIS_STATUS_OK);
    sd = entasis_static_body(child_compound, pose_at(60, 0, 0), entasis_ccd_discrete());
    ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &sh, NULL) == ENTASIS_STATUS_OK);
    entasis_query_context_t q = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_init(&q, &f.world, NULL, NULL, NULL) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 3; ++i)
        ENTASIS_TEST_CHECK(entasis_world_step(&f.world, 1.0f / 64.0f, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&f.world, NULL) == ENTASIS_STATUS_OK);
    entasis_contact_manifold_t manifold = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_collision_query(&q, f.custom, pose_at(0, 0, 0), f.sphere, pose_at(0, 1, 0), 0, &manifold, NULL) == ENTASIS_STATUS_OK);
    entasis_ray_hit_t ray = {0};
    entasis_ray_t r = {{45, 0, 0}, {1, 0, 0}, 8};
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&q, r, NULL, &ray, NULL) == ENTASIS_STATUS_OK);
    entasis_sweep_hit_t sweep = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_sweep_closest(&q, f.sphere, pose_at(55, 0, 0), entasis_velocity((entasis_vector3_t){1, 0, 0}, (entasis_vector3_t){0, 0, 0}), 10, NULL, NULL, &sweep, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&f.world, NULL) == ENTASIS_STATUS_OK);
    if (output)
        printf("integrated scope=%u custom=%u ray=%08x sweep=%08x child=%d contact=%08x\n", scope, custom_step, ef_bits(ray.t), ef_bits(sweep.sweep.t1), sweep.sweep.child_b, ef_bits(manifold.convex.contacts[0].depth));
    for (unsigned i = 0; i < 9; ++i)
    {
        entasis_body_state_t b = {0};
        float impulse = 0;
        uint64_t written = 0, required = 0;
        ENTASIS_TEST_CHECK(entasis_body_get(&f.world, bodies[i], &b, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulses(&f.world, constraints[i], &impulse, 1, &written, &required, NULL) == ENTASIS_STATUS_OK && written == 1 && required == 1);
        if (output)
            printf("body=%u p=%08x,%08x,%08x v=%08x,%08x,%08x impulse=%08x\n", i, ef_bits(b.pose.position.x), ef_bits(b.pose.position.y), ef_bits(b.pose.position.z), ef_bits(b.velocity.linear.x), ef_bits(b.velocity.linear.y), ef_bits(b.velocity.linear.z), ef_bits(impulse));
    }
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&f.world, &stats, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.step_index == 3 && s.initialized == 2 && s.completed == 12 && s.scheduled == 6 && s.contacts > 0 && s.child_contacts > 0 && s.velocity > 0 && s.errors == 0);
    ENTASIS_TEST_CHECK(task_count(&f.task.errors) == 0 && task_count(&f.task.child) > 0 && atomic_load(&constraint.errors) == 0 && atomic_load(&constraint.full) > 0 && atomic_load(&constraint.partial) > 0);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&q, NULL) == ENTASIS_STATUS_OK && entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(s.disposed == 2 && f.shape.disposals == 1 && f.shape.errors == 0);
    if (output)
        printf("completed=%u scheduled=%u disposed=%u\n", s.completed, s.scheduled, s.disposed);
    return 0;
}
#endif
