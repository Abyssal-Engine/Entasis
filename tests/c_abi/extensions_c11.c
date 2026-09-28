#include "entasis.h"
#include "test_support.h"
#include "generated_layout_asserts.h"
#include "custom_constraints_fixture.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

static entasis_rigid_pose_t at(float x, float y, float z)
{
    return entasis_pose((entasis_vector3_t){x, y, z}, (entasis_quaternion_t){0, 0, 0, 1});
}
static entasis_world_description_t small_world(void)
{
    entasis_world_description_t value = entasis_world_description_default();
    value.gravity = (entasis_vector3_t){0, 0, 0};
    value.damping.linear = 0;
    value.damping.angular = 0;
    value.threading.worker_count = 1;
    value.profiling = ENTASIS_TRUE;
    value.capacity.bodies = 32;
    value.capacity.statics = 16;
    value.capacity.shapes_per_type = 16;
    value.capacity.constraints = 64;
    value.capacity.pairs = 256;
    value.capacity.broad_phase_candidates = 256;
    return value;
}
static int add_sphere(entasis_world_t *world, float x, float y, entasis_shape_handle_t *shape, entasis_body_handle_t *body)
{
    entasis_diagnostic_t d = {0};
    const entasis_sphere_t sphere = entasis_sphere(1);
    ENTASIS_TEST_CHECK(entasis_shape_add(world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, shape, &d) == ENTASIS_STATUS_OK);
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1, &inertia) == ENTASIS_STATUS_OK);
    const entasis_body_description_t desc = entasis_body_dynamic(*shape, inertia, at(x, y, 0),
                                                                 entasis_velocity((entasis_vector3_t){0, 0, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
    ENTASIS_TEST_CHECK(entasis_body_add(world, &desc, body, &d) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct stage_state_t
{
    entasis_step_scope_t current, expired;
    unsigned stages, calls, errors, schedules;
    unsigned order[32];
    entasis_bool_t fail, nested;
} stage_state_t;
static entasis_status_t ENTASIS_CALL stage_completed(void *raw, entasis_world_t world, entasis_timestep_completion_stage_t stage, float dt)
{
    stage_state_t *state = (stage_state_t *)raw;
    entasis_diagnostic_t d = {0};
    if (state->stages < 32u)
        state->order[state->stages] = stage;
    state->stages += 1u;
    if (!(dt > 0) || entasis_world_stage_sleep(&world, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->errors += 1u;
    if (state->nested && entasis_step_scope_sleep(state->current, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->errors += 1u;
    return ENTASIS_STATUS_OK;
}
static int32_t ENTASIS_CALL substep_iterations(void *raw, int32_t index)
{
    stage_state_t *state = (stage_state_t *)raw;
    if (index != (int32_t)(state->schedules % 2u))
        state->errors += 1u;
    state->schedules += 1u;
    return 3;
}
static entasis_status_t ENTASIS_CALL custom_step(void *raw, entasis_step_scope_t scope, float dt)
{
    stage_state_t *state = (stage_state_t *)raw;
    entasis_diagnostic_t d = {0};
    state->calls += 1u;
    if (state->expired.world.opaque != NULL && entasis_step_scope_sleep(state->expired, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->errors += 1u;
    state->current = scope;
    state->nested = ENTASIS_TRUE;
    entasis_status_t status = entasis_step_scope_sleep(scope, &d);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_report_completion(scope, ENTASIS_TIMESTEP_SLEPT, dt, &d);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_predict_bounds(scope, dt, &d);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_report_completion(scope, ENTASIS_TIMESTEP_BEFORE_COLLISION_DETECTION, dt, &d);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_collision_detection(scope, dt, &d);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_report_completion(scope, ENTASIS_TIMESTEP_COLLISIONS_DETECTED, dt, &d);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_solve(scope, dt, &d);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_report_completion(scope, ENTASIS_TIMESTEP_CONSTRAINTS_SOLVED, dt, &d);
    if (status == ENTASIS_STATUS_OK)
        status = entasis_step_scope_optimize(scope, &d);
    state->nested = ENTASIS_FALSE;
    state->expired = scope;
    if (status != ENTASIS_STATUS_OK)
        return status;
    return state->fail ? ENTASIS_STATUS_INVALID_ARGUMENT : ENTASIS_STATUS_OK;
}
static int test_stages_scopes_scheduler_and_descriptor_copy(void)
{
    stage_state_t state = {0};
    entasis_timestep_callbacks_t callbacks = {(uint32_t)sizeof(callbacks), 1, &state, stage_completed};
    entasis_timestepper_t stepper = {(uint32_t)sizeof(stepper), 1, &state, custom_step};
    entasis_substep_scheduler_t scheduler = {(uint32_t)sizeof(scheduler), 1, &state, substep_iterations};
    entasis_world_extensions_t ext = entasis_world_extensions_default();
    ext.timestep_callbacks = &callbacks;
    ext.substep_scheduler = &scheduler;
    entasis_world_description_t desc = small_world();
    desc.solve.substeps = 2;
    entasis_world_t world = {0};
    entasis_diagnostic_t d = {0};
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, &d) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t shape = {0};
    entasis_body_handle_t body = {0};
    ENTASIS_TEST_CHECK(add_sphere(&world, 0, 10, &shape, &body) == 0);
    ENTASIS_TEST_CHECK(entasis_world_stage_sleep(&world, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stage_predict_bounds(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stage_collision_detection(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stage_solve(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stage_optimize(&world, &d) == ENTASIS_STATUS_OK);
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.step_index == 0 && state.stages == 0u);
    callbacks.stage_completed = NULL;
    scheduler.iterations = NULL;
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.stages == 4u && state.errors == 0u && state.schedules == 4u);
    for (unsigned i = 0; i < 4u; ++i)
        ENTASIS_TEST_CHECK(state.order[i] == i);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stage_sleep(&world, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&world, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &d) == ENTASIS_STATUS_OK);

    state = (stage_state_t){0};
    callbacks.stage_completed = stage_completed;
    scheduler.iterations = substep_iterations;
    ext.timestepper = &stepper;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(add_sphere(&world, 0, 10, &shape, &body) == 0);
    stepper.step = NULL;
    for (unsigned i = 0; i < 2u; ++i)
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.step_index == 2 && state.calls == 2u && state.errors == 0u && state.stages == 8u);
    ENTASIS_TEST_CHECK(entasis_step_scope_sleep(state.expired, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
    state.fail = ENTASIS_TRUE;
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.step_index == 2);
    ENTASIS_TEST_CHECK(entasis_step_scope_solve(state.expired, 1.0f / 64.0f, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &d) == ENTASIS_STATUS_OK);
    return 0;
}

typedef struct callback_state_t
{
    entasis_world_t world;
    unsigned velocity_init, velocity_dispose, contact_init, contact_dispose;
    unsigned prepared, integrated[4], errors[4], allowed[4], child_allowed[4], configured[4], child_configured[4];
    unsigned selected;
    entasis_bool_t fail_velocity, fail_contact;
    entasis_body_handle_t bodies[12];
    unsigned body_count;
} callback_state_t;
static entasis_status_t ENTASIS_CALL velocity_init(void *raw, entasis_world_t world)
{
    callback_state_t *state = (callback_state_t *)raw;
    state->velocity_init += 1u;
    state->world = world;
    entasis_diagnostic_t d = {0};
    if (entasis_world_clear(&world, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->errors[0] += 1u;
    return state->fail_velocity ? ENTASIS_STATUS_INVALID_ARGUMENT : ENTASIS_STATUS_OK;
}
static entasis_status_t ENTASIS_CALL velocity_prepare(void *raw, float dt)
{
    callback_state_t *state = (callback_state_t *)raw;
    state->prepared += 1u;
    return dt > 0 ? ENTASIS_STATUS_OK : ENTASIS_STATUS_INVALID_ARGUMENT;
}
static void ENTASIS_CALL velocity_integrate(void *raw, const entasis_pose_integration_view_t *view)
{
    callback_state_t *state = (callback_state_t *)raw;
    const unsigned worker = (unsigned)view->worker_index;
    if (worker >= 4u)
        return;
    state->integrated[worker] += 1u;
    if (((uintptr_t)view->active_mask | (uintptr_t)view->position | (uintptr_t)view->orientation |
         (uintptr_t)view->inertia | (uintptr_t)view->dt | (uintptr_t)view->velocity) %
            32u !=
        0u)
        state->errors[worker] += 1u;
    for (unsigned lane = 0; lane < 8u; ++lane)
    {
        const entasis_body_handle_t handle = view->body_handles[lane];
        if (view->active_mask->lanes[lane] != 0)
        {
            entasis_bool_t found = ENTASIS_FALSE;
            for (unsigned i = 0; i < state->body_count; ++i)
                if (state->bodies[i].value == handle.value)
                    found = ENTASIS_TRUE;
            if (!found || !(view->dt->lanes[lane] > 0) || !isfinite(view->position->x.lanes[lane]))
                state->errors[worker] += 1u;
            view->velocity->linear.x.lanes[lane] = 3.0f;
        }
        else
        {
            if (entasis_body_handle_is_valid(handle))
                state->errors[worker] += 1u;
            view->velocity->linear.x.lanes[lane] = 123456.0f;
        }
    }
}
static void ENTASIS_CALL velocity_dispose(void *raw)
{
    ((callback_state_t *)raw)->velocity_dispose += 1u;
}
static entasis_status_t ENTASIS_CALL contact_init(void *raw, entasis_world_t world)
{
    callback_state_t *state = (callback_state_t *)raw;
    state->world = world;
    state->contact_init += 1u;
    return state->fail_contact ? ENTASIS_STATUS_INVALID_ARGUMENT : ENTASIS_STATUS_OK;
}
static entasis_bool_t ENTASIS_CALL contact_allow(void *raw, int32_t worker, entasis_collidable_reference_t a, entasis_collidable_reference_t b, float *margin)
{
    callback_state_t *state = (callback_state_t *)raw;
    (void)a;
    (void)b;
    state->allowed[worker] += 1u;
    *margin = 0.125f;
    return ENTASIS_TRUE;
}
static entasis_bool_t ENTASIS_CALL child_allow(void *raw, int32_t worker, entasis_collidable_reference_t a, entasis_collidable_reference_t b, int32_t ca, int32_t cb)
{
    callback_state_t *state = (callback_state_t *)raw;
    (void)a;
    (void)b;
    (void)ca;
    (void)cb;
    state->child_allowed[worker] += 1u;
    return ENTASIS_TRUE;
}
static entasis_bool_t ENTASIS_CALL contact_configure(void *raw, int32_t worker, entasis_collidable_reference_t a, entasis_collidable_reference_t b, entasis_contact_manifold_t *manifold, entasis_contact_material_t *material)
{
    callback_state_t *state = (callback_state_t *)raw;
    (void)a;
    (void)b;
    state->configured[worker] += 1u;
    *material = entasis_contact_material_default();
    material->friction_coefficient = 0.125f;
    material->maximum_recovery_velocity = 2.0f;
    if (manifold->kind == ENTASIS_MANIFOLD_CONVEX)
    {
        for (int32_t i = 0; i < manifold->convex.count; ++i)
            manifold->convex.contacts[i].depth = 0.0625f;
    }
    else
    {
        for (int32_t i = 0; i < manifold->nonconvex.count; ++i)
            manifold->nonconvex.contacts[i].depth = 0.0625f;
    }
    entasis_diagnostic_t d = {0};
    if (entasis_world_clear(&state->world, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->errors[worker] += 1u;
    return ENTASIS_TRUE;
}
static entasis_bool_t ENTASIS_CALL child_configure(void *raw, int32_t worker, entasis_collidable_reference_t a, entasis_collidable_reference_t b, int32_t ca, int32_t cb, entasis_convex_contact_manifold_t *manifold)
{
    callback_state_t *state = (callback_state_t *)raw;
    (void)a;
    (void)b;
    (void)ca;
    (void)cb;
    state->child_configured[worker] += 1u;
    for (int32_t i = 0; i < manifold->count; ++i)
        manifold->contacts[i].depth = 0.125f;
    return ENTASIS_TRUE;
}
static void ENTASIS_CALL contact_dispose(void *raw)
{
    ((callback_state_t *)raw)->contact_dispose += 1u;
}
static int32_t ENTASIS_CALL select_constraint(void *raw, entasis_collidable_reference_t a, entasis_collidable_reference_t b, const entasis_contact_manifold_t *manifold, int32_t native_type)
{
    callback_state_t *state = (callback_state_t *)raw;
    (void)a;
    (void)b;
    (void)manifold;
    state->selected += 1u;
    return native_type;
}
static entasis_velocity_callbacks_t velocities(callback_state_t *state)
{
    entasis_velocity_callbacks_t c = {0};
    c.struct_size = (uint32_t)sizeof(c);
    c.struct_version = 1;
    c.user_context = state;
    c.initialize = velocity_init;
    c.prepare = velocity_prepare;
    c.integrate = velocity_integrate;
    c.dispose = velocity_dispose;
    return c;
}
static entasis_contact_callbacks_t contacts(callback_state_t *state)
{
    entasis_contact_callbacks_t c = {0};
    c.struct_size = (uint32_t)sizeof(c);
    c.struct_version = 1;
    c.user_context = state;
    c.initialize = contact_init;
    c.allow = contact_allow;
    c.allow_child = child_allow;
    c.configure = contact_configure;
    c.configure_child = child_configure;
    c.dispose = contact_dispose;
    c.select_constraint = select_constraint;
    return c;
}
static int test_velocity_lanes_handles_and_failed_initialization(void)
{
    entasis_diagnostic_t d = {0};
    entasis_world_t world = {0};
    callback_state_t state = {0};
    entasis_velocity_callbacks_t velocity = velocities(&state);
    entasis_contact_callbacks_t contact = contacts(&state);
    entasis_world_extensions_t ext = entasis_world_extensions_default();
    ext.velocity_callbacks = &velocity;
    ext.contact_callbacks = &contact;
    entasis_world_description_t desc = small_world();
    desc.threading.worker_count = 2;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, &d) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 10u; ++i)
    {
        entasis_shape_handle_t shape = {0};
        ENTASIS_TEST_CHECK(add_sphere(&world, (float)i * 5.0f, 20, &shape, &state.bodies[i]) == 0);
    }
    state.body_count = 10;
    ENTASIS_TEST_CHECK(entasis_body_remove(&world, state.bodies[0], &d) == ENTASIS_STATUS_OK);
    state.bodies[0] = state.bodies[9];
    state.body_count = 9;
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.prepared > 0u && state.integrated[0] + state.integrated[1] > 0u);
    ENTASIS_TEST_CHECK(state.errors[0] + state.errors[1] == 0u);
    for (unsigned i = 0; i < state.body_count; ++i)
    {
        entasis_body_state_t body = {0};
        ENTASIS_TEST_CHECK(entasis_body_get(&world, state.bodies[i], &body, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(body.velocity.linear.x == 3.0f);
    }
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.velocity_init == 1u && state.velocity_dispose == 1u && state.contact_init == 1u && state.contact_dispose == 1u);
    for (unsigned failure = 0; failure < 2u; ++failure)
    {
        state = (callback_state_t){0};
        state.fail_velocity = (entasis_bool_t)(failure == 0);
        state.fail_contact = (entasis_bool_t)(failure == 1);
        entasis_buffer_pool_t pool = {0};
        ENTASIS_TEST_CHECK(entasis_buffer_pool_init(&pool, 4096, 4, NULL, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_init_with_pool_extended(&world, &desc, &pool, &ext, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(world.opaque == NULL && state.velocity_init == 1u && state.contact_dispose == 0u);
        ENTASIS_TEST_CHECK(state.velocity_dispose == failure && state.contact_init == failure);
        ENTASIS_TEST_CHECK(entasis_buffer_pool_destroy(&pool, &d) == ENTASIS_STATUS_OK);
    }
    state = (callback_state_t){0};
    velocity.integrate = NULL;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, &d) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(state.velocity_init == 0u && state.contact_init == 0u);
    velocity = velocities(&state);
    ext.allocation_scope = 2; /* unknown scope. All_Owned (1) is now implemented */
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, &d) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ext.allocation_scope = ENTASIS_ALLOCATION_LEGACY;
    desc.use_pose_policy = ENTASIS_TRUE;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, &d) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(state.velocity_init == 0u);
    return 0;
}
static int test_contact_manifold_material_and_child_propagation(void)
{
    entasis_diagnostic_t d = {0};
    entasis_world_t world = {0};
    callback_state_t state = {0};
    entasis_contact_callbacks_t contact = contacts(&state);
    entasis_world_extensions_t ext = entasis_world_extensions_default();
    ext.contact_callbacks = &contact;
    entasis_world_description_t desc = small_world();
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, &d) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t sphere = {0};
    entasis_body_handle_t body = {0};
    ENTASIS_TEST_CHECK(add_sphere(&world, 0, 0.8f, &sphere, &body) == 0);
    const entasis_box_t box = entasis_box(2, 2, 2);
    entasis_shape_handle_t box_shape = {0}, compound = {0};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_BOX, &box, &box_shape, &d) == ENTASIS_STATUS_OK);
    const entasis_compound_child_t children[2] = {entasis_compound_child(box_shape, at(-0.6f, -1, 0)), entasis_compound_child(box_shape, at(0.6f, -1, 0))};
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&world, children, 2, &compound, &d) == ENTASIS_STATUS_OK);
    const entasis_static_description_t floor = entasis_static_body(compound, at(0, 0, 0), entasis_ccd_discrete());
    entasis_static_handle_t floor_handle = {0};
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &floor, ENTASIS_AWAKENING_NONE, &floor_handle, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stage_predict_bounds(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_stage_collision_detection(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.allowed[0] > 0u && state.child_allowed[0] > 0u && state.configured[0] > 0u && state.child_configured[0] > 0u && state.selected > 0u && state.errors[0] == 0u);
    entasis_constraint_info_t infos[16] = {0};
    uint64_t written = 0, required = 0;
    ENTASIS_TEST_CHECK(entasis_constraint_enumerate(&world, infos, 16, &written, &required, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(written > 0);
    entasis_bool_t found = ENTASIS_FALSE;
    for (uint64_t i = 0; i < written; ++i)
    {
        entasis_solver_contact_data_t data = {0};
        if (entasis_solver_contact_data(&world, infos[i].handle, &data, &d) != ENTASIS_STATUS_OK)
            continue;
        ENTASIS_TEST_CHECK(data.material.friction_coefficient == 0.125f && data.material.maximum_recovery_velocity == 2.0f);
        ENTASIS_TEST_CHECK(data.contact_count > 0);
        for (unsigned j = 0; j < data.contact_count; ++j)
            ENTASIS_TEST_CHECK(data.contacts[j].depth == 0.0625f);
        found = ENTASIS_TRUE;
    }
    ENTASIS_TEST_CHECK(found);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.contact_init == 1u && state.contact_dispose == 1u);
    return 0;
}

typedef struct static_filter_state_t
{
    entasis_world_t *world;
    entasis_body_handle_t body;
    entasis_bool_t allow;
    unsigned calls, errors;
} static_filter_state_t;
static entasis_bool_t ENTASIS_CALL static_filter(void *raw, entasis_body_handle_t body)
{
    static_filter_state_t *state = (static_filter_state_t *)raw;
    entasis_diagnostic_t d = {0};
    state->calls += 1u;
    if (body.value != state->body.value || entasis_world_clear(state->world, &d) != ENTASIS_STATUS_INVALID_ARGUMENT)
        state->errors += 1u;
    return state->allow;
}
static int test_filtered_static_awakening(void)
{
    for (unsigned operation = 0; operation < 3u; ++operation)
    {
        entasis_world_t world = {0};
        entasis_diagnostic_t d = {0};
        const entasis_world_description_t desc = small_world();
        ENTASIS_TEST_CHECK(entasis_world_init(&world, &desc, &d) == ENTASIS_STATUS_OK);
        entasis_shape_handle_t shape = {0};
        entasis_body_handle_t body = {0};
        ENTASIS_TEST_CHECK(add_sphere(&world, 0, 0, &shape, &body) == 0);
        const entasis_activity_description_t activity = entasis_body_activity(1, 1);
        ENTASIS_TEST_CHECK(entasis_body_set_activity(&world, body, &activity, &d) == ENTASIS_STATUS_OK);
        entasis_bool_t sleeping = ENTASIS_FALSE;
        for (unsigned i = 0; i < 64u && !sleeping; ++i)
        {
            ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64.0f, &d) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_body_is_sleeping(&world, body, &sleeping, &d) == ENTASIS_STATUS_OK);
        }
        ENTASIS_TEST_CHECK(sleeping);
        const entasis_static_description_t fixed = entasis_static_body(shape, at(0, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t handle = {0};
        static_filter_state_t state = {&world, body, ENTASIS_FALSE, 0, 0};
        if (operation != 0u)
            ENTASIS_TEST_CHECK(entasis_static_add(&world, &fixed, ENTASIS_AWAKENING_NONE, &handle, &d) == ENTASIS_STATUS_OK);
        for (unsigned allow = 0; allow < 2u; ++allow)
        {
            state.allow = (entasis_bool_t)allow;
            state.calls = 0;
            if (operation == 0u)
                ENTASIS_TEST_CHECK(entasis_static_add_filtered(&world, &fixed, static_filter, &state, &handle, &d) == ENTASIS_STATUS_OK);
            if (operation == 1u)
                ENTASIS_TEST_CHECK(entasis_static_apply_filtered(&world, handle, &fixed, static_filter, &state, &d) == ENTASIS_STATUS_OK);
            if (operation == 2u)
                ENTASIS_TEST_CHECK(entasis_static_remove_filtered(&world, handle, static_filter, &state, &d) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(state.calls > 0u && state.errors == 0u);
            ENTASIS_TEST_CHECK(entasis_body_is_sleeping(&world, body, &sleeping, &d) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(sleeping == (allow == 0u));
            if (allow == 0u && operation == 0u)
                ENTASIS_TEST_CHECK(entasis_static_remove(&world, handle, ENTASIS_AWAKENING_NONE, &d) == ENTASIS_STATUS_OK);
            if (allow == 0u && operation == 2u)
                ENTASIS_TEST_CHECK(entasis_static_add(&world, &fixed, ENTASIS_AWAKENING_NONE, &handle, &d) == ENTASIS_STATUS_OK);
        }
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &d) == ENTASIS_STATUS_OK);
    }
    return 0;
}

static int test_body_control_inputs_targets_and_guards(void)
{
    for (unsigned scope = 0; scope < 2; ++scope)
    {
        entasis_world_t world = {0};
        entasis_diagnostic_t d = {0};
        entasis_world_description_t desc = small_world();
        desc.threading.worker_count = 2;
        desc.solve.substeps = 4;
        entasis_world_extensions_t ext = entasis_world_extensions_default();
        ext.allocation_scope = scope;
        ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, &d) == ENTASIS_STATUS_OK);
        entasis_shape_handle_t shape = {0};
        entasis_body_handle_t body = {0};
        ENTASIS_TEST_CHECK(add_sphere(&world, 0, 0, &shape, &body) == 0);
        entasis_body_control_configuration_t config = entasis_body_control_configuration_default();
        config.struct_version = 0;
        ENTASIS_TEST_CHECK(entasis_world_enable_body_control(&world, &config, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
        config = entasis_body_control_configuration_default();
        ENTASIS_TEST_CHECK(entasis_world_enable_body_control(&world, &config, &d) == ENTASIS_STATUS_OK);
        config.input_capacity = 0; /* copied, not borrowed */
        ENTASIS_TEST_CHECK(entasis_body_add_force(&world, body, (entasis_vector3_t){4, 0, 0}, ENTASIS_BODY_INPUT_FORCE, ENTASIS_BODY_CONTROL_WAKE, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_add_torque(&world, body, (entasis_vector3_t){0, 0, 2}, ENTASIS_BODY_INPUT_FORCE, ENTASIS_BODY_CONTROL_WAKE, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_add_force(&world, body, (entasis_vector3_t){0, 0, 0}, UINT32_MAX, ENTASIS_BODY_CONTROL_WAKE, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, &d) == ENTASIS_STATUS_OK);
        entasis_body_damping_t damping = {0};
        ENTASIS_TEST_CHECK(entasis_body_get_damping(&world, body, &damping, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_clear_inputs(&world, body, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_disable_body_control(&world, &d) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_end_read(&world, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.25f, &d) == ENTASIS_STATUS_OK);
        entasis_body_state_t state = {0};
        ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &state, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(fabsf(state.velocity.linear.x - 1) < 0.00001f && fabsf(state.velocity.angular.z - 1.25f) < 0.00001f);
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.25f, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &state, &d) == ENTASIS_STATUS_OK && fabsf(state.velocity.linear.x - 1) < 0.00001f);
        ENTASIS_TEST_CHECK(entasis_body_add_force_at_position(&world, body, (entasis_vector3_t){0, 2, 0}, state.pose.position, ENTASIS_BODY_CONTROL_WAKE, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_clear_inputs(&world, body, &d) == ENTASIS_STATUS_OK);
        damping.linear = 2;
        damping.mode = ENTASIS_BODY_DAMPING_OVERRIDE;
        ENTASIS_TEST_CHECK(entasis_body_set_damping(&world, body, &damping, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_get_damping(&world, body, &damping, &d) == ENTASIS_STATUS_OK && damping.linear == 2);
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.5f, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &state, &d) == ENTASIS_STATUS_OK && fabsf(state.velocity.linear.x - expf(-1)) < 0.00001f);
        const entasis_body_description_t moving = entasis_body_kinematic(shape, at(0, 10, 0), entasis_velocity((entasis_vector3_t){0, 0, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
        entasis_body_handle_t kinematic = {0};
        ENTASIS_TEST_CHECK(entasis_body_add(&world, &moving, &kinematic, &d) == ENTASIS_STATUS_OK);
        entasis_rigid_pose_t target = at(1, 10, 0), output = at(99, 0, 0);
        ENTASIS_TEST_CHECK(entasis_body_set_kinematic_target(&world, kinematic, &target, ENTASIS_BODY_CONTROL_WAKE, &d) == ENTASIS_STATUS_OK);
        target.position.x = 42;
        ENTASIS_TEST_CHECK(entasis_body_get_kinematic_target(&world, kinematic, &output, &d) == ENTASIS_STATUS_OK && output.position.x == 1);
        ENTASIS_TEST_CHECK(entasis_body_control_reserve(&world, 1, 1, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.5f, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_get(&world, kinematic, &state, &d) == ENTASIS_STATUS_OK && fabsf(state.pose.position.x - 1) < 0.00001f);
        output = at(99, 0, 0);
        ENTASIS_TEST_CHECK(entasis_body_get_kinematic_target(&world, kinematic, &output, &d) == ENTASIS_STATUS_NOT_FOUND && output.position.x == 99);
        ENTASIS_TEST_CHECK(entasis_body_clear_kinematic_target(&world, kinematic, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_get(&world, kinematic, &state, &d) == ENTASIS_STATUS_OK && state.velocity.linear.x == 0);
        ENTASIS_TEST_CHECK(entasis_world_disable_body_control(&world, &d) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &d) == ENTASIS_STATUS_OK);
    }
    return 0;
}

static int test_axis_lock_api(void)
{
    for (unsigned scope = 0; scope < 2; ++scope)
    {
        entasis_world_t world = {0};
        entasis_world_description_t desc = small_world();
        desc.solve.substeps = 4;
        desc.threading.worker_count = 2;
        entasis_world_extensions_t ext = entasis_world_extensions_default();
        ext.allocation_scope = scope;
        ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_enable_body_control(&world, NULL, NULL) == ENTASIS_STATUS_OK);
        entasis_shape_handle_t shape = {0};
        entasis_body_handle_t body = {0};
        ENTASIS_TEST_CHECK(add_sphere(&world, 0, 0, &shape, &body) == 0);
        entasis_body_axis_lock_t lock = entasis_body_axis_lock_default(at(0, 0, 0)), read = {0};
        lock.linear_axes = ENTASIS_BODY_LOCK_Y;
        lock.angular_axes = ENTASIS_BODY_LOCK_ALL;
        ENTASIS_TEST_CHECK(entasis_body_set_axis_lock(&world, body, &lock, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_get_axis_lock(&world, body, &read, NULL) == ENTASIS_STATUS_OK && read.linear_axes == lock.linear_axes);
        lock.linear_axes = UINT32_MAX;
        ENTASIS_TEST_CHECK(entasis_body_set_axis_lock(&world, body, &lock, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        lock.linear_axes = ENTASIS_BODY_LOCK_Y;
        lock.struct_version = 0;
        ENTASIS_TEST_CHECK(entasis_body_set_axis_lock(&world, body, &lock, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        lock.struct_version = 1;
        ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_get_axis_lock(&world, body, &read, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_body_set_axis_lock(&world, body, &lock, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_body_clear_axis_lock(&world, body, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_end_read(&world, NULL) == ENTASIS_STATUS_OK);
        for (unsigned i = 0; i < 40; ++i)
        {
            ENTASIS_TEST_CHECK(entasis_body_add_force(&world, body, (entasis_vector3_t){2, 8, 0}, ENTASIS_BODY_INPUT_ACCELERATION, ENTASIS_BODY_CONTROL_WAKE, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_body_add_torque(&world, body, (entasis_vector3_t){1, 2, 3}, ENTASIS_BODY_INPUT_ACCELERATION, ENTASIS_BODY_CONTROL_WAKE, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, NULL) == ENTASIS_STATUS_OK);
        }
        entasis_body_state_t state = {0};
        ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &state, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(state.pose.position.x > 0.2f && fabsf(state.pose.position.y) < 0.02f && fabsf(state.velocity.angular.z) < 0.02f);
        lock.linear_axes = lock.angular_axes = ENTASIS_BODY_LOCK_NONE;
        ENTASIS_TEST_CHECK(entasis_body_set_axis_lock(&world, body, &lock, NULL) == ENTASIS_STATUS_OK);
        read.reference.position.x = 999;
        ENTASIS_TEST_CHECK(entasis_body_get_axis_lock(&world, body, &read, NULL) == ENTASIS_STATUS_NOT_FOUND && read.reference.position.x == 999);
        ENTASIS_TEST_CHECK(entasis_body_clear_axis_lock(&world, body, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
    }
    return 0;
}

typedef enum reaction_mode_t
{
    REACTION_VALID,
    REACTION_ERROR,
    REACTION_NONFINITE,
    REACTION_UNKNOWN_STATUS
} reaction_mode_t;
typedef struct reaction_state_t
{
    entasis_world_t *world;
    reaction_mode_t mode;
    float scale;
    unsigned calls, errors;
} reaction_state_t;
static entasis_status_t ENTASIS_CALL read_joint_reaction(void *raw, const entasis_joint_reaction_provider_input_t *input, entasis_joint_impulse_wrench_t *output)
{
    reaction_state_t *state = (reaction_state_t *)raw;
    ++state->calls;
    if (input->type_id != 56 || input->body_count != 1 || input->description_size != sizeof(float) ||
        input->impulse_count != 1 || input->substep_duration != 1.0f / 64 || input->reserved != 0 ||
        *(const float *)input->description != 2 || input->impulses[0] != 2 || input->poses[0].orientation.w != 1 ||
        entasis_world_clear(state->world, NULL) != ENTASIS_STATUS_INVALID_ARGUMENT)
        ++state->errors;
    output[0].linear = (entasis_vector3_t){input->impulses[0] * state->scale, 0, 0};
    output[0].angular = (entasis_vector3_t){0, 0, 0};
    if (state->mode == REACTION_ERROR)
        return ENTASIS_STATUS_INVALID_DESCRIPTION;
    if (state->mode == REACTION_NONFINITE)
        output[0].linear.x = NAN;
    if (state->mode == REACTION_UNKNOWN_STATUS)
        return (entasis_status_t)255;
    return ENTASIS_STATUS_OK;
}
static int test_joint_break_providers(void)
{
    entasis_world_t worlds[2] = {{0}, {0}};
    cc_state_t constraints[2] = {0};
    reaction_state_t states[2] = {0};
    entasis_constraint_handle_t joints[2] = {0};
    for (unsigned index = 0; index < 2; ++index)
    {
        entasis_world_t *world = &worlds[index];
        ENTASIS_TEST_CHECK(cc_init(world, &constraints[index], 1, 2, 2, 8, NULL) == 0);
        entasis_custom_constraint_registration_t registration = cc_registration(&constraints[index], 56);
        ENTASIS_TEST_CHECK(entasis_custom_constraint_register(world, &registration, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_enable_joint_breaks(world, 2, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_enable_body_control(world, NULL, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_enable_restitution(world, NULL, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_enable_triggers(world, NULL, NULL) == ENTASIS_STATUS_OK);
        entasis_collider_part_configuration_t parts_configuration = entasis_collider_part_configuration_default();
        parts_configuration.event_subscription = ENTASIS_COLLIDER_PART_EVENTS_ENABLED;
        ENTASIS_TEST_CHECK(entasis_collider_parts_reserve(world, &parts_configuration, NULL) == ENTASIS_STATUS_OK);
        const entasis_sphere_t sphere = entasis_sphere(1);
        entasis_shape_handle_t sphere_handle = {0}, compound = {0};
        ENTASIS_TEST_CHECK(entasis_shape_add(world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &sphere_handle, NULL) == ENTASIS_STATUS_OK);
        const entasis_compound_child_t children[2] = {entasis_compound_child(sphere_handle, at(0, 0, 0)), entasis_compound_child(sphere_handle, at(3, 0, 0))};
        ENTASIS_TEST_CHECK(entasis_shape_import_compound(world, children, 2, &compound, NULL) == ENTASIS_STATUS_OK);
        const entasis_static_description_t sensor_description = entasis_static_body(compound, at(0, 0, 0), entasis_ccd_discrete());
        const entasis_static_description_t target_description = entasis_static_body(sphere_handle, at(3.5f, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t sensor = {0}, target_static = {0};
        ENTASIS_TEST_CHECK(entasis_static_add(world, &sensor_description, ENTASIS_AWAKENING_NONE, &sensor, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_static_add(world, &target_description, ENTASIS_AWAKENING_NONE, &target_static, NULL) == ENTASIS_STATUS_OK);
        entasis_collidable_reference_t reference = {0};
        ENTASIS_TEST_CHECK(entasis_static_collidable_reference(world, sensor, &reference, NULL) == ENTASIS_STATUS_OK);
        const entasis_collider_part_settings_t part_settings = {231, ENTASIS_COLLIDER_PART_TRIGGER, ENTASIS_COLLIDER_PART_STATIC_STATIC, {0}};
        entasis_collider_part_info_t part_info = {0};
        ENTASIS_TEST_CHECK(entasis_collider_part_set(world, reference, 1, &part_settings, &part_info, NULL) == ENTASIS_STATUS_OK);

        states[index].world = world;
        states[index].scale = (float)(index + 1);
        entasis_joint_reaction_provider_t provider = {sizeof(provider), 1, read_joint_reaction, &states[index]};
        ENTASIS_TEST_CHECK(entasis_constraint_set_reaction_provider(world, 56, &provider, NULL) == ENTASIS_STATUS_OK);
        memset(&provider, 0, sizeof(provider));
        entasis_body_handle_t body = {0};
        ENTASIS_TEST_CHECK(cc_body(world, 0, 0, &body) == 0);
        const float target = 2;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_add(world, 56, &body, 1, &target, sizeof(target), &joints[index], NULL) == ENTASIS_STATUS_OK);
        const entasis_joint_break_limits_t limits = {ENTASIS_JOINT_BREAK_FORCE, 100000, 0, 0, 700 + index};
        ENTASIS_TEST_CHECK(entasis_constraint_set_break_limits(world, joints[index], &limits, NULL) == ENTASIS_STATUS_OK);
    }
    for (unsigned index = 0; index < 2; ++index)
    {
        entasis_world_t *world = &worlds[index];
        reaction_state_t replacement = {world, REACTION_ERROR, 100, 0, 0};
        const entasis_joint_reaction_provider_t rejected = {sizeof(rejected), 1, read_joint_reaction, &replacement};
        ENTASIS_TEST_CHECK(entasis_constraint_set_reaction_provider(world, 56, &rejected, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_step(world, 1.0f / 32, NULL) == ENTASIS_STATUS_OK);
        entasis_trigger_event_t parent = {0};
        entasis_collider_part_event_t part = {0};
        uint64_t event_count = 0, event_required = 0;
        ENTASIS_TEST_CHECK(entasis_trigger_events_drain(world, &parent, 1, &event_count, &event_required, NULL) == ENTASIS_STATUS_OK && event_count == 1);
        ENTASIS_TEST_CHECK(entasis_trigger_part_events_drain(world, &part, 1, &event_count, &event_required, NULL) == ENTASIS_STATUS_OK && event_count == 1);
        ENTASIS_TEST_CHECK(parent.kind == ENTASIS_TRIGGER_ENTER && part.kind == ENTASIS_TRIGGER_ENTER);

        entasis_joint_reaction_t committed = {0};
        entasis_joint_break_limits_t limits = {0};
        ENTASIS_TEST_CHECK(entasis_world_begin_read(world, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_constraint_reaction(world, joints[index], &committed, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_constraint_get_break_limits(world, joints[index], &limits, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(committed.state == ENTASIS_JOINT_REACTION_SOLVED && committed.step == 1 && committed.sample.body_count == 1);
        ENTASIS_TEST_CHECK(committed.maximum_force == 128.0f * states[index].scale && committed.maximum_torque == 0);
        ENTASIS_TEST_CHECK(limits.user_id == 700 + index && limits.reserved == 0);
        ENTASIS_TEST_CHECK(entasis_constraint_clear_break_limits(world, joints[index], NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_constraint_set_reaction_provider(world, 56, NULL, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_constraint_break_events_discard(world, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_end_read(world, NULL) == ENTASIS_STATUS_OK);
        for (unsigned failure = 1; failure <= 3; ++failure)
        {
            states[index].mode = (reaction_mode_t)failure;
            const entasis_status_t expected = failure == 1 ? ENTASIS_STATUS_INVALID_DESCRIPTION : ENTASIS_STATUS_INVALID_ARGUMENT;
            ENTASIS_TEST_CHECK(entasis_world_step(world, 1.0f / 32, NULL) == expected);
            entasis_joint_reaction_t retained = {0};
            ENTASIS_TEST_CHECK(entasis_constraint_reaction(world, joints[index], &retained, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(memcmp(&retained, &committed, sizeof(retained)) == 0);
            ENTASIS_TEST_CHECK(entasis_trigger_events_drain(world, &parent, 1, &event_count, &event_required, NULL) == ENTASIS_STATUS_OK && event_count == 0);
            ENTASIS_TEST_CHECK(entasis_trigger_part_events_drain(world, &part, 1, &event_count, &event_required, NULL) == ENTASIS_STATUS_OK && event_count == 0);
            entasis_collider_part_pair_t previous_pair = {0};
            ENTASIS_TEST_CHECK(entasis_trigger_part_overlaps(world, &previous_pair, 1, &event_count, &event_required, NULL) == ENTASIS_STATUS_OK && event_count == 1);
            ENTASIS_TEST_CHECK(previous_pair.a.user_id == 231 || previous_pair.b.user_id == 231);

            uint64_t written = 99, required = 99;
            ENTASIS_TEST_CHECK(entasis_constraint_break_events_drain(world, NULL, 0, &written, &required, NULL) == ENTASIS_STATUS_OK && written == 0 && required == 0);
        }
        states[index].mode = REACTION_VALID;
        ENTASIS_TEST_CHECK(entasis_constraint_clear_break_limits(world, joints[index], NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_constraint_set_reaction_provider(world, 56, NULL, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_constraint_set_break_limits(world, joints[index], &limits, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        const entasis_joint_reaction_provider_t provider = {sizeof(provider), 1, read_joint_reaction, &states[index]};
        ENTASIS_TEST_CHECK(entasis_constraint_set_reaction_provider(world, 56, &provider, NULL) == ENTASIS_STATUS_OK);
        limits.force = 0;
        ENTASIS_TEST_CHECK(entasis_constraint_set_break_limits(world, joints[index], &limits, NULL) == ENTASIS_STATUS_OK);
        entasis_fixed_stepper_t stepper = entasis_fixed_stepper(1.0f / 32, 4);
        entasis_joint_break_update_result_t update = {0};
        ENTASIS_TEST_CHECK(entasis_joint_break_stepper_update(&stepper, world, 1.0f / 16, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, 0, &update, NULL) == ENTASIS_STATUS_CAPACITY_MISSING);
        ENTASIS_TEST_CHECK(update.completed_steps == 1 && update.break_required == 1 && update.pending == ENTASIS_JOINT_BREAK_PENDING_BREAK && stepper.accumulator == 1.0f / 32);
        ENTASIS_TEST_CHECK(entasis_world_disable_joint_breaks(world, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        entasis_joint_break_event_t event = {0};
        uint64_t count = 71;
        ENTASIS_TEST_CHECK(entasis_constraint_break_events_drain(world, &event, 1, &count, &count, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT && count == 71);
        ENTASIS_TEST_CHECK(entasis_joint_break_stepper_update(&stepper, world, 0, &event, 1, NULL, 0, NULL, 0, NULL, NULL, 0, &update, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(update.completed_steps == 1 && update.break_events_written == 1 && update.pending == 0 && stepper.accumulator == 0);
        ENTASIS_TEST_CHECK(event.constraint.value == joints[index].value && event.user_id == 700 + index && event.exceeded == ENTASIS_JOINT_BREAK_FORCE && event.reserved == 0);
        ENTASIS_TEST_CHECK(states[index].calls > 0 && states[index].errors == 0 && replacement.calls == 0);
        ENTASIS_TEST_CHECK(entasis_world_disable_joint_breaks(world, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_enable_joint_breaks(world, 1, NULL) == ENTASIS_STATUS_OK);
        entasis_body_handle_t body = {0};
        ENTASIS_TEST_CHECK(cc_body(world, 4, 0, &body) == 0);
        const float target = 2;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_add(world, 56, &body, 1, &target, sizeof(target), &joints[index], NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_constraint_set_break_limits(world, joints[index], &limits, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_destroy(world, NULL) == ENTASIS_STATUS_OK);
    }
    return 0;
}

int main(int argc, char **argv)
{
    const char *group = argc == 1 ? "all" : argv[1];
    if (argc > 2 || (strcmp(group, "all") != 0 && strcmp(group, "stages") != 0 &&
                     strcmp(group, "callbacks") != 0 && strcmp(group, "body-control") != 0 && strcmp(group, "joint-breaks") != 0))
    {
        return 2;
    }
    if (strcmp(group, "all") == 0 || strcmp(group, "joint-breaks") == 0)
    {
        ENTASIS_TEST_CHECK(test_joint_break_providers() == 0);
    }
    if (strcmp(group, "all") == 0 || strcmp(group, "body-control") == 0)
    {
        ENTASIS_TEST_CHECK(test_axis_lock_api() == 0);
        ENTASIS_TEST_CHECK(test_body_control_inputs_targets_and_guards() == 0);
    }
    if (strcmp(group, "all") == 0 || strcmp(group, "stages") == 0)
    {
        ENTASIS_TEST_CHECK(test_stages_scopes_scheduler_and_descriptor_copy() == 0);
    }
    if (strcmp(group, "all") == 0 || strcmp(group, "callbacks") == 0)
    {
        ENTASIS_TEST_CHECK(test_filtered_static_awakening() == 0);
        ENTASIS_TEST_CHECK(test_velocity_lanes_handles_and_failed_initialization() == 0);
        ENTASIS_TEST_CHECK(test_contact_manifold_material_and_child_propagation() == 0);
    }
    return 0;
}
