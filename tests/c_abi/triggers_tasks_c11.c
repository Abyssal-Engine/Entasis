#include "compound_task_fixture.h"

static int test_trigger_custom_geometry(void)
{
    for (unsigned scenario = 0; scenario < 6; ++scenario)
    {
        const unsigned kind = scenario % 3;
        task_fixture_t f = {0};
        ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0);
        ENTASIS_TEST_CHECK(task_fixture_bind(&f) == 0);
        entasis_shape_handle_t targets[3];
        ENTASIS_TEST_CHECK(compound_targets(&f, targets) == 0);
        if (scenario >= 3)
        {
            /* existing C task callbacks serve nested native and mesh leaves */
            for (unsigned depth = 0; depth < 2; ++depth)
            {
                const entasis_compound_child_t child = entasis_compound_child(targets[kind], pose_at(0, 0, 0));
                entasis_shape_handle_t parent = {0};
                ENTASIS_TEST_CHECK(entasis_shape_import_compound(&f.world, &child, 1, &parent, NULL) == ENTASIS_STATUS_OK);
                targets[kind] = parent;
            }
        }
        entasis_trigger_configuration_t tc = entasis_trigger_configuration_default();
        tc.pair_capacity = 32;
        tc.candidates_per_worker = 64;
        tc.child_capacity = 256;
        ENTASIS_TEST_CHECK(entasis_world_enable_triggers(&f.world, &tc, NULL) == ENTASIS_STATUS_OK);
        entasis_static_description_t sd = entasis_static_body(targets[kind], pose_at(0, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t sensor;
        ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &sensor, NULL) == ENTASIS_STATUS_OK);
        entasis_collidable_reference_t ref;
        ENTASIS_TEST_CHECK(entasis_static_collidable_reference(&f.world, sensor, &ref, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_trigger_set(&f.world, ref, NULL, NULL) == ENTASIS_STATUS_OK);
        entasis_body_velocity_t zero = {0};
        entasis_body_description_t bd = entasis_body_kinematic(f.custom, pose_at(0, 0.5f, 0), zero, entasis_body_activity(-1, 255));
        entasis_body_handle_t body;
        ENTASIS_TEST_CHECK(entasis_body_add(&f.world, &bd, &body, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_step(&f.world, 0.01f, NULL) == ENTASIS_STATUS_OK);
        entasis_trigger_event_t events[8];
        uint64_t count = 0, required = 0;
        ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&f.world, events, 8, &count, &required, NULL) == ENTASIS_STATUS_OK && count == 1 && events[0].kind == ENTASIS_TRIGGER_ENTER);
        ENTASIS_TEST_CHECK(f.task.wide > 0 && f.task.errors == 0);
        const entasis_rigid_pose_t separated_pose = pose_at(0, 20, 0);
        ENTASIS_TEST_CHECK(entasis_body_set_pose(&f.world, body, &separated_pose, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_step(&f.world, 0.01f, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&f.world, events, 8, &count, &required, NULL) == ENTASIS_STATUS_OK && count == 1 && events[0].kind == ENTASIS_TRIGGER_EXIT);
        ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    }
    return 0;
}
typedef struct trigger_batch_callback_t
{
    task_state_t *task;
    unsigned calls, fail_at;
} trigger_batch_callback_t;
static entasis_status_t ENTASIS_CALL trigger_batch_scalar(void *ctx,
                                                          const entasis_collision_task_input_t *input, entasis_task_access_t access,
                                                          entasis_convex_contact_manifold_t *output)
{
    return task_scalar(((trigger_batch_callback_t *)ctx)->task, input, access, output);
}
static entasis_status_t ENTASIS_CALL trigger_batch_wide(void *ctx,
                                                        const entasis_collision_task_wide_input_t *input, entasis_task_access_t access,
                                                        entasis_convex_manifold_wide_t *output)
{
    trigger_batch_callback_t *state = (trigger_batch_callback_t *)ctx;
    if (++state->calls == state->fail_at)
        return ENTASIS_STATUS_INVALID_DESCRIPTION;
    return task_wide(state->task, input, access, output);
}
static int test_trigger_streamed_custom_failure(void)
{
    enum
    {
        PAIRS = 1025
    };
    task_fixture_t f = {0};
    ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0);
    trigger_batch_callback_t callback = {&f.task, 0, 80};
    entasis_collision_task_registration_t registration = task_collision_registration(&f);
    registration.user_context = &callback;
    registration.test = trigger_batch_scalar;
    registration.wide_test = trigger_batch_wide;
    int32_t task_id = -1;
    ENTASIS_TEST_CHECK(entasis_collision_task_register(&f.world, &registration, &task_id, NULL) == ENTASIS_STATUS_OK);
    entasis_trigger_configuration_t c = entasis_trigger_configuration_default();
    c.pair_capacity = PAIRS;
    c.candidates_per_worker = PAIRS;
    c.child_capacity = 128;
    ENTASIS_TEST_CHECK(entasis_world_enable_triggers(&f.world, &c, NULL) == ENTASIS_STATUS_OK);
    for (int i = 0; i < PAIRS; ++i)
    {
        entasis_static_description_t d = entasis_static_body(f.sphere, pose_at((float)(6 * i), 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t sensor, visitor;
        ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &d, ENTASIS_AWAKENING_NONE, &sensor, NULL) == ENTASIS_STATUS_OK);
        d = entasis_static_body(f.custom, pose_at((float)(6 * i) + 1, 0, 0), entasis_ccd_discrete());
        ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &d, ENTASIS_AWAKENING_NONE, &visitor, NULL) == ENTASIS_STATUS_OK);
        entasis_collidable_reference_t ref;
        ENTASIS_TEST_CHECK(entasis_static_collidable_reference(&f.world, sensor, &ref, NULL) == ENTASIS_STATUS_OK);
        const entasis_trigger_settings_t settings = {.user_id = (uint64_t)i + 1, .stay = 1, .static_static = 1};
        ENTASIS_TEST_CHECK(entasis_trigger_set(&f.world, ref, &settings, NULL) == ENTASIS_STATUS_OK);
    }
    entasis_trigger_event_t *events = (entasis_trigger_event_t *)calloc(PAIRS, sizeof(*events));
    entasis_trigger_pair_t *pairs = (entasis_trigger_pair_t *)calloc(PAIRS, sizeof(*pairs));
    ENTASIS_TEST_CHECK(events && pairs);
    uint64_t count = 99, required = 99;
    /* error after at least one chunk completed must not publish a partial batch */
    ENTASIS_TEST_CHECK(entasis_world_step(&f.world, 0.01f, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && callback.calls == 80);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&f.world, events, PAIRS, &count, &required, NULL) == ENTASIS_STATUS_OK && count == 0);
    ENTASIS_TEST_CHECK(entasis_trigger_overlaps(&f.world, pairs, PAIRS, &count, &required, NULL) == ENTASIS_STATUS_OK && count == 0);
    callback.calls = 0;
    callback.fail_at = 0;
    ENTASIS_TEST_CHECK(entasis_world_step(&f.world, 0.01f, NULL) == ENTASIS_STATUS_OK && callback.calls > 80);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&f.world, events, PAIRS - 1, &count, &required, NULL) == ENTASIS_STATUS_CAPACITY_MISSING && count == 0 && required == PAIRS);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&f.world, events, PAIRS, &count, &required, NULL) == ENTASIS_STATUS_OK && count == PAIRS);
    for (int i = 0; i < PAIRS; ++i)
    {
        ENTASIS_TEST_CHECK(events[i].kind == ENTASIS_TRIGGER_ENTER);
        pairs[i] = events[i].pair;
    }
    const entasis_trigger_pair_t first = pairs[0], last = pairs[PAIRS - 1];
    ENTASIS_TEST_CHECK(entasis_trigger_mark_filters_dirty(&f.world, NULL) == ENTASIS_STATUS_OK);
    callback.calls = 0;
    callback.fail_at = 80;
    ENTASIS_TEST_CHECK(entasis_world_step(&f.world, 0.01f, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && callback.calls == 80);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&f.world, events, PAIRS, &count, &required, NULL) == ENTASIS_STATUS_OK && count == 0);
    ENTASIS_TEST_CHECK(entasis_trigger_overlaps(&f.world, pairs, PAIRS, &count, &required, NULL) == ENTASIS_STATUS_OK && count == PAIRS);
    ENTASIS_TEST_CHECK(memcmp(&first, &pairs[0], sizeof first) == 0 && memcmp(&last, &pairs[PAIRS - 1], sizeof last) == 0);
    callback.calls = 0;
    callback.fail_at = 0;
    ENTASIS_TEST_CHECK(entasis_world_step(&f.world, 0.01f, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&f.world, events, PAIRS, &count, &required, NULL) == ENTASIS_STATUS_OK && count == PAIRS);
    for (int i = 0; i < PAIRS; ++i)
        ENTASIS_TEST_CHECK(events[i].kind == ENTASIS_TRIGGER_STAY && memcmp(&events[i].pair, &pairs[i], sizeof pairs[i]) == 0);
    ENTASIS_TEST_CHECK(task_count(&f.task.errors) == 0);
    free(pairs);
    free(events);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
int main(void)
{
    if (test_trigger_custom_geometry() || test_trigger_streamed_custom_failure())
        return 1;
    puts("TRIGGER_CUSTOM_GEOMETRY_OK");
    return 0;
}
