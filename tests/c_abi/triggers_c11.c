#include "entasis.h"
#include "test_support.h"
#include "generated_layout_asserts.h"
#include <string.h>

static entasis_rigid_pose_t trigger_pose(float x, float y, float z)
{
    const entasis_vector3_t p = {x, y, z};
    const entasis_quaternion_t q = {0, 0, 0, 1};
    return entasis_pose(p, q);
}
static int triggers_scene(int workers, int owned)
{
    entasis_world_t world = {0};
    entasis_world_description_t d = entasis_world_description_default();
    d.gravity.x = d.gravity.y = d.gravity.z = 0;
    d.threading.worker_count = workers;
    entasis_world_extensions_t ex = entasis_world_extensions_default();
    ex.allocation_scope = owned ? ENTASIS_ALLOCATION_ALL_OWNED : ENTASIS_ALLOCATION_LEGACY;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &d, &ex, NULL) == ENTASIS_STATUS_OK);
    entasis_trigger_configuration_t config = entasis_trigger_configuration_default();
    config.pair_capacity = 32;
    config.candidates_per_worker = 1;
    config.child_capacity = 128;
    config.struct_version = 0;
    ENTASIS_TEST_CHECK(entasis_world_enable_triggers(&world, &config, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    config.struct_version = 1;
    ENTASIS_TEST_CHECK(entasis_world_enable_triggers(&world, &config, NULL) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(1);
    entasis_shape_handle_t shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL) == ENTASIS_STATUS_OK);
    entasis_static_description_t sd = entasis_static_body(shape, trigger_pose(0, 0, 0), entasis_ccd_discrete());
    entasis_static_handle_t sensor = entasis_static_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &sd, ENTASIS_AWAKENING_NONE, &sensor, NULL) == ENTASIS_STATUS_OK);
    entasis_collidable_reference_t ref = {0};
    ENTASIS_TEST_CHECK(entasis_static_collidable_reference(&world, sensor, &ref, NULL) == ENTASIS_STATUS_OK);
    entasis_trigger_settings_t settings = {0};
    settings.user_id = 777;
    settings.stay = 1;
    ENTASIS_TEST_CHECK(entasis_trigger_set(&world, ref, &settings, NULL) == ENTASIS_STATUS_OK);
    settings.user_id = 888;
    entasis_trigger_settings_t copied = {0};
    ENTASIS_TEST_CHECK(entasis_trigger_get(&world, ref, &copied, NULL) == ENTASIS_STATUS_OK && copied.user_id == 777);
    entasis_body_velocity_t zero = {0};
    const entasis_body_description_t bd = entasis_body_kinematic(shape, trigger_pose(1, 0, 0), zero, entasis_body_activity(-1, 255));
    entasis_body_handle_t body = entasis_body_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &bd, &body, NULL) == ENTASIS_STATUS_OK);
    entasis_collidable_reference_t body_ref = {0};
    ENTASIS_TEST_CHECK(entasis_body_collidable_reference(&world, body, &body_ref, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_set_user_id(&world, body_ref, 12345, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, NULL) == ENTASIS_STATUS_OK);
    uint64_t written = 99, required = 99;
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&world, NULL, 0, &written, &required, NULL) == ENTASIS_STATUS_CAPACITY_MISSING && written == 0 && required == 1);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    entasis_trigger_event_t events[8];
    memset(events, 0, sizeof events);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&world, events, 8, &written, &required, NULL) == ENTASIS_STATUS_OK && written == 1 && events[0].kind == ENTASIS_TRIGGER_ENTER && events[0].pair.token_a != events[0].pair.token_b);
    ENTASIS_TEST_CHECK(events[0].pair.user_a == 777 && events[0].pair.user_b == 12345);
    const uint64_t token_a = events[0].pair.token_a, token_b = events[0].pair.token_b;
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_set(&world, ref, NULL, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_trigger_get(&world, ref, &copied, NULL) == ENTASIS_STATUS_OK);
    entasis_trigger_pair_t pairs[8];
    ENTASIS_TEST_CHECK(entasis_trigger_overlaps(&world, pairs, 8, &written, &required, NULL) == ENTASIS_STATUS_OK && written == 1);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&world, events, 8, &written, &required, NULL) == ENTASIS_STATUS_OK && written == 1 && events[0].kind == ENTASIS_TRIGGER_STAY && events[0].pair.token_a == token_a && events[0].pair.token_b == token_b);
    const entasis_rigid_pose_t away = trigger_pose(9, 0, 0);
    ENTASIS_TEST_CHECK(entasis_body_set_pose(&world, body, &away, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&world, events, 8, &written, &required, NULL) == ENTASIS_STATUS_OK && written == 1 && events[0].kind == ENTASIS_TRIGGER_EXIT);
    const entasis_rigid_pose_t near = trigger_pose(1, 0, 0);
    ENTASIS_TEST_CHECK(entasis_body_set_pose(&world, body, &near, NULL) == ENTASIS_STATUS_OK);
    entasis_fixed_stepper_t stepper = entasis_fixed_stepper(0.01f, 8);
    entasis_trigger_update_result_t update = {0};
    ENTASIS_TEST_CHECK(entasis_trigger_stepper_update(&stepper, &world, 0.025f, NULL, 0, &update, NULL) == ENTASIS_STATUS_CAPACITY_MISSING && update.completed_steps == 1 && update.notification_pending);
    ENTASIS_TEST_CHECK(entasis_trigger_stepper_update(&stepper, &world, 0, events, 8, &update, NULL) == ENTASIS_STATUS_OK && update.completed_steps == 1 && update.events_written == 2 && update.required == 0 && !update.notification_pending);
    ENTASIS_TEST_CHECK(entasis_trigger_remove(&world, ref, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_mark_filters_dirty(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_reserve(&world, &config, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_disable_triggers(&world, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_trigger_events_discard(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_disable_triggers(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
static int mixed_parts_scene(int workers, int owned, int nested)
{
    entasis_world_t world = {0};
    entasis_world_description_t d = entasis_world_description_default();
    d.gravity.x = d.gravity.y = d.gravity.z = 0;
    d.threading.worker_count = workers;
    entasis_world_extensions_t ex = entasis_world_extensions_default();
    ex.allocation_scope = owned ? ENTASIS_ALLOCATION_ALL_OWNED : ENTASIS_ALLOCATION_LEGACY;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &d, &ex, NULL) == ENTASIS_STATUS_OK);
    entasis_trigger_configuration_t tc = entasis_trigger_configuration_default();
    tc.pair_capacity = 16;
    tc.candidates_per_worker = 32;
    tc.child_capacity = 128;
    ENTASIS_TEST_CHECK(entasis_world_enable_triggers(&world, &tc, NULL) == ENTASIS_STATUS_OK);
    entasis_collider_part_configuration_t config = entasis_collider_part_configuration_default();
    config.instance_capacity = 8;
    config.part_capacity = 16;
    config.pair_capacity = 16;
    config.observations_per_worker = 64;
    config.event_subscription = ENTASIS_COLLIDER_PART_EVENTS_ENABLED;
    config.struct_version = 0;
    ENTASIS_TEST_CHECK(entasis_collider_parts_reserve(&world, &config, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    config.struct_version = 1;
    ENTASIS_TEST_CHECK(entasis_collider_parts_reserve(&world, &config, NULL) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(1);
    entasis_shape_handle_t shape = entasis_shape_handle_invalid(), compound = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t part_shape = shape;
    if (nested)
    {
        const entasis_compound_child_t inner = entasis_compound_child(shape, trigger_pose(0, 0, 0));
        ENTASIS_TEST_CHECK(entasis_shape_import_big_compound(&world, &inner, 1, &part_shape, NULL) == ENTASIS_STATUS_OK);
    }
    entasis_compound_child_t children[2] = {entasis_compound_child(part_shape, trigger_pose(0, 0, 0)), entasis_compound_child(part_shape, trigger_pose(3, 0, 0))};
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&world, children, 2, &compound, NULL) == ENTASIS_STATUS_OK);
    entasis_static_description_t sd = entasis_static_body(compound, trigger_pose(0, 0, 0), entasis_ccd_discrete());
    entasis_static_handle_t stat = entasis_static_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &sd, ENTASIS_AWAKENING_NONE, &stat, NULL) == ENTASIS_STATUS_OK);
    entasis_collidable_reference_t reference = {0};
    ENTASIS_TEST_CHECK(entasis_static_collidable_reference(&world, stat, &reference, NULL) == ENTASIS_STATUS_OK);
    entasis_collider_part_settings_t settings = {0};
    settings.role = ENTASIS_COLLIDER_PART_TRIGGER;
    settings.user_id = 712;
    entasis_collider_part_info_t info = {0};
    ENTASIS_TEST_CHECK(entasis_collider_part_set(&world, reference, 1, &settings, &info, NULL) == ENTASIS_STATUS_OK && info.identity.serial != 0);
    const uint64_t serial = info.identity.serial;
    ENTASIS_TEST_CHECK(entasis_trigger_set(&world, reference, NULL, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    settings.role = 255;
    ENTASIS_TEST_CHECK(entasis_collider_part_set(&world, reference, 1, &settings, &info, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_collider_part_get(&world, reference, 1, &info, NULL) == ENTASIS_STATUS_OK && info.identity.serial == serial && info.settings.role == ENTASIS_COLLIDER_PART_TRIGGER);
    entasis_body_velocity_t zero = {0};
    entasis_body_handle_t body = entasis_body_handle_invalid();
    entasis_body_description_t bd = entasis_body_kinematic(shape, trigger_pose(3.5f, 0, 0), zero, entasis_body_activity(-1, 255));
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &bd, &body, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collider_part_remove(&world, reference, 1, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_collider_part_get(&world, reference, 1, &info, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&world, NULL) == ENTASIS_STATUS_OK);
    uint64_t n = 0, required = 0;
    ENTASIS_TEST_CHECK(entasis_collider_parts(&world, reference, NULL, 0, &n, &required, NULL) == ENTASIS_STATUS_CAPACITY_MISSING && n == 0 && required == 1);
    ENTASIS_TEST_CHECK(entasis_collider_parts(&world, reference, &info, 1, &n, &required, NULL) == ENTASIS_STATUS_OK && n == 1 && info.identity.reserved == 0);
    entasis_fixed_stepper_t stepper = entasis_fixed_stepper(0.01f, 4);
    entasis_collider_part_update_result_t update = {0};
    entasis_trigger_event_t parent[8];
    entasis_collider_part_event_t parts[8];
    memset(parent, 0, sizeof parent);
    memset(parts, 0, sizeof parts);
    ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update(&stepper, &world, 0.02f, parent, 8, NULL, 0, &update, NULL) == ENTASIS_STATUS_CAPACITY_MISSING && update.completed_steps == 1 && update.part_required == 1 && update.events_written == 0 && (update.pending & ENTASIS_COLLIDER_PART_PENDING));
    ENTASIS_TEST_CHECK(entasis_world_disable_triggers(&world, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    // aliased count pointers must not acknowledge pending history
    n = 71;
    ENTASIS_TEST_CHECK(entasis_trigger_part_events_drain(&world, parts, 8, &n, &n, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT && n == 71);
    ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update(&stepper, &world, 0, parent, 8, parts, 8, &update, NULL) == ENTASIS_STATUS_OK && update.completed_steps == 1 && update.events_written == 1 && update.part_events_written == 1 && update.required == 0 && update.part_required == 0 && update.pending == 0);
    ENTASIS_TEST_CHECK(parts[0].kind == ENTASIS_TRIGGER_ENTER && parts[0].step == parent[0].step);
    const entasis_collider_part_endpoint_t endpoint = parts[0].pair.a.serial == serial ? parts[0].pair.a : parts[0].pair.b;
    ENTASIS_TEST_CHECK(endpoint.serial == serial && endpoint.user_id == 712 && endpoint.child_index == 1);
    entasis_collider_part_pair_t pair = {0};
    ENTASIS_TEST_CHECK(entasis_trigger_part_overlaps(&world, &pair, 1, &n, &required, NULL) == ENTASIS_STATUS_OK && n == 1);

    /* a separate contact history must be delivered before catch-up advances */
    entasis_contact_tracker_t tracker = {0};
    ENTASIS_TEST_CHECK(entasis_contact_tracker_init(&tracker, 0, NULL, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_bind(&tracker, &world, NULL, NULL) == ENTASIS_STATUS_OK);
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1, &inertia) == ENTASIS_STATUS_OK);
    bd = entasis_body_dynamic(shape, inertia, trigger_pose(1.5f, 0, 0), zero, entasis_body_activity(-1, 255));
    ENTASIS_TEST_CHECK(entasis_body_apply(&world, body, &bd, NULL) == ENTASIS_STATUS_OK);
    settings.role = ENTASIS_COLLIDER_PART_TRIGGER;
    settings.options = ENTASIS_COLLIDER_PART_STAY;
    ENTASIS_TEST_CHECK(entasis_collider_part_set(&world, reference, 1, &settings, &info, NULL) == ENTASIS_STATUS_OK);
    stepper = entasis_fixed_stepper(1.0f / 64, 4);
    entasis_collider_part_contact_update_result_t combined = {0};
    entasis_contact_event_t contacts[8];
    memset(contacts, 0, sizeof contacts);
    ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update_with_contacts(&stepper, &world, 2.0f / 64, parent, 8, parts, 8, &tracker, contacts, 8, &combined, NULL) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(combined.completed_steps == 1 && combined.contact_required == 1 && combined.contact_events_written == 0 && combined.pending == 7 && stepper.accumulator == 1.0f / 64);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_ensure_capacity(&tracker, 8, NULL) == ENTASIS_STATUS_OK);
    const float retained_time = stepper.accumulator;
    ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update_with_contacts(&stepper, &world, 0, parent, 8, parts, 8, &tracker, contacts, 8, (entasis_collider_part_contact_update_result_t *)parent, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(stepper.accumulator == retained_time);
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update_with_contacts(&stepper, &world, 0, parent, 8, parts, 8, &tracker, contacts, 8, &combined, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stepper.accumulator == retained_time);
    ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update_with_contacts(&stepper, &world, 0, parent, 8, NULL, 0, &tracker, contacts, 8, &combined, NULL) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(combined.completed_steps == 0 && combined.contact_events_written == 1 && combined.contact_required == 0 && combined.pending == 3 && contacts[0].kind == ENTASIS_CONTACT_EVENT_BEGIN);
    ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update_with_contacts(&stepper, &world, 0, parent, 8, parts, 8, &tracker, contacts, 8, &combined, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(combined.completed_steps == 1 && combined.contact_events_written == 1 && combined.events_written == 2 && combined.part_events_written == 2 && combined.pending == 0 && stepper.accumulator == 0);
    ENTASIS_TEST_CHECK(combined.required == 0 && combined.part_required == 0 && combined.contact_required == 0);
    ENTASIS_TEST_CHECK(parts[0].step == parent[0].step && parts[1].step == parent[1].step && parts[1].step == parts[0].step + 1);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_destroy(&tracker, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collider_part_remove(&world, reference, 1, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_part_events_drain(&world, parts, 8, &n, &required, NULL) == ENTASIS_STATUS_OK && n == 1 && parts[0].kind == ENTASIS_TRIGGER_EXIT);
    ENTASIS_TEST_CHECK(entasis_trigger_events_discard(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_part_events_discard(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_disable_triggers(&world, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    for (int owned = 0; owned < 2; ++owned)
        for (int w = 1; w <= 4; w *= 2)
            if (triggers_scene(w, owned) || mixed_parts_scene(w, owned, 0) || mixed_parts_scene(w, owned, 1))
                return 1;
    puts("TRIGGERS_OK");
    return 0;
}
