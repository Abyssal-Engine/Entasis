#include "entasis.h"
#include "test_support.h"
#include "generated_layout_asserts.h"
#include <type_traits>
static_assert(std::is_standard_layout_v<entasis_trigger_event_t>);
static_assert(std::is_trivially_copyable_v<entasis_trigger_event_t>);
int main()
{
    entasis_world_t world{};
    entasis_world_description_t d = entasis_world_description_default();
    d.gravity = {0, 0, 0};
    entasis_world_extensions_t ext = entasis_world_extensions_default();
    ext.allocation_scope = ENTASIS_ALLOCATION_ALL_OWNED;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &d, &ext, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_enable_triggers(&world, nullptr, nullptr) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(1);
    entasis_shape_handle_t shape{};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, nullptr) == ENTASIS_STATUS_OK);
    const entasis_rigid_pose_t pose = entasis_pose({0, 0, 0}, {0, 0, 0, 1});
    entasis_static_description_t sd = entasis_static_body(shape, pose, entasis_ccd_discrete());
    entasis_static_handle_t a{}, b{};
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &sd, ENTASIS_AWAKENING_NONE, &a, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &sd, ENTASIS_AWAKENING_NONE, &b, nullptr) == ENTASIS_STATUS_OK);
    entasis_collidable_reference_t reference{};
    ENTASIS_TEST_CHECK(entasis_static_collidable_reference(&world, a, &reference, nullptr) == ENTASIS_STATUS_OK);
    entasis_trigger_settings_t settings{};
    settings.static_static = 1;
    settings.user_id = 42;
    ENTASIS_TEST_CHECK(entasis_trigger_set(&world, reference, &settings, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, nullptr) == ENTASIS_STATUS_OK);
    entasis_trigger_event_t event{};
    uint64_t written{}, required{};
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&world, &event, 1, &written, &required, nullptr) == ENTASIS_STATUS_OK && written == 1 && event.kind == ENTASIS_TRIGGER_ENTER);
    ENTASIS_TEST_CHECK(event.pair.reserved == 0 && (event.pair.flags & ENTASIS_TRIGGER_PAIR_A) != 0);
    ENTASIS_TEST_CHECK(entasis_trigger_remove(&world, reference, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&world, &event, 1, &written, &required, nullptr) == ENTASIS_STATUS_OK && written == 1 && event.kind == ENTASIS_TRIGGER_EXIT);
    entasis_collider_part_configuration_t config = entasis_collider_part_configuration_default();
    config.event_subscription = ENTASIS_COLLIDER_PART_EVENTS_ENABLED;
    ENTASIS_TEST_CHECK(entasis_collider_parts_reserve(&world, &config, nullptr) == ENTASIS_STATUS_OK);
    entasis_collider_part_settings_t part{};
    part.role = ENTASIS_COLLIDER_PART_TRIGGER;
    part.options = ENTASIS_COLLIDER_PART_STATIC_STATIC;
    part.user_id = 87;
    entasis_collider_part_info_t info{};
    ENTASIS_TEST_CHECK(entasis_collider_part_set(&world, reference, 0, &part, &info, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.identity.serial != 0 && info.settings.user_id == 87);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, nullptr) == ENTASIS_STATUS_OK);
    entasis_collider_part_event_t part_event{};
    ENTASIS_TEST_CHECK(entasis_trigger_part_events_drain(&world, nullptr, 0, &written, &required, nullptr) == ENTASIS_STATUS_CAPACITY_MISSING && written == 0 && required == 1);
    ENTASIS_TEST_CHECK(entasis_trigger_events_drain(&world, &event, 1, &written, &required, nullptr) == ENTASIS_STATUS_OK && written == 1);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.01f, nullptr) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_trigger_part_events_drain(&world, &part_event, 1, &written, &required, nullptr) == ENTASIS_STATUS_OK && written == 1);
    ENTASIS_TEST_CHECK(part_event.kind == ENTASIS_TRIGGER_ENTER && part_event.pair.reserved == 0);
    ENTASIS_TEST_CHECK(part_event.pair.a.user_id == 87 || part_event.pair.b.user_id == 87);
    static_assert(std::is_standard_layout_v<entasis_collider_part_contact_update_result_t>);
    static_assert(std::is_trivially_copyable_v<entasis_collider_part_contact_update_result_t>);
    entasis_contact_tracker_t tracker{};
    ENTASIS_TEST_CHECK(entasis_contact_tracker_init(&tracker, 8, nullptr, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_bind(&tracker, &world, nullptr, nullptr) == ENTASIS_STATUS_OK);
    entasis_fixed_stepper_t stepper = entasis_fixed_stepper(1.0f / 64, 4);
    entasis_collider_part_contact_update_result_t update{};
    update.reserved[0] = update.reserved[1] = update.reserved[2] = 0xcd;
    // no new transitions or physical contacts: empty batches acknowledge without
    // forcing the application to drain artificial events between catch-up steps
    ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update_with_contacts(&stepper, &world, 2.0f / 64,
                                                                          nullptr, 0, nullptr, 0, &tracker, nullptr, 0, &update, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(update.completed_steps == 2 && update.events_written == 0 &&
                       update.part_events_written == 0 && update.contact_events_written == 0 && update.pending == 0);
    ENTASIS_TEST_CHECK(update.required == 0 && update.part_required == 0 && update.contact_required == 0);
    ENTASIS_TEST_CHECK(update.reserved[0] == 0 && update.reserved[1] == 0 && update.reserved[2] == 0);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_destroy(&tracker, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, nullptr) == ENTASIS_STATUS_OK);
    puts("TRIGGERS_CPP20_OK");
    return 0;
}
