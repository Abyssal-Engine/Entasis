#include "extensions_fixture.h"
static int restitution_semantic(unsigned scope, unsigned iterations)
{
    entasis_world_t world = {0};
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = (entasis_vector3_t){0, -6, 0};
    description.damping.linear = description.damping.angular = 0;
    description.threading.worker_count = 2;
    description.solve.velocity_iterations = (int32_t)iterations;
    entasis_world_extensions_t extensions = entasis_world_extensions_default();
    extensions.allocation_scope = scope;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &description, &extensions, NULL) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(1);
    entasis_shape_handle_t shape = {0};
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1, &inertia) == ENTASIS_STATUS_OK);
    entasis_restitution_configuration_t config = entasis_restitution_configuration_default();
    config.fallback.coefficient = 0.5f;
    ENTASIS_TEST_CHECK(entasis_world_enable_restitution(&world, &config, NULL) == ENTASIS_STATUS_OK);
    entasis_body_handle_t bodies[9];
    for (unsigned i = 0; i < 9; ++i)
    {
        const entasis_static_description_t ground = entasis_static_body(shape, pose_at((float)i * 4, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t handle = {0};
        ENTASIS_TEST_CHECK(entasis_static_add(&world, &ground, ENTASIS_AWAKENING_NONE, &handle, NULL) == ENTASIS_STATUS_OK);
        const entasis_body_description_t body = entasis_body_dynamic(shape, inertia, pose_at((float)i * 4, 2, 0),
                                                                     entasis_velocity((entasis_vector3_t){0, -10, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
        ENTASIS_TEST_CHECK(entasis_body_add(&world, &body, &bodies[i], NULL) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, NULL) == ENTASIS_STATUS_OK);
    printf("restitution scope=%u iterations=%u\n", scope, iterations);
    for (unsigned i = 0; i < 9; ++i)
    {
        entasis_body_state_t body = {0};
        ENTASIS_TEST_CHECK(entasis_body_get(&world, bodies[i], &body, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(fabsf(body.velocity.linear.y - 5.046875f) < 0.0002f);
        printf("bounce=%u y=%08x vy=%08x\n", i, ef_bits(body.pose.position.y), ef_bits(body.velocity.linear.y));
    }
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}

static int body_control_semantic(unsigned scope, unsigned substeps)
{
    entasis_world_t world = {0};
    entasis_world_description_t desc = entasis_world_description_default();
    desc.gravity = (entasis_vector3_t){0, 0, 0};
    desc.damping.linear = desc.damping.angular = 0;
    desc.threading.worker_count = 2;
    desc.solve.substeps = (int32_t)substeps;
    entasis_world_extensions_t ext = entasis_world_extensions_default();
    ext.allocation_scope = scope;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &ext, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_enable_body_control(&world, NULL, NULL) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(1);
    entasis_shape_handle_t shape = {0};
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 2, &inertia) == ENTASIS_STATUS_OK);
    const entasis_body_description_t desc_body = entasis_body_dynamic(shape, inertia, pose_at(0, 0, 0), entasis_velocity((entasis_vector3_t){0, 0, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
    entasis_body_handle_t body = {0};
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &desc_body, &body, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add_force(&world, body, (entasis_vector3_t){8, 0, 0}, ENTASIS_BODY_INPUT_FORCE, ENTASIS_BODY_CONTROL_WAKE, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add_torque(&world, body, (entasis_vector3_t){0, 0, 2}, ENTASIS_BODY_INPUT_FORCE, ENTASIS_BODY_CONTROL_WAKE, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.25f, NULL) == ENTASIS_STATUS_OK);
    entasis_body_state_t value = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &value, NULL) == ENTASIS_STATUS_OK);
    printf("body-control scope=%u substeps=%u x=%08x vx=%08x wz=%08x\n", scope, substeps, ef_bits(value.pose.position.x), ef_bits(value.velocity.linear.x), ef_bits(value.velocity.angular.z));
    entasis_body_axis_lock_t lock = entasis_body_axis_lock_default(value.pose);
    lock.linear_axes = ENTASIS_BODY_LOCK_X;
    lock.angular_axes = ENTASIS_BODY_LOCK_Z;
    ENTASIS_TEST_CHECK(entasis_body_set_axis_lock(&world, body, &lock, NULL) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &value, NULL) == ENTASIS_STATUS_OK);
    printf("axis-lock scope=%u substeps=%u x=%08x vx=%08x wz=%08x\n", scope, substeps, ef_bits(value.pose.position.x), ef_bits(value.velocity.linear.x), ef_bits(value.velocity.angular.z));
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}

typedef enum semantic_delivery_t
{
    SEMANTIC_PARTS,
    SEMANTIC_PARTS_AND_BREAKS
} semantic_delivery_t;
static int mixed_contact_semantic(unsigned scope, unsigned custom_step, semantic_delivery_t delivery)
{
    entasis_world_t world = {0};
    entasis_world_description_t desc = entasis_world_description_default();
    desc.gravity = (entasis_vector3_t){0, 0, 0};
    desc.damping.linear = desc.damping.angular = 0;
    desc.threading.worker_count = 2;
    desc.solve.substeps = 2;
    entasis_world_extensions_t extensions = entasis_world_extensions_default();
    entasis_timestepper_t timestep = {sizeof(timestep), 1, NULL, ef_step};
    extensions.allocation_scope = scope;
    if (custom_step)
    {
        extensions.timestepper = &timestep;
    }
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &desc, &extensions, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_enable_triggers(&world, NULL, NULL) == ENTASIS_STATUS_OK);
    entasis_collider_part_configuration_t parts_config = entasis_collider_part_configuration_default();
    parts_config.event_subscription = ENTASIS_COLLIDER_PART_EVENTS_ENABLED;
    ENTASIS_TEST_CHECK(entasis_collider_parts_reserve(&world, &parts_config, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_enable_body_control(&world, NULL, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_enable_restitution(&world, NULL, NULL) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(1);
    entasis_shape_handle_t shape = {0}, compound = {0};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL) == ENTASIS_STATUS_OK);
    const entasis_compound_child_t children[2] = {entasis_compound_child(shape, pose_at(0, 0, 0)), entasis_compound_child(shape, pose_at(3, 0, 0))};
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&world, children, 2, &compound, NULL) == ENTASIS_STATUS_OK);
    const entasis_static_description_t ground = entasis_static_body(compound, pose_at(0, 0, 0), entasis_ccd_discrete());
    entasis_static_handle_t stat = {0};
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &ground, ENTASIS_AWAKENING_NONE, &stat, NULL) == ENTASIS_STATUS_OK);
    entasis_collidable_reference_t reference = {0};
    ENTASIS_TEST_CHECK(entasis_static_collidable_reference(&world, stat, &reference, NULL) == ENTASIS_STATUS_OK);
    const entasis_collider_part_settings_t settings = {52, ENTASIS_COLLIDER_PART_TRIGGER, ENTASIS_COLLIDER_PART_STAY, {0}};
    entasis_collider_part_info_t info = {0};
    ENTASIS_TEST_CHECK(entasis_collider_part_set(&world, reference, 1, &settings, &info, NULL) == ENTASIS_STATUS_OK);
    const entasis_restitution_settings_t bounce = {0.5f, 0};
    ENTASIS_TEST_CHECK(entasis_restitution_set(&world, reference, &bounce, NULL) == ENTASIS_STATUS_OK);
    entasis_body_inertia_t inertia = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1, &inertia) == ENTASIS_STATUS_OK);
    const entasis_body_description_t bd = entasis_body_dynamic(shape, inertia, pose_at(1.5f, 0, 0),
                                                               entasis_velocity((entasis_vector3_t){-1, 0, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
    entasis_body_handle_t body = {0};
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &bd, &body, NULL) == ENTASIS_STATUS_OK);
    entasis_collidable_reference_t body_reference = {0};
    ENTASIS_TEST_CHECK(entasis_body_collidable_reference(&world, body, &body_reference, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_restitution_set(&world, body_reference, &bounce, NULL) == ENTASIS_STATUS_OK);
    entasis_body_axis_lock_t lock = entasis_body_axis_lock_default(bd.pose);
    lock.linear_axes = ENTASIS_BODY_LOCK_Z;
    ENTASIS_TEST_CHECK(entasis_body_set_axis_lock(&world, body, &lock, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add_force(&world, body, (entasis_vector3_t){0, 0, 8}, ENTASIS_BODY_INPUT_FORCE, ENTASIS_BODY_CONTROL_WAKE, NULL) == ENTASIS_STATUS_OK);
    entasis_constraint_handle_t joint = {0};
    if (delivery == SEMANTIC_PARTS_AND_BREAKS)
    {
        ENTASIS_TEST_CHECK(entasis_world_enable_joint_breaks(&world, 1, NULL) == ENTASIS_STATUS_OK);
        const entasis_body_description_t independent = entasis_body_shapeless(inertia, pose_at(10, 0, 0), entasis_velocity((entasis_vector3_t){0, 0, 0}, (entasis_vector3_t){0, 0, 0}), entasis_body_activity(-1, 255));
        entasis_body_handle_t joint_body = {0};
        ENTASIS_TEST_CHECK(entasis_body_add(&world, &independent, &joint_body, NULL) == ENTASIS_STATUS_OK);
        const entasis_one_body_linear_motor_t motor = {{0, 0, 0}, {10, 0, 0}, entasis_motor_settings(1000, 0.01f)};
        ENTASIS_TEST_CHECK(entasis_constraint_add(&world, ENTASIS_CONSTRAINT_TYPE_ONE_BODY_LINEAR_MOTOR, &joint_body, 1, &motor, sizeof(motor), &joint, NULL) == ENTASIS_STATUS_OK);
        const entasis_joint_break_limits_t limits = {ENTASIS_JOINT_BREAK_FORCE, 0, 0, 0, 417};
        ENTASIS_TEST_CHECK(entasis_constraint_set_break_limits(&world, joint, &limits, NULL) == ENTASIS_STATUS_OK);
    }
    entasis_contact_tracker_t tracker = {0};
    ENTASIS_TEST_CHECK(entasis_contact_tracker_init(&tracker, 8, NULL, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_contact_tracker_bind(&tracker, &world, NULL, NULL) == ENTASIS_STATUS_OK);
    entasis_trigger_event_t parent[8];
    entasis_collider_part_event_t parts[8];
    entasis_contact_event_t contacts[8];
    entasis_fixed_stepper_t stepper = entasis_fixed_stepper(1.0f / 64, 4);
    entasis_collider_part_contact_update_result_t result = {0};
    if (delivery == SEMANTIC_PARTS_AND_BREAKS)
    {
        entasis_joint_break_event_t breaks[2] = {0};
        entasis_joint_break_update_result_t combined = {0};
        ENTASIS_TEST_CHECK(entasis_joint_break_stepper_update(&stepper, &world, 3.0f / 64, NULL, 0, parent, 8, parts, 8, &tracker, contacts, 8, &combined, NULL) == ENTASIS_STATUS_CAPACITY_MISSING);
        ENTASIS_TEST_CHECK(combined.completed_steps == 1 && combined.break_required == 1 && (combined.pending & ENTASIS_JOINT_BREAK_PENDING_BREAK) != 0 && stepper.accumulator == 2.0f / 64);
        const int32_t parent_prefix = combined.events_written, part_prefix = combined.part_events_written, contact_prefix = combined.contact_events_written;
        ENTASIS_TEST_CHECK(entasis_joint_break_stepper_update(&stepper, &world, 0, breaks, 2, parent + parent_prefix, 8 - parent_prefix, parts + part_prefix, 8 - part_prefix, &tracker, contacts + contact_prefix, 8 - contact_prefix, &combined, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(combined.completed_steps == 2 && combined.break_events_written == 1 && combined.pending == 0 && stepper.accumulator == 0);
        ENTASIS_TEST_CHECK(breaks[0].constraint.value == joint.value && breaks[0].user_id == 417 && breaks[0].reaction.maximum_force > 0 && breaks[0].reaction.sample.substep_duration == 1.0f / 128);
        result.completed_steps = combined.completed_steps + 1;
        result.events_written = combined.events_written + parent_prefix;
        result.part_events_written = combined.part_events_written + part_prefix;
        result.contact_events_written = combined.contact_events_written + contact_prefix;
        printf("joint-stream scope=%u custom=%u step=%llu lifetime=%llu force=%08x torque=%08x duration=%08x\n", scope, custom_step,
               (unsigned long long)breaks[0].reaction.step, (unsigned long long)breaks[0].lifetime,
               ef_bits(breaks[0].reaction.maximum_force), ef_bits(breaks[0].reaction.maximum_torque), ef_bits(breaks[0].reaction.sample.substep_duration));
    }
    else
    {
        ENTASIS_TEST_CHECK(entasis_collider_part_stepper_update_with_contacts(&stepper, &world, 3.0f / 64, parent, 8, parts, 8, &tracker, contacts, 8, &result, NULL) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(result.completed_steps == 3 && result.events_written == 3 && result.part_events_written == 3 && result.contact_events_written == 3 && result.pending == 0);
    entasis_body_state_t state = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &state, NULL) == ENTASIS_STATUS_OK);
    printf("mixed scope=%u custom=%u steps=%d parent=%d parts=%d contacts=%d x=%08x vx=%08x z=%08x vz=%08x\n", scope, custom_step,
           result.completed_steps, result.events_written, result.part_events_written, result.contact_events_written,
           ef_bits(state.pose.position.x), ef_bits(state.velocity.linear.x), ef_bits(state.pose.position.z), ef_bits(state.velocity.linear.z));
    for (unsigned i = 0; i < 3; ++i)
    {
        ENTASIS_TEST_CHECK(parent[i].step == parts[i].step && parts[i].step == i + 1);
        printf("mixed-event=%u parent=%u part=%u contact=%u normal=%08x flags=%u\n", i, parent[i].kind, parts[i].kind, contacts[i].kind, ef_bits(contacts[i].normal.x), parts[i].pair.flags);
    }
    ENTASIS_TEST_CHECK(entasis_contact_tracker_destroy(&tracker, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    for (unsigned scope = 0; scope < 2; ++scope)
        for (unsigned step = 0; step < 2; ++step)
            ENTASIS_TEST_CHECK(ef_run(scope, step, NULL, 1) == 0);
    for (unsigned scope = 0; scope < 2; ++scope)
        for (unsigned iterations = 1; iterations <= 4; iterations += 3)
            ENTASIS_TEST_CHECK(restitution_semantic(scope, iterations) == 0);
    for (unsigned scope = 0; scope < 2; ++scope)
        for (unsigned substeps = 1; substeps <= 4; substeps += 3)
            ENTASIS_TEST_CHECK(body_control_semantic(scope, substeps) == 0);
    for (unsigned scope = 0; scope < 2; ++scope)
        for (unsigned custom = 0; custom < 2; ++custom)
            for (unsigned delivery = 0; delivery < 2; ++delivery)
                ENTASIS_TEST_CHECK(mixed_contact_semantic(scope, custom, (semantic_delivery_t)delivery) == 0);
    return 0;
}
