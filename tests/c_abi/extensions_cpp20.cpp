#include "entasis.h"
#include "generated_layout_asserts.h"
#include "test_support.h"
#include <cstdint>
#include <cmath>
#include <cstring>
#include <type_traits>

static_assert(alignof(entasis_f32x8_t) == 32);
static_assert(alignof(entasis_body_velocity_wide_t) == 32);
static_assert(std::is_standard_layout_v<entasis_pose_integration_view_t>);

struct CallbackState
{
    unsigned initialized{}, disposed{}, calls{}, errors{};
    entasis_body_handle_t body{};
};
static entasis_status_t ENTASIS_CALL initialize(void *raw, entasis_world_t)
{
    ++static_cast<CallbackState *>(raw)->initialized;
    return ENTASIS_STATUS_OK;
}
static entasis_status_t ENTASIS_CALL prepare(void *, float dt)
{
    return static_cast<entasis_status_t>(dt > 0 ? ENTASIS_STATUS_OK : ENTASIS_STATUS_INVALID_ARGUMENT);
}
static void ENTASIS_CALL integrate(void *raw, const entasis_pose_integration_view_t *view)
{
    CallbackState &state = *static_cast<CallbackState *>(raw);
    ++state.calls;
    if (reinterpret_cast<std::uintptr_t>(view->velocity) % 32 != 0)
        ++state.errors;
    for (unsigned lane = 0; lane < 8; ++lane)
    {
        if (view->active_mask->lanes[lane] == 0)
            continue;
        if (view->body_handles[lane].value != state.body.value)
            ++state.errors;
        view->velocity->linear.y.lanes[lane] = 2.0f;
    }
}
static void ENTASIS_CALL dispose(void *raw)
{
    ++static_cast<CallbackState *>(raw)->disposed;
}
static int restitution_test()
{
    entasis_world_description_t d = entasis_world_description_default();
    d.gravity = {0, 0, 0};
    d.damping.linear = d.damping.angular = 0;
    d.threading.worker_count = 2;
    entasis_world_t world{};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &d, nullptr) == ENTASIS_STATUS_OK);
    const entasis_sphere_t sphere = entasis_sphere(1);
    entasis_shape_handle_t shape{};
    entasis_body_inertia_t inertia{};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &sphere, 1, &inertia) == ENTASIS_STATUS_OK);
    const entasis_static_description_t ground = entasis_static_body(shape, entasis_pose({0, 0, 0}, {0, 0, 0, 1}), entasis_ccd_discrete());
    entasis_static_handle_t sh{};
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &ground, ENTASIS_AWAKENING_NONE, &sh, nullptr) == ENTASIS_STATUS_OK);
    const entasis_body_description_t body = entasis_body_dynamic(shape, inertia, entasis_pose({0, 2, 0}, {0, 0, 0, 1}), entasis_velocity({0, -10, 0}, {0, 0, 0}), entasis_body_activity(-1, 255));
    entasis_body_handle_t bh{};
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body, &bh, nullptr) == ENTASIS_STATUS_OK);
    entasis_restitution_configuration_t config = entasis_restitution_configuration_default();
    config.fallback = {0.75f, 1};
    ENTASIS_TEST_CHECK(entasis_world_enable_restitution(&world, &config, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, nullptr) == ENTASIS_STATUS_OK);
    entasis_body_state_t state{};
    ENTASIS_TEST_CHECK(entasis_body_get(&world, bh, &state, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(std::fabs(state.velocity.linear.y - 7.5f) < 0.0002f);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, nullptr) == ENTASIS_STATUS_OK);
    return 0;
}

static int body_controls_cpp()
{
    entasis_world_t world{};
    entasis_world_description_t desc = entasis_world_description_default();
    desc.gravity = {};
    desc.damping = {};
    desc.threading.worker_count = 1;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &desc, nullptr) == ENTASIS_STATUS_OK);
    entasis_body_control_configuration_t config = entasis_body_control_configuration_default();
    ENTASIS_TEST_CHECK(entasis_world_enable_body_control(&world, &config, nullptr) == ENTASIS_STATUS_OK);
    const entasis_sphere_t shape_data = entasis_sphere(1);
    entasis_shape_handle_t shape{};
    entasis_body_inertia_t inertia{};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &shape_data, &shape, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_inertia(ENTASIS_SHAPE_TYPE_SPHERE, &shape_data, 2, &inertia) == ENTASIS_STATUS_OK);
    const entasis_body_description_t value = entasis_body_dynamic(shape, inertia, entasis_pose({}, {0, 0, 0, 1}), entasis_velocity({}, {}), entasis_body_activity(-1, 255));
    entasis_body_handle_t body{};
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &value, &body, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add_force(&world, body, {8, 0, 0}, ENTASIS_BODY_INPUT_FORCE, ENTASIS_BODY_CONTROL_WAKE, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.5f, nullptr) == ENTASIS_STATUS_OK);
    entasis_body_state_t actual{};
    ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &actual, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(std::fabs(actual.velocity.linear.x - 2) < 0.00001f);
    entasis_body_axis_lock_t lock = entasis_body_axis_lock_default(actual.pose);
    lock.linear_axes = ENTASIS_BODY_LOCK_X;
    lock.angular_axes = ENTASIS_BODY_LOCK_Z;
    ENTASIS_TEST_CHECK(entasis_body_set_axis_lock(&world, body, &lock, nullptr) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 32; ++i)
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get(&world, body, &actual, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(std::fabs(actual.pose.position.x - lock.reference.position.x) < 0.001f);
    entasis_body_axis_lock_t read{};
    ENTASIS_TEST_CHECK(entasis_body_get_axis_lock(&world, body, &read, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(read.linear_axes == lock.linear_axes && read.angular_axes == lock.angular_axes);
    ENTASIS_TEST_CHECK(entasis_world_disable_body_control(&world, nullptr) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, nullptr) == ENTASIS_STATUS_OK);
    return 0;
}

static int callback_composition()
{
    CallbackState state{};
    entasis_velocity_callbacks_t callbacks{};
    callbacks.struct_size = sizeof(callbacks);
    callbacks.struct_version = 1;
    callbacks.user_context = &state;
    callbacks.initialize = initialize;
    callbacks.prepare = prepare;
    callbacks.integrate = integrate;
    callbacks.dispose = dispose;
    entasis_world_extensions_t extensions = entasis_world_extensions_default();
    extensions.velocity_callbacks = &callbacks;
    entasis_world_description_t description = entasis_world_description_default();
    description.threading.worker_count = 1;
    description.capacity.bodies = 16;
    description.capacity.constraints = 32;
    description.capacity.statics = 16;
    description.capacity.shapes_per_type = 16;
    description.capacity.pairs = 64;
    description.capacity.broad_phase_candidates = 64;
    entasis_world_t world{};
    entasis_diagnostic_t diagnostic{};
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &description, &extensions, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_inertia_t inertia{};
    inertia.inverse_mass = 1;
    const entasis_body_description_t body = entasis_body_shapeless(inertia,
                                                                   entasis_pose({0, 0, 0}, {0, 0, 0, 1}), entasis_velocity({0, 0, 0}, {0, 0, 0}), entasis_body_activity(-1, 255));
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body, &state.body, &diagnostic) == ENTASIS_STATUS_OK);
    callbacks.integrate = nullptr;
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_state_t result{};
    ENTASIS_TEST_CHECK(entasis_body_get(&world, state.body, &result, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(result.velocity.linear.y == 2.0f && state.calls > 0 && state.errors == 0);
    ENTASIS_TEST_CHECK(entasis_world_enable_body_control(&world, nullptr, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_damping_t damping{2, 0, ENTASIS_BODY_DAMPING_OVERRIDE};
    ENTASIS_TEST_CHECK(entasis_body_set_damping(&world, state.body, &damping, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    damping.mode = ENTASIS_BODY_DAMPING_ADDITIONAL;
    ENTASIS_TEST_CHECK(entasis_body_set_damping(&world, state.body, &damping, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_add_force(&world, state.body, {1, 0, 0}, ENTASIS_BODY_INPUT_FORCE, ENTASIS_BODY_CONTROL_WAKE, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 64, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_get(&world, state.body, &result, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(std::fabs(result.velocity.linear.y - 2 * std::exp(-2.0f / 64)) < 0.00001f && state.errors == 0);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.initialized == 1 && state.disposed == 1);
    return 0;
}

struct joint_provider_state_t
{
    entasis_world_t *world;
    unsigned calls;
};
static entasis_status_t ENTASIS_CALL joint_validate_cpp(void *, entasis_constraint_type_id_t, const void *description, uint32_t size)
{
    return static_cast<entasis_status_t>(size == sizeof(float) && std::isfinite(*static_cast<const float *>(description)) ? ENTASIS_STATUS_OK : ENTASIS_STATUS_INVALID_ARGUMENT);
}
static void ENTASIS_CALL joint_kernel_cpp(void *, const entasis_constraint_kernel_view_t *view)
{
    if (view->phase != ENTASIS_CONSTRAINT_KERNEL_SOLVE)
        return;
    for (unsigned lane = 0; lane < 8; ++lane)
    {
        if (view->active_mask->lanes[lane] != 0)
        {
            view->impulses[0].lanes[lane] = view->prestep[0].lanes[lane];
            view->bodies[0].linear_velocity->x.lanes[lane] = view->prestep[0].lanes[lane];
        }
    }
}
static entasis_status_t ENTASIS_CALL joint_provider_cpp(void *raw, const entasis_joint_reaction_provider_input_t *input, entasis_joint_impulse_wrench_t *output)
{
    joint_provider_state_t *state = static_cast<joint_provider_state_t *>(raw);
    ++state->calls;
    if (input->body_count != 1 || input->impulse_count != 1 || input->description_size != sizeof(float) ||
        input->substep_duration != 1.0f / 64 || input->impulses[0] != 2 || input->reserved != 0 ||
        entasis_world_clear(state->world, nullptr) != ENTASIS_STATUS_INVALID_ARGUMENT)
        return ENTASIS_STATUS_INVALID_ARGUMENT;
    output[0] = {{input->impulses[0], 0, 0}, {0, 0, 0}};
    return ENTASIS_STATUS_OK;
}
static int joint_breaks_cpp()
{
    entasis_world_t world{};
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = {};
    description.damping = {};
    description.threading.worker_count = 2;
    description.solve.substeps = 1;
    entasis_world_extensions_t extensions = entasis_world_extensions_default();
    extensions.allocation_scope = ENTASIS_ALLOCATION_ALL_OWNED;
    ENTASIS_TEST_CHECK(entasis_world_init_extended(&world, &description, &extensions, nullptr) == ENTASIS_STATUS_OK);
    joint_provider_state_t state{&world, 0};
    entasis_custom_constraint_registration_t registration = entasis_custom_constraint_registration_default();
    registration.type_id = 56;
    registration.body_count = 1;
    registration.description_size = sizeof(float);
    registration.prestep_bundle_size = registration.impulse_bundle_size = 32;
    registration.initial_access[0] = ENTASIS_BODY_ACCESS_ALL;
    registration.solve_access[0] = ENTASIS_BODY_ACCESS_MASS | ENTASIS_BODY_ACCESS_LINEAR_VELOCITY;
    registration.validate = joint_validate_cpp;
    registration.kernel = joint_kernel_cpp;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&world, &registration, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_enable_joint_breaks(&world, 1, nullptr) == ENTASIS_STATUS_OK);
    entasis_joint_reaction_provider_t provider{sizeof(provider), 1, joint_provider_cpp, &state};
    ENTASIS_TEST_CHECK(entasis_constraint_set_reaction_provider(&world, 56, &provider, nullptr) == ENTASIS_STATUS_OK);
    provider = {};
    entasis_body_inertia_t inertia{};
    inertia.inverse_mass = 1;
    inertia.inverse_inertia_tensor.xx = inertia.inverse_inertia_tensor.yy = inertia.inverse_inertia_tensor.zz = 1;
    const entasis_body_description_t body_description = entasis_body_shapeless(inertia, entasis_pose({}, {0, 0, 0, 1}), entasis_velocity({}, {}), entasis_body_activity(-1, 255));
    entasis_body_handle_t body{};
    ENTASIS_TEST_CHECK(entasis_body_add(&world, &body_description, &body, nullptr) == ENTASIS_STATUS_OK);
    const float target = 2;
    entasis_constraint_handle_t joint{};
    ENTASIS_TEST_CHECK(entasis_custom_constraint_add(&world, 56, &body, 1, &target, sizeof(target), &joint, nullptr) == ENTASIS_STATUS_OK);
    const entasis_joint_break_limits_t limits{ENTASIS_JOINT_BREAK_FORCE, 0, 0, 0, 913};
    ENTASIS_TEST_CHECK(entasis_constraint_set_break_limits(&world, joint, &limits, nullptr) == ENTASIS_STATUS_OK);
    entasis_joint_reaction_t reaction{};
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_constraint_reaction(&world, joint, &reaction, nullptr) == ENTASIS_STATUS_OK && reaction.state == ENTASIS_JOINT_REACTION_UNSOLVED);
    ENTASIS_TEST_CHECK(entasis_joint_break_reserve(&world, 2, nullptr) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&world, nullptr) == ENTASIS_STATUS_OK);
    entasis_fixed_stepper_t stepper = entasis_fixed_stepper(1.0f / 64, 4);
    entasis_joint_break_update_result_t result{};
    ENTASIS_TEST_CHECK(entasis_joint_break_stepper_update(&stepper, &world, 2.0f / 64, nullptr, 0, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, &result, nullptr) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(result.completed_steps == 1 && result.break_required == 1 && result.pending == ENTASIS_JOINT_BREAK_PENDING_BREAK);
    entasis_joint_break_event_t event{};
    ENTASIS_TEST_CHECK(entasis_joint_break_stepper_update(&stepper, &world, 0, &event, 1, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, &result, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(result.completed_steps == 1 && result.break_events_written == 1 && result.pending == 0 && stepper.accumulator == 0);
    ENTASIS_TEST_CHECK(event.user_id == 913 && event.constraint.value == joint.value && event.reaction.maximum_force == 128 && event.reserved == 0 && state.calls == 1);
    ENTASIS_TEST_CHECK(entasis_world_disable_joint_breaks(&world, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, nullptr) == ENTASIS_STATUS_OK);
    return 0;
}

int main(int argc, char **argv)
{
    const char *group = argc == 1 ? "all" : argv[1];
    if (argc > 2 || (std::strcmp(group, "all") != 0 && std::strcmp(group, "callbacks") != 0 &&
                     std::strcmp(group, "body-control") != 0 && std::strcmp(group, "restitution") != 0 && std::strcmp(group, "joint-breaks") != 0))
    {
        return 2;
    }
    if (std::strcmp(group, "all") == 0 || std::strcmp(group, "joint-breaks") == 0)
    {
        ENTASIS_TEST_CHECK(joint_breaks_cpp() == 0);
    }
    if (std::strcmp(group, "all") == 0 || std::strcmp(group, "body-control") == 0)
    {
        ENTASIS_TEST_CHECK(body_controls_cpp() == 0);
    }
    if (std::strcmp(group, "all") == 0 || std::strcmp(group, "restitution") == 0)
    {
        ENTASIS_TEST_CHECK(restitution_test() == 0);
    }
    if (std::strcmp(group, "all") == 0 || std::strcmp(group, "callbacks") == 0)
    {
        ENTASIS_TEST_CHECK(callback_composition() == 0);
    }
    return 0;
}
