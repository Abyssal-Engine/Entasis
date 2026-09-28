#include "entasis.h"
#include "generated_layout_asserts.h"
#include "test_support.h"
#include <cmath>
#include <cstdint>
#include <type_traits>

static_assert(std::is_standard_layout_v<entasis_custom_constraint_registration_t>);
static_assert(std::is_trivially_copyable_v<entasis_constraint_kernel_view_t>);
static_assert(std::is_same_v<decltype(entasis_constraint_body_view_t::position), const entasis_vector3_wide_t *>);
static_assert(sizeof(entasis_constraint_body_view_t) == 48 && sizeof(entasis_constraint_kernel_view_t) == 56);
struct State
{
    unsigned calls{}, errors{};
};
static entasis_status_t ENTASIS_CALL validate(void *, entasis_constraint_type_id_t id, const void *p, std::uint32_t size)
{
    return static_cast<entasis_status_t>(id == 56 && size == sizeof(float) && std::isfinite(*static_cast<const float *>(p)) ? ENTASIS_STATUS_OK : ENTASIS_STATUS_INVALID_DESCRIPTION);
}
static void ENTASIS_CALL kernel(void *raw, const entasis_constraint_kernel_view_t *v)
{
    State &s = *static_cast<State *>(raw);
    ++s.calls;
    if (v->body_count != 2 || v->prestep_field_count != 1 || v->impulse_field_count != 1)
    {
        ++s.errors;
        return;
    }
    for (unsigned i = 0; i < 2; ++i)
    {
        const entasis_constraint_body_view_t &b = v->bodies[i];
        if (b.position || b.orientation || b.inverse_mass || b.inverse_inertia || b.angular_velocity || !b.linear_velocity)
        {
            ++s.errors;
            continue;
        }
        if (reinterpret_cast<std::uintptr_t>(b.linear_velocity) % 32)
            ++s.errors;
        if (v->phase == ENTASIS_CONSTRAINT_KERNEL_SOLVE)
            for (unsigned lane = 0; lane < 8; ++lane)
                if (v->active_mask->lanes[lane])
                    b.linear_velocity->x.lanes[lane] = v->prestep[0].lanes[lane] + static_cast<float>(i);
    }
}
int main()
{
    entasis_world_t world{};
    State s{};
    entasis_world_description_t d = entasis_world_description_default();
    d.threading.worker_count = 1;
    d.gravity = {0, 0, 0};
    d.solve.substeps = 2;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &d, nullptr) == ENTASIS_STATUS_OK);
    entasis_custom_constraint_registration_t r = entasis_custom_constraint_registration_default();
    r.type_id = 56;
    r.body_count = 2;
    r.description_size = 4;
    r.prestep_bundle_size = 32;
    r.impulse_bundle_size = 32;
    r.user_context = &s;
    r.validate = validate;
    r.kernel = kernel;
    for (unsigned i = 0; i < 2; ++i)
        r.initial_access[i] = r.solve_access[i] = ENTASIS_BODY_ACCESS_LINEAR_VELOCITY;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&world, &r, nullptr) == ENTASIS_STATUS_OK);
    r = {};
    entasis_body_inertia_t inertia{};
    inertia.inverse_mass = 1;
    inertia.inverse_inertia_tensor.xx = inertia.inverse_inertia_tensor.yy = inertia.inverse_inertia_tensor.zz = 1;
    const entasis_body_description_t bd = entasis_body_shapeless(inertia, entasis_pose({0, 0, 0}, {0, 0, 0, 1}), entasis_velocity({0, 0, 0}, {0, 0, 0}), entasis_body_activity(-1, 255));
    entasis_body_handle_t bodies[2]{};
    for (entasis_body_handle_t &b : bodies)
        ENTASIS_TEST_CHECK(entasis_body_add(&world, &bd, &b, nullptr) == ENTASIS_STATUS_OK);
    float target = 2;
    entasis_constraint_handle_t handle{};
    ENTASIS_TEST_CHECK(entasis_custom_constraint_add(&world, 56, bodies, 2, &target, sizeof(target), &handle, nullptr) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, nullptr) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 2; ++i)
    {
        entasis_body_state_t b{};
        ENTASIS_TEST_CHECK(entasis_body_get(&world, bodies[i], &b, nullptr) == ENTASIS_STATUS_OK && b.velocity.linear.x == 2 + static_cast<float>(i));
    }
    target = 4;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_apply(&world, handle, 56, &target, 4, nullptr) == ENTASIS_STATUS_OK);
    target = 0;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_get(&world, handle, 56, &target, 4, nullptr) == ENTASIS_STATUS_OK && target == 4);
    ENTASIS_TEST_CHECK(s.calls > 0 && s.errors == 0);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, nullptr) == ENTASIS_STATUS_OK);
    puts("CPP_CUSTOM_CONSTRAINTS_OK");
    return 0;
}
