#include "entasis.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace
{

entasis_quaternion_t identity_orientation() noexcept
{
    return entasis_quaternion_t{0.0F, 0.0F, 0.0F, 1.0F};
}

entasis_body_inertia_t unit_inertia() noexcept
{
    entasis_body_inertia_t inertia{};
    inertia.inverse_inertia_tensor.xx = 1.0F;
    inertia.inverse_inertia_tensor.yy = 1.0F;
    inertia.inverse_inertia_tensor.zz = 1.0F;
    inertia.inverse_mass = 1.0F;
    return inertia;
}

entasis_body_description_t shapeless_body(float x) noexcept
{
    const entasis_rigid_pose_t pose = entasis_pose(
        entasis_vector3_t{x, 0.0F, 0.0F}, identity_orientation());
    const entasis_body_velocity_t velocity = entasis_velocity(
        entasis_vector3_t{0.0F, 0.0F, 0.0F},
        entasis_vector3_t{0.0F, 0.0F, 0.0F});
    return entasis_body_shapeless(
        unit_inertia(), pose, velocity, entasis_body_activity(0.01F, UINT8_C(255)));
}

} // namespace

int main()
{
    entasis_world_description_t description = entasis_world_description_default();
    description.gravity = entasis_vector3_t{0.0F, 0.0F, 0.0F};
    description.threading.worker_count = UINT32_C(1);
    description.capacity.bodies = INT32_C(8);
    description.capacity.constraints = INT32_C(8);
    description.capacity.initial_constraints_per_type_batch = INT32_C(4);
    description.capacity.minimum_constraints_per_body = INT32_C(4);
    description.capacity.broad_phase_candidates = INT32_C(32);
    description.capacity.pairs = INT32_C(32);
    description.capacity.collision_child_pairs = INT32_C(32);

    entasis_world_t world{};
    entasis_diagnostic_t diagnostic{};
    if (entasis_world_init(&world, &description, &diagnostic) != ENTASIS_STATUS_OK)
        return 1;

    std::array<entasis_body_handle_t, 2> bodies{
        entasis_body_handle_invalid(), entasis_body_handle_invalid()};
    for (std::size_t index = 0; index < bodies.size(); ++index)
    {
        const entasis_body_description_t body = shapeless_body(static_cast<float>(index) * 2.0F);
        if (entasis_body_add(&world, &body, &bodies[index], &diagnostic) != ENTASIS_STATUS_OK)
            return 2;
    }

    const entasis_ball_socket_t ball_socket{
        entasis_vector3_t{0.0F, 0.0F, 0.0F},
        entasis_vector3_t{0.0F, 0.0F, 0.0F},
        entasis_spring_settings(30.0F, 1.0F)};
    entasis_constraint_handle_t constraint = entasis_constraint_handle_invalid();
    if (entasis_constraint_add(
            &world,
            ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET,
            bodies.data(),
            static_cast<uint32_t>(bodies.size()),
            &ball_socket,
            static_cast<uint32_t>(sizeof(ball_socket)),
            &constraint,
            &diagnostic) != ENTASIS_STATUS_OK)
        return 3;

    entasis_constraint_info_t info{};
    if (entasis_constraint_inspect(&world, constraint, &info, &diagnostic) != ENTASIS_STATUS_OK)
        return 4;
    if (info.type_id != ENTASIS_CONSTRAINT_TYPE_BALL_SOCKET || info.body_count != UINT8_C(2))
        return 5;

    if (entasis_body_apply_linear_impulse(
            &world, bodies[0], entasis_vector3_t{1.0F, 0.0F, 0.0F}, &diagnostic) != ENTASIS_STATUS_OK)
        return 6;

    entasis_fixed_stepper_t stepper = entasis_fixed_stepper(1.0F / 60.0F, UINT8_C(4));
    uint32_t steps = 0;
    float alpha = 0.0F;
    if (entasis_fixed_stepper_update(
            &stepper, &world, 1.0F / 30.0F, &steps, &alpha, &diagnostic) != ENTASIS_STATUS_OK)
        return 7;
    if (steps != UINT32_C(2) || alpha < 0.0F || alpha >= 1.0F)
        return 8;

    if (entasis_constraint_remove(&world, constraint, &diagnostic) != ENTASIS_STATUS_OK)
        return 9;
    for (const entasis_body_handle_t body : bodies)
    {
        if (entasis_body_remove(&world, body, &diagnostic) != ENTASIS_STATUS_OK)
            return 10;
    }
    if (entasis_world_destroy(&world, &diagnostic) != ENTASIS_STATUS_OK)
        return 11;
    return 0;
}
