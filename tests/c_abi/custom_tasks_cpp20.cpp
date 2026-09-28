#include "entasis.h"
#include "generated_layout_asserts.h"
#include "test_support.h"
#include <cstdint>
#include <cstring>
#include <type_traits>

static_assert(std::is_standard_layout_v<entasis_custom_shape_registration_t>);
static_assert(std::is_trivially_copyable_v<entasis_custom_shape_view_t>);
static_assert(sizeof(entasis_shape_access_t) == sizeof(void *));
struct alignas(32) Payload
{
    entasis_shape_handle_t child{};
    std::uint32_t tag{};
    std::uint8_t reserved[24]{};
};
struct State
{
    unsigned disposed{}, collision{}, sweep{};
};
static entasis_status_t ENTASIS_CALL bounds(void *, const void *raw, const entasis_quaternion_t *q,
                                            entasis_shape_access_t scope, entasis_shape_bounds_t *out)
{
    return entasis_shape_access_bounds(scope, static_cast<const Payload *>(raw)->child, q, out);
}
static entasis_status_t ENTASIS_CALL inertia(void *, const void *raw, float mass,
                                             entasis_shape_access_t scope, entasis_body_inertia_t *out)
{
    return entasis_shape_access_inertia(scope, static_cast<const Payload *>(raw)->child, mass, out);
}
static entasis_status_t ENTASIS_CALL ray(void *, const void *raw, const entasis_rigid_pose_t *pose,
                                         const entasis_ray_t *value, entasis_shape_access_t scope, entasis_shape_ray_hit_t *out)
{
    return entasis_shape_access_ray(scope, static_cast<const Payload *>(raw)->child, pose, value, out);
}
static entasis_status_t ENTASIS_CALL support(void *, const void *raw, const entasis_vector3_t *direction,
                                             entasis_shape_access_t scope, entasis_vector3_t *out)
{
    return entasis_shape_access_support(scope, static_cast<const Payload *>(raw)->child, direction, out);
}
static entasis_status_t ENTASIS_CALL dispose(void *raw, void *, entasis_shape_access_t)
{
    ++static_cast<State *>(raw)->disposed;
    return ENTASIS_STATUS_OK;
}
static_assert(alignof(entasis_convex_manifold_wide_t) == 32);
static_assert(std::is_standard_layout_v<entasis_collision_task_registration_t>);
static_assert(std::is_standard_layout_v<entasis_compound_task_registration_t>);
static_assert(std::is_trivially_copyable_v<entasis_compound_task_registration_t>);
static_assert(sizeof(entasis_compound_task_registration_t) == 28);
static_assert(std::is_trivially_copyable_v<entasis_sweep_task_input_t>);
static entasis_status_t ENTASIS_CALL collide(void *raw, const entasis_collision_task_input_t *input,
                                             entasis_task_access_t scope, entasis_convex_contact_manifold_t *out)
{
    ++static_cast<State *>(raw)->collision;
    entasis_collision_task_input_t pair = *input;
    const entasis_sphere_t a = entasis_sphere(2);
    pair.shape_a = &a;
    pair.type_a = ENTASIS_SHAPE_TYPE_SPHERE;
    return entasis_task_collide_convex(scope, &pair, out);
}
static entasis_status_t ENTASIS_CALL wide(void *, const entasis_collision_task_wide_input_t *,
                                          entasis_task_access_t, entasis_convex_manifold_wide_t *)
{
    return ENTASIS_STATUS_INVALID_DESCRIPTION; // this consumer exercises scalar queries only
}
static entasis_status_t ENTASIS_CALL sweep(void *raw, const entasis_sweep_task_input_t *input,
                                           entasis_task_access_t scope, entasis_sweep_task_result_t *out)
{
    ++static_cast<State *>(raw)->sweep;
    entasis_sweep_task_input_t pair = *input;
    const entasis_sphere_t a = entasis_sphere(2);
    pair.shape_a = &a;
    pair.type_a = ENTASIS_SHAPE_TYPE_SPHERE;
    return entasis_task_sweep_convex(scope, &pair, out);
}
int main()
{
    State state{};
    entasis_world_t world{};
    entasis_diagnostic_t diag{};
    entasis_world_description_t description = entasis_world_description_default();
    description.threading.worker_count = 1;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diag) == ENTASIS_STATUS_OK);
    entasis_custom_shape_registration_t registration = entasis_custom_shape_registration_default();
    registration.size = sizeof(Payload);
    registration.alignment = alignof(Payload);
    registration.user_context = &state;
    registration.bounds = bounds;
    registration.inertia = inertia;
    registration.ray = ray;
    registration.support = support;
    registration.dispose = dispose;
    entasis_shape_type_id_t type{};
    ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &registration, &type, &diag) == ENTASIS_STATUS_OK);
    registration = {}; // copied table must remain usable
    Payload payload{};
    payload.tag = 123;
    const entasis_sphere_t sphere = entasis_sphere(2);
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &payload.child, &diag) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t shape{};
    ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, type, &payload, sizeof(payload), &shape, &diag) == ENTASIS_STATUS_OK);
    Payload result{};
    std::uint64_t required{};
    ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, shape, &result, sizeof(result), &required, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(required == sizeof(Payload) && result.tag == payload.tag && result.child.packed == payload.child.packed);
    entasis_collision_task_registration_t collision = entasis_collision_task_registration_default();
    collision.shape_type_a = type;
    collision.shape_type_b = ENTASIS_SHAPE_TYPE_SPHERE;
    collision.test = collide;
    collision.wide_test = wide;
    collision.user_context = &state;
    entasis_sweep_task_registration_t task = entasis_sweep_task_registration_default();
    task.shape_type_a = type;
    task.shape_type_b = ENTASIS_SHAPE_TYPE_SPHERE;
    task.test = sweep;
    task.child_test = sweep;
    task.user_context = &state;
    std::int32_t collision_id = -1, sweep_id = -1;
    ENTASIS_TEST_CHECK(entasis_collision_task_register(&world, &collision, &collision_id, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_sweep_task_register(&world, &task, &sweep_id, &diag) == ENTASIS_STATUS_OK);
    collision = {};
    task = {};
    entasis_compound_task_registration_t compound = entasis_compound_task_registration_default();
    compound.shape_type_a = type;
    compound.shape_type_b = ENTASIS_SHAPE_TYPE_COMPOUND;
    std::int32_t compound_id = -1;
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&world, &compound, &compound_id, &diag) == ENTASIS_STATUS_OK && compound_id >= 0);
    ENTASIS_TEST_CHECK(entasis_collision_task_compound_register(&world, &compound, &compound_id, &diag) == ENTASIS_STATUS_INVALID_DESCRIPTION && compound_id == -1);
    entasis_contact_manifold_t manifold{};
    ENTASIS_TEST_CHECK(entasis_collision_query(&world, payload.child, entasis_pose({0, 1, 0}, {0, 0, 0, 1}), shape, entasis_pose({0, 0, 0}, {0, 0, 0, 1}), 0, &manifold, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(manifold.convex.count == 1 && manifold.convex.contacts[0].depth == 3 && state.collision == 1);
    const entasis_static_description_t sd = entasis_static_body(shape, entasis_pose({0, 0, 0}, {0, 0, 0, 1}), entasis_ccd_discrete());
    entasis_static_handle_t handle{};
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &sd, ENTASIS_AWAKENING_NONE, &handle, &diag) == ENTASIS_STATUS_OK);
    entasis_query_context_t query{};
    entasis_bool_t hit{};
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &world, nullptr, nullptr, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_query_context_sweep_any(&query, payload.child, entasis_pose({-5, 0, 0}, {0, 0, 0, 1}), entasis_velocity({1, 0, 0}, {0, 0, 0}), 10, nullptr, nullptr, &hit, &diag) == ENTASIS_STATUS_OK && hit);
    ENTASIS_TEST_CHECK(state.sweep == 1);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diag) == ENTASIS_STATUS_OK && state.disposed == 1);
    puts("CPP_CUSTOM_TASKS_OK");
    return 0;
}
