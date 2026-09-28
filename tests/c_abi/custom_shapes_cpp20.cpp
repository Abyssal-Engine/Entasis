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
    unsigned disposed{};
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
    entasis_body_inertia_t body_inertia{};
    ENTASIS_TEST_CHECK(entasis_custom_shape_inertia(&world, shape, 2, &body_inertia, &diag) == ENTASIS_STATUS_OK && body_inertia.inverse_mass == 0.5f);
    const entasis_static_description_t description_static = entasis_static_body(shape, entasis_pose({0, 0, 0}, {0, 0, 0, 1}), entasis_ccd_discrete());
    entasis_static_handle_t handle{};
    ENTASIS_TEST_CHECK(entasis_static_add(&world, &description_static, ENTASIS_AWAKENING_NONE, &handle, &diag) == ENTASIS_STATUS_OK);
    entasis_query_context_t query{};
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &world, nullptr, nullptr, &diag) == ENTASIS_STATUS_OK);
    const entasis_ray_t input = entasis_ray({-5, 0, 0}, {1, 0, 0}, 10);
    entasis_ray_hit_t hit{};
    ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&query, input, nullptr, &hit, &diag) == ENTASIS_STATUS_OK && hit.t == 3);
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, &diag) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diag) == ENTASIS_STATUS_OK && state.disposed == 1);
    puts("CPP_CUSTOM_SHAPES_OK");
    return 0;
}
