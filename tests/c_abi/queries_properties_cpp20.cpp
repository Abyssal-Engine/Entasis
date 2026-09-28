#include "entasis.h"
#include "generated_layout_asserts.h"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <type_traits>

struct property_payload
{
    std::uint64_t entity;
    std::uint32_t flags;
    std::uint32_t pad;
};

int main(int argc, char **argv)
{
    if (argc > 2 || (argc == 2 && std::strcmp(argv[1], "queries") != 0))
    {
        return 2;
    }
    static_assert(std::is_standard_layout_v<entasis_distance_query_t>);
    static_assert(std::is_standard_layout_v<entasis_distance_query_result_t>);
    static_assert(sizeof(entasis_distance_query_t) == 100u);
    static_assert(sizeof(entasis_distance_query_result_t) == 84u);

    static_assert(std::is_standard_layout_v<entasis_query_t>);
    static_assert(std::is_standard_layout_v<entasis_contact_event_t>);
    static_assert(sizeof(entasis_query_t) == 128u);
    static_assert(sizeof(entasis_contact_event_t) == 392u);

    const entasis_ray_t ray = entasis_ray(
        entasis_vector3_t{0.0f, 1.0f, 2.0f},
        entasis_vector3_t{1.0f, 0.0f, 0.0f},
        50.0f);
    const entasis_query_filter_t filter = entasis_query_filter_all();
    const entasis_query_t query = entasis_query_ray_any(ray, &filter);
    assert(query.kind == ENTASIS_QUERY_RAY_ANY);

    entasis_diagnostic_t diagnostic{};
    if (argc == 1)
    {
        entasis_body_property_table_t table{};
        assert(entasis_body_property_init(
                   &table,
                   sizeof(property_payload),
                   alignof(property_payload),
                   8u,
                   nullptr,
                   &diagnostic) == ENTASIS_STATUS_OK);

        const entasis_body_handle_t handle{7};
        const property_payload input{UINT64_C(0x1122334455667788), UINT32_C(0xa5a5), 0u};
        assert(entasis_body_property_set(&table, handle, &input, &diagnostic) == ENTASIS_STATUS_OK);

        const void *borrowed = nullptr;
        assert(entasis_body_property_get(&table, handle, &borrowed, &diagnostic) == ENTASIS_STATUS_OK);
        assert(borrowed != nullptr);
        assert(std::memcmp(borrowed, &input, sizeof(input)) == 0);

        entasis_body_property_key_t key{};
        assert(entasis_body_property_key(&table, handle, &key, &diagnostic) == ENTASIS_STATUS_OK);
        borrowed = nullptr;
        assert(entasis_body_property_get_key(&table, key, &borrowed, &diagnostic) == ENTASIS_STATUS_OK);
        assert(std::memcmp(borrowed, &input, sizeof(input)) == 0);

        assert(entasis_body_property_remove(&table, handle, &diagnostic) == ENTASIS_STATUS_OK);
        borrowed = nullptr;
        assert(entasis_body_property_get_key(&table, key, &borrowed, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
        assert(entasis_body_property_destroy(&table, &diagnostic) == ENTASIS_STATUS_OK);
    }
    entasis_world_t world{};
    entasis_query_context_t query_context{};
    entasis_world_description_t description = entasis_world_description_default();
    description.threading.worker_count = 1;
    assert(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t sphere{};
    const entasis_sphere_t geometry = entasis_sphere(1);
    assert(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &geometry, &sphere, &diagnostic) == ENTASIS_STATUS_OK);
    assert(entasis_query_context_init(&query_context, &world, nullptr, nullptr, &diagnostic) == ENTASIS_STATUS_OK);
    assert(entasis_query_context_distance_query_reserve(&query_context, nullptr, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_distance_query_t distance{};
    distance.kind = ENTASIS_DISTANCE_QUERY_DISTANCE;
    distance.shape_a = sphere;
    distance.shape_b = sphere;
    distance.pose_a = entasis_pose(entasis_vector3_t{0, 0, 0}, entasis_quaternion_t{0, 0, 0, 1});
    distance.pose_b = entasis_pose(entasis_vector3_t{4, 0, 0}, entasis_quaternion_t{0, 0, 0, 1});
    distance.settings = entasis_distance_query_settings_default();
    entasis_distance_query_result_t result{};
    assert(entasis_world_begin_read(&world, &diagnostic) == ENTASIS_STATUS_OK);
    assert(entasis_query_context_distance_query_batch(&query_context, &distance, 1, &result, 1, &diagnostic) == ENTASIS_STATUS_OK);
    assert(result.shape.geometry.state == ENTASIS_DISTANCE_SEPARATED && result.shape.geometry.distance == 2);
    assert(entasis_world_end_read(&world, &diagnostic) == ENTASIS_STATUS_OK);
    assert(entasis_query_context_destroy(&query_context, &diagnostic) == ENTASIS_STATUS_OK);
    assert(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}
