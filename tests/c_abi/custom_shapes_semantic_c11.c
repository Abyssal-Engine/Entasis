#include "custom_shape_fixture.h"
#include <inttypes.h>

static uint32_t bits(float value)
{
    uint32_t output = 0;
    memcpy(&output, &value, sizeof(output));
    return output;
}
int main(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    for (unsigned w = 0; w < 2; ++w)
    {
        entasis_world_t world = {0};
        entasis_diagnostic_t diag = {0};
        entasis_world_description_t d = description();
        ENTASIS_TEST_CHECK(entasis_world_init(&world, &d, &diag) == ENTASIS_STATUS_OK);
        shape_state_t state = {0};
        state.world = &world;
        state.alignment = 32;
        state.delegate_child = 1;
        entasis_custom_shape_registration_t r = registration(&state);
        entasis_shape_type_id_t type = -1;
        ENTASIS_TEST_CHECK(entasis_custom_shape_register(&world, &r, &type, &diag) == ENTASIS_STATUS_OK);
        r = (entasis_custom_shape_registration_t){0};
        payload_t input = {0};
        input.radius = (float)(w + 1);
        input.tag = 123 + w;
        const entasis_sphere_t sphere = entasis_sphere(input.radius);
        ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &input.child, &diag) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_custom_shape_add(&world, type, &input, sizeof(input), &state.self, &diag) == ENTASIS_STATUS_OK);
        payload_t output = {0};
        uint64_t required = 0;
        ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, state.self, &output, sizeof(output), &required, &diag) == ENTASIS_STATUS_OK);
        state.check_access = 1;
        state.check_support = 1;
        entasis_body_inertia_t inertia = {0};
        ENTASIS_TEST_CHECK(entasis_custom_shape_inertia(&world, state.self, 2, &inertia, &diag) == ENTASIS_STATUS_OK);
        const entasis_static_description_t stat = entasis_static_body(state.self, pose_at(0, 0, 0), entasis_ccd_discrete());
        entasis_static_handle_t handle = {0};
        ENTASIS_TEST_CHECK(entasis_static_add(&world, &stat, ENTASIS_AWAKENING_NONE, &handle, &diag) == ENTASIS_STATUS_OK);
        entasis_query_context_t query = {0};
        ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &world, NULL, NULL, &diag) == ENTASIS_STATUS_OK);
        const entasis_ray_t ray = entasis_ray((entasis_vector3_t){-5, 0, 0}, (entasis_vector3_t){1, 0, 0}, 10);
        entasis_ray_hit_t hit = {0};
        ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, &diag) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_query_context_ray_cast_closest(&query, ray, NULL, &hit, &diag) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_custom_shape_get(&world, state.self, &output, sizeof(output), &required, &diag) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_end_read(&world, &diag) == ENTASIS_STATUS_OK);
        printf("world=%u type=%" PRId32 " tag=%" PRIu32 " bytes=%" PRIu64 " mass=%" PRIu32 " tensor=%" PRIu32 " ray=%" PRIu32 " normal=%" PRIu32 "\n",
               w, type, output.tag, required, bits(inertia.inverse_mass), bits(inertia.inverse_inertia_tensor.xx), bits(hit.t), bits(hit.normal.x));
        ENTASIS_TEST_CHECK(state.errors == 0 && state.support >= 2 && state.sweep_support > 0);
        ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, &diag) == ENTASIS_STATUS_OK);
        state.check_access = 0;
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diag) == ENTASIS_STATUS_OK);
        printf("disposed=%u\n", state.disposals);
    }
    return 0;
}
