#include "entasis.h"
#include "test_support.h"

#include <stdint.h>

static int run_policy_world(const entasis_pose_policy_t *pose)
{
    entasis_world_description_t description = entasis_world_description_default();
    entasis_default_narrow_policy_t narrow = entasis_default_narrow_policy();
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_description_set_callbacks(&description, &narrow, pose) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    for (int step = 0; step < 4; ++step)
    {
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    }
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.step_index == UINT64_C(4));
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    const entasis_spring_settings_t spring = entasis_spring_settings(30.0f, 1.0f);
    ENTASIS_TEST_CHECK(spring.angular_frequency > 0.0f);
    ENTASIS_TEST_CHECK(spring.twice_damping_ratio == 2.0f);

    const entasis_contact_material_t material = entasis_contact_material(0.8f, 2.0f, spring);
    ENTASIS_TEST_CHECK(material.friction_coefficient == 0.8f);
    ENTASIS_TEST_CHECK(material.maximum_recovery_velocity == 2.0f);
    const entasis_default_narrow_policy_t narrow = entasis_narrow_policy_default(material);
    ENTASIS_TEST_CHECK(narrow.material.friction_coefficient == 0.8f);

    const entasis_uniform_gravity_policy_t uniform = entasis_uniform_gravity_policy(
        (entasis_vector3_t){0.0f, -9.81f, 0.0f}, 0.01f, 0.02f);
    const entasis_pose_policy_t uniform_pose = entasis_pose_policy_uniform_mode(
        &uniform, ENTASIS_ANGULAR_INTEGRATION_CONSERVE_MOMENTUM);
    ENTASIS_TEST_CHECK(uniform_pose.kind == ENTASIS_POSE_POLICY_UNIFORM_GRAVITY);
    ENTASIS_TEST_CHECK(uniform_pose.angular_mode == ENTASIS_ANGULAR_INTEGRATION_CONSERVE_MOMENTUM);
    ENTASIS_TEST_CHECK(run_policy_world(&uniform_pose) == 0);

    const entasis_planetary_gravity_policy_t planetary = entasis_planetary_gravity_policy(
        (entasis_vector3_t){1.0f, 2.0f, 3.0f}, 120.0f, 0.03f, 0.04f);
    const entasis_pose_policy_t planetary_pose = entasis_pose_policy_planetary(&planetary);
    ENTASIS_TEST_CHECK(planetary_pose.kind == ENTASIS_POSE_POLICY_PLANETARY_GRAVITY);
    ENTASIS_TEST_CHECK(run_policy_world(&planetary_pose) == 0);

    const entasis_vector3_t gravity_by_handle[4] = {
        {0.0f, -1.0f, 0.0f},
        {0.0f, -2.0f, 0.0f},
        {0.0f, -3.0f, 0.0f},
        {0.0f, -4.0f, 0.0f}};
    const entasis_per_body_gravity_policy_t per_body = entasis_per_body_gravity_policy(
        gravity_by_handle, UINT64_C(4), 0.05f, 0.06f);
    const entasis_pose_policy_t per_body_pose = entasis_pose_policy_per_body(&per_body);
    ENTASIS_TEST_CHECK(per_body_pose.kind == ENTASIS_POSE_POLICY_PER_BODY_GRAVITY);
    ENTASIS_TEST_CHECK(run_policy_world(&per_body_pose) == 0);

    entasis_world_description_t invalid_description = entasis_world_description_default();
    entasis_pose_policy_t invalid_pose = uniform_pose;
    invalid_pose.struct_version = UINT32_C(2);
    ENTASIS_TEST_CHECK(entasis_world_description_set_callbacks(&invalid_description, NULL, &invalid_pose) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    invalid_pose = uniform_pose;
    invalid_pose.angular_mode = UINT8_C(255);
    ENTASIS_TEST_CHECK(entasis_world_description_set_callbacks(&invalid_description, NULL, &invalid_pose) == ENTASIS_STATUS_OK);
    entasis_world_t invalid_world = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&invalid_world, &invalid_description, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);

    invalid_description = entasis_world_description_default();
    entasis_pose_policy_t invalid_per_body = entasis_pose_policy_per_body(NULL);
    ENTASIS_TEST_CHECK(entasis_world_description_set_callbacks(&invalid_description, NULL, &invalid_per_body) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_init(&invalid_world, &invalid_description, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    return 0;
}
