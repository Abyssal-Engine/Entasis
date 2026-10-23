#include <entasis/entasis.h>
#include <entasis/cooking.h>

#include <string.h>
#include <stdio.h>
#include <math.h>

int main()
{
    if (entasis_abi_version() != ENTASIS_ABI_VERSION)
        return 1;
    if (entasis_cooking_abi_version() != ENTASIS_COOKING_ABI_VERSION)
        return 2;

    const entasis_version_t version = entasis_version_current();
    if (version.major != ENTASIS_VERSION_MAJOR || version.minor != ENTASIS_VERSION_MINOR ||
        version.patch != ENTASIS_VERSION_PATCH ||
        version.prerelease.count != strlen(ENTASIS_VERSION_PRERELEASE) ||
        (version.prerelease.count != 0u &&
         memcmp(version.prerelease.data, ENTASIS_VERSION_PRERELEASE,
                (size_t)version.prerelease.count) != 0))
    {
        fputs("Entasis header/library product version mismatch\n", stderr);
        return 5;
    }

    entasis_world_t world{};
    entasis_diagnostic_t diagnostic{};
    const entasis_world_description_t description = entasis_world_description_default();
    if (entasis_world_init(&world, &description, &diagnostic) != ENTASIS_STATUS_OK)
        return 3;
    const entasis_global_position_t origin = {1000000000.0, 0.0, 0.0};
    entasis_origin_frame_t frame;
    if (entasis_origin_frame_at(&origin, &frame, &diagnostic) != ENTASIS_STATUS_OK)
        return 6;
    const entasis_vector3_t shift = {256.0f, 0.0f, 0.0f};
    if (entasis_world_shift_origin(&world, &frame, &shift, &diagnostic) != ENTASIS_STATUS_OK ||
        frame.origin.x != origin.x + 256.0)
        return 7;
#if ENTASIS_PRECISION_MIXED64
    const entasis_vector3_t zero = {0.0f, 0.0f, 0.0f};
    const entasis_quaternion_t identity = {0.0f, 0.0f, 0.0f, 1.0f};
    const entasis_sphere_t sphere = entasis_sphere(0.5f);
    entasis_shape_handle_t shape;
    if (entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, &diagnostic) != ENTASIS_STATUS_OK)
        return 8;
    const entasis_body_description_t body = entasis_body_kinematic(shape, entasis_pose(zero, identity),
                                                                   entasis_velocity(zero, zero), entasis_body_activity(-1.0f, 255));
    const entasis_global_position_t position = {1000000000.002, 3.0, 0.0};
    entasis_body_handle_t handle;
    if (entasis_body_add_global(&world, &body, &position, &handle, &diagnostic) != ENTASIS_STATUS_OK)
        return 9;
    entasis_query_context_t query;
    memset(&query, 0, sizeof(query));
    if (entasis_query_context_init(&query, &world, NULL, NULL, &diagnostic) != ENTASIS_STATUS_OK)
        return 10;
    const entasis_world_precision_segment_t segment = {
        {position.x - 2.0, 3.0, 0.0}, {position.x + 2.0, 3.0, 0.0}};
    int32_t stack[128];
    entasis_global_ray_hit_t hit;
    if (entasis_query_context_ray_cast_global_closest(&query, segment, NULL, &hit, stack, 128, &diagnostic) != ENTASIS_STATUS_OK ||
        fabs(hit.location.x - (position.x - 0.5)) > 0.000001)
        return 11;
    entasis_global_pose_t exported;
    uint64_t completed = 0;
    if (entasis_world_export_global_poses(&world, &hit.collidable, 1, &exported, 1, NULL, &completed, &diagnostic) != ENTASIS_STATUS_OK ||
        completed != 1 || exported.position.x != position.x)
        return 12;
    if (entasis_query_context_destroy(&query, &diagnostic) != ENTASIS_STATUS_OK)
        return 13;
    printf("SDK_PRECISION_OK precision=mixed64 position=%.9f\n", exported.position.x);
#else
    puts("SDK_PRECISION_OK precision=float32 origin-shift=256");
#endif
    return entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK ? 0 : 4;
}
