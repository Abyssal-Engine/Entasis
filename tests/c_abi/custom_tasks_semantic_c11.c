#include "compound_task_fixture.h"
#include <inttypes.h>
static uint32_t bits(float value)
{
    uint32_t output = 0;
    memcpy(&output, &value, sizeof(output));
    return output;
}
static void print_compound(unsigned target, unsigned mode, unsigned batch, unsigned flipped,
                           const entasis_contact_manifold_t *m)
{
    printf("compound target=%u context=%u batch=%u flip=%u kind=%u count=%d offset=%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n",
           target, mode, batch, flipped, (unsigned)m->kind, m->nonconvex.count, bits(m->nonconvex.offset_b.x), bits(m->nonconvex.offset_b.y), bits(m->nonconvex.offset_b.z));
    for (int i = 0; i < m->nonconvex.count; ++i)
    {
        const entasis_contact_point_t *c = &m->nonconvex.contacts[i];
        printf("contact index=%d offset=%" PRIu32 ",%" PRIu32 ",%" PRIu32 " normal=%" PRIu32 ",%" PRIu32 ",%" PRIu32 " depth=%" PRIu32 " feature=%d\n",
               i, bits(c->offset.x), bits(c->offset.y), bits(c->offset.z), bits(c->normal.x), bits(c->normal.y), bits(c->normal.z), bits(c->depth), c->feature_id);
    }
}
int main(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    task_fixture_t f = {0};
    ENTASIS_TEST_CHECK(task_fixture_init(&f, NULL) == 0 && task_fixture_bind(&f) == 0);
    for (unsigned flipped = 0; flipped < 2; ++flipped)
    {
        entasis_contact_manifold_t m = {0};
        const entasis_status_t status = flipped ? entasis_collision_query(&f.world, f.sphere, pose_at(0, 1, 0), f.custom, pose_at(0, 0, 0), 0, &m, NULL) : entasis_collision_query(&f.world, f.custom, pose_at(0, 0, 0), f.sphere, pose_at(0, 1, 0), 0, &m, NULL);
        ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
        printf("collision flip=%u count=%d depth=%" PRIu32 " normal=%" PRIu32 " feature=%d\n", flipped, m.convex.count, bits(m.convex.contacts[0].depth), bits(m.convex.normal.y), m.convex.contacts[0].feature_id);
    }
    entasis_static_handle_t handle = {0};
    entasis_static_description_t sd = entasis_static_body(f.custom, pose_at(0, 0, 0), entasis_ccd_discrete());
    ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &handle, NULL) == ENTASIS_STATUS_OK);
    entasis_query_context_t query = {0};
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &f.world, NULL, NULL, NULL) == ENTASIS_STATUS_OK);
    entasis_sweep_hit_t hit = {0};
    const entasis_body_velocity_t velocity = entasis_velocity((entasis_vector3_t){1, 0, 0}, (entasis_vector3_t){0, 0, 0});
    for (unsigned context = 0; context < 2; ++context)
    {
        const entasis_status_t status = context ? entasis_query_context_sweep_closest(&query, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &hit, NULL) : entasis_sweep_closest(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &hit, NULL);
        ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
        printf("sweep context=%u t0=%" PRIu32 " t1=%" PRIu32 " normal=%" PRIu32 " child_a=%d child_b=%d\n", context, bits(hit.sweep.t0), bits(hit.sweep.t1), bits(hit.sweep.normal.x), hit.sweep.child_a, hit.sweep.child_b);
        entasis_bool_t found = 1;
        f.task.sweep_status = ENTASIS_STATUS_CAPACITY_MISSING;
        const entasis_status_t failure = context ? entasis_query_context_sweep_any(&query, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &found, NULL) : entasis_sweep_any(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &found, NULL);
        ENTASIS_TEST_CHECK(failure == ENTASIS_STATUS_CAPACITY_MISSING && found == 0);
        printf("failure context=%u status=%u found=%u\n", context, (unsigned)failure, (unsigned)found);
        f.task.sweep_status = 0;
    }
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_remove(&f.world, handle, ENTASIS_AWAKENING_NONE, NULL) == ENTASIS_STATUS_OK);
    const entasis_compound_child_t children[2] = {entasis_compound_child(f.custom, pose_at(-0.4f, 0, 0)), entasis_compound_child(f.custom, pose_at(0.4f, 0, 0))};
    entasis_shape_handle_t compound = {0};
    ENTASIS_TEST_CHECK(entasis_shape_import_compound(&f.world, children, 2, &compound, NULL) == ENTASIS_STATUS_OK);
    sd = entasis_static_body(compound, pose_at(0, 0, 0), entasis_ccd_discrete());
    ENTASIS_TEST_CHECK(entasis_static_add(&f.world, &sd, ENTASIS_AWAKENING_NONE, &handle, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_sweep_closest(&f.world, f.sphere, pose_at(-5, 0, 0), velocity, 10, NULL, NULL, &hit, NULL) == ENTASIS_STATUS_OK);
    printf("child t0=%" PRIu32 " t1=%" PRIu32 " normal=%" PRIu32 " child_b=%d\n", bits(hit.sweep.t0), bits(hit.sweep.t1), bits(hit.sweep.normal.x), hit.sweep.child_b);
    ENTASIS_TEST_CHECK(entasis_static_remove(&f.world, handle, ENTASIS_AWAKENING_NONE, NULL) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t targets[3] = {0};
    ENTASIS_TEST_CHECK(compound_targets(&f, targets) == 0);
    ENTASIS_TEST_CHECK(entasis_query_context_init(&query, &f.world, NULL, NULL, NULL) == ENTASIS_STATUS_OK);
    for (unsigned target = 0; target < 3; ++target)
    {
        const entasis_rigid_pose_t a = pose_at(0, target == 2 ? 0.5f : 0, 0), b = pose_at(0, 0, 0);
        entasis_collision_query_t pairs[2] = {0};
        pairs[0].shape_a = f.custom;
        pairs[0].pose_a = a;
        pairs[0].shape_b = targets[target];
        pairs[0].pose_b = b;
        pairs[1].shape_a = targets[target];
        pairs[1].pose_a = b;
        pairs[1].shape_b = f.custom;
        pairs[1].pose_b = a;
        for (unsigned mode = 0; mode < 2; ++mode)
        {
            for (unsigned flipped = 0; flipped < 2; ++flipped)
            {
                const entasis_collision_query_t *pair = &pairs[flipped];
                entasis_contact_manifold_t m = {0};
                const entasis_status_t status = mode ? entasis_query_context_collision_query(&query, pair->shape_a, pair->pose_a, pair->shape_b, pair->pose_b, 0, &m, NULL) : entasis_collision_query(&f.world, pair->shape_a, pair->pose_a, pair->shape_b, pair->pose_b, 0, &m, NULL);
                ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
                print_compound(target, mode, 0, flipped, &m);
            }
            entasis_collision_query_result_t results[2] = {0};
            const entasis_status_t status = mode ? entasis_query_context_collision_query_batch(&query, pairs, 2, results, 2, NULL) : entasis_collision_query_batch(&f.world, pairs, 2, results, 2, NULL);
            ENTASIS_TEST_CHECK(status == ENTASIS_STATUS_OK);
            for (unsigned flipped = 0; flipped < 2; ++flipped)
            {
                ENTASIS_TEST_CHECK(results[flipped].status == ENTASIS_STATUS_OK && results[flipped].hit);
                print_compound(target, mode, 1, flipped, &results[flipped].manifold);
            }
            entasis_contact_manifold_t m = {0};
            f.task.wide_status = ENTASIS_STATUS_CAPACITY_MISSING;
            const entasis_status_t failure = mode ? entasis_query_context_collision_query(&query, f.custom, a, targets[target], b, 0, &m, NULL) : entasis_collision_query(&f.world, f.custom, a, targets[target], b, 0, &m, NULL);
            ENTASIS_TEST_CHECK(failure == ENTASIS_STATUS_CAPACITY_MISSING);
            printf("compound-failure target=%u context=%u status=%u\n", target, mode, (unsigned)failure);
            f.task.wide_status = ENTASIS_STATUS_OK;
            const entasis_status_t retry = mode ? entasis_query_context_collision_query(&query, f.custom, a, targets[target], b, 0, &m, NULL) : entasis_collision_query(&f.world, f.custom, a, targets[target], b, 0, &m, NULL);
            ENTASIS_TEST_CHECK(retry == ENTASIS_STATUS_OK);
        }
    }
    ENTASIS_TEST_CHECK(entasis_query_context_destroy(&query, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(task_count(&f.task.errors) == 0 && task_count(&f.task.child) > 0);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&f.world, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
