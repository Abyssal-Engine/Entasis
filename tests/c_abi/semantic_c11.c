#include "entasis.h"
#include "test_support.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

static int emit_world(uint32_t worker_count)
{
    entasis_world_description_t description = entasis_world_description_default();
    description.threading.worker_count = worker_count;
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    for (int step = 0; step < 8; ++step)
    {
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 120.0f, &diagnostic) == ENTASIS_STATUS_OK);
    }
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    (void)printf(
        "workers=%" PRIu32 " step=%" PRIu64 " active=%" PRId64 " sleeping=%" PRId64
        " islands=%" PRId64 " statics=%" PRId64 " active_constraints=%" PRId64
        " sleeping_constraints=%" PRId64 " active_pairs=%" PRId64 " inactive_pairs=%" PRId64
        " shapes=%" PRId64 " shape_types=%" PRId64 "\n",
        worker_count,
        stats.step_index,
        stats.active_bodies,
        stats.sleeping_bodies,
        stats.sleeping_islands,
        stats.statics,
        stats.active_constraints,
        stats.sleeping_constraints,
        stats.active_pairs,
        stats.inactive_pairs,
        stats.registered_shapes,
        stats.registered_shape_types);
    ENTASIS_TEST_CHECK(entasis_world_clear(&world, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_world_stats_t cleared = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &cleared, &diagnostic) == ENTASIS_STATUS_OK);
    (void)printf(
        "workers=%" PRIu32 " cleared_step=%" PRIu64 " active=%" PRId64 " sleeping=%" PRId64
        " statics=%" PRId64 " shapes=%" PRId64 " shape_types=%" PRId64 "\n",
        worker_count,
        cleared.step_index,
        cleared.active_bodies,
        cleared.sleeping_bodies,
        cleared.statics,
        cleared.registered_shapes,
        cleared.registered_shape_types);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    ENTASIS_TEST_CHECK(emit_world(UINT32_C(1)) == 0);
    ENTASIS_TEST_CHECK(emit_world(UINT32_C(4)) == 0);
    return 0;
}
