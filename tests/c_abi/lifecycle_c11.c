#include "entasis.h"
#include "test_support.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct tracked_allocator_state_t
{
    uint64_t request_count;
    uint64_t allocate_count;
    uint64_t reallocate_count;
    uint64_t deallocate_count;
    uint64_t live_allocations;
    uint64_t fail_on_request;
} tracked_allocator_state_t;

static size_t normalized_alignment(uint64_t requested)
{
    size_t alignment = (size_t)requested;
    if (alignment < sizeof(void *))
        alignment = sizeof(void *);
    return alignment;
}

static size_t rounded_size(uint64_t requested, size_t alignment)
{
    size_t size = (size_t)(requested == 0u ? 1u : requested);
    const size_t remainder = size % alignment;
    if (remainder != 0u)
        size += alignment - remainder;
    return size;
}

static int tracked_should_fail(tracked_allocator_state_t *state)
{
    state->request_count += UINT64_C(1);
    return state->fail_on_request != 0u && state->request_count == state->fail_on_request;
}

static void *ENTASIS_CALL tracked_allocate(void *user_context, uint64_t size, uint64_t alignment)
{
    tracked_allocator_state_t *state = (tracked_allocator_state_t *)user_context;
    if (state == NULL || tracked_should_fail(state))
        return NULL;
    const size_t normalized = normalized_alignment(alignment);
    void *memory = ENTASIS_TEST_ALIGNED_ALLOC(normalized, rounded_size(size, normalized));
    if (memory != NULL)
    {
        state->allocate_count += UINT64_C(1);
        state->live_allocations += UINT64_C(1);
    }
    return memory;
}

static void *ENTASIS_CALL tracked_reallocate(
    void *user_context,
    void *memory,
    uint64_t old_size,
    uint64_t new_size,
    uint64_t alignment)
{
    tracked_allocator_state_t *state = (tracked_allocator_state_t *)user_context;
    if (state == NULL || tracked_should_fail(state))
        return NULL;
    const size_t normalized = normalized_alignment(alignment);
    void *replacement = ENTASIS_TEST_ALIGNED_ALLOC(normalized, rounded_size(new_size, normalized));
    if (replacement == NULL)
        return NULL;
    if (memory != NULL && old_size != 0u && new_size != 0u)
    {
        const size_t copy_size = (size_t)(old_size < new_size ? old_size : new_size);
        (void)memcpy(replacement, memory, copy_size);
    }
    ENTASIS_TEST_ALIGNED_FREE(memory);
    state->reallocate_count += UINT64_C(1);
    if (memory == NULL)
        state->live_allocations += UINT64_C(1);
    return replacement;
}

static void ENTASIS_CALL tracked_deallocate(
    void *user_context,
    void *memory,
    uint64_t size,
    uint64_t alignment)
{
    tracked_allocator_state_t *state = (tracked_allocator_state_t *)user_context;
    (void)size;
    (void)alignment;
    if (state == NULL || memory == NULL)
        return;
    ENTASIS_TEST_ALIGNED_FREE(memory);
    state->deallocate_count += UINT64_C(1);
    if (state->live_allocations > 0u)
        state->live_allocations -= UINT64_C(1);
}

static entasis_allocator_t tracked_allocator(tracked_allocator_state_t *state)
{
    entasis_allocator_t allocator = {
        (uint32_t)sizeof(entasis_allocator_t),
        UINT32_C(1),
        state,
        tracked_allocate,
        tracked_reallocate,
        tracked_deallocate};
    return allocator;
}

static int validate_zero_world_stats(const entasis_world_stats_t *stats, uint64_t step_index)
{
    return stats != NULL &&
           stats->step_index == step_index &&
           stats->active_bodies == 0 &&
           stats->sleeping_bodies == 0 &&
           stats->sleeping_islands == 0 &&
           stats->statics == 0 &&
           stats->active_constraints == 0 &&
           stats->sleeping_constraints == 0 &&
           stats->active_pairs == 0 &&
           stats->inactive_pairs == 0 &&
           stats->registered_shapes == 0 &&
           stats->registered_shape_types == 9;
}

static int test_invalid_arguments(void)
{
    entasis_diagnostic_t diagnostic = {0};
    entasis_world_description_t description = entasis_world_description_default();
    entasis_world_t world = {0};

    ENTASIS_TEST_CHECK(entasis_world_init(NULL, &description, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(diagnostic.operation == ENTASIS_DIAGNOSTIC_OPERATION_WORLD_INITIALIZE);
    ENTASIS_TEST_CHECK(entasis_world_init(&world, NULL, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);

    world.opaque = (void *)(uintptr_t)1u;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    world.opaque = NULL;

    description.struct_version = UINT32_C(2);
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    description = entasis_world_description_default();
    description.struct_size = UINT32_C(0);
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    description = entasis_world_description_default();
    description.damping.linear = 1.01f;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    description = entasis_world_description_default();
    description.solve.velocity_iterations = 0;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);

    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    ENTASIS_TEST_CHECK(entasis_world_clear(&world, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, NULL, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    return 0;
}

static int test_allocator_fault_cleanup(void)
{
    for (uint64_t failure = UINT64_C(1); failure <= UINT64_C(2); ++failure)
    {
        tracked_allocator_state_t state = {0};
        state.fail_on_request = failure;
        entasis_world_description_t description = entasis_world_description_default();
        description.allocator = tracked_allocator(&state);
        entasis_world_t world = {0};
        entasis_diagnostic_t diagnostic = {0};
        ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
        ENTASIS_TEST_CHECK(world.opaque == NULL);
        ENTASIS_TEST_CHECK(state.live_allocations == 0u);
    }

    tracked_allocator_state_t invalid_state = {0};
    entasis_world_description_t invalid = entasis_world_description_default();
    invalid.allocator = tracked_allocator(&invalid_state);
    invalid.allocator.struct_version = UINT32_C(2);
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &invalid, &diagnostic) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(invalid_state.request_count == 0u);
    return 0;
}

static int test_repeated_world_lifecycle(void)
{
    for (int iteration = 0; iteration < 16; ++iteration)
    {
        tracked_allocator_state_t state = {0};
        entasis_world_description_t description = entasis_world_description_default();
        description.allocator = tracked_allocator(&state);
        entasis_world_t world = {0};
        entasis_diagnostic_t diagnostic = {0};

        ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(world.opaque != NULL);
        ENTASIS_TEST_CHECK(state.live_allocations == 2u);

        entasis_world_stats_t stats = {0};
        ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(validate_zero_world_stats(&stats, UINT64_C(0)));

        const uint64_t requests_before_steps = state.request_count;
        for (int step = 0; step < 32; ++step)
        {
            ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 120.0f, &diagnostic) == ENTASIS_STATUS_OK);
        }
        ENTASIS_TEST_CHECK(state.request_count == requests_before_steps);
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 0.0f, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_world_step(&world, -1.0f, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);

        ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(validate_zero_world_stats(&stats, UINT64_C(32)));

        entasis_capacity_hints_t capacity = entasis_capacity_hints_default();
        ENTASIS_TEST_CHECK(entasis_world_ensure_capacity(&world, &capacity, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_resize(&world, &capacity, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_clear(&world, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(validate_zero_world_stats(&stats, UINT64_C(0)));

        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(world.opaque == NULL);
        ENTASIS_TEST_CHECK(state.live_allocations == 0u);
        ENTASIS_TEST_CHECK(state.allocate_count == state.deallocate_count);
        ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    }
    return 0;
}

static int test_external_pool_ownership(void)
{
    tracked_allocator_state_t state = {0};
    entasis_allocator_t allocator = tracked_allocator(&state);
    entasis_diagnostic_t diagnostic = {0};
    entasis_buffer_pool_t pool = {0};

    ENTASIS_TEST_CHECK(entasis_buffer_pool_init(&pool, INT32_C(16384), INT32_C(16), &allocator, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pool.opaque != NULL);
    ENTASIS_TEST_CHECK(state.live_allocations == 1u);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_clear(&pool, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_world_description_t description = entasis_world_description_default();
    description.allocator = allocator;
    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init_with_pool(&world, &description, &pool, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_clear(&pool, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_destroy(&pool, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 60.0f, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_buffer_pool_clear(&pool, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_destroy(&pool, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(pool.opaque == NULL);
    ENTASIS_TEST_CHECK(state.live_allocations == 0u);
    ENTASIS_TEST_CHECK(entasis_buffer_pool_destroy(&pool, &diagnostic) == ENTASIS_STATUS_DISPOSED);

    entasis_buffer_pool_t missing = {0};
    ENTASIS_TEST_CHECK(entasis_world_init_with_pool(&world, &description, &missing, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 2 || (argc == 2 && strcmp(argv[1], "memory") != 0))
    {
        return 2;
    }
    if (argc == 2)
    {
        ENTASIS_TEST_CHECK(test_allocator_fault_cleanup() == 0);
        ENTASIS_TEST_CHECK(test_external_pool_ownership() == 0);
        return 0;
    }
    ENTASIS_TEST_CHECK(test_invalid_arguments() == 0);
    ENTASIS_TEST_CHECK(test_allocator_fault_cleanup() == 0);
    ENTASIS_TEST_CHECK(test_repeated_world_lifecycle() == 0);
    ENTASIS_TEST_CHECK(test_external_pool_ownership() == 0);
    return 0;
}
