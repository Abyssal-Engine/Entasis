#include "entasis.h"
#include "test_support.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct property_value_t
{
    uint64_t id;
    uint32_t flags;
    float weight;
} property_value_t;

typedef struct allocation_state_t
{
    uint64_t allocations;
    uint64_t reallocations;
    uint64_t deallocations;
    uint64_t live;
    uint64_t fail_after;
} allocation_state_t;

static size_t normalized_alignment(uint64_t requested)
{
    size_t alignment = (size_t)requested;
    if (alignment < sizeof(void *))
        alignment = sizeof(void *);
    return alignment;
}

static size_t rounded_size(uint64_t size, size_t alignment)
{
    size_t result = size == 0u ? 1u : (size_t)size;
    const size_t remainder = result % alignment;
    if (remainder != 0u)
        result += alignment - remainder;
    return result;
}

static int allocation_should_fail(allocation_state_t *state)
{
    if (state->fail_after == 0u)
        return 0;
    return state->allocations + state->reallocations >= state->fail_after;
}

static void *ENTASIS_CALL property_allocate(void *context, uint64_t size, uint64_t alignment)
{
    allocation_state_t *state = (allocation_state_t *)context;
    if (state == NULL || allocation_should_fail(state))
        return NULL;
    const size_t normalized = normalized_alignment(alignment);
    void *memory = ENTASIS_TEST_ALIGNED_ALLOC(normalized, rounded_size(size, normalized));
    if (memory != NULL)
    {
        state->allocations += UINT64_C(1);
        state->live += UINT64_C(1);
    }
    return memory;
}

static void *ENTASIS_CALL property_reallocate(
    void *context,
    void *memory,
    uint64_t old_size,
    uint64_t new_size,
    uint64_t alignment)
{
    allocation_state_t *state = (allocation_state_t *)context;
    if (state == NULL || allocation_should_fail(state))
        return NULL;
    const size_t normalized = normalized_alignment(alignment);
    void *replacement = ENTASIS_TEST_ALIGNED_ALLOC(normalized, rounded_size(new_size, normalized));
    if (replacement == NULL)
        return NULL;
    if (memory != NULL)
    {
        const size_t count = (size_t)(old_size < new_size ? old_size : new_size);
        if (count > 0u)
            (void)memcpy(replacement, memory, count);
        ENTASIS_TEST_ALIGNED_FREE(memory);
    }
    else
    {
        state->live += UINT64_C(1);
    }
    state->reallocations += UINT64_C(1);
    return replacement;
}

static void ENTASIS_CALL property_deallocate(
    void *context,
    void *memory,
    uint64_t size,
    uint64_t alignment)
{
    allocation_state_t *state = (allocation_state_t *)context;
    (void)size;
    (void)alignment;
    if (state == NULL || memory == NULL)
        return;
    ENTASIS_TEST_ALIGNED_FREE(memory);
    state->deallocations += UINT64_C(1);
    if (state->live > 0u)
        state->live -= UINT64_C(1);
}

static entasis_allocator_t property_allocator(allocation_state_t *state)
{
    const entasis_allocator_t allocator = {
        (uint32_t)sizeof(entasis_allocator_t),
        UINT32_C(1),
        state,
        property_allocate,
        property_reallocate,
        property_deallocate};
    return allocator;
}

static property_value_t property_value(uint64_t id)
{
    const property_value_t value = {
        id,
        (uint32_t)(id ^ (id >> UINT64_C(32))),
        (float)(id & UINT64_C(0xffff)) * 0.125f};
    return value;
}

static int property_value_equal(const property_value_t *a, const property_value_t *b)
{
    return a != NULL && b != NULL && memcmp(a, b, sizeof(*a)) == 0;
}

static int test_body_properties(void)
{
    allocation_state_t state = {0};
    const entasis_allocator_t allocator = property_allocator(&state);
    entasis_diagnostic_t diagnostic = {0};
    entasis_body_property_table_t table = {0};
    entasis_body_property_table_t other = {0};

    ENTASIS_TEST_CHECK(entasis_body_property_init(
                           &table, UINT64_C(0), UINT64_C(8), UINT64_C(0), &allocator, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(table.opaque == NULL);
    ENTASIS_TEST_CHECK(state.live == UINT64_C(0));

    ENTASIS_TEST_CHECK(entasis_body_property_init(
                           &table,
                           (uint64_t)sizeof(property_value_t),
                           UINT64_C(64),
                           UINT64_C(4096),
                           &allocator,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_property_capacity(&table) >= UINT64_C(4096));
    ENTASIS_TEST_CHECK(entasis_body_property_ensure_capacity(
                           &table, UINT64_C(8192), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_property_capacity(&table) >= UINT64_C(8192));
    ENTASIS_TEST_CHECK(state.live == UINT64_C(2));

    const entasis_body_handle_t handle = {17};
    const property_value_t first = property_value(UINT64_C(17));
    const property_value_t updated = property_value(UINT64_C(1700));
    const void *loaded = NULL;
    entasis_body_property_key_t key = {0};

    ENTASIS_TEST_CHECK(entasis_body_property_set(
                           &table, handle, &first, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_property_get(
                           &table, handle, &loaded, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(property_value_equal((const property_value_t *)loaded, &first));
    ENTASIS_TEST_CHECK(((uintptr_t)loaded & (uintptr_t)63) == (uintptr_t)0);
    ENTASIS_TEST_CHECK(entasis_body_property_key(
                           &table, handle, &key, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(key.generation != UINT64_C(0));
    ENTASIS_TEST_CHECK(key.epoch != UINT64_C(0));

    ENTASIS_TEST_CHECK(entasis_body_property_set(
                           &table, handle, &updated, &diagnostic) == ENTASIS_STATUS_OK);
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_body_property_get_key(
                           &table, key, &loaded, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(property_value_equal((const property_value_t *)loaded, &updated));

    ENTASIS_TEST_CHECK(entasis_body_property_init(
                           &other,
                           (uint64_t)sizeof(property_value_t),
                           (uint64_t)_Alignof(property_value_t),
                           UINT64_C(0),
                           &allocator,
                           &diagnostic) == ENTASIS_STATUS_OK);
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_body_property_get_key(
                           &other, key, &loaded, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);

    const uint64_t allocations_before_batch = state.allocations;
    enum
    {
        BATCH_COUNT = 1024
    };
    entasis_body_handle_t handles[BATCH_COUNT];
    property_value_t values[BATCH_COUNT];
    for (uint64_t index = 0; index < (uint64_t)BATCH_COUNT; ++index)
    {
        handles[index].value = (int32_t)(512u + index);
        values[index] = property_value(index + UINT64_C(10000));
    }
    uint64_t completed = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_body_property_set_batch(
                           &table,
                           handles,
                           values,
                           (uint64_t)sizeof(values[0]),
                           (uint64_t)BATCH_COUNT,
                           &completed,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == (uint64_t)BATCH_COUNT);
    ENTASIS_TEST_CHECK(state.allocations == allocations_before_batch);
    for (uint64_t index = 0; index < (uint64_t)BATCH_COUNT; index += UINT64_C(97))
    {
        loaded = NULL;
        ENTASIS_TEST_CHECK(entasis_body_property_get(
                               &table, handles[index], &loaded, &diagnostic) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(property_value_equal((const property_value_t *)loaded, &values[index]));
    }

    completed = UINT64_C(99);
    ENTASIS_TEST_CHECK(entasis_body_property_set_batch(
                           &table,
                           handles,
                           values,
                           UINT64_C(1),
                           (uint64_t)BATCH_COUNT,
                           &completed,
                           &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(completed == UINT64_C(0));

    ENTASIS_TEST_CHECK(entasis_body_property_remove(
                           &table, handle, &diagnostic) == ENTASIS_STATUS_OK);
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_body_property_get_key(
                           &table, key, &loaded, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_body_property_get(
                           &table, handle, &loaded, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);

    ENTASIS_TEST_CHECK(entasis_body_property_set(
                           &table, handle, &first, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_body_property_key_t replacement_key = {0};
    ENTASIS_TEST_CHECK(entasis_body_property_key(
                           &table, handle, &replacement_key, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(replacement_key.generation != key.generation);
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_body_property_get_key(
                           &table, key, &loaded, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);

    completed = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_body_property_remove_batch(
                           &table, handles, (uint64_t)BATCH_COUNT, &completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(completed == (uint64_t)BATCH_COUNT);

    ENTASIS_TEST_CHECK(entasis_body_property_clear(&table, &diagnostic) == ENTASIS_STATUS_OK);
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_body_property_get_key(
                           &table, replacement_key, &loaded, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);

    ENTASIS_TEST_CHECK(entasis_body_property_destroy(&other, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_property_destroy(&table, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_property_destroy(&table, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    ENTASIS_TEST_CHECK(state.live == UINT64_C(0));
    ENTASIS_TEST_CHECK(state.allocations == state.deallocations);
    return 0;
}

static int test_static_and_collidable_properties(void)
{
    entasis_diagnostic_t diagnostic = {0};
    entasis_static_property_table_t statics = {0};
    entasis_collidable_property_table_t collidables = {0};
    const entasis_static_handle_t static_handle = {9};
    const property_value_t static_value = property_value(UINT64_C(900));
    const void *loaded = NULL;

    ENTASIS_TEST_CHECK(entasis_static_property_init(
                           &statics,
                           (uint64_t)sizeof(property_value_t),
                           (uint64_t)_Alignof(property_value_t),
                           UINT64_C(16),
                           NULL,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_property_capacity(&statics) >= UINT64_C(16));
    ENTASIS_TEST_CHECK(entasis_static_property_ensure_capacity(
                           &statics, UINT64_C(64), &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_property_capacity(&statics) >= UINT64_C(64));
    ENTASIS_TEST_CHECK(entasis_static_property_set(
                           &statics, static_handle, &static_value, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_static_property_key_t static_key = {0};
    ENTASIS_TEST_CHECK(entasis_static_property_key(
                           &statics, static_handle, &static_key, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_property_get_key(
                           &statics, static_key, &loaded, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(property_value_equal((const property_value_t *)loaded, &static_value));
    ENTASIS_TEST_CHECK(entasis_static_property_remove(
                           &statics, static_handle, &diagnostic) == ENTASIS_STATUS_OK);
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_static_property_get_key(
                           &statics, static_key, &loaded, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);

    const entasis_static_handle_t static_handles[4] = {{20}, {21}, {22}, {23}};
    const property_value_t static_values[4] = {
        property_value(UINT64_C(20)), property_value(UINT64_C(21)),
        property_value(UINT64_C(22)), property_value(UINT64_C(23))};
    uint64_t static_completed = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_static_property_set_batch(
                           &statics, static_handles, static_values, (uint64_t)sizeof(static_values[0]),
                           UINT64_C(4), &static_completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(static_completed == UINT64_C(4));
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_static_property_get(
                           &statics, static_handles[2], &loaded, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(property_value_equal((const property_value_t *)loaded, &static_values[2]));
    static_completed = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_static_property_remove_batch(
                           &statics, static_handles, UINT64_C(4), &static_completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(static_completed == UINT64_C(4));
    ENTASIS_TEST_CHECK(entasis_static_property_clear(&statics, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_static_property_destroy(&statics, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(entasis_collidable_property_init(
                           &collidables,
                           (uint64_t)sizeof(property_value_t),
                           UINT64_C(32),
                           UINT64_C(16),
                           UINT64_C(16),
                           NULL,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collidable_property_ensure_capacity(
                           &collidables, UINT64_C(64), UINT64_C(64), &diagnostic) == ENTASIS_STATUS_OK);
    const entasis_collidable_reference_t dynamic_ref = {7u};
    const entasis_collidable_reference_t static_ref = {(UINT32_C(2) << UINT32_C(30)) | UINT32_C(7)};
    const entasis_collidable_reference_t invalid_ref = {(UINT32_C(3) << UINT32_C(30))};
    const property_value_t dynamic_value = property_value(UINT64_C(700));
    const property_value_t collidable_static_value = property_value(UINT64_C(701));
    ENTASIS_TEST_CHECK(entasis_collidable_property_set(
                           &collidables, dynamic_ref, &dynamic_value, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collidable_property_set(
                           &collidables, static_ref, &collidable_static_value, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collidable_property_set(
                           &collidables, invalid_ref, &dynamic_value, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);

    const entasis_collidable_reference_t collidable_batch[2] = {dynamic_ref, static_ref};
    const property_value_t collidable_values[2] = {dynamic_value, collidable_static_value};
    uint64_t collidable_completed = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_collidable_property_set_batch(
                           &collidables, collidable_batch, collidable_values,
                           (uint64_t)sizeof(collidable_values[0]), UINT64_C(2),
                           &collidable_completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(collidable_completed == UINT64_C(2));

    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_collidable_property_get(
                           &collidables, dynamic_ref, &loaded, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(property_value_equal((const property_value_t *)loaded, &dynamic_value));
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_collidable_property_get(
                           &collidables, static_ref, &loaded, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(property_value_equal((const property_value_t *)loaded, &collidable_static_value));

    entasis_collidable_property_key_t key = {0};
    ENTASIS_TEST_CHECK(entasis_collidable_property_key(
                           &collidables, static_ref, &key, &diagnostic) == ENTASIS_STATUS_OK);
    collidable_completed = UINT64_C(0);
    ENTASIS_TEST_CHECK(entasis_collidable_property_remove_batch(
                           &collidables, collidable_batch, UINT64_C(2), &collidable_completed, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(collidable_completed == UINT64_C(2));
    loaded = NULL;
    ENTASIS_TEST_CHECK(entasis_collidable_property_get_key(
                           &collidables, key, &loaded, &diagnostic) == ENTASIS_STATUS_NOT_FOUND);
    ENTASIS_TEST_CHECK(entasis_collidable_property_clear(
                           &collidables, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_collidable_property_destroy(
                           &collidables, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

static int test_failed_growth_preserves_state(void)
{
    allocation_state_t state = {0};
    entasis_allocator_t allocator = property_allocator(&state);
    entasis_diagnostic_t diagnostic = {0};
    entasis_body_property_table_t table = {0};
    const entasis_body_handle_t first_handle = {1};
    const entasis_body_handle_t high_handle = {4096};
    const property_value_t first = property_value(UINT64_C(1));
    const property_value_t high = property_value(UINT64_C(4096));
    const void *loaded = NULL;

    ENTASIS_TEST_CHECK(entasis_body_property_init(
                           &table,
                           (uint64_t)sizeof(property_value_t),
                           (uint64_t)_Alignof(property_value_t),
                           UINT64_C(2),
                           &allocator,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_property_set(
                           &table, first_handle, &first, &diagnostic) == ENTASIS_STATUS_OK);
    const uint64_t capacity_before = entasis_body_property_capacity(&table);
    state.fail_after = state.allocations;
    ENTASIS_TEST_CHECK(entasis_body_property_set(
                           &table, high_handle, &high, &diagnostic) == ENTASIS_STATUS_CAPACITY_MISSING);
    ENTASIS_TEST_CHECK(entasis_body_property_capacity(&table) == capacity_before);
    ENTASIS_TEST_CHECK(entasis_body_property_get(
                           &table, first_handle, &loaded, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(property_value_equal((const property_value_t *)loaded, &first));
    ENTASIS_TEST_CHECK(entasis_body_property_destroy(&table, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(state.live == UINT64_C(0));
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_body_properties() == 0);
    ENTASIS_TEST_CHECK(test_static_and_collidable_properties() == 0);
    ENTASIS_TEST_CHECK(test_failed_growth_preserves_state() == 0);
    return 0;
}
