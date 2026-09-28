#include "entasis.h"
#include "test_support.h"

#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>

#if defined(_WIN32)
#include <windows.h>
#ifdef interface
#undef interface
#endif
#else
#include <pthread.h>
#endif

enum
{
    TEST_WORKER_COUNT = 4
};

typedef struct dispatch_probe_t
{
    _Atomic uint64_t mask;
    _Atomic uint32_t calls;
} dispatch_probe_t;

static void ENTASIS_CALL probe_work(uint32_t worker_index, void *work_context)
{
    dispatch_probe_t *probe = (dispatch_probe_t *)work_context;
    if (probe == NULL || worker_index >= 64u)
        return;
    (void)atomic_fetch_or_explicit(&probe->mask, UINT64_C(1) << worker_index, memory_order_relaxed);
    (void)atomic_fetch_add_explicit(&probe->calls, UINT32_C(1), memory_order_relaxed);
}

typedef struct worker_call_t
{
    entasis_dispatch_work_proc_t work;
    void *work_context;
    uint32_t worker_index;
} worker_call_t;

#if defined(_WIN32)
typedef HANDLE worker_thread_t;
static DWORD WINAPI worker_entry(void *context)
{
    worker_call_t *call = (worker_call_t *)context;
    call->work(call->worker_index, call->work_context);
    return (DWORD)0;
}
#else
typedef pthread_t worker_thread_t;
static void *worker_entry(void *context)
{
    worker_call_t *call = (worker_call_t *)context;
    call->work(call->worker_index, call->work_context);
    return NULL;
}
#endif

typedef struct external_dispatcher_t
{
    entasis_buffer_pool_t pools[TEST_WORKER_COUNT];
    _Atomic uint32_t dispatch_calls;
    _Atomic uint32_t worker_pool_calls;
} external_dispatcher_t;

static entasis_dispatcher_status_t ENTASIS_CALL external_dispatch(
    void *user_context,
    entasis_dispatch_work_proc_t work,
    void *work_context,
    uint32_t worker_count)
{
    external_dispatcher_t *dispatcher = (external_dispatcher_t *)user_context;
    if (dispatcher == NULL || work == NULL || worker_count == 0u || worker_count > TEST_WORKER_COUNT)
    {
        return ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT;
    }
    (void)atomic_fetch_add_explicit(&dispatcher->dispatch_calls, UINT32_C(1), memory_order_relaxed);

    worker_thread_t threads[TEST_WORKER_COUNT - 1];
    worker_call_t calls[TEST_WORKER_COUNT - 1];
    for (uint32_t index = 1u; index < worker_count; ++index)
    {
        calls[index - 1u].work = work;
        calls[index - 1u].work_context = work_context;
        calls[index - 1u].worker_index = index;
#if defined(_WIN32)
        threads[index - 1u] = CreateThread(NULL, 0u, worker_entry, &calls[index - 1u], 0u, NULL);
        if (threads[index - 1u] == NULL)
        {
#else
        if (pthread_create(&threads[index - 1u], NULL, worker_entry, &calls[index - 1u]) != 0)
        {
#endif
            abort();
        }
    }
    work(UINT32_C(0), work_context);
    for (uint32_t index = 1u; index < worker_count; ++index)
    {
#if defined(_WIN32)
        if (WaitForSingleObject(threads[index - 1u], INFINITE) != WAIT_OBJECT_0)
            abort();
        if (CloseHandle(threads[index - 1u]) == 0)
            abort();
#else
        if (pthread_join(threads[index - 1u], NULL) != 0)
            abort();
#endif
    }
    return ENTASIS_DISPATCHER_STATUS_OK;
}

static entasis_dispatcher_status_t ENTASIS_CALL external_worker_pool(
    void *user_context,
    uint32_t worker_index,
    entasis_buffer_pool_t *out_pool)
{
    external_dispatcher_t *dispatcher = (external_dispatcher_t *)user_context;
    if (out_pool == NULL)
        return ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT;
    out_pool->opaque = NULL;
    if (dispatcher == NULL || worker_index >= TEST_WORKER_COUNT)
    {
        return ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT;
    }
    (void)atomic_fetch_add_explicit(&dispatcher->worker_pool_calls, UINT32_C(1), memory_order_relaxed);
    *out_pool = dispatcher->pools[worker_index];
    return ENTASIS_DISPATCHER_STATUS_OK;
}

static int test_interface_validation(void)
{
    entasis_dispatcher_interface_t interface = {0};
    ENTASIS_TEST_CHECK(entasis_dispatcher_interface_validate(NULL) == ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_dispatcher_interface_validate(&interface) == ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT);
    interface.struct_size = (uint32_t)sizeof(interface);
    interface.struct_version = UINT32_C(1);
    interface.worker_count = UINT32_C(1);
    ENTASIS_TEST_CHECK(entasis_dispatcher_interface_validate(&interface) == ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT);
    interface.worker_pool = external_worker_pool;
    ENTASIS_TEST_CHECK(entasis_dispatcher_interface_validate(&interface) == ENTASIS_DISPATCHER_STATUS_OK);
    interface.worker_count = UINT32_C(2);
    ENTASIS_TEST_CHECK(entasis_dispatcher_interface_validate(&interface) == ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT);
    interface.dispatch = external_dispatch;
    interface.worker_count = ENTASIS_MAXIMUM_WORKER_COUNT + UINT32_C(1);
    ENTASIS_TEST_CHECK(entasis_dispatcher_interface_validate(&interface) == ENTASIS_DISPATCHER_STATUS_INVALID_ARGUMENT);
    return 0;
}

static int test_included_thread_pool(void)
{
    entasis_diagnostic_t diagnostic = {0};
    entasis_thread_pool_t thread_pool = {0};
    ENTASIS_TEST_CHECK(entasis_thread_pool_init(&thread_pool, TEST_WORKER_COUNT, UINT32_C(16384), NULL, &diagnostic) == ENTASIS_DISPATCHER_STATUS_OK);

    entasis_dispatcher_interface_t interface = {0};
    ENTASIS_TEST_CHECK(entasis_dispatcher_from_thread_pool(&thread_pool, &interface) == ENTASIS_DISPATCHER_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_dispatcher_interface_validate(&interface) == ENTASIS_DISPATCHER_STATUS_OK);
    ENTASIS_TEST_CHECK(interface.worker_count == TEST_WORKER_COUNT);

    dispatch_probe_t probe = {0};
    ENTASIS_TEST_CHECK(interface.dispatch(interface.user_context, probe_work, &probe, TEST_WORKER_COUNT) == ENTASIS_DISPATCHER_STATUS_OK);
    ENTASIS_TEST_CHECK(atomic_load_explicit(&probe.calls, memory_order_relaxed) == TEST_WORKER_COUNT);
    ENTASIS_TEST_CHECK(atomic_load_explicit(&probe.mask, memory_order_relaxed) == UINT64_C(0xF));

    for (uint32_t worker = 0u; worker < TEST_WORKER_COUNT; ++worker)
    {
        entasis_buffer_pool_t borrowed = {0};
        ENTASIS_TEST_CHECK(interface.worker_pool(interface.user_context, worker, &borrowed) == ENTASIS_DISPATCHER_STATUS_OK);
        ENTASIS_TEST_CHECK(borrowed.opaque != NULL);
        ENTASIS_TEST_CHECK(entasis_buffer_pool_clear(&borrowed, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
        ENTASIS_TEST_CHECK(entasis_buffer_pool_destroy(&borrowed, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);

        entasis_world_t rejected_world = {0};
        entasis_world_description_t rejected_description = entasis_world_description_default();
        ENTASIS_TEST_CHECK(entasis_world_init_with_pool(&rejected_world, &rejected_description, &borrowed, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    }

    entasis_world_description_t description = entasis_world_description_default();
    description.threading.worker_count = TEST_WORKER_COUNT;
    description.threading.external_dispatcher = &interface;
    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_thread_pool_destroy(&thread_pool, &diagnostic) == ENTASIS_DISPATCHER_STATUS_BUSY);
    for (int step = 0; step < 8; ++step)
    {
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 120.0f, &diagnostic) == ENTASIS_STATUS_OK);
    }
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.step_index == UINT64_C(8));
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_thread_pool_destroy(&thread_pool, &diagnostic) == ENTASIS_DISPATCHER_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_thread_pool_destroy(&thread_pool, &diagnostic) == ENTASIS_DISPATCHER_STATUS_DISPOSED);
    return 0;
}

static int external_dispatcher_initialize(external_dispatcher_t *dispatcher, entasis_diagnostic_t *diagnostic)
{
    for (uint32_t worker = 0u; worker < TEST_WORKER_COUNT; ++worker)
    {
        const entasis_status_t status = entasis_buffer_pool_init(
            &dispatcher->pools[worker], INT32_C(16384), INT32_C(16), NULL, diagnostic);
        if (status != ENTASIS_STATUS_OK)
        {
            for (uint32_t release = 0u; release < worker; ++release)
            {
                (void)entasis_buffer_pool_destroy(&dispatcher->pools[release], diagnostic);
            }
            return 0;
        }
    }
    return 1;
}

static int external_dispatcher_destroy(external_dispatcher_t *dispatcher, entasis_diagnostic_t *diagnostic)
{
    for (uint32_t worker = 0u; worker < TEST_WORKER_COUNT; ++worker)
    {
        if (entasis_buffer_pool_destroy(&dispatcher->pools[worker], diagnostic) != ENTASIS_STATUS_OK)
            return 0;
    }
    return 1;
}

static int test_external_dispatcher(void)
{
    external_dispatcher_t external = {0};
    entasis_diagnostic_t diagnostic = {0};
    ENTASIS_TEST_CHECK(external_dispatcher_initialize(&external, &diagnostic));

    entasis_dispatcher_interface_t interface = {
        (uint32_t)sizeof(entasis_dispatcher_interface_t),
        UINT32_C(1),
        &external,
        TEST_WORKER_COUNT,
        UINT32_C(0),
        external_dispatch,
        external_worker_pool};
    ENTASIS_TEST_CHECK(entasis_dispatcher_interface_validate(&interface) == ENTASIS_DISPATCHER_STATUS_OK);

    dispatch_probe_t probe = {0};
    ENTASIS_TEST_CHECK(interface.dispatch(interface.user_context, probe_work, &probe, TEST_WORKER_COUNT) == ENTASIS_DISPATCHER_STATUS_OK);
    ENTASIS_TEST_CHECK(atomic_load_explicit(&probe.calls, memory_order_relaxed) == TEST_WORKER_COUNT);
    ENTASIS_TEST_CHECK(atomic_load_explicit(&probe.mask, memory_order_relaxed) == UINT64_C(0xF));

    entasis_world_description_t description = entasis_world_description_default();
    description.threading.worker_count = TEST_WORKER_COUNT;
    description.threading.external_dispatcher = &interface;
    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &diagnostic) == ENTASIS_STATUS_OK);
    for (int step = 0; step < 6; ++step)
    {
        ENTASIS_TEST_CHECK(entasis_world_step(&world, 1.0f / 90.0f, &diagnostic) == ENTASIS_STATUS_OK);
    }
    ENTASIS_TEST_CHECK(atomic_load_explicit(&external.dispatch_calls, memory_order_relaxed) > UINT32_C(1));
    ENTASIS_TEST_CHECK(atomic_load_explicit(&external.worker_pool_calls, memory_order_relaxed) > UINT32_C(0));
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_world_description_t single_description = entasis_world_description_default();
    entasis_world_t single_world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&single_world, &single_description, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step_external(&single_world, 1.0f / 60.0f, &interface, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_world_stats_t stats = {0};
    ENTASIS_TEST_CHECK(entasis_world_stats(&single_world, &stats, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(stats.step_index == UINT64_C(1));
    ENTASIS_TEST_CHECK(entasis_world_destroy(&single_world, &diagnostic) == ENTASIS_STATUS_OK);

    ENTASIS_TEST_CHECK(external_dispatcher_destroy(&external, &diagnostic));
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_interface_validation() == 0);
    ENTASIS_TEST_CHECK(test_included_thread_pool() == 0);
    ENTASIS_TEST_CHECK(test_external_dispatcher() == 0);
    return 0;
}
