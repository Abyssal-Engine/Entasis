#include "custom_constraints_fixture.h"

static int test_arities(void)
{
    for (unsigned arity = 1; arity <= 4; ++arity)
        for (unsigned workers = 1; workers <= 2; ++workers)
        {
            entasis_world_t worlds[2] = {{0}, {0}};
            cc_state_t states[2] = {0};
            entasis_body_handle_t bodies[2][36];
            entasis_constraint_handle_t handles[2][9];
            for (unsigned wi = 0; wi < 2; ++wi)
            {
                cc_state_t *s = &states[wi];
                entasis_world_t *w = &worlds[wi];
                s->bias = wi ? 7.0f : -3.0f;
                s->reentry = 1;
                ENTASIS_TEST_CHECK(cc_init(w, s, arity, workers, 2, 8, NULL) == 0);
                int id = -1;
                ENTASIS_TEST_CHECK(entasis_custom_constraint_next_type_id(w, &id, NULL) == ENTASIS_STATUS_OK && id == 56);
                entasis_custom_constraint_registration_t r = cc_registration(s, id);
                ENTASIS_TEST_CHECK(entasis_custom_constraint_register(w, &r, NULL) == ENTASIS_STATUS_OK);
                memset(&r, 0, sizeof(r));
                float targets[9];
                for (unsigned ci = 0; ci < 9; ++ci)
                {
                    targets[ci] = (float)(ci + 1);
                    for (unsigned bi = 0; bi < arity; ++bi)
                        ENTASIS_TEST_CHECK(cc_body(w, (float)((ci * arity + bi) * 3), 0, &bodies[wi][ci * arity + bi]) == 0);
                }
                uint64_t done = 99;
                ENTASIS_TEST_CHECK(entasis_custom_constraint_add_batch(w, id, bodies[wi], 9 * arity, targets, sizeof(float), 9, handles[wi], &done, NULL) == ENTASIS_STATUS_OK && done == 9);
            }
            /* both worlds remain alive with the same type ID and different bindings */
            for (unsigned wi = 0; wi < 2; ++wi)
            {
                entasis_world_t *w = &worlds[wi];
                cc_state_t *s = &states[wi];
                ENTASIS_TEST_CHECK(entasis_world_step(w, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
                for (unsigned ci = 0; ci < 9; ++ci)
                    for (unsigned bi = 0; bi < arity; ++bi)
                    {
                        entasis_body_state_t b = {0};
                        ENTASIS_TEST_CHECK(entasis_body_get(w, bodies[wi][ci * arity + bi], &b, NULL) == ENTASIS_STATUS_OK);
                        ENTASIS_TEST_CHECK(b.velocity.linear.x == (float)(ci + 1) + s->bias + (float)bi);
                    }
                for (unsigned phase = 0; phase < 4; ++phase)
                    ENTASIS_TEST_CHECK(atomic_load(&s->phases[phase]) > 0);
                ENTASIS_TEST_CHECK(atomic_load(&s->full) > 0 && atomic_load(&s->partial) > 0 && atomic_load(&s->errors) == 0);
                float stored[9] = {0};
                uint64_t done = 0;
                ENTASIS_TEST_CHECK(entasis_world_begin_read(w, NULL) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(entasis_custom_constraint_get_batch(w, 56, handles[wi], 9, stored, sizeof(float), &done, NULL) == ENTASIS_STATUS_OK && done == 9);
                ENTASIS_TEST_CHECK(entasis_custom_constraint_apply(w, handles[wi][0], 56, stored, sizeof(float), NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
                ENTASIS_TEST_CHECK(entasis_world_end_read(w, NULL) == ENTASIS_STATUS_OK);
                for (unsigned i = 0; i < 9; ++i)
                    ENTASIS_TEST_CHECK(stored[i] == (float)(i + 1));
                entasis_constraint_info_t info = {0};
                ENTASIS_TEST_CHECK(entasis_constraint_inspect(w, handles[wi][8], &info, NULL) == ENTASIS_STATUS_OK && info.type_id == 56 && info.body_count == arity);
                ENTASIS_TEST_CHECK(entasis_constraint_remove(w, handles[wi][0], NULL) == ENTASIS_STATUS_OK);
                ENTASIS_TEST_CHECK(entasis_custom_constraint_get(w, handles[wi][8], 56, stored, sizeof(float), NULL) == ENTASIS_STATUS_OK && stored[0] == 9);
                ENTASIS_TEST_CHECK(entasis_world_destroy(w, NULL) == ENTASIS_STATUS_OK);
            }
        }
    return 0;
}
static int test_registration_and_prefix(void)
{
    entasis_world_t w = {0};
    cc_state_t s = {0};
    ENTASIS_TEST_CHECK(cc_init(&w, &s, 1, 1, 1, 8, NULL) == 0);
    entasis_custom_constraint_registration_t good = cc_registration(&s, 56);
    for (unsigned variant = 0; variant < 18; ++variant)
    {
        entasis_custom_constraint_registration_t bad = good;
        switch (variant)
        {
        case 0:
            bad.struct_size = 8;
            break;
        case 1:
            bad.struct_version = 9;
            break;
        case 2:
            bad.type_id = 0;
            break;
        case 3:
            bad.type_id = 64;
            break;
        case 4:
            bad.body_count = 0;
            break;
        case 5:
            bad.body_count = 5;
            break;
        case 6:
            bad.description_size = 0;
            break;
        case 7:
            bad.description_size = 260;
            break;
        case 8:
            bad.description_size = 3;
            break;
        case 9:
            bad.prestep_bundle_size = 64;
            break;
        case 10:
            bad.impulse_bundle_size = 0;
            break;
        case 11:
            bad.impulse_bundle_size = 1056;
            break;
        case 12:
            bad.impulse_bundle_size = 33;
            break;
        case 13:
            bad.validate = NULL;
            break;
        case 14:
            bad.kernel = NULL;
            break;
        case 15:
            bad.initial_access[1] = 1;
            break;
        case 16:
            bad.solve_access[0] = 128;
            break;
        default:
            bad._reserved0 = 1;
            break;
        }
        ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &bad, NULL) != ENTASIS_STATUS_OK);
        int id = -1;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_next_type_id(&w, &id, NULL) == ENTASIS_STATUS_OK && id == 56);
    }
    ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &good, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &good, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    entasis_body_handle_t b[3];
    for (unsigned i = 0; i < 3; ++i)
        ENTASIS_TEST_CHECK(cc_body(&w, (float)i * 3, 0, &b[i]) == 0);
    struct strided
    {
        float target;
        uint32_t guard;
    } desc[3] = {{1, 17}, {NAN, 18}, {3, 19}}, out[3] = {{-77, 21}, {-77, 22}, {-77, 23}};
    entasis_constraint_handle_t h[3] = {{777}, {777}, {777}};
    uint64_t done = 99;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_add_batch(&w, 56, b, 3, desc, sizeof(desc[0]), 3, h, &done, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(done == 1 && h[0].value >= 0 && h[1].value == -1 && h[2].value == 777);
    desc[1].target = 2;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_add_batch(&w, 56, &b[1], 2, &desc[1], sizeof(desc[0]), 2, &h[1], &done, NULL) == ENTASIS_STATUS_OK && done == 2);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_get_batch(&w, 56, h, 3, out, sizeof(out[0]), &done, NULL) == ENTASIS_STATUS_OK && done == 3);
    for (unsigned i = 0; i < 3; ++i)
        ENTASIS_TEST_CHECK(out[i].target == (float)(i + 1) && out[i].guard == 21 + i);
    desc[0].target = 4;
    desc[1].target = NAN;
    desc[2].target = 6;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_apply_batch(&w, 56, h, 3, desc, sizeof(desc[0]), &done, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION && done == 1);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_get_batch(&w, 56, h, 3, out, sizeof(out[0]), &done, NULL) == ENTASIS_STATUS_OK && out[0].target == 4 && out[1].target == 2 && out[2].target == 3);
    const entasis_constraint_handle_t invalids[3] = {h[0], {-1}, h[2]};
    out[1].target = -77;
    out[2].target = -77;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_get_batch(&w, 56, invalids, 3, out, sizeof(out[0]), &done, NULL) != ENTASIS_STATUS_OK && done == 1 && out[1].target == -77 && out[2].target == -77);
    for (unsigned mode = 0; mode < 5; ++mode)
    {
        const uint64_t count = mode == 0 ? UINT64_MAX : 3;
        const uint64_t stride = mode == 1 ? UINT64_MAX : (mode == 2 ? 3 : 8);
        void *ptr = mode == 3 ? (void *)(uintptr_t)(UINTPTR_MAX - 3) : (mode == 4 ? NULL : (void *)desc);
        ENTASIS_TEST_CHECK(entasis_custom_constraint_apply_batch(&w, 56, h, count, ptr, stride, &done, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT && done == 0);
    }
    ENTASIS_TEST_CHECK(entasis_custom_constraint_add_batch(&w, 56, NULL, 0, NULL, 0, 0, NULL, &done, NULL) == ENTASIS_STATUS_OK && done == 0);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_get_batch(&w, 56, NULL, 0, NULL, 0, &done, NULL) == ENTASIS_STATUS_OK && done == 0);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_apply_batch(&w, 56, NULL, 0, NULL, 0, &done, NULL) == ENTASIS_STATUS_OK && done == 0);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_get_batch(&w, 56, h, 3, out, 8, NULL, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    entasis_constraint_handle_t failed = {77};
    ENTASIS_TEST_CHECK(entasis_custom_constraint_add(&w, 57, b, 1, desc, 4, &failed, NULL) == ENTASIS_STATUS_NOT_FOUND && failed.value == -1);
    good.type_id = 57;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &good, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    s.validation_error = 255;
    float target = 3;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_apply(&w, h[0], 56, &target, 4, NULL) == ENTASIS_STATUS_INVALID_ARGUMENT);
    s.validation_error = 0;
    ENTASIS_TEST_CHECK(entasis_world_clear(&w, NULL) == ENTASIS_STATUS_OK);
    int id = 0;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_next_type_id(&w, &id, NULL) == ENTASIS_STATUS_OK && id == 57);
    ENTASIS_TEST_CHECK(cc_body(&w, 0, 0, b) == 0 && entasis_custom_constraint_add(&w, 56, b, 1, &target, 4, h, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&w, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK && atomic_load(&s.errors) == 0);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
static int test_sleeping_fallback(void)
{
    entasis_world_t w = {0};
    cc_state_t s = {0};
    ENTASIS_TEST_CHECK(cc_init(&w, &s, 1, 1, 1, 1, NULL) == 0);
    entasis_custom_constraint_registration_t r = cc_registration(&s, 56);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &r, NULL) == ENTASIS_STATUS_OK);
    entasis_body_handle_t body;
    ENTASIS_TEST_CHECK(cc_body(&w, 0, 1, &body) == 0);
    entasis_constraint_handle_t h[3];
    float zero = 0;
    for (unsigned i = 0; i < 3; ++i)
        ENTASIS_TEST_CHECK(entasis_custom_constraint_add(&w, 56, &body, 1, &zero, 4, &h[i], NULL) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(entasis_world_step(&w, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    entasis_bool_t asleep = 0;
    ENTASIS_TEST_CHECK(entasis_body_is_sleeping(&w, body, &asleep, NULL) == ENTASIS_STATUS_OK && asleep);
    float value = 2, read = 99;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_get(&w, h[2], 56, &read, 4, NULL) == ENTASIS_STATUS_OK && read == 0);
    s.validation_error = ENTASIS_STATUS_INVALID_DESCRIPTION;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_apply(&w, h[2], 56, &value, 4, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    ENTASIS_TEST_CHECK(entasis_body_awaken(&w, body, NULL) == ENTASIS_STATUS_INVALID_DESCRIPTION);
    s.validation_error = 0;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_apply(&w, h[2], 56, &value, 4, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_awaken(&w, body, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_step(&w, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    entasis_body_state_t state = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(&w, body, &state, NULL) == ENTASIS_STATUS_OK && state.velocity.linear.x == 2);
    ENTASIS_TEST_CHECK(atomic_load(&s.errors) == 0 && atomic_load(&s.partial) > 0);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
typedef struct cc_alloc_t
{
    unsigned calls, fail_from, live, errors;
    uint64_t bytes;
    struct
    {
        void *ptr;
        uint64_t size, alignment;
    } records[64];
    entasis_custom_constraint_registration_t *mutate;
} cc_alloc_t;
static void *ENTASIS_CALL cc_allocate(void *ctx, uint64_t size, uint64_t alignment)
{
    cc_alloc_t *a = (cc_alloc_t *)ctx;
    ++a->calls;
    if (a->mutate)
    {
        memset(a->mutate, 0, sizeof(*a->mutate));
        a->mutate = NULL;
    }
    if (a->fail_from && a->calls >= a->fail_from)
        return NULL;
    const uint64_t align = alignment < sizeof(void *) ? sizeof(void *) : alignment;
    void *p = ENTASIS_TEST_ALIGNED_ALLOC((size_t)align, (size_t)((size + align - 1) & ~(align - 1)));
    if (!p)
        return NULL;
    for (unsigned i = 0; i < 64; ++i)
        if (!a->records[i].ptr)
        {
            a->records[i].ptr = p;
            a->records[i].size = size;
            a->records[i].alignment = alignment;
            ++a->live;
            a->bytes += size;
            return p;
        }
    ENTASIS_TEST_ALIGNED_FREE(p);
    ++a->errors;
    return NULL;
}
static void ENTASIS_CALL cc_free(void *ctx, void *p, uint64_t size, uint64_t alignment)
{
    cc_alloc_t *a = (cc_alloc_t *)ctx;
    for (unsigned i = 0; i < 64; ++i)
        if (a->records[i].ptr == p)
        {
            if ((size && size != a->records[i].size) || (size && alignment != a->records[i].alignment))
                ++a->errors;
            a->bytes -= a->records[i].size;
            a->records[i].ptr = NULL;
            --a->live;
            ENTASIS_TEST_ALIGNED_FREE(p);
            return;
        }
    ++a->errors;
}
static int test_allocation(void)
{
    for (unsigned fail_offset = 1; fail_offset <= 2; ++fail_offset)
    {
        cc_alloc_t a = {0};
        entasis_allocator_t allocator = {sizeof(entasis_allocator_t), 1, &a, cc_allocate, NULL, cc_free};
        entasis_world_t w = {0};
        cc_state_t s = {0};
        ENTASIS_TEST_CHECK(cc_init(&w, &s, 1, 1, 1, 8, &allocator) == 0);
        const unsigned base_live = a.live;
        const uint64_t base_bytes = a.bytes;
        entasis_custom_constraint_registration_t r = cc_registration(&s, 56);
        a.fail_from = a.calls + fail_offset;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &r, NULL) == ENTASIS_STATUS_CAPACITY_MISSING && a.live == base_live && a.bytes == base_bytes);
        int id = -1;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_next_type_id(&w, &id, NULL) == ENTASIS_STATUS_OK && id == 56);
        a.fail_from = 0;
        a.mutate = &r;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &r, NULL) == ENTASIS_STATUS_OK && r.struct_size == 0);
        ENTASIS_TEST_CHECK(a.live == base_live + 2 && a.bytes == base_bytes + 168);
        for (int type = 57; type < 64; ++type)
        {
            r = cc_registration(&s, type);
            ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &r, NULL) == ENTASIS_STATUS_OK);
        }
        ENTASIS_TEST_CHECK(entasis_custom_constraint_next_type_id(&w, &id, NULL) == ENTASIS_STATUS_CAPACITY_MISSING && id == -1);
        ENTASIS_TEST_CHECK(entasis_world_clear(&w, NULL) == ENTASIS_STATUS_OK && a.live == base_live + 16 && a.bytes == base_bytes + 8 * 168);
        a.fail_from = a.calls + 1;
        ENTASIS_TEST_CHECK(entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK && a.live == 0 && a.bytes == 0 && a.errors == 0);
    }
    return 0;
}
int main(void)
{
    ENTASIS_TEST_CHECK(test_arities() == 0);
    ENTASIS_TEST_CHECK(test_registration_and_prefix() == 0);
    ENTASIS_TEST_CHECK(test_sleeping_fallback() == 0);
    ENTASIS_TEST_CHECK(test_allocation() == 0);
    puts("C_CUSTOM_CONSTRAINTS_OK");
    return 0;
}
