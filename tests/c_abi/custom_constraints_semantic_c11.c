#include "custom_constraints_fixture.h"
static uint32_t cc_bits(float f)
{
    uint32_t v;
    memcpy(&v, &f, 4);
    return v;
}
int main(void)
{
    ENTASIS_TEST_CHECK(ENTASIS_TEST_PREPARE_STDOUT() == 0);
    for (unsigned arity = 1; arity <= 4; ++arity)
    {
        entasis_world_t w = {0};
        cc_state_t s = {0};
        ENTASIS_TEST_CHECK(cc_init(&w, &s, arity, 1, 2, 8, NULL) == 0);
        entasis_custom_constraint_registration_t r = cc_registration(&s, 56);
        ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &r, NULL) == ENTASIS_STATUS_OK);
        entasis_body_handle_t b[36];
        entasis_constraint_handle_t h[9];
        float targets[9];
        for (unsigned ci = 0; ci < 9; ++ci)
        {
            targets[ci] = (float)(ci + 1);
            for (unsigned bi = 0; bi < arity; ++bi)
                ENTASIS_TEST_CHECK(cc_body(&w, (float)((ci * arity + bi) * 3), 0, &b[ci * arity + bi]) == 0);
        }
        uint64_t done = 0;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_add_batch(&w, 56, b, 9 * arity, targets, 4, 9, h, &done, NULL) == ENTASIS_STATUS_OK && done == 9);
        targets[8] = 12;
        ENTASIS_TEST_CHECK(entasis_custom_constraint_apply(&w, h[8], 56, &targets[8], 4, NULL) == ENTASIS_STATUS_OK);
        ENTASIS_TEST_CHECK(entasis_world_step(&w, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
        for (unsigned ci = 0; ci < 9; ++ci)
        {
            float stored = 0, impulse = 0;
            uint64_t written = 0, required = 0;
            ENTASIS_TEST_CHECK(entasis_custom_constraint_get(&w, h[ci], 56, &stored, 4, NULL) == ENTASIS_STATUS_OK);
            ENTASIS_TEST_CHECK(entasis_constraint_accumulated_impulses(&w, h[ci], &impulse, 1, &written, &required, NULL) == ENTASIS_STATUS_OK && written == 1 && required == 1);
            printf("arity=%u index=%u handle=%d description=%08x impulse=%08x", arity, ci, h[ci].value, cc_bits(stored), cc_bits(impulse));
            for (unsigned bi = 0; bi < arity; ++bi)
            {
                entasis_body_state_t state = {0};
                ENTASIS_TEST_CHECK(entasis_body_get(&w, b[ci * arity + bi], &state, NULL) == ENTASIS_STATUS_OK);
                printf(" v%u=%08x", bi, cc_bits(state.velocity.linear.x));
            }
            puts("");
        }
        ENTASIS_TEST_CHECK(atomic_load(&s.errors) == 0 && entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK);
    }
    entasis_world_t w = {0};
    cc_state_t s = {0};
    ENTASIS_TEST_CHECK(cc_init(&w, &s, 1, 1, 1, 1, NULL) == 0);
    entasis_custom_constraint_registration_t r = cc_registration(&s, 56);
    ENTASIS_TEST_CHECK(entasis_custom_constraint_register(&w, &r, NULL) == ENTASIS_STATUS_OK);
    entasis_body_handle_t b;
    ENTASIS_TEST_CHECK(cc_body(&w, 0, 1, &b) == 0);
    entasis_constraint_handle_t h[3];
    float zero = 0;
    for (unsigned i = 0; i < 3; ++i)
        ENTASIS_TEST_CHECK(entasis_custom_constraint_add(&w, 56, &b, 1, &zero, 4, &h[i], NULL) == ENTASIS_STATUS_OK);
    for (unsigned i = 0; i < 4; ++i)
        ENTASIS_TEST_CHECK(entasis_world_step(&w, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    entasis_bool_t sleeping = 0;
    ENTASIS_TEST_CHECK(entasis_body_is_sleeping(&w, b, &sleeping, NULL) == ENTASIS_STATUS_OK);
    float invalid = NAN;
    entasis_status_t rejected = entasis_custom_constraint_apply(&w, h[2], 56, &invalid, 4, NULL);
    float target = 2;
    ENTASIS_TEST_CHECK(entasis_custom_constraint_apply(&w, h[2], 56, &target, 4, NULL) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_body_awaken(&w, b, NULL) == ENTASIS_STATUS_OK && entasis_world_step(&w, 1.0f / 60.0f, NULL) == ENTASIS_STATUS_OK);
    entasis_body_state_t state = {0};
    ENTASIS_TEST_CHECK(entasis_body_get(&w, b, &state, NULL) == ENTASIS_STATUS_OK);
    printf("fallback sleeping=%u invalid=%u velocity=%08x\n", sleeping, rejected, cc_bits(state.velocity.linear.x));
    ENTASIS_TEST_CHECK(entasis_world_destroy(&w, NULL) == ENTASIS_STATUS_OK);
    return 0;
}
