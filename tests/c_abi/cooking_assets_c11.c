#include "entasis.h"
#include "entasis_cooking.h"
#include "test_support.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct allocation_state_t
{
    uint64_t allocations;
    uint64_t deallocations;
    uint64_t live;
} allocation_state_t;

static size_t normalize_alignment(uint64_t requested)
{
    size_t value = (size_t)requested;
    if (value < sizeof(void *))
        value = sizeof(void *);
    return value;
}

static size_t normalize_size(uint64_t requested, size_t alignment)
{
    size_t size = requested == 0u ? 1u : (size_t)requested;
    const size_t remainder = size % alignment;
    return remainder == 0u ? size : size + alignment - remainder;
}

static void *ENTASIS_CALL test_allocate(void *user_context, uint64_t size, uint64_t alignment)
{
    allocation_state_t *state = (allocation_state_t *)user_context;
    const size_t normalized_alignment = normalize_alignment(alignment);
    void *memory = ENTASIS_TEST_ALIGNED_ALLOC(normalized_alignment, normalize_size(size, normalized_alignment));
    if (memory != NULL)
    {
        state->allocations += UINT64_C(1);
        state->live += UINT64_C(1);
    }
    return memory;
}

static void *ENTASIS_CALL test_reallocate(
    void *user_context,
    void *memory,
    uint64_t old_size,
    uint64_t new_size,
    uint64_t alignment)
{
    allocation_state_t *state = (allocation_state_t *)user_context;
    const size_t normalized_alignment = normalize_alignment(alignment);
    void *replacement = ENTASIS_TEST_ALIGNED_ALLOC(normalized_alignment, normalize_size(new_size, normalized_alignment));
    if (replacement == NULL)
        return NULL;
    if (memory != NULL)
    {
        (void)memcpy(replacement, memory, (size_t)(old_size < new_size ? old_size : new_size));
        ENTASIS_TEST_ALIGNED_FREE(memory);
    }
    else
    {
        state->live += UINT64_C(1);
    }
    state->allocations += UINT64_C(1);
    if (memory != NULL)
        state->deallocations += UINT64_C(1);
    return replacement;
}

static void ENTASIS_CALL test_deallocate(void *user_context, void *memory, uint64_t size, uint64_t alignment)
{
    allocation_state_t *state = (allocation_state_t *)user_context;
    (void)size;
    (void)alignment;
    if (memory == NULL)
        return;
    ENTASIS_TEST_ALIGNED_FREE(memory);
    state->deallocations += UINT64_C(1);
    state->live -= UINT64_C(1);
}

static entasis_allocator_t test_allocator(allocation_state_t *state)
{
    return (entasis_allocator_t){
        (uint32_t)sizeof(entasis_allocator_t),
        UINT32_C(1),
        state,
        test_allocate,
        test_reallocate,
        test_deallocate};
}

static int test_cooked_assets(void)
{
    allocation_state_t allocations = {0};
    const entasis_allocator_t allocator = test_allocator(&allocations);
    entasis_diagnostic_t diagnostic = {0};
    entasis_cooking_context_t context = {0};
    ENTASIS_TEST_CHECK(entasis_cooking_context_init(&context, INT32_C(131072), INT32_C(32), &allocator, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(context.opaque != NULL);

    entasis_world_description_t world_description = entasis_world_description_default();
    world_description.gravity = (entasis_vector3_t){0.0f, 0.0f, 0.0f};
    world_description.capacity.shapes_per_type = 64;
    entasis_world_t world = {0};
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &world_description, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_vector3_t hull_points[8] = {
        {-1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, -1.0f}, {-1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}, {-1.0f, -1.0f, 1.0f}, {1.0f, -1.0f, 1.0f}, {-1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    entasis_cooked_hull_t hull = {0};
    entasis_vector3_t hull_center = {0};
    ENTASIS_TEST_CHECK(entasis_cook_hull(&context, hull_points, UINT64_C(8), &hull, &hull_center, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooked_hull_validate(&hull) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(fabsf(hull_center.x) < 1.0e-5f);
    entasis_shape_handle_t hull_shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_cooked_hull_import(&world, &hull, &hull_shape, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_handle_is_valid(hull_shape) == ENTASIS_TRUE);

    const entasis_triangle_t tetrahedron[4] = {
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}}};
    entasis_cooked_mesh_t mesh = {0};
    ENTASIS_TEST_CHECK(entasis_cook_mesh(
                           &context,
                           tetrahedron,
                           UINT64_C(4),
                           (entasis_vector3_t){1.0f, 1.0f, 1.0f},
                           &mesh,
                           &diagnostic) == ENTASIS_STATUS_OK);
    entasis_mesh_mass_properties_t closed_properties = {0};
    entasis_mesh_mass_properties_t open_properties = {0};
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_closed_mass_properties(&mesh, 3.0f, ENTASIS_TRUE, &closed_properties, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_open_mass_properties(&mesh, 3.0f, ENTASIS_FALSE, &open_properties, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(closed_properties.inertia.inverse_mass > 0.0f);
    ENTASIS_TEST_CHECK(open_properties.inertia.inverse_mass > 0.0f);
    entasis_shape_handle_t mesh_shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_import(&world, &mesh, &mesh_shape, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_body_inertia_t runtime_open_inertia = {0};
    entasis_body_inertia_t runtime_closed_inertia = {0};
    entasis_vector3_t runtime_open_center = {0};
    entasis_vector3_t runtime_closed_center = {0};
    float runtime_closed_volume = 0.0f;
    ENTASIS_TEST_CHECK(entasis_mesh_open_inertia(
                           &world,
                           mesh_shape,
                           2.0f,
                           &runtime_open_inertia,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_mesh_closed_inertia(
                           &world,
                           mesh_shape,
                           2.0f,
                           &runtime_closed_inertia,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_mesh_open_center_of_mass(
                           &world,
                           mesh_shape,
                           &runtime_open_center,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_mesh_closed_center_of_mass(
                           &world,
                           mesh_shape,
                           &runtime_closed_volume,
                           &runtime_closed_center,
                           &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(fabsf(runtime_open_inertia.inverse_mass - 0.5f) < 1.0e-5f);
    ENTASIS_TEST_CHECK(fabsf(runtime_closed_inertia.inverse_mass - 0.5f) < 1.0e-5f);
    ENTASIS_TEST_CHECK(runtime_closed_volume > 0.0f);
    ENTASIS_TEST_CHECK(isfinite(runtime_open_center.x));
    ENTASIS_TEST_CHECK(isfinite(runtime_closed_center.x));

    const entasis_sphere_t sphere = entasis_sphere(0.5f);
    const entasis_box_t box = entasis_box(1.0f, 1.0f, 1.0f);
    entasis_shape_handle_t slots[2] = {0};
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &slots[0], &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_BOX, &box, &slots[1], &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_cooked_compound_child_t compound_children[2] = {
        {{{0.0f, 0.0f, 0.0f, 1.0f}, {-1.0f, 0.0f, 0.0f}, 0.0f}, INT32_C(0)},
        {{{0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, 0.0f}, INT32_C(1)}};
    entasis_cooked_compound_t compound = {0};
    ENTASIS_TEST_CHECK(entasis_cook_compound(&context, compound_children, UINT64_C(2), &compound, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t compound_shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_cooked_compound_import(&world, &compound, slots, UINT64_C(2), &compound_shape, &diagnostic) == ENTASIS_STATUS_OK);

    const entasis_cooked_big_compound_child_t big_children[2] = {
        {{{0.0f, 0.0f, 0.0f, 1.0f}, {-1.0f, 0.0f, 0.0f}, 0.0f},
         INT32_C(0),
         {{-0.5f, -0.5f, -0.5f}, 0.0f, {0.5f, 0.5f, 0.5f}, 0.0f}},
        {{{0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, 0.0f},
         INT32_C(1),
         {{-0.5f, -0.5f, -0.5f}, 0.0f, {0.5f, 0.5f, 0.5f}, 0.0f}}};
    entasis_cooked_big_compound_t big_compound = {0};
    ENTASIS_TEST_CHECK(entasis_cook_big_compound(&context, big_children, UINT64_C(2), &big_compound, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t big_shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_cooked_big_compound_import(&world, &big_compound, slots, UINT64_C(2), &big_shape, &diagnostic) == ENTASIS_STATUS_OK);

    /* runtime and cooking are separate libraries but share one read-phase owner */
    ENTASIS_TEST_CHECK(entasis_world_begin_read(&world, &diagnostic) == ENTASIS_STATUS_OK);
    entasis_shape_handle_t blocked_shape = entasis_shape_handle_invalid();
    ENTASIS_TEST_CHECK(entasis_cooked_hull_import(&world, &hull, &blocked_shape, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_import(&world, &mesh, &blocked_shape, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_cooked_compound_import(&world, &compound, slots, 2, &blocked_shape, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_cooked_big_compound_import(&world, &big_compound, slots, 2, &blocked_shape, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_world_end_read(&world, &diagnostic) == ENTASIS_STATUS_OK);

    entasis_shape_info_t info = {0};
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, hull_shape, &info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.type_id == ENTASIS_SHAPE_TYPE_CONVEX_HULL);
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, mesh_shape, &info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.type_id == ENTASIS_SHAPE_TYPE_MESH);
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, compound_shape, &info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.type_id == ENTASIS_SHAPE_TYPE_COMPOUND);
    ENTASIS_TEST_CHECK(entasis_shape_inspect(&world, big_shape, &info, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(info.type_id == ENTASIS_SHAPE_TYPE_BIG_COMPOUND);

    ENTASIS_TEST_CHECK(entasis_cooking_context_clear(&context, &diagnostic) == ENTASIS_STATUS_SHAPE_IN_USE);
    ENTASIS_TEST_CHECK(entasis_cooking_context_destroy(&context, &diagnostic) == ENTASIS_STATUS_SHAPE_IN_USE);

    ENTASIS_TEST_CHECK(entasis_cooked_hull_destroy(&hull, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooked_hull_validate(&hull) == ENTASIS_STATUS_DISPOSED);
    ENTASIS_TEST_CHECK(entasis_cooked_hull_destroy(&hull, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    ENTASIS_TEST_CHECK(entasis_cooked_mesh_destroy(&mesh, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooked_compound_destroy(&compound, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooked_big_compound_destroy(&big_compound, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooking_context_clear(&context, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooking_context_destroy(&context, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooking_context_destroy(&context, &diagnostic) == ENTASIS_STATUS_DISPOSED);

    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(allocations.live == UINT64_C(0));
    ENTASIS_TEST_CHECK(allocations.allocations == allocations.deallocations);
    return 0;
}

static int test_invalid_inputs(void)
{
    entasis_diagnostic_t diagnostic = {0};
    entasis_cooking_context_t context = {0};
    entasis_cooked_hull_t hull = {0};
    entasis_vector3_t center = {0};
    const entasis_vector3_t too_few[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
    ENTASIS_TEST_CHECK(entasis_cooking_context_init(&context, INT32_C(0), INT32_C(1), NULL, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_cooking_context_init(&context, INT32_C(4096), INT32_C(4), NULL, &diagnostic) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cook_hull(&context, too_few, UINT64_C(3), &hull, &center, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_cook_hull(&context, NULL, UINT64_C(8), &hull, &center, &diagnostic) == ENTASIS_STATUS_INVALID_ARGUMENT);
    ENTASIS_TEST_CHECK(entasis_cooked_hull_import(NULL, &hull, &(entasis_shape_handle_t){0}, &diagnostic) == ENTASIS_STATUS_DISPOSED);
    ENTASIS_TEST_CHECK(entasis_cooking_context_destroy(&context, &diagnostic) == ENTASIS_STATUS_OK);
    return 0;
}

int main(void)
{
    ENTASIS_TEST_CHECK(test_cooked_assets() == 0);
    ENTASIS_TEST_CHECK(test_invalid_inputs() == 0);
    return 0;
}
