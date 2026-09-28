/* link to the selected runtime/cooking pair, then pass accept or reject.
 * uses only original public declarations, including with retained old headers */
#include "entasis.h"
#include "entasis_cooking.h"
#include "test_support.h"
#include <string.h>

int main(int argc, char **argv)
{
    if (argc != 2 || (strcmp(argv[1], "accept") != 0 && strcmp(argv[1], "reject") != 0))
        return 2;
    const entasis_status_t expected = strcmp(argv[1], "accept") == 0 ? ENTASIS_STATUS_OK : ENTASIS_STATUS_INVALID_ARGUMENT;
    entasis_world_t world = {0};
    entasis_cooking_context_t context = {0};
    entasis_cooked_hull_t hull = {0};
    entasis_diagnostic_t d = {0};
    entasis_world_description_t description = entasis_world_description_default();
    description.threading.worker_count = 1;
    description.capacity.bodies = 16;
    description.capacity.statics = 16;
    description.capacity.shapes_per_type = 16;
    description.capacity.constraints = 32;
    ENTASIS_TEST_CHECK(entasis_world_init(&world, &description, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooking_context_init(&context, 16384, 4, NULL, &d) == ENTASIS_STATUS_OK);
    const entasis_vector3_t points[8] = {
        {-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1}, {-1, -1, 1}, {1, -1, 1}, {-1, 1, 1}, {1, 1, 1}};
    entasis_vector3_t center = {0};
    entasis_shape_handle_t shape = {0};
    ENTASIS_TEST_CHECK(entasis_cook_hull(&context, points, 8, &hull, &center, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooked_hull_import(&world, &hull, &shape, &d) == expected);
    ENTASIS_TEST_CHECK(entasis_cooked_hull_destroy(&hull, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_cooking_context_destroy(&context, &d) == ENTASIS_STATUS_OK);
    ENTASIS_TEST_CHECK(entasis_world_destroy(&world, &d) == ENTASIS_STATUS_OK);
    return 0;
}
