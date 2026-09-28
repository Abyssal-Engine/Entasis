#ifndef ENTASIS_C_ABI_TEST_SUPPORT_H
#define ENTASIS_C_ABI_TEST_SUPPORT_H

#include <stdio.h>

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#include <malloc.h>
#define ENTASIS_TEST_ALIGNED_ALLOC(alignment, size) _aligned_malloc((size), (alignment))
#define ENTASIS_TEST_ALIGNED_FREE(memory) _aligned_free((memory))
#define ENTASIS_TEST_PREPARE_STDOUT() (_setmode(_fileno(stdout), _O_BINARY) == -1 ? 1 : 0)
#else
#include <stdlib.h>
#define ENTASIS_TEST_ALIGNED_ALLOC(alignment, size) aligned_alloc((alignment), (size))
#define ENTASIS_TEST_ALIGNED_FREE(memory) free((memory))
#define ENTASIS_TEST_PREPARE_STDOUT() 0
#endif

#define ENTASIS_TEST_CHECK(condition)                                                           \
    do                                                                                          \
    {                                                                                           \
        if (!(condition))                                                                       \
        {                                                                                       \
            (void)fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #condition); \
            return 1;                                                                           \
        }                                                                                       \
    } while (0)

#endif
