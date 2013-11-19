#include <gtest/gtest.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

/* C++ complains about the C99 'restrict' qualifier. Just ignore it. */
#define restrict

/* Buffer equality testing */
#define EXPECT_BUFFER_EQ(reference, actual, element_count) \
    {\
        for (int cmp_i = 0; cmp_i < element_count; cmp_i++ ) {\
            EXPECT_EQ((const uint8_t)reference[cmp_i], (const uint8_t)actual[cmp_i]);\
        }\
    }
