/* ===========================================================
 * test_util.h
 * Macros minimas de aserto para los tests en C (ctest mira el exit code).
 * Cada ejecutable de test incluye este header una sola vez.
 * =========================================================== */
#ifndef TEST_UTIL_H_INCLUDED
#define TEST_UTIL_H_INCLUDED

#include <stdio.h>
#include <math.h>

static int g_test_fail;

#define EXPECT_TRUE(c) do {                                                   \
    if (!(c)) {                                                               \
        printf("FAIL %s:%d: EXPECT_TRUE(%s)\n", __FILE__, __LINE__, #c);      \
        g_test_fail++;                                                        \
    }                                                                         \
} while (0)

#define EXPECT_FALSE(c) do {                                                  \
    if (c) {                                                                  \
        printf("FAIL %s:%d: EXPECT_FALSE(%s)\n", __FILE__, __LINE__, #c);     \
        g_test_fail++;                                                        \
    }                                                                         \
} while (0)

#define EXPECT_FLOAT_EQ(a, b) do {                                            \
    float _a = (float)(a);                                                    \
    float _b = (float)(b);                                                    \
    if (fabsf(_a - _b) > 1e-6f) {                                             \
        printf("FAIL %s:%d: %s (%g) != %s (%g)\n",                            \
               __FILE__, __LINE__, #a, (double)_a, #b, (double)_b);           \
        g_test_fail++;                                                        \
    }                                                                         \
} while (0)

#define TEST_RETURN() return (g_test_fail == 0) ? 0 : 1

#endif /* TEST_UTIL_H_INCLUDED */
