/* ===========================================================
 * test_atmosphere.c
 * Cubre Atmosphere_IsaDensity (atmosphere.c):
 *   - tabla ISA (mismos puntos que dynamics/tests/test_atmosfera.py)
 *   - densidad decreciente con la altitud
 *   - saturación fuera de [0, 11000] m
 * =========================================================== */
#include <math.h>

#include "atmosphere.h"
#include "test_util.h"

/* T1: tabla ISA con 4 decimales */
static void test_tabla(void)
{
    EXPECT_TRUE(fabs(Atmosphere_IsaDensity(0.0)    - 1.2250) < 1e-4);
    EXPECT_TRUE(fabs(Atmosphere_IsaDensity(1000.0) - 1.1117) < 1e-4);
    EXPECT_TRUE(fabs(Atmosphere_IsaDensity(5000.0) - 0.7361) < 1e-4);
}

/* T2: rho(1000) > rho(5000) (aceptación de TODO 4.1) */
static void test_decreciente(void)
{
    EXPECT_TRUE(Atmosphere_IsaDensity(1000.0) > Atmosphere_IsaDensity(5000.0));
}

/* T3: fuera de la troposfera se satura */
static void test_saturacion(void)
{
    EXPECT_TRUE(Atmosphere_IsaDensity(-50.0)   == Atmosphere_IsaDensity(0.0));
    EXPECT_TRUE(Atmosphere_IsaDensity(12000.0) == Atmosphere_IsaDensity(ISA_H_MAX_M));
}

int main(void)
{
    test_tabla();
    test_decreciente();
    test_saturacion();
    TEST_RETURN();
}
