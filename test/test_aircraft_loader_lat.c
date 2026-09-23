/* ===========================================================
 * test_aircraft_loader_lat.c
 * Cubre las claves lateral-direccionales de Aircraft_LoadParams:
 *   - el fixture Navion carga cada derivada, inercia y theta0_deg
 *   - un JSON sin claves laterales deja los defaults (I_x == 1, L_v == 0)
 *
 * Defines desde CMake:
 *   NAVION_LAT_JSON : ruta absoluta a test/data/navion_lat.json
 *   TEST_TMP_DIR    : directorio de escritura para temporales
 * =========================================================== */
#include <stdio.h>

#include "aircraft_loader.h"
#include "test_util.h"

#ifndef NAVION_LAT_JSON
#error "NAVION_LAT_JSON no definido (ver CMakeLists.txt)"
#endif
#ifndef TEST_TMP_DIR
#error "TEST_TMP_DIR no definido (ver CMakeLists.txt)"
#endif

/* T1: fixture Navion (PLAN_LATERAL.md 1.4) -> cada campo cargado. */
static void test_fixture_navion(void)
{
    Params p;
    EXPECT_TRUE(Aircraft_LoadParams(NAVION_LAT_JSON, &p));

    EXPECT_FLOAT_EQ(p.u0_ms,      53.6448f);
    EXPECT_FLOAT_EQ(p.theta0_deg, 0.0f);
    EXPECT_FLOAT_EQ(p.I_x,        1420.9f);
    EXPECT_FLOAT_EQ(p.I_z,        4786.0f);
    EXPECT_FLOAT_EQ(p.I_xz,       0.0f);

    EXPECT_FLOAT_EQ(p.Y_v,  -0.254f);
    EXPECT_FLOAT_EQ(p.Y_p,   0.0f);
    EXPECT_FLOAT_EQ(p.Y_r,   0.0f);
    EXPECT_FLOAT_EQ(p.Y_dr,  3.80f);

    EXPECT_FLOAT_EQ(p.L_v,  -0.29863f);
    EXPECT_FLOAT_EQ(p.L_p,  -8.40f);
    EXPECT_FLOAT_EQ(p.L_r,   2.19f);
    EXPECT_FLOAT_EQ(p.L_da, 28.98f);
    EXPECT_FLOAT_EQ(p.L_dr,  2.548f);

    EXPECT_FLOAT_EQ(p.N_v,   0.08370f);
    EXPECT_FLOAT_EQ(p.N_p,  -0.35f);
    EXPECT_FLOAT_EQ(p.N_r,  -0.76f);
    EXPECT_FLOAT_EQ(p.N_da, -0.2218f);
    EXPECT_FLOAT_EQ(p.N_dr, -4.597f);
}

/* T2: JSON sin claves laterales -> defaults (y warnings por stderr). */
static void test_sin_claves_laterales(void)
{
    const char *path = TEST_TMP_DIR "/sin_lateral.json";
    FILE *fp = fopen(path, "w");
    EXPECT_TRUE(fp != NULL);
    if (fp == NULL) {
        return;
    }
    fputs("{\n  \"name\": \"solo_lon\",\n  \"u0_ms\": 30.0\n}\n", fp);
    fclose(fp);

    Params p;
    EXPECT_TRUE(Aircraft_LoadParams(path, &p));
    EXPECT_FLOAT_EQ(p.u0_ms, 30.0f);
    EXPECT_FLOAT_EQ(p.I_x,   1.0f);
    EXPECT_FLOAT_EQ(p.I_z,   1.0f);
    EXPECT_FLOAT_EQ(p.I_xz,  0.0f);
    EXPECT_FLOAT_EQ(p.L_v,   0.0f);
    EXPECT_FLOAT_EQ(p.N_dr,  0.0f);
    EXPECT_FLOAT_EQ(p.theta0_deg, 0.0f);
}

int main(void)
{
    test_fixture_navion();
    test_sin_claves_laterales();
    TEST_RETURN();
}
