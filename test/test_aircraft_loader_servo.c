/* ===========================================================
 * test_aircraft_loader_servo.c
 * Cubre Aircraft_LoadServoCfg:
 *   - overlay parcial (solo claves presentes, dt_s nunca)
 *   - archivo faltante / vacio -> false y out[] intacto
 *   - round-trip del JSON real -> == tabla compilada de servo_sim.c
 *
 * Defines desde CMake:
 *   AIRCRAFT_JSON_REAL : ruta absoluta a aircraft/ss_v1.json
 *   TEST_TMP_DIR       : directorio de escritura para temporales
 * =========================================================== */
#include <stdio.h>

#include "aircraft_loader.h"
#include "servo_sim.h"
#include "test_util.h"

#ifndef AIRCRAFT_JSON_REAL
#error "AIRCRAFT_JSON_REAL no definido (ver CMakeLists.txt)"
#endif
#ifndef TEST_TMP_DIR
#error "TEST_TMP_DIR no definido (ver CMakeLists.txt)"
#endif

/* Misma tabla que test_servo_cfg.c: replica de g_cfg[] en servo_sim.c. */
static const ServoCfg_s kExpectedDefault[SERVO_SIM_N_SERVOS] = {
    [SERVO_ELEVATOR] = {0.01f, 0.18f,  0.001f, 0.0f, -25.0f, 25.0f, 900.0f},
    [SERVO_AILERON]  = {0.01f, 0.18f,  0.001f, 0.0f, -20.0f, 20.0f, 300.0f},
    [SERVO_RUDDER]   = {0.07f, 0.18f,  0.001f, 0.0f, -30.0f, 30.0f, 250.0f},
    [SERVO_THROTTLE] = {0.20f, 0.002f, 0.001f, 0.0f,   0.0f,  1.0f,   2.0f},
};

static void expect_cfg_eq(const ServoCfg_s *a, const ServoCfg_s *b)
{
    EXPECT_FLOAT_EQ(a->tau_s,          b->tau_s);
    EXPECT_FLOAT_EQ(a->k_deg_per_us,   b->k_deg_per_us);
    EXPECT_FLOAT_EQ(a->dt_s,           b->dt_s);
    EXPECT_FLOAT_EQ(a->y0_deg,         b->y0_deg);
    EXPECT_FLOAT_EQ(a->y_min_deg,      b->y_min_deg);
    EXPECT_FLOAT_EQ(a->y_max_deg,      b->y_max_deg);
    EXPECT_FLOAT_EQ(a->rate_limit_dps, b->rate_limit_dps);
}

static void write_file(const char *path, const char *content)
{
    FILE *fp = fopen(path, "w");
    EXPECT_TRUE(fp != NULL);
    if (fp == NULL) {
        return;
    }
    fputs(content, fp);
    fclose(fp);
}

/* T1: overlay parcial -- solo las claves presentes cambian; dt_s nunca. */
static void test_overlay_parcial(void)
{
    const char *path = TEST_TMP_DIR "/servo_override.json";
    write_file(path,
               "{\n"
               "  \"servo_elevator_tau_s\": 0.5,\n"
               "  \"servo_rudder_y_max_deg\": 12.0\n"
               "}\n");

    ServoCfg_s cfg[SERVO_SIM_N_SERVOS];
    Servos_GetDefaultCfg(cfg);
    EXPECT_TRUE(Aircraft_LoadServoCfg(path, cfg));

    EXPECT_FLOAT_EQ(cfg[SERVO_ELEVATOR].tau_s,      0.5f);
    EXPECT_FLOAT_EQ(cfg[SERVO_RUDDER].y_max_deg,    12.0f);

    /* Campos no mencionados: intactos. */
    EXPECT_FLOAT_EQ(cfg[SERVO_ELEVATOR].k_deg_per_us,   0.18f);
    EXPECT_FLOAT_EQ(cfg[SERVO_AILERON].rate_limit_dps,  300.0f);
    EXPECT_FLOAT_EQ(cfg[SERVO_THROTTLE].y_max_deg,      1.0f);

    /* dt_s nunca lo toca el loader. */
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        EXPECT_FLOAT_EQ(cfg[i].dt_s, 0.001f);
    }

    remove(path);
}

/* T2: archivo faltante -> false y cfg intacto. */
static void test_archivo_faltante(void)
{
    ServoCfg_s cfg[SERVO_SIM_N_SERVOS];
    Servos_GetDefaultCfg(cfg);
    EXPECT_FALSE(Aircraft_LoadServoCfg(TEST_TMP_DIR "/no_existe_xyz.json", cfg));
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        expect_cfg_eq(&cfg[i], &kExpectedDefault[i]);
    }
}

/* T3: round-trip del JSON real -> los 4 servos igualan la tabla compilada.
   Bloquea "sin cambio de comportamiento": si el JSON y g_cfg[] se
   desincronizan, este test falla. */
static void test_roundtrip_json_real(void)
{
    ServoCfg_s cfg[SERVO_SIM_N_SERVOS];
    Servos_GetDefaultCfg(cfg);
    EXPECT_TRUE(Aircraft_LoadServoCfg(AIRCRAFT_JSON_REAL, cfg));
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        expect_cfg_eq(&cfg[i], &kExpectedDefault[i]);
    }
}

/* T4: JSON vacio -> false y cfg intacto. */
static void test_json_vacio(void)
{
    const char *path = TEST_TMP_DIR "/servo_vacio.json";
    write_file(path, "");

    ServoCfg_s cfg[SERVO_SIM_N_SERVOS];
    Servos_GetDefaultCfg(cfg);
    EXPECT_FALSE(Aircraft_LoadServoCfg(path, cfg));
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        expect_cfg_eq(&cfg[i], &kExpectedDefault[i]);
    }

    remove(path);
}

int main(void)
{
    test_overlay_parcial();
    test_archivo_faltante();
    test_roundtrip_json_real();
    test_json_vacio();

    if (g_test_fail == 0) {
        printf("test_aircraft_loader_servo: OK\n");
    }
    TEST_RETURN();
}
