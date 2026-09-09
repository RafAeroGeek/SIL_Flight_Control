/* ===========================================================
 * test_servo_cfg.c
 * Cubre el split de servo_sim.c:
 *   - Servos_Init_All_Cfg toma la config del arreglo pasado
 *   - Servos_GetDefaultCfg entrega la tabla compilada g_cfg[]
 *   - Servos_Init_All() y Servos_Init_All_Cfg(NULL) == defaults
 * =========================================================== */
#include "servo_sim.h"
#include "test_util.h"

/* Replica de g_cfg[] en servo_sim.c (lineas 22-33). Si cambian los defaults
   compilados, actualizar esta tabla tambien: el test lo detecta. */
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

/* T1: la config sale del arreglo pasado; y0 fuera de rango se clampa. */
static void test_init_all_cfg_uses_array(void)
{
    ServoCfg_s cfg[SERVO_SIM_N_SERVOS];
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        cfg[i].tau_s          = 0.05f;
        cfg[i].k_deg_per_us   = 0.1f;
        cfg[i].dt_s           = 0.001f;
        cfg[i].y0_deg         = 0.0f;
        cfg[i].y_min_deg      = -10.0f;
        cfg[i].y_max_deg      = 10.0f;
        cfg[i].rate_limit_dps = 0.0f;   /* sin rate limit para este test */
    }
    cfg[SERVO_ELEVATOR].y0_deg = 999.0f;   /* Servo_Init debe clampear a y_max */

    Servos_Init_All_Cfg(cfg);

    EXPECT_FLOAT_EQ(Servos_GetOutDeg(SERVO_ELEVATOR), 10.0f);
    EXPECT_FLOAT_EQ(Servos_GetOutDeg(SERVO_AILERON),   0.0f);

    /* Un paso con comando positivo -> salida positiva y dentro de limites. */
    Servos_SetCmdDeltaUs(SERVO_AILERON, 200.0f);
    Servos_Update_1ms();
    EXPECT_TRUE(Servos_GetOutDeg(SERVO_AILERON) >  0.0f);
    EXPECT_TRUE(Servos_GetOutDeg(SERVO_AILERON) <= 10.0f);
}

/* T2: Servos_GetDefaultCfg == tabla compilada. */
static void test_get_default_cfg(void)
{
    ServoCfg_s cfg[SERVO_SIM_N_SERVOS];
    Servos_GetDefaultCfg(cfg);
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        expect_cfg_eq(&cfg[i], &kExpectedDefault[i]);
    }
}

/* T3 + T4: el wrapper sin args y el fallback NULL equivalen a los defaults. */
static void test_init_all_and_null_fallback(void)
{
    ServoCfg_s def[SERVO_SIM_N_SERVOS];
    Servos_GetDefaultCfg(def);

    Servos_Init_All();
    float y_init_all[SERVO_SIM_N_SERVOS];
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        y_init_all[i] = Servos_GetOutDeg(i);
        EXPECT_FLOAT_EQ(y_init_all[i], def[i].y0_deg);   /* y0 = 0 -> en rango */
    }

    Servos_Init_All_Cfg(NULL);
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        EXPECT_FLOAT_EQ(Servos_GetOutDeg(i), y_init_all[i]);
    }
}

int main(void)
{
    test_init_all_cfg_uses_array();
    test_get_default_cfg();
    test_init_all_and_null_fallback();

    if (g_test_fail == 0) {
        printf("test_servo_cfg: OK\n");
    }
    TEST_RETURN();
}
