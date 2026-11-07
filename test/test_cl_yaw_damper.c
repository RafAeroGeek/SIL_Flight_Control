/* ===========================================================
 * test_cl_yaw_damper.c
 * Cubre cl_yaw_damper.c (ley pura, sin FM):
 *   T1 arranque sin escalón
 *   T2 washout contra la solución discreta exacta
 *   T3 ganancia: dr_deg == K_r * r_wo_dps
 *   T4 Reset vuelve a arrancar sin escalón
 *   T5 igualdad exacta con la implementación anterior (fm_yaw_damper)
 * =========================================================== */
#include <math.h>
#include <stdbool.h>

#include "cl_yaw_damper.h"
#include "test_util.h"

#define DT_S   0.005f
#define TAU_S  1.0f
#define K_R    0.35f

/* T1: la primera Update arranca el washout en r -> salida 0 exacta */
static void test_arranque(void)
{
    ClYawDamper yd;
    ClYawDamper_Reset(&yd);

    const float dr = ClYawDamper_Update(&yd, K_R, TAU_S, 10.0f, DT_S);
    EXPECT_TRUE(dr == 0.0f);
    EXPECT_TRUE(yd.dr_deg == 0.0f);
    EXPECT_TRUE(yd.r_wo_dps == 0.0f);
    EXPECT_TRUE(yd.activo);
}

/* T2 y T3: r0 en la primera muestra y luego r1 constante:
   r_wo_k = (tau/(tau+dt))^k * (r1 - r0), y dr == K_r * r_wo en cada paso */
static void test_washout_y_ganancia(void)
{
    const float r0 = 2.0f, r1 = 12.0f;
    const double a = (double)TAU_S / ((double)TAU_S + (double)DT_S);
    ClYawDamper yd;

    ClYawDamper_Reset(&yd);
    (void)ClYawDamper_Update(&yd, K_R, TAU_S, r0, DT_S);
    for (int k = 1; k <= 2000; ++k) {
        const float dr = ClYawDamper_Update(&yd, K_R, TAU_S, r1, DT_S);
        const double esperado = pow(a, k) * (double)(r1 - r0);

        EXPECT_TRUE(fabs((double)yd.r_wo_dps - esperado) <= 1e-5);
        EXPECT_TRUE(dr == yd.dr_deg);
        EXPECT_TRUE(yd.dr_deg == K_R * yd.r_wo_dps);
    }
}

/* T4: tras Reset, la siguiente Update vuelve a arrancar sin escalón */
static void test_reset(void)
{
    ClYawDamper yd;
    ClYawDamper_Reset(&yd);

    (void)ClYawDamper_Update(&yd, K_R, TAU_S, 0.0f, DT_S);
    for (int k = 0; k < 50; ++k) {
        (void)ClYawDamper_Update(&yd, K_R, TAU_S, 8.0f, DT_S);
    }
    EXPECT_TRUE(yd.dr_deg != 0.0f);

    ClYawDamper_Reset(&yd);
    EXPECT_FALSE(yd.activo);
    EXPECT_TRUE(yd.r_wo_dps == 0.0f);
    EXPECT_TRUE(yd.dr_deg == 0.0f);

    EXPECT_TRUE(ClYawDamper_Update(&yd, K_R, TAU_S, -5.0f, DT_S) == 0.0f);
}

/* Copia literal de la implementación anterior (fm_yaw_damper en
   flight_management.c antes de extraer la ley), con dos static float */
static float g_x_wo_dps;
static bool  g_yd_activo;
static float g_old_r_wo_dps;
static float g_old_dr_deg;

static void yd_viejo(float r_dps, float dt_s)
{
    if (!g_yd_activo) {
        g_x_wo_dps  = r_dps;
        g_yd_activo = true;
    }

    const float tau = TAU_S;
    g_x_wo_dps += dt_s / (tau + dt_s) * (r_dps - g_x_wo_dps);

    g_old_r_wo_dps = r_dps - g_x_wo_dps;
    g_old_dr_deg   = K_R * g_old_r_wo_dps;
}

/* T5: 2000 pasos con r(t) senoidal más escalones, y un reinicio a la mitad:
   igualdad == en cada paso */
static void test_equivalencia(void)
{
    ClYawDamper yd;
    int distintos = 0;

    ClYawDamper_Reset(&yd);
    g_x_wo_dps  = 0.0f;
    g_yd_activo = false;

    for (int k = 0; k < 2000; ++k) {
        const float t = (float)k * DT_S;
        float r = 15.0f * sinf(2.0f * 3.14159265f * 0.7f * t);
        if (k >= 300)  r += 6.0f;
        if (k >= 900)  r -= 11.0f;
        if (k >= 1500) r += 4.5f;

        if (k == 1000) {            // reinicio, como al salir de GYRO_STAB
            ClYawDamper_Reset(&yd);
            g_yd_activo = false;
        }

        const float dr = ClYawDamper_Update(&yd, K_R, TAU_S, r, DT_S);
        yd_viejo(r, DT_S);

        if (!(dr == g_old_dr_deg && yd.r_wo_dps == g_old_r_wo_dps
              && yd.x_wo_dps == g_x_wo_dps)) {
            distintos++;
        }
    }
    EXPECT_TRUE(distintos == 0);
}

int main(void)
{
    test_arranque();
    test_washout_y_ganancia();
    test_reset();
    test_equivalencia();
    TEST_RETURN();
}
