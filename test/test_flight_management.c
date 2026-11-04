/* ===========================================================
 * test_flight_management.c
 * Cubre la infraestructura de flight_management.c en modo MANUAL:
 *   - FM_Init arranca en MANUAL y desarmado
 *   - paso directo bit a bit de los sticks (salida == entrada)
 *   - la salida se retiene entre corridas de FM_Update_5ms
 *   - modos no implementados o fuera de rango se ignoran
 *   - armado y status
 *   - mapeo de mode_switch mientras ATTITUDE no está implementado
 *   - yaw damper en GYRO_STAB: ley, washout, entrada sin salto, pedal y
 *     estado inválido
 * =========================================================== */
#include <math.h>
#include <string.h>

#include "flight_management.h"
#include "test_util.h"

static const FM_CommandsNorm k_cmd = {
    .roll_norm = 0.123456f, .pitch_norm = -0.987654f,
    .yaw_norm  = 1.0f,      .throttle   = 0.5f,
};

/* T1: tras FM_Init el modo es MANUAL y no está armado */
static void test_init(void)
{
    FM_Init(NULL, FM_MODE_MANUAL);
    EXPECT_TRUE(FM_GetMode() == FM_MODE_MANUAL);
    EXPECT_FALSE(FM_IsArmed());

    /* Un modo inicial no implementado cae a MANUAL */
    FM_Init(NULL, FM_MODE_ATTITUDE);
    EXPECT_TRUE(FM_GetMode() == FM_MODE_MANUAL);
}

/* T2: MANUAL copia los sticks sin tocarlos */
static void test_paso_directo(void)
{
    FM_Init(NULL, FM_MODE_MANUAL);
    const FM_State st = {0};
    FM_Update_5ms(&k_cmd, &st, 0.005f);

    FM_CommandsNorm out;
    FM_GetCmdOut(&out);
    EXPECT_TRUE(memcmp(&out, &k_cmd, sizeof out) == 0);
}

/* T3: pedir un modo no implementado o fuera de rango no cambia el modo */
static void test_modo_no_implementado(void)
{
    FM_Init(NULL, FM_MODE_MANUAL);
    FM_SetMode(FM_MODE_ATTITUDE);
    EXPECT_TRUE(FM_GetMode() == FM_MODE_MANUAL);
    FM_SetMode((FM_Mode)99);
    EXPECT_TRUE(FM_GetMode() == FM_MODE_MANUAL);
    FM_SetMode((FM_Mode)-1);
    EXPECT_TRUE(FM_GetMode() == FM_MODE_MANUAL);
}

/* T4: armado y status reflejan entrada, salida y modo */
static void test_status(void)
{
    FM_Init(NULL, FM_MODE_MANUAL);
    FM_SetArmed(true);
    EXPECT_TRUE(FM_IsArmed());

    const FM_State st = {0};
    FM_Update_5ms(&k_cmd, &st, 0.005f);

    FM_Status s;
    FM_GetStatus(&s);
    EXPECT_TRUE(s.mode == FM_MODE_MANUAL);
    EXPECT_TRUE(s.armed);
    EXPECT_FLOAT_EQ(s.cmd_in.roll_norm,  k_cmd.roll_norm);
    EXPECT_FLOAT_EQ(s.cmd_out.pitch_norm, k_cmd.pitch_norm);

    FM_SetArmed(false);
    EXPECT_FALSE(FM_IsArmed());
}

/* T6: mode_switch. ATTITUDE aún no está implementado: 1 y 2 no cambian ni el
   modo ni yaw_mode, igual que un valor fuera de rango. La aceptación real se
   prueba al implementar cada modo (F5, F6). */
static void test_mode_switch(void)
{
    FM_Config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.yaw_mode = FM_YAW_MODE_HEADING_HOLD;
    FM_Init(&cfg, FM_MODE_MANUAL);

    const int rechazados[] = { 1, 2, 4, -1 };
    for (unsigned i = 0u; i < sizeof rechazados / sizeof rechazados[0]; ++i) {
        FM_SetModeSwitch(rechazados[i]);

        FM_Status s;
        FM_GetStatus(&s);
        EXPECT_TRUE(s.mode == FM_MODE_MANUAL);
        EXPECT_TRUE(s.yaw_mode == FM_YAW_MODE_HEADING_HOLD);
    }

    FM_SetModeSwitch(0);
    FM_Status s;
    FM_GetStatus(&s);
    EXPECT_TRUE(s.mode == FM_MODE_MANUAL);
    EXPECT_TRUE(s.yaw_mode == FM_YAW_MODE_RATE);
}

/* T5: la salida vale cero antes de la primera corrida y luego se retiene:
   Task_1ms ve el mismo comando hasta la siguiente corrida de Task_5ms */
static void test_retencion(void)
{
    FM_Init(NULL, FM_MODE_MANUAL);

    FM_CommandsNorm out;
    FM_GetCmdOut(&out);
    EXPECT_FLOAT_EQ(out.roll_norm,  0.0f);
    EXPECT_FLOAT_EQ(out.pitch_norm, 0.0f);
    EXPECT_FLOAT_EQ(out.yaw_norm,   0.0f);
    EXPECT_FLOAT_EQ(out.throttle,   0.0f);

    const FM_State st = {0};
    FM_Update_5ms(&k_cmd, &st, 0.005f);
    FM_GetCmdOut(&out);
    EXPECT_TRUE(memcmp(&out, &k_cmd, sizeof out) == 0);

    /* Otra lectura sin nueva corrida: mismo comando */
    FM_GetCmdOut(&out);
    EXPECT_TRUE(memcmp(&out, &k_cmd, sizeof out) == 0);
}

/* Yaw damper de diseño, en GYRO_STAB y con el stick de -90 deg por unidad */
static void init_yaw_damper(void)
{
    FM_Config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.lat.K_r = 0.42f;
    cfg.lat.tau_wo_s = 1.0f;
    cfg.rudder_deg_per_stick = -90.0f;
    FM_Init(&cfg, FM_MODE_MANUAL);
    FM_SetModeSwitch(3);
}

static FM_Status paso_yd(const FM_CommandsNorm *in, float r_dps, bool valid)
{
    FM_State st;
    memset(&st, 0, sizeof st);
    st.r_dps = r_dps;
    st.valid = valid;
    FM_Update_5ms(in, &st, 0.005f);

    FM_Status s;
    FM_GetStatus(&s);
    return s;
}

/* T7: el switch 3 activa GYRO_STAB y al entrar no hay salto, aunque r != 0 */
static void test_yd_entrada_sin_salto(void)
{
    const FM_CommandsNorm cero = {0};
    init_yaw_damper();
    EXPECT_TRUE(FM_GetMode() == FM_MODE_GYRO_STAB);

    FM_Status s;
    FM_GetStatus(&s);
    EXPECT_TRUE(s.yaw_mode == FM_YAW_MODE_RATE);

    s = paso_yd(&cero, 10.0f, true);
    EXPECT_FLOAT_EQ(s.yd_r_wo_dps, 0.0f);
    EXPECT_FLOAT_EQ(s.cmd_out.yaw_norm, 0.0f);
}

/* T8: ley y signo: r > 0 -> delta_r > 0 (N_dr < 0 frena la guiñada), que en
   stick es negativo. El pedal se suma y los otros ejes pasan directos. */
static void test_yd_ley_y_pedal(void)
{
    init_yaw_damper();
    (void)paso_yd(&k_cmd, 10.0f, true);

    const FM_Status s = paso_yd(&k_cmd, 20.0f, true);
    const float x_wo = 10.0f + (0.005f / 1.005f) * 10.0f;
    const float dr   = 0.42f * (20.0f - x_wo);
    EXPECT_FLOAT_EQ(s.yd_r_wo_dps, 20.0f - x_wo);
    EXPECT_FLOAT_EQ(s.yd_dr_deg, dr);
    EXPECT_TRUE(dr > 0.0f);
    EXPECT_FLOAT_EQ(s.cmd_out.yaw_norm, k_cmd.yaw_norm + dr / -90.0f);
    EXPECT_FLOAT_EQ(s.cmd_out.roll_norm, k_cmd.roll_norm);
    EXPECT_FLOAT_EQ(s.cmd_out.pitch_norm, k_cmd.pitch_norm);
    EXPECT_FLOAT_EQ(s.cmd_out.throttle, k_cmd.throttle);
}

/* T9: una r constante se lava: el timón no pelea contra un viraje sostenido */
static void test_yd_washout(void)
{
    const FM_CommandsNorm cero = {0};
    init_yaw_damper();
    (void)paso_yd(&cero, 0.0f, true);

    FM_Status s = paso_yd(&cero, 10.0f, true);
    EXPECT_TRUE(s.yd_r_wo_dps > 9.9f);

    for (int k = 0; k < 6000; ++k) {   // 30 s = 30 tau
        s = paso_yd(&cero, 10.0f, true);
    }
    EXPECT_TRUE(fabsf(s.yd_r_wo_dps) < 1e-3f);
}

/* T10: sin estado válido queda solo el pedal, y al recuperarlo no hay salto */
static void test_yd_estado_invalido(void)
{
    init_yaw_damper();
    (void)paso_yd(&k_cmd, 10.0f, true);

    FM_Status s = paso_yd(&k_cmd, 50.0f, false);
    EXPECT_FLOAT_EQ(s.yd_dr_deg, 0.0f);
    EXPECT_FLOAT_EQ(s.cmd_out.yaw_norm, k_cmd.yaw_norm);

    s = paso_yd(&k_cmd, 50.0f, true);
    EXPECT_FLOAT_EQ(s.yd_dr_deg, 0.0f);
    EXPECT_FLOAT_EQ(s.cmd_out.yaw_norm, k_cmd.yaw_norm);
}

/* T11: MANUAL no usa la ley, y volver a GYRO_STAB reinicia el washout */
static void test_yd_reentrada(void)
{
    const FM_CommandsNorm cero = {0};
    init_yaw_damper();
    (void)paso_yd(&cero, 0.0f, true);

    FM_SetModeSwitch(0);
    FM_Status s = paso_yd(&cero, 30.0f, true);
    EXPECT_TRUE(s.mode == FM_MODE_MANUAL);
    EXPECT_FLOAT_EQ(s.yd_dr_deg, 0.0f);
    EXPECT_FLOAT_EQ(s.cmd_out.yaw_norm, 0.0f);

    FM_SetModeSwitch(3);
    s = paso_yd(&cero, 30.0f, true);
    EXPECT_FLOAT_EQ(s.yd_dr_deg, 0.0f);
    EXPECT_FLOAT_EQ(s.cmd_out.yaw_norm, 0.0f);
}

int main(void)
{
    test_init();
    test_paso_directo();
    test_modo_no_implementado();
    test_status();
    test_retencion();
    test_mode_switch();
    test_yd_entrada_sin_salto();
    test_yd_ley_y_pedal();
    test_yd_washout();
    test_yd_estado_invalido();
    test_yd_reentrada();
    TEST_RETURN();
}
