/* ===========================================================
 * test_flight_management.c
 * Cubre la infraestructura de flight_management.c en modo MANUAL:
 *   - FM_Init arranca en MANUAL y desarmado
 *   - paso directo bit a bit de los sticks (salida == entrada)
 *   - la salida se retiene entre corridas de FM_Update_5ms
 *   - modos no implementados o fuera de rango se ignoran
 *   - armado y status
 *   - mapeo de mode_switch mientras ATTITUDE no está implementado
 * =========================================================== */
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
    FM_SetMode(FM_MODE_GYRO_STAB);
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

    const int rechazados[] = { 1, 2, 3, -1 };
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

int main(void)
{
    test_init();
    test_paso_directo();
    test_modo_no_implementado();
    test_status();
    test_retencion();
    test_mode_switch();
    TEST_RETURN();
}
