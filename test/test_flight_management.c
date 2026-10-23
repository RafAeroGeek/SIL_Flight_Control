/* ===========================================================
 * test_flight_management.c
 * Cubre la infraestructura de flight_management.c en modo MANUAL:
 *   - FM_Init arranca en MANUAL y desarmado
 *   - paso directo bit a bit de los sticks (salida == entrada)
 *   - modos no implementados o fuera de rango se ignoran
 *   - armado y status
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

/* T2: MANUAL copia los sticks sin tocarlos (regresión bit a bit en main.c) */
static void test_paso_directo(void)
{
    FM_Init(NULL, FM_MODE_MANUAL);
    const FM_State st = {0};
    FM_Update(&k_cmd, &st, 0.001f);

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
    FM_Update(&k_cmd, &st, 0.001f);

    FM_Status s;
    FM_GetStatus(&s);
    EXPECT_TRUE(s.mode == FM_MODE_MANUAL);
    EXPECT_TRUE(s.armed);
    EXPECT_FLOAT_EQ(s.cmd_in.roll_norm,  k_cmd.roll_norm);
    EXPECT_FLOAT_EQ(s.cmd_out.pitch_norm, k_cmd.pitch_norm);

    FM_SetArmed(false);
    EXPECT_FALSE(FM_IsArmed());
}

int main(void)
{
    test_init();
    test_paso_directo();
    test_modo_no_implementado();
    test_status();
    TEST_RETURN();
}
