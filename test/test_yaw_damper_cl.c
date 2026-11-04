/* ===========================================================
 * test_yaw_damper_cl.c
 * Yaw damper en lazo cerrado, sin ruido: planta lateral y servos a 1 ms, FM a
 * 5 ms, con el mismo ruteo que main.c. Tras una perturbación de r:
 *   - en lazo abierto el dutch roll sigue oscilando (zeta = 0.10)
 *   - con el yaw damper decae mucho más rápido (diseño: polos en -5.49 +/- 3.76j
 *     y -2.32 +/- 0.55j, docs/teoria/control/control_lateral.tex)
 * =========================================================== */
#include <math.h>
#include <string.h>

#include "aircraft_loader.h"
#include "flight_management.h"
#include "flight_sim.h"
#include "pilot_sim.h"
#include "servo_sim.h"
#include "test_util.h"

#define DT_S       0.001f
#define R0_RADPS   0.1      // perturbación de r
#define PI_D       3.14159265358979323846

/* Mayor |r| en [t1_s, t2_s] tras la perturbación, con el switch de modo dado */
static double correr(int mode_switch, double t1_s, double t2_s)
{
    Params p;
    EXPECT_TRUE(Aircraft_LoadParams(UGLY_STICK_JSON, &p));

    ServoCfg_s servo_cfg[SERVO_SIM_N_SERVOS];
    Servos_GetDefaultCfg(servo_cfg);
    (void)Aircraft_LoadServoCfg(UGLY_STICK_JSON, servo_cfg);
    servo_cfg[SERVO_ELEVATOR].y0_deg = p.delta_e_trim_deg;
    Servos_Init_All_Cfg(servo_cfg);

    float trim_us[SERVO_SIM_N_SERVOS] = {0};
    trim_us[SERVO_ELEVATOR] = p.delta_e_trim_deg / servo_cfg[SERVO_ELEVATOR].k_deg_per_us;
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        Servos_SetCmdDeltaUs(i, trim_us[i]);
    }

    FM_Config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.lat.K_r = 0.42f;
    cfg.lat.tau_wo_s = 1.0f;
    cfg.rudder_deg_per_stick = rc_yaw_deg_per_stick(servo_cfg[SERVO_RUDDER].k_deg_per_us);
    FM_Init(&cfg, FM_MODE_MANUAL);

    FlightSim sim;
    FlightSim_Init(&sim, &p, NULL);

    const unsigned k_pert = 100u;   // el modo ya está activo al perturbar
    const unsigned k_fin  = k_pert + (unsigned)(t2_s * 1000.0);
    double r_max = 0.0;
    FM_Actuators act;

    for (unsigned k = 1u; k <= k_fin; ++k) {
        if (k == k_pert) {
            sim.X_lat[2] = R0_RADPS;
        }

        if ((k % 5u) == 0u) {
            FM_State st;
            memset(&st, 0, sizeof st);
            st.r_dps = (float)(sim.X_lat[2] * 180.0 / PI_D);
            st.valid = true;

            const FM_CommandsNorm in = {0};
            FM_SetModeSwitch(mode_switch);
            FM_Update_5ms(&in, &st, 0.005f);

            FM_CommandsNorm out;
            FM_GetCmdOut(&out);
            SimRc_s rc;
            memset(&rc, 0, sizeof rc);
            rc.s[RC_CH_ROLL] = out.roll_norm;
            rc.s[RC_CH_YAW]  = out.yaw_norm;

            float cmd_us[SERVO_SIM_N_SERVOS];
            rc_to_servo_cmds_us(&rc, trim_us, cmd_us);
            for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
                Servos_SetCmdDeltaUs(i, cmd_us[i]);
            }
        }

        Servos_Update_1ms();
        FlightSim_GetActuators(&act, Servos_GetOutDeg(SERVO_AILERON),
                               Servos_GetOutDeg(SERVO_ELEVATOR),
                               Servos_GetOutDeg(SERVO_RUDDER), 0.0f);
        FlightSim_SetActuatorsFromFM(&sim, &act);
        FlightSim_Step(&sim, DT_S);

        const double t = ((double)k - (double)k_pert) * 0.001;
        if (t >= t1_s && t <= t2_s && fabs(sim.X_lat[2]) > r_max) {
            r_max = fabs(sim.X_lat[2]);
        }
    }
    return r_max;
}

/* T1: lazo abierto: a los 2-3 s sigue vivo. Decaimiento de diseño e^(-0.413 t):
   entre 0.29 y 0.44 de R0 */
static void test_lazo_abierto(void)
{
    const double r = correr(0, 2.0, 3.0);
    EXPECT_TRUE(r > 0.15 * R0_RADPS);
}

/* T2: con el yaw damper, a los 2-3 s queda bien por debajo del lazo abierto.
   Polo dominante de diseño -2.32: e^(-2.32*2) = 0.01 */
static void test_lazo_cerrado(void)
{
    const double abierto = correr(0, 2.0, 3.0);
    const double cerrado = correr(3, 2.0, 3.0);
    EXPECT_TRUE(cerrado < 0.05 * R0_RADPS);
    EXPECT_TRUE(cerrado < 0.15 * abierto);
}

int main(void)
{
    test_lazo_abierto();
    test_lazo_cerrado();
    TEST_RETURN();
}
