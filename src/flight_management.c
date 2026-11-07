#include "flight_management.h"
#include "cl_yaw_damper.h"
#include "gyro_filter.h"
#include "lpf1.h"
#include "rc_mixer.h"

#include <math.h>
#include <string.h>

/* Estado y helpers aun sin usar: se conectan al implementar los modos con
   lazos de control (Paso 4, F4+). Quitar FM_UNUSED en cuanto se usen. */
#define FM_UNUSED __attribute__((unused))

/* ============================================================
 * Estado interno
 * ============================================================ */
static FM_Config    g_cfg;
static FM_Status    g_status;
static FM_Actuators g_act_prev FM_UNUSED;

/* Yaw damper: estado de la ley (cl_yaw_damper) */
static ClYawDamper g_yd;

/* Filtro de giros: corre en Flight_Task_5ms */
static GyroFilter g_gyro_filter;
static double     g_gyro_filt_radps[GYRO_FILTER_N_EJES];

/* Filtro de la IAS del pitot: corre en Flight_Task_5ms para que las tareas
   lentas lean una señal ya filtrada y no el ruido crudo submuestreado */
static Lpf1   g_pitot_filter;
static double g_pitot_filt_ms;

/* Mismo literal que SENSORS_SIM_PI: el FM no incluye sensors_sim.h */
#define FC_PI 3.14159265358979323846

/* ============================================================
 * Helpers internos
 * ============================================================ */
static FM_UNUSED float fm_slew_limit(float y_prev, float y_cmd, float rate_per_s, float dt)
{
    if (dt <= 0.0f) return y_cmd;
    const float dy_max = fabsf(rate_per_s) * dt;
    float dy = y_cmd - y_prev;
    if (dy >  dy_max) dy =  dy_max;
    if (dy < -dy_max) dy = -dy_max;
    return y_prev + dy;
}

/* Modos con ley de control implementada. Crece con cada fase del Paso 4. */
static bool fm_modo_implementado(FM_Mode mode)
{
    switch (mode) {
        case FM_MODE_MANUAL:    return true;
        case FM_MODE_GYRO_STAB: return true;
        default:             return false;
    }
}

/* Adaptador del FM para el yaw damper (la ley está en cl_yaw_damper.c).
   Devuelve el stick de yaw: el pedal más la ley convertida a stick. Sin estado
   válido o sin conversión, reinicia la ley y queda solo el pedal. */
static float fm_yaw_damper(float yaw_norm, const FM_State *st, float dt_s)
{
    const float deg_per_stick = g_cfg.rudder_deg_per_stick;

    if (st == NULL || !st->valid || fabsf(deg_per_stick) < 1e-6f) {
        ClYawDamper_Reset(&g_yd);
        return yaw_norm;
    }

    (void)ClYawDamper_Update(&g_yd, g_cfg.lat.K_r, g_cfg.lat.tau_wo_s,
                             st->r_dps, dt_s);

    g_status.yd_r_wo_dps = g_yd.r_wo_dps;
    g_status.yd_dr_deg   = g_yd.dr_deg;

    return yaw_norm + g_status.yd_dr_deg / deg_per_stick;
}

/* Estado para el FM desde los sensores (FM_State va en grados y deg/s).
   En el SIL la IMU muestrea en t = 1, 6, 11 ms y Flight_Task_5ms corre en
   t = 5, 10, 15 ms: los giros llegan con 4 ms de antigüedad, despreciable
   frente a los 160 ms de retardo que tolera el diseño (control_lateral.tex). */
static FM_State fm_state_desde_sensores(const FC_SensorsIn *d,
                                        const double gyro_radps[GYRO_FILTER_N_EJES])
{
    const double rad2deg = 180.0 / FC_PI;
    const FM_State st = {
        .phi_deg   = (float)d->ahrs_phi_deg,
        .theta_deg = (float)d->ahrs_theta_deg,
        .psi_deg   = (float)d->ahrs_psi_deg,
        .p_dps     = (float)(gyro_radps[0] * rad2deg),
        .q_dps     = (float)(gyro_radps[1] * rad2deg),
        .r_dps     = (float)(gyro_radps[2] * rad2deg),
        .alt_m     = (float)d->gps_alt_m,
        .vz_ms     = (float)(-d->gps_vd_ms),
        .arsp_ms   = (float)g_pitot_filt_ms,
        .valid     = d->imu_valid && d->ahrs_valid,
    };
    return st;
}

/* Copia la telemetría de los filtros a g_status */
static void fm_telemetria_filtros(void)
{
    for (unsigned i = 0u; i < GYRO_FILTER_N_EJES; ++i) {
        g_status.gyro_filt_radps[i] = g_gyro_filt_radps[i];
        g_status.gyro_fc_hz[i]      = g_gyro_filter.eje[i].fc_hz;
    }
    g_status.pitot_filt_ms     = g_pitot_filt_ms;
    g_status.pitot_fc_hz       = g_pitot_filter.fc_hz;
    g_status.pitot_con_muestra = g_pitot_filter.con_muestra;
}

/* ============================================================
 * API
 * ============================================================ */
void FM_Init(const FM_Config *cfg, FM_Mode initial_mode)
{
    memset(&g_cfg, 0, sizeof(g_cfg));
    if (cfg != NULL) g_cfg = *cfg;

    /* Con cfg == NULL los cortes quedan en 0: sin filtro */
    GyroFilter_Init(&g_gyro_filter, g_cfg.gyro_fc_hz);
    Lpf1_Init(&g_pitot_filter, g_cfg.pitot_fc_hz);
    for (unsigned i = 0u; i < GYRO_FILTER_N_EJES; ++i) g_gyro_filt_radps[i] = 0.0;
    g_pitot_filt_ms = 0.0;

    memset(&g_status, 0, sizeof(g_status));
    g_status.mode     = FM_MODE_MANUAL;
    g_status.yaw_mode = g_cfg.yaw_mode;
    ClYawDamper_Reset(&g_yd);
    /* Cortes visibles desde el inicio, antes de la primera Flight_Task_5ms */
    fm_telemetria_filtros();
    FM_SetMode(initial_mode);
}

void FM_SetMode(FM_Mode mode)
{
    if (fm_modo_implementado(mode)) g_status.mode = mode;
}

void FM_SetModeSwitch(int mode_switch)
{
    FM_Mode    mode;
    FM_YawMode yaw_mode;

    switch (mode_switch) {
        case 0: mode = FM_MODE_MANUAL;   yaw_mode = FM_YAW_MODE_RATE;         break;
        case 1: mode = FM_MODE_ATTITUDE; yaw_mode = FM_YAW_MODE_RATE;         break;
        case 2: mode = FM_MODE_ATTITUDE; yaw_mode = FM_YAW_MODE_HEADING_HOLD; break;
        case 3: mode = FM_MODE_GYRO_STAB; yaw_mode = FM_YAW_MODE_RATE;        break;
        default: return;
    }

    if (!fm_modo_implementado(mode)) return;
    g_status.mode     = mode;
    g_status.yaw_mode = yaw_mode;
}

FM_Mode FM_GetMode(void)
{
    return g_status.mode;
}

void FM_SetArmed(bool armed)
{
    g_status.armed = armed;
}

bool FM_IsArmed(void)
{
    return g_status.armed;
}

void FM_Update_5ms(const FM_CommandsNorm *rc_cmd,
                   const FM_State        *st,
                   float                  dt_s)
{
    if (rc_cmd == NULL) return;

    g_status.cmd_in      = *rc_cmd;
    g_status.yd_r_wo_dps = 0.0f;
    g_status.yd_dr_deg   = 0.0f;

    switch (g_status.mode) {
        case FM_MODE_GYRO_STAB:
            /* Solo yaw damper: el resto pasa directo */
            g_status.cmd_out = *rc_cmd;
            g_status.cmd_out.yaw_norm = fm_yaw_damper(rc_cmd->yaw_norm, st, dt_s);
            break;

        case FM_MODE_MANUAL:
        default:
            /* Paso directo: el piloto manda las superficies */
            ClYawDamper_Reset(&g_yd);
            g_status.cmd_out = *rc_cmd;
            break;
    }
}

void FM_GetCmdOut(FM_CommandsNorm *out)
{
    if (out != NULL) *out = g_status.cmd_out;
}

void FM_GetStatus(FM_Status *out)
{
    if (out != NULL) *out = g_status;
}

/* ============================================================
 * Tareas del software de vuelo
 * ============================================================ */

void Flight_Task_5ms(uint32_t now_ms, uint32_t dt_ms,
                     const SimRc_s *rc_sim, int mode_cmd,
                     const FC_SensorsIn *sens,
                     float servo_cmd_us[SERVO_SIM_N_SERVOS])
{
    (void)now_ms;
    const float dt_s = (float)dt_ms * 0.001f;

    if (rc_sim == NULL || sens == NULL || servo_cmd_us == NULL) return;

    /* Modo y armado. El modo pedido se ignora si aún no está implementado.
       rc_sim->s[RC_CH_FLAPS] se ignora: no hay superficie de flaps. */
    FM_SetArmed(rc_sim->s[RC_CH_ARM] > 0.5f);
    FM_SetModeSwitch(mode_cmd);
    const FM_CommandsNorm fm_in = {
        .roll_norm  = rc_sim->s[RC_CH_ROLL],
        .pitch_norm = rc_sim->s[RC_CH_PITCH],
        .yaw_norm   = rc_sim->s[RC_CH_YAW],
        .throttle   = rc_sim->s[RC_CH_THROTTLE],
    };

    /* Filtros */
    GyroFilter_Update(&g_gyro_filter, sens->gyro_radps, (double)dt_s, g_gyro_filt_radps);

    /* Sin medición válida se retiene el último valor */
    if (sens->pitot_valid) {
        g_pitot_filt_ms = Lpf1_Update(&g_pitot_filter, sens->pitot_ias_ms, (double)dt_s);
    }

    /* Leyes según el modo: sticks -> comandos de superficie. En MANUAL pasan
       sin cambios. */
    const FM_State st = fm_state_desde_sensores(sens, g_gyro_filt_radps);
    FM_Update_5ms(&fm_in, &st, dt_s);

    /* Comandos -> delta PWM [us] por servo, con trim del transmisor */
    SimRc_s rc_fm = *rc_sim;
    rc_fm.s[RC_CH_ROLL]     = g_status.cmd_out.roll_norm;
    rc_fm.s[RC_CH_PITCH]    = g_status.cmd_out.pitch_norm;
    rc_fm.s[RC_CH_YAW]      = g_status.cmd_out.yaw_norm;
    rc_fm.s[RC_CH_THROTTLE] = g_status.cmd_out.throttle;
    rc_to_servo_cmds_us(&rc_fm, g_cfg.trim_us, servo_cmd_us);

    fm_telemetria_filtros();
}

void Flight_Task_30ms(uint32_t now_ms, uint32_t dt_ms)
{
    (void)now_ms; (void)dt_ms;
    /* Lazo externo de actitud: wing leveler (cl_wing_leveler, F5) */
}

void Flight_Task_150ms(uint32_t now_ms, uint32_t dt_ms)
{
    (void)now_ms; (void)dt_ms;
    /* Rumbo y navegación: heading hold (cl_heading_hold, F6) */
}

void Flight_Task_500ms(uint32_t now_ms, uint32_t dt_ms)
{
    (void)now_ms; (void)dt_ms;
    /* Health y telemetría */
}
