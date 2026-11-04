#include "flight_management.h"

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

/* Yaw damper: estado del washout y si la ley corrió en el paso anterior */
static float g_x_wo_dps;
static bool  g_yd_activo;

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

/* Yaw damper: delta_r = K_r * r_wo, con washout por Euler hacia atrás
   (docs/teoria/control/control_lateral.tex). Devuelve el stick de yaw: el pedal
   más la ley. Al activarse, el washout arranca en la r actual para evitar un
   escalón de timón. Sin estado válido o sin conversión, queda solo el pedal. */
static float fm_yaw_damper(float yaw_norm, const FM_State *st, float dt_s)
{
    const float deg_per_stick = g_cfg.rudder_deg_per_stick;

    if (st == NULL || !st->valid || fabsf(deg_per_stick) < 1e-6f) {
        g_yd_activo = false;
        return yaw_norm;
    }

    if (!g_yd_activo) {
        g_x_wo_dps  = st->r_dps;
        g_yd_activo = true;
    }

    const float tau = g_cfg.lat.tau_wo_s;
    g_x_wo_dps += dt_s / (tau + dt_s) * (st->r_dps - g_x_wo_dps);

    g_status.yd_r_wo_dps = st->r_dps - g_x_wo_dps;
    g_status.yd_dr_deg   = g_cfg.lat.K_r * g_status.yd_r_wo_dps;

    return yaw_norm + g_status.yd_dr_deg / deg_per_stick;
}

/* ============================================================
 * API
 * ============================================================ */
void FM_Init(const FM_Config *cfg, FM_Mode initial_mode)
{
    memset(&g_cfg, 0, sizeof(g_cfg));
    if (cfg != NULL) g_cfg = *cfg;

    memset(&g_status, 0, sizeof(g_status));
    g_status.mode     = FM_MODE_MANUAL;
    g_status.yaw_mode = g_cfg.yaw_mode;
    g_x_wo_dps  = 0.0f;
    g_yd_activo = false;
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
            g_yd_activo = false;
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
 * Ejecuciones por cascada
 * ============================================================ */

  void Flight_Control_Attitude_Tier(void)
  {

      //

  }
