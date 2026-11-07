#include "flight_management.h"
#include "cl_yaw_damper.h"

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
    ClYawDamper_Reset(&g_yd);
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
 * Ejecuciones por cascada
 * ============================================================ */

  void Flight_Control_Attitude_Tier(void)
  {

      //

  }
