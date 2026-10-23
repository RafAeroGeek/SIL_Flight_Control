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
        case FM_MODE_MANUAL: return true;
        default:             return false;
    }
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
    FM_SetMode(initial_mode);
}

void FM_SetMode(FM_Mode mode)
{
    if (fm_modo_implementado(mode)) g_status.mode = mode;
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

void FM_Update(const FM_CommandsNorm *rc_cmd,
               const FM_State        *st,
               float                  dt_s)
{
    (void)st; (void)dt_s;   // los usan los modos con lazos (F4+)
    if (rc_cmd == NULL) return;

    g_status.cmd_in = *rc_cmd;

    switch (g_status.mode) {
        case FM_MODE_MANUAL:
        default:
            /* Paso directo: el piloto manda las superficies */
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
