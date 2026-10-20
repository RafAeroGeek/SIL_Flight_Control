#include "flight_management.h"

#include <math.h>
#include <string.h>

/* Estado y helpers aun sin usar: se conectan al implementar FM_Init/FM_Update
   (v0.3). Quitar FM_UNUSED en cuanto se usen. */
#define FM_UNUSED __attribute__((unused))

/* ============================================================
 * Estado interno
 * ============================================================ */
static FM_Config    g_cfg      FM_UNUSED;
static FM_Status    g_status   FM_UNUSED;
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

/* ============================================================
 * Ejecuciones por cascada
 * ============================================================ */

  void Flight_Control_Attitude_Tier(void)
  {

      //

  }

