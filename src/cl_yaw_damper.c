#include "cl_yaw_damper.h"

void ClYawDamper_Reset(ClYawDamper *yd)
{
    yd->x_wo_dps = 0.0f;
    yd->activo   = false;
    yd->r_wo_dps = 0.0f;
    yd->dr_deg   = 0.0f;
}

float ClYawDamper_Update(ClYawDamper *yd, float K_r, float tau_wo_s,
                         float r_dps, float dt_s)
{
    if (!yd->activo) { yd->x_wo_dps = r_dps; yd->activo = true; }
    yd->x_wo_dps += dt_s / (tau_wo_s + dt_s) * (r_dps - yd->x_wo_dps);
    yd->r_wo_dps  = r_dps - yd->x_wo_dps;
    yd->dr_deg    = K_r * yd->r_wo_dps;
    return yd->dr_deg;
}
