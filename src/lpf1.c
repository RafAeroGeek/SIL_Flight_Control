#include "lpf1.h"

#include "dynamics.h"       // rk4_step_1st
#include "sim_settings.h"

#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
#include <stdio.h>
#endif

#define LPF1_PI 3.14159265358979323846

static void lpf1_dinamica(double y, double u, double a, double b, double *ydot)
{
    *ydot = a * y + b * u;
}

void Lpf1_Init(Lpf1 *f, float fc_hz)
{
    if (f == NULL) return;

    if (fc_hz < 0.0f) {
        fc_hz = 0.0f;
    } else if (fc_hz > LPF1_FC_MAX_HZ) {
#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
        fprintf(stderr, "Lpf1_Init: fc = %.1f Hz limitado a %.1f Hz\n",
                (double)fc_hz, (double)LPF1_FC_MAX_HZ);
#endif
        fc_hz = LPF1_FC_MAX_HZ;
    }

    f->fc_hz       = fc_hz;
    f->y           = 0.0;
    f->con_muestra = false;
}

double Lpf1_Update(Lpf1 *f, double u, double dt_s)
{
    if (f == NULL) return u;

    if (f->fc_hz <= 0.0f || !f->con_muestra) {
        f->y = u;
    } else {
        const double tau = 1.0 / (2.0 * LPF1_PI * (double)f->fc_hz);
        double y_next = f->y;
        rk4_step_1st(lpf1_dinamica, f->y, u, dt_s, -1.0 / tau, 1.0 / tau, &y_next);
        f->y = y_next;
    }
    f->con_muestra = true;

    return f->y;
}
