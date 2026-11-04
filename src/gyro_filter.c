#include "gyro_filter.h"

#include "dynamics.h"       // rk4_step_1st
#include "sim_settings.h"

#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
#include <stdio.h>
#endif

#define GYRO_FILTER_PI 3.14159265358979323846

static void filtro_dinamica(double y, double u, double a, double b, double *ydot)
{
    *ydot = a * y + b * u;
}

void GyroFilter_Init(GyroFilter *f, const float fc_hz[GYRO_FILTER_N_EJES])
{
    if (f == NULL) return;

    for (unsigned i = 0u; i < GYRO_FILTER_N_EJES; ++i) {
        float fc = (fc_hz != NULL) ? fc_hz[i] : 0.0f;

        if (fc < 0.0f) {
            fc = 0.0f;
        } else if (fc > GYRO_FILTER_FC_MAX_HZ) {
#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
            fprintf(stderr, "GyroFilter_Init: eje %u, fc = %.1f Hz limitado a %.1f Hz\n",
                    i, (double)fc, (double)GYRO_FILTER_FC_MAX_HZ);
#endif
            fc = GYRO_FILTER_FC_MAX_HZ;
        }

        f->fc_hz[i] = fc;
        f->y[i]     = 0.0;
    }
    f->con_muestra = false;
}

void GyroFilter_Update(GyroFilter *f,
                       const double u[GYRO_FILTER_N_EJES],
                       double dt_s,
                       double y_out[GYRO_FILTER_N_EJES])
{
    if (f == NULL || u == NULL || y_out == NULL) return;

    for (unsigned i = 0u; i < GYRO_FILTER_N_EJES; ++i) {
        if (f->fc_hz[i] <= 0.0f) {
            f->y[i] = u[i];
        } else if (!f->con_muestra) {
            f->y[i] = u[i];
        } else {
            const double tau = 1.0 / (2.0 * GYRO_FILTER_PI * (double)f->fc_hz[i]);
            double y_next = f->y[i];
            rk4_step_1st(filtro_dinamica, f->y[i], u[i], dt_s, -1.0 / tau, 1.0 / tau, &y_next);
            f->y[i] = y_next;
        }
        y_out[i] = f->y[i];
    }
    f->con_muestra = true;
}
