#include "gyro_filter.h"

#include <stddef.h>

void GyroFilter_Init(GyroFilter *f, const float fc_hz[GYRO_FILTER_N_EJES])
{
    if (f == NULL) return;

    for (unsigned i = 0u; i < GYRO_FILTER_N_EJES; ++i) {
        Lpf1_Init(&f->eje[i], (fc_hz != NULL) ? fc_hz[i] : 0.0f);
    }
}

void GyroFilter_Update(GyroFilter *f,
                       const double u[GYRO_FILTER_N_EJES],
                       double dt_s,
                       double y_out[GYRO_FILTER_N_EJES])
{
    if (f == NULL || u == NULL || y_out == NULL) return;

    for (unsigned i = 0u; i < GYRO_FILTER_N_EJES; ++i) {
        y_out[i] = Lpf1_Update(&f->eje[i], u[i], dt_s);
    }
}
