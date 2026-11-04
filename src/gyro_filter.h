#ifndef GYRO_FILTER_H
#define GYRO_FILTER_H

#include "lpf1.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Filtro de los tres giros, un Lpf1 por eje. Software de vuelo: corre en
   Task_5ms. */
#define GYRO_FILTER_N_EJES      3

typedef struct
{
    Lpf1 eje[GYRO_FILTER_N_EJES];
} GyroFilter;

void GyroFilter_Init(GyroFilter *f, const float fc_hz[GYRO_FILTER_N_EJES]);

void GyroFilter_Update(GyroFilter *f,
                       const double u[GYRO_FILTER_N_EJES],
                       double dt_s,
                       double y_out[GYRO_FILTER_N_EJES]);

#ifdef __cplusplus
}
#endif

#endif /* GYRO_FILTER_H */
