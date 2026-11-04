#ifndef GYRO_FILTER_H
#define GYRO_FILTER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Filtro pasa-bajas de 1er orden para los giros, con la misma forma que la
   dinamica de los servos: y' = (u - y) / tau, tau = 1 / (2 pi fc).
   Software de vuelo: corre en Task_5ms.

   Con RK4 y dt = 5 ms la respuesta se aleja de la exacta al subir fc y diverge
   desde 89 Hz, por eso fc se limita a GYRO_FILTER_FC_MAX_HZ. */
#define GYRO_FILTER_N_EJES      3
#define GYRO_FILTER_FC_MAX_HZ   40.0f

typedef struct
{
    float  fc_hz[GYRO_FILTER_N_EJES];   // corte aplicado; 0 = sin filtro
    double y[GYRO_FILTER_N_EJES];
    bool   con_muestra;                 // false hasta la primera muestra
} GyroFilter;

/* fc <= 0 deja ese eje sin filtro. fc > GYRO_FILTER_FC_MAX_HZ se limita y se
   avisa por stderr en la plataforma PC. */
void GyroFilter_Init(GyroFilter *f, const float fc_hz[GYRO_FILTER_N_EJES]);

/* La primera muestra inicializa el estado, sin transitorio desde cero. */
void GyroFilter_Update(GyroFilter *f,
                       const double u[GYRO_FILTER_N_EJES],
                       double dt_s,
                       double y_out[GYRO_FILTER_N_EJES]);

#ifdef __cplusplus
}
#endif

#endif /* GYRO_FILTER_H */
