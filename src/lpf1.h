#ifndef LPF1_H
#define LPF1_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Filtro pasa-bajas de 1er orden con la misma forma que la dinamica de los
   servos: y' = (u - y) / tau, tau = 1 / (2 pi fc), integrado con RK4.

   Con dt = 5 ms la respuesta se aleja de la exacta al subir fc y diverge desde
   89 Hz, por eso fc se limita a LPF1_FC_MAX_HZ. */
#define LPF1_FC_MAX_HZ   40.0f

typedef struct
{
    float  fc_hz;        // corte aplicado; 0 = sin filtro
    double y;
    bool   con_muestra;  // false hasta la primera muestra
} Lpf1;

/* fc <= 0 deja la senal sin filtro. fc > LPF1_FC_MAX_HZ se limita y se avisa
   por stderr en la plataforma PC. */
void Lpf1_Init(Lpf1 *f, float fc_hz);

/* La primera muestra inicializa el estado, sin transitorio desde cero. */
double Lpf1_Update(Lpf1 *f, double u, double dt_s);

#ifdef __cplusplus
}
#endif

#endif /* LPF1_H */
