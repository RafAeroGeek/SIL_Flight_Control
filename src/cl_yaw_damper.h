#ifndef CL_YAW_DAMPER_H
#define CL_YAW_DAMPER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Yaw damper: delta_r = K_r * r_wo, washout de 1er orden por Euler hacia
   atrás (docs/teoria/control/control_lateral.tex). Ley pura: el estado vive
   en el struct; no conoce modos, RC ni validez del estado. Unidades físicas:
   r en deg/s, delta_r en deg, tiempos en s. */
typedef struct
{
    float x_wo_dps;   // estado del washout (r de baja frecuencia)
    bool  activo;     // false -> la próxima Update arranca el washout en r
    float r_wo_dps;   // salida de telemetría: r lavada
    float dr_deg;     // salida de telemetría: delta_r de la ley (respecto al trim)
} ClYawDamper;

/* activo = false, estado y salidas en 0 */
void ClYawDamper_Reset(ClYawDamper *yd);

/* Un paso de la ley. Al arrancar (activo == false) el washout se inicializa
   en r_dps, así que la primera salida es 0 (sin escalón de timón).
   Devuelve delta_r [deg]. */
float ClYawDamper_Update(ClYawDamper *yd, float K_r, float tau_wo_s,
                         float r_dps, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* CL_YAW_DAMPER_H */
