#ifndef ATMOSPHERE_H_INCLUDED
#define ATMOSPHERE_H_INCLUDED

/* ============================================================
 * Atmósfera estándar ISA, troposfera (h < 11 km).
 * Misma formulación y constantes que dynamics/atmosfera.py:
 *
 *   T = T0 - L*h        rho = rho0 * (T/T0)^(g0/(R*L) - 1)
 *
 * Teoría: docs/teoria/sensores/ruido_sensores.tex
 * ============================================================ */

#ifdef __cplusplus
extern "C" {
#endif

#define ISA_RHO0_KG_M3   1.225
#define ISA_T0_K         288.15
#define ISA_L_K_M        0.0065
#define ISA_R_J_KG_K     287.05287
#define ISA_G0_MPS2      9.80665
#define ISA_H_MAX_M      11000.0

/* Densidad ISA [kg/m^3] a la altitud geopotencial h [m].
 * h se satura a [0, ISA_H_MAX_M]: el C no aborta por salir de la troposfera. */
double Atmosphere_IsaDensity(double h_m);

#ifdef __cplusplus
}
#endif

#endif /* ATMOSPHERE_H_INCLUDED */
