#include "atmosphere.h"

#include <math.h>

double Atmosphere_IsaDensity(double h_m)
{
    if (h_m < 0.0)          h_m = 0.0;
    if (h_m > ISA_H_MAX_M)  h_m = ISA_H_MAX_M;

    /* g0/(R*L) - 1 = 4.2559 */
    const double expo = ISA_G0_MPS2 / (ISA_R_J_KG_K * ISA_L_K_M) - 1.0;

    return ISA_RHO0_KG_M3 * pow(1.0 - ISA_L_K_M * h_m / ISA_T0_K, expo);
}
