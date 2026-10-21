"""
atmosfera.py
============
Atmósfera estándar ISA (troposfera, h < 11 km), común a lon_analysis.py y
latdir_analysis.py.

    T = T0 − L·h        ρ = ρ0·(T/T0)^(g/(R·L) − 1)
"""

RHO0_KG_M3 = 1.225
T0_K       = 288.15
L_K_M      = 0.0065
R_J_KG_K   = 287.05287
G0_MPS2    = 9.80665

_EXP = G0_MPS2 / (R_J_KG_K * L_K_M) - 1.0   # 4.2559


def densidad_isa(h_m: float) -> float:
    """Densidad ISA [kg/m³] a la altitud geopotencial h [m]."""
    if not 0.0 <= h_m < 11000.0:
        raise ValueError(f"ISA troposfera: h = {h_m} m fuera de [0, 11000)")
    return RHO0_KG_M3 * (1.0 - L_K_M * h_m / T0_K) ** _EXP
