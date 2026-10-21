#!/usr/bin/env python3
"""
a_aircraft.py
=============
Lleva las derivadas longitudinales (<aeronave>_lon_resultados.json, salida de
lon_analysis.py) y lateral-direccionales (<aeronave>_latdir_resultados.json,
salida de latdir_analysis.py) al JSON de aeronave que lee el simulador C
(aircraft/*.json), pasándolas de ejes de estabilidad a ejes cuerpo.

Uso (desde la raíz del repo):
    python3 dynamics/a_aircraft.py dynamics/ugly_lon_resultados.json \\
        dynamics/ugly_latdir_resultados.json aircraft/ugly_stick.json [--dry-run]

Ejes: los análisis están en ejes de estabilidad; el C integra en ejes cuerpo,
girados α = atan2(w0, u0) (α_trim) alrededor de y. Con T = [[c, −s], [s, c]]
(c = cos α, s = sin α), que lleva componentes (x, z) de estabilidad a cuerpo:

  Longitudinal: x_b = P·x_s con P = diag(T, 1, 1) sobre [du, dw, dq, dθ]
      A_b = P·A_s·P⁻¹    B_b = P·B_s
    Sale A_b[0][2] = −w0, A_b[1][2] = u0, X_θ = −g cos θ0, Z_θ = −g sin θ0
    y X_δe = −s·Z_δe (el C los arma con w0_ms, u0_ms y X_de).

  Lateral: con I_x, I_z de ejes cuerpo (estimadas) e I_xz = 0, se giran los
  momentos 𝓛 = I_x·L, 𝓝 = I_z·N:
      [𝓛; 𝓝]_b = T·[𝓛; 𝓝]_s          (en v y en los controles)
      [[𝓛p, 𝓛r], [𝓝p, 𝓝r]]_b = T·(…)_s·Tᵀ
      [Y_p, Y_r]_b = [Y_p, Y_r]_s·Tᵀ
  y se vuelve a dividir entre I_x, I_z. Estado β → dv: Y_v = Yβ/V0,
  L_v = Lβ/V0, N_v = Nβ/V0.

Solo reescribe los valores de claves que ya existen en el JSON de aeronave,
conservando su formato (orden, sangría y líneas en blanco).
"""

import argparse
import json
import math
import re
import sys

import numpy as np

from atmosfera import densidad_isa

G_MPS2 = 9.81            # la misma g de lon_analysis.py y latdir_analysis.py
TOL_V0_MS = 0.01         # tolerancia de V0 entre análisis y aeronave
TOL_MASA_KG = 0.01
TOL_RHO_KG_M3 = 1e-4     # ~1 m de altitud
TOL_FORMA = 1e-9         # términos que el C fija por construcción

LONGITUDINAL_SOURCE = "lon_analysis.py (Nelson cap. 3-4), ejes cuerpo"
LATERAL_SOURCE      = "latdir_analysis.py (Nelson cap. 5), ejes cuerpo"


def _T(alpha: float) -> np.ndarray:
    c, s = math.cos(alpha), math.sin(alpha)
    return np.array([[c, -s], [s, c]])


def alpha_trim(lon: dict) -> float:
    cv = lon["condicion_vuelo"]
    return math.atan2(cv["w0_m_s"], cv["u0_m_s"])


def mapear_longitudinal(lon: dict) -> dict:
    """{clave_del_C: valor} longitudinales en ejes cuerpo."""
    cv = lon["condicion_vuelo"]
    a  = alpha_trim(lon)

    P = np.eye(4)
    P[:2, :2] = _T(a)
    A_s = np.array(lon["espacio_de_estados"]["A"])
    B_s = np.array(lon["espacio_de_estados"]["B"])
    A = P @ A_s @ np.linalg.inv(P)
    B = P @ B_s

    u0, w0 = cv["u0_m_s"], cv["w0_m_s"]
    # Términos que build_state_space_matrices() no lee de derivadas
    forma = {
        "A[0][2] = -w0": A[0][2] + w0,
        "A[1][2] = u0" : A[1][2] - u0,
        "A[2][3] = 0 (M_theta_hat)": A[2][3],
        "B[3] = 0"     : B[3][0],
    }
    for nombre, err in forma.items():
        if abs(err) > TOL_FORMA * max(1.0, u0):
            raise ValueError(f"la A longitudinal en ejes cuerpo no tiene la forma del C: "
                             f"{nombre} (error {err:.3g})")

    return {
        "u0_ms"     : u0,
        "w0_ms"     : w0,
        "gamma0_deg": cv["gamma0_deg"],
        "theta0_deg": math.degrees(a) + cv["gamma0_deg"],

        "X_u"    : A[0][0],
        "X_w"    : A[0][1],
        "X_theta": A[0][3],
        "X_de"   : B[0][0],

        "Z_u_hat": A[1][0],
        "Z_w_hat": A[1][1],
        "Z_theta": A[1][3],
        "Z_de"   : B[1][0],

        "M_u_hat": A[2][0],
        "M_w_hat": A[2][1],
        "M_q_hat": A[2][2],
        "M_de"   : B[2][0],
    }


def mapear_lateral(lat: dict, alpha: float) -> dict:
    """{clave_del_C: valor} laterales en ejes cuerpo."""
    V0 = lat["condicion_vuelo"]["V0_m_s"]
    Ix = lat["condicion_vuelo"]["Ix_kg_m2"]
    Iz = lat["condicion_vuelo"]["Iz_kg_m2"]
    d = {}
    for grupo in lat["derivadas_dimensionales"].values():
        d.update(grupo)

    T = _T(alpha)

    def momentos(L, N):
        """[L; N] de estabilidad → ejes cuerpo, normalizados con I_x, I_z."""
        Lb, Nb = T @ np.array([Ix * L, Iz * N])
        return Lb / Ix, Nb / Iz

    L_v, N_v   = momentos(d["Lb_s2"] / V0, d["Nb_s2"] / V0)
    L_da, N_da = momentos(d["Lda_s2"], d["Nda_s2"])
    L_dr, N_dr = momentos(d["Ldr_s2"], d["Ndr_s2"])

    D_s = np.array([[Ix * d["Lp_s1"], Ix * d["Lr_s1"]],
                    [Iz * d["Np_s1"], Iz * d["Nr_s1"]]])
    D_b = T @ D_s @ T.T
    Y_p, Y_r = np.array([d["Yp_m_s"], d["Yr_m_s"]]) @ T.T

    return {
        "I_x" : Ix,
        "I_z" : Iz,
        "I_xz": 0.0,

        "Y_v" : d["Yb_m_s2"] / V0,
        "Y_p" : Y_p,
        "Y_r" : Y_r,
        "Y_dr": d["Ydr_m_s2"],

        "L_v" : L_v,
        "L_p" : D_b[0][0] / Ix,
        "L_r" : D_b[0][1] / Ix,
        "L_da": L_da,
        "L_dr": L_dr,

        "N_v" : N_v,
        "N_p" : D_b[1][0] / Iz,
        "N_r" : D_b[1][1] / Iz,
        "N_da": N_da,
        "N_dr": N_dr,
    }


def verificar_condicion(lon: dict, lat: dict, avion: dict) -> list[str]:
    """Errores si lon, lat y aeronave no comparten la condición de vuelo."""
    errores = []
    cl, ca = lon["condicion_vuelo"], lat["condicion_vuelo"]
    for nombre, V0 in (("lon", cl["V0_m_s"]), ("lat", ca["V0_m_s"])):
        if abs(V0 - avion["V0_ms"]) > TOL_V0_MS:
            errores.append(f"V0: {nombre} {V0:.4f} m/s vs aeronave {avion['V0_ms']} m/s")
    for nombre, W in (("lon", cl["W_N"]), ("lat", ca["W_N"])):
        if abs(W / G_MPS2 - avion["masa_kg"]) > TOL_MASA_KG:
            errores.append(f"masa: {nombre} {W / G_MPS2:.4f} kg vs aeronave {avion['masa_kg']} kg")
    if abs(cl["rho_kg_m3"] - ca["rho_kg_m3"]) > 1e-9:
        errores.append(f"rho: lon {cl['rho_kg_m3']:.6f} vs lat {ca['rho_kg_m3']:.6f} kg/m3")
    if abs(densidad_isa(avion["Alt_m"]) - cl["rho_kg_m3"]) > TOL_RHO_KG_M3:
        errores.append(f"altitud: rho del análisis {cl['rho_kg_m3']:.4f} no es la ISA "
                       f"de Alt_m = {avion['Alt_m']} m")
    if lat["derivadas_dimensionales"]["control_aleron"].get("Yda_m_s2", 0.0) != 0.0:
        errores.append("Yda != 0: el modelo C no tiene Y_da")
    return errores


def reemplazar_valor(texto: str, clave: str, valor) -> str:
    """Sustituye el valor de "clave": ... conservando el resto de la línea."""
    if isinstance(valor, str):
        nuevo = json.dumps(valor)
    else:
        nuevo = f"{float(valor):.6g}"
        if not any(c in nuevo for c in ".en"):   # 0 -> 0.0, como el resto del JSON
            nuevo += ".0"
    patron = re.compile(r'("' + re.escape(clave) + r'"\s*:\s*)("[^"]*"|[^,\n}]+)')
    texto_nuevo, n = patron.subn(lambda m: m.group(1) + nuevo, texto)
    if n != 1:
        raise KeyError(f"la clave '{clave}' aparece {n} veces en el JSON de aeronave")
    return texto_nuevo


def convertir(ruta_lon: str, ruta_lat: str, ruta_avion: str,
              dry_run: bool = False) -> dict:
    with open(ruta_lon, encoding="utf-8") as f:
        lon = json.load(f)
    with open(ruta_lat, encoding="utf-8") as f:
        lat = json.load(f)
    with open(ruta_avion, encoding="utf-8") as f:
        texto = f.read()
    avion = json.loads(texto)

    errores = verificar_condicion(lon, lat, avion)
    if errores:
        raise ValueError("condición de vuelo distinta:\n  " + "\n  ".join(errores))

    nuevos = mapear_longitudinal(lon)
    nuevos["longitudinal_source"] = LONGITUDINAL_SOURCE
    nuevos.update(mapear_lateral(lat, alpha_trim(lon)))
    nuevos["lateral_source"] = LATERAL_SOURCE

    for clave, valor in nuevos.items():
        texto = reemplazar_valor(texto, clave, valor)

    json.loads(texto)   # el resultado debe seguir siendo JSON válido

    print(f"{'clave':20s} {'antes':>14s} {'ahora':>14s}")
    for clave, valor in nuevos.items():
        antes = avion[clave]
        if isinstance(valor, str):
            print(f"{clave:20s} {antes!s:>14.14s} {valor:>14.14s}")
        else:
            print(f"{clave:20s} {antes:>14.6g} {float(valor):>14.6g}")

    if not dry_run:
        with open(ruta_avion, "w", encoding="utf-8") as f:
            f.write(texto)
        print(f"\n[OK] Escrito: {ruta_avion}")
    else:
        print("\n[DRY-RUN] No se escribió nada")
    return nuevos


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("lon", help="<aeronave>_lon_resultados.json")
    ap.add_argument("lat", help="<aeronave>_latdir_resultados.json")
    ap.add_argument("aeronave", help="aircraft/<aeronave>.json a actualizar")
    ap.add_argument("--dry-run", action="store_true", help="solo mostrar cambios")
    args = ap.parse_args()
    try:
        convertir(args.lon, args.lat, args.aeronave, args.dry_run)
    except (ValueError, KeyError) as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
