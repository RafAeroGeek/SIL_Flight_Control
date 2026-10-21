#!/usr/bin/env python3
"""
latdir_a_aircraft.py
====================
Lleva las derivadas lateral-direccionales de <aeronave>_latdir_resultados.json
(salida de latdir_analysis.py) al JSON de aeronave que lee el simulador C
(aircraft/*.json).

Uso (desde la raíz del repo):
    python3 dynamics/latdir_a_aircraft.py dynamics/ugly_latdir_resultados.json \\
        aircraft/ugly_stick.json [--dry-run]

Conversión de estado β (script) → dv (C), con u0 = V0 del análisis:
    Y_v = Yβ/u0    L_v = Lβ/u0    N_v = Nβ/u0
El resto de derivadas pasa directo (mismas unidades). I_xz = 0: el análisis
no modela producto de inercia, así que la corrección I_xz del C no actúa.

Solo reescribe los valores de claves que ya existen en el JSON de aeronave,
conservando su formato (orden, sangría y líneas en blanco).
"""

import argparse
import json
import re
import sys

G_MPS2 = 9.81            # la misma g con la que latdir_analysis.py calcula W
TOL_V0_MS = 0.01         # tolerancia de V0 entre análisis y aeronave
TOL_MASA_KG = 0.01

LATERAL_SOURCE = "latdir_analysis.py (Nelson cap. 5)"


def mapear_derivadas(res: dict) -> dict:
    """Devuelve {clave_del_C: valor} a partir del JSON de resultados."""
    u0 = res["condicion_vuelo"]["V0_m_s"]
    d = {}
    for grupo in res["derivadas_dimensionales"].values():
        d.update(grupo)

    return {
        "I_x" : res["condicion_vuelo"]["Ix_kg_m2"],
        "I_z" : res["condicion_vuelo"]["Iz_kg_m2"],
        "I_xz": 0.0,

        "Y_v" : d["Yb_m_s2"] / u0,
        "Y_p" : d["Yp_m_s"],
        "Y_r" : d["Yr_m_s"],
        "Y_dr": d["Ydr_m_s2"],

        "L_v" : d["Lb_s2"] / u0,
        "L_p" : d["Lp_s1"],
        "L_r" : d["Lr_s1"],
        "L_da": d["Lda_s2"],
        "L_dr": d["Ldr_s2"],

        "N_v" : d["Nb_s2"] / u0,
        "N_p" : d["Np_s1"],
        "N_r" : d["Nr_s1"],
        "N_da": d["Nda_s2"],
        "N_dr": d["Ndr_s2"],
    }


def verificar_condicion(res: dict, avion: dict) -> list[str]:
    """Errores si la condición de vuelo del análisis no es la de la aeronave."""
    errores = []
    V0_res = res["condicion_vuelo"]["V0_m_s"]
    m_res  = res["condicion_vuelo"]["W_N"] / G_MPS2
    if abs(V0_res - avion["V0_ms"]) > TOL_V0_MS:
        errores.append(f"V0: análisis {V0_res:.4f} m/s vs aeronave {avion['V0_ms']} m/s")
    if abs(m_res - avion["masa_kg"]) > TOL_MASA_KG:
        errores.append(f"masa: análisis {m_res:.4f} kg vs aeronave {avion['masa_kg']} kg")
    if "Yda_m_s2" in res["derivadas_dimensionales"]["control_aleron"] and \
            res["derivadas_dimensionales"]["control_aleron"]["Yda_m_s2"] != 0.0:
        errores.append("Yda != 0: el modelo C no tiene Y_da")
    return errores


def reemplazar_valor(texto: str, clave: str, valor) -> str:
    """Sustituye el valor de "clave": ... conservando el resto de la línea."""
    if isinstance(valor, str):
        nuevo = json.dumps(valor)
    else:
        nuevo = f"{valor:.6g}"
        if not any(c in nuevo for c in ".en"):   # 0 -> 0.0, como el resto del JSON
            nuevo += ".0"
    patron = re.compile(r'("' + re.escape(clave) + r'"\s*:\s*)("[^"]*"|[^,\n}]+)')
    texto_nuevo, n = patron.subn(lambda m: m.group(1) + nuevo, texto)
    if n != 1:
        raise KeyError(f"la clave '{clave}' aparece {n} veces en el JSON de aeronave")
    return texto_nuevo


def convertir(ruta_res: str, ruta_avion: str, dry_run: bool = False) -> dict:
    with open(ruta_res, encoding="utf-8") as f:
        res = json.load(f)
    with open(ruta_avion, encoding="utf-8") as f:
        texto = f.read()
    avion = json.loads(texto)

    errores = verificar_condicion(res, avion)
    if errores:
        raise ValueError("condición de vuelo distinta:\n  " + "\n  ".join(errores))

    nuevos = mapear_derivadas(res)
    nuevos["lateral_source"] = LATERAL_SOURCE

    for clave, valor in nuevos.items():
        texto = reemplazar_valor(texto, clave, valor)

    json.loads(texto)   # el resultado debe seguir siendo JSON válido

    print(f"{'clave':15s} {'antes':>14s} {'ahora':>14s}")
    for clave, valor in nuevos.items():
        antes = avion[clave]
        if isinstance(valor, str):
            print(f"{clave:15s} {antes!s:>14.14s} {valor:>14.14s}")
        else:
            print(f"{clave:15s} {antes:>14.6g} {valor:>14.6g}")

    if not dry_run:
        with open(ruta_avion, "w", encoding="utf-8") as f:
            f.write(texto)
        print(f"\n[OK] Escrito: {ruta_avion}")
    else:
        print("\n[DRY-RUN] No se escribió nada")
    return nuevos


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("resultados", help="<aeronave>_latdir_resultados.json")
    ap.add_argument("aeronave", help="aircraft/<aeronave>.json a actualizar")
    ap.add_argument("--dry-run", action="store_true", help="solo mostrar cambios")
    args = ap.parse_args()
    try:
        convertir(args.resultados, args.aeronave, args.dry_run)
    except (ValueError, KeyError) as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
