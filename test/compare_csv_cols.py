#!/usr/bin/env python3
"""Compara columnas de dos CSV del SIL como TEXTO (regresion bit a bit).

El formato de printf es el mismo en ambas corridas, asi que comparar las
cadenas detecta cualquier cambio sin tolerancias.

Uso (desde la raiz del repo):
    python3 test/compare_csv_cols.py data/baseline_4affdca.csv data/SIL_sim_servos2.csv \\
        --cols t_s,du_mps,dw_mps,dq_radps,dtheta_rad,y_elev_deg,pitot_ms,AoA_deg,gyro_y_radps,acc_x_mps2,gps_alt_m,laser_alt_m

Sale con 0 si las columnas son identicas y con 1 si hay diferencias (reporta
la primera fila distinta de cada columna).
"""
import argparse
import sys

import pandas as pd


def load(path):
    df = pd.read_csv(path, dtype=str, keep_default_na=False)
    df.columns = df.columns.str.strip()   # headers viejos traen "laser_alt_m "
    return df.apply(lambda s: s.str.strip())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("baseline")
    ap.add_argument("candidate")
    ap.add_argument("--cols", required=True, help="columnas separadas por coma")
    args = ap.parse_args()

    cols = [c.strip() for c in args.cols.split(",") if c.strip()]
    base = load(args.baseline)
    cand = load(args.candidate)

    ok = True
    missing = [c for c in cols if c not in base.columns or c not in cand.columns]
    if missing:
        print(f"Columnas ausentes: {', '.join(missing)}")
        ok = False

    if len(base) != len(cand):
        print(f"Numero de filas distinto: {len(base)} vs {len(cand)}")
        ok = False

    n = min(len(base), len(cand))
    for c in cols:
        if c in missing:
            continue
        diff = base[c].iloc[:n].values != cand[c].iloc[:n].values
        if diff.any():
            i = int(diff.argmax())
            print(f"{c}: {int(diff.sum())} filas distintas; primera en la fila {i + 2} "
                  f"(t_s={cand['t_s'].iloc[i] if 't_s' in cand else '?'}): "
                  f"'{base[c].iloc[i]}' != '{cand[c].iloc[i]}'")
            ok = False

    if ok:
        print(f"OK: {len(cols)} columnas identicas en {n} filas")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
