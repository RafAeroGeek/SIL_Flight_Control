#!/usr/bin/env python3
"""
control_lat_design.py
=====================
Diseño clásico en cascada del control lateral-direccional (Paso 4, F2) sobre
el modelo lineal del simulador: yaw damper, alabeo (p, φ) y rumbo (ψ).

Uso (desde la raíz del repo):
    python3 dynamics/control_lat_design.py aircraft/ugly_stick.json \\
        [--salida dynamics/ugly_control_lat.json] \\
        [--datos docs/teoria/control/datos]

Lee las derivadas laterales en ejes cuerpo y los servos del JSON de aeronave
(lo mismo que integra el C), evalúa las ganancias elegidas contra los
criterios y escribe:
  - <salida>: ganancias, polos, respuestas al escalón y márgenes;
  - <datos>/*.dat (opcional): barridos y respuestas para las figuras de
    docs/teoria/control/control_lateral.tex (pgfplots).
Sale con código 1 si algún criterio no se cumple.

Leyes (convención de Nelson: L_δa > 0, N_δr < 0):
    yaw damper:  δr = K_r · r_wo,     r_wo = s τ_w / (1 + s τ_w) · r
    alabeo:      δa = K_φ (φ_c − φ) − K_p p
    rumbo:       φ_c = sat(K_ψ (ψ_c − ψ), ±φ_max)
Unidades: ángulos en rad (o grados: las ganancias son adimensionales, K_p en s).

Teoría y justificación: docs/teoria/control/control_lateral.tex
"""

import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.linalg import expm

G_MPS2 = 9.81            # la misma g de src/dynamics.c (DYN_GRAVITY_MPS2)

# ─────────────────────────────────────────────────────────────────────────────
# Ganancias elegidas (justificación en el documento, sección de diseño)
# ─────────────────────────────────────────────────────────────────────────────
GANANCIAS = {
    "K_r":        0.42,    # [rad/(rad/s)] yaw damper (δr por r lavada)
    "tau_wo_s":   1.0,     # [s] constante del washout
    "K_p":        0.10,    # [rad/(rad/s)] amortiguamiento de alabeo
    "K_phi":      0.80,    # [rad/rad] actitud de alabeo
    "K_psi":      1.00,    # [rad/rad] rumbo -> φ_c
    "phi_max_deg": 30.0,   # límite de φ_c (modo estabilizado y rumbo)
}

# ─────────────────────────────────────────────────────────────────────────────
# Criterios (decididos con el usuario el 2026-10-23)
# ─────────────────────────────────────────────────────────────────────────────
CRITERIOS = {
    "zeta_min":        0.40,   # todo polo complejo de lazo cerrado (ζ_DR ≥ 0.4)
    "phi_os_max_pct":  5.0,    # sobrepaso del escalón de φ
    "phi_tr_max_s":    0.8,    # tiempo de subida 10–90 % de φ
    "psi_os_max_pct":  5.0,    # sobrepaso del escalón de ψ
    "psi_ts_max_s":    10.0,   # establecimiento al 5 % de ψ
    "gm_min_db":       6.0,    # margen de ganancia (subida) por canal
    "retardo_min_s":   0.05,   # retardo puro tolerado en ambos canales
}

# Índices de estado del modelo de diseño
IV, IP, IR, IPHI, IPSI, IDA, IDR, IW = range(8)
N_X = 8
NOMBRES_X = ["v", "p", "r", "phi", "psi", "da", "dr", "x_wo"]


# ─────────────────────────────────────────────────────────────────────────────
# 1. Modelo
# ─────────────────────────────────────────────────────────────────────────────

def matrices_laterales(p: dict):
    """A (5x5) y B (5x2) iguales a build_lateral_matrices() (src/dynamics.c)."""
    Ix, Iz, Ixz = p["I_x"], p["I_z"], p["I_xz"]
    if Ix <= 0.0 or Iz <= 0.0 or 1.0 - Ixz**2 / (Ix * Iz) <= 0.0:
        Ix, Iz, Ixz = 1.0, 1.0, 0.0
    D = 1.0 - Ixz**2 / (Ix * Iz)
    kL, kN = Ixz / Ix, Ixz / Iz
    Ls = np.array([p["L_v"], p["L_p"], p["L_r"], p["L_da"], p["L_dr"]]) / D
    Ns = np.array([p["N_v"], p["N_p"], p["N_r"], p["N_da"], p["N_dr"]]) / D
    Lc, Nc = Ls + kL * Ns, Ns + kN * Ls

    u0, w0 = p["u0_ms"], p["w0_ms"]
    th = math.radians(p["theta0_deg"])
    A = np.array([
        [p["Y_v"], p["Y_p"] + w0, -(u0 - p["Y_r"]), G_MPS2 * math.cos(th), 0.0],
        [Lc[0],    Lc[1],         Lc[2],            0.0,                   0.0],
        [Nc[0],    Nc[1],         Nc[2],            0.0,                   0.0],
        [0.0,      1.0,           math.tan(th),     0.0,                   0.0],
        [0.0,      0.0,           1.0 / math.cos(th), 0.0,                 0.0],
    ])
    B = np.array([
        [0.0,   p["Y_dr"]],
        [Lc[3], Lc[4]],
        [Nc[3], Nc[4]],
        [0.0,   0.0],
        [0.0,   0.0],
    ])
    return A, B


def planta_diseno(p: dict, tau_wo: float):
    """
    Planta de diseño de 8 estados [v, p, r, φ, ψ, δa, δr, x_wo]:
    lateral 5x5 + servos de 1er orden (τ del JSON) + estado del washout
    (x_wo' = (r − x_wo)/τ_w, r_wo = r − x_wo). Entradas [δa_c, δr_c] en rad.
    """
    A5, B5 = matrices_laterales(p)
    ta = p.get("servo_aileron_tau_s", 0.05)
    tr = p.get("servo_rudder_tau_s", 0.07)
    A = np.zeros((N_X, N_X))
    A[:5, :5] = A5
    A[:5, IDA] = B5[:, 0]
    A[:5, IDR] = B5[:, 1]
    A[IDA, IDA] = -1.0 / ta
    A[IDR, IDR] = -1.0 / tr
    A[IW, IR] = 1.0 / tau_wo
    A[IW, IW] = -1.0 / tau_wo
    B = np.zeros((N_X, 2))
    B[IDA, 0] = 1.0 / ta
    B[IDR, 1] = 1.0 / tr
    return A, B


def ganancia_estado(k: dict, lazos: str) -> np.ndarray:
    """
    K (2xN) con u = K x + (referencias). lazos ⊂ {"y", "a", "h"}:
    y = yaw damper, a = alabeo (p, φ), h = rumbo (requiere a).
    """
    K = np.zeros((2, N_X))
    if "y" in lazos:
        K[1, IR] = k["K_r"]
        K[1, IW] = -k["K_r"]
    if "a" in lazos:
        K[0, IP] = -k["K_p"]
        K[0, IPHI] = -k["K_phi"]
    if "h" in lazos:
        K[0, IPSI] = -k["K_phi"] * k["K_psi"]
    return K


def lazo_cerrado(A, B, K):
    return A + B @ K


# ─────────────────────────────────────────────────────────────────────────────
# 2. Métricas
# ─────────────────────────────────────────────────────────────────────────────

def polos(Acl):
    """Polos con ζ y ωn, sin el de ψ en lazo abierto de rumbo (λ = 0)."""
    out = []
    for e in sorted(np.linalg.eigvals(Acl), key=lambda z: (abs(z), z.imag)):
        wn = abs(e)
        z = -e.real / wn if wn > 1e-9 else 1.0
        out.append({"re": float(e.real), "im": float(e.imag),
                    "wn": float(wn), "zeta": float(z)})
    return out


def zeta_min(Acl):
    c = [e for e in np.linalg.eigvals(Acl) if abs(e.imag) > 1e-6]
    return min(-e.real / abs(e) for e in c) if c else 1.0


def max_re(Acl, sin_cero=False):
    ev = np.linalg.eigvals(Acl)
    if sin_cero:
        ev = [e for e in ev if abs(e) > 1e-7]
    return float(max(e.real for e in ev))


def escalon(Acl, b, C, T, dt=0.005):
    """Respuesta al escalón unitario, discretización exacta (ZOH)."""
    n = Acl.shape[0]
    M = np.zeros((n + 1, n + 1))
    M[:n, :n] = Acl
    M[:n, n] = b
    E = expm(M * dt)
    Ad, bd = E[:n, :n], E[:n, n]
    x = np.zeros(n)
    t = np.arange(1, int(round(T / dt)) + 1) * dt
    Y = np.zeros((len(t), len(C)))
    for i in range(len(t)):
        x = Ad @ x + bd
        Y[i] = [c @ x for c in C]
    return t, Y


def metricas_escalon(t, y, ref=1.0):
    i10 = int(np.argmax(y >= 0.1 * ref))
    i90 = int(np.argmax(y >= 0.9 * ref))
    fuera = np.where(np.abs(y - ref) > 0.05 * abs(ref))[0]
    return {
        "tr_s": float(t[i90] - t[i10]),
        "sobrepaso_pct": float(max(0.0, (y.max() - ref) / ref * 100.0)),
        "ts5_s": float(t[fuera[-1]]) if len(fuera) else 0.0,
        "final": float(y[-1] / ref),
    }


def rango_ganancia(A, B, K, canal, kmax=50.0, n=5000):
    """[k_min, k_max] (multiplicador sobre la fila 'canal' de K) estable."""
    ks = np.linspace(kmax / n, kmax, n)
    ok = []
    for kv in ks:
        Kk = K.copy()
        Kk[canal] *= kv
        if max_re(A + B @ Kk) < 0.0:
            ok.append(kv)
    return (float(min(ok)), float(max(ok))) if ok else (float("nan"), float("nan"))


def max_re_con_retardo(A, B, K, T):
    """Retardo puro T en ambas entradas (Padé de orden 2)."""
    a1, a0 = 6.0 / T, 12.0 / T**2
    Ad = np.array([[0.0, 1.0], [-a0, -a1]])
    bd = np.array([0.0, 1.0])
    cd = np.array([0.0, -2.0 * a1])     # (s² − a1 s + a0)/(s² + a1 s + a0) = 1 + cd(sI−Ad)⁻¹bd
    n = A.shape[0]
    Ac = np.zeros((n + 4, n + 4))
    Ac[:n, :n] = A
    for c in range(2):
        i = n + 2 * c
        Ac[i:i + 2, i:i + 2] = Ad
        Ac[i:i + 2, :n] += np.outer(bd, K[c])
        Ac[:n, i:i + 2] += np.outer(B[:, c], cd)
        Ac[:n, :n] += np.outer(B[:, c], K[c])
    return max_re(Ac)


def retardo_max(A, B, K, Tmax=0.5, paso=0.005):
    T = paso
    while T <= Tmax:
        if max_re_con_retardo(A, B, K, T) >= 0.0:
            return float(T - paso)
        T += paso
    return float(Tmax)


# ─────────────────────────────────────────────────────────────────────────────
# 3. Evaluación del diseño
# ─────────────────────────────────────────────────────────────────────────────

def evaluar(p: dict, k: dict = GANANCIAS) -> dict:
    A, B = planta_diseno(p, k["tau_wo_s"])
    A5, _ = matrices_laterales(p)
    res = {"ganancias": dict(k)}

    res["polos_lazo_abierto"] = polos(A5)
    Kyd = ganancia_estado(k, "y")
    res["polos_yaw_damper"] = polos(lazo_cerrado(A, B, Kyd))
    res["zeta_dr_yaw_damper"] = zeta_min(lazo_cerrado(A, B, Kyd))

    # Modo estabilizado: yaw damper + alabeo, entrada φ_c
    Kst = ganancia_estado(k, "ya")
    Ast = lazo_cerrado(A, B, Kst)
    b_phi = B[:, 0] * k["K_phi"]
    t, Y = escalon(Ast, b_phi, [np.eye(N_X)[IPHI], np.eye(N_X)[IDA]], T=5.0)
    res["polos_estabilizado"] = polos(Ast)
    res["zeta_min_estabilizado"] = zeta_min(Ast)
    res["escalon_phi"] = metricas_escalon(t, Y[:, 0])
    res["escalon_phi"]["da_max_por_rad"] = float(np.abs(Y[:, 1]).max())

    # Rumbo: yaw damper + alabeo + rumbo, entrada ψ_c
    Kh = ganancia_estado(k, "yah")
    Ah = lazo_cerrado(A, B, Kh)
    b_psi = B[:, 0] * k["K_phi"] * k["K_psi"]
    t, Y = escalon(Ah, b_psi, [np.eye(N_X)[IPSI], np.eye(N_X)[IPHI],
                               np.eye(N_X)[IDR]], T=20.0)
    res["polos_rumbo"] = polos(Ah)
    res["zeta_min_rumbo"] = zeta_min(Ah)
    res["escalon_psi"] = metricas_escalon(t, Y[:, 0])
    res["escalon_psi"]["phi_max_por_rad"] = float(np.abs(Y[:, 1]).max())
    res["escalon_psi"]["dr_max_por_rad"] = float(np.abs(Y[:, 2]).max())

    # Robustez en el modo de rumbo (el más exigente)
    gm = {}
    for c, nombre in ((0, "aleron"), (1, "timon")):
        kmin, kmax = rango_ganancia(A, B, Kh, c)
        gm[nombre] = {"k_min": kmin, "k_max": kmax,
                      "gm_sube_db": 20 * math.log10(kmax),
                      "gm_baja_db": 20 * math.log10(kmin)}
    res["rango_ganancia_rumbo"] = gm
    res["retardo_max_s"] = retardo_max(A, B, Kh)
    return res


def verificar(res: dict, c: dict = CRITERIOS) -> list[str]:
    fallas = []
    for clave in ("zeta_dr_yaw_damper", "zeta_min_estabilizado", "zeta_min_rumbo"):
        if res[clave] < c["zeta_min"]:
            fallas.append(f"{clave} = {res[clave]:.3f} < {c['zeta_min']}")
    ph, ps = res["escalon_phi"], res["escalon_psi"]
    if ph["sobrepaso_pct"] > c["phi_os_max_pct"]:
        fallas.append(f"sobrepaso de φ {ph['sobrepaso_pct']:.1f} %")
    if ph["tr_s"] > c["phi_tr_max_s"]:
        fallas.append(f"subida de φ {ph['tr_s']:.2f} s")
    if ps["sobrepaso_pct"] > c["psi_os_max_pct"]:
        fallas.append(f"sobrepaso de ψ {ps['sobrepaso_pct']:.1f} %")
    if ps["ts5_s"] > c["psi_ts_max_s"]:
        fallas.append(f"establecimiento de ψ {ps['ts5_s']:.1f} s")
    for nombre, g in res["rango_ganancia_rumbo"].items():
        if g["gm_sube_db"] < c["gm_min_db"]:
            fallas.append(f"margen de ganancia {nombre} {g['gm_sube_db']:.1f} dB")
    if res["retardo_max_s"] < c["retardo_min_s"]:
        fallas.append(f"retardo tolerado {res['retardo_max_s'] * 1000:.0f} ms")
    # En modo estabilizado ψ queda libre: su λ = 0 es neutro, no inestable
    for clave in ("polos_estabilizado", "polos_rumbo"):
        if max(pp["re"] for pp in res[clave] if pp["wn"] > 1e-7) >= 0.0:
            fallas.append(f"{clave}: hay polos inestables")
    return fallas


# ─────────────────────────────────────────────────────────────────────────────
# 4. Barridos para el documento (.dat de pgfplots)
# ─────────────────────────────────────────────────────────────────────────────

def _escribir_dat(ruta, encabezado, filas):
    with open(ruta, "w", encoding="utf-8") as f:
        f.write(" ".join(encabezado) + "\n")
        for fila in filas:
            f.write(" ".join(f"{x:.6g}" for x in fila) + "\n")


def escribir_datos(p: dict, carpeta: str, k: dict = GANANCIAS):
    os.makedirs(carpeta, exist_ok=True)
    A, B = planta_diseno(p, k["tau_wo_s"])

    # Lugar de las raíces del yaw damper: polos complejos vs K_r
    filas = []
    for Kr in np.linspace(0.0, 1.2, 121):
        kk = dict(k, K_r=float(Kr))
        for e in np.linalg.eigvals(lazo_cerrado(A, B, ganancia_estado(kk, "y"))):
            if e.imag > 1e-6 and abs(e) < 15.0:
                filas.append((Kr, e.real, e.imag, -e.real / abs(e)))
    _escribir_dat(os.path.join(carpeta, "lugar_yaw_damper.dat"),
                  ["Kr", "re", "im", "zeta"], filas)

    # ζ_min del modo rumbo vs K_psi para varios K_r
    filas = []
    for Kpsi in np.linspace(0.2, 2.0, 91):
        fila = [Kpsi]
        for Kr in (0.27, 0.42, 0.60):
            kk = dict(k, K_r=Kr, K_psi=float(Kpsi))
            fila.append(zeta_min(lazo_cerrado(A, B, ganancia_estado(kk, "yah"))))
        filas.append(fila)
    _escribir_dat(os.path.join(carpeta, "zeta_rumbo.dat"),
                  ["Kpsi", "zmin_Kr027", "zmin_Kr042", "zmin_Kr060"], filas)

    # Respuestas al escalón (φ_c = 10°, ψ_c = 30°) del diseño elegido
    Ast = lazo_cerrado(A, B, ganancia_estado(k, "ya"))
    t, Y = escalon(Ast, B[:, 0] * k["K_phi"],
                   [np.eye(N_X)[IPHI], np.eye(N_X)[IDA], np.eye(N_X)[IR]], T=4.0)
    ref = 10.0
    _escribir_dat(os.path.join(carpeta, "escalon_phi.dat"),
                  ["t", "phi_deg", "da_deg", "r_dps"],
                  [(ti, ref * y[0], ref * y[1], ref * y[2]) for ti, y in zip(t[::4], Y[::4])])

    Ah = lazo_cerrado(A, B, ganancia_estado(k, "yah"))
    t, Y = escalon(Ah, B[:, 0] * k["K_phi"] * k["K_psi"],
                   [np.eye(N_X)[IPSI], np.eye(N_X)[IPHI], np.eye(N_X)[IDA],
                    np.eye(N_X)[IDR]], T=15.0)
    ref = 30.0
    _escribir_dat(os.path.join(carpeta, "escalon_psi.dat"),
                  ["t", "psi_deg", "phi_deg", "da_deg", "dr_deg"],
                  [(ti, ref * y[0], ref * y[1], ref * y[2], ref * y[3])
                   for ti, y in zip(t[::10], Y[::10])])


# ─────────────────────────────────────────────────────────────────────────────
# 5. CLI
# ─────────────────────────────────────────────────────────────────────────────

def _fmt_polos(ps):
    return ", ".join(f"{q['re']:+.3f}{q['im']:+.3f}j" for q in ps if q["im"] >= 0.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("aeronave", help="aircraft/<aeronave>.json")
    ap.add_argument("--salida", default=None,
                    help="JSON de resultados (default dynamics/<aeronave>_control_lat.json)")
    ap.add_argument("--datos", default=None,
                    help="carpeta para los .dat de las figuras (opcional)")
    args = ap.parse_args()

    with open(args.aeronave, encoding="utf-8") as f:
        p = json.load(f)
    nombre = os.path.splitext(os.path.basename(args.aeronave))[0].split("_")[0]
    salida = args.salida or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         f"{nombre}_control_lat.json")

    res = evaluar(p)
    fallas = verificar(res)
    res["criterios"] = dict(CRITERIOS)
    res["cumple"] = not fallas
    res["fallas"] = fallas
    with open(salida, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
        f.write("\n")
    if args.datos:
        escribir_datos(p, args.datos)

    k = res["ganancias"]
    print(f"Ganancias: K_r={k['K_r']} τ_w={k['tau_wo_s']} s  K_p={k['K_p']} s  "
          f"K_φ={k['K_phi']}  K_ψ={k['K_psi']}  φ_max={k['phi_max_deg']}°")
    print(f"Lazo abierto : {_fmt_polos(res['polos_lazo_abierto'])}")
    print(f"Yaw damper   : ζ_DR = {res['zeta_dr_yaw_damper']:.2f}")
    print(f"Estabilizado : ζ_min = {res['zeta_min_estabilizado']:.2f}  "
          f"φ: t_r = {res['escalon_phi']['tr_s']:.2f} s, "
          f"sobrepaso = {res['escalon_phi']['sobrepaso_pct']:.1f} %")
    print(f"Rumbo        : ζ_min = {res['zeta_min_rumbo']:.2f}  "
          f"ψ: t_s5% = {res['escalon_psi']['ts5_s']:.1f} s, "
          f"sobrepaso = {res['escalon_psi']['sobrepaso_pct']:.1f} %")
    for c, g in res["rango_ganancia_rumbo"].items():
        print(f"Rango estable {c:7s}: k ∈ [{g['k_min']:.2f}, {g['k_max']:.2f}] "
              f"({g['gm_sube_db']:+.1f} / {g['gm_baja_db']:+.1f} dB)")
    print(f"Retardo tolerado: {res['retardo_max_s'] * 1000:.0f} ms")
    print(f"\n[OK] Escrito: {salida}" if not fallas else "")
    if fallas:
        print("[FALLA] " + "\n[FALLA] ".join(fallas), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
