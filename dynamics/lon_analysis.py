#!/usr/bin/env python3
"""
lon_analysis.py
===============
Análisis longitudinal de una aeronave a partir de un archivo JSON de
parámetros de entrada.

Uso (desde la raíz del repo):
    python3 dynamics/lon_analysis.py dynamics/ugly_longitudinal.json [--plots]

Genera, junto al archivo de entrada:
    <aeronave>_lon_resultados.json

Con --plots muestra además la respuesta del periodo corto, la envolvente de
la respuesta al escalón y el lugar de las raíces (requiere matplotlib y
python-control).

Estados: x = [du, dw, dq, dθ]    Entrada: δe
Referencia teórica: Nelson, Flight Stability and Automatic Control, cap. 3–4.
"""

import argparse
import json
import math
import os
import sys
from datetime import datetime

import numpy as np

G_MPS2 = 9.81


# ─────────────────────────────────────────────────────────────────────────────
# 1. LECTURA DE DATOS
# ─────────────────────────────────────────────────────────────────────────────

def cargar_datos(ruta_json: str) -> dict:
    """Lee el archivo JSON de parámetros de entrada."""
    if not os.path.isfile(ruta_json):
        raise FileNotFoundError(f"No se encontró el archivo: {ruta_json}")
    with open(ruta_json, "r", encoding="utf-8") as f:
        return json.load(f)


# ─────────────────────────────────────────────────────────────────────────────
# 2. CONDICIÓN DE VUELO Y TRIM
# ─────────────────────────────────────────────────────────────────────────────

def calcular_condicion_vuelo(datos: dict) -> dict:
    """
    Densidad, presión dinámica, C_L/C_D de trim, Cmα, efectividad del
    elevador, trim (δe, α) y velocidades u0/w0 en ejes cuerpo.
    """
    Alt      = datos["Alt[m]"]
    V0       = datos["V_0[m/s]"]
    gamma0   = datos["gamma_0[deg]"]
    m        = datos["masa[Kg]"]
    x_cg     = datos["x_cg[m]"]
    x_ca     = datos["x_ca[m]"]           # punto neutro
    b        = datos["b[m]"]
    S        = datos["S[m2]"]
    cam      = datos["cam[m]"]
    l_ht     = datos["l_ht[m]"]
    S_ht     = datos["S_ht[m2]"]
    S_e      = datos["S_e[m2]"]
    alpha_0  = datos["alpha_0[deg]"]      # ángulo de cero sustentación
    CL_alpha = datos["CL_alpha"]          # avión completo
    CD_0     = datos["CD_0"]
    k        = datos["k"]
    CM_0     = datos["CM_0"]
    CL_a_ht  = datos["CL_alpha_ht"]

    rho = 1.225 * math.exp(-Alt / 8400)
    W   = m * G_MPS2
    Q   = 0.5 * rho * V0**2

    AR   = b**2 / S
    V_ht = S_ht * l_ht / (S * cam)

    CL_trim  = W / (Q * S)
    CD_trim  = CD_0 + k * CL_trim**2
    CD_alpha = 2 * k * CL_trim * CL_alpha

    CM_alpha = -CL_alpha * (x_ca - x_cg) / cam

    # Efectividad del elevador τ = √(S_e/S_ht)
    CM_de = -CL_a_ht * V_ht * (S_e / S_ht)**0.5
    CL_de =  CL_a_ht * S_ht / S * (S_e / S_ht)**0.5

    d_e_trim   = -(CM_0 * CL_alpha + CM_alpha * CL_trim) / (CM_de * CL_alpha - CM_alpha * CL_de)
    alpha_trim = (CL_trim - CL_de * d_e_trim) / CL_alpha + alpha_0 / 57.3

    u0     = V0 * math.cos(alpha_trim)
    w0     = V0 * math.sin(alpha_trim)
    theta0 = alpha_trim + gamma0 / 57.3

    return {
        "rho_kg_m3"       : rho,
        "V0_m_s"          : V0,
        "u0_m_s"          : u0,
        "w0_m_s"          : w0,
        "Q_Pa"            : Q,
        "W_N"             : W,
        "AR"              : AR,
        "V_ht"            : V_ht,
        "CL_trim"         : CL_trim,
        "CD_trim"         : CD_trim,
        "CD_alpha"        : CD_alpha,
        "CM_alpha"        : CM_alpha,
        "CM_de"           : CM_de,
        "CL_de"           : CL_de,
        "delta_e_trim_deg": d_e_trim * 57.3,
        "alpha_trim_deg"  : alpha_trim * 57.3,
        "gamma0_deg"      : gamma0,
        "theta0_deg"      : theta0 * 57.3,
        "Iy_kg_m2"        : datos["Iy"],
        "_alpha_trim_rad" : alpha_trim,
        "_theta0_rad"     : theta0,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 3. COEFICIENTES DINÁMICOS
# ─────────────────────────────────────────────────────────────────────────────

def calcular_coeficientes(datos: dict, aux: dict) -> dict:
    """Cmq, Cmα̇, CZα̇ y dε/dα (Nelson cap. 3)."""
    S       = datos["S[m2]"]
    cam     = datos["cam[m]"]
    l_ht    = datos["l_ht[m]"]
    S_ht    = datos["S_ht[m2]"]
    CL_a_ht = datos["CL_alpha_ht"]
    CL_a_w  = datos["CL_alpha_w"]
    AR      = aux["AR"]
    V_ht    = aux["V_ht"]

    eta_ht = 1.0
    epsilon_alpha = 2 * CL_a_w / (math.pi * AR)

    CM_q    = -2 * eta_ht * (S_ht / S) * (l_ht / cam)**2 * CL_a_ht
    CM_adot = -2 * eta_ht * V_ht * CL_a_ht * epsilon_alpha * (l_ht / cam)
    CZ_adot = -2 * eta_ht * V_ht * CL_a_ht * epsilon_alpha

    return {
        "eta_ht"       : eta_ht,
        "epsilon_alpha": epsilon_alpha,
        "CM_q"         : CM_q,
        "CM_adot"      : CM_adot,
        "CZ_adot"      : CZ_adot,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 4. DERIVADAS DIMENSIONALES
# ─────────────────────────────────────────────────────────────────────────────

def calcular_derivadas_dimensionales(datos: dict, aux: dict, coef: dict) -> dict:
    """Derivadas dimensionales (SI) y las "hat" corregidas por Z_ẇ."""
    m    = datos["masa[Kg]"]
    S    = datos["S[m2]"]
    cam  = datos["cam[m]"]
    Iy   = datos["Iy"]
    CL_u = datos["CL_u"]
    CD_u = datos["CD_u"]
    CM_u = datos["CM_u"]
    CL_alpha = datos["CL_alpha"]

    V0 = aux["V0_m_s"]
    u0 = aux["u0_m_s"]
    Q  = aux["Q_Pa"]
    CL_trim  = aux["CL_trim"]
    CD_trim  = aux["CD_trim"]
    CD_alpha = aux["CD_alpha"]
    CM_alpha = aux["CM_alpha"]

    X_u     = -(CD_u + 2 * CD_trim) * Q * S / (m * u0)
    X_w     = -(CD_alpha - CL_trim) * Q * S / (m * u0)
    Z_u     = -(CL_u + 2 * CL_trim) * Q * S / (m * u0)
    M_u     = Q * S * cam / (u0 * Iy) * CM_u
    Z_w     = -(CL_alpha + CD_trim) * Q * S / (u0 * m)
    Z_w_dot = Q * S * cam * coef["CZ_adot"] / (2 * m * u0**2)
    Z_alpha = Z_w * u0
    M_w     = Q * S * cam * CM_alpha / (Iy * u0)
    M_q     = Q * S * cam**2 * coef["CM_q"] / (Iy * V0 * 2)
    M_alpha = Q * S * cam * CM_alpha / Iy
    # Nelson: M_ẇ = Cmα̇ (c̄/2u0) Q S c̄ / (u0 Iy), mismo /2 que Z_ẇ y M_q
    M_w_dot = Q * S * cam**2 * coef["CM_adot"] / (2 * Iy * u0**2)
    M_alpha_dot = u0 * M_w_dot

    M_de = Q * S * cam * aux["CM_de"] / Iy
    Z_de = -aux["CL_de"] * Q * S / m

    # ── "Hats" (corrección por Z_ẇ) ─────────────────────────────────────────
    den = (1.0 - Z_w_dot) if abs(1.0 - Z_w_dot) > 1e-9 else 1e-9

    th0 = aux["_theta0_rad"]

    return {
        "X_u": X_u, "X_w": X_w,
        "X_theta": -G_MPS2 * math.cos(th0),
        "Z_u": Z_u, "Z_w": Z_w, "Z_w_dot": Z_w_dot, "Z_alpha": Z_alpha,
        "Z_theta": -G_MPS2 * math.sin(th0),
        "M_u": M_u, "M_w": M_w, "M_w_dot": M_w_dot, "M_q": M_q,
        "M_alpha": M_alpha, "M_alpha_dot": M_alpha_dot,
        "Z_de": Z_de, "M_de": M_de,

        "Z_u_hat" : Z_u / den,
        "Z_w_hat" : Z_w / den,
        "M_u_hat" : (M_u + M_w_dot * Z_u) / den,
        "M_w_hat" : (M_w + M_w_dot * Z_w) / den,
        "M_q_hat" : (M_q + M_alpha_dot) / den,
        "Z_de_hat": Z_de / den,
        "M_de_hat": (M_de + M_w_dot * Z_de) / den,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 5. MATRICES DE ESPACIO DE ESTADOS
# ─────────────────────────────────────────────────────────────────────────────

def calcular_matrices_estado(aux: dict, dim: dict) -> dict:
    """
    A (4×4) y B (4×1), misma forma que build_state_space_matrices()
    (src/dynamics.c):

      du̇ = X_u·du + X_w·dw                 + X_θ·dθ
      dẇ = Ẑ_u·du + Ẑ_w·dw + u0·dq        + Z_θ·dθ + Ẑ_δe·δe
      dq̇ = M̂_u·du + M̂_w·dw + M̂_q·dq                + M̂_δe·δe
      dθ̇ =                     dq
    """
    u0 = aux["u0_m_s"]
    A = [
        [dim["X_u"],     dim["X_w"],     0.0,            dim["X_theta"]],
        [dim["Z_u_hat"], dim["Z_w_hat"], u0,             dim["Z_theta"]],
        [dim["M_u_hat"], dim["M_w_hat"], dim["M_q_hat"], 0.0           ],
        [0.0,            0.0,            1.0,            0.0           ],
    ]
    B = [
        [0.0            ],
        [dim["Z_de_hat"]],
        [dim["M_de_hat"]],
        [0.0            ],
    ]
    return {
        "estados" : ["du_m_s", "dw_m_s", "dq_rad_s", "dtheta_rad"],
        "entradas": ["delta_e_rad"],
        "A"       : A,
        "B"       : B,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 6. MODOS LONGITUDINALES
# ─────────────────────────────────────────────────────────────────────────────

def _metricas(lam: complex) -> dict:
    wn   = abs(lam)
    zeta = -lam.real / wn if wn > 1e-12 else float("nan")
    T    = 2 * math.pi / abs(lam.imag) if abs(lam.imag) > 1e-9 else float("inf")
    return {"lambda_real_s1": lam.real, "lambda_imag_s1": abs(lam.imag),
            "wn_rad_s": wn, "zeta": zeta, "T_s": T, "estable": lam.real < 0}


def calcular_modos(aux: dict, dim: dict, ss: dict) -> dict:
    """
    Aproximaciones de periodo corto y fugoide (Nelson) y modos exactos
    a partir de los eigenvalores de A.
    """
    u0 = aux["u0_m_s"]
    V0 = aux["V0_m_s"]

    omega_sp = (dim["M_q"] * dim["Z_w"] - dim["M_alpha"])**0.5
    zeta_sp  = -(dim["M_q"] + dim["Z_w"] + dim["M_alpha_dot"]) / (2 * omega_sp)
    n_a      = -dim["Z_alpha"] / G_MPS2

    zeta_ph  = (1 / 2**0.5) * (aux["CD_trim"] / aux["CL_trim"])
    omega_ph = G_MPS2 * 2**0.5 / V0

    aprox = {
        "periodo_corto": {
            "wn_rad_s"      : omega_sp,
            "zeta"          : zeta_sp,
            "T_s"           : 2 * math.pi / omega_sp,
            "n_alpha_g_rad" : n_a,
            "T_theta2_s"    : -u0 / dim["Z_alpha"],
            "CAP"           : omega_sp**2 / n_a,
        },
        "fugoide": {
            "wn_rad_s": omega_ph,
            "zeta"    : zeta_ph,
            "T_s"     : 2 * math.pi / omega_ph,
        },
    }

    # Exactos: el par complejo de mayor ω_n es el periodo corto
    ev = np.linalg.eigvals(np.array(ss["A"]))
    pares = sorted((complex(l) for l in ev if l.imag > 1e-7), key=abs)
    exactos = {}
    if len(pares) == 2:
        exactos["fugoide"]       = _metricas(pares[0])
        exactos["periodo_corto"] = _metricas(pares[1])
    exactos["eigenvalores"] = [[l.real, l.imag] for l in
                               sorted((complex(l) for l in ev),
                                      key=lambda z: (z.real, z.imag))]

    return {"aproximados": aprox, "exactos": exactos}


# ─────────────────────────────────────────────────────────────────────────────
# 7. ENSAMBLADO, RESUMEN Y ESCRITURA
# ─────────────────────────────────────────────────────────────────────────────

def _sin_privadas(d: dict) -> dict:
    return {k: v for k, v in d.items() if not k.startswith("_")}


def ensamblar_resultados(datos, aux, coef, dim, modos, ss) -> dict:
    return {
        "aeronave"   : datos.get("name", "aeronave"),
        "descripcion": "Análisis longitudinal (lon_analysis.py)",
        "condicion_vuelo"            : _sin_privadas(aux),
        "coeficientes_adimensionales": coef,
        "derivadas_dimensionales"    : dim,
        "modos_longitudinales"       : modos,
        "espacio_de_estados"         : ss,
    }


def imprimir_resumen(res: dict) -> None:
    sep = "─" * 60
    print(f"\n{'═'*60}")
    print(f"  Análisis longitudinal — {res['aeronave'].upper()}")
    # La fecha solo va a consola: en el JSON haría que cambie en cada corrida
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'═'*60}")

    cv = res["condicion_vuelo"]
    print(f"\n{sep}\n  CONDICIÓN DE VUELO Y TRIM\n{sep}")
    print(f"  ρ          = {cv['rho_kg_m3']:.4f}  kg/m³")
    print(f"  V₀ (TAS)   = {cv['V0_m_s']:.2f}  m/s   (u0 = {cv['u0_m_s']:.3f}, w0 = {cv['w0_m_s']:.3f})")
    print(f"  CL_trim    = {cv['CL_trim']:.4f}   CD_trim = {cv['CD_trim']:.4f}")
    print(f"  α_trim     = {cv['alpha_trim_deg']:.3f}  deg")
    print(f"  δe_trim    = {cv['delta_e_trim_deg']:.3f}  deg")
    print(f"  θ0         = {cv['theta0_deg']:.3f}  deg")
    print(f"  Iy         = {cv['Iy_kg_m2']:.3f}  kg·m²")

    print(f"\n{sep}\n  DERIVADAS DIMENSIONALES\n{sep}")
    for k, v in res["derivadas_dimensionales"].items():
        print(f"  {k:12s} = {v:+.4f}")

    ml = res["modos_longitudinales"]
    print(f"\n{sep}\n  MODOS LONGITUDINALES\n{sep}")
    for nombre, clave in (("Periodo corto", "periodo_corto"), ("Fugoide", "fugoide")):
        a = ml["aproximados"][clave]
        print(f"  {nombre}:")
        print(f"    aprox.  ω_n = {a['wn_rad_s']:.4f} rad/s  ζ = {a['zeta']:.4f}  T = {a['T_s']:.2f} s")
        e = ml["exactos"].get(clave)
        if e:
            print(f"    exacto  ω_n = {e['wn_rad_s']:.4f} rad/s  ζ = {e['zeta']:.4f}  T = {e['T_s']:.2f} s"
                  f"  (λ = {e['lambda_real_s1']:+.4f} ± {e['lambda_imag_s1']:.4f}i)")
    sp = ml["aproximados"]["periodo_corto"]
    print(f"  n_α = {sp['n_alpha_g_rad']:.3f} g/rad   T_θ2 = {sp['T_theta2_s']:.4f} s   CAP = {sp['CAP']:.4f}")

    ss = res["espacio_de_estados"]
    print(f"\n{sep}\n  A (filas/columnas: du, dw, dq, dθ)\n{sep}")
    for fila in ss["A"]:
        print("  " + "  ".join(f"{x:+11.5f}" for x in fila))
    print(f"\n  B (δe)")
    for fila in ss["B"]:
        print(f"  {fila[0]:+11.5f}")
    print(f"\n{'═'*60}\n")


def guardar_resultados(res: dict, directorio: str = ".") -> str:
    """Guarda el JSON con el nombre <aeronave>_lon_resultados.json"""
    aeronave = res["aeronave"].lower().replace(" ", "_")
    ruta = os.path.join(directorio, f"{aeronave}_lon_resultados.json")
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
        f.write("\n")
    return ruta


# ─────────────────────────────────────────────────────────────────────────────
# 8. GRÁFICAS (opcionales)
# ─────────────────────────────────────────────────────────────────────────────

def graficar(res: dict) -> None:
    """Respuesta del periodo corto, envolvente al escalón y lugar de raíces."""
    import matplotlib.pyplot as plt
    import control

    sp = res["modos_longitudinales"]["aproximados"]["periodo_corto"]
    ph = res["modos_longitudinales"]["aproximados"]["fugoide"]
    omega_sp, zeta_sp, n_a = sp["wn_rad_s"], sp["zeta"], sp["n_alpha_g_rad"]
    omega_ph, zeta_ph = ph["wn_rad_s"], ph["zeta"]
    M_de = res["derivadas_dimensionales"]["M_de"]
    periodo_sp = sp["T_s"]

    # ── Respuesta libre en α ─────────────────────────────────────────────────
    Tf = round(periodo_sp * 4, 1)
    ts = round(periodo_sp / 25, 3)
    N  = int(Tf / ts)
    Delta_alpha_0 = 10
    T = [n * ts for n in range(N + 1)]
    Alpha = [Delta_alpha_0 * np.exp(-(zeta_sp * omega_sp) * t)
             * ((zeta_sp / (1 - zeta_sp**0.5)**0.5) * np.sin((omega_sp * (1 - zeta_sp**0.5)**0.5) * t)
                + np.cos((omega_sp * (1 - zeta_sp**0.5)**0.5) * t))
             for t in T]

    plt.close("all")
    plt.figure(num=1, figsize=(15, 8))
    ax1 = plt.subplot(1, 1, 1)
    ax1.plot(T, Alpha, ".-", linewidth=1.0, label=r"$\Delta \alpha (t)$")
    ax1.set_xlabel("Tiempo (s)", fontsize=12)
    ax1.set_ylabel(r"$\Delta \alpha (t)$", fontsize=12)
    ax1.legend()
    ax1.minorticks_on()
    ax1.grid(True, which="major", linestyle="-", linewidth="0.5", color="gray")
    ax1.grid(True, which="minor", linestyle=":", linewidth="0.5", color="slategray")
    plt.show()

    # ── Escalón normalizado con envolvente ───────────────────────────────────
    sys_sp = control.TransferFunction([M_de * n_a], [1, 2 * zeta_sp * omega_sp, omega_sp**2])
    sys_ph = control.TransferFunction([omega_ph**2], [1, 2 * zeta_ph * omega_ph, omega_ph**2])
    T, yout = control.step_response(sys_sp)
    SS_response = M_de * n_a / (omega_sp**2)

    t  = np.linspace(0, T[-1], 1000)
    tp = np.pi / (omega_sp * (1 - zeta_sp**2)**0.5)

    y1 = np.piecewise(t, [t < tp/3, (t >= tp/3) & (t < tp), t >= tp],
                      [-0.25, lambda t: -0.25 + (0.5 / (tp/3*2)) * (t - tp/3), 0.25])
    y2 = np.piecewise(t, [t < tp/3, (t >= tp/3) & (t < tp), t >= tp],
                      [0, lambda t: 0 + (0.85 / (tp/3*2)) * (t - tp/3), 0.85])
    y3 = np.piecewise(t, [t < tp/3, t >= tp/3], [lambda t: 0.5 + (0.7 / (tp/3)) * t, 1.2])
    y4 = np.piecewise(t, [t < tp/3, t >= tp/3], [lambda t: 1 + (0.7 / (tp/3)) * t, 1.7])

    plt.figure(figsize=(12, 7))
    plt.plot(t, y1, "k-", linewidth=2, label="Línea 1 (y=-0.25 a 0.25)")
    plt.plot(t, y2, "k-", linewidth=2, label="Línea 2 (y=0 a 0.75)")
    plt.plot(t, y3, "k-", linewidth=2, label="Línea 3 (y=0.5 a 1.2)")
    plt.plot(t, y4, "k-", linewidth=2, label="Línea 4 (y=1 a 1.7)")
    plt.fill_between(t, -10, y1, where=(-10 < y1), color="red", alpha=0.3)
    plt.fill_between(t, y1, y2, where=(y1 < y2), color="yellow", alpha=0.3)
    plt.fill_between(t, y2, y3, where=(y2 < y3), color="green", alpha=0.3)
    plt.fill_between(t, y3, y4, where=(y3 < y4), color="yellow", alpha=0.3)
    plt.fill_between(t, y4, 10, where=(y4 < 10), color="red", alpha=0.3)
    plt.plot(T, yout / SS_response)
    plt.axvline(x=3, color="gray", linestyle="--", alpha=0.7)
    plt.axvline(x=9, color="gray", linestyle="--", alpha=0.7)
    plt.xlabel("Tiempo [s]", fontsize=12)
    plt.ylabel("Amplitud", fontsize=12)
    plt.title("Envolvente", fontsize=14)
    plt.legend(loc="upper left")
    plt.xlim(0, T[-1])
    plt.ylim(-0.5, 2.0)
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.show()

    # ── Lugar de las raíces ──────────────────────────────────────────────────
    plt.figure()
    control.root_locus(sys_sp * sys_ph, kvect=np.logspace(0, 1, 1), grid=True, plot=True)
    plt.axvspan(0, 1, facecolor="red", alpha=0.3, label="Región inestable (Re(s) > 0)")
    plt.title("Lugar de las Raíces (sys_sp * sys_ph)")
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# 9. MAIN
# ─────────────────────────────────────────────────────────────────────────────

def analizar(ruta_entrada: str) -> dict:
    datos = cargar_datos(ruta_entrada)
    aux   = calcular_condicion_vuelo(datos)
    coef  = calcular_coeficientes(datos, aux)
    dim   = calcular_derivadas_dimensionales(datos, aux, coef)
    ss    = calcular_matrices_estado(aux, dim)
    modos = calcular_modos(aux, dim, ss)
    return ensamblar_resultados(datos, aux, coef, dim, modos, ss)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("entrada", help="JSON de parámetros de la aeronave")
    ap.add_argument("--plots", action="store_true", help="mostrar gráficas")
    args = ap.parse_args()

    try:
        res = analizar(args.entrada)
    except (FileNotFoundError, KeyError) as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)

    imprimir_resumen(res)
    ruta = guardar_resultados(res, os.path.dirname(os.path.abspath(args.entrada)))
    print(f"[OK] Resultados guardados en: {ruta}\n")

    if args.plots:
        graficar(res)


if __name__ == "__main__":
    main()
