"""
latdir_analysis.py
==================
Análisis lateral-direccional de una aeronave a partir de un archivo JSON
de parámetros de entrada.

Uso:
    python latdir_analysis.py <archivo_entrada.json>

Ejemplo:
    python latdir_analysis.py c525_latdir_datos.json

El script genera automáticamente:
    <aeronave>_latdir_resultados.json

Referencia teórica:
    Cook, M.V. — Flight Dynamics Principles (2nd Ed.)
    Notas de clase: Dinámica Lateral-Direccional, Cessna Citation C525
"""

import json
import math
import sys
import os
from datetime import datetime


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
# 2. MAGNITUDES AUXILIARES DE LA CONDICIÓN DE VUELO
# ─────────────────────────────────────────────────────────────────────────────

def calcular_condicion_vuelo(datos: dict) -> dict:
    """
    Calcula densidad, velocidad, presión dinámica, peso, C_L_trim,
    parámetros geométricos adimensionales e inercias estimadas.
    """
    v = datos["vuelo"]
    a = datos["ala"]
    cv = datos["cola_vertical"]
    ini = datos["inercia"]

    m   = v["m_kg"]
    H   = v["H_km"]
    M   = v["M_mach"]
    g   = v["g_mps2"]
    b   = a["b_m"]
    S   = a["S_m2"]
    c_r = a["c_r_m"]
    c_t = a["c_t_m"]
    S_v = cv["S_v_m2"]
    S_r = cv["S_r_m2"]
    l_v = cv["l_v_m"]
    l_f = ini["l_f_m"]

    # Densidad (ISA polinomial, H en km)
    rho = 1.225 * (20 - H) / (20 + H)

    # Velocidad verdadera (TAS)
    V0 = 340.3 * M * (1 - 0.011825 * H)

    # Presión dinámica
    Q_inf = 0.5 * rho * V0**2

    # Peso
    W = m * g

    # Coeficiente de sustentación en trimado
    CL_trim = W / (Q_inf * S)

    # Alargamiento
    AR = b**2 / S

    # Relación de conicidad
    lam = c_t / c_r

    # Volumen de cola vertical
    V_v = (l_v * S_v) / (b * S)

    # Parámetro geométrico Z_lambda
    Z_lam = (1 / 3) * ((1 / lam + 2) / (1 / lam + 1))

    # Efectividad del timón de dirección
    tau = math.sqrt(S_r / S_v)

    # Ángulos en radianes
    Gamma_rad     = math.radians(a["Gamma_deg"])
    Lambda_rad    = math.radians(a["Lambda_deg"])
    Lambda14_rad  = math.radians(a["Lambda14_deg"])

    # ── Momentos de inercia ─────────────────────────────────────────────────
    # Alabeo: I_x = m * (r_x * b/2)^2  →  simplificado con r_x = 0.25
    r_x = ini["radio_giro_x_adim"]
    Ix  = (b**2 * m * r_x**2) / 4

    # Guiñada (Roskam): I_z = m * (e * R_z / 2)^2,  e = L_ref = (b + l_f)/2
    #   →  I_z = m * R_z^2 / 4,  R_z = r_z * L_ref
    r_z   = ini["radio_giro_z_adim"]
    L_ref = (b + l_f) / 2
    R_z   = r_z * L_ref
    Iz    = (m * R_z**2) / 4

    return {
        "rho_kg_m3"   : rho,
        "V0_m_s"      : V0,
        "Q_inf_Pa"    : Q_inf,
        "W_N"         : W,
        "CL_trim"     : CL_trim,
        "AR"          : AR,
        "lambda_taper": lam,
        "V_v"         : V_v,
        "Z_lambda"    : Z_lam,
        "tau"         : tau,
        "Gamma_rad"   : Gamma_rad,
        "Lambda_rad"  : Lambda_rad,
        "Lambda14_rad": Lambda14_rad,
        "Ix_kg_m2"    : Ix,
        "Iz_kg_m2"    : Iz,
        "L_ref_m"     : L_ref,
        "R_z_m"       : R_z,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 3. COEFICIENTES ADIMENSIONALES
# ─────────────────────────────────────────────────────────────────────────────

def calcular_coeficientes(datos: dict, aux: dict) -> dict:
    """
    Calcula los 15 coeficientes laterales-direccionales adimensionales
    de estabilidad y control.
    """
    a  = datos["ala"]
    cv = datos["cola_vertical"]
    ae = datos["aerodinamica"]

    S       = a["S_m2"]
    b       = a["b_m"]
    c_r     = a["c_r_m"]
    S_a     = a["S_a_m2"]
    b1      = a["b1_m"]
    b2      = a["b2_m"]
    S_v     = cv["S_v_m2"]
    l_v     = cv["l_v_m"]
    z_v     = cv["z_v_m"]
    eta_v   = cv["eta_v"]
    K_v     = cv["K_v"]
    ds_db   = cv["dsigma_dbeta"]
    CLa_w   = ae["CLa_w"]
    CD0_w   = ae["CD0_w"]
    CLa_v   = ae["CLa_v"]

    CL_trim = aux["CL_trim"]
    AR      = aux["AR"]
    V_v     = aux["V_v"]
    Z_lam   = aux["Z_lambda"]
    tau     = aux["tau"]
    lam     = aux["lambda_taper"]
    Gamma   = aux["Gamma_rad"]
    Lam     = aux["Lambda_rad"]
    Lam14   = aux["Lambda14_rad"]

    # ── Derivadas en β ───────────────────────────────────────────────────────
    # Contribución de la cola vertical, referida al ala (Nelson cap. 5):
    #   CYβ_v = -η (S_v/S) CLα_v (1 + dσ/dβ)
    CYb_tail = -eta_v * (S_v / S) * CLa_v * (1 + ds_db)

    CYb = CYb_tail   # sin contribución de fuselaje

    Cnb = (0.2 * eta_v * V_v * CLa_v
           + eta_v * V_v * CLa_v * (1 + ds_db))

    # Nelson: ΔClβ_v = CYβ_v · z_v/b  (CYβ_v ya incluye η S_v/S)
    Clb = (-0.5 * CLa_w * Gamma * Z_lam * math.cos(Lam14)**2
           + CYb_tail * z_v / b)

    # ── Derivadas en p ───────────────────────────────────────────────────────
    CYp = CL_trim * (AR + math.cos(Lam)) / (AR + 4 * math.cos(Lam)) * math.tan(Lam)

    Cnp = -CL_trim / 8

    Clp = -(CLa_w - CD0_w) * (c_r * b * (1 + 3 * lam)) / (24 * S)

    # ── Derivadas en r ───────────────────────────────────────────────────────
    # Nelson: CYr = -2 CYβ_v (l_v/b)  (positiva)
    CYr = -2 * CYb_tail * (l_v / b)

    Cnr = -2 * eta_v * V_v * (l_v / b) * CLa_v

    Clr = (CL_trim / 4
           - 2 * (l_v / b) * (z_v / b) * CYb_tail)

    # ── Derivadas de control — alerón ────────────────────────────────────────
    CYda = 0.0

    Clda = CLa_w * math.sqrt(S_a / S) * (b2**2 - b1**2) / b**2

    Cnda = 2 * K_v * CL_trim * Clda

    # ── Derivadas de control — timón de dirección ────────────────────────────
    CYdr = (S_v / S) * tau * CLa_v

    Cndr = -V_v * eta_v * tau * CLa_v

    Cldr = (z_v / b) * tau * CLa_v

    return {
        # estabilidad en β
        "CYb": CYb, "Cnb": Cnb, "Clb": Clb,
        # estabilidad en p
        "CYp": CYp, "Cnp": Cnp, "Clp": Clp,
        # estabilidad en r
        "CYr": CYr, "Cnr": Cnr, "Clr": Clr,
        # control — alerón
        "CYda": CYda, "Clda": Clda, "Cnda": Cnda,
        # control — timón
        "CYdr": CYdr, "Cndr": Cndr, "Cldr": Cldr,
        # auxiliar para derivadas dimensionales
        "_CYb_tail": CYb_tail,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 4. DERIVADAS DIMENSIONALES
# ─────────────────────────────────────────────────────────────────────────────

def calcular_derivadas_dimensionales(datos: dict,
                                     aux: dict,
                                     coef: dict) -> dict:
    """
    Convierte los coeficientes adimensionales a derivadas dimensionales
    (unidades SI).

    Fórmulas de conversión
    ─────────────────────────────────────────────────────────────────────
    Derivadas estáticas (en β o δ):
        Y_() = Q∞ S C_Y / m                  [m/s²]
        N_() = Q∞ S b C_n / Iz               [s⁻²]
        L_() = Q∞ S b C_l / Ix               [s⁻²]

    Derivadas dinámicas (en p o r):
        Y_() = Q∞ S b C_Y / (2 m u0)        [m/s]
        N_() = Q∞ S b² C_n / (2 Iz u0)      [s⁻¹]
        L_() = Q∞ S b² C_l / (2 Ix u0)      [s⁻¹]
    """
    a = datos["ala"]
    S   = a["S_m2"]
    b   = a["b_m"]
    m   = datos["vuelo"]["m_kg"]
    u0  = aux["V0_m_s"]
    Q   = aux["Q_inf_Pa"]
    Ix  = aux["Ix_kg_m2"]
    Iz  = aux["Iz_kg_m2"]

    QS   = Q * S
    QSb  = Q * S * b
    QSb2 = Q * S * b**2

    def Yder_est(C):  return QS   * C / m
    def Nder_est(C):  return QSb  * C / Iz
    def Lder_est(C):  return QSb  * C / Ix

    def Yder_din(C):  return QSb  * C / (2 * m  * u0)
    def Nder_din(C):  return QSb2 * C / (2 * Iz * u0)
    def Lder_din(C):  return QSb2 * C / (2 * Ix * u0)

    return {
        # ── en β  (estáticas) ──────────────────────────────────────────
        "Yb_m_s2"  : Yder_est(coef["CYb"]),
        "Nb_s2"    : Nder_est(coef["Cnb"]),
        "Lb_s2"    : Lder_est(coef["Clb"]),

        # ── en p  (dinámicas) ──────────────────────────────────────────
        "Yp_m_s"   : Yder_din(coef["CYp"]),
        "Np_s1"    : Nder_din(coef["Cnp"]),
        "Lp_s1"    : Lder_din(coef["Clp"]),

        # ── en r  (dinámicas) ──────────────────────────────────────────
        "Yr_m_s"   : Yder_din(coef["CYr"]),
        "Nr_s1"    : Nder_din(coef["Cnr"]),
        "Lr_s1"    : Lder_din(coef["Clr"]),

        # ── control alerón  (estáticas) ────────────────────────────────
        "Yda_m_s2" : Yder_est(coef["CYda"]),
        "Nda_s2"   : Nder_est(coef["Cnda"]),
        "Lda_s2"   : Lder_est(coef["Clda"]),

        # ── control timón  (estáticas) ─────────────────────────────────
        "Ydr_m_s2" : Yder_est(coef["CYdr"]),
        "Ndr_s2"   : Nder_est(coef["Cndr"]),
        "Ldr_s2"   : Lder_est(coef["Cldr"]),
    }


# ─────────────────────────────────────────────────────────────────────────────
# 5. MODOS LATERALES-DIRECCIONALES
# ─────────────────────────────────────────────────────────────────────────────

def calcular_modos(aux: dict, dim: dict) -> dict:
    """
    Estima los tres modos clásicos laterales-direccionales mediante
    aproximaciones analíticas de orden reducido.

    ┌─────────────────────────────────────────────────────────────────┐
    │  Roll mode   : λ_roll = Lp    (1er orden, raíz real)           │
    │  Spiral mode : λ_s = (Lβ·Nr − Lr·Nβ) / Lβ  (1er orden)       │
    │  Dutch roll  : sistema 2×2 reducido (β, r)                     │
    └─────────────────────────────────────────────────────────────────┘
    """
    u0 = aux["V0_m_s"]

    Yb = dim["Yb_m_s2"]
    Lb = dim["Lb_s2"]
    Nb = dim["Nb_s2"]
    Lp = dim["Lp_s1"]
    Nr = dim["Nr_s1"]
    Lr = dim["Lr_s1"]
    Yr = dim["Yr_m_s"]
    Lda= dim["Lda_s2"]

    # ── Roll mode ─────────────────────────────────────────────────────────────
    lambda_roll = Lp
    tau_roll    = -1.0 / Lp if Lp != 0 else float("inf")
    ganancia_roll = -Lda / Lp if Lp != 0 else float("inf")
    stable_roll   = lambda_roll < 0

    # ── Spiral mode ───────────────────────────────────────────────────────────
    numerador_spiral = Lb * Nr - Lr * Nb
    lambda_spiral    = numerador_spiral / Lb if Lb != 0 else float("inf")
    tau_spiral       = -1.0 / lambda_spiral if lambda_spiral != 0 else float("inf")
    stable_spiral    = lambda_spiral < 0
    # Condición analítica: Lβ·Nr > Lr·Nβ
    crit_spiral_lhs  = Lb * Nr
    crit_spiral_rhs  = Lr * Nb

    # ── Dutch roll ────────────────────────────────────────────────────────────
    # Matriz reducida 2×2 (β, r):
    #   a11 = Yβ/u0       a12 = -(1 - Yr/u0)
    #   a21 = Nβ           a22 = Nr
    a11 = Yb / u0
    a12 = -(1.0 - Yr / u0)
    a21 = Nb
    a22 = Nr

    # Polinomio característico: λ² - (a11+a22)λ + (a11·a22 - a12·a21) = 0
    traza = a11 + a22          # = −2ζωn
    det   = a11 * a22 - a12 * a21   # = ωn²

    wn_DR = math.sqrt(abs(det))
    zeta_DR = -traza / (2 * wn_DR) if wn_DR > 0 else 0.0

    # Discriminante para verificar par complejo
    discriminante = traza**2 - 4 * det
    if discriminante < 0:
        # Par complejo conjugado (caso esperado)
        wd_DR = wn_DR * math.sqrt(max(0, 1 - zeta_DR**2))
        sigma_DR = zeta_DR * wn_DR
        lambda_DR_real = -sigma_DR
        lambda_DR_imag =  wd_DR
    else:
        # Caso real (poco probable para Dutch roll)
        wd_DR = 0.0
        r1 = (-traza + math.sqrt(discriminante)) / 2
        r2 = (-traza - math.sqrt(discriminante)) / 2
        lambda_DR_real = max(r1, r2)   # raíz menos negativa
        lambda_DR_imag = 0.0

    T_DR = (2 * math.pi / wd_DR) if wd_DR > 0 else float("inf")
    stable_DR = lambda_DR_real < 0

    # ── Criterio MIL-F-8785C Nivel 1 ─────────────────────────────────────────
    MIL_zeta_min = 0.19
    MIL_wn_min   = 1.0   # rad/s
    cumple_MIL_zeta = zeta_DR >= MIL_zeta_min
    cumple_MIL_wn   = wn_DR   >= MIL_wn_min

    return {
        # Roll mode
        "roll": {
            "lambda_s1"      : lambda_roll,
            "tau_p_s"        : tau_roll,
            "ganancia_p_rad" : ganancia_roll,
            "estable"        : stable_roll,
        },
        # Spiral mode
        "spiral": {
            "lambda_s1"        : lambda_spiral,
            "tau_s_s"          : tau_spiral,
            "estable"          : stable_spiral,
            "Lb_Nr"            : crit_spiral_lhs,
            "Lr_Nb"            : crit_spiral_rhs,
            "crit_estabilidad" : crit_spiral_lhs > crit_spiral_rhs,
        },
        # Dutch roll
        "dutch_roll": {
            "lambda_real_s1"  : lambda_DR_real,
            "lambda_imag_s1"  : lambda_DR_imag,
            "wn_rad_s"        : wn_DR,
            "zeta"            : zeta_DR,
            "wd_rad_s"        : wd_DR,
            "T_s"             : T_DR,
            "estable"         : stable_DR,
            "MIL_zeta_min"    : MIL_zeta_min,
            "MIL_wn_min_rad_s": MIL_wn_min,
            "cumple_MIL_zeta" : cumple_MIL_zeta,
            "cumple_MIL_wn"   : cumple_MIL_wn,
        },
        # Matriz 2×2 del Dutch roll (referencia)
        "matriz_dutch_roll": {
            "a11_Yb_u0": a11,
            "a12"      : a12,
            "a21_Nb"   : a21,
            "a22_Nr"   : a22,
        },
    }


# ─────────────────────────────────────────────────────────────────────────────
# 6. MATRICES DE ESPACIO DE ESTADOS (referencia)
# ─────────────────────────────────────────────────────────────────────────────

def calcular_matrices_estado(aux: dict, dim: dict) -> dict:
    """
    Construye las matrices A (4×4) y B (4×2) del modelo lineal
    lateral-direccional en espacio de estados.

    Vector de estados:  x = [β, p, r, φ]ᵀ
    Vector de entradas: u = [δ_a, δ_r]ᵀ

    Ecuaciones (ejes viento, vuelo simétrico nivelado):
      β̇ = Yβ/u0·β  +  Yp/u0·p  −(1−Yr/u0)·r  +  g/u0·φ  +  Yda/u0·δa  +  Ydr/u0·δr
      ṗ = Lβ·β    +  Lp·p      +  Lr·r                    +  Lda·δa     +  Ldr·δr
      ṙ = Nβ·β    +  Np·p      +  Nr·r                    +  Nda·δa     +  Ndr·δr
      φ̇ =              p
    """
    u0 = aux["V0_m_s"]
    g  = 9.81   # m/s² (constante)

    Yb  = dim["Yb_m_s2"]
    Yp  = dim["Yp_m_s"]
    Yr  = dim["Yr_m_s"]
    Lb  = dim["Lb_s2"]
    Lp  = dim["Lp_s1"]
    Lr  = dim["Lr_s1"]
    Nb  = dim["Nb_s2"]
    Np  = dim["Np_s1"]
    Nr  = dim["Nr_s1"]
    Yda = dim["Yda_m_s2"]
    Ydr = dim["Ydr_m_s2"]
    Lda = dim["Lda_s2"]
    Ldr = dim["Ldr_s2"]
    Nda = dim["Nda_s2"]
    Ndr = dim["Ndr_s2"]

    # Matriz A  (filas: β, p, r, φ  —  columnas: β, p, r, φ)
    A = [
        [Yb/u0,  Yp/u0,  -(1 - Yr/u0),  g/u0],
        [Lb,     Lp,      Lr,             0.0 ],
        [Nb,     Np,      Nr,             0.0 ],
        [0.0,    1.0,     0.0,            0.0 ],
    ]

    # Matriz B  (filas: β, p, r, φ  —  columnas: δ_a, δ_r)
    B = [
        [Yda/u0,  Ydr/u0],
        [Lda,     Ldr   ],
        [Nda,     Ndr   ],
        [0.0,     0.0   ],
    ]

    return {
        "estados"  : ["beta_rad", "p_rad_s", "r_rad_s", "phi_rad"],
        "entradas" : ["delta_a_rad", "delta_r_rad"],
        "A"        : A,
        "B"        : B,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 7. ENSAMBLADO DEL JSON DE RESULTADOS
# ─────────────────────────────────────────────────────────────────────────────

def ensamblar_resultados(datos: dict,
                         aux  : dict,
                         coef : dict,
                         dim  : dict,
                         modos: dict,
                         ss   : dict) -> dict:
    """Reúne todos los bloques en un único diccionario de resultados."""

    aeronave = datos.get("aeronave", "aeronave")

    # Limpiar clave interna auxiliar
    coef_limpio = {k: v for k, v in coef.items() if not k.startswith("_")}

    return {
        "aeronave"   : aeronave,
        "descripcion": datos.get("descripcion", ""),
        "fecha_calculo": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),

        "condicion_vuelo": {
            "rho_kg_m3"      : aux["rho_kg_m3"],
            "V0_m_s"         : aux["V0_m_s"],
            "Q_inf_Pa"       : aux["Q_inf_Pa"],
            "W_N"            : aux["W_N"],
            "CL_trim"        : aux["CL_trim"],
            "AR"             : aux["AR"],
            "lambda_taper"   : aux["lambda_taper"],
            "V_v"            : aux["V_v"],
            "Z_lambda"       : aux["Z_lambda"],
            "tau"            : aux["tau"],
            "Ix_kg_m2"       : aux["Ix_kg_m2"],
            "Iz_kg_m2"       : aux["Iz_kg_m2"],
            "L_ref_m"        : aux["L_ref_m"],
            "R_z_m"          : aux["R_z_m"],
        },

        "coeficientes_adimensionales": {
            "estabilidad": {
                "CYb": coef_limpio["CYb"],
                "Cnb": coef_limpio["Cnb"],
                "Clb": coef_limpio["Clb"],
                "CYp": coef_limpio["CYp"],
                "Cnp": coef_limpio["Cnp"],
                "Clp": coef_limpio["Clp"],
                "CYr": coef_limpio["CYr"],
                "Cnr": coef_limpio["Cnr"],
                "Clr": coef_limpio["Clr"],
            },
            "control": {
                "CYda": coef_limpio["CYda"],
                "Clda": coef_limpio["Clda"],
                "Cnda": coef_limpio["Cnda"],
                "CYdr": coef_limpio["CYdr"],
                "Cndr": coef_limpio["Cndr"],
                "Cldr": coef_limpio["Cldr"],
            },
        },

        "derivadas_dimensionales": {
            "en_beta": {
                "Yb_m_s2": dim["Yb_m_s2"],
                "Nb_s2"  : dim["Nb_s2"],
                "Lb_s2"  : dim["Lb_s2"],
            },
            "en_p": {
                "Yp_m_s": dim["Yp_m_s"],
                "Np_s1" : dim["Np_s1"],
                "Lp_s1" : dim["Lp_s1"],
            },
            "en_r": {
                "Yr_m_s": dim["Yr_m_s"],
                "Nr_s1" : dim["Nr_s1"],
                "Lr_s1" : dim["Lr_s1"],
            },
            "control_aleron": {
                "Yda_m_s2": dim["Yda_m_s2"],
                "Nda_s2"  : dim["Nda_s2"],
                "Lda_s2"  : dim["Lda_s2"],
            },
            "control_timon": {
                "Ydr_m_s2": dim["Ydr_m_s2"],
                "Ndr_s2"  : dim["Ndr_s2"],
                "Ldr_s2"  : dim["Ldr_s2"],
            },
        },

        "modos_laterales": modos,
        "espacio_de_estados": ss,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 8. IMPRESIÓN EN CONSOLA (resumen)
# ─────────────────────────────────────────────────────────────────────────────

def imprimir_resumen(res: dict) -> None:
    sep = "─" * 60
    print(f"\n{'═'*60}")
    print(f"  Análisis lateral-direccional — {res['aeronave'].upper()}")
    print(f"  {res['fecha_calculo']}")
    print(f"{'═'*60}")

    cv = res["condicion_vuelo"]
    print(f"\n{sep}")
    print("  CONDICIÓN DE VUELO")
    print(sep)
    print(f"  ρ          = {cv['rho_kg_m3']:.4f}  kg/m³")
    print(f"  V₀ (TAS)   = {cv['V0_m_s']:.2f}  m/s")
    print(f"  Q∞         = {cv['Q_inf_Pa']:.2f}  Pa")
    print(f"  W          = {cv['W_N']:.1f}  N")
    print(f"  CL_trim    = {cv['CL_trim']:.4f}")
    print(f"  AR         = {cv['AR']:.4f}")
    print(f"  Ix         = {cv['Ix_kg_m2']:.2f}  kg·m²")
    print(f"  Iz         = {cv['Iz_kg_m2']:.2f}  kg·m²")

    ca = res["coeficientes_adimensionales"]
    print(f"\n{sep}")
    print("  COEFICIENTES ADIMENSIONALES — ESTABILIDAD")
    print(sep)
    for k, v in ca["estabilidad"].items():
        print(f"  {k:6s} = {v:+.4f}")
    print(f"\n  COEFICIENTES ADIMENSIONALES — CONTROL")
    print(sep)
    for k, v in ca["control"].items():
        print(f"  {k:6s} = {v:+.4f}")

    dd = res["derivadas_dimensionales"]
    print(f"\n{sep}")
    print("  DERIVADAS DIMENSIONALES")
    print(sep)
    grupos = [
        ("en_beta",        "En β"),
        ("en_p",           "En p"),
        ("en_r",           "En r"),
        ("control_aleron", "Control δ_a"),
        ("control_timon",  "Control δ_r"),
    ]
    for key, nombre in grupos:
        print(f"  {nombre}:")
        for k, v in dd[key].items():
            print(f"    {k:12s} = {v:+.4f}")

    ml = res["modos_laterales"]
    print(f"\n{sep}")
    print("  MODOS LATERALES-DIRECCIONALES")
    print(sep)

    ro = ml["roll"]
    print(f"  Roll mode:")
    print(f"    λ_roll  = {ro['lambda_s1']:+.4f}  s⁻¹")
    print(f"    τ_p     = {ro['tau_p_s']:.4f}  s")
    print(f"    Estable = {ro['estable']}")

    sp = ml["spiral"]
    print(f"  Spiral mode:")
    print(f"    λ_s     = {sp['lambda_s1']:+.4f}  s⁻¹")
    print(f"    τ_s     = {sp['tau_s_s']:.4f}  s")
    print(f"    Estable = {sp['estable']}")

    dr = ml["dutch_roll"]
    print(f"  Dutch roll:")
    print(f"    λ_DR    = {dr['lambda_real_s1']:+.4f} ± {dr['lambda_imag_s1']:.4f}i  s⁻¹")
    print(f"    ω_n     = {dr['wn_rad_s']:.4f}  rad/s")
    print(f"    ζ       = {dr['zeta']:.4f}")
    print(f"    T_DR    = {dr['T_s']:.4f}  s")
    print(f"    Estable          = {dr['estable']}")
    print(f"    Cumple MIL ζ≥0.19= {dr['cumple_MIL_zeta']}")
    print(f"    Cumple MIL ω≥1.0 = {dr['cumple_MIL_wn']}")

    print(f"\n{'═'*60}\n")


# ─────────────────────────────────────────────────────────────────────────────
# 9. ESCRITURA DEL JSON DE SALIDA
# ─────────────────────────────────────────────────────────────────────────────

def guardar_resultados(res: dict, directorio: str = ".") -> str:
    """Guarda el JSON con el nombre <aeronave>_latdir_resultados.json"""
    aeronave    = res["aeronave"].lower().replace(" ", "_")
    nombre_arch = f"{aeronave}_latdir_resultados.json"
    ruta_salida = os.path.join(directorio, nombre_arch)
    with open(ruta_salida, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    return ruta_salida


# ─────────────────────────────────────────────────────────────────────────────
# 10. SELECTOR INTERACTIVO DE ARCHIVO
# ─────────────────────────────────────────────────────────────────────────────

def buscar_jsons(directorio: str) -> list[str]:
    """
    Devuelve todos los .json del directorio dado que NO sean archivos de
    resultados (excluye los que contienen 'resultados' en el nombre).
    Los ordena alfabéticamente.
    """
    try:
        archivos = [
            f for f in os.listdir(directorio)
            if f.lower().endswith(".json") and "resultados" not in f.lower()
        ]
        archivos.sort()
        return archivos
    except PermissionError:
        return []


def seleccionar_archivo() -> str:
    """
    Muestra un menú interactivo para elegir el JSON de parámetros.

    Modos de uso
    ─────────────────────────────────────────────────────────────────────
    1. Argumento CLI  →  python latdir_analysis.py mi_avion.json
    2. Menú numérico  →  lista los .json del directorio actual + opción
                         para escribir una ruta libre
    3. Ruta directa   →  el usuario escribe la ruta completa o relativa
    """

    # ── Modo 1: argumento en línea de comandos ────────────────────────────────
    if len(sys.argv) >= 2:
        ruta = sys.argv[1]
        if not ruta.lower().endswith(".json"):
            print(f"[ERROR] El archivo debe ser .json  →  '{ruta}'")
            sys.exit(1)
        if not os.path.isfile(ruta):
            print(f"[ERROR] No se encontró el archivo: '{ruta}'")
            sys.exit(1)
        return ruta

    # ── Modo 2/3: menú interactivo ────────────────────────────────────────────
    directorio_actual = os.getcwd()
    jsons_locales = buscar_jsons(directorio_actual)

    sep = "─" * 60
    print(f"\n{'═'*60}")
    print("  Análisis lateral-direccional de aeronaves")
    print("  Selección de archivo de parámetros (.json)")
    print(f"{'═'*60}")

    while True:
        # Listar archivos encontrados en el directorio actual
        print(f"\n  Directorio activo: {directorio_actual}")
        print(f"\n{sep}")

        if jsons_locales:
            print("  Archivos .json disponibles:\n")
            for i, nombre in enumerate(jsons_locales, start=1):
                # Mostrar nombre de aeronave si está en el JSON
                etiqueta = nombre
                try:
                    with open(os.path.join(directorio_actual, nombre),
                              encoding="utf-8") as _f:
                        _d = json.load(_f)
                        aeronave = _d.get("aeronave", "")
                        if aeronave:
                            etiqueta = f"{nombre}  [{aeronave}]"
                except Exception:
                    pass
                print(f"    [{i}]  {etiqueta}")
        else:
            print("  No se encontraron archivos .json en el directorio actual.")

        print(f"\n    [R]  Escribir ruta manualmente")
        print(f"    [D]  Cambiar directorio de búsqueda")
        print(f"    [S]  Salir")
        print(sep)

        opcion = input("\n  Selección: ").strip()

        # ── Salir ─────────────────────────────────────────────────────────────
        if opcion.upper() == "S":
            print("\n[INFO] Operación cancelada por el usuario.\n")
            sys.exit(0)

        # ── Cambiar directorio ────────────────────────────────────────────────
        if opcion.upper() == "D":
            nueva_ruta = input("  Ruta del directorio: ").strip()
            nueva_ruta = os.path.expanduser(nueva_ruta)   # soporta ~
            if os.path.isdir(nueva_ruta):
                directorio_actual = os.path.abspath(nueva_ruta)
                jsons_locales = buscar_jsons(directorio_actual)
                if not jsons_locales:
                    print(f"  [AVISO] No hay .json de parámetros en: {directorio_actual}")
            else:
                print(f"  [ERROR] Directorio no encontrado: '{nueva_ruta}'")
            continue

        # ── Ruta manual ───────────────────────────────────────────────────────
        if opcion.upper() == "R":
            ruta_manual = input("  Ruta del archivo .json: ").strip()
            ruta_manual = os.path.expanduser(ruta_manual)
            # Si no es absoluta, se busca relativa al directorio actual
            if not os.path.isabs(ruta_manual):
                ruta_manual = os.path.join(directorio_actual, ruta_manual)
            ruta_manual = os.path.abspath(ruta_manual)
            if not ruta_manual.lower().endswith(".json"):
                print("  [ERROR] El archivo debe tener extensión .json")
                continue
            if not os.path.isfile(ruta_manual):
                print(f"  [ERROR] No se encontró: '{ruta_manual}'")
                continue
            return ruta_manual

        # ── Selección numérica ────────────────────────────────────────────────
        if opcion.isdigit():
            idx = int(opcion) - 1
            if 0 <= idx < len(jsons_locales):
                return os.path.join(directorio_actual, jsons_locales[idx])
            else:
                print(f"  [ERROR] Número fuera de rango. Elige entre 1 y {len(jsons_locales)}.")
                continue

        print("  [ERROR] Opción no reconocida. Intenta de nuevo.")


# ─────────────────────────────────────────────────────────────────────────────
# 11. MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    # Obtener ruta del JSON (CLI o menú interactivo)
    ruta_entrada = seleccionar_archivo()

    # Directorio de salida = mismo que el archivo de entrada
    directorio_salida = os.path.dirname(os.path.abspath(ruta_entrada))

    print(f"\n[INFO] Leyendo datos desde: {ruta_entrada}")
    datos = cargar_datos(ruta_entrada)

    print("[INFO] Calculando condición de vuelo y parámetros auxiliares...")
    aux = calcular_condicion_vuelo(datos)

    print("[INFO] Calculando coeficientes adimensionales...")
    coef = calcular_coeficientes(datos, aux)

    print("[INFO] Calculando derivadas dimensionales...")
    dim = calcular_derivadas_dimensionales(datos, aux, coef)

    print("[INFO] Estimando modos laterales-direccionales...")
    modos = calcular_modos(aux, dim)

    print("[INFO] Construyendo matrices de espacio de estados...")
    ss = calcular_matrices_estado(aux, dim)

    print("[INFO] Ensamblando resultados...")
    resultados = ensamblar_resultados(datos, aux, coef, dim, modos, ss)

    imprimir_resumen(resultados)

    ruta_salida = guardar_resultados(resultados, directorio_salida)
    print(f"[OK] Resultados guardados en: {ruta_salida}\n")


if __name__ == "__main__":
    main()
