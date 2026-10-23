"""Pruebas del convertidor lon_analysis + latdir_analysis -> aircraft/*.json.

Correr desde la raíz del repo:  pytest dynamics
"""

import json
import math
import os
import shutil
import sys

import numpy as np
import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(RAIZ, "dynamics"))

import a_aircraft as conv  # noqa: E402

RES_LON  = os.path.join(RAIZ, "dynamics", "ugly_lon_resultados.json")
RES_LAT  = os.path.join(RAIZ, "dynamics", "ugly_latdir_resultados.json")
AERONAVE = os.path.join(RAIZ, "aircraft", "ugly_stick.json")
G_C      = 9.81   # DYN_GRAVITY_MPS2 de src/dynamics.c


@pytest.fixture
def avion_tmp(tmp_path):
    ruta = tmp_path / "avion.json"
    shutil.copy(AERONAVE, ruta)
    return ruta


def _cargar(ruta):
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def _ordenados(ev):
    return np.array(sorted(ev, key=lambda z: (round(z.real, 6), z.imag)))


def _A_lon_como_el_C(p):
    """Misma A que build_state_space_matrices() (src/dynamics.c)."""
    return np.array([
        [p["X_u"],     p["X_w"],     -p["w0_ms"],  p["X_theta"]],
        [p["Z_u_hat"], p["Z_w_hat"], p["u0_ms"],   p["Z_theta"]],
        [p["M_u_hat"], p["M_w_hat"], p["M_q_hat"], 0.0],
        [0.0,          0.0,          1.0,          0.0],
    ])


def _A_lat_como_el_C(p):
    """Misma A que build_lateral_matrices() (src/dynamics.c) con I_xz = 0."""
    th = math.radians(p["theta0_deg"])
    return np.array([
        [p["Y_v"], p["Y_p"] + p["w0_ms"], -(p["u0_ms"] - p["Y_r"]), G_C * math.cos(th)],
        [p["L_v"], p["L_p"], p["L_r"], 0.0],
        [p["N_v"], p["N_p"], p["N_r"], 0.0],
        [0.0, 1.0, math.tan(th), 0.0],
    ])


def _A_lat_estabilidad(lat, alpha):
    """
    A lateral en ejes de estabilidad armada por otro camino: I_x, I_z de ejes
    cuerpo pasadas a estabilidad (aparece I_xz) y momentos de latdir sin girar.
    Estados [v, p_s, r_s, φ_s]; vuelo nivelado (γ0 = 0).
    """
    cv = lat["condicion_vuelo"]
    V0, Ix, Iz = cv["V0_m_s"], cv["Ix_kg_m2"], cv["Iz_kg_m2"]
    d = {}
    for g in lat["derivadas_dimensionales"].values():
        d.update(g)
    c, s = math.cos(alpha), math.sin(alpha)
    T = np.array([[c, -s], [s, c]])
    J_s = T.T @ np.diag([Ix, Iz]) @ T
    M = np.array([[Ix * d["Lb_s2"] / V0, Ix * d["Lp_s1"], Ix * d["Lr_s1"]],
                  [Iz * d["Nb_s2"] / V0, Iz * d["Np_s1"], Iz * d["Nr_s1"]]])
    filas_pr = np.linalg.solve(J_s, M)
    return np.array([
        [d["Yb_m_s2"] / V0, d["Yp_m_s"], d["Yr_m_s"] - V0, G_C],
        [*filas_pr[0], 0.0],
        [*filas_pr[1], 0.0],
        [0.0, 1.0, 0.0, 0.0],
    ])


def test_eigenvalores_lon_iguales_en_ambos_ejes(avion_tmp):
    """Cambiar de ejes es una semejanza: los modos no cambian."""
    conv.convertir(RES_LON, RES_LAT, str(avion_tmp))
    A_c = _A_lon_como_el_C(_cargar(avion_tmp))
    A_s = np.array(_cargar(RES_LON)["espacio_de_estados"]["A"])
    np.testing.assert_allclose(_ordenados(np.linalg.eigvals(A_c)),
                               _ordenados(np.linalg.eigvals(A_s)), atol=1e-4)


def test_eigenvalores_lat_iguales_en_ambos_ejes(avion_tmp):
    conv.convertir(RES_LON, RES_LAT, str(avion_tmp))
    lon = _cargar(RES_LON)
    assert lon["condicion_vuelo"]["gamma0_deg"] == 0.0
    A_c = _A_lat_como_el_C(_cargar(avion_tmp))
    A_s = _A_lat_estabilidad(_cargar(RES_LAT), conv.alpha_trim(lon))
    np.testing.assert_allclose(_ordenados(np.linalg.eigvals(A_c)),
                               _ordenados(np.linalg.eigvals(A_s)), atol=1e-4)


def test_trim_en_ejes_cuerpo(avion_tmp):
    conv.convertir(RES_LON, RES_LAT, str(avion_tmp))
    p   = _cargar(avion_tmp)
    lon = _cargar(RES_LON)
    a   = conv.alpha_trim(lon)
    V0  = lon["condicion_vuelo"]["V0_m_s"]
    th  = math.radians(p["theta0_deg"])
    assert p["u0_ms"] == pytest.approx(V0 * math.cos(a), rel=1e-5)
    assert p["w0_ms"] == pytest.approx(V0 * math.sin(a), rel=1e-5)
    assert p["X_theta"] == pytest.approx(-G_C * math.cos(th), rel=1e-5)
    assert p["Z_theta"] == pytest.approx(-G_C * math.sin(th), rel=1e-5)
    # Vuelo nivelado: V_D = -u0 sin(theta0) + w0 cos(theta0) = 0
    assert -p["u0_ms"] * math.sin(th) + p["w0_ms"] * math.cos(th) == pytest.approx(0.0, abs=1e-4)
    Z_de_s = lon["espacio_de_estados"]["B"][1][0]
    assert p["X_de"] == pytest.approx(-math.sin(a) * Z_de_s, rel=1e-5)
    assert p["delta_e_trim_deg"] == pytest.approx(
        lon["condicion_vuelo"]["delta_e_trim_deg"], rel=1e-5)
    assert p["delta_e_trim_deg"] != 0.0


def test_lateral_sin_giro_es_beta_a_dv():
    """Con alpha = 0 solo queda el paso β -> dv de la versión anterior."""
    res = _cargar(RES_LAT)
    u0  = res["condicion_vuelo"]["V0_m_s"]
    d   = res["derivadas_dimensionales"]
    p   = conv.mapear_lateral(res, 0.0)
    assert p["Y_v"] == pytest.approx(d["en_beta"]["Yb_m_s2"] / u0, rel=1e-12)
    assert p["L_v"] == pytest.approx(d["en_beta"]["Lb_s2"] / u0, rel=1e-12)
    assert p["N_v"] == pytest.approx(d["en_beta"]["Nb_s2"] / u0, rel=1e-12)
    assert p["L_p"] == pytest.approx(d["en_p"]["Lp_s1"], rel=1e-12)
    assert p["N_r"] == pytest.approx(d["en_r"]["Nr_s1"], rel=1e-12)
    assert p["L_da"] == pytest.approx(d["control_aleron"]["Lda_s2"], rel=1e-12)
    assert p["I_xz"] == 0.0


def test_conserva_claves_no_tocadas(avion_tmp):
    antes = _cargar(avion_tmp)
    nuevos = conv.convertir(RES_LON, RES_LAT, str(avion_tmp))
    despues = _cargar(avion_tmp)
    assert list(antes) == list(despues)            # mismo orden de claves
    for k in antes:
        if k not in nuevos:
            assert antes[k] == despues[k], k


def test_dry_run_no_escribe(avion_tmp):
    antes = avion_tmp.read_bytes()
    conv.convertir(RES_LON, RES_LAT, str(avion_tmp), dry_run=True)
    assert avion_tmp.read_bytes() == antes


@pytest.mark.parametrize("clave, delta, mensaje", [
    ("V0_ms",   1.0,   "V0"),
    ("masa_kg", 1.0,   "masa"),
    ("Alt_m",   100.0, "altitud"),
])
def test_rechaza_condicion_distinta(avion_tmp, clave, delta, mensaje):
    p = _cargar(avion_tmp)
    p[clave] = p[clave] + delta
    avion_tmp.write_text(json.dumps(p), encoding="utf-8")
    with pytest.raises(ValueError, match=mensaje):
        conv.convertir(RES_LON, RES_LAT, str(avion_tmp))
