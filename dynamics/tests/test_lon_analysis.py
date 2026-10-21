"""Pruebas del análisis longitudinal (lon_analysis.py).

Correr desde la raíz del repo:  pytest dynamics
"""

import json
import os
import sys

import numpy as np
import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(RAIZ, "dynamics"))

import lon_analysis as lon  # noqa: E402

ENTRADA    = os.path.join(RAIZ, "dynamics", "ugly_data.json")
RESULTADOS = os.path.join(RAIZ, "dynamics", "ugly_lon_resultados.json")


@pytest.fixture(scope="module")
def res():
    return lon.analizar(ENTRADA)


def test_resultados_commiteados_al_dia(res):
    """El JSON del repo es la salida actual del script (determinista)."""
    with open(RESULTADOS, encoding="utf-8") as f:
        assert json.loads(json.dumps(res)) == json.load(f)


def test_A_con_la_forma_del_C(res):
    """Disposición de build_state_space_matrices() (src/dynamics.c) + M̂_θ."""
    d  = res["derivadas_dimensionales"]
    u0 = res["condicion_vuelo"]["V0_m_s"]      # ejes de estabilidad
    A  = np.array(res["espacio_de_estados"]["A"])
    esperada = np.array([
        [d["X_u"],     d["X_w"],     0.0,          d["X_theta"]],
        [d["Z_u_hat"], d["Z_w_hat"], u0,           d["Z_theta"]],
        [d["M_u_hat"], d["M_w_hat"], d["M_q_hat"], d["M_theta_hat"]],
        [0.0,          0.0,          1.0,          0.0],
    ])
    np.testing.assert_array_equal(A, esperada)


def test_modos_exactos_son_los_eigenvalores(res):
    A  = np.array(res["espacio_de_estados"]["A"])
    ev = np.linalg.eigvals(A)
    ex = res["modos_longitudinales"]["exactos"]
    for clave in ("periodo_corto", "fugoide"):
        lam = complex(ex[clave]["lambda_real_s1"], ex[clave]["lambda_imag_s1"])
        assert np.min(np.abs(ev - lam)) < 1e-9, clave
    assert ex["periodo_corto"]["wn_rad_s"] > ex["fugoide"]["wn_rad_s"]
