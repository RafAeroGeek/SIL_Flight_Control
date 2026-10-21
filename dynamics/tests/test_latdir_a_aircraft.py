"""Pruebas del convertidor latdir_analysis -> aircraft/*.json.

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

import latdir_a_aircraft as conv  # noqa: E402

RESULTADOS = os.path.join(RAIZ, "dynamics", "ugly_latdir_resultados.json")
AERONAVE   = os.path.join(RAIZ, "aircraft", "ugly_stick.json")
G_C        = 9.80665   # DYN_GRAVITY_MPS2 del C


@pytest.fixture
def avion_tmp(tmp_path):
    ruta = tmp_path / "avion.json"
    shutil.copy(AERONAVE, ruta)
    return ruta


def _cargar(ruta):
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def _A_como_el_C(p):
    """Misma A que build_lateral_matrices() (src/dynamics.c) con I_xz = 0."""
    th = math.radians(p["theta0_deg"])
    return np.array([
        [p["Y_v"], p["Y_p"], -(p["u0_ms"] - p["Y_r"]), G_C * math.cos(th)],
        [p["L_v"], p["L_p"], p["L_r"], 0.0],
        [p["N_v"], p["N_p"], p["N_r"], 0.0],
        [0.0, 1.0, 0.0, 0.0],
    ])


def _ordenados(ev):
    return np.array(sorted(ev, key=lambda z: (round(z.real, 6), z.imag)))


def test_eigenvalores_coinciden_con_el_analisis(avion_tmp):
    conv.convertir(RESULTADOS, str(avion_tmp))
    A_c   = _A_como_el_C(_cargar(avion_tmp))
    A_res = np.array(_cargar(RESULTADOS)["espacio_de_estados"]["A"])
    # Diferencias esperadas: u0_ms (C) vs V0 (análisis), cos(theta0) y g;
    # los tres son de orden 1e-3 en u0 y en el término de gravedad.
    np.testing.assert_allclose(_ordenados(np.linalg.eigvals(A_c)),
                               _ordenados(np.linalg.eigvals(A_res)), atol=2e-2)


def test_conversion_beta_a_dv(avion_tmp):
    conv.convertir(RESULTADOS, str(avion_tmp))
    p   = _cargar(avion_tmp)
    res = _cargar(RESULTADOS)
    u0  = res["condicion_vuelo"]["V0_m_s"]
    d   = res["derivadas_dimensionales"]
    assert p["Y_v"] == pytest.approx(d["en_beta"]["Yb_m_s2"] / u0, rel=1e-5)
    assert p["L_v"] == pytest.approx(d["en_beta"]["Lb_s2"] / u0, rel=1e-5)
    assert p["N_v"] == pytest.approx(d["en_beta"]["Nb_s2"] / u0, rel=1e-5)
    assert p["L_da"] == pytest.approx(d["control_aleron"]["Lda_s2"], rel=1e-5)
    assert p["I_xz"] == 0.0
    assert p["lateral_source"] == conv.LATERAL_SOURCE


def test_conserva_claves_no_laterales(avion_tmp):
    antes = _cargar(avion_tmp)
    conv.convertir(RESULTADOS, str(avion_tmp))
    despues = _cargar(avion_tmp)
    tocadas = set(conv.mapear_derivadas(_cargar(RESULTADOS))) | {"lateral_source"}
    assert list(antes) == list(despues)            # mismo orden de claves
    for k in antes:
        if k not in tocadas:
            assert antes[k] == despues[k], k


def test_dry_run_no_escribe(avion_tmp):
    antes = avion_tmp.read_bytes()
    conv.convertir(RESULTADOS, str(avion_tmp), dry_run=True)
    assert avion_tmp.read_bytes() == antes


def test_rechaza_condicion_distinta(avion_tmp):
    p = _cargar(avion_tmp)
    p["V0_ms"] = p["V0_ms"] + 1.0
    avion_tmp.write_text(json.dumps(p), encoding="utf-8")
    with pytest.raises(ValueError, match="V0"):
        conv.convertir(RESULTADOS, str(avion_tmp))
