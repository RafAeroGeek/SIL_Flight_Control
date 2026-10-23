"""Pruebas del diseño del control lateral (control_lat_design.py).

Correr desde la raíz del repo:  pytest dynamics
"""

import json
import math
import os
import sys

import numpy as np
import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(RAIZ, "dynamics"))
sys.path.insert(0, os.path.dirname(__file__))

import control_lat_design as cd  # noqa: E402
from test_a_aircraft import _A_lat_como_el_C  # noqa: E402

AERONAVE = os.path.join(RAIZ, "aircraft", "ugly_stick.json")


@pytest.fixture(scope="module")
def avion():
    with open(AERONAVE, encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture(scope="module")
def resultado(avion):
    return cd.evaluar(avion)


def test_modelo_igual_al_del_C(avion):
    """El bloque 4x4 es la A del C; la fila de ψ es sec(θ0)·r."""
    A, B = cd.matrices_laterales(avion)
    np.testing.assert_allclose(A[:4, :4], _A_lat_como_el_C(avion), rtol=1e-12)
    th = math.radians(avion["theta0_deg"])
    assert A[4, 2] == pytest.approx(1.0 / math.cos(th))
    assert np.all(A[:, 4] == 0.0)
    assert B[2, 1] == pytest.approx(avion["N_dr"])      # N_δr < 0: Nelson
    assert avion["N_dr"] < 0.0 and avion["L_da"] > 0.0


def test_lazo_abierto(resultado):
    """Modos conocidos del Ugly Stick y el λ = 0 de ψ."""
    ps = resultado["polos_lazo_abierto"]
    re = sorted(p["re"] for p in ps)
    assert re[0] == pytest.approx(-13.31, abs=0.01)                 # roll
    assert max(re) == pytest.approx(0.0746, abs=1e-3)               # espiral inestable
    assert any(abs(p["wn"]) < 1e-9 for p in ps)                     # ψ
    dr = [p for p in ps if p["im"] > 1.0]
    assert dr[0]["zeta"] == pytest.approx(0.10, abs=0.005)


def test_cumple_criterios(resultado):
    assert cd.verificar(resultado) == []
    assert resultado["zeta_dr_yaw_damper"] >= 0.4
    assert resultado["zeta_min_rumbo"] >= 0.4


def test_espiral_estable_con_alabeo(resultado):
    ps = [p for p in resultado["polos_estabilizado"] if p["wn"] > 1e-7]
    assert max(p["re"] for p in ps) < 0.0


def test_rumbo_sin_error_estacionario(resultado):
    assert resultado["escalon_psi"]["final"] == pytest.approx(1.0, abs=0.01)


def test_signo_del_yaw_damper(avion):
    """Con N_δr < 0, K_r > 0 amortigua el dutch roll y K_r < 0 lo desestabiliza."""
    A, B = cd.planta_diseno(avion, 1.0)
    z = {}
    for kr in (-0.2, 0.0, 0.2):
        k = dict(cd.GANANCIAS, K_r=kr)
        z[kr] = cd.zeta_min(cd.lazo_cerrado(A, B, cd.ganancia_estado(k, "y")))
    assert z[-0.2] < z[0.0] < z[0.2]
    assert z[0.0] == pytest.approx(0.10, abs=0.005)


def test_rumbo_necesita_yaw_damper(resultado):
    """Sin yaw damper (timón con k < k_min) el modo de rumbo es inestable."""
    assert resultado["rango_ganancia_rumbo"]["timon"]["k_min"] > 0.05


def test_datos_para_figuras(avion, tmp_path):
    cd.escribir_datos(avion, str(tmp_path))
    for nombre in ("lugar_yaw_damper", "zeta_rumbo", "escalon_phi", "escalon_psi"):
        filas = (tmp_path / f"{nombre}.dat").read_text().splitlines()
        assert len(filas) > 10
