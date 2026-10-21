"""Pruebas de la atmósfera ISA (atmosfera.py)."""

import os
import sys

import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(RAIZ, "dynamics"))

from atmosfera import densidad_isa  # noqa: E402


@pytest.mark.parametrize("h_m, rho_tabla", [
    (0.0,    1.2250),
    (1000.0, 1.1117),
    (5000.0, 0.7361),
])
def test_densidad_contra_tabla_isa(h_m, rho_tabla):
    assert densidad_isa(h_m) == pytest.approx(rho_tabla, abs=1e-4)


def test_fuera_de_troposfera():
    with pytest.raises(ValueError):
        densidad_isa(12000.0)
