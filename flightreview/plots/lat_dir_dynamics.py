# -*- coding: utf-8 -*-
"""Graficas de estados lateral-direccionales del SIL: velocidad lateral dv,
tasas de alabeo/guinada dp/dr y angulo de alabeo dphi, cada una en su propia
figura con eje de tiempo enlazado.
"""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import state_column

# (canonico, titulo, etiqueta_eje_y, campo, es_angular, indice_color)
_STATES = [
    ("dv", "Velocidad lateral (dv)", "dv [m/s]", "latdir_dv_mps", False, 0),
    ("roll_rate", "Tasa de alabeo (dp)", "dp [deg/s]", "latdir_dp_degs", True, 1),
    ("yaw_rate", "Tasa de guinada (dr)", "dr [deg/s]", "latdir_dr_degs", True, 2),
    ("roll", "Angulo de alabeo (dphi)", "dphi [deg]", "latdir_dphi_deg", True, 3),
]


def build(log: FlightLog, source, x_range):
    return state_column(log, source, x_range, _STATES)
