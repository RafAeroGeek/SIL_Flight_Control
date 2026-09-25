# -*- coding: utf-8 -*-
"""Graficas de estados longitudinales del SIL: velocidades du/dw y tasa/angulo
de pitch, cada una en su propia figura con eje de tiempo enlazado.
"""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import state_column

# (canonico, titulo, etiqueta_eje_y, campo, es_angular, indice_color)
_STATES = [
    ("du", "Velocidad longitudinal (du)", "du [m/s]", "longi_du_mps", False, 0),
    ("dw", "Velocidad vertical (dw)", "dw [m/s]", "longi_dw_mps", False, 1),
    ("pitch_rate", "Tasa de cabeceo (dq)", "dq [deg/s]", "longi_dq_degs", True, 2),
    ("pitch", "Angulo de cabeceo (dtheta)", "dtheta [deg]", "longi_dtheta_deg", True, 3),
]


def build(log: FlightLog, source, x_range):
    return state_column(log, source, x_range, _STATES)
