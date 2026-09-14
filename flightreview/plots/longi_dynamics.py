# -*- coding: utf-8 -*-
"""Graficas de estados longitudinales del SIL: velocidades du/dw y tasa/angulo
de pitch, cada una en su propia figura con eje de tiempo enlazado.
"""

from __future__ import annotations

from bokeh.layouts import column

from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import to_deg
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

# (canonico, titulo, etiqueta_eje_y, campo, es_angular, indice_color)
_STATES = [
    ("du", "Velocidad longitudinal (du)", "du [m/s]", "longi_du_mps", False, 0),
    ("dw", "Velocidad vertical (dw)", "dw [m/s]", "longi_dw_mps", False, 1),
    ("pitch_rate", "Tasa de cabeceo (dq)", "dq [deg/s]", "longi_dq_degs", True, 2),
    ("pitch", "Angulo de cabeceo (dtheta)", "dtheta [deg]", "longi_dtheta_deg", True, 3),
]


def _build_state(log: FlightLog, source, x_range, canon, titulo, y_label, field, es_angular, ci):
    fig = time_figure(titulo, y_label, x_range=x_range)
    t = log.cols.t
    col = log.cols[canon]

    if col is None:
        fig.title.text = f"{titulo}  (no disponible)"
        return fig

    source.data[field] = to_deg(log.df_plot[col]) if es_angular else log.df_plot[col]
    add_series(fig, source, t, field, y_label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
    add_hover(fig, t, [(field, y_label)])
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig


def build(log: FlightLog, source, x_range):
    figuras = [
        _build_state(log, source, x_range, canon, titulo, y_label, field, es_angular, ci)
        for canon, titulo, y_label, field, es_angular, ci in _STATES
    ]
    return column(*figuras, sizing_mode="stretch_width")
