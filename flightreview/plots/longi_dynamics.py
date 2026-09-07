# -*- coding: utf-8 -*-
"""Graficas de estados longitudinales del SIL: du, dw, dq y dtheta, cada uno
en su propia figura con eje de tiempo enlazado.
"""

from __future__ import annotations

from bokeh.layouts import column

from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import to_deg
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

# (canonico, titulo, etiqueta_serie, campo, unidad_y, es_angular, indice_color)
_STATES = [
    ("du", "du", "du [m/s]", "longi_du_mps", "Velocidad [m/s]", False, 0),
    ("dw", "dw", "dw [m/s]", "longi_dw_mps", "Velocidad [m/s]", False, 1),
    ("pitch_rate", "dq", "dq [deg/s]", "longi_dq_degs", "Velocidad angular [deg/s]", True, 2),
    ("pitch", "dtheta", "dtheta [deg]", "longi_dtheta_deg", "Angulo [deg]", True, 3),
]


def _build_one(log: FlightLog, source, x_range, canon, titulo, label, field, y_label, es_angular, ci):
    fig = time_figure(titulo, y_label, x_range=x_range)
    t = log.cols.t

    col = log.cols[canon]
    if col is None:
        fig.title.text = f"{titulo}  (no disponible)"
        return fig

    source.data[field] = to_deg(log.df_plot[col]) if es_angular else log.df_plot[col]
    add_series(fig, source, t, field, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
    add_hover(fig, t, [(field, label)])
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig


def build(log: FlightLog, source, x_range):
    figuras = [
        _build_one(log, source, x_range, canon, titulo, label, field, y_label, es_angular, ci)
        for canon, titulo, label, field, y_label, es_angular, ci in _STATES
    ]
    return column(*figuras, sizing_mode="stretch_width")
