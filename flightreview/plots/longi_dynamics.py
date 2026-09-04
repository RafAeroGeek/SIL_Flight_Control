# -*- coding: utf-8 -*-
"""Grafica de estados longitudinales del SIL: velocidades du/dw y tasa/angulo
de pitch, todo en una sola figura con eje de tiempo enlazado.
"""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import to_deg
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

# (canonico, etiqueta, campo, es_angular, indice_color)
_STATES = [
    ("du", "du [m/s]", "longi_du_mps", False, 0),
    ("dw", "dw [m/s]", "longi_dw_mps", False, 1),
    ("pitch_rate", "dq [deg/s]", "longi_dq_degs", True, 2),
    ("pitch", "dtheta [deg]", "longi_dtheta_deg", True, 3),
]


def build(log: FlightLog, source, x_range):
    fig = time_figure("Estados longitudinales SIL", "Estado (unidades mixtas)", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for canon, label, field, es_angular, ci in _STATES:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        source.data[field] = to_deg(log.df_plot[col]) if es_angular else log.df_plot[col]
        add_series(fig, source, t, field, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((field, label))

    if faltan:
        fig.title.text = f"Estados longitudinales SIL  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
