# -*- coding: utf-8 -*-
"""Grafica de rapidez angular: giroscopios en los tres ejes (deg/s).

Sobrepone las tasas del estado (``dp_radps``, ``dq_radps``, ``dr_radps``) como
lineas discontinuas si estan disponibles y son distintas del giroscopio del eje.
"""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import to_deg
from flightreview.plots.base import (
    SERIES_PALETTE,
    add_hover,
    add_series,
    add_state_rate_overlays,
    time_figure,
)

# (canonico, etiqueta, indice_color)
_GYROS = [
    ("gyro_x", "Roll rate", 0),
    ("gyro_y", "Pitch rate", 1),
    ("gyro_z", "Yaw rate", 2),
]

# Tasas del estado sobrepuestas: (canonico, giro_del_eje, campo, etiqueta, hover, indice_color)
_STATE_RATES = [
    ("roll_rate", "gyro_x", "rate_dp_degps", "Roll rate (dp)", "dp", 4),
    ("pitch_rate", "gyro_y", "rate_dq_degps", "Pitch rate (dq)", "dq", 3),
    ("yaw_rate", "gyro_z", "rate_dr_degps", "Yaw rate (dr)", "dr", 5),
]


def build(log: FlightLog, source, x_range):
    fig = time_figure("Rapidez angular", "Velocidad angular [deg/s]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for canon, label, ci in _GYROS:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        field = f"rate_{canon}_degps"
        source.data[field] = to_deg(log.df_plot[col])
        add_series(fig, source, t, field, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((field, label))

    hover_series += add_state_rate_overlays(fig, log, source, t, _STATE_RATES)

    if faltan:
        fig.title.text = f"Rapidez angular  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
