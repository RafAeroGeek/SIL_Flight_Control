# -*- coding: utf-8 -*-
"""Grafica de rapidez angular: giroscopios en los tres ejes (deg/s).

Sobrepone ``dq_radps`` (tasa de pitch del estado) como linea discontinua si esta
disponible y es distinta del giroscopio en Y.
"""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import to_deg
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

# (canonico, etiqueta, indice_color)
_GYROS = [
    ("gyro_x", "Roll rate", 0),
    ("gyro_y", "Pitch rate", 1),
    ("gyro_z", "Yaw rate", 2),
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

    dq_col = log.cols.pitch_rate
    if dq_col is not None and dq_col != log.cols.gyro_y:
        source.data["rate_dq_degps"] = to_deg(log.df_plot[dq_col])
        add_series(fig, source, t, "rate_dq_degps", "Pitch rate (dq)",
                   SERIES_PALETTE[3 % len(SERIES_PALETTE)], dash="dashed")
        hover_series.append(("rate_dq_degps", "dq"))

    if faltan:
        fig.title.text = f"Rapidez angular  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
