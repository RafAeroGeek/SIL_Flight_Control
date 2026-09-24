# -*- coding: utf-8 -*-
"""Graficas del IMU: giroscopo (deg/s) y acelerometro (m/s^2), en dos figuras
con eje de tiempo enlazado.

Sobre Gyro X y Gyro Z se superponen en discontinua las tasas del estado
(``dp_radps``, ``dr_radps``): estado verdadero frente a la medicion con ruido.
"""

from __future__ import annotations

from bokeh.layouts import column

from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import to_deg
from flightreview.plots.base import (
    SERIES_PALETTE,
    add_hover,
    add_series,
    add_state_rate_overlays,
    time_figure,
)

_GYROS = [("gyro_x", "Gyro X", 0), ("gyro_y", "Gyro Y", 1), ("gyro_z", "Gyro Z", 2)]
# Tasas del estado sobrepuestas, con el color de su giro:
# (canonico, giro_del_eje, campo, etiqueta, hover, indice_color)
_STATE_RATES = [
    ("roll_rate", "gyro_x", "sensor_dp_degs", "dp (estado)", "dp", 0),
    ("yaw_rate", "gyro_z", "sensor_dr_degs", "dr (estado)", "dr", 2),
]
_ACCS = [("acc_x", "Acc X", 0), ("acc_y", "Acc Y", 1), ("acc_z", "Acc Z", 2)]


def _build_gyro(log: FlightLog, source, x_range):
    fig = time_figure("Giroscopo", "Velocidad angular [deg/s]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for canon, label, ci in _GYROS:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        field = f"sensor_{canon}_degs"
        source.data[field] = to_deg(log.df_plot[col])
        add_series(fig, source, t, field, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((field, label))

    hover_series += add_state_rate_overlays(fig, log, source, t, _STATE_RATES)

    if faltan:
        fig.title.text = f"Giroscopo  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig


def _build_accel(log: FlightLog, source, x_range):
    fig = time_figure("Acelerometro", "Aceleracion [m/s2]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for canon, label, ci in _ACCS:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        add_series(fig, source, t, col, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((col, label))

    if faltan:
        fig.title.text = f"Acelerometro  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig


def build(log: FlightLog, source, x_range):
    return column(
        _build_gyro(log, source, x_range),
        _build_accel(log, source, x_range),
        sizing_mode="stretch_width",
    )
