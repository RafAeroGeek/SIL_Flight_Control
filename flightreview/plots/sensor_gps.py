# -*- coding: utf-8 -*-
"""Graficas de GPS: posicion (lat/lon/alt) y velocidad (N/E/D), en dos figuras
con eje de tiempo enlazado.
"""

from __future__ import annotations

from bokeh.layouts import column

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

_POS = [
    ("gps_lat", "Lat [deg]", 0),
    ("gps_lon", "Lon [deg]", 1),
    ("gps_alt", "Alt [m]", 2),
]
_VEL = [
    ("gps_vn", "Vn [m/s]", 0),
    ("gps_ve", "Ve [m/s]", 1),
    ("gps_vd", "Vd [m/s]", 2),
]


def _build_pos(log: FlightLog, source, x_range):
    fig = time_figure("Posicion GPS", "Posicion (unidades mixtas)", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for canon, label, ci in _POS:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        field = f"sensor_{canon}"
        source.data[field] = log.df_plot[col]
        add_series(fig, source, t, field, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((field, label))

    if faltan:
        fig.title.text = f"Posicion GPS  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig


def _build_vel(log: FlightLog, source, x_range):
    fig = time_figure("Velocidad GPS", "Velocidad [m/s]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for canon, label, ci in _VEL:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        field = f"sensor_{canon}"
        source.data[field] = log.df_plot[col]
        add_series(fig, source, t, field, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((field, label))

    if faltan:
        fig.title.text = f"Velocidad GPS  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig


def build(log: FlightLog, source, x_range):
    return column(
        _build_pos(log, source, x_range),
        _build_vel(log, source, x_range),
        sizing_mode="stretch_width",
    )
