# -*- coding: utf-8 -*-
"""Grafica del laser altimetro."""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure


def build(log: FlightLog, source, x_range):
    fig = time_figure("Laser altimetro", "Altitud [m]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    col = log.cols.laser_alt
    if col is None:
        faltan.append("Laser")
    else:
        field = "sensor_laser_alt_m"
        source.data[field] = log.df_plot[col]
        add_series(fig, source, t, field, "Laser", SERIES_PALETTE[0])
        hover_series.append((field, "Laser"))

    if faltan:
        fig.title.text = f"Laser altimetro  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
