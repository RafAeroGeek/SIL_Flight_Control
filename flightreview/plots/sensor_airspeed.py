# -*- coding: utf-8 -*-
"""Grafica de velocidad aerodinamica (pitot)."""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure


def build(log: FlightLog, source, x_range):
    fig = time_figure("Velocidad aerodinamica", "Velocidad [m/s]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    col = log.cols.airspeed
    if col is None:
        faltan.append("Pitot")
    else:
        field = "sensor_airspeed_ms"
        source.data[field] = log.df_plot[col]
        add_series(fig, source, t, field, "Pitot", SERIES_PALETTE[0])
        hover_series.append((field, "Pitot"))

    if faltan:
        fig.title.text = f"Velocidad aerodinamica  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
