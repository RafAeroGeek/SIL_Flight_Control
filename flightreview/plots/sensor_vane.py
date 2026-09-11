# -*- coding: utf-8 -*-
"""Grafica de la veleta: angulo de ataque y de derrape."""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

_VANE = [("aoa", "Angulo de ataque", 0), ("sideslip", "Angulo de derrape", 1)]


def build(log: FlightLog, source, x_range):
    fig = time_figure("Angulos de Ataque y Derrape", "Angulo [deg]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for canon, label, ci in _VANE:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        add_series(fig, source, t, col, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((col, label))

    if faltan:
        fig.title.text = f"Angulos de Ataque y Derrape  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
