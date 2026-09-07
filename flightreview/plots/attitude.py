# -*- coding: utf-8 -*-
"""Grafica de actitud: angulos estimados de Roll / Pitch / Yaw vs setpoints.

El simulador es longitudinal: hoy solo se resuelve pitch (``dtheta_rad``). Roll,
Yaw y los setpoints se dibujan solo si el CSV los trae; si no, se omiten y se
deja constancia en el titulo.
"""

from __future__ import annotations

from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import to_deg
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

# (canonico_estimado, canonico_setpoint, etiqueta, indice_color)
_AXES = [
    ("roll", "roll_sp", "Roll", 0),
    ("pitch", "pitch_sp", "Pitch", 1),
    ("yaw", "yaw_sp", "Yaw", 2),
]


def build(log: FlightLog, source, x_range):
    fig = time_figure("Actitud", "Angulo [deg]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for est, sp, label, ci in _AXES:
        color = SERIES_PALETTE[ci % len(SERIES_PALETTE)]
        est_col = log.cols[est]
        if est_col is not None:
            field = f"att_{est}_deg"
            source.data[field] = to_deg(log.df_plot[est_col])
            add_series(fig, source, t, field, f"{label} (est)", color)
            hover_series.append((field, f"{label} est"))
        else:
            faltan.append(label)

        sp_col = log.cols[sp]
        if sp_col is not None:
            field = f"att_{sp}_deg"
            source.data[field] = to_deg(log.df_plot[sp_col])
            add_series(fig, source, t, field, f"{label} (sp)", color, dash="dashed")
            hover_series.append((field, f"{label} sp"))

    if faltan:
        fig.title.text = f"Actitud  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
