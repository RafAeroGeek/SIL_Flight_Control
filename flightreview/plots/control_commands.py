# -*- coding: utf-8 -*-
"""Graficas de la cadena de control de superficies: mando del piloto, comandos
RC crudos, canales RC, salida a actuadores y superficies/throttle resultantes.

Cinco figuras independientes (una por etapa de la cadena), todas con el mismo
eje de tiempo enlazado.
"""

from __future__ import annotations

from bokeh.layouts import column

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

# Cada subfigura: (titulo, etiqueta_eje_y, prefijo_campo, series)
# series: lista de (canonico, etiqueta, indice_color)
_SUBFIGS = [
    (
        "Mando piloto", "Mando (normalizado)", "ctrl_pilot",
        [
            ("pilot_roll", "Roll", 0),
            ("pilot_pitch", "Pitch", 1),
            ("pilot_yaw", "Yaw", 2),
            ("pilot_throttle", "Throttle", 3),
        ],
    ),
    (
        "Comandos RC crudos", "Comando [us]", "ctrl_cmd",
        [
            ("elev_cmd", "Elevator", 0),
            ("ail_cmd", "Aileron", 1),
            ("rud_cmd", "Rudder", 2),
            ("thro_cmd", "Throttle", 3),
        ],
    ),
    (
        "Canales RC", "Canal RC", "ctrl_rc",
        [
            ("rc_pitch", "Pitch", 0),
            ("rc_roll", "Roll", 1),
            ("rc_yaw", "Yaw", 2),
            ("rc_throttle", "Throttle", 3),
            ("rc_flaps", "Flaps", 4),
        ],
    ),
    (
        "Salida a actuadores", "Salida [us]", "ctrl_u",
        [
            ("u_elev", "Elevator", 0),
            ("u_ail", "Aileron", 1),
            ("u_rud", "Rudder", 2),
            ("u_thro", "Throttle", 3),
        ],
    ),
    (
        "Superficies/throttle resultantes", "Salida (deg / norm)", "ctrl_y",
        [
            ("y_elev", "Elevator", 0),
            ("y_ail", "Aileron", 1),
            ("y_rud", "Rudder", 2),
            ("y_thro", "Throttle", 3),
        ],
    ),
]


def _build_subfig(log: FlightLog, source, x_range, titulo, y_label, prefijo, series):
    fig = time_figure(titulo, y_label, x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    for canon, label, ci in series:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        field = f"{prefijo}_{canon}"
        source.data[field] = log.df_plot[col]
        add_series(fig, source, t, field, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((field, label))

    if faltan:
        fig.title.text = f"{titulo}  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig


def build(log: FlightLog, source, x_range):
    figuras = [
        _build_subfig(log, source, x_range, titulo, y_label, prefijo, series)
        for titulo, y_label, prefijo, series in _SUBFIGS
    ]
    return column(*figuras, sizing_mode="stretch_width")
