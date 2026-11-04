# -*- coding: utf-8 -*-
"""Filtro de giros: por eje, giro crudo, giro filtrado y estado verdadero.

La leyenda indica el corte (``gyro_*_fc_hz``) y el ruido antes y despues del
filtro. La sigma se mide sobre giro menos estado verdadero, para que las
maniobras no cuenten como ruido; si el log no trae el estado, se mide sobre la
senal completa y la leyenda lo indica.
"""

from __future__ import annotations

import numpy as np
from bokeh.layouts import column

from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import to_deg
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

# (eje, giro, giro filtrado, corte, estado, etiqueta del estado, color)
_EJES = [
    ("X", "gyro_x", "gyro_x_filt", "gyro_x_fc", "roll_rate", "dp (estado)", 0),
    ("Y", "gyro_y", "gyro_y_filt", "gyro_y_fc", "pitch_rate", "dq (estado)", 1),
    ("Z", "gyro_z", "gyro_z_filt", "gyro_z_fc", "yaw_rate", "dr (estado)", 2),
]
_COLOR_CRUDO = "#bdbdbd"


def _texto_fc(log: FlightLog, canon_fc: str) -> str:
    col = log.cols[canon_fc]
    if col is None:
        return "fc desconocido"
    fc = float(log.df_plot[col].median())
    return f"fc = {fc:g} Hz" if fc > 0.0 else "sin filtro"


def _sigmas(crudo, filtrado, estado) -> tuple[float, float, str]:
    """Sigma [deg/s] del crudo y del filtrado, y sufijo de la leyenda."""
    if estado is None:
        return float(np.std(crudo)), float(np.std(filtrado)), " (senal completa)"
    return float(np.std(crudo - estado)), float(np.std(filtrado - estado)), ""


def _build_eje(log: FlightLog, source, x_range, eje):
    nombre, canon_giro, canon_filt, canon_fc, canon_estado, etiqueta_estado, ci = eje
    fig = time_figure(f"Giro {nombre}: filtro", "Velocidad angular [deg/s]", x_range=x_range)
    t = log.cols.t
    color = SERIES_PALETTE[ci % len(SERIES_PALETTE)]
    hover_series: list[tuple[str, str]] = []

    col_giro = log.cols[canon_giro]
    col_filt = log.cols[canon_filt]
    col_estado = log.cols[canon_estado]
    if col_estado == col_giro:
        col_estado = None

    crudo = None
    if col_giro is not None:
        crudo = to_deg(log.df_plot[col_giro])
        field = f"filtro_{canon_giro}_degs"
        source.data[field] = crudo
        add_series(fig, source, t, field, "Crudo", _COLOR_CRUDO, width=1.0)
        hover_series.append((field, "Crudo"))

    estado = None
    if col_estado is not None:
        estado = to_deg(log.df_plot[col_estado])
        field = f"filtro_{canon_estado}_degs"
        source.data[field] = estado

    if col_filt is None:
        fig.title.text = f"Giro {nombre}: filtro  (no disponible en este log)"
    else:
        filtrado = to_deg(log.df_plot[col_filt])
        field = f"filtro_{canon_filt}_degs"
        source.data[field] = filtrado
        etiqueta = f"Filtrado ({_texto_fc(log, canon_fc)}"
        if crudo is not None:
            s_crudo, s_filt, sufijo = _sigmas(crudo, filtrado, estado)
            etiqueta += f", σ {s_crudo:.2f} → {s_filt:.2f} deg/s{sufijo}"
        etiqueta += ")"
        add_series(fig, source, t, field, etiqueta, color, width=1.5)
        hover_series.append((field, "Filtrado"))

    # El estado va al final para quedar encima de las otras series.
    if estado is not None:
        field = f"filtro_{canon_estado}_degs"
        add_series(fig, source, t, field, etiqueta_estado, "#000000", dash="dashed", width=1.0)
        hover_series.append((field, etiqueta_estado))

    if hover_series:
        add_hover(fig, t, hover_series)
        fig.legend.click_policy = "hide"
        fig.legend.location = "top_left"
    return fig


def build(log: FlightLog, source, x_range):
    return column(
        *[_build_eje(log, source, x_range, eje) for eje in _EJES],
        sizing_mode="stretch_width",
    )
