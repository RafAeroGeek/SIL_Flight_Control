# -*- coding: utf-8 -*-
"""Grafica de la veleta: angulo de ataque y de derrape.

Si el log trae ``dv_mps`` y la velocidad del pitot, sobrepone en discontinua el
derrape estimado desde el estado, ``beta_est = atan2(dv, V)``, para contrastarlo
con la medicion ``SSA_deg`` (el simulador calcula esta con sqrt(u^2+w^2) ~ V).
"""

from __future__ import annotations

import numpy as np

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

_VANE = [("aoa", "Angulo de ataque", 0), ("sideslip", "Angulo de derrape", 1)]

# Por debajo de esta velocidad el cociente dv/V no tiene sentido (V ~ 0).
_V_MIN_MS = 1.0


def _beta_est_deg(log: FlightLog):
    """Derrape estimado desde el estado (deg); None si faltan dv o velocidad."""
    dv_col, v_col = log.cols.dv, log.cols.airspeed
    if dv_col is None or v_col is None:
        return None
    dv = log.df_plot[dv_col].to_numpy(dtype=float)
    v = log.df_plot[v_col].to_numpy(dtype=float)
    beta = np.degrees(np.arctan2(dv, v))
    beta[v < _V_MIN_MS] = np.nan
    return beta


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

    beta_est = _beta_est_deg(log)
    if beta_est is not None:
        source.data["vane_beta_est_deg"] = beta_est
        add_series(fig, source, t, "vane_beta_est_deg", "Derrape estimado atan2(dv, V)",
                   SERIES_PALETTE[1 % len(SERIES_PALETTE)], dash="dashed")
        hover_series.append(("vane_beta_est_deg", "beta_est"))

    if faltan:
        fig.title.text = f"Angulos de Ataque y Derrape  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
