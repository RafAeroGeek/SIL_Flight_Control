# -*- coding: utf-8 -*-
"""Grafica de vibracion: aceleraciones en los tres ejes y su modulo, con bandas
de aviso para identificar niveles anormales.
"""

from __future__ import annotations

from bokeh.models import BoxAnnotation

from flightreview.config import VIBRATION_OK_MPS2, VIBRATION_WARN_MPS2
from flightreview.parser.loader import FlightLog
from flightreview.parser.schema import accel_magnitude
from flightreview.plots.base import SERIES_PALETTE, add_hover, add_series, time_figure

_ACCS = [("acc_x", "Acc X", 0), ("acc_y", "Acc Y", 1), ("acc_z", "Acc Z", 2)]


def build(log: FlightLog, source, x_range):
    fig = time_figure("Vibracion", "Aceleracion [m/s2]", x_range=x_range)
    t = log.cols.t
    hover_series: list[tuple[str, str]] = []
    faltan: list[str] = []

    present = []
    for canon, label, ci in _ACCS:
        col = log.cols[canon]
        if col is None:
            faltan.append(label)
            continue
        add_series(fig, source, t, col, label, SERIES_PALETTE[ci % len(SERIES_PALETTE)])
        hover_series.append((col, label))
        present.append(col)

    # Modulo de la aceleracion si estan los tres ejes.
    if len(present) == 3:
        source.data["vib_acc_mag"] = accel_magnitude(
            log.df_plot[present[0]], log.df_plot[present[1]], log.df_plot[present[2]]
        )
        add_series(fig, source, t, "vib_acc_mag", "|a|",
                   SERIES_PALETTE[3 % len(SERIES_PALETTE)], width=2.0)
        hover_series.append(("vib_acc_mag", "|a|"))
        # Bandas de aviso sobre el modulo.
        fig.renderers.append(BoxAnnotation(
            bottom=VIBRATION_OK_MPS2, top=VIBRATION_WARN_MPS2,
            fill_color="orange", fill_alpha=0.06, line_width=0, level="underlay"))
        fig.renderers.append(BoxAnnotation(
            bottom=VIBRATION_WARN_MPS2,
            fill_color="red", fill_alpha=0.06, line_width=0, level="underlay"))

    if faltan:
        fig.title.text = f"Vibracion  (no disponible: {', '.join(faltan)})"
    if hover_series:
        add_hover(fig, t, hover_series)
    fig.legend.click_policy = "hide"
    fig.legend.location = "top_left"
    return fig
