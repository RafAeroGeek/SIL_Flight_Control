# -*- coding: utf-8 -*-
"""Registro de graficas predefinidas.

``PREDEFINED`` es la unica fuente de verdad del conjunto de graficas: la usan
tanto el servidor (``app.py``) como el reporte HTML (``report.py``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from bokeh.models import Range1d

from flightreview.parser.loader import FlightLog
from flightreview.plots.base import add_mode_background, new_x_range

# Firma de un constructor de grafica: (log, source, x_range) -> modelo Bokeh.
BuildFn = Callable[..., object]


@dataclass(frozen=True)
class PlotGroup:
    """Una grafica predefinida con nombre y funcion constructora."""

    name: str
    build: BuildFn

    def render(self, log: FlightLog, source, x_range: Range1d):
        fig = self.build(log, source, x_range)
        # El fondo por modo se aplica de forma uniforme a toda figura del registro.
        for f in _iter_figures(fig):
            add_mode_background(f, log.mode_intervals)
        return fig


def _iter_figures(model):
    """Devuelve las figuras contenidas en ``model`` (figura suelta o layout)."""
    from bokeh.models import Plot

    if isinstance(model, Plot):
        return [model]
    children = getattr(model, "children", None)
    if not children:
        return []
    out = []
    for child in children:
        out.extend(_iter_figures(child))
    return out


from flightreview.plots import angular_rate, attitude, vibration  # noqa: E402

PREDEFINED: list[PlotGroup] = [
    PlotGroup("Actitud", attitude.build),
    PlotGroup("Rapidez angular", angular_rate.build),
    PlotGroup("Vibracion", vibration.build),
]


def render_all(log: FlightLog):
    """Construye todas las graficas de ``PREDEFINED`` con eje de tiempo enlazado.

    Devuelve (lista_de_modelos, x_range_compartido).
    """
    from flightreview.plots.base import log_source

    source = log_source(log)
    x_range = new_x_range(log)
    models = [g.render(log, source, x_range) for g in PREDEFINED]
    return models, x_range
