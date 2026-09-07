# -*- coding: utf-8 -*-
"""Primitivas de graficado: figura temporal con zoom/pan/hover y fondo por modo.

Todas las figuras predefinidas se construyen encima de ``time_figure`` y
comparten el mismo ``x_range`` para que el eje de tiempo se mueva enlazado
(comportamiento de Flight Review).
"""

from __future__ import annotations

from bokeh.models import (
    BoxAnnotation,
    ColumnDataSource,
    CrosshairTool,
    Div,
    HoverTool,
    Range1d,
    WheelZoomTool,
)
from bokeh.palettes import Category10_10
from bokeh.plotting import figure

from flightreview.parser.flight_modes import ModeInterval
from flightreview.parser.loader import FlightLog

# Paleta estable para las series de una figura.
SERIES_PALETTE = list(Category10_10)


def time_figure(
    title: str,
    y_axis_label: str,
    x_range: Range1d | None = None,
    height: int = 260,
):
    """Crea una figura temporal con las herramientas de interaccion listas.

    - rueda del raton = zoom (``active_scroll``)
    - arrastrar = pan
    - hover con linea vertical y crosshair
    - si se pasa ``x_range``, se reutiliza para enlazar el eje de tiempo.
    """
    kwargs = {}
    if x_range is not None:
        kwargs["x_range"] = x_range

    fig = figure(
        title=title,
        height=height,
        sizing_mode="stretch_width",
        tools="pan,box_zoom,reset,save",
        x_axis_label="Tiempo [s]",
        y_axis_label=y_axis_label,
        **kwargs,
    )

    wheel = WheelZoomTool(dimensions="both")
    fig.add_tools(wheel, CrosshairTool(dimensions="height"))
    fig.toolbar.active_scroll = wheel
    fig.toolbar.logo = None
    return fig


def add_hover(fig, t_field: str, series: list[tuple[str, str]]) -> HoverTool:
    """Anade un HoverTool en modo linea vertical.

    ``series`` es una lista de (campo_en_source, etiqueta).
    """
    tooltips = [("t", f"@{{{t_field}}}{{0.000}} s")]
    tooltips += [(label, f"@{{{field}}}{{0.000}}") for field, label in series]
    hover = HoverTool(tooltips=tooltips, mode="vline")
    fig.add_tools(hover)
    return hover


def add_series(fig, source: ColumnDataSource, t_field: str, y_field: str,
               label: str, color: str, dash: str = "solid", width: float = 1.5):
    """Dibuja una serie temporal desde ``source`` con entrada de leyenda."""
    return fig.line(
        x=t_field, y=y_field, source=source,
        line_color=color, line_width=width, line_dash=dash,
        legend_label=label,
    )


def add_mode_background(fig, intervals: list[ModeInterval], alpha: float = 0.12) -> None:
    """Pinta una banda de color de fondo por cada tramo de modo de vuelo.

    Se anaden como renderers en nivel ``underlay`` para que queden por debajo de
    las series.
    """
    for iv in intervals:
        fig.renderers.append(
            BoxAnnotation(
                left=iv.t0, right=iv.t1,
                fill_color=iv.color, fill_alpha=alpha,
                line_width=0, level="underlay",
            )
        )


def mode_legend_div(intervals: list[ModeInterval]) -> Div:
    """Leyenda HTML (color -> nombre de modo), sin repetir codigos."""
    seen: dict[int, tuple[str, str]] = {}
    for iv in intervals:
        seen.setdefault(iv.code, (iv.name, iv.color))
    if not seen:
        return Div(text="<b>Modo de vuelo:</b> sin datos de modo")
    chips = "".join(
        f'<span style="display:inline-block;margin-right:10px">'
        f'<span style="display:inline-block;width:12px;height:12px;'
        f'background:{color};border:1px solid #888;vertical-align:middle"></span> '
        f"{name}</span>"
        for name, color in seen.values()
    )
    return Div(text=f"<b>Modo de vuelo:</b> {chips}")


def log_source(log: FlightLog) -> ColumnDataSource:
    """ColumnDataSource con los datos (posiblemente decimados) del log.

    Las graficas pueden anadir columnas derivadas (p. ej. grados) con
    ``source.data['pitch_deg'] = ...``.
    """
    return ColumnDataSource(log.df_plot)


def new_x_range(log: FlightLog) -> Range1d:
    """Rango temporal inicial que compartiran todas las figuras."""
    t = log.df_plot[log.cols.t]
    return Range1d(start=float(t.iloc[0]), end=float(t.iloc[-1]))
