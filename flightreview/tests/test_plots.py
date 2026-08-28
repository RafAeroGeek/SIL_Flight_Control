# -*- coding: utf-8 -*-
from bokeh.models import BoxAnnotation, Div, HoverTool, Plot, WheelZoomTool

from flightreview.parser.loader import load_log
from flightreview.parser.flight_modes import mode_intervals
from flightreview.plots.base import (
    add_hover,
    add_mode_background,
    log_source,
    mode_legend_div,
    new_x_range,
    time_figure,
)


def test_time_figure_tiene_zoom_scroll_y_pan():
    fig = time_figure("t", "y")
    assert isinstance(fig, Plot)
    wheels = [t for t in fig.tools if isinstance(t, WheelZoomTool)]
    assert wheels, "debe haber WheelZoomTool"
    assert fig.toolbar.active_scroll in wheels, "el scroll debe hacer zoom"
    assert any(t.__class__.__name__ == "PanTool" for t in fig.tools)


def test_time_figure_enlaza_x_range():
    f1 = time_figure("a", "y")
    f2 = time_figure("b", "y", x_range=f1.x_range)
    assert f2.x_range is f1.x_range


def test_add_hover_vline():
    fig = time_figure("t", "y")
    hover = add_hover(fig, "t_s", [("pitch_deg", "Pitch")])
    assert isinstance(hover, HoverTool)
    assert hover.mode == "vline"
    assert hover in fig.tools


def test_add_mode_background_pinta_bandas(synthetic_csv):
    log = load_log(synthetic_csv)
    fig = time_figure("t", "y")
    n_before = len(fig.renderers)
    add_mode_background(fig, log.mode_intervals)
    boxes = [r for r in fig.renderers if isinstance(r, BoxAnnotation)]
    # 3 tramos 0->1->0
    assert len(boxes) == 3
    assert len(fig.renderers) == n_before + 3


def test_mode_legend_div():
    div = mode_legend_div([])
    assert isinstance(div, Div) and "sin datos" in div.text

    import pandas as pd
    df = pd.DataFrame({"t_s": [0, 1, 2, 3], "mode": [0, 0, 1, 1]})
    div = mode_legend_div(mode_intervals(df, "t_s", "mode"))
    assert "MANUAL" in div.text and "GYRO_STAB" in div.text


def test_log_source_y_x_range(synthetic_csv):
    log = load_log(synthetic_csv)
    src = log_source(log)
    assert "dtheta_rad" in src.data
    xr = new_x_range(log)
    assert xr.start == 0.0
    assert xr.end > 0.5
