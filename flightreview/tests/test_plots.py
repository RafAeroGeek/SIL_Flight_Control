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


# --------------------------------------------------------------------------
# Graficas predefinidas
# --------------------------------------------------------------------------
from bokeh.models import GlyphRenderer  # noqa: E402

from flightreview.plots.registry import PREDEFINED, render_all  # noqa: E402


def _lines(fig):
    return [r for r in fig.renderers if isinstance(r, GlyphRenderer)]


def test_predefined_son_las_tres():
    assert [g.name for g in PREDEFINED] == ["Actitud", "Rapidez angular", "Vibracion"]


def test_cada_builder_devuelve_figura_con_series_y_hover(synthetic_csv):
    log = load_log(synthetic_csv)
    models, xr = render_all(log)
    assert len(models) == 3
    for m in models:
        assert isinstance(m, Plot)
        assert _lines(m), "cada grafica debe tener al menos una serie"
        assert any(isinstance(t, HoverTool) for t in m.tools)
        # x_range compartido -> el mismo objeto en las tres
        assert m.x_range is xr


def test_actitud_pitch_presente_roll_yaw_no(synthetic_csv):
    log = load_log(synthetic_csv)
    fig = PREDEFINED[0].render(log, log_source(log), new_x_range(log))
    assert len(_lines(fig)) == 1  # solo pitch
    assert "no disponible" in fig.title.text
    assert "Roll" in fig.title.text and "Yaw" in fig.title.text


def test_vibracion_tiene_modulo_y_bandas(synthetic_csv):
    log = load_log(synthetic_csv)
    fig = PREDEFINED[2].render(log, log_source(log), new_x_range(log))
    from bokeh.models import BoxAnnotation
    boxes = [r for r in fig.renderers if isinstance(r, BoxAnnotation)]
    # 3 bandas de modo (0->1->0) + 2 bandas de aviso
    assert len(boxes) == 5
    assert len(_lines(fig)) == 4  # acc x/y/z + |a|


def test_angular_rate_incluye_dq_dashed(synthetic_csv):
    log = load_log(synthetic_csv)
    fig = PREDEFINED[1].render(log, log_source(log), new_x_range(log))
    dashes = {tuple(r.glyph.line_dash) if r.glyph.line_dash else () for r in _lines(fig)}
    assert len(_lines(fig)) == 4  # gyro x/y/z + dq
    assert any(d for d in dashes), "dq debe ir discontinua"


def test_real_csv_render(real_csv):
    log = load_log(real_csv)
    models, _ = render_all(log)
    assert len(models) == 3
    assert all(_lines(m) for m in models)
