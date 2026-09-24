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


def test_actitud_roll_pitch_presentes_yaw_no(synthetic_csv):
    log = load_log(synthetic_csv)
    fig = PREDEFINED[0].render(log, log_source(log), new_x_range(log))
    assert len(_lines(fig)) == 2  # roll (dphi) + pitch (dtheta)
    assert "no disponible" in fig.title.text
    assert "Yaw" in fig.title.text and "Roll" not in fig.title.text


def test_vibracion_tiene_modulo_y_bandas(synthetic_csv):
    log = load_log(synthetic_csv)
    fig = PREDEFINED[2].render(log, log_source(log), new_x_range(log))
    from bokeh.models import BoxAnnotation
    boxes = [r for r in fig.renderers if isinstance(r, BoxAnnotation)]
    # 3 bandas de modo (0->1->0) + 2 bandas de aviso
    assert len(boxes) == 5
    assert len(_lines(fig)) == 4  # acc x/y/z + |a|


def test_angular_rate_incluye_dp_dq_dr_dashed(synthetic_csv):
    log = load_log(synthetic_csv)
    fig = PREDEFINED[1].render(log, log_source(log), new_x_range(log))
    lineas = _lines(fig)
    assert len(lineas) == 6  # gyro x/y/z + dp/dq/dr
    discontinuas = [r for r in lineas if r.glyph.line_dash]
    assert len(discontinuas) == 3, "dp, dq y dr deben ir discontinuas"


def test_real_csv_render(real_csv):
    log = load_log(real_csv)
    models, _ = render_all(log)
    assert len(models) == 3
    assert all(_lines(m) for m in models)


# --------------------------------------------------------------------------
# Pestana Dinamica Lat-Dir
# --------------------------------------------------------------------------
from flightreview.plots import lat_dir_dynamics  # noqa: E402
from flightreview.plots.registry import PlotGroup  # noqa: E402


def _lat_dir(log):
    xr = new_x_range(log)
    return PlotGroup("Estados", lat_dir_dynamics.build).render(log, log_source(log), xr), xr


def test_lat_dir_cuatro_figuras(synthetic_csv):
    log = load_log(synthetic_csv)
    col, xr = _lat_dir(log)
    figs = col.children
    assert [f.title.text for f in figs] == [
        "Velocidad lateral (dv)", "Tasa de alabeo (dp)",
        "Tasa de guinada (dr)", "Angulo de alabeo (dphi)",
    ]
    for f in figs:
        assert len(_lines(f)) == 1
        assert any(isinstance(t, HoverTool) for t in f.tools)
        assert f.x_range is xr
        # 3 tramos de modo 0->1->0
        assert len([r for r in f.renderers if isinstance(r, BoxAnnotation)]) == 3


def test_lat_dir_sin_columnas_laterales(tmp_path, synthetic_df):
    # CSV longitudinal (p. ej. linea base previa a v0.2-lateral).
    p = tmp_path / "solo_lon.csv"
    synthetic_df.drop(columns=["dv_mps", "dp_radps", "dr_radps", "dphi_rad "]).to_csv(p, index=False)
    col, _ = _lat_dir(load_log(str(p)))
    for f in col.children:
        assert "(no disponible)" in f.title.text
        assert not _lines(f)
