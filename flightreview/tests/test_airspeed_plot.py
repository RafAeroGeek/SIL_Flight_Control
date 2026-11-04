# -*- coding: utf-8 -*-
"""Sensores / Airspeed: IAS cruda y filtrada."""

from bokeh.models import GlyphRenderer

from flightreview.parser.loader import load_log
from flightreview.plots import sensor_airspeed
from flightreview.plots.base import log_source, new_x_range


def _fig(log):
    return sensor_airspeed.build(log, log_source(log), new_x_range(log))


def _lineas(fig):
    return [r for r in fig.renderers if isinstance(r, GlyphRenderer)]


def _leyenda(fig):
    return [item.label.value for item in fig.legend[0].items]


def _csv(tmp_path, synthetic_df, fc=5.0):
    df = synthetic_df.copy()
    df["pitot_ms"] = 24.0
    df["pitot_ms_filtered"] = 24.5
    df["pitot_fc_hz"] = fc
    p = tmp_path / "con_filtro.csv"
    df.to_csv(p, index=False)
    return str(p)


def test_superpone_filtrada_con_su_corte(tmp_path, synthetic_df):
    fig = _fig(load_log(_csv(tmp_path, synthetic_df)))
    assert len(_lineas(fig)) == 2
    assert _leyenda(fig) == ["Pitot", "Filtrada (fc = 5 Hz)"]


def test_fc_cero_es_sin_filtro(tmp_path, synthetic_df):
    fig = _fig(load_log(_csv(tmp_path, synthetic_df, fc=0.0)))
    assert _leyenda(fig)[1] == "Filtrada (sin filtro)"


def test_log_viejo_solo_muestra_el_pitot(synthetic_csv):
    fig = _fig(load_log(synthetic_csv))
    assert _leyenda(fig) == ["Pitot"]
