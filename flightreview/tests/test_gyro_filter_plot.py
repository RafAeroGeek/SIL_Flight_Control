# -*- coding: utf-8 -*-
"""Sensores / Filtro giros: crudo, filtrado y estado por eje."""

import numpy as np
from bokeh.models import GlyphRenderer

from flightreview.parser.loader import load_log
from flightreview.plots import sensor_gyro_filter
from flightreview.plots.base import log_source, new_x_range
from flightreview.plots.registry import build_tab_groups


def _figs(log):
    return sensor_gyro_filter.build(log, log_source(log), new_x_range(log)).children


def _lineas(fig):
    return [r for r in fig.renderers if isinstance(r, GlyphRenderer)]


def _leyenda(fig):
    return [item.label.value for item in fig.legend[0].items]


def _csv_con_filtro(tmp_path, synthetic_df, fc=20.0, quitar=()):
    df = synthetic_df.copy()
    ruido = 0.02 * (-1.0) ** np.arange(len(df))
    for eje, estado in (("x", "dp_radps"), ("y", "dq_radps"), ("z", "dr_radps")):
        df[f"gyro_{eje}_radps"] = df[estado] + ruido
        df[f"gyro_{eje}_radps_filtered"] = df[estado] + 0.5 * ruido
        df[f"gyro_{eje}_fc_hz"] = fc
    p = tmp_path / "con_filtro.csv"
    df.drop(columns=list(quitar)).to_csv(p, index=False)
    return str(p)


def test_una_figura_por_eje_con_tiempo_enlazado(tmp_path, synthetic_df):
    figs = _figs(load_log(_csv_con_filtro(tmp_path, synthetic_df)))
    assert len(figs) == 3
    for fig in figs:
        lineas = _lineas(fig)
        assert len(lineas) == 3  # crudo, filtrado, estado
        assert len([r for r in lineas if r.glyph.line_dash]) == 1
    assert figs[0].x_range is figs[2].x_range


def test_leyenda_con_fc_y_sigma_sin_contar_el_estado(tmp_path, synthetic_df):
    # Ruido de 0.02 rad/s (1.15 deg/s), reducido a la mitad por el "filtro"
    etiqueta = _leyenda(_figs(load_log(_csv_con_filtro(tmp_path, synthetic_df)))[0])[1]
    assert "fc = 20 Hz" in etiqueta
    assert "σ 1.15 → 0.57 deg/s" in etiqueta
    assert "senal completa" not in etiqueta


def test_fc_cero_es_sin_filtro(tmp_path, synthetic_df):
    log = load_log(_csv_con_filtro(tmp_path, synthetic_df, fc=0.0))
    assert "sin filtro" in _leyenda(_figs(log)[0])[1]


def test_sin_estado_usa_senal_completa(tmp_path, synthetic_df):
    log = load_log(_csv_con_filtro(tmp_path, synthetic_df, quitar=["dp_radps"]))
    assert "senal completa" in _leyenda(_figs(log)[0])[1]


def test_log_viejo_no_falla(synthetic_csv):
    figs = _figs(load_log(synthetic_csv))
    assert len(figs) == 3
    assert "no disponible" in figs[0].title.text
    assert len(_lineas(figs[0])) == 2  # crudo y estado


def test_esta_en_la_pestana_de_sensores():
    pestanas = {t.name: t for t in build_tab_groups()}
    assert "Filtro giros" in [g.name for g in pestanas["Sensores e Inercial"].plot_groups]
