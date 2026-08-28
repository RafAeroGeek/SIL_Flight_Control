# -*- coding: utf-8 -*-
"""Comprobaciones ligeras del servidor Bokeh (sin navegador)."""

from bokeh.models import FileInput


def test_app_construye_layout():
    import flightreview.app as app

    assert app.root.children, "el layout raiz debe tener hijos"
    assert any(isinstance(w, FileInput) for w in _walk(app.root))


def test_cargar_ruta_inexistente_muestra_error():
    import flightreview.app as app

    app.cargar_ruta("no/existe/log.csv")
    assert "Error" in app.estado.text


def test_cargar_ruta_valida(synthetic_csv):
    import flightreview.app as app

    app.cargar_ruta(synthetic_csv)
    assert app._estado_doc["log"] is not None
    assert app._estado_doc["log"].n_samples == 60
    # leyenda de modos + 3 graficas
    assert len(app.graficas.children) == 4


def _walk(model):
    yield model
    for child in getattr(model, "children", []) or []:
        yield from _walk(child)
