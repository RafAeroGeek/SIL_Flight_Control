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
    titulos = [panel.title for panel in app.tabs_container.tabs]
    assert titulos == [
        "Dinamica Longitudinal", "Dinamica Lat-Dir", "Control de Superficies",
        "Sensores e Inercial", "Resumen",
    ]
    for panel in app.tabs_container.tabs:
        assert panel.child.children  # cada pestana: leyenda + grafica(s)


def test_cargar_csv_sin_columnas_laterales(tmp_path, synthetic_df):
    import flightreview.app as app
    from bokeh.models import Plot

    # CSV longitudinal (p. ej. linea base previa a v0.2-lateral).
    p = tmp_path / "solo_lon.csv"
    synthetic_df.drop(columns=["dv_mps", "dp_radps", "dr_radps", "dphi_rad "]).to_csv(p, index=False)
    app.cargar_ruta(str(p))
    assert "Error" not in app.estado.text
    tabs = {panel.title: panel for panel in app.tabs_container.tabs}
    assert len(tabs) == 5
    for panel in tabs.values():
        assert panel.child.children
    lat_dir = [m for m in _walk(tabs["Dinamica Lat-Dir"].child) if isinstance(m, Plot)]
    assert len(lat_dir) == 4
    assert all("(no disponible)" in f.title.text for f in lat_dir)


def test_boton_export_genera_html(synthetic_csv, tmp_path, monkeypatch):
    import os

    import flightreview.app as app

    salidas = []
    monkeypatch.setattr(
        app, "build_html",
        lambda log: salidas.append(str(tmp_path / "r.html")) or _write(tmp_path / "r.html"),
    )
    app.cargar_ruta(synthetic_csv)
    app._on_export()
    assert "Reporte HTML escrito" in app.estado.text
    assert os.path.exists(tmp_path / "r.html")


def _write(p):
    p.write_text("<html>ok</html>", encoding="utf-8")
    return str(p)


def _walk(model):
    yield model
    for child in getattr(model, "children", []) or []:
        yield from _walk(child)
