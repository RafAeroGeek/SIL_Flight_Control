# -*- coding: utf-8 -*-
import re

from flightreview.report import build_html, main, render_html_string

# Etiquetas que cargarian recursos externos al abrir el HTML.
_EXTERNAL_SCRIPT = re.compile(r'<script[^>]+src=["\']https?://', re.I)
_EXTERNAL_LINK = re.compile(r'<link[^>]+href=["\']https?://', re.I)


def _sin_recursos_externos(html: str) -> bool:
    # BokehJS lleva URLs de respaldo perezosas (MathJax, webfont de iconos) dentro
    # de su bundle; no se cargan para estas graficas. Lo que importa es que no
    # haya <script src> ni <link href> apuntando fuera.
    return not _EXTERNAL_SCRIPT.search(html) and not _EXTERNAL_LINK.search(html)


def test_render_html_string_autocontenido(synthetic_csv):
    html = render_html_string(synthetic_csv)
    assert "<script" in html and "Bokeh" in html
    assert "cdn.bokeh.org" not in html
    assert _sin_recursos_externos(html)
    assert len(html) > 50_000  # el JS de Bokeh va embebido


def test_build_html_escribe_archivo(synthetic_csv, tmp_path):
    out = tmp_path / "reporte.html"
    ruta = build_html(synthetic_csv, str(out))
    assert ruta == str(out)
    assert out.exists() and out.stat().st_size > 50_000
    texto = out.read_text(encoding="utf-8")
    assert "flightreview" in texto
    assert _sin_recursos_externos(texto)


def test_build_html_ruta_por_defecto(synthetic_csv):
    import os

    ruta = build_html(synthetic_csv)
    try:
        assert os.path.exists(ruta)
        assert ruta.endswith(".html")
        assert "reports" in ruta.replace("\\", "/")
    finally:
        if os.path.exists(ruta):
            os.remove(ruta)


def test_cli_main(synthetic_csv, tmp_path, capsys):
    out = tmp_path / "r.html"
    rc = main([synthetic_csv, "-o", str(out)])
    assert rc == 0
    assert out.exists()
    assert "reporte escrito en" in capsys.readouterr().out


def test_cli_main_csv_inexistente(capsys):
    rc = main(["no/existe.csv"])
    assert rc == 1
    assert "error" in capsys.readouterr().err
