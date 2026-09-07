# -*- coding: utf-8 -*-
"""Exportacion del analisis a un reporte HTML autocontenido.

Usa recursos ``INLINE`` de Bokeh: el HTML resultante no depende de internet ni
de un servidor, se puede enviar por correo o abrir directamente.

Uso como CLI:

    python -m flightreview.report data/SIL_sim_servos2.csv -o reporte.html
"""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import sys

from bokeh.embed import file_html
from bokeh.layouts import column
from bokeh.models import Div
from bokeh.resources import INLINE

from flightreview.parser.loader import FlightLog, load_log
from flightreview.plots.base import mode_legend_div
from flightreview.plots.registry import render_all

# Carpeta por defecto para los reportes generados (ignorada por git).
REPORTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")


def _as_log(log_or_path: FlightLog | str, use_cache: bool = True) -> FlightLog:
    if isinstance(log_or_path, FlightLog):
        return log_or_path
    return load_log(str(log_or_path), use_cache=use_cache)


def build_layout(log: FlightLog):
    """Layout Bokeh con cabecera, leyenda de modos y las graficas predefinidas."""
    modelos, _xr = render_all(log)
    cabecera = Div(text=(
        f"<h2>flightreview &mdash; {os.path.basename(log.path)}</h2>"
        f"<p>{log.n_samples} muestras &middot; {log.duration_s:.2f} s &middot; "
        f"{log.sample_rate_hz:.0f} Hz &middot; "
        f"generado {_dt.datetime.now():%Y-%m-%d %H:%M}</p>"
    ))
    return column(cabecera, mode_legend_div(log.mode_intervals), *modelos,
                  sizing_mode="stretch_width")


def render_html_string(log_or_path: FlightLog | str) -> str:
    """Devuelve el HTML del reporte como cadena (recursos INLINE)."""
    log = _as_log(log_or_path)
    layout = build_layout(log)
    return file_html(layout, resources=INLINE,
                     title=f"flightreview - {os.path.basename(log.path)}")


def _default_out(log: FlightLog) -> str:
    stem = os.path.splitext(os.path.basename(log.path))[0]
    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(REPORTS_DIR, f"{stem}_{ts}.html")


def build_html(log_or_path: FlightLog | str, out_path: str | None = None,
               use_cache: bool = True) -> str:
    """Genera el reporte y lo escribe en disco. Devuelve la ruta escrita."""
    log = _as_log(log_or_path, use_cache=use_cache)
    out_path = out_path or _default_out(log)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    html = file_html(build_layout(log), resources=INLINE,
                     title=f"flightreview - {os.path.basename(log.path)}")
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(html)
    return out_path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="flightreview.report",
        description="Genera un reporte HTML autocontenido de un log de vuelo CSV.",
    )
    ap.add_argument("csv", help="ruta al archivo .csv del log")
    ap.add_argument("-o", "--out", help="ruta del HTML de salida "
                    "(por defecto flightreview/reports/<log>_<fecha>.html)")
    ap.add_argument("--no-cache", action="store_true",
                    help="no usar la cache parquet al parsear")
    args = ap.parse_args(argv)

    try:
        out = build_html(args.csv, args.out, use_cache=not args.no_cache)
    except Exception as exc:  # noqa: BLE001
        print(f"error: {exc}", file=sys.stderr)
        return 1
    print(f"reporte escrito en: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
