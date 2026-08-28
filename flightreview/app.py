# -*- coding: utf-8 -*-
"""Servidor Bokeh del visor de logs de vuelo.

Ejecutar desde la raiz del repositorio:

    bokeh serve --show flightreview/app.py

Carga un CSV (selector del navegador o ruta local), muestra las graficas
predefinidas con el eje de tiempo enlazado y el fondo coloreado por modo de
vuelo.
"""

from __future__ import annotations

import base64
import os
import sys
import tempfile
import traceback

# `bokeh serve flightreview/app.py` ejecuta este archivo sin la raiz del repo en
# sys.path, asi que el paquete `flightreview` no seria importable. Se anade aqui.
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.models import Button, Div, FileInput, TextInput

from flightreview.parser.loader import load_log
from flightreview.plots.base import mode_legend_div
from flightreview.plots.registry import render_all
from flightreview.report import build_html

# Ruta candidata para autocarga al arrancar (relativa a la raiz del repo).
DEFAULT_LOG = os.path.join("data", "SIL_sim_servos2.csv")

# --------------------------------------------------------------------------
# Widgets
# --------------------------------------------------------------------------
titulo = Div(text="<h2>flightreview &mdash; visor de logs de vuelo</h2>")
estado = Div(text="Sin log cargado.")
file_input = FileInput(accept=".csv", title="Cargar CSV")
ruta_input = TextInput(title="...o ruta local", placeholder=DEFAULT_LOG, width=420)
ruta_btn = Button(label="Cargar ruta", button_type="primary", width=110)
export_btn = Button(label="Exportar reporte HTML", button_type="success", width=180)

graficas = column(sizing_mode="stretch_width")

root = column(
    titulo,
    row(file_input, ruta_input, ruta_btn),
    row(export_btn),
    estado,
    graficas,
    sizing_mode="stretch_width",
)

# Estado del documento (log actual). Se guarda para que report.py lo reutilice.
_estado_doc: dict[str, object] = {"log": None}


# --------------------------------------------------------------------------
# Logica
# --------------------------------------------------------------------------
def _mostrar_log(log) -> None:
    _estado_doc["log"] = log
    modelos, _xr = render_all(log)
    graficas.children = [mode_legend_div(log.mode_intervals), *modelos]
    estado.text = (
        f"<b>{os.path.basename(log.path)}</b> &mdash; "
        f"{log.n_samples} muestras, {log.duration_s:.2f} s, "
        f"{log.sample_rate_hz:.0f} Hz, "
        f"{len(log.mode_intervals)} tramo(s) de modo"
    )


def cargar_ruta(path: str) -> None:
    path = (path or "").strip()
    if not path:
        estado.text = "Indica una ruta o usa el selector de archivo."
        return
    try:
        _mostrar_log(load_log(path))
    except Exception as exc:  # noqa: BLE001 - mostrar cualquier fallo al usuario
        estado.text = f"<span style='color:#c00'>Error cargando {path}: {exc}</span>"
        traceback.print_exc()


def _on_file_input(attr, old, new) -> None:
    if not new:
        return
    try:
        raw = base64.b64decode(new)
        suffix = os.path.splitext(file_input.filename or "log.csv")[1] or ".csv"
        with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as fh:
            fh.write(raw)
            tmp = fh.name
        log = load_log(tmp, use_cache=False)
        # Reetiquetar con el nombre original para la UI.
        log.path = os.path.abspath(file_input.filename or tmp)
        _mostrar_log(log)
    except Exception as exc:  # noqa: BLE001
        estado.text = f"<span style='color:#c00'>Error leyendo el archivo: {exc}</span>"
        traceback.print_exc()


def _on_export() -> None:
    log = _estado_doc["log"]
    if log is None:
        estado.text = "Carga un log antes de exportar."
        return
    try:
        out = build_html(log)
    except Exception as exc:  # noqa: BLE001
        estado.text = f"<span style='color:#c00'>Error exportando: {exc}</span>"
        traceback.print_exc()
        return
    estado.text = f"Reporte HTML escrito en <code>{out}</code>"


file_input.on_change("value", _on_file_input)
ruta_btn.on_click(lambda: cargar_ruta(ruta_input.value or DEFAULT_LOG))
export_btn.on_click(_on_export)

# Autocarga al arrancar si el CSV por defecto existe.
if os.path.exists(DEFAULT_LOG):
    cargar_ruta(DEFAULT_LOG)

curdoc().add_root(root)
curdoc().title = "flightreview"


if __name__ == "__main__":
    print("Ejecuta:  bokeh serve --show flightreview/app.py")
