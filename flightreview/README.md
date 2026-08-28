# flightreview

Visor interactivo de logs de vuelo en formato **CSV**, inspirado en
[Flight Review](https://logs.px4.io). Pensado para los CSV que genera el
simulador SIL de este repo (`data/SIL_sim_servos2.csv`), pero acepta cualquier
CSV con un eje temporal y resuelve los nombres de columna de forma flexible.

## Funcionalidades (MVP)

- Carga de CSV con **resolución flexible de columnas** y **caché** en parquet.
- Gráficas interactivas Bokeh: **zoom con scroll**, **pan** sobre el tiempo,
  **hover** con valor + timestamp; eje de tiempo **enlazado** entre todas.
- Gráficas predefinidas: **actitud**, **rapidez angular**, **vibración**.
- **Fondo de color según el modo de vuelo** (columna `mode`).
- Exportación a un **reporte HTML autocontenido** (offline, sin servidor).

## Uso

Ejecutar siempre desde la raíz del repositorio (para que `import flightreview`
resuelva y para encontrar `data/`).

```bash
# Servidor interactivo (abre el navegador)
bokeh serve --show flightreview/app.py

# Reporte HTML autocontenido
python -m flightreview.report data/SIL_sim_servos2.csv -o reporte.html

# Pruebas
pytest flightreview
```

Si `flightreview` no está en el path, exportar `PYTHONPATH=flightreview` o
instalar en modo editable: `pip install -e flightreview`.

## Estructura

```
flightreview/            (paquete importable: layout plano)
├── config.py            alias de columnas, tabla de modos, constantes
├── parser/              loader (CSV + caché), schema (columnas), flight_modes
├── plots/               base interactiva + actitud / rapidez angular / vibración
├── report.py            exportación a HTML autocontenido
├── app.py               servidor Bokeh
├── tests/
└── scripts/make_report.py  CLI de reporte
```

## Pendientes conocidos

- El simulador es longitudinal: de Roll/Pitch/Yaw solo hay pitch (`dtheta_rad`).
- Los setpoints de actitud aún no se registran en el CSV (trazas cableadas y
  vacías).
- `mode` es constante `0` en los datos actuales; el fondo por modo se ve como
  una sola banda hasta que el lado C emita modos distintos.
