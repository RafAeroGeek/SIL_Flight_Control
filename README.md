# SIL_Flight_Control

Simulador SIL (Software-In-the-Loop) de control de vuelo: dinámica longitudinal
y lateral-direccional de aeronave en espacio de estados (lineal, desacoplada),
integrada con RK4, con simulación de servos, sensores y un scheduler
cooperativo que imita los periodos de tarea de un RTOS.

Vectores de estado (ejes cuerpo):
- Longitudinal: `X_lon = [du, dw, dq, dtheta]`, entrada `delta_e`.
- Lateral-direccional: `X_lat = [dv, dp, dr, dphi]`, entradas `[delta_a, delta_r]`.

Aeronave activa: `aircraft/ugly_stick.json` (ver `AIRCRAFT_JSON_PATH`).

Las derivadas **longitudinales y laterales** de `ugly_stick.json` salen de
`dynamics/` (estimación con Nelson a H = 1000 m, V0 = 25.25 m/s, m = 20 kg,
vuelo nivelado, pasadas a ejes cuerpo; ver
[Derivadas de estabilidad](#derivadas-de-estabilidad-dynamics)).

> ⚠️ Las de `aircraft_a.json` siguen siendo **placeholders del Navion**
> (`"lateral_source": "PLACEHOLDER_NAVION_Nelson"`).

## Estructura

```
SIL_Flight_Control/
├── CMakeLists.txt      Configuración de compilación
├── src/                Código C (10 módulos)
├── python/             Scripts de análisis y graficación
├── dynamics/           Estimación de derivadas lateral-direccionales (Python)
├── flightreview/       Visor interactivo de logs (servidor Bokeh + reporte HTML)
└── data/               CSV y HTML generados (ignorados por git)
```

### Módulos en `src/`

| Archivo                | Rol                                                   |
|------------------------|-------------------------------------------------------|
| `main.c`               | Entrada, registro de tareas, logging del CSV          |
| `sim_task_management`  | Scheduler cooperativo 1 / 5 / 30 / 150 / 500 ms       |
| `pilot_sim`            | Piloto simulado: escalones, mapeo a canales RC        |
| `servo_sim`            | Servos de 1er orden con RK4 y limitador de velocidad  |
| `flight_sim`           | Planta: estado, actuadores, paso de integración       |
| `dynamics`             | Matrices A/B lon y lat, RK4 (4x1 y n/m), señales      |
| `aircraft_loader`      | Carga parámetros y config de servos desde el JSON     |
| `sensors_sim`          | Pitot, veleta (α, β), IMU, GPS y láser, ruido y bias  |
| `atmosphere`           | Densidad ISA (troposfera), igual a `dynamics/`        |
| `flight_management`    | Capa de control (**tipos definidos, sin implementar**)|
| `sim_PWM_processing`   | Banco de canales PWM (compilado, aún sin usar)        |
| `sim_settings.h`       | Selección de plataforma y tiempo de simulación        |

## Compilar y ejecutar

Requisitos: `sudo apt install build-essential cmake`

```bash
cmake -S . -B build            # configurar (una sola vez)
cmake --build build           # compilar
./build/SIL_Flight_Control    # ejecutar DESDE LA RAÍZ del proyecto
```

El ejecutable escribe `data/SIL_sim_servos2.csv` con una ruta **relativa**, así
que hay que lanzarlo desde la raíz del proyecto o el `fopen` falla.

Para recompilar tras editar código basta con `cmake --build build`.
Para una compilación optimizada:

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release
```

## Modelo longitudinal

Nelson cap. 3–4, en **ejes cuerpo** (`u0 = u0_ms`, `w0 = w0_ms`; con
`w0 = 0` y `X_de = 0` queda la forma de Nelson en ejes de estabilidad):

```
      | X_u    X_w    -w0     X_θ |          | X_de |
  A = | Ẑ_u    Ẑ_w     u0     Z_θ |      B = | Z_de |
      | M̂_u    M̂_w    M̂_q     0   |          | M_de |
      | 0      0      1       0   |          | 0    |
```

`X_θ = −g·cos θ0`, `Z_θ = −g·sin θ0`; `X_de` (m/s², opcional, default 0) es la
componente en X del elevador al girar a ejes cuerpo.

## Modelo lateral-direccional

Nelson, *Flight Stability and Automatic Control*, cap. 5. Pequeñas perturbaciones
alrededor de vuelo recto y nivelado, en **ejes cuerpo**:

```
x = [dv, dp, dr, dphi, dpsi]      u = [da, dr]

      | Y_v   Y_p + w0   -(u0 - Y_r)   g·cos(θ0)  0 |          |  0     Y_dr |
      | L_v   L_p         L_r           0          0 |          | L_da   L_dr |
  A = | N_v   N_p         N_r           0          0 |      B = | N_da   N_dr |
      | 0     1           tan(θ0)       0          0 |          |  0     0    |
      | 0     0           sec(θ0)       0          0 |          |  0     0    |
```

`dpsi` (rumbo, ψ0 = 0 hacia el Norte) es cinemática pura: no realimenta, así
que agrega un eigenvalor en 0 y deja los demás modos igual.

`build_lateral_matrices` (`src/dynamics.c`) aplica además la corrección por
producto de inercia: con `D = 1 - I_xz²/(I_x·I_z)`, las filas L y N usan
`L*x + (I_xz/I_x)·N*x` y `N*x + (I_xz/I_z)·L*x` (`L*x = L_x/D`, `N*x = N_x/D`).

Supuestos: canales desacoplados (integrar el lateral no cambia el estado
longitudinal). Los sensores usan los mismos ejes: `w = w0 + dw` y
`θ = θ0 + dθ`. El GPS pasa (u, v, w) a NED con la matriz de rotación completa
(φ, θ, ψ), así que `gps_ve_ms` refleja el rumbo y la altitud integrada siente
el alabeo a segundo orden.

**Trim de superficies.** Los servos dan la deflexión absoluta; la planta
integra la perturbación `dδ = δ − δ_trim` (`FlightSim_SetActuatorsFromFM`,
`src/flight_sim.c`). Para que con el stick al centro la superficie quede en
trim, `main.c` suma al comando de cada servo un trim de transmisor
`δ_trim / k_deg_per_us` [µs] y arranca el servo en `δ_trim` (en lugar de
`servo_*_y0_deg`). En el Ugly Stick: δe_trim = −0.49°, k = 0.04 → −12.2 µs.

### Claves JSON nuevas (`aircraft/*.json`)

| Clave | Unidad | Nota |
|---|---|---|
| `theta0_deg` | deg | cabeceo de trim (γ0 + atan2(w0, u0)) |
| `X_de` | m/s² | longitudinal, opcional (default 0) |
| `delta_e_trim_deg` | deg | elevador de trim, absoluto (lo escribe `a_aircraft.py`) |
| `I_x`, `I_z`, `I_xz` | kg·m² | inercias (default `I_x = I_z = 1`) |
| `Y_v` | 1/s | fuerza lateral |
| `Y_p`, `Y_r` | m/s | fuerza lateral |
| `Y_dr` | m/s² | fuerza lateral (no hay `Y_da`: es 0) |
| `L_v`, `N_v` | 1/(m·s) | |
| `L_p`, `L_r`, `N_p`, `N_r` | 1/s | |
| `L_da`, `L_dr`, `N_da`, `N_dr` | 1/s² | superficies en rad |

`L_x`, `N_x` van **sin asterisco** (`Q·S·b·C/I`); la corrección I_xz se hace en C.
`u0` sale de la clave existente `u0_ms`. Si falta alguna clave lateral, el
loader avisa por stderr y deja el default.

## Tests

Tests unitarios en C (opt-in, no cambian el build por defecto):

```bash
cmake -S . -B build -DSIL_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

### Regresión longitudinal contra una línea base

Los canales longitudinal y lateral están desacoplados, así que cambios en el
lateral (o en sensores laterales) no deben mover ni un bit de las columnas
longitudinales del CSV. `test/compare_csv_cols.py` compara columnas **como
texto** entre dos corridas y sale con código 1 si hay diferencias (reporta la
primera fila distinta). No está en ctest porque depende de una línea base local
(`data/*.csv` no se trackea).

```bash
# 1) Generar la línea base en el commit de referencia
git checkout <commit_base>
cmake --build build && ./build/SIL_Flight_Control > /dev/null
cp data/SIL_sim_servos2.csv data/baseline_<commit_base>.csv

# 2) Volver a la rama, compilar y correr
git checkout <rama>
cmake --build build && ./build/SIL_Flight_Control > /dev/null

# 3) Comparar
python3 test/compare_csv_cols.py data/baseline_<commit_base>.csv data/SIL_sim_servos2.csv \
    --cols t_s,du_mps,dw_mps,dq_radps,dtheta_rad,y_elev_deg,AoA_deg,gyro_y_radps,acc_x_mps2
```

La línea base puede generarse con `ROUTINE_LONGITUDINAL` y la comparación
correrse con `ROUTINE_LAT_DIR`: ambas rutinas tienen el mismo `pitch`, así que el
estado longitudinal debe coincidir. **No** entran en esa comparación las
columnas del pitot (`pitot_ms`, `tas_ms`, `dp_pa`: V = √(u²+v²+w²) incluye el
`v` lateral) ni las de altitud (`gps_alt_m`, `laser_alt_m`: V_D depende de φ y
v a segundo orden). Cada sensor tiene su propia secuencia de ruido, así que agregar o
quitar ruido en uno no cambia las columnas de los demás.

La línea base vigente es `2220e20` (pitot con Δp e ISA, ruido de veleta,
secuencia de ruido por sensor); las anteriores (`e041d25`, `5ea17b6`, …) ya no
aplican.

## Documentación teórica (`docs/teoria/`)

La teoría de los modelos va en LaTeX, un documento por tema (ver
`docs/teoria/README.md`). Hoy: `docs/teoria/sensores/ruido_sensores.tex`
(pitot, veleta, ISA y generador de ruido). Compilar con
`cd docs/teoria/sensores && latexmk -pdf ruido_sensores.tex`.

## Derivadas de estabilidad (`dynamics/`)

Una sola entrada, `dynamics/ugly_data.json` (condición de vuelo, geometría,
aerodinámica e inercias), alimenta los dos análisis, ambos en **ejes de
estabilidad** y con atmósfera ISA (`dynamics/atmosfera.py`):

- `lon_analysis.py` (Nelson cap. 3–4): trim (α, δe), derivadas con las "hat"
  por `M_ẇ`, A/B `[du, dw, dq, dθ]` y modos → `ugly_lon_resultados.json`.
  `--plots` muestra respuesta del periodo corto, envolvente y lugar de raíces
  (requiere matplotlib y python-control).
- `latdir_analysis.py` (Nelson cap. 5; posición del ala con DATCOM 5.2.1.1)
  → `ugly_latdir_resultados.json`.

Inercias con radios de giro de Roskam: `I_x`, `I_z` (latdir) e
`I_y = m·(R̄y·l_f)²/4` con R̄y = 0.38 (lon), todas de ejes cuerpo.

`dynamics/a_aircraft.py` lleva ambos resultados al JSON de la aeronave
**girándolos a ejes cuerpo** con α = atan2(w0, u0): semejanza `P·A·P⁻¹` en el
longitudinal y giro de momentos y de (p, r) en el lateral (con β → dv:
`Y_v = Yβ/V0`, etc.). Se niega si V0, masa, ρ o altitud no coinciden.

```bash
python3 dynamics/lon_analysis.py dynamics/ugly_data.json
python3 dynamics/latdir_analysis.py dynamics/ugly_data.json
python3 dynamics/a_aircraft.py dynamics/ugly_lon_resultados.json \
    dynamics/ugly_latdir_resultados.json aircraft/ugly_stick.json   # --dry-run
pytest dynamics
```

Modos del Ugly Stick (eigenvalores exactos; iguales en ambos ejes):

| Modo | λ | ωn [rad/s] | ζ |
|---|---|---|---|
| Periodo corto | −4.21 ± 3.09j | 5.22 | 0.81 |
| Fugoide | −0.017 ± 0.413j | 0.41 | 0.041 |
| Roll | −13.3 | — | — |
| Dutch roll | −0.413 ± 4.10j | 4.12 | 0.10 |
| Espiral | **+0.075** (duplica en 9.3 s) | — | — |

Trim: α = θ0 = 5.92°, δe = −0.49° (no se escribe en la aeronave: el C toma el
elevador absoluto como perturbación). La espiral inestable viene de un `Cnr`
débil frente a `Clr`; las aproximaciones de orden reducido de latdir no sirven
para ella (`Lβ ≈ 0`). Son estimaciones teóricas: inercias y factores empíricos
son la mayor incertidumbre.

## Graficar los resultados

Desde Spyder (Anaconda), abrir y ejecutar `python/plot_state_exp_v3.py`, que es
el script más completo y actualizado. Resuelve la ruta del CSV a partir de su
propia ubicación, así que funciona sin importar el directorio de trabajo.

Requiere `pandas`, `numpy`, `matplotlib` y `plotly`.

## Visor interactivo (`flightreview/`)

Aplicación aparte, en `flightreview/`, para analizar el CSV de forma interactiva
al estilo [Flight Review](https://logs.px4.io): servidor Bokeh con zoom (rueda),
pan y hover; gráficas predefinidas de actitud, rapidez angular y vibración;
fondo coloreado según el modo de vuelo; y exportación a un reporte HTML
autocontenido.

```bash
# Desde la raíz del repo
bokeh serve --show flightreview/app.py                          # servidor
python -m flightreview.report data/SIL_sim_servos2.csv -o r.html # reporte HTML
pytest flightreview                                              # pruebas
```

Detalles y pendientes en `flightreview/README.md`.

## Configuración de la simulación

En `src/sim_settings.h`:

- `SIMULATION_TIME_s` — duración simulada (por defecto 10 s)
- `SYSTEM_SIM_ENV` — `SIM_PLATFORM_PC` (con `printf` y CSV) o `SIM_PLATFORM_RTOS`
- `AIRCRAFT_SIM` — aeronave seleccionada
- `AIRCRAFT_JSON_PATH` — JSON de la aeronave activa (relativo a la raíz del repo)
- `SIL_CONFIG_LATERAL` — `1` integra el canal lateral-direccional (por defecto),
  `0` solo longitudinal (las columnas laterales del CSV quedan en 0)
- `SIM_PILOT_ROUTINE` — rutina del piloto simulado. Por defecto
  `ROUTINE_LAT_DIR`: el mismo `pitch` que `ROUTINE_LONGITUDINAL` (doblete de
  ±0.25 de stick, 1–3 s) más un doblete
  de alerón (±1.8°, 1–3 s) y uno de timón (±2.7°, 5–7 s; pedal derecho
  primero, que con la convención de Nelson es δr < 0). `ROUTINE_LONGITUDINAL`
  satura el alerón a 20° y queda solo para la regresión longitudinal.

El CSV agrega al final las columnas `dv_mps, dp_radps, dr_radps, dphi_rad`.

## Pendientes conocidos

- `flight_management.c` está prácticamente vacío: `FM_Init`, `FM_Update` y los
  tres tiers de control están declarados pero no implementados. Las tareas de
  5, 30, 150 y 500 ms en `main.c` tampoco hacen nada todavía.
- El throttle no realimenta la planta, y su `tau`/`rate_limit` de ESC en
  `servo_sim.c` son valores propuestos sin datos de banco.
- `sim_PWM_processing` se compila pero nadie lo llama.
