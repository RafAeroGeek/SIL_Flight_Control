# SIL_Flight_Control

Simulador SIL (Software-In-the-Loop) de control de vuelo: dinámica longitudinal
y lateral-direccional de aeronave en espacio de estados (lineal, desacoplada),
integrada con RK4, con simulación de servos, sensores y un scheduler
cooperativo que imita los periodos de tarea de un RTOS.

Vectores de estado:
- Longitudinal: `X_lon = [du, dw, dq, dtheta]`, entrada `delta_e`.
- Lateral-direccional: `X_lat = [dv, dp, dr, dphi]`, entradas `[delta_a, delta_r]`.

Aeronave activa: `aircraft/ugly_stick.json` (ver `AIRCRAFT_JSON_PATH`).

> ⚠️ Las derivadas **laterales** de `ugly_stick.json` y `aircraft_a.json` son
> **placeholders del Navion** (Nelson), no las de esas aeronaves
> (`"lateral_source": "PLACEHOLDER_NAVION_Nelson"`).

## Estructura

```
SIL_Flight_Control/
├── CMakeLists.txt      Configuración de compilación
├── src/                Código C (10 módulos)
├── python/             Scripts de análisis y graficación
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
| `dynamic_models`       | Catálogo de modelos (`aircraft_a`, `aircraft_b`)      |
| `sensors_sim`          | Pitot, veleta (α, β), IMU, GPS y láser, ruido y bias  |
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

## Modelo lateral-direccional

Nelson, *Flight Stability and Automatic Control*, cap. 5. Pequeñas perturbaciones
alrededor de vuelo recto y nivelado, en **ejes de estabilidad**:

```
      | Y_v   Y_p   -(u0 - Y_r)   g·cos(θ0) |          |  0     Y_dr |
  A = | L_v   L_p    L_r           0        |      B = | L_da   L_dr |
      | N_v   N_p    N_r           0        |          | N_da   N_dr |
      | 0     1      0             0        |          |  0     0    |
```

`build_lateral_matrices` (`src/dynamics.c`) aplica además la corrección por
producto de inercia: con `D = 1 - I_xz²/(I_x·I_z)`, las filas L y N usan
`L*x + (I_xz/I_x)·N*x` y `N*x + (I_xz/I_z)·L*x` (`L*x = L_x/D`, `N*x = N_x/D`).

Supuestos: canales desacoplados (integrar el lateral no cambia el
longitudinal); se desprecia el término `+w0·dp` de ejes cuerpo; no hay `dpsi`,
así que la velocidad Este del GPS sigue en 0.

### Claves JSON nuevas (`aircraft/*.json`)

| Clave | Unidad | Nota |
|---|---|---|
| `theta0_deg` | deg | cabeceo de trim (γ0 + atan2(w0, u0)) |
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
    --cols t_s,du_mps,dw_mps,dq_radps,dtheta_rad,y_elev_deg,pitot_ms,AoA_deg,gyro_y_radps,acc_x_mps2,gps_alt_m,laser_alt_m
```

La línea base puede generarse con `ROUTINE_LONGITUDINAL` y la comparación
correrse con `ROUTINE_LAT_DIR`: ambas rutinas tienen el mismo `pitch`, así que el
estado longitudinal debe coincidir. Para la fase v0.2-lateral la línea base es
`4affdca`.

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
  `ROUTINE_LAT_DIR`: el mismo `pitch` que `ROUTINE_LONGITUDINAL` más un doblete
  de alerón (±1.8°, 1–3 s) y uno de timón (±2.7°, 5–7 s). `ROUTINE_LONGITUDINAL`
  satura el alerón a 20° y queda solo para la regresión longitudinal.

El CSV agrega al final las columnas `dv_mps, dp_radps, dr_radps, dphi_rad`.

## Pendientes conocidos

- `flight_management.c` está prácticamente vacío: `FM_Init`, `FM_Update` y los
  tres tiers de control están declarados pero no implementados. Las tareas de
  5, 30, 150 y 500 ms en `main.c` tampoco hacen nada todavía.
- Los índices de servo de `main.c` (`ELEVATOR=0, AILERON=1, RUDDER=2,
  THROTTLE=3`) no coinciden con la tabla de configuración de `servo_sim.c`
  (`Elevator, Aileron L, Aileron R, Rudder`). El "throttle" usa hoy la
  constante de tiempo del rudder.
- `SERVO_FLAPS = 4` queda fuera de rango (`SERVO_SIM_N_SERVOS = 4`); el clamp
  lo descarta en silencio.
- `sim_PWM_processing` se compila pero nadie lo llama.
