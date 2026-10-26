# Status del proyecto — SIL_Flight_Control

**Fecha de revisión:** 2026-10-26
**Último commit en `main`:** `b48585b` Merge PR #21 (diseño del control lateral)
**Tags:** `v0.1-mvp` (2026-09-09), `v0.2-lateral` (2026-10-20, `7b4e35c`), `v0.3-dynamics` (2026-10-22, `717c056`)
**Versión en `CMakeLists.txt`:** 0.3.0

---

## 1. Estado actual

### Git
- `main` al día con `origin/main`; PRs #1–#21 mergeados (el #19 entró en
  `feat/dpsi` y llegó a `main` con el #21). Sin ramas abiertas.

### Build y tests (verificado 2026-10-26)
- Compila con **0 warnings** `-Wall`.
- **ctest: 11/11** (`test_servo_cfg`, `test_aircraft_loader_servo`, `test_rk4_n`,
  `test_aircraft_loader_lat`, `test_lateral_matrices`,
  `test_pilot_lateral_servo`, `test_lon_matrices`, `test_flight_sim_trim`,
  `test_atmosphere`, `test_sensors`, `test_flight_management`).
- **pytest: 73/73** en `main` (`dynamics/` 24 + `flightreview/` 49).
- Línea base de regresión vigente: `data/baseline_b48585b.csv` (local, no
  trackeada; giros y acelerómetros con 6 decimales). Frente a `cf639a6` solo
  cambian esos 6 canales (≤ 5e-4, redondeo). Las anteriores ya no aplican.
- Documentación teórica en `docs/teoria/` (LaTeX):
  `sensores/ruido_sensores.tex` y `control/control_lateral.tex`.

### Funcionalidad entregada
- **v0.1:** dinámica longitudinal `[du, dw, dq, dtheta]`, JSON de aeronave,
  servos, sensores, scheduler cooperativo, flightreview (Bokeh + reporte HTML).
- **v0.2-lateral:** modelo lateral `[dv, dp, dr, dphi]` (Nelson cap. 5) con
  corrección I_xz, RK4 genérico n/m, IMU y veleta laterales, `ROUTINE_LAT_DIR`,
  pestaña Lat-Dir en flightreview.
- **Después del tag (PRs #10 y #11, sin tag todavía):**
  - `dynamics/`: `lon_analysis.py` y `latdir_analysis.py` estiman las derivadas
    del Ugly Stick desde geometría (Nelson, DATCOM, Roskam, ISA) con una sola
    entrada `ugly_data.json`; `a_aircraft.py` las gira a ejes cuerpo y escribe
    `aircraft/ugly_stick.json`. Condición: 1000 m, 25.25 m/s, 20 kg, nivelado.
  - C en **ejes cuerpo** en ambos canales (`−w0·dq`, `X_de`, `+w0·dp`,
    `tan θ0·dr`) y sensores con `w = w0 + dw`, `θ = θ0 + dθ` (8.5 y 8.6).
  - Pitch de ambas rutinas: doblete 1–3 s; τ de elevador y alerón 0.05 s.
- **Después de `v0.3-dynamics` (PRs #14–#16):**
  - Trim del elevador (−0.49°) con trim de transmisor; servo de elevador
    k = 0.04 °/µs.
  - Pitot con Δp, ISA e IAS/TAS; ruido en la veleta; una secuencia de ruido
    por sensor.
  - Timón con la convención de Nelson (pedal derecho → δr < 0 → nariz a la
    derecha) y test de la cadena lateral piloto → servo → planta.
- **Paso 4 en curso (PRs #17–#21):**
  - `dpsi` como 5º estado (ψ̇ = r·sec θ0), GPS con rumbo (DCM completa),
    lat/lon con 7 decimales, rumbo en flightreview.
  - Diseño del control lateral en cascada: ζ_DR 0.10 → 0.83, espiral estable,
    rumbo sin sobrepaso; el modo rumbo exige el yaw damper.
  - `flight_management` con modo MANUAL (paso directo, bit a bit); giros y
    acelerómetros del CSV con 6 decimales.

### Modos del Ugly Stick (estimación teórica)
| Modo | λ | ωn [rad/s] | ζ |
|---|---|---|---|
| Periodo corto | −4.21 ± 3.09j | 5.22 | 0.81 |
| Fugoide | −0.017 ± 0.413j | 0.41 | 0.041 |
| Roll | −13.3 | — | — |
| Dutch roll | −0.413 ± 4.10j | 4.12 | 0.10 (no cumple MIL ζ ≥ 0.19) |
| Espiral | +0.075 (inestable, T2 = 9.3 s) | — | — |

Trim: α = θ0 = 5.92°, δe = −0.49°.

---

## 2. Pendientes (consolidado de TODO.md + código)

### Media prioridad
| # | Pendiente |
|---|---|
| 5.1 / 8.9 | Modos con lazos en `flight_management.c` y tareas 5/30/150/500 ms de `main.c` vacías → Paso 4 (F3b–F6) |
| 8.12 | Validar inercias (I_x, I_y, I_z), factores empíricos y σ de pitot/veleta con datos de banco y de vuelo |
| 8.2 | Contrastar derivadas de control del Navion con Nelson apéndice B |

### Baja prioridad
| # | Pendiente |
|---|---|
| 8.14 | `aircraft_a.json` (lon manual, lat Navion) se integra ahora en ejes cuerpo con `w0_ms = 4.71`: regenerarla o poner `w0_ms = 0` |
| 8.8 | Migrar longitudinal a `rk4_step_n` y borrar `rk4_step` |
| 8.11 | Typo `L_v` en diapositivas |
| 7.1 / 7.2 | Roadmap en README, diagrama de flujo de ramas |
| — | flightreview: selector libre de variables, drag-and-drop, downsampling min/max |
| — | `sim_PWM_processing` se compila pero nadie lo llama |
| — | Throttle: tau/rate del ESC "PROPUESTOS - REVISAR"; el throttle no realimenta la planta |
| — | `lon_analysis.py --plots` sin probar (requiere `python-control`) |

---

## 3. Plan

Pasos 0–3 hechos (higiene y tag `v0.3-dynamics`, trim del elevador, pitot y
veleta, test lateral y signo del timón).

### Paso 4 — v0.4: control lateral (8.4 / 5.1 / 8.9)
Una rama y un PR por fase; al final de cada una se revisa y se decide si se
inserta algo antes de seguir.

| Fase | Contenido |
|---|---|
| F1 ✅ | `dpsi` como 5º estado: lateral 5x5 (ψ̇ = r·sec θ0), GPS con rumbo, columna `dpsi_rad` |
| F2 ✅ | Diseño de ganancias en Python (clásico en cascada, ζ_DR ≥ 0.4, espiral estable) + `docs/teoria/control/control_lateral.tex` |
| F3a ✅ | Infraestructura de `flight_management` (MANUAL pass-through, bit a bit) — #20 |
| F3b | AHRS ideal con ruido + adaptar `FM_Config` (rumbo por alabeo, no por r_cmd) |
| F4 | Yaw damper |
| F5 | Modo estabilizado: wing leveler, stick → φ_cmd |
| F6 | Heading hold + rutina de piloto con cambio de modo |
| F7 | flightreview con setpoints, docs, tag `v0.4-control` |

Modos (`mode_switch`): 0 MANUAL, 1 estabilizado, 2 heading hold.
Fuera de alcance: control longitudinal y estimador de actitud real (AHRS).

### Opcional en paralelo
- 8.14 (`aircraft_a.json`), 8.8 (unificar RK4), mejoras de flightreview.
- 8.12 en cuanto haya datos de banco o de vuelo.
