# Status del proyecto — SIL_Flight_Control

**Fecha de revisión:** 2026-10-21
**Último commit en `main`:** `06d1f23` Merge PR #11 (derivadas longitudinales + ejes cuerpo)
**Tags:** `v0.1-mvp` (2026-09-09), `v0.2-lateral` (2026-10-20, `7b4e35c`)
**Versión en `CMakeLists.txt`:** 0.2.0

---

## 1. Estado actual

### Git
- `main` al día con `origin/main`; PRs #1–#11 mergeados.
- Rama `feat/derivadas-longitudinales-ugly` (local y remota) ya mergeada en #11:
  se puede borrar.
- Siguen los **4 stashes viejos** de `dinamica-longitudinal-sensores`:
  - `stash@{0}`–`{2}`: contenido que ya está en `main` o binarios de build.
  - `stash@{3}`: un `data.zip` de 3.4 MB → revisarlo antes de descartar.
- Carpeta `build_tests/` en la raíz: build viejo (ignorado por git), residuo.

### Build y tests (verificado 2026-10-21)
- Compila con **0 warnings** `-Wall` (6.1 hecho en PR #9).
- **ctest: 6/6** (`test_servo_cfg`, `test_aircraft_loader_servo`, `test_rk4_n`,
  `test_aircraft_loader_lat`, `test_lateral_matrices`, `test_lon_matrices`).
- **pytest: 62/62** (`dynamics/` 16 + `flightreview/` 46).
- Línea base de regresión vigente: `data/baseline_5ea17b6.csv` (local, no
  trackeada). Las anteriores (`4affdca`, `0959bad`, `f1b6478`,
  `pitch_unificado`) ya no aplican.

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

### Alta prioridad
| # | Pendiente | Nota |
|---|---|---|
| 4.1 | Modelo de pitot | `SensorsSim_UpdatePitot` (`src/sensors_sim.c`) sigue con `rho0 = 1.225` fijo y ruido sobre la velocidad. Ya existe `dynamics/atmosfera.py` (ISA) como referencia para portar a C. Cambia la línea base (agrupar con 8.7). |

### Media prioridad
| # | Pendiente |
|---|---|
| 8.13 | Trim del elevador: δe_trim se calcula pero `delta_e_trim_deg` queda en 0 porque el C usa el elevador absoluto como perturbación. Restar el trim antes de la planta. |
| 8.12 | Validar inercias (I_x, I_y, I_z) y factores empíricos lon/lat con péndulo bifilar y dobletes en vuelo |
| 3.2 / 6.3 | Test `test_pilot_lateral_servo.c` + registrarlo en ctest |
| 8.2 | Contrastar derivadas de control del Navion con Nelson apéndice B |
| 8.3 | Decidir convención de signo del timón (yaw>0 → dr<0) |
| 8.4 | `dpsi` como 5º estado (GPS `ve` ≠ 0, heading hold) |
| — | Doblete de pitch: lleva el elevador a ±22° y dθ > 2 rad, fuera del rango del modelo lineal. Reducir amplitud de la rutina (cambia la línea base). |

### Baja prioridad / v0.3
| # | Pendiente |
|---|---|
| 5.1 / 8.9 | `flight_management.c` y tareas 5/30/150/500 ms de `main.c` vacías |
| 8.14 | `aircraft_a.json` (lon manual, lat Navion) se integra ahora en ejes cuerpo con `w0_ms = 4.71`: regenerarla o poner `w0_ms = 0` |
| 8.7 | Ruido en veleta β (regenera línea base: desplaza el LCG) |
| 8.8 | Migrar longitudinal a `rk4_step_n` y borrar `rk4_step` |
| 8.11 | Typo `L_v` en diapositivas |
| 7.1 / 7.2 | Roadmap en README, diagrama de flujo de ramas |
| — | flightreview: selector libre de variables, drag-and-drop, downsampling min/max |
| — | `sim_PWM_processing` se compila pero nadie lo llama |
| — | Throttle: tau/rate del ESC "PROPUESTOS - REVISAR"; el throttle no realimenta la planta |
| — | `lon_analysis.py --plots` sin probar (requiere `python-control`) |

---

## 3. Plan propuesto

### Paso 0 — Higiene (~30 min, no cambia el simulador)
1. Borrar `feat/derivadas-longitudinales-ugly` (local + remoto).
2. Revisar `stash@{3}` (`data.zip`) y descartar los stashes.
3. Borrar `build_tests/`.
4. Tag `v0.3-dynamics` (o similar) sobre `main` para marcar el modelo en ejes
   cuerpo con derivadas del Ugly Stick; subir `CMakeLists.txt` de versión.

### Paso 1 — Planta coherente con el trim (8.13 + amplitud del doblete)
- Restar δe_trim antes de la planta y escribirlo desde `a_aircraft.py`.
- Reducir la amplitud del doblete para quedar en rango lineal.
- Ambos cambian la línea base: un commit cada uno, regenerar al final.

### Paso 2 — Fidelidad de sensores (4.1 pitot + 8.7)
- Portar la ISA a C (ρ(h) en el pitot), separar TAS/IAS, ruido en presión
  dinámica; ruido en β. Una sola regeneración de línea base.

### Paso 3 — Tests de integración lateral (3.2 / 6.3)
- `test_pilot_lateral_servo.c`: escalones en alerón y timón, saturación,
  ruteo canal→servo. Registrar en ctest.

### Paso 4 — v0.3: control (5.1 / 8.9 / 8.4)
- `dpsi` como 5º estado.
- `flight_management.c`: yaw damper (el dutch roll tiene ζ = 0.10) → wing
  leveler (la espiral es inestable) → heading hold; usar las tareas vacías.
- Emitir `mode` y setpoints en el CSV → flightreview ya los sabe pintar.

### Opcional en paralelo
- 8.14 (`aircraft_a.json`), 8.8 (unificar RK4), mejoras de flightreview.
- 8.12 en cuanto haya datos de banco o de vuelo.
