# SIL_Flight_Control - TODO List
## Fase 2: Lateral-Directional Dynamics Integration

**Generado:** 2026-09-09  
**Actualizado:** 2026-11-02 (plan de ruido de IMU, 4.2–4.6)  
**Estatus:** Dinámica lateral-direccional lineal integrada (placeholders Navion)

---

## Phase Context
- **v0.1-mvp:** Longitudinal dynamics + JSON aircraft config + flightreview visualization ✅
- **Next milestone:** Lateral-directional (roll, yaw) coupled with longitudinal for full 6-DOF
- **Target:** v0.2-lateral (est. 2 weeks)
- **v0.2-lateral (hecho):** modelo lateral de 4 estados `[dv, dp, dr, dphi]`
  (Nelson cap. 5) con corrección I_xz, RK4 genérico, claves JSON laterales,
  `SIL_CONFIG_LATERAL`, rutina `ROUTINE_LAT_DIR`, IMU (p, r, ay, phi) y veleta (β)

---

## SECTION 1: Lateral-Directional Dynamics Core

### 1.3 Eigenvalue analysis of lateral A matrix ✅
- **Task:** Compute eigenvalues/eigenvectors to verify stability characteristics
- **Tools:** Matlab/Python (numpy)
- **Acceptance:** Report: (λ values, damping ratios, natural frequencies, stability margins)
- **Priority:** MEDIUM (diagnostic, not blocking)
- **Estimate:** 1.5h
- **Related:** Helps validate system response before testing
- **Status:** Navion verificado (numpy: −8.4330, −0.4861±2.3343j, −0.00893).
  Ugly Stick (2026-10-21, `dynamics/`, ejes cuerpo, ISA 1000 m, 25.25 m/s,
  20 kg): roll −13.31, dutch roll −0.413±4.096j (ζ = 0.10, no cumple MIL
  ζ ≥ 0.19), espiral +0.0746 (inestable, T2 = 9.3 s). Longitudinal: periodo
  corto −4.206±3.088j (ζ = 0.81), fugoide −0.0168±0.4125j (ζ = 0.041).
  `dynamics/tests` verifica que las A del C dan los mismos eigenvalores que
  el análisis en ejes de estabilidad.

---

## SECTION 3: Servo & Control Expansion

### 3.2 Acceptance test: servo + pilot commands (lateral) ✅
- **Hecho (2026-10-23):** `test/test_pilot_lateral_servo.c`: ruteo canal RC ->
  servo (`rc_to_servo_cmds_us`, `pilot_sim.c`), escalón de 1er orden,
  saturación y limitador de velocidad de alerón y timón, y signo de punta a
  punta con las derivadas del Ugly Stick (stick derecho -> p > 0, pedal
  derecho -> r > 0).

---

## SECTION 4: Sensor Integration

### 4.1 Fix pitot model (altitude/TAS/IAS) ✅
- **Hecho (2026-10-22):** V no lineal √(u²+v²+w²), ρ ISA en C
  (`src/atmosphere.c`), ruido y bias en Δp [Pa] (σ = 5.6 Pa), salidas IAS y
  TAS (columnas `pitot_ms` = IAS, `tas_ms`, `dp_pa`). Tests `test_atmosphere`
  y `test_sensors`. Teoría en `docs/teoria/sensores/ruido_sensores.tex`.

### 4.2 Ruido de IMU desde vuelos de prueba ✅
- **Hecho (2026-11-02):** PR #23. Frente a
  `b48585b` solo cambian las 6 columnas de la IMU; línea base `2632b23`.
  Validación espectral con `m = 1` (6×600 s): error mediano 1.7 %, máximo 26 %.
  Con la envolvente aleatoria hacen falta más corridas: 6×600 s dio máximo 39 %
  y 40×600 s, 29 %. El RMS medido del reporte no cuadra con su propia PSD en GyrY
  y AccZ, y de ahí sale el error máximo.
- **Task:** Reemplazar el ruido blanco arbitrario de giros y acelerómetros
  (σ 0.002 rad/s y 0.05 m/s²) por el modelo ajustado al log de vuelo de
  `docs/teoria/sensores/imu_noise/` (`Reporte_Ruido_IMU.md`). En vuelo el ruido
  real es 11–42 veces el actual en giros y 11–26 veces en acelerómetros.
- **Priority:** HIGH (antes de F3b: el control va a realimentar estos giros)
- **Rama:** `feat/imu-ruido`, commits atómicos
- **Modelo (por canal):**
  `y = u + b0 + (blanco + OU + m·(plano + 2 AR(2)))`, entradas gaussianas.
- **Decisiones (2026-11-02):**
  - Modelo completo, perfil `floor` con `flight = 1` siempre (el SIL arranca en
    vuelo y no tiene fase de tierra).
  - IMU a 200 Hz con retención (validada a esa tasa); el resto del SIL a 1 kHz.
  - Envolvente `m = 1` fija por defecto: en 10 s la envolvente aleatoria da
    0.16×–2.09× de RMS según la semilla (P5–P95). Con `imu_vib_env < 0` se activa
    para corridas largas.
  - Log tomado como Ugly Stick con ArduPilot: la vibración es dato del avión.
  - La IMU usa xoroshiro128+ y Box-Muller del reporte, no la LCG (ver 4.4).
    Semilla: `sensors_seed_stream(seed, SENSORS_RNG_IMU)`; el enum no cambia para
    que los demás sensores conserven sus secuencias.
  - Sesgo `b0` aleatorio por semilla con `bias_scale = 1` (el de acelerómetro,
    0.05 m/s², es un supuesto del reporte). Los `*_bias_*` deterministas se suman
    encima. Con `enable_noise = false` no se llama a `imu_step`.
- **Pasos:**
  1. docs: versionar `Reporte_Ruido_IMU.md`, el JSON, `imu_noise_model.py`,
     `imu_export_c.py` y los 2 PNG únicos; ignorar el `.zip`, el PNG duplicado y
     las copias del C.
  2. Modelo: `src/imu_params.h` (generado) y `src/imu_sensor.h`, header-only.
     `imu_sensor.h` no compila tal cual: usa `M_PI` y el proyecto compila con
     `-std=c11` sin extensiones; cambiarlo por una constante local.
  3. `src/sensors_sim.{h,c}`: quitar `gyro_noise_std_radps` y
     `accel_noise_std_mps2`; agregar `imu_profile`, `imu_flight`, `imu_vib_env`,
     `imu_{noise,white,vib,bias}_scale`, `ImuSensor imu` e `imu_accum_s`.
     `SensorsSim_UpdateImu` conserva la verdad actual, llama a `imu_step` cada
     5 ms y retiene la salida; la primera llamada dispara de inmediato.
  4. `src/main.c`: configuración nueva y comentario "ruido sigma 0.002" del CSV.
  5. Tests en `test/test_sensors.c`.
  6. Docs: sección IMU en `ruido_sensores.tex`, tabla de parámetros y la frase
     "todos los sensores se actualizan cada paso"; `README.md`,
     `docs/teoria/README.md`; línea base nueva en `status.md`.
- **Acceptance:**
  - 0 warnings `-Wall`, ctest completo y `pytest flightreview dynamics` en verde.
  - `enable_noise = false`: salida igual a la verdad.
  - `floor` sin vuelo y `bias_scale = 0`: σ por canal ±5 % de √(σ_w²+σ_c²).
  - `flight = 1`, `m = 1`: σ ±10 % del total (giro 0.085/0.036/0.023 rad/s,
    acel 0.56/1.17/1.32 m/s²).
  - Retención: valor constante durante 5 ms y nuevo en el 5.º paso.
  - Misma semilla, misma salida; encender la IMU no mueve ni un bit de Δp.
  - `test/compare_csv_cols.py` contra `data/baseline_b48585b.csv`: solo cambian
    `gyro_{x,y,z}_radps` y `acc_{x,y,z}_mps2`. Después, guardar línea base nueva.
  - Validación espectral única: `test/imu_test.c`, submuestreado a 50 Hz,
    contra la tabla 8.3 del reporte (RMS > 5 Hz y PSD 5–12 / 12–24 Hz, error
    máximo ≈ 30 %). Registrarla en el `.tex`.
- **Límites conocidos:**
  - `white_scale` casi no influye en vuelo (σ_w ~6e-4 rad/s frente a 0.085 de
    vibración). La incertidumbre real es la vibración de 25–100 Hz, y ninguna
    perilla la cubre.
  - Sin filtro, el control recibe la vibración completa (`p`: 0.085 rad/s RMS a
    11–18 Hz). El log parece venir ya filtrado por ArduPilot (inferencia por los
    nombres de columnas), así que no suponer otro LPF igual sin revisarlo. F4
    agregó un filtro de 1er orden configurable por eje; el corte queda por ajustar.

### 4.3 Datos del .bin de ArduPilot
- **Task:** Del log original, si los tiene, sacar `INS_GYRO_FILTER` e
  `INS_ACCEL_FILTER`, throttle (`RCOU`/`CTUN`) para ligar la envolvente a la
  potencia y separar `floor` de `idle`, y RPM para descartar alias en 17–19 Hz.
  Agregar `imu_noise_analysis.py` y `sensors_data.csv`, que faltan en
  `imu_noise/` y hacen falta para reproducir el ajuste.
- **Priority:** MEDIUM

### 4.4 Periodo de la LCG
- **Task:** `sensors_rand_uniform_0_1` usa los 24 bits bajos, con periodo 2²⁴:
  a 6 normales por paso a 1 kHz (GPS, y la IMU actual) el ruido se repite cada
  233 s. No afecta a corridas de 10 s. Usar los bits altos; cambia todas las
  columnas con ruido, así que va aparte y con línea base nueva.
- **Priority:** LOW

### 4.5 Verdad del acelerómetro
- **Task:** Agregar `-r·v` en `ax` y `+p·v` en `az`, y usar
  `SENSORS_SIM_GRAVITY_MPS2` en lugar de 9.81 en `SensorsSim_UpdateImu`.
- **Priority:** LOW

### 4.6 IMU en flightreview
- **Task:** Revisar los umbrales de vibración (3 y 6 m/s² sobre |a|, que ya
  quedan superados por |a| ≈ 9.8) y agregar una gráfica de PSD de la IMU.
- **Priority:** LOW

---

## SECTION 5: Flight Management System (Future)

### 5.1 Stub flight_management.c
- **Task:** Create basic autopilot framework for lateral hold (wing level, heading hold)
- **Files:** `src/flight_management.c`, `src/flight_management.h`
- **Scope:** Minimal - just structure and placeholder control laws
- **Priority:** LOW (design phase only)
- **Estimate:** 2h
- **Related:** Will integrate with lateral dynamics in v0.3

---

## SECTION 6: Code Quality & CI/CD

### 6.1 Resolve remaining compiler warnings ✅
- **Task:** Audit `-Wall` output across all .c files, fix warnings
- **Files:** `src/**/*.c`, `test/**/*.c`
- **Acceptance:** `cmake --build build -- -Wall` produces 0 warnings
- **Hecho (2026-10-20):** 0 warnings. Variables reservadas de `main.c` con
  `(void)`; andamiaje de `flight_management.c` con `FM_UNUSED` hasta v0.3.
- **Priority:** MEDIUM
- **Estimate:** 1.5h

### 6.3 Add ctest harness for lateral servo/pilot test ✅
- **Hecho (2026-10-23):** `test_pilot_lateral_servo` registrado en ctest.

---

## SECTION 7: Documentation & Cleanup

### 7.1 Update README.md with v0.2 roadmap
- **Task:** Document lateral-directional phase plan and architecture
- **Files:** `README.md`
- **Priority:** LOW (nice-to-have)
- **Estimate:** 1h

### 7.2 Create branch workflow diagram
- **Task:** Document expected branches for v0.2 (feat/lateral-*, wip/*, etc.)
- **Files:** `docs/git_workflow.md` or `README.md`
- **Priority:** LOW
- **Estimate:** 0.5h

---

## SECTION 8: Pendientes v0.2-lateral

### 8.1 Derivadas laterales reales (reemplazar placeholders Navion) ✅
- **Task:** Cargar en `aircraft/ugly_stick.json` las derivadas laterales reales
  (desde MATLAB) y las de `aircraft/aircraft_a.json`.
- **Hecho (ugly_stick, 2026-10-20):** estimadas con `dynamics/latdir_analysis.py`
  (Nelson; ala alta DATCOM) y escritas con `dynamics/a_aircraft.py` (antes
  `latdir_a_aircraft.py`; desde 2026-10-21 también el longitudinal, en ejes cuerpo).
  `aircraft_a.json` sigue con placeholders del Navion.
- **Note:** Los valores originales de `aircraft_a` eran `L_da = 4.66`, `L_p = -1.30`.
- **Priority:** HIGH

### 8.2 Verificar derivadas de control del Navion
- **Task:** Contrastar `L_da, L_dr, N_da, N_dr, Y_dr` contra Nelson, apéndice B.
- **Priority:** MEDIUM

### 8.3 Signo del timón ✅
- **Hecho (2026-10-23):** convención de Nelson: δr > 0 = borde de salida a la
  izquierda (N_dr < 0, nariz a la izquierda). El pedal derecho (yaw > 0) da
  δr < 0 y lleva la nariz a la derecha (`RC_YAW_TO_RUDDER_SIGN = -1` en
  `pilot_sim.c`).

### 8.4 dpsi como 5º estado ✅
- **Hecho (2026-10-23):** lateral 5x5 con dpsi' = sec(θ0)·dr; GPS con la
  rotación cuerpo -> NED completa (`gps_ve_ms` ≠ 0); columna `dpsi_rad` y
  rumbo en flightreview. El heading hold va en el Paso 4 (F6).

### 8.5 Término w0 de ejes cuerpo en la fuerza lateral ✅
- **Hecho (2026-10-21):** lon y lat en ejes cuerpo: `-w0·dq` y `X_de` en el
  longitudinal, `+w0·dp` y `tan(θ0)·dr` en el lateral.

### 8.6 theta de sensores vs theta0 del modelo lateral ✅
- **Hecho (2026-10-21):** sensores con `θ = θ0 + dθ` y `w = w0 + dw`.

### 8.7 Ruido en la veleta de beta ✅
- **Hecho (2026-10-22):** ruido de 0.5° y bias configurable en α y β. Cada
  sensor tiene ahora su propia secuencia de LCG, así que agregar ruido ya no
  desplaza los demás sensores.

### 8.8 Migrar longitudinal a rk4_step_n
- **Task:** Usar `rk4_step_n` también en el longitudinal y eliminar `rk4_step`
  (`test_rk4_n` T1 ya prueba la equivalencia bit a bit).
- **Priority:** LOW

### 8.9 Leyes de control laterales
- **Task:** Yaw damper, wing leveler, heading hold en `flight_management.c`
  (ver 5.1).
- **Priority:** LOW (v0.3)

### 8.10 Plots del canal lateral en flightreview ✅
- **Task:** Graficar `dv/dp/dr/dphi`, `gyro_x/z`, `acc_y`, `SSA_deg` (otra rama).
- **Hecho:** pestaña "Dinamica Lat-Dir" (dv/dp/dr/dphi), roll en Actitud y dp/dr
  sobre los giros en Rapidez angular. `gyro_x/z`, `acc_y` y `SSA_deg` ya salían
  en Sensores e Inercial.
- **Priority:** MEDIUM

### 8.11 Typo en diapositivas: L_v del Navion
- **Task:** Corregir `L_v = -0.0298` -> `-0.299` (con el typo el espiral sale
  inestable: λ = +0.0385).
- **Priority:** LOW

### 8.12 Validar derivadas e inercias del Ugly Stick
- **Task:** Las inercias salen de radios de giro de aviones reales y varios
  factores son empíricos (`K_v`, `η_v`, el término `0.2·η·V_v·CLα_v` de Cnβ;
  `τ = √(S_e/S_ht)`, `η_ht`, `CL_u`, `CD_u`, `CM_u` en el longitudinal);
  no se modela el fuselaje (Cnβ, Cnr). Medir I_x/I_y/I_z con péndulo bifilar y
  ajustar Cnr/Clβ y Cmq con datos de vuelo (respuesta a dobletes). La vibración
  medida de la IMU ya tiene modelo (4.2); falta contrastarla con más vuelos, porque
  la potencia media de un solo log tiene una incertidumbre del orden de ±50 %.
- **Priority:** MEDIUM

### 8.13 Trim del elevador ✅
- **Hecho (2026-10-22):** la planta recibe `δ − δ_trim` (`flight_sim.c`),
  `main.c` aplica un trim de transmisor `δ_trim / k` en µs y arranca los servos
  en trim, y `a_aircraft.py` escribe `delta_e_trim_deg` (−0.49° en el Ugly
  Stick). Línea base `e041d25`.

### 8.14 aircraft_a.json en ejes cuerpo
- **Task:** Sus derivadas longitudinales son manuales y las laterales del
  Navion (ejes de estabilidad), pero desde 2026-10-21 el C las integra en ejes
  cuerpo (`w0_ms = 4.71` ahora entra en `-w0·dq`, `+w0·dp` y sensores).
  Regenerarla con `dynamics/` o poner `w0_ms = 0` y `theta0_deg = gamma0_deg`.
- **Priority:** LOW

---

## SECTION 9: Pendientes del refactor de tareas de vuelo

Fuera de alcance de `docs/PLAN_FLIGHT_TASKS.md` (refactor puro, CSV idéntico).

### 9.1 Desfase de 1 ms del estado en el CSV
- **Task:** En `Task_1ms` el estado (`du`…`dtheta`, `X_lat`) se lee **antes**
  de `FlightSim_Step` y los sensores **después**: en la misma fila del CSV el
  estado corresponde a t − 1 ms y los sensores a t (ver la NOTA en
  `arnes_servos_planta`). Corregirlo cambia las columnas de estado y obliga a
  sacar una línea base nueva: va en un commit aparte.
- **Priority:** MEDIUM

### 9.2 `rc_mixer` portable al STM32
- **Task:** Renombrar `SimRc_s` → `RcInput_s` y desacoplar `rc_mixer` de
  `servo_sim.h` y de `sim_settings.h`.
- **Priority:** LOW

### 9.3 `FM_Update_5ms` static
- **Task:** Volver `FM_Update_5ms` `static` y migrar `test_yaw_damper_cl` a
  `Flight_Task_5ms`.
- **Priority:** LOW

### 9.4 F5: wing leveler
- **Task:** `cl_wing_leveler` en `Flight_Task_30ms`, activar `fm_slew_limit`
  y aceptar `mode_switch` 1.
- **Priority:** HIGH (siguiente fase)

### 9.5 F6: heading hold
- **Task:** `cl_heading_hold` en `Flight_Task_150ms`; bloquear el modo 2 si el
  yaw damper no está activo.

### 9.6 Rutinas de piloto como tabla de eventos
- **Task:** Reemplazar el `switch` con `step_cmd` de `pilot_source_commands`
  por una tabla de eventos.
- **Priority:** LOW

---

## DEPENDENCIES & SEQUENCING
