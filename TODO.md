# SIL_Flight_Control - TODO List
## Fase 2: Lateral-Directional Dynamics Integration

**Generado:** 2026-09-09  
**Actualizado:** 2026-10-20 (cierre de v0.2-lateral)  
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

### 1.3 Eigenvalue analysis of lateral A matrix
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

### 3.2 Acceptance test: servo + pilot commands (lateral)
- **Task:** Create `test/test_pilot_lateral_servo.c`
  - Exercise rudder and aileron channels with step inputs
  - Verify servo saturation limits respected
  - Verify pilot command routing to servo module
- **Files:** `test/test_pilot_lateral_servo.c`
- **Acceptance:** All assertions pass, no undefined behavior detected
- **Priority:** MEDIUM
- **Estimate:** 2h
- **Related:** Previous acceptance test for throttle was vacuous (reads 0 throughout)

---

## SECTION 4: Sensor Integration

### 4.1 Fix pitot model (altitude/TAS/IAS)
- **Task:** Correct pitot tube model in `SensorsSim_UpdatePitot` (`src/sensors_sim.c`)
  - Bug: Fixed sea-level density vs. altitude (should vary with altitude;
    hoy `rho0 = 1.225` fijo)
  - Bug: TAS/IAS convention mixing
  - Bug: Noise model applied to velocity instead of pressure
- **Files:** `src/sensors_sim.c`, `src/sensors_sim.h`
- **Note:** cambia la línea base de regresión (agrupar con 8.7)
- **Reference:** Standard atmosphere model (density decay)
- **Acceptance:** 
  - Density decreases with altitude (verify at 1000m vs. 5000m)
  - IAS computed correctly from TAS
  - Noise injected on pressure, not velocity
- **Priority:** HIGH (affects simulation fidelity)
- **Estimate:** 2h

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

### 6.3 Add ctest harness for lateral servo/pilot test
- **Task:** Register `test/test_pilot_lateral_servo.c` (issue 3.2) in ctest
- **Note:** Lateral dynamics tests already in ctest (v0.2): `test_rk4_n`,
  `test_aircraft_loader_lat`, `test_lateral_matrices`
- **Files:** `CMakeLists.txt`
- **Priority:** MEDIUM
- **Estimate:** 0.5h

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

### 8.1 Derivadas laterales reales (reemplazar placeholders Navion)
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

### 8.3 Signo del timón
- **Observado:** yaw > 0 -> dr < 0 (consistente con `N_dr < 0`, convención de
  Nelson). Decidir si la convención stick/PWM del rudder debe invertirse.
- **Priority:** MEDIUM

### 8.4 dpsi como 5º estado
- **Task:** Agregar dpsi -> GPS `ve` distinto de 0 y heading hold.
- **Priority:** MEDIUM

### 8.5 Término w0 de ejes cuerpo en la fuerza lateral ✅
- **Hecho (2026-10-21):** lon y lat en ejes cuerpo: `-w0·dq` y `X_de` en el
  longitudinal, `+w0·dp` y `tan(θ0)·dr` en el lateral.

### 8.6 theta de sensores vs theta0 del modelo lateral ✅
- **Hecho (2026-10-21):** sensores con `θ = θ0 + dθ` y `w = w0 + dw`.

### 8.7 Ruido en la veleta de beta
- **Task:** Agregar ruido a `SideSlipAngle_deg` en un solo commit que regenere la
  línea base (una llamada nueva a `sensors_noise()` desplaza el LCG).
- **Priority:** LOW

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
  ajustar Cnr/Clβ y Cmq con datos de vuelo (respuesta a dobletes).
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

## DEPENDENCIES & SEQUENCING
