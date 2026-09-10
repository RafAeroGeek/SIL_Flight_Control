# SIL_Flight_Control - TODO List
## Fase 2: Lateral-Directional Dynamics Integration

**Generado:** 2026-09-09  
**Estatus:** Post-MVP (v0.1-mvp merged to main)

---

## Phase Context
- **v0.1-mvp:** Longitudinal dynamics + JSON aircraft config + flightreview visualization ✅
- **Next milestone:** Lateral-directional (roll, yaw) coupled with longitudinal for full 6-DOF
- **Target:** v0.2-lateral (est. 2 weeks)

---

## SECTION 1: Lateral-Directional Dynamics Core

### 1.1 Extract A matrix from Matlab
- **Task:** Extract 5×5 lateral-directional A matrix from Matlab SS_v1 model
- **Files:** `matlab/SS_v1_lateral_matrix.m` → Copy matrix values
- **Acceptance:** Numeric values hardcoded in comments, eigenvalues logged
- **Priority:** HIGH
- **Estimate:** 1h
- **Related:** Branch `dinamica-longitudinal-sensors` has partial work

### 1.2 Create ss_lateral_dir.c stub
- **Task:** Implement `src/dynamic_models/ss_lateral_dir.c` with RK4 integration
- **Files:** 
  - Create: `src/dynamic_models/ss_lateral_dir.c`
  - Header: `src/dynamic_models/ss_lateral_dir.h`
  - State vector: `[dp, dr, dv, dphi, dpsi]` (roll rate, yaw rate, lateral vel, roll angle, yaw angle)
- **Reference:** Mirror structure from `ss_longitudinal.c`
- **Acceptance:** Compiles with -Wall, RK4 integrates states correctly
- **Priority:** HIGH
- **Estimate:** 2h
- **Related:** TODO.md issue #1.1

### 1.3 Eigenvalue analysis of lateral A matrix
- **Task:** Compute eigenvalues/eigenvectors to verify stability characteristics
- **Tools:** Matlab/Python (numpy)
- **Acceptance:** Report: (λ values, damping ratios, natural frequencies, stability margins)
- **Priority:** MEDIUM (diagnostic, not blocking)
- **Estimate:** 1.5h
- **Related:** Helps validate system response before testing

---

## SECTION 2: Conditional Compilation & Config

### 2.1 Add SIL_CONFIG_LATERAL to CMakeLists.txt
- **Task:** Implement `#define SIL_CONFIG` with option to select LONGITUDINAL or LATERAL_DIR or COMBINED
- **Files:** 
  - `CMakeLists.txt`: Add compile flag option
  - `src/ss_combined.c`: Conditionally call `ss_longitudinal_step()` or `ss_lateral_dir_step()`
- **Acceptance:** Build succeeds with `-DSIL_CONFIG=COMBINED`, state vector expands to 9D
- **Priority:** HIGH
- **Estimate:** 1.5h
- **Related:** Architecture decision from chat history

### 2.2 Update aircraft JSON for lateral params
- **Task:** Add lateral-specific fields to `aircraft_configs/ss_v1.json`
  - Servo limits (aileron, rudder)
  - Sensor noise params for yaw gyro
  - Control gains for lateral autopilot (if any)
- **Files:** `aircraft_configs/ss_v1.json`
- **Acceptance:** JSON parses correctly, values match Matlab SS_v1
- **Priority:** MEDIUM
- **Estimate:** 1h

---

## SECTION 3: Servo & Control Expansion

### 3.1 Expand pilot_sim.c for aileron/rudder
- **Task:** Add step/ramp command generators for aileron and rudder channels
- **Files:** `src/pilot_sim.c`
- **Details:**
  - Mirror existing throttle/elevator logic
  - Pilot command units: normalized [-1, +1] (aileron, rudder)
  - Test routine: step inputs at T=5s, T=15s, T=25s spread across 30s flight
- **Acceptance:** CSV output shows aileron/rudder commands changing at expected times
- **Priority:** HIGH
- **Estimate:** 1.5h
- **Related:** Issue: Servo subsystem pilot command ambiguity (partially resolved in MVP)

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
- **Task:** Correct pitot tube model in `src/sensors/pitot.c`
  - Bug: Fixed sea-level density vs. altitude (should vary with altitude)
  - Bug: TAS/IAS convention mixing
  - Bug: Noise model applied to velocity instead of pressure
- **Files:** `src/sensors/pitot.c`, `src/sensors/pitot.h`
- **Reference:** Standard atmosphere model (density decay)
- **Acceptance:** 
  - Density decreases with altitude (verify at 1000m vs. 5000m)
  - IAS computed correctly from TAS
  - Noise injected on pressure, not velocity
- **Priority:** HIGH (affects simulation fidelity)
- **Estimate:** 2h

### 4.2 Add yaw gyro sensor
- **Task:** Implement yaw rate (r) measurement in IMU sensor suite
- **Files:** 
  - Update: `src/sensors/imu.c`
  - Config: `aircraft_configs/ss_v1.json` (yaw gyro scale, bias, noise)
- **Acceptance:** Sensor outputs dψ/dt with realistic noise profile
- **Priority:** MEDIUM
- **Estimate:** 1.5h
- **Related:** Necessary for lateral-directional closed-loop control

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

### 6.1 Resolve remaining compiler warnings
- **Task:** Audit `-Wall` output across all .c files, fix warnings
- **Files:** `src/**/*.c`, `test/**/*.c`
- **Acceptance:** `cmake --build build -- -Wall` produces 0 warnings
- **Priority:** MEDIUM
- **Estimate:** 1.5h

### 6.2 Decide: Track .claude/ directory in repo
- **Task:** Decision: Include `.claude/` (Claude Code artifacts) in Git or .gitignore?
- **Pro track:** Reproducible runs, audit trail of code changes
- **Pro ignore:** Cleaner history, fewer conflicts
- **Files:** `.gitignore` or `.claude/` subdirs
- **Decision maker:** Rafa
- **Priority:** LOW (administrative)
- **Estimate:** 0.5h

### 6.3 Add ctest harness for lateral dynamics
- **Task:** Create `test/CMakeLists.txt` entry for lateral servo/pilot acceptance tests
- **Files:** `test/CMakeLists.txt`
- **Acceptance:** `ctest --test-dir build` runs all tests including new lateral suite
- **Priority:** MEDIUM
- **Estimate:** 1h

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

## DEPENDENCIES & SEQUENCING
