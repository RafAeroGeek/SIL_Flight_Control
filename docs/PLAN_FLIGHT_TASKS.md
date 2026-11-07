# PLAN — Software de vuelo en `flight_management`: `Flight_Task_5ms` y una ley de control por archivo

**Proyecto:** `RafAeroGeek/SIL_Flight_Control`
**Rama de trabajo:** `refactor/flight-task-5ms` (sale de `main` @ `f140925`, merge del PR #28)
**Destinatario:** Claude Code
**Fecha:** 2026-11-06

---

## 0. Reglas de trabajo para Claude Code (LEER PRIMERO)

1. **Un commit por paso** (sección 5). Mensajes en español con prefijo convencional
   (`refactor:`, `feat:`, `test:`, `docs:`, `chore:`).
2. **Al terminar cada paso: DETENERSE.** Mostrar el diff resumido y el resultado de
   build, ctest y regresión, y **esperar la confirmación de Rafa** antes de hacer el
   commit y antes de pasar al siguiente paso.
3. **Antes del Paso 0:** presentar a Rafa las decisiones de la sección 3 y esperar su
   respuesta. Si cambia alguna, ajustar los pasos afectados y mostrar el ajuste antes
   de escribir código.
4. **No hacer `git push`**, ni merge, ni borrar ramas. Eso lo hace Rafa a mano.
5. **No tocar** `flightreview/`, `python/`, `dynamics/`, `docs/teoria/` ni el esquema
   del CSV (mismas columnas y en el mismo orden).
6. C11, `-Wall`, **0 warnings** (hoy hay 0). Sin `malloc`. Sin estado global nuevo en
   las leyes de control: su estado vive en un struct que es propiedad de
   `flight_management.c`.
7. **Regla de oro de este plan: es un refactor puro.** El CSV de la simulación debe
   quedar **idéntico bit a bit** (comparado como texto, las 65 columnas) contra las
   líneas base, en **todos** los pasos. Si algún paso da diferencias, se detiene y se
   reporta: no se "arregla" cambiando la línea base.
8. Comandos de verificación, desde la raíz del repo:
   ```bash
   cmake -S . -B build -DSIL_BUILD_TESTS=ON
   cmake --build build 2>&1 | grep -E "warning|error"     # debe salir vacío
   ctest --test-dir build --output-on-failure
   bash test/regresion_tareas.sh                          # desde el Paso 1
   ```

---

## 1. Objetivo

Hoy `main.c` hace dos papeles mezclados:

- **Arnés del SIL:** piloto simulado, servos, planta, sensores y log CSV (`Task_1ms`).
- **Software de vuelo:** lo que correría en el STM32 (`Task_5ms`): modo y armado,
  filtro de giros, filtro del pitot, `FM_State`, `FM_Update_5ms` y la mezcla a µs.

Se busca:

1. Que **`flight_management.c/.h` administre el software de vuelo**. Expone una tarea
   por periodo del RTOS (`Flight_Task_5ms`, y como esqueleto `Flight_Task_30ms`,
   `_150ms` y `_500ms`) y decide qué leyes corren según el modo.
2. Que **cada ley de control viva en su propio `.c/.h`** (`cl_<ley>.c/.h`), pura y
   testeable sola. La primera es el yaw damper, que hoy está como `fm_yaw_damper()`
   dentro de `flight_management.c`.
3. Que `Task_5ms` de `main.c` quede como **adaptador**: arma las entradas desde el
   simulador, llama a `Flight_Task_5ms` y entrega los µs a los servos.

```
              ARNÉS SIL (main.c)                     SOFTWARE DE VUELO (portable a STM32)
 ┌──────────────────────────────────────┐       ┌─────────────────────────────────────────────┐
 │ Task_1ms: piloto → RC → servos →     │       │ flight_management.c                          │
 │           planta → sensores → CSV    │       │   Flight_Task_5ms(now, dt, rc, mode, sens,   │
 │                                      │  rc   │                   servo_cmd_us)              │
 │ Task_5ms:                            │──────▶│     modo/armado → filtros (giros, pitot)    │
 │   rc   = g_rc_sim                    │ sens  │     → FM_State → leyes según el modo         │
 │   sens = fc_sensores_desde_sim(...)  │──────▶│        └─ cl_yaw_damper.c  (cl_*.c futuras) │
 │   Flight_Task_5ms(...)               │  µs   │     → rc_mixer.c (stick → µs + trim)        │
 │   Servos_SetCmdDeltaUs(µs)           │◀──────│                                             │
 │ Task_30/150/500ms → Flight_Task_*    │       │   Flight_Task_30/150/500ms (vacías por ahora)│
 └──────────────────────────────────────┘       └─────────────────────────────────────────────┘
```

---

## 2. Estado actual relevante (`main` @ `f140925`)

| Archivo | Situación |
|---|---|
| `main.c` `Task_5ms` | Lee `g_rc_sim` y `g_sens_data`; `FM_SetArmed`, `FM_SetModeSwitch`, `GyroFilter_Update`, `Lpf1_Update` (solo si `pitot.valid`), `fm_state_desde_sensores`, `FM_Update_5ms`, `FM_GetCmdOut`, `rc_to_servo_cmds_us(…, g_trim_us, …)`, `Servos_SetCmdDeltaUs` |
| `main.c` globales del FC | `g_gyro_filter`, `g_gyro_filt_radps[3]`, `g_pitot_filter`, `g_pitot_filt_ms`. El CSV los lee en `Task_1ms` (giros filtrados, `fc_hz`, `pitot_ms_filtered` con `con_muestra ? filt : ias`) |
| `main.c` `task_init_1ms` | `GyroFilter_Init` y `Lpf1_Init` con `fm_cfg.gyro_fc_hz` y `fm_cfg.pitot_fc_hz`; imprime los cortes |
| `main.c` `Task_30/150/500ms` | Vacías |
| `flight_management.c` | Estado global `g_cfg`, `g_status`, `g_x_wo_dps`, `g_yd_activo`. `fm_yaw_damper()` estática. `Flight_Control_Attitude_Tier()` vacía; `_Cruice_Tier` y `_Mission_Tier` declaradas sin definir y sin uso |
| `pilot_sim.c/.h` | Mezcla piloto simulado (`pilot_source_commands`, `rc_source`) con cosas del FC: `SimRc_s`, `RC_CH_*`, `rc_to_servo_cmds_us`, `rc_yaw_deg_per_stick`, `RC_YAW_TO_RUDDER_SIGN` |
| Tests que usan el FM | `test_flight_management` (unidad, MANUAL y yaw damper), `test_yaw_damper_cl` (lazo cerrado; usa `FM_Update_5ms` y `rc_to_servo_cmds_us`), `test_pilot_lateral_servo` (usa `rc_to_servo_cmds_us`) |
| Rutinas | `ROUTINE_LAT_DIR` (default, todo en MANUAL) y `ROUTINE_YAW_DAMPER` (cambia a `mode_switch` 3 = `GYRO_STAB` en t = 5 s). Con las dos se cubren las dos ramas del FM |

---

## 3. Decisiones (confirmar con Rafa ANTES del Paso 0)

| # | Decisión | Recomendación (lo que asume el plan) | Alternativa |
|---|---|---|---|
| D1 | Firma de `Flight_Task_5ms` | `void Flight_Task_5ms(uint32_t now_ms, uint32_t dt_ms, const SimRc_s *rc_sim, int mode_cmd, const FC_SensorsIn *sens, float servo_cmd_us[SERVO_SIM_N_SERVOS]);` Es la de Rafa más dos cosas: los **sensores** (el FM los necesita y no debe leer globales de `main.c`) y la **salida en µs**. `rc_sim` va por puntero `const` en vez de por valor (64 B), que es lo habitual en embebido | La firma literal `(now_ms, dt_ms, SimRc_s rc_sim, int mode_cmd)`, con sensores y salida por setters/getters aparte (`FM_SetSensors`, `FM_GetServoCmdUs`) |
| D2 | `mode_cmd` aparte de `rc_sim.s[RC_CH_MODE]` | Se **mantiene**, como pidió Rafa: separa de dónde viene el modo (switch del RC hoy; estación terrena o failsafe después). `main.c` pasa `(int)g_rc_sim.s[RC_CH_MODE]`, así que el resultado no cambia. `arm` sigue saliendo de `rc_sim.s[RC_CH_ARM]` | Quitar `mode_cmd` y leerlo del canal adentro |
| D3 | Tipo de entrada de sensores | Struct neutro **`FC_SensorsIn`** en `flight_management.h`, con solo lo que usa el FC (giros, AHRS, IAS, alt/vd del GPS, flags `valid`). `main.c` lo llena desde `SensorsSimData` (`fc_sensores_desde_sim`). Así el FM no incluye `sensors_sim.h` y en el STM32 lo llenan los drivers | Pasar `const SensorsSimData *`. Es más corto, pero ata el FC al simulador |
| D4 | Dónde viven `SimRc_s`, `RC_CH_*` y la mezcla a µs | Nuevo **`src/rc_mixer.c/.h`** (software de vuelo). `pilot_sim.h` lo incluye, así que todo lo que hoy compila con `pilot_sim.h` sigue compilando. El nombre `SimRc_s` no cambia | Dejar todo en `pilot_sim.c` y que `flight_management.c` incluya `pilot_sim.h`. Así el FC depende del piloto simulado |
| D5 | Ubicación de las leyes | Plano en `src/`, prefijo **`cl_`** (`cl_yaw_damper.c/.h`; luego `cl_wing_leveler`, `cl_heading_hold`…). No hay que tocar include dirs | Subcarpeta `src/control_laws/` (hay que agregarla a `target_include_directories` del ejecutable y de cada test) |
| D6 | Unidades de una ley | La ley recibe y devuelve **unidades físicas** (deg, deg/s, s). La conversión a stick y la validez del estado las hace el FM. La ley no conoce el modo ni el RC | Que la ley devuelva el stick directamente |
| D7 | `FM_Update_5ms` | Se queda **pública** (la usan `test_yaw_damper_cl` y `test_flight_management`). `Flight_Task_5ms` la llama adentro | Volverla `static` y migrar los tests (otro PR) |
| D8 | Tareas lentas | `Flight_Task_30ms/150ms/500ms` vacías en el FM, llamadas desde `Task_30/150/500ms` de `main.c`. Se borran los prototipos `Flight_Control_*_Tier` | Dejar las tareas lentas en `main.c` hasta F5 |

---

## 4. Línea base de regresión (ANTES del Paso 1)

`data/*.csv` está en `.gitignore`: las líneas base son locales y **no se commitean**.

```bash
git checkout main && git pull                          # main @ f140925
git checkout -b refactor/flight-task-5ms

# Rutina por defecto (ROUTINE_LAT_DIR: MANUAL todo el vuelo)
cmake -S . -B build_reg_lat >/dev/null && cmake --build build_reg_lat
./build_reg_lat/SIL_Flight_Control > /dev/null
cp data/SIL_sim_servos2.csv data/baseline_f140925_latdir.csv

# Rutina del yaw damper (MANUAL → GYRO_STAB en t = 5 s)
cmake -S . -B build_reg_yd -DCMAKE_C_FLAGS=-DSIM_PILOT_ROUTINE=ROUTINE_YAW_DAMPER >/dev/null
cmake --build build_reg_yd
./build_reg_yd/SIL_Flight_Control > /dev/null
cp data/SIL_sim_servos2.csv data/baseline_f140925_yd.csv
```

Chequeo de cordura: en `baseline_f140925_yd.csv`, la columna `mode` vale 0 y luego 1
desde t = 5 s, y `max|yd_dr_deg| ≈ 3.73`. Si no es así, detenerse.

Si `build_reg_*` no están en `.gitignore`, agregarlos en el Paso 0.

---

## 5. Pasos (un commit cada uno)

### Paso 0 — `chore: plan del refactor de tareas de vuelo`
- Copiar este archivo a `docs/PLAN_FLIGHT_TASKS.md`.
- Agregar `build_reg_*/` al `.gitignore` (sección de builds), si hace falta.
- Generar las dos líneas base de la sección 4. No se commitean.
- **Aceptación:** `git status` limpio salvo los dos archivos; las líneas base existen y
  pasan el chequeo de cordura.

### Paso 1 — `test: regresion bit a bit de todas las columnas para las dos rutinas`
- `test/compare_csv_cols.py`: agregar `--all` (excluyente con `--cols`). Compara todas
  las columnas del baseline y falla si los headers difieren en nombre u orden. El uso
  actual con `--cols` no cambia.
- Nuevo `test/regresion_tareas.sh`, que:
  1. configura y compila `build_reg_lat` y `build_reg_yd` (como en la sección 4);
  2. corre cada uno, copia el CSV a `data/cand_latdir.csv` / `data/cand_yd.csv`;
  3. ejecuta `python3 test/compare_csv_cols.py <baseline> <cand> --all` en los dos;
  4. sale con 1 si alguno falla e imprime un resumen de una línea por rutina.
  - Las rutas de las líneas base van en variables al inicio del script
    (`BASE_LAT=data/baseline_f140925_latdir.csv`, `BASE_YD=…`) para cambiarlas después.
  - `set -euo pipefail`; correr siempre desde la raíz del repo (verificarlo al inicio).
- **Aceptación:** sin cambios de código C, el script da `OK: 65 columnas identicas en
  10000 filas` para las dos rutinas. Prueba negativa manual: editar una celda de una
  copia del baseline debe hacer fallar el script (no se commitea).

### Paso 2 — `refactor(fc): rc_mixer.c/.h con canales RC y mezcla a us`
Mover de `pilot_sim.c/.h` a `src/rc_mixer.c/.h`, **sin cambiar ni una operación**:
- `RC_CH_MAX`, `RC_CH_*`, `SimRc_s`
- `RC_YAW_TO_RUDDER_SIGN` (con su comentario de la convención de Nelson)
- `rc_to_servo_cmds_us()`, `rc_yaw_deg_per_stick()`

`rc_mixer.h` incluye `servo_sim.h` (por `SERVO_SIM_N_SERVOS` y el enum de canales).
`rc_mixer.c` incluye `sim_settings.h` (por `RC_STICK_TO_US`).
`pilot_sim.h` hace `#include "rc_mixer.h"` y conserva `pilotcommand_s`, `RoutineID`,
`pilot_source_commands`, `rc_source`.

- `CMakeLists.txt`: agregar `src/rc_mixer.c` al ejecutable y a **todo** target que hoy
  compila `src/pilot_sim.c` (`test_yaw_damper_cl`, `test_pilot_lateral_servo`).
- Aprovechar para corregir el comentario de `ROUTINE_LAT_DIR` en el enum ("escalones
  de yaw y throttle" → "dobletes de alerón y timón, pitch de regresión").
- **Aceptación:** 0 warnings, ctest completo, `regresion_tareas.sh` OK en las dos rutinas.

### Paso 3 — `refactor(fc): yaw damper en cl_yaw_damper.c/.h`
Extraer la ley de `fm_yaw_damper()`. API propuesta:

```c
/* cl_yaw_damper.h — Yaw damper: delta_r = K_r * r_wo, washout de 1er orden por
   Euler hacia atrás (docs/teoria/control/control_lateral.tex). Ley pura: el
   estado vive en el struct; no conoce modos, RC ni validez del estado. */
typedef struct
{
    float x_wo_dps;   // estado del washout (r de baja frecuencia)
    bool  activo;     // false -> la próxima Update arranca el washout en r
    float r_wo_dps;   // salida de telemetría: r lavada
    float dr_deg;     // salida de telemetría: delta_r de la ley (respecto al trim)
} ClYawDamper;

void  ClYawDamper_Reset(ClYawDamper *yd);   // activo = false, salidas en 0
/* Devuelve delta_r [deg] */
float ClYawDamper_Update(ClYawDamper *yd, float K_r, float tau_wo_s,
                         float r_dps, float dt_s);
```

**Equivalencia exacta.** Copiar las operaciones en el mismo orden y con los mismos tipos:
```c
if (!yd->activo) { yd->x_wo_dps = r_dps; yd->activo = true; }
yd->x_wo_dps += dt_s / (tau_wo_s + dt_s) * (r_dps - yd->x_wo_dps);
yd->r_wo_dps  = r_dps - yd->x_wo_dps;
yd->dr_deg    = K_r * yd->r_wo_dps;
return yd->dr_deg;
```

En `flight_management.c`:
- `static ClYawDamper g_yd;` reemplaza a `g_x_wo_dps` y `g_yd_activo`.
- `fm_yaw_damper()` se queda como **adaptador del FM**. Revisa `st == NULL ||
  !st->valid || |deg_per_stick| < 1e-6` → `ClYawDamper_Reset` y devuelve el pedal.
  Si no, llama a la ley, copia `r_wo_dps` y `dr_deg` a `g_status`, y devuelve
  `yaw_norm + dr_deg / deg_per_stick` (misma expresión que hoy).
- Rama MANUAL: `ClYawDamper_Reset(&g_yd)` en lugar de `g_yd_activo = false`.
- `FM_Init`: `ClYawDamper_Reset(&g_yd)`.
- Cuidado: hoy `g_status.yd_*` se ponen en 0 al inicio de cada `FM_Update_5ms` y la ley
  los sobrescribe. Mantener ese orden. Con estado inválido, hoy los `yd_*` quedan en 0:
  verificar que sigue igual.

Nuevo test `test/test_cl_yaw_damper.c` (agregar a CMake y ctest; solo enlaza
`cl_yaw_damper.c`):
- **T1 arranque sin escalón:** primera `Update` con r = 10 deg/s → `dr_deg == 0` exacto.
- **T2 washout:** primera `Update` con r0 = 2 deg/s y luego r1 = 12 deg/s constante →
  en la k-ésima muestra con r1 (k ≥ 1), `r_wo = (tau/(tau+dt))^k · (r1 − r0)`,
  tolerancia 1e-5.
- **T3 ganancia:** `dr_deg == K_r * r_wo_dps` exacto en cada paso.
- **T4 Reset:** tras `Reset`, la siguiente `Update` vuelve a arrancar sin escalón.
- **T5 equivalencia con la implementación anterior:** dentro del test, copia literal de
  la fórmula vieja con dos `static float` y 2000 pasos con r(t) senoidal más escalones:
  igualdad `==` en cada paso.

- **Aceptación:** `test_flight_management` y `test_yaw_damper_cl` pasan **sin
  modificarlos**, el test nuevo pasa, 0 warnings, regresión OK en las dos rutinas
  (la del yaw damper es la que ejercita este paso).

### Paso 4 — `refactor(fm): Flight_Task_5ms administra el software de vuelo`
El paso grande. Mueve el cuerpo de `Task_5ms` al FM **sin cambiar el orden de las
operaciones ni los tipos**.

**4.1 `flight_management.h`:**
```c
#include "rc_mixer.h"          // SimRc_s, SERVO_SIM_N_SERVOS

/* Entradas de sensores del software de vuelo. En el SIL las llena main.c desde
   SensorsSimData; en el STM32, los drivers. Mismos tipos que SensorsSimData
   para conservar la regresión bit a bit. */
typedef struct
{
    double gyro_radps[3];   // x, y, z
    bool   imu_valid;
    double ahrs_phi_deg, ahrs_theta_deg, ahrs_psi_deg;
    bool   ahrs_valid;
    double pitot_ias_ms;
    bool   pitot_valid;
    double gps_alt_m;
    double gps_vd_ms;
} FC_SensorsIn;
```
- `FM_Config`: agregar `float trim_us[SERVO_SIM_N_SERVOS];` (trim del transmisor; lo
  calcula `main.c` con `aplicar_trim_servo` antes de `FM_Init`).
- `FM_Status`: agregar telemetría de filtros. El log y la consola la leen de aquí:
  ```c
  double gyro_filt_radps[3];
  float  gyro_fc_hz[3];        // corte aplicado (tras el límite de Lpf1)
  double pitot_filt_ms;
  float  pitot_fc_hz;
  bool   pitot_con_muestra;
  ```
  Alternativa si Rafa prefiere no engordar `FM_Status`: `FM_GetFilters(FM_Filters *out)`.
- Prototipos nuevos:
  ```c
  void Flight_Task_5ms(uint32_t now_ms, uint32_t dt_ms,
                       const SimRc_s *rc_sim, int mode_cmd,
                       const FC_SensorsIn *sens,
                       float servo_cmd_us[SERVO_SIM_N_SERVOS]);
  ```
- Borrar los prototipos `Flight_Control_Attitude_Tier/Cruice_Tier/Mission_Tier` y la
  definición vacía de `Flight_Control_Attitude_Tier` en el `.c` (los reemplaza el Paso 6).

**4.2 `flight_management.c`:**
- Estado nuevo, estático: `GyroFilter g_gyro_filter; double g_gyro_filt_radps[3];
  Lpf1 g_pitot_filter; double g_pitot_filt_ms;`
- `FM_Init`: después de copiar `cfg`, llamar
  `GyroFilter_Init(&g_gyro_filter, g_cfg.gyro_fc_hz)`,
  `Lpf1_Init(&g_pitot_filter, g_cfg.pitot_fc_hz)` y poner en 0 `g_gyro_filt_radps` y
  `g_pitot_filt_ms`. Con `cfg == NULL` los cortes quedan en 0 (sin filtro), igual que
  hoy en los tests que llaman `FM_Init(NULL, …)`.
- `fm_state_desde_sensores()` se mueve aquí como `static`, ahora desde `FC_SensorsIn`.
  **Mismas expresiones**:
  `rad2deg = 180.0 / FC_PI` (con `#define FC_PI 3.14159265358979323846`, el **mismo
  literal** que `SENSORS_SIM_PI`; el FM no incluye `sensors_sim.h`),
  `(float)(gyro * rad2deg)`, `vz = (float)(-gps_vd_ms)`, `arsp = (float)g_pitot_filt_ms`,
  `valid = imu_valid && ahrs_valid`.
- `Flight_Task_5ms`, en este orden exacto (es el orden actual de `Task_5ms`):
  1. `(void)now_ms; const float dt_s = (float)dt_ms * 0.001f;`
  2. `if (rc_sim == NULL || sens == NULL || servo_cmd_us == NULL) return;`
  3. `FM_SetArmed(rc_sim->s[RC_CH_ARM] > 0.5f); FM_SetModeSwitch(mode_cmd);`
  4. `fm_in` desde `RC_CH_ROLL/PITCH/YAW/THROTTLE`.
  5. `GyroFilter_Update(&g_gyro_filter, sens->gyro_radps, (double)dt_s, g_gyro_filt_radps);`
  6. `if (sens->pitot_valid) g_pitot_filt_ms = Lpf1_Update(&g_pitot_filter, sens->pitot_ias_ms, (double)dt_s);`
  7. `st = fm_state_desde_sensores(sens, g_gyro_filt_radps); FM_Update_5ms(&fm_in, &st, dt_s);`
  8. `SimRc_s rc_fm = *rc_sim;` sobrescribir ROLL/PITCH/YAW/THROTTLE con `g_status.cmd_out`.
  9. `rc_to_servo_cmds_us(&rc_fm, g_cfg.trim_us, servo_cmd_us);`
  10. Copiar la telemetría de filtros a `g_status` (valores, `fc_hz` de cada `Lpf1` y
      `con_muestra` del pitot).
  - Comentar que `flaps` se ignora (sin superficie de flaps), como hoy.
- Quitar `FM_UNUSED` de lo que ya se use. **No** activar `fm_slew_limit` (es F5).

**4.3 `main.c`:**
- `Task_5ms` queda como adaptador:
  ```c
  static void Task_5ms(uint32_t now_ms, uint32_t dt_ms)
  {
      const FC_SensorsIn sens = fc_sensores_desde_sim(&g_sens_data);
      float servo_cmd_us[SERVO_SIM_N_SERVOS];
      Flight_Task_5ms(now_ms, dt_ms, &g_rc_sim, (int)g_rc_sim.s[RC_CH_MODE],
                      &sens, servo_cmd_us);
      for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i)
          Servos_SetCmdDeltaUs(i, servo_cmd_us[i]);
  }
  ```
  `fc_sensores_desde_sim()` es `static` en `main.c` (es del arnés: traduce simulador →
  FC). Copia los campos sin convertir tipos.
- `task_init_1ms`: `fm_cfg.trim_us[i] = g_trim_us[i]` antes de `FM_Init`. Quitar
  `GyroFilter_Init` y `Lpf1_Init`. Para imprimir los cortes, leerlos con
  `FM_GetStatus` después de `FM_Init`. El texto impreso no cambia.
- Borrar `fm_state_desde_sensores` de `main.c`.
- Log CSV en `Task_1ms`: como `Task_5ms` ya no actualiza los globales de filtros, en
  este mismo paso el log los toma de `fm_status` (ya se obtiene ahí con
  `FM_GetStatus`): `gyro_filt_radps[]`, `gyro_fc_hz[]`, `pitot_filt_ms`,
  `pitot_con_muestra` y `pitot_fc_hz`. El formato del `fprintf` no cambia; en
  particular,
  `pitot_con_muestra ? pitot_filt_ms : (double)g_sens_data.pitot.ias_ms`.
- Borrar los 4 globales (`g_gyro_filter`, `g_gyro_filt_radps`, `g_pitot_filter`,
  `g_pitot_filt_ms`) y los includes de `gyro_filter.h`/`lpf1.h` que queden sin uso.
- Ojo con los tipos al copiar a `FM_Status`: `gyro_fc_hz` y `pitot_fc_hz` son `float`
  (como `Lpf1.fc_hz`) y el `fprintf` los pasa con `(double)`, igual que hoy.

**4.4 `CMakeLists.txt`:**
- `test_flight_management` ahora enlaza `flight_management.c`, `cl_yaw_damper.c`,
  `rc_mixer.c`, `gyro_filter.c`, `lpf1.c`, `servo_sim.c` y `dynamics.c` (lo que pida el
  linker; `lpf1.c` ya enlaza `dynamics.c` en `test_gyro_filter`).
- `test_yaw_damper_cl`: agregar `cl_yaw_damper.c`, `gyro_filter.c` y `lpf1.c`.

**4.5 Tests nuevos en `test/test_flight_management.c`** (los existentes no se tocan):
- **T_FT1 MANUAL = arnés anterior:** con `trim_us = {−12.25, 0, 0, 0}` y un `SimRc_s` con
  sticks arbitrarios y `mode_cmd = 0`, `Flight_Task_5ms` da los mismos µs que
  `rc_to_servo_cmds_us(rc, trim_us, …)` (`==`).
- **T_FT2 modo desde `mode_cmd`:** `mode_cmd = 3` con `rc.s[RC_CH_MODE] = 0` → modo
  `GYRO_STAB`. Documenta que manda `mode_cmd` y no el canal.
- **T_FT3 armado:** `rc.s[RC_CH_ARM] = 1.0f` → `FM_IsArmed()`; `0.4f` → desarmado.
- **T_FT4 pitot inválido retiene:** con `pitot_fc_hz = 5`, dos corridas válidas y una
  con `pitot_valid = false` y otra IAS → `pitot_filt_ms` no cambia.
- **T_FT5 filtros desde la config:** `gyro_fc_hz = {20, 20, 20}`, `pitot_fc_hz = 5` →
  `FM_Status` reporta esos cortes; con `FM_Init(NULL, …)` reporta 0.
- **T_FT6 validez del estado:** `imu_valid = true, ahrs_valid = false` en `GYRO_STAB` →
  el yaw damper no actúa (`yd_dr_deg == 0` y yaw de salida == pedal).
- **T_FT7 entradas NULL:** `Flight_Task_5ms` con `sens == NULL` no escribe
  `servo_cmd_us` (prellenado con un centinela) y no truena.

- **Aceptación:** 0 warnings, ctest completo (con los tests nuevos), regresión **OK en
  las dos rutinas, 65 columnas**. `grep -n "GyroFilter\|Lpf1" src/main.c` vacío.

### Paso 5 — `refactor(main): Task_1ms con fases explicitas`
Sin cambio de comportamiento. Solo hace legible lo que ya está:
- Separar `Task_1ms` en `static` helpers con nombre de fase, **en el mismo orden**:
  `arnes_piloto()` (piloto → `g_rc_sim`), `arnes_servos_planta(dt_s)` (servos, actuadores,
  `FlightSim_Step`), `arnes_sensores(dt_s)`, `arnes_log_csv(now_ms, …)`,
  `arnes_consola(now_ms, …)`.
- Detalle conocido, **NO corregirlo en este plan** (cambia el CSV): el estado
  `du…dtheta` y `X_lat` se leen **antes** de `FlightSim_Step` y los sensores **después**.
  En la misma fila del CSV, el estado corresponde a t − 1 ms y los sensores a t.
  Conservar el orden y dejar un comentario `/* NOTA: … ver TODO 9.x */`.
- Corregir el comentario "por ejemplo 60 segundos" del loop (son
  `SIMULATION_TIME_s`).
- **Aceptación:** regresión OK en las dos rutinas; `Task_1ms` cabe en una pantalla.

### Paso 6 — `feat(fm): esqueleto de Flight_Task_30ms/150ms/500ms`
- En el FM: `Flight_Task_30ms`, `Flight_Task_150ms` y `Flight_Task_500ms`, con la misma
  forma `(uint32_t now_ms, uint32_t dt_ms)`, **vacías**, con comentario de qué irá en
  cada una (30 ms: lazo externo de actitud / wing leveler en F5; 150 ms: rumbo y
  navegación en F6; 500 ms: health y telemetría).
- `main.c`: `Task_30/150/500ms` las llaman.
- Comentario de cabecera en `flight_management.h` con el **patrón para agregar una
  ley**:
  1. `cl_<ley>.c/.h` con struct de estado, `Reset` y `Update` en unidades físicas;
  2. instancia `static` en `flight_management.c`, con `Reset` en `FM_Init` y al salir
     del modo;
  3. llamada desde la `Flight_Task_*` del periodo que le toca, dentro del `switch` de
     modo;
  4. telemetría en `FM_Status`;
  5. `test/test_cl_<ley>.c` + test de lazo cerrado si aplica.
- **Aceptación:** regresión OK en las dos rutinas; 0 warnings.

### Paso 7 — `docs: arquitectura de tareas del software de vuelo`
- `README.md`: sección "Arquitectura de tareas": arnés vs software de vuelo, el
  diagrama de la sección 1, la firma de `Flight_Task_5ms`, `FC_SensorsIn`, `rc_mixer`,
  el patrón de leyes `cl_*` y cómo correr `test/regresion_tareas.sh`.
- `status.md`: fecha, último commit en `main` (hoy dice `13493f2`; ya es `f140925`),
  F4+ ✅, el refactor, la línea base vigente `f140925` (dos rutinas), el conteo de ctest.
- `TODO.md`: agregar los pendientes de la sección 6.
- **Aceptación:** sin cambios de código; los enlaces y nombres del README coinciden
  con el código.

---

## 6. Fuera de alcance (pendientes para después)

- **Desfase de 1 ms del estado en el CSV** (Paso 5). Corregirlo cambia las columnas de
  estado y obliga a sacar una línea base nueva: va en un commit aparte.
- Renombrar `SimRc_s` → `RcInput_s` y desacoplar `rc_mixer` de `servo_sim.h` y de
  `sim_settings.h` para el STM32.
- Volver `FM_Update_5ms` `static` y migrar `test_yaw_damper_cl` a `Flight_Task_5ms`.
- F5: wing leveler (`cl_wing_leveler`), `fm_slew_limit`, aceptar `mode_switch` 1.
- F6: heading hold (`cl_heading_hold`), bloquear el modo 2 sin yaw damper.
- Rutinas de piloto como tabla de eventos en vez de `switch` con `step_cmd`.

---

## 7. Resumen de archivos tocados

| Paso | Archivos |
|---|---|
| 0 | `docs/PLAN_FLIGHT_TASKS.md`, `.gitignore` |
| 1 | `test/compare_csv_cols.py`, `test/regresion_tareas.sh` |
| 2 | `src/rc_mixer.c/.h` (nuevos), `src/pilot_sim.c/.h`, `CMakeLists.txt` |
| 3 | `src/cl_yaw_damper.c/.h` (nuevos), `src/flight_management.c`, `test/test_cl_yaw_damper.c`, `CMakeLists.txt` |
| 4 | `src/flight_management.c/.h`, `src/main.c`, `test/test_flight_management.c`, `CMakeLists.txt` |
| 5 | `src/main.c` |
| 6 | `src/flight_management.c/.h`, `src/main.c` |
| 7 | `README.md`, `status.md`, `TODO.md` |
