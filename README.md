# SIL_Flight_Control

Simulador SIL (Software-In-the-Loop) de control de vuelo: dinámica longitudinal
de aeronave en espacio de estados, integrada con RK4, con simulación de servos,
sensores y un scheduler cooperativo que imita los periodos de tarea de un RTOS.

Estado del vector: `X = [du, dw, dq, dtheta]`.
Modelo activo: `SS_v1` (4.7 kg, V0 = 36 m/s, 1000 m, gamma0 = -15 deg).

## Estructura

```
SIL_Flight_Control/
├── CMakeLists.txt      Configuración de compilación
├── src/                Código C (10 módulos)
├── python/             Scripts de análisis y graficación
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
| `dynamics`             | Matrices A/B, RK4, señales de prueba, saturación      |
| `dynamic_models`       | Catálogo de modelos (`SS_v1`, `SS_v2`)                |
| `sensors_sim`          | Pitot, IMU, GPS y altímetro láser, con ruido y bias   |
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

## Graficar los resultados

Desde Spyder (Anaconda), abrir y ejecutar `python/plot_state_exp_v3.py`, que es
el script más completo y actualizado. Resuelve la ruta del CSV a partir de su
propia ubicación, así que funciona sin importar el directorio de trabajo.

Requiere `pandas`, `numpy`, `matplotlib` y `plotly`.

## Configuración de la simulación

En `src/sim_settings.h`:

- `SIMULATION_TIME_s` — duración simulada (por defecto 10 s)
- `SYSTEM_SIM_ENV` — `SIM_PLATFORM_PC` (con `printf` y CSV) o `SIM_PLATFORM_RTOS`
- `AIRCRAFT_SIM` — aeronave seleccionada

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
