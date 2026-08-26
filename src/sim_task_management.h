#ifndef SIM_TASK_MANAGEMENT_H
#define SIM_TASK_MANAGEMENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Callback de tarea: el argumento dt_ms es el periodo nominal de esa tarea */
typedef void (*SimTaskFn)(uint32_t now_ms, uint32_t dt_ms);

/* IDs opcionales para debug/telemetría */
typedef enum
{
    SIM_TASK_1MS = 0,
    SIM_TASK_5MS,
    SIM_TASK_30MS,
    SIM_TASK_150MS,
    SIM_TASK_500MS,
    SIM_TASK_COUNT
} SimTaskId;

/* Configuración del “scheduler cooperativo” */
typedef struct
{
    /* Paso base de simulación, en ms. Usualmente 1 ms. */
    uint32_t base_tick_ms;

    /* Si true: si por alguna razón “se saltan ticks”, corre tareas atrasadas
       las veces necesarias para ponerse al corriente. */
    bool catch_up_enabled;

    /* Límite de catch-up para evitar espiral de muerte (por seguridad). */
    uint32_t catch_up_max_loops;
} SimTM_Config;

/* Registro de tareas (periodos fijos) */
typedef struct
{
    SimTaskFn fn;
    uint32_t  period_ms;
    uint32_t  next_run_ms;
    uint32_t  last_run_ms;
    bool      enabled;
} SimTM_Task;

/* API */
void SimTM_Init(const SimTM_Config *cfg);
void SimTM_RegisterTask(SimTaskId id, uint32_t period_ms, SimTaskFn fn, bool enabled);
void SimTM_EnableTask(SimTaskId id, bool enabled);

/* Avanza el tiempo simulado base_tick_ms y ejecuta tareas que correspondan */
void SimTM_Step(void);

/* Accesores */
uint32_t SimTM_NowMs(void);
const SimTM_Task* SimTM_GetTask(SimTaskId id);

#ifdef __cplusplus
}
#endif

#endif /* SIM_TASK_MANAGEMENT_H */

