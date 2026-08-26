#include "sim_task_management.h"

#include <string.h>

/* Estado interno */
static SimTM_Config g_cfg;
static SimTM_Task   g_tasks[SIM_TASK_COUNT];
static uint32_t     g_now_ms = 0U;

static void run_task_if_due(SimTM_Task *t, uint32_t now_ms)
{
    if (!t || !t->enabled || (t->fn == (SimTaskFn)0)) return;

    /* Ejecutar si ya toca */
    if (now_ms >= t->next_run_ms)
    {
        /* catch-up: correr tantas veces como periodos vencidos */
        uint32_t loops = 0U;
        do
        {
            t->last_run_ms = t->next_run_ms; /* marca “cuando debió correr” */
            t->fn(t->last_run_ms, t->period_ms);
            t->next_run_ms += t->period_ms;

            loops++;
            if (!g_cfg.catch_up_enabled) break;
            if (loops >= g_cfg.catch_up_max_loops) break;

        } while (now_ms >= t->next_run_ms);
    }
}

void SimTM_Init(const SimTM_Config *cfg)
{
    memset(&g_cfg, 0, sizeof(g_cfg));
    memset(&g_tasks, 0, sizeof(g_tasks));
    g_now_ms = 0U;

    if (cfg)
    {
        g_cfg = *cfg;
    }

    if (g_cfg.base_tick_ms == 0U) g_cfg.base_tick_ms = 1U;
    if (g_cfg.catch_up_max_loops == 0U) g_cfg.catch_up_max_loops = 5U;

    /* Inicializa next_run para todos (se setea al registrar) */
}

void SimTM_RegisterTask(SimTaskId id, uint32_t period_ms, SimTaskFn fn, bool enabled)
{
    if (id >= SIM_TASK_COUNT) return;
    if (period_ms == 0U) return;

    g_tasks[id].fn         = fn;
    g_tasks[id].period_ms  = period_ms;
    g_tasks[id].enabled    = enabled;

    /* Primera corrida: al siguiente múltiplo desde “ahora” */
    g_tasks[id].last_run_ms = g_now_ms;

    /* Si quieres que corra inmediatamente en t=0, pon next_run_ms = g_now_ms */
    g_tasks[id].next_run_ms = g_now_ms + period_ms;
}

void SimTM_EnableTask(SimTaskId id, bool enabled)
{
    if (id >= SIM_TASK_COUNT) return;
    g_tasks[id].enabled = enabled;
}

void SimTM_Step(void)
{
    /* Avanza tiempo base */
    g_now_ms += g_cfg.base_tick_ms;

    /* Ejecuta de más rápida a más lenta (reduce jitter en cascada) */
    run_task_if_due(&g_tasks[SIM_TASK_1MS],   g_now_ms);
    run_task_if_due(&g_tasks[SIM_TASK_5MS],   g_now_ms);
    run_task_if_due(&g_tasks[SIM_TASK_30MS],  g_now_ms);
    run_task_if_due(&g_tasks[SIM_TASK_150MS], g_now_ms);
    run_task_if_due(&g_tasks[SIM_TASK_500MS], g_now_ms);
}

uint32_t SimTM_NowMs(void)
{
    return g_now_ms;
}

const SimTM_Task* SimTM_GetTask(SimTaskId id)
{
    if (id >= SIM_TASK_COUNT) return (const SimTM_Task*)0;
    return &g_tasks[id];
}
