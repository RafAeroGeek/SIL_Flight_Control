/* ===========================================================
 * servo_sim.c
 * =========================================================== */
#include "servo_sim.h"

/* ===================== CONFIG LOCAL ===================== */

static Servo_s g_servos[SERVO_SIM_N_SERVOS];
static float   g_u_delta_us[SERVO_SIM_N_SERVOS];
static float   g_y_out_deg[SERVO_SIM_N_SERVOS];

/* Si quieres clamp de entrada delta PWM (recomendado) */
#ifndef SERVO_SIM_DPWM_LIMIT_US
#define SERVO_SIM_DPWM_LIMIT_US (500.0f) // +/-500us típico
#endif

/* Tabla de configuración (ajústala a tus servos reales).
   Indexada por ServoChannel_e (servo_sim.h): una fila por canal. */
static const ServoCfg_s g_cfg[] =
{
    //                    tau     k(deg/us) dt      y0     ymin    ymax    rate(dps)
    [SERVO_ELEVATOR] =   {0.01f,  0.18f,    0.001f,  0.0f, -25.0f,  25.0f, 900.0f},
    [SERVO_AILERON]  =   {0.01f,  0.18f,    0.001f,  0.0f, -20.0f,  20.0f, 300.0f},
    [SERVO_RUDDER]   =   {0.07f,  0.18f,    0.001f,  0.0f, -30.0f,  30.0f, 250.0f},
    /* PROPUESTO - REVISAR: canal throttle normalizado [0,1]. No es un servo de
       posición: la "unidad real" es fracción [0,1], pero se reutilizan los
       campos _deg del struct tal cual. k=1 => y_ss = 1 * u con u en [0,1]. */
    [SERVO_THROTTLE] =   {0.20f,  1.0f,     0.001f,  0.0f,   0.0f,   1.0f,   2.0f},
};

_Static_assert((sizeof(g_cfg) / sizeof(g_cfg[0])) == SERVO_SIM_CH_COUNT,
               "g_cfg[] debe tener exactamente una fila por canal de ServoChannel_e");
_Static_assert(SERVO_SIM_CH_COUNT == SERVO_SIM_N_SERVOS,
               "SERVO_SIM_N_SERVOS debe coincidir con el conteo de ServoChannel_e");

/* ===================== HELPERS ===================== */

static float clampf(float x, float mn, float mx)
{
    if (x < mn) return mn;
    if (x > mx) return mx;
    return x;
}

/* Rate limiter: limita cambio de y a +/- rate_limit_dps * dt */
static float rate_limit(float y_prev, float y_cmd, float rate_limit_dps, float dt_s)
{
    if ((rate_limit_dps <= 0.0f) || (dt_s <= 0.0f)) {
        return y_cmd;
    }

    const float dy_max = rate_limit_dps * dt_s;
    float dy = y_cmd - y_prev;

    if (dy >  dy_max) dy =  dy_max;
    if (dy < -dy_max) dy = -dy_max;

    return y_prev + dy;
}

/* Dinámica 1er orden: ydot = a*y + b*u */
static void servo_dynamics(double y, double u, double a, double b, double *ydot)
{
    *ydot = a*y + b*u;
}

/* ===================== SERVO INDIVIDUAL ===================== */

void Servo_Init(Servo_s *s,
                float tau_s,
                float k_deg_per_us,
                float dt_s,
                float y0_deg,
                float y_min_deg,
                float y_max_deg,
                float rate_limit_dps)
{
    if (s == NULL) return;

    s->tau_s          = tau_s;
    s->k_deg_per_us   = k_deg_per_us;
    s->dt_s           = dt_s;

    s->y_min_deg      = y_min_deg;
    s->y_max_deg      = y_max_deg;

    s->rate_limit_dps = rate_limit_dps;

    s->y_deg          = clampf(y0_deg, y_min_deg, y_max_deg);
    s->is_init        = true;
}

float Servo_Update(Servo_s *s, float u_delta_us)
{
    if (s == NULL) return 0.0f;
    if (!s->is_init) return s->y_deg;

    /* Limitar comando (opcional pero muy útil) */
    u_delta_us = clampf(u_delta_us, -SERVO_SIM_DPWM_LIMIT_US, SERVO_SIM_DPWM_LIMIT_US);

    if (s->dt_s <= 0.0f) return s->y_deg;

    /* Caso tau inválido: usar estática + rate limit */
    if (s->tau_s <= 0.0f)
    {
        float y_cmd = s->k_deg_per_us * u_delta_us;
        y_cmd = clampf(y_cmd, s->y_min_deg, s->y_max_deg);

        float y_rl = rate_limit(s->y_deg, y_cmd, s->rate_limit_dps, s->dt_s);
        s->y_deg = clampf(y_rl, s->y_min_deg, s->y_max_deg);
        return s->y_deg;
    }

    const double a = -1.0 / (double)s->tau_s;
    const double b =  (double)s->k_deg_per_us / (double)s->tau_s;

    double y_next = (double)s->y_deg;

    rk4_step_1st(servo_dynamics,
                 (double)s->y_deg,
                 (double)u_delta_us,
                 (double)s->dt_s,
                 a, b,
                 &y_next);

    const float y_rk4 = (float)y_next;

    /* Rate limit (deg/s) */
    const float y_rl = rate_limit(s->y_deg, y_rk4, s->rate_limit_dps, s->dt_s);

    /* Saturación mecánica final */
    s->y_deg = clampf(y_rl, s->y_min_deg, s->y_max_deg);

    return s->y_deg;
}

/* ===================== MULTI-SERVO API ===================== */

void Servos_Init_All(void)
{
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i)
    {
        Servo_Init(&g_servos[i],
                   g_cfg[i].tau_s,
                   g_cfg[i].k_deg_per_us,
                   g_cfg[i].dt_s,
                   g_cfg[i].y0_deg,
                   g_cfg[i].y_min_deg,
                   g_cfg[i].y_max_deg,
                   g_cfg[i].rate_limit_dps);

        g_u_delta_us[i] = 0.0f;
        g_y_out_deg[i]  = g_servos[i].y_deg;
    }
}

void Servos_SetCmdDeltaUs(unsigned idx, float u_delta_us)
{
    if (idx >= SERVO_SIM_N_SERVOS) return;
    g_u_delta_us[idx] = u_delta_us;
}

float Servos_GetCmdDeltaUs(unsigned idx)
{
    if (idx >= SERVO_SIM_N_SERVOS) return 0.0f;
    return g_u_delta_us[idx];
}

float Servos_GetOutDeg(unsigned idx)
{
    if (idx >= SERVO_SIM_N_SERVOS) return 0.0f;
    return g_y_out_deg[idx];
}

void Servos_GetOutDeg_All(float y_deg_out[SERVO_SIM_N_SERVOS])
{
    if (y_deg_out == NULL) return;

    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i)
    {
        y_deg_out[i] = g_y_out_deg[i];
    }
}

void Servos_Update_1ms(void)
{
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i)
    {
        g_y_out_deg[i] = Servo_Update(&g_servos[i], g_u_delta_us[i]);
    }
}
