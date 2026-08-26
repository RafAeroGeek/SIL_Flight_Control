#include "sim_PWM_processing.h"

static SimPwmBank_t g_pwm;

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Si quieres conservar tu lógica:
static uint16_t sim_PWM_Invert(uint16_t v)
{
    // ejemplo típico 1000-2000 us
    const uint16_t lo=1000u, hi=2000u;
    v = clamp_u16(v, lo, hi);
    return (uint16_t)(lo + (hi - v));
}

void sim_PWM_Init(void)
{
    for (uint8_t i=0; i<SIM_PWM_CH_MAX; i++) g_pwm.s[i] = 1500u;
}

void sim_PWM_Manager(uint16_t value, uint8_t channel)
{
    // canal válido 1..14
    if ((channel < 1u) || (channel > SIM_PWM_CH_MAX)) return;

    // inversión igual que en STM (ajusta tus canales)
    if ((channel == 4u) || (channel == 2u))
    {
        value = sim_PWM_Invert(value);
    }

    // opcional: clamp para sim
    value = clamp_u16(value, 1000u, 2000u);

    // escribir S(channel)
    g_pwm.s[channel - 1u] = value;
}

const SimPwmBank_t* sim_PWM_GetBank(void)
{
    return &g_pwm;
}

