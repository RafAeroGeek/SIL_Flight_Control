// sim_pwm_processing.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define SIM_PWM_CH_MAX 14u

typedef struct
{
    uint16_t s[SIM_PWM_CH_MAX];   // s[0]=S1 ... s[13]=S14
} SimPwmBank_t;

void sim_PWM_Init(void);
void sim_PWM_Manager(uint16_t value, uint8_t channel);
const SimPwmBank_t* sim_PWM_GetBank(void);
