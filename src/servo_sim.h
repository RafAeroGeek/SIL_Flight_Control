/* ===========================================================
 * servo_sim.h
 * Simulación de servo 1er orden con RK4 + rate limiter (deg/s)
 * Entrada: delta PWM [us] respecto a 1500us (u_delta_us)
 * Salida: posición [deg]
 * =========================================================== */
#ifndef SERVO_SIM_H_INCLUDED
#define SERVO_SIM_H_INCLUDED

#include <stdbool.h>
#include <stddef.h>

#include "dynamics.h"   // rk4_step_1st, DynFunc_1st, etc.

#ifdef __cplusplus
extern "C" {
#endif

/* Fuente unica de verdad del mapeo canal -> superficie de control.
   La tabla g_cfg[] en servo_sim.c se indexa con este enum. Para agregar un
   servo: anadir el enumerador aqui y su fila en g_cfg[]; el _Static_assert
   de servo_sim.c obliga a que ambos concuerden. */
typedef enum
{
    SERVO_ELEVATOR = 0,
    SERVO_AILERON,
    SERVO_RUDDER,
    SERVO_THROTTLE,
    SERVO_SIM_CH_COUNT      /* centinela: numero de canales */
} ServoChannel_e;

#ifndef SERVO_SIM_N_SERVOS
#define SERVO_SIM_N_SERVOS (4u)
#endif

typedef struct
{
    // Config
    float tau_s;           // [s] constante de tiempo
    float k_deg_per_us;    // [deg/us] ganancia estática: y_ss = k*u_delta_us
    float dt_s;            // [s] paso de integración

    // Saturación mecánica
    float y_min_deg;       // [deg]
    float y_max_deg;       // [deg]

    // Rate limiter
    float rate_limit_dps;  // [deg/s] 0 => deshabilitado

    // Estado
    float y_deg;           // [deg] salida actual
    bool  is_init;
} Servo_s;

typedef struct
{
    float tau_s;
    float k_deg_per_us;
    float dt_s;
    float y0_deg;
    float y_min_deg;
    float y_max_deg;
    float rate_limit_dps;
} ServoCfg_s;

/* Inicializa un servo individual */
void Servo_Init(Servo_s *s,
                float tau_s,
                float k_deg_per_us,
                float dt_s,
                float y0_deg,
                float y_min_deg,
                float y_max_deg,
                float rate_limit_dps);

/* Actualiza un servo individual con entrada delta PWM [us] */
float Servo_Update(Servo_s *s, float u_delta_us);

/* ===================== API MULTI-SERVO ===================== */

/* Inicializa todos los servos usando la tabla interna de config */
void Servos_Init_All(void);

/* Igual que Servos_Init_All pero tomando la config desde un arreglo externo
   (p.ej. rellenado desde el JSON de la aeronave). Si cfg == NULL cae a g_cfg[]. */
void Servos_Init_All_Cfg(const ServoCfg_s cfg[SERVO_SIM_N_SERVOS]);

/* Copia la tabla de config por defecto (compilada) al arreglo del usuario,
   base para superponer overrides del JSON. */
void Servos_GetDefaultCfg(ServoCfg_s out[SERVO_SIM_N_SERVOS]);

/* Set del comando por servo (delta PWM en microsegundos) */
void Servos_SetCmdDeltaUs(unsigned idx, float u_delta_us);

/* Get del comando por servo (delta PWM en microsegundos) */
float Servos_GetCmdDeltaUs(unsigned idx);

/* Get de la salida por servo (posición en deg) */
float Servos_GetOutDeg(unsigned idx);

/* Copia de todas las salidas (deg) a un arreglo del usuario (opcional) */
void Servos_GetOutDeg_All(float y_deg_out[SERVO_SIM_N_SERVOS]);

/* Update masivo pensado para llamarse cada 1 ms (o al dt configurado) */
void Servos_Update_1ms(void);

#ifdef __cplusplus
}
#endif

#endif // SERVO_SIM_H_INCLUDED
