#include "rc_mixer.h"
#include <stddef.h>         // NULL
#include "sim_settings.h"   // RC_STICK_TO_US

/* Signo del timón, convención de Nelson: delta_r > 0 = borde de salida a la
   izquierda -> fuerza lateral a la derecha y N_dr < 0 (nariz a la izquierda).
   Pedal derecho (yaw > 0) debe llevar la nariz a la derecha -> delta_r < 0. */
#define RC_YAW_TO_RUDDER_SIGN   (-1.0f)


float rc_yaw_deg_per_stick(float rudder_k_deg_per_us)
{
    return rudder_k_deg_per_us * RC_STICK_TO_US * RC_YAW_TO_RUDDER_SIGN;
}

void rc_to_servo_cmds_us(const SimRc_s *rc,
                         const float trim_us[SERVO_SIM_N_SERVOS],
                         float cmd_us[SERVO_SIM_N_SERVOS])
{
    if (rc == NULL || cmd_us == NULL) return;

    cmd_us[SERVO_AILERON]  = rc->s[RC_CH_ROLL]     * RC_STICK_TO_US;  // Canal 0: Roll -> Aileron
    cmd_us[SERVO_ELEVATOR] = rc->s[RC_CH_PITCH]    * RC_STICK_TO_US;  // Canal 1: Pitch -> Elevator
    cmd_us[SERVO_RUDDER]   = rc->s[RC_CH_YAW]      * RC_STICK_TO_US * RC_YAW_TO_RUDDER_SIGN;  // Canal 2: Yaw -> Rudder
    cmd_us[SERVO_THROTTLE] = rc->s[RC_CH_THROTTLE] * RC_STICK_TO_US;  // Canal 3: Throttle

    /* + trim del transmisor: stick al centro -> superficie en trim */
    if (trim_us != NULL) {
        for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
            cmd_us[i] += trim_us[i];
        }
    }
}
