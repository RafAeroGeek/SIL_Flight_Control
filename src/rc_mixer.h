#ifndef RC_MIXER_H_INCLUDED
#define RC_MIXER_H_INCLUDED

/* Canales RC y mezcla stick -> delta PWM [us] de cada servo. Es software de
   vuelo (no del piloto simulado): lo usan el FM y el arnés del SIL. */

#include "servo_sim.h"   // ServoChannel_e, SERVO_SIM_N_SERVOS

#define RC_CH_MAX 16u

#define RC_CH_ROLL      0
#define RC_CH_PITCH     1
#define RC_CH_YAW       2
#define RC_CH_THROTTLE  3
#define RC_CH_FLAPS     4
#define RC_CH_MODE      5
#define RC_CH_ARM       6

typedef struct
{
    float s[RC_CH_MAX];   // s[0]=S1 ... s[13]=S14
} SimRc_s;

/* Ruteo canal RC -> servo: sticks normalizados -> delta PWM [us] de cada
   servo (indice ServoChannel_e), mas el trim del transmisor trim_us[] por
   servo (NULL = sin trim). Roll -> alerón, pitch -> elevador, yaw -> timón
   (con signo invertido: convención de Nelson, ver RC_YAW_TO_RUDDER_SIGN),
   throttle -> ESC. */
void rc_to_servo_cmds_us(const SimRc_s *rc,
                         const float trim_us[SERVO_SIM_N_SERVOS],
                         float cmd_us[SERVO_SIM_N_SERVOS]);

/* Deflexión del timón por unidad de stick de yaw [deg], con el mismo ruteo de
   rc_to_servo_cmds_us. Convierte a stick las leyes que calculan delta_r. */
float rc_yaw_deg_per_stick(float rudder_k_deg_per_us);

#endif // RC_MIXER_H_INCLUDED
