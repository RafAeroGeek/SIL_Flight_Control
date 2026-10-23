#ifndef PILOT_SIM_H_INCLUDED
#define PILOT_SIM_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

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

typedef enum
{
    ROUTINE_NULA         = 0,   // todo a cero
    ROUTINE_LONGITUDINAL = 1,   // roll + pitch: rutina de regresión actual
    ROUTINE_ESTATICA     = 2,   // todo a cero (compat con el case 2 previo)
    ROUTINE_LAT_DIR      = 3,   // escalones de yaw y throttle
    ROUTINE_COUNT
} RoutineID;

typedef struct
{
    //Comandos de vuelo primarios
    float roll;
    float pitch;
    float yaw;
    float throttle;

    //Comandos de vuelo secundarios
    float flaps;


    //Activaciones
    int mode_switch;
    bool  arm_switch;

}pilotcommand_s;


pilotcommand_s pilot_source_commands(uint32_t now_ms, unsigned id_routine);

SimRc_s rc_source(pilotcommand_s pilot);

/* Ruteo canal RC -> servo: sticks normalizados -> delta PWM [us] de cada
   servo (indice ServoChannel_e), mas el trim del transmisor trim_us[] por
   servo (NULL = sin trim). Roll -> alerón, pitch -> elevador, yaw -> timón
   (con signo invertido: convención de Nelson, ver RC_YAW_TO_RUDDER_SIGN),
   throttle -> ESC. */
void rc_to_servo_cmds_us(const SimRc_s *rc,
                         const float trim_us[SERVO_SIM_N_SERVOS],
                         float cmd_us[SERVO_SIM_N_SERVOS]);


#endif // PILOT_SIM_H_INCLUDED
