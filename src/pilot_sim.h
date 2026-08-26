#ifndef PILOT_SIM_H_INCLUDED
#define PILOT_SIM_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>

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
    maneuver0 = 0,
    maneuver1,
    // agrega más...
    maneuvertot
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


#endif // PILOT_SIM_H_INCLUDED
