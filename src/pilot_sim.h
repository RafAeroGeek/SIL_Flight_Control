#ifndef PILOT_SIM_H_INCLUDED
#define PILOT_SIM_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

#include "rc_mixer.h"    // SimRc_s, RC_CH_*, rc_to_servo_cmds_us, rc_yaw_deg_per_stick

typedef enum
{
    ROUTINE_NULA         = 0,   // todo a cero
    ROUTINE_LONGITUDINAL = 1,   // roll + pitch: rutina de regresión actual
    ROUTINE_ESTATICA     = 2,   // todo a cero (compat con el case 2 previo)
    ROUTINE_LAT_DIR      = 3,   // dobletes de alerón y timón, pitch de regresión
    ROUTINE_YAW_DAMPER   = 4,   // mismo doblete de timón en MANUAL y en GYRO_STAB
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

#endif // PILOT_SIM_H_INCLUDED
