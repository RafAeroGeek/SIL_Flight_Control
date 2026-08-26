#include "pilot_sim.h"

//prototipos
float step_cmd(uint32_t now_ms, uint32_t time_ms , float value);

pilotcommand_s pilot_source_commands(uint32_t now_ms, unsigned id_routine)
{
    //
    pilotcommand_s current_commands;

    switch(id_routine){
        case 1:
            current_commands.roll       = 0.0 + step_cmd(now_ms, 40u, 1.0);// + step_cmd(now_ms, 300u, -1.0);
            current_commands.pitch      = 0.0 + step_cmd(now_ms, 50u, 1.0) + step_cmd(now_ms, 150u, -2.0) + step_cmd(now_ms, 200u, 1.0);
            current_commands.yaw        = 0.0;
            current_commands.throttle   = 0.0;

            current_commands.mode_switch= 0;
            current_commands.arm_switch = 0;

            current_commands.flaps      = 0.0;
            break;
        case 2:
            current_commands.roll       = 0.0;
            current_commands.pitch      = 0.0;
            current_commands.yaw        = 0.0;
            current_commands.throttle   = 0.0;

            current_commands.mode_switch= 0;
            current_commands.arm_switch = 0;

            current_commands.flaps      = 0.0;
            break;
        default:
            current_commands.roll       = 0.0;
            current_commands.pitch      = 0.0;
            current_commands.yaw        = 0.0;
            current_commands.throttle   = 0.0;

            current_commands.mode_switch= 0;
            current_commands.arm_switch = 0;

            current_commands.flaps      = 0.0;
            break;
    }
    return current_commands;
}

float step_cmd(uint32_t now_ms, uint32_t time_ms , float value){
    //
    return (now_ms >= time_ms) ?  value : 0.0f;  //
}


SimRc_s rc_source(pilotcommand_s pilot) {
    SimRc_s rc_out = {0};  // Inicializar todo a cero primero

    // Mapeo claro de comandos a canales
    rc_out.s[RC_CH_ROLL]     = pilot.roll;
    rc_out.s[RC_CH_PITCH]    = pilot.pitch;
    rc_out.s[RC_CH_YAW]      = pilot.yaw;
    rc_out.s[RC_CH_THROTTLE] = pilot.throttle;
    rc_out.s[RC_CH_FLAPS]    = pilot.flaps;
    rc_out.s[RC_CH_MODE]     = (float)pilot.mode_switch;
    rc_out.s[RC_CH_ARM]      = pilot.arm_switch ? 1.0f : 0.0f;

    return rc_out;
}
