#include "pilot_sim.h"

//prototipos
float step_cmd(uint32_t now_ms, uint32_t time_ms , float value);

pilotcommand_s pilot_source_commands(uint32_t now_ms, unsigned id_routine)
{
    //
    pilotcommand_s current_commands;

    switch(id_routine){
        case ROUTINE_LONGITUDINAL:
            current_commands.roll       = 0.0 + step_cmd(now_ms, 40u, 1.0);// + step_cmd(now_ms, 300u, -1.0);
            current_commands.pitch      = 0.0 + step_cmd(now_ms, 50u, 1.0) + step_cmd(now_ms, 150u, -2.0) + step_cmd(now_ms, 200u, 1.0);
            current_commands.yaw        = 0.0;
            current_commands.throttle   = 0.0;

            current_commands.mode_switch= 0;
            current_commands.arm_switch = 0;

            current_commands.flaps      = 0.0;
            break;
        case ROUTINE_LAT_DIR:
            /* pitch identico a ROUTINE_LONGITUDINAL: los canales estan
               desacoplados, asi que el estado longitudinal debe coincidir con
               la linea base de esa rutina (regresion). */
            current_commands.pitch      = 0.0 + step_cmd(now_ms, 50u, 1.0) + step_cmd(now_ms, 150u, -2.0) + step_cmd(now_ms, 200u, 1.0);
            /* roll: doblete de aleron entre 1.0 y 3.0 s; 0.02 de stick -> 10 us
               -> ~1.8 deg (~0.031 rad) en el aleron (k=0.18 deg/us). */
            current_commands.roll       = step_cmd(now_ms, 1000u, 0.02) + step_cmd(now_ms, 2000u, -0.04) + step_cmd(now_ms, 3000u, 0.02);
            /* yaw: doblete de timon entre 5.0 y 7.0 s; 0.03 de stick -> 15 us
               -> ~2.7 deg en el rudder (k=0.18 deg/us, tau=0.07 s). */
            current_commands.yaw        = step_cmd(now_ms, 5000u, 0.03) + step_cmd(now_ms, 6000u, -0.06) + step_cmd(now_ms, 7000u, 0.03);
            /* throttle: la planta no usa throttle */
            current_commands.throttle   = 0.0;

            current_commands.mode_switch= 0;
            current_commands.arm_switch = 0;

            current_commands.flaps      = 0.0;
            break;
        case ROUTINE_ESTATICA:
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
