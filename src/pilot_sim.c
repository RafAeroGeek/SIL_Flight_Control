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
            /* roll/pitch identicos a ROUTINE_LONGITUDINAL: la planta se integra
               solo con delta_elv, asi que el estado debe coincidir con esa
               rutina (criterio de aceptacion). Solo yaw y throttle cambian. */
            current_commands.roll       = 0.0 + step_cmd(now_ms, 40u, 1.0);
            current_commands.pitch      = 0.0 + step_cmd(now_ms, 50u, 1.0) + step_cmd(now_ms, 150u, -2.0) + step_cmd(now_ms, 200u, 1.0);
            /* yaw: doblete repartido entre 1.0 y 4.0 s; 0.14 de stick -> ~70 us
               -> ~+/-12.6 deg en el rudder (k=0.18 deg/us, tau=0.07 s). */
            current_commands.yaw        = step_cmd(now_ms, 1000u, 0.14) + step_cmd(now_ms, 2500u, -0.28) + step_cmd(now_ms, 4000u, 0.14);
            /* throttle: pulso entre 6.0 y 8.5 s; 0.44 de stick -> ~220 us
               -> ~0.44 normalizado a la salida (k=0.002, tau=0.20 s). */
            current_commands.throttle   = step_cmd(now_ms, 6000u, 0.44) + step_cmd(now_ms, 8500u, -0.44);

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
