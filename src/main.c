#include <stdint.h>
#include <string.h>
#include <math.h>


#include "sim_settings.h"
#include "dynamic_models.h"
#include "dynamics.h"
#include "flight_sim.h"
#include "sim_PWM_processing.h"
#include "sim_task_management.h"
#include "servo_sim.h"
#include "pilot_sim.h"
#include "sensors_sim.h"

#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
#include <stdio.h>
#endif

/* Los identificadores de canal (SERVO_ELEVATOR, ...) viven en servo_sim.h
   como enum ServoChannel_e: fuente unica de verdad del mapeo canal->superficie. */

/* ====== Globals SIM ====== */
static int g_Total_Time_ms = SIMULATION_TIME_ms;
static FlightSim g_sim;
static Params g_params;          // parámetros accesibles desde tareas
static FM_Actuators g_act;
static double g_A[4][4];
static double g_B[4];
static SensorsSim g_sensors;
static SensorsSimData g_sens_data;

// CSV
#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
FILE *g_fp = NULL;
const char *csv_path = "data/SIL_sim_servos2.csv";
#endif




/* ====== Prototipos ====== */
static void print_params(const Params *p);

static void Task_1ms(uint32_t now_ms, uint32_t dt_ms);
static void Task_5ms(uint32_t now_ms, uint32_t dt_ms);
static void Task_30ms(uint32_t now_ms, uint32_t dt_ms);
static void Task_150ms(uint32_t now_ms, uint32_t dt_ms);
static void Task_500ms(uint32_t now_ms, uint32_t dt_ms);


static void task_init_1ms(void);

/**
  * @fn  	task_init_1ms
  * @brief  Inicializaciones para tarea de 1 milisegundo
  */
void task_init_1ms(void)
{
    //

     /* 1) Cargar modelo */
    g_params = get_dynamic_model(DYN_MODEL_SS_V1);

    /* 2) Inicializar planta (t, X, H, deltas, etc.) */
    FlightSimInit ic = {0};

    ic.zero_all_first = true;
    ic.build_ss_matrices = true;

    ic.t0_s     = 0.0;
    ic.X0[0]    = 0.0; ic.X0[1] = 0.0; ic.X0[2] = 0.0; ic.X0[3] = 0.0;  // [du,dw,dq,dtheta]
    ic.H0_m     = (double)g_params.Alt_m;

    ic.delta_elv0 = 0.0;
    ic.delta_ail0 = 0.0;
    ic.delta_rud0 = 0.0;
    ic.throttle0  = 0.0;

    ic.w_g0 = 0.0;

    FlightSim_Init(&g_sim, &g_params, &ic);

    SensorsSimConfig scfg = {0};

    scfg.lat0_deg = 20.6736;     // ejemplo Guadalajara/Zapopan
    scfg.lon0_deg = -103.3440;
    scfg.alt0_m   = g_params.Alt_m;

    scfg.ground_alt_m = 0.0;

    scfg.pitot_enabled = true;
    scfg.imu_enabled   = true;
    scfg.gps_enabled   = true;
    scfg.laser_enabled = true;

    scfg.random_seed = 1234u;

    scfg.noise.enable_noise = true;

    scfg.noise.pitot_noise_std_ms = 0.20;
    scfg.noise.pitot_bias_ms      = 0.00;

    scfg.noise.gyro_noise_std_radps = 0.002;
    scfg.noise.gyro_bias_x_radps    = 0.0;
    scfg.noise.gyro_bias_y_radps    = 0.0;
    scfg.noise.gyro_bias_z_radps    = 0.0;

    scfg.noise.accel_noise_std_mps2 = 0.05;
    scfg.noise.accel_bias_x_mps2    = 0.0;
    scfg.noise.accel_bias_y_mps2    = 0.0;
    scfg.noise.accel_bias_z_mps2    = 0.0;

    scfg.noise.gps_pos_noise_std_m = 1.5;
    scfg.noise.gps_vel_noise_std_ms = 0.10;
    scfg.noise.gps_alt_noise_std_m = 2.0;

    scfg.noise.laser_noise_std_m = 0.03;
    scfg.noise.laser_bias_m      = 0.0;

    SensorsSim_Init(&g_sensors, &scfg, &g_sim);

    FlightSim_Actuators_init(&g_act, &g_params);

    /* 3) Servos */
    Servos_Init_All();

    #if(SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
        /* 4) (Opcional) imprimir A,B desde el sim */

        imprimir_matriz_4x4(g_sim.A, g_sim.B);

        printf("Actuadores inicializados:\n");
        printf("  Aileron:  %.2f deg\n", g_act.aileron);
        printf("  Elevator: %.2f deg\n", g_act.elevator);
        printf("  Rudder:   %.2f deg\n", g_act.rudder);
        printf("  Throttle: %.3f [0-1]\n", g_act.throttle);

         /* 5) Abrir CSV */
        g_fp = fopen(csv_path, "w");
        if (g_fp == NULL) {
            perror("fopen csv");
        } else {
            /* Header CSV (solo servo) */
            fprintf(g_fp, "t_s,"                          // tiempo
                         "elev_cmd_us,ail_cmd_us,rud_cmd_us,thro_cmd_us,"  // comandos us
                         "rc_pitch,rc_roll,rc_yaw,rc_throttle,rc_flaps,"   // señales RC
                         "y_elev_deg,y_ail_deg,y_rud_deg,y_thro_norm,"     // salidas
                         "u_elev_us,u_ail_us,u_rud_us,u_thro_us,"          // comandos efectivos
                         "pilot_roll,pilot_pitch,pilot_yaw,pilot_throttle," // comandos originales
                         "mode,arm,"                                         // modos
                         "du_mps,dw_mps,dq_radps,dtheta_rad,"            // Estado longitudinal
                        "pitot_ms,"                                         // arsp
                        "gyro_x_radps,gyro_y_radps,gyro_z_radps,"           // giros
                        "acc_x_mps2,acc_y_mps2,acc_z_mps2,"                 // aceleraciones
                        "gps_lat_deg,gps_lon_deg,gps_alt_m,"                // GPS data
                        "gps_vn_ms,gps_ve_ms,gps_vd_ms,"
                        "laser_alt_m \n" );                                   // Laser

            fflush(g_fp);
            printf("CSV abierto: %s\n", csv_path);
        }

    #elif(SYSTEM_SIM_ENV == SIM_PLATFORM_RTOS)
        // No-op: sin printf, sin archivo

    #else
        #error "Plataforma no definida"
    #endif

}


/* ============================================================
 * main
 * ============================================================ */
int main(int argc, char **argv)
{
    /* Config scheduler sim */
    SimTM_Config cfg = {
        .base_tick_ms = 1U,
        .catch_up_enabled = true,
        .catch_up_max_loops = 10U
    };
    SimTM_Init(&cfg);

    /* Registrar tareas */
    SimTM_RegisterTask(SIM_TASK_1MS,   1U,   Task_1ms,   true);
    SimTM_RegisterTask(SIM_TASK_5MS,   5U,   Task_5ms,   true);
    SimTM_RegisterTask(SIM_TASK_30MS,  30U,  Task_30ms,  true);
    SimTM_RegisterTask(SIM_TASK_150MS, 150U, Task_150ms, true);
    SimTM_RegisterTask(SIM_TASK_500MS, 500U, Task_500ms, true);

    // inicializaciones
    task_init_1ms();

    /* Parámetros */
    #if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
        print_params(&g_params);
    #endif

    /* Loop de simulación: por ejemplo 60 segundos a 1 ms */
    const uint32_t t_end_ms = g_Total_Time_ms;
    while (SimTM_NowMs() < t_end_ms)
    {
        SimTM_Step();
    }

    #if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
        /* Cerrar CSV */
        if (g_fp != NULL) {
            fflush(g_fp);
            fclose(g_fp);
            g_fp = NULL;
            printf("CSV cerrado.\n");
        }
    #endif

    return 0;
}

/* ===============================
 * Tareas
 * =============================== */

static void Task_1ms(uint32_t now_ms, uint32_t dt_ms)
{
    const float dt_s = (float)dt_ms * 0.001f;
    const float t_s = (float)now_ms * 0.001f;
    static pilotcommand_s pilot;
    SimRc_s rc_sim = {0};

     /* Comandos ejemplo (delta PWM us respecto a 1500) */

     pilot = pilot_source_commands(now_ms, SIM_PILOT_ROUTINE);

     rc_sim = rc_source(pilot);

     /* Comandos para cada servo basados en los canales RC */
     /* Canales RC en unidades normalizadas -> delta PWM [us] (RC_STICK_TO_US) */
     float aile_cmd = rc_sim.s[RC_CH_ROLL]     * RC_STICK_TO_US;  // Canal 0: Roll -> Aileron
     float elev_cmd = rc_sim.s[RC_CH_PITCH]    * RC_STICK_TO_US;  // Canal 1: Pitch -> Elevator
     float rud_cmd  = rc_sim.s[RC_CH_YAW]      * RC_STICK_TO_US;  // Canal 2: Yaw -> Rudder
     float thro_cmd = rc_sim.s[RC_CH_THROTTLE] * RC_STICK_TO_US;  // Canal 3: Throttle

     /* Opcional: flaps, modo, armado */
     float flaps_cmd    = rc_sim.s[RC_CH_FLAPS];
     int mode_cmd       = (int)rc_sim.s[RC_CH_MODE];
     bool arm_cmd       = (rc_sim.s[RC_CH_ARM] > 0.5f);

     /* Enviar comandos a los servos/actuadores */
     Servos_SetCmdDeltaUs(SERVO_AILERON, aile_cmd);   // Roll
     Servos_SetCmdDeltaUs(SERVO_ELEVATOR, elev_cmd);  // Pitch
     Servos_SetCmdDeltaUs(SERVO_RUDDER, rud_cmd);     // Yaw
     Servos_SetCmdDeltaUs(SERVO_THROTTLE, thro_cmd);  // Throttle (si es servo o ESC)

     /* “Tarea” 1 ms */
     Servos_Update_1ms();

     /* Leer salidas */
     /* Leer salidas de los servos (en grados o valor normalizado) */
     float y_elev = Servos_GetOutDeg(SERVO_ELEVATOR);
     float y_ail  = Servos_GetOutDeg(SERVO_AILERON);
     float y_rud  = Servos_GetOutDeg(SERVO_RUDDER);
     float y_thro = Servos_GetOutDeg(SERVO_THROTTLE);

     /* También leer los comandos efectivos (con saturación, etc.) */
     float u_elev = Servos_GetCmdDeltaUs(SERVO_ELEVATOR);
     float u_ail  = Servos_GetCmdDeltaUs(SERVO_AILERON);
     float u_rud  = Servos_GetCmdDeltaUs(SERVO_RUDDER);
     float u_thro = Servos_GetCmdDeltaUs(SERVO_THROTTLE);

     double du     = g_sim.X[0];
     double dw     = g_sim.X[1];
     double dq     = g_sim.X[2];
     double dtheta = g_sim.X[3];

     FlightSim_GetActuators(&g_act, y_ail, y_elev, y_rud, y_thro);
     FlightSim_SetActuatorsFromFM(&g_sim, &g_act);
     FlightSim_Step(&g_sim, dt_s);

    SensorsSim_Update(&g_sensors, &g_sim, dt_s);
    SensorsSim_GetData(&g_sensors, &g_sens_data);


     #if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
        /* Logging CSV: tiempo, elev_cmd, u_elev, y_elev */
        if (g_fp != NULL)
        {

            /* Formato CSV con todas las señales:
               tiempo,
               comandos_us: elev, ail, rud, thro,
               salidas_deg: elev, ail, rud, thro,
               valores_rc: pitch, roll, yaw, throttle, flaps, mode, arm,
               comandos_originales: roll, pitch, yaw, throttle del piloto */


            fprintf(g_fp,
                    "%.6f,"   // t_s
                    "%.3f,%.3f,%.3f,%.3f,"   // elev_cmd_us, ail_cmd_us, rud_cmd_us, thro_cmd_us
                    "%.6f,%.6f,%.6f,%.6f,%.3f," // rc_pitch, rc_roll, rc_yaw, rc_throttle, rc_flaps
                    "%.3f,%.3f,%.3f,%.3f,"   // y_elev_deg, y_ail_deg, y_rud_deg, y_thro_perc
                    "%.3f,%.3f,%.3f,%.3f,"   // u_elev_us, u_ail_us, u_rud_us, u_thro_us
                    "%.3f,%.3f,%.3f,%.3f,"   // pilot_roll, pilot_pitch, pilot_yaw, pilot_throttle
                    "%.0f,%.0f,"             // mode, arm
                    "%.6f,%.6f,%.6f,%.6f,"  // du, dw, dq, dtheta
                    "%.6f,"                 // ARSP
                    "%.3f,%.3f,%.3f,"       // giros
                    "%.3f,%.3f,%.3f,"       // acelerometros
                    "%.3f,%.3f,%.3f,"       // gps data
                    "%.3f,%.3f,%.3f,"       // gps vn ve vd
                    "%.6f\n",               // Laser
                    (double)t_s,                                            // 1. tiempo
                    (double)elev_cmd, (double)aile_cmd,                     // comandos us
                    (double)rud_cmd, (double)thro_cmd,                      //
                    (double)rc_sim.s[RC_CH_PITCH],                          // 6. valores RC
                    (double)rc_sim.s[RC_CH_ROLL],
                    (double)rc_sim.s[RC_CH_YAW],
                    (double)rc_sim.s[RC_CH_THROTTLE],
                    (double)rc_sim.s[RC_CH_FLAPS],
                    g_act.elevator, g_act.aileron,                          //11. salidas deg servo - superficie d control
                    g_act.rudder,  g_act.throttle,
                    (double)u_elev, (double)u_ail,                          // 15. señal norm servos deg
                    (double)u_rud,  (double)u_thro,
                    (double)pilot.roll, (double)pilot.pitch,                // 19. comandos originales
                    (double)pilot.yaw, (double)pilot.throttle,
                    (double)pilot.mode_switch, (double)pilot.arm_switch,    // modos
                    (double)du,(double)dw,(double)dq,(double)dtheta ,      // du, dw, dq, dtheta

                    (double)g_sens_data.pitot.airspeed_ms,                  // ARSP

                    (double)g_sens_data.imu.gyro_radps.x,                   // giros
                    (double)g_sens_data.imu.gyro_radps.y,
                    (double)g_sens_data.imu.gyro_radps.z,

                    (double)g_sens_data.imu.accel_mps2.x,                   // acelerometros
                    (double)g_sens_data.imu.accel_mps2.y,
                    (double)g_sens_data.imu.accel_mps2.z,

                    (double)g_sens_data.gps.lat_deg,                        // gps data
                    (double)g_sens_data.gps.lon_deg,
                    (double)g_sens_data.gps.alt_m,

                    (double)g_sens_data.gps.vn_ms,                          // gps vn ve vd
                    (double)g_sens_data.gps.ve_ms,
                    (double)g_sens_data.gps.vd_ms,

                    (double)g_sens_data.laser.range_m );                    // Laser

            /* Flush cada 100 ms */
            if ((now_ms % 100u) == 0u) {
                fflush(g_fp);
            }
        }

        /* Consola cada 50 ms */
        if ((now_ms % 50u) == 0u)
        {
            printf("t=%.3f | CMD[us]: E=%.1f A=%.1f R=%.1f T=%.1f | OUT[deg]: E=%.2f A=%.2f R=%.2f T=%.2f\n",
                   (double)t_s,
                   (double)elev_cmd, (double)aile_cmd, (double)rud_cmd, (double)thro_cmd,
                   (double)y_elev, (double)y_ail, (double)y_rud, (double)y_thro);
        }
    #endif
}

static void Task_5ms(uint32_t now_ms, uint32_t dt_ms)
{
    (void)now_ms;
    const float dt_s = (float)dt_ms * 0.001f;

    /* Inner loops (rate) o “control layer” rápida */


    /* Aplicar a planta (en sim, normalmente lo haces en 1ms como ZOH) */
    /* Plant_SetActuators(g_act); */
}

static void Task_30ms(uint32_t now_ms, uint32_t dt_ms)
{
    (void)now_ms; (void)dt_ms;
    /* Outer loop actitud / setpoint shaping / FBW */
}

static void Task_150ms(uint32_t now_ms, uint32_t dt_ms)
{
    (void)now_ms; (void)dt_ms;
    /* Navegación / ALT_HOLD / ARSP_HOLD (setpoints lentos) */
}

static void Task_500ms(uint32_t now_ms, uint32_t dt_ms)
{
    (void)dt_ms;
    /* Logging / telemetría / health */
    /* printf("[t=%ums] phi=%.2f theta=%.2f psi=%.2f\n", now_ms, g_st.phi_deg, g_st.theta_deg, g_st.psi_deg); */
}


static void print_params(const Params *p)
{
    printf("======================================================================\n");
    printf("DINÁMICA LONGITUDINAL DE AERONAVE - MÉTODO RK4 (C11)\n");
    printf("======================================================================\n");
    printf("Configuración: %s\n", p->name);
    printf("Altitud: %.1f m\n", p->Alt_m);
    printf("Velocidad de vuelo: %.3f m/s\n", p->V0_ms);
    printf("Masa: %.3f kg\n", p->masa_kg);
    printf("gamma_0: %.2f deg\n", p->gamma0_deg);
    printf("---------------------------------------------------------------------\n");
}
