#ifndef FLIGHT_MANAGEMENT_H
#define FLIGHT_MANAGEMENT_H

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * Comandos del piloto
 * =========================
 * Ejes: [-1, +1]
 * Throttle: [0, 1]
 */
typedef struct
{
    float roll_norm;      // [-1, +1]
    float pitch_norm;     // [-1, +1]
    float yaw_norm;       // [-1, +1]
    float throttle;       // [ 0,  1]
} FM_CommandsNorm;

/* =========================
 * Setpoints activos
 * ========================= */
typedef struct
{
    // Actitud
    float phi_cmd_deg;       // [deg]
    float theta_cmd_deg;     // [deg]

    // Tasas (inner loops)
    float p_cmd_dps;         // [deg/s]
    float q_cmd_dps;         // [deg/s]

    // Yaw: heading hold + yaw-rate damping
    float psi_cmd_deg;       // heading command [deg] (outer loop)
    float r_cmd_dps;         // yaw-rate command [deg/s] (inner loop)

    // Navegación/energía
    float alt_cmd_m;         // [m]
    float arsp_cmd_ms;       // [m/s]
    float vz_cmd_ms;         // [m/s] opcional
} FM_Setpoints;

/* =========================
 * Estado estimado/medido
 * ========================= */
typedef struct
{
    float phi_deg;
    float theta_deg;
    float psi_deg;           // heading [deg]

    float p_dps;
    float q_dps;
    float r_dps;             // yaw-rate [deg/s]

    float alt_m;
    float vz_ms;
    float arsp_ms;

    bool  valid;
} FM_State;

/* =========================
 * Actuadores (salida final)
 * ========================= */
typedef struct
{
    float aileron;   // [-delta_a_min   , +delta_a_max  ]
    float elevator;  // [-delta_e_min   , +delta_e_max  ]
    float rudder;    // [-delta_r_min   , +delta_r_max  ]
    float throttle;  // [-delta_thr_min , +delta_thr_max]
} FM_Actuators;

/* =========================
 * Modos
 * ========================= */
typedef enum
{
    FM_MODE_MANUAL = 0,
    FM_MODE_GYRO_STAB,
    FM_MODE_ATTITUDE,
    FM_MODE_FLY_BY_WIRE,
    FM_MODE_ALT_HOLD,
    FM_MODE_ARSP_HOLD,
    FM_MODE_FAILSAFE
} FM_Mode;

/* =========================
 * Configuración
 * ========================= */
typedef enum
{
    FM_YAW_MODE_RATE = 0,        // stick -> r_cmd (solo damping)
    FM_YAW_MODE_HEADING_HOLD     // psi hold -> r_cmd, con override por stick
} FM_YawMode;

typedef struct
{
    // Envelope
    float max_bank_deg;
    float max_pitch_deg;
    float max_yaw_rate_dps;
    float max_roll_rate_dps;
    float max_pitch_rate_dps;

    // Throttle [0,1]
    float thr_min;          // ej. 0.0
    float thr_max;          // ej. 1.0

    // Slew limits (por segundo)
    float surf_slew_per_s;
    float thr_slew_per_s;

    // Deadbands
    float deadband_axes;    // ej. 0.03
    float deadband_thr;     // ej. 0.02

    // Yaw behavior
    FM_YawMode yaw_mode;
    float yaw_heading_capture_db;   // umbral de override por stick (ej. 0.05)
    float yaw_heading_kp;           // outer loop: psi error -> r_cmd
    float yaw_heading_rmax_dps;     // saturación r_cmd por heading hold

    // Failsafe
    float failsafe_thr;     // [0,1]
} FM_Config;

/* =========================
 * Status para debug
 * ========================= */
typedef struct
{
    FM_Mode mode;
    FM_YawMode yaw_mode;

    bool armed;
    bool failsafe_active;

    FM_CommandsNorm cmd_in;
    FM_Setpoints    sp;
    FM_Actuators    act_out;

    // Útil para telemetría:
    float psi_err_deg;      // heading error actual (wrap)
    float r_cmd_from_heading_dps;
} FM_Status;
/* ============================================================
 * Ejecuciones por cascada
 * ============================================================ */

 void Flight_Control_Attitude_Tier(void);

 void Flight_Control_Cruice_Tier(void);

 void Flight_Control_Mission_Tier(void);


/* =========================
 * API
 * ========================= */
void FM_Init(const FM_Config *cfg, FM_Mode initial_mode);

void    FM_SetMode(FM_Mode mode);
FM_Mode FM_GetMode(void);

/* Llamar al entrar a un modo (para capturar setpoints: psi_cmd, alt_cmd, arsp_cmd, etc.) */
void FM_OnModeEnter(const FM_State *st);

void FM_SetArmed(bool armed);
bool FM_IsArmed(void);

void FM_SetFailsafe(bool enabled);

void FM_ResetIntegrators(void);

/* Nota: rc_cmd->throttle ya viene en [0,1] */
void FM_Update(const FM_CommandsNorm *rc_cmd,
               const FM_State        *st,
               float                  dt_s);

void FM_GetActuators(FM_Actuators *out);
void FM_GetStatus(FM_Status *out);

/* Helpers */
float FM_Clamp(float x, float xmin, float xmax);
float FM_ApplyDeadband(float x, float db);
float FM_WrapAngleDeg(float ang_deg);          // [-180,180)
float FM_AngleErrorDeg(float cmd_deg, float meas_deg); // wrap(cmd-meas)

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_MANAGEMENT_H */
