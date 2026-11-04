#ifndef SENSORS_SIM_H
#define SENSORS_SIM_H

#include <stdint.h>
#include <stdbool.h>

#include "flight_sim.h"
#include "dynamics.h"
#include "imu_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Constantes
 * ============================================================ */

#ifndef SENSORS_SIM_PI
#define SENSORS_SIM_PI 3.14159265358979323846
#endif

#define SENSORS_SIM_GRAVITY_MPS2      9.80665
#define SENSORS_SIM_EARTH_RADIUS_M    6378137.0

/* Periodo de la IMU: el modelo de ruido se validó a 200 Hz */
#define SENSORS_SIM_IMU_DT_S          0.005

/* ============================================================
 * Tipos auxiliares
 * ============================================================ */

/* Una secuencia pseudoaleatoria por sensor: agregar o quitar ruido en uno
 * no desplaza la secuencia de los demás. SENSORS_RNG_IMU solo da la semilla
 * del generador propio de la IMU. */
typedef enum
{
    SENSORS_RNG_PITOT = 0,
    SENSORS_RNG_VANE,
    SENSORS_RNG_IMU,
    SENSORS_RNG_GPS,
    SENSORS_RNG_LASER,
    SENSORS_RNG_AHRS,    // al final: la semilla de cada sensor depende de su índice
    SENSORS_RNG_N
} SensorsRng_e;

typedef struct
{
    double x;
    double y;
    double z;
} SensorVec3;

/* ============================================================
 * Configuración de ruido y bias
 * ============================================================ */

typedef struct
{
    bool   enable_noise;

    /* Pitot: ruido y bias del sensor de presión diferencial */
    double pitot_dp_noise_std_pa;
    double pitot_dp_bias_pa;

    /* Veleta: misma sigma para alpha y beta, bias por ángulo */
    double vane_noise_std_deg;
    double vane_alpha_bias_deg;
    double vane_beta_bias_deg;

    /* Gyros: sesgo determinista; el ruido viene del modelo de IMU */
    double gyro_bias_x_radps;
    double gyro_bias_y_radps;
    double gyro_bias_z_radps;

    /* Acelerómetros: sesgo determinista */
    double accel_bias_x_mps2;
    double accel_bias_y_mps2;
    double accel_bias_z_mps2;

    /* Modelo de ruido de la IMU (imu_sensor.h). Teoría:
     * docs/teoria/sensores/ruido_sensores.tex */
    int    imu_profile;        // IMU_PROFILE_FLOOR o IMU_PROFILE_IDLE
    bool   imu_flight;         // true: suma la vibración de vuelo
    double imu_vib_env;        // >= 0: envolvente fija; < 0: aleatoria interna
    double imu_noise_scale;    // escala todo el ruido
    double imu_white_scale;    // escala el ruido blanco
    double imu_vib_scale;      // escala la vibración
    double imu_bias_scale;     // escala el sesgo aleatorio b0

    /* GPS */
    double gps_pos_noise_std_m;
    double gps_vel_noise_std_ms;
    double gps_alt_noise_std_m;

    /* Altímetro láser */
    double laser_noise_std_m;
    double laser_bias_m;

    /* AHRS ideal: misma sigma para phi y theta, otra para psi; bias por ángulo */
    double ahrs_att_noise_std_deg;
    double ahrs_psi_noise_std_deg;
    double ahrs_phi_bias_deg;
    double ahrs_theta_bias_deg;
    double ahrs_psi_bias_deg;

} SensorsSimNoiseConfig;

/* ============================================================
 * Configuración inicial del simulador de sensores
 * ============================================================ */

typedef struct
{
    /* Posición inicial GPS */
    double lat0_deg;
    double lon0_deg;
    double alt0_m;

    /* Terreno para altímetro láser */
    double ground_alt_m;

    /* Activa/desactiva sensores */
    bool pitot_enabled;
    bool imu_enabled;
    bool gps_enabled;
    bool laser_enabled;
    bool ahrs_enabled;

    /* Semilla para ruido pseudoaleatorio */
    uint32_t random_seed;

    /* Configuración de ruido */
    SensorsSimNoiseConfig noise;

} SensorsSimConfig;

/* ============================================================
 * Salidas simuladas de sensores
 * ============================================================ */

typedef struct
{
    bool valid;

    double diff_pressure_pa;   // Δp medida (con ruido y bias) [Pa]
    double ias_ms;             // IAS = sqrt(2 Δp / rho0) [m/s]
    double tas_ms;             // TAS = IAS sqrt(rho0 / rho(h)) [m/s]

} SensorPitotData;

typedef struct
{
    bool valid;

    double AngleOfAttack_deg;
    double SideSlipAngle_deg;

} SensorVane;



typedef struct
{
    bool valid;

    SensorVec3 gyro_radps;
    SensorVec3 accel_mps2;

    double theta_rad;

} SensorImuData;

typedef struct
{
    bool valid;

    double lat_deg;
    double lon_deg;
    double alt_m;

    double vn_ms;
    double ve_ms;
    double vd_ms;

    double ground_speed_ms;

} SensorGpsData;

typedef struct
{
    bool valid;

    double range_m;

} SensorLaserAltData;

/* AHRS ideal: actitud verdadera + ruido + bias (sin estimador) */
typedef struct
{
    bool valid;

    double phi_deg;
    double theta_deg;
    double psi_deg;            // [-180, 180)

} SensorAhrsData;

typedef struct
{
    double t_s;

    SensorPitotData    pitot;
    SensorVane         vane;
    SensorImuData      imu;
    SensorGpsData      gps;
    SensorLaserAltData laser;
    SensorAhrsData     ahrs;

} SensorsSimData;

/* ============================================================
 * Estado interno del simulador de sensores
 * ============================================================ */

typedef struct
{
    bool initialized;

    SensorsSimConfig cfg;
    SensorsSimData   data;

    /* Integración de navegación */
    double lat_rad;
    double lon_rad;
    double alt_m;

    /* Para estimar aceleraciones por diferencia finita */
    double prev_u_ms;
    double prev_w_ms;
    double prev_v_ms;
    bool   prev_valid;

    /* IMU: modelo de ruido y tiempo acumulado desde la última muestra */
    ImuSensor imu;
    double    imu_accum_s;

    /* Generador pseudoaleatorio: un estado por sensor (SensorsRng_e) */
    uint32_t rng_state[SENSORS_RNG_N];

} SensorsSim;

/* ============================================================
 * API pública
 * ============================================================ */

void SensorsSim_Init(SensorsSim *ss,
                     const SensorsSimConfig *cfg,
                     const FlightSim *sim);

void SensorsSim_Reset(SensorsSim *ss,
                      const SensorsSimConfig *cfg,
                      const FlightSim *sim);

void SensorsSim_Update(SensorsSim *ss,
                       const FlightSim *sim,
                       double dt_s);

void SensorsSim_GetData(const SensorsSim *ss,
                        SensorsSimData *out);

const SensorsSimData* SensorsSim_GetDataPtr(const SensorsSim *ss);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_SIM_H */
