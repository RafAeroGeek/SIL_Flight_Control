#include "sensors_sim.h"

#include <string.h>
#include <math.h>

/* ============================================================
 * Helpers internos
 * ============================================================ */

static double deg2rad(double x_deg)
{
    return x_deg * SENSORS_SIM_PI / 180.0;
}

static double rad2deg(double x_rad)
{
    return x_rad * 180.0 / SENSORS_SIM_PI;
}

static double clamp_min(double x, double xmin)
{
    return (x < xmin) ? xmin : x;
}

/* Generador pseudoaleatorio simple tipo LCG.
 * Ventaja: sirve igual en PC y STM32.
 */
static uint32_t sensors_lcg_next(uint32_t *state)
{
    if (*state == 0u) {
        *state = 1u;
    }

    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

static double sensors_rand_uniform_0_1(uint32_t *state)
{
    uint32_t r = sensors_lcg_next(state);

    /* Convertir a [0, 1) usando 24 bits */
    return (double)(r & 0x00FFFFFFu) / (double)0x01000000u;
}

/* Ruido gaussiano aproximado usando suma de uniformes.
 * No es perfecto, pero es barato y suficientemente bueno para SIL inicial.
 */
static double sensors_rand_normal(uint32_t *state, double std)
{
    if (std <= 0.0) {
        return 0.0;
    }

    double sum = 0.0;

    for (int i = 0; i < 12; ++i) {
        sum += sensors_rand_uniform_0_1(state);
    }

    /* Aproximación N(0,1): suma de 12 uniformes - 6 */
    return (sum - 6.0) * std;
}

static double sensors_noise(SensorsSim *ss, double std)
{
    if (!ss->cfg.noise.enable_noise) {
        return 0.0;
    }

    return sensors_rand_normal(&ss->rng_state, std);
}

/* ============================================================
 * Reconstrucción del estado verdadero desde FlightSim
 * ============================================================ */

static double sensors_get_u_body_ms(const FlightSim *sim)
{
    if (!sim) return 0.0;

    /*
     * Tu estado X[0] = du.
     * La velocidad total longitudinal de cuerpo es:
     *
     * u = u0_ms + du
     *
     * En tu Params tienes p->u0_ms y p->V0_ms.
     * Uso u0_ms si está disponible; si no, V0_ms.
     */
    double u_trim = (double)sim->params.u0_ms;

    if (fabs(u_trim) < 1.0e-6) {
        u_trim = (double)sim->params.V0_ms;
    }

    return u_trim + sim->X[0];
}

static double sensors_get_w_body_ms(const FlightSim *sim)
{
    if (!sim) return 0.0;

    /*
     * Tu estado X[1] = dw.
     * En modelo longitudinal inicial usamos:
     *
     * w = dw
     */
    return sim->X[1];
}

static double sensors_get_q_radps(const FlightSim *sim)
{
    if (!sim) return 0.0;

    /*
     * Tu estado X[2] = dq.
     */
    return sim->X[2];
}

static double sensors_get_theta_rad(const FlightSim *sim)
{
    if (!sim) return 0.0;

    /*
     * Tu estado X[3] = dtheta.
     *
     * Primera aproximación:
     * theta = gamma0 + dtheta
     *
     * Más adelante puedes usar:
     * theta = alpha_trim + gamma0 + dtheta
     */
    double gamma0_rad = deg2rad((double)sim->params.gamma0_deg);

    return gamma0_rad + sim->X[3];
}

/* ============================================================
 * Modelos de sensores
 * ============================================================ */

static void SensorsSim_UpdatePitot(SensorsSim *ss,
                                   const FlightSim *sim)
{
    /*
     * Tu estado X[0] = du.
     * La velocidad total Viento:
     *
     * V = V0 + dV
     *                     dw
     * dV = du + w0_ms * ------
     *                    u0_ms
     *
     * Aircraft systems identification Morelli Pag.81
     */
    if (!ss->cfg.pitot_enabled) {
        ss->data.pitot.valid = false;
        return;
    }

    const double V0 = (double)sim->params.V0_ms;

    const double u0 = (double)sim->params.u0_ms;

    const double w0 = (double)sim->params.w0_ms;

    double du = sim->X[0];

    double dw = sim->X[1];

    double dV = du + w0 * dw / u0;

    const double rho0 = 1.225;

    double v_air = V0 + dV ;

    v_air += ss->cfg.noise.pitot_bias_ms;
    v_air += sensors_noise(ss, ss->cfg.noise.pitot_noise_std_ms);

    v_air = clamp_min(v_air, 0.0);

    ss->data.pitot.valid = true;
    ss->data.pitot.airspeed_ms = v_air;
    ss->data.pitot.dynamic_pressure_pa = 0.5 * rho0 * v_air * v_air;
}

static void SensorSim_UpdateVanes(SensorsSim *ss,
                                   const FlightSim *sim)
{
       /*
     * Tu estado X[0] = du.
     * La velocidad total Viento:
     *
     * u = u0 + du
     * w = w0 + dw       _    _
     *                -1|  w   |
     *   dalpha = tan   |------|
     *                  |_ u  _|
     *
     *
     * Aircraft systems identification Morelli Pag.50
     */
    const double u0 = (double)sim->params.u0_ms;

    const double w0 = (double)sim->params.w0_ms;

    double u = u0 + sim->X[0];  // u0 + du
    double w = w0 + sim->X[1];  // w0 + dw

    // Validación de singularidad
    if (fabs(u) < 0.1) {
        ss->data.vane.valid = false;
        return;
    }

    double alpha_rad = atan2(w, u);
    ss->data.vane.AngleOfAttack_deg = alpha_rad * 57.29577951308;  // rad to deg
    ss->data.vane.valid = true;
}

static void SensorsSim_UpdateImu(SensorsSim *ss,
                                 const FlightSim *sim,
                                 double dt_s)
{
    if (!ss->cfg.imu_enabled) {
        ss->data.imu.valid = false;
        return;
    }

    double u = sensors_get_u_body_ms(sim);
    double w = sensors_get_w_body_ms(sim);
    double q = sensors_get_q_radps(sim);
    double theta = sensors_get_theta_rad(sim);
    double phi = 0.0 ; // TODO cambiar en la implemantacion de lateral - direccional.

    double du_dot = 0.0;
    double dw_dot = 0.0;

    if (ss->prev_valid && dt_s > 0.0) {
        du_dot = (u - ss->prev_u_ms) / dt_s;
        dw_dot = (w - ss->prev_w_ms) / dt_s;
    }

    /*
     * Primera aproximación de acelerómetro.
     *
     * Nota importante:
     * Un acelerómetro real mide fuerza específica, no aceleración absoluta.
     * Esta versión sirve para arrancar arquitectura.
     *
     * Más adelante podemos reemplazar esto por:
     *
     * a_x = u_dot + q*w + g*sin(theta)
     * a_y = v_dot - p*w + r*u - g*cos(theta)*sin(phi)
     * a_z = w_dot - q*u - g*cos(theta)
     *
     * según convención de ejes.
     */
    double ax = du_dot + q * w + 9.81 * sin(theta);
    double ay = 0.0;
    double az = dw_dot - q * u - 9.81 * cos(theta) * sin(phi);

    /* Gyro */
    double gx = 0.0;
    double gy = q;
    double gz = 0.0;

    gx += ss->cfg.noise.gyro_bias_x_radps;
    gy += ss->cfg.noise.gyro_bias_y_radps;
    gz += ss->cfg.noise.gyro_bias_z_radps;

    gx += sensors_noise(ss, ss->cfg.noise.gyro_noise_std_radps);
    gy += sensors_noise(ss, ss->cfg.noise.gyro_noise_std_radps);
    gz += sensors_noise(ss, ss->cfg.noise.gyro_noise_std_radps);

    ax += ss->cfg.noise.accel_bias_x_mps2;
    ay += ss->cfg.noise.accel_bias_y_mps2;
    az += ss->cfg.noise.accel_bias_z_mps2;

    ax += sensors_noise(ss, ss->cfg.noise.accel_noise_std_mps2);
    ay += sensors_noise(ss, ss->cfg.noise.accel_noise_std_mps2);
    az += sensors_noise(ss, ss->cfg.noise.accel_noise_std_mps2);

    ss->data.imu.valid = true;

    ss->data.imu.gyro_radps.x = gx;
    ss->data.imu.gyro_radps.y = gy;
    ss->data.imu.gyro_radps.z = gz;

    ss->data.imu.accel_mps2.x = ax;
    ss->data.imu.accel_mps2.y = ay;
    ss->data.imu.accel_mps2.z = az;

    ss->data.imu.theta_rad = theta;
}

static void SensorsSim_UpdateGps(SensorsSim *ss,
                                 const FlightSim *sim,
                                 double dt_s)
{
    if (!ss->cfg.gps_enabled) {
        ss->data.gps.valid = false;
        return;
    }

    double u = sensors_get_u_body_ms(sim);
    double w = sensors_get_w_body_ms(sim);
    double theta = sensors_get_theta_rad(sim);

    /*
     * Conversión cuerpo -> NED simplificada longitudinal.
     *
     * Asumimos:
     * - no hay velocidad lateral
     * - no hay heading todavía
     * - todo se mueve sobre el eje Norte
     *
     * V_N = u cos(theta) + w sin(theta)
     * V_D = -u sin(theta) + w cos(theta)
     */
    double vn = u * cos(theta) + w * sin(theta);
    double ve = 0.0;
    double vd = -u * sin(theta) + w * cos(theta);

    /* Integración de posición */
    if (dt_s > 0.0) {
        double d_north = vn * dt_s;
        double d_east  = ve * dt_s;
        double d_down  = vd * dt_s;

        double cos_lat = cos(ss->lat_rad);

        if (fabs(cos_lat) < 1.0e-6) {
            cos_lat = 1.0e-6;
        }

        ss->lat_rad += d_north / SENSORS_SIM_EARTH_RADIUS_M;
        ss->lon_rad += d_east / (SENSORS_SIM_EARTH_RADIUS_M * cos_lat);
        ss->alt_m   -= d_down;
    }

    double lat_m_noise = sensors_noise(ss, ss->cfg.noise.gps_pos_noise_std_m);
    double lon_m_noise = sensors_noise(ss, ss->cfg.noise.gps_pos_noise_std_m);
    double alt_noise   = sensors_noise(ss, ss->cfg.noise.gps_alt_noise_std_m);

    double lat_noise_rad = lat_m_noise / SENSORS_SIM_EARTH_RADIUS_M;

    double cos_lat2 = cos(ss->lat_rad);
    if (fabs(cos_lat2) < 1.0e-6) {
        cos_lat2 = 1.0e-6;
    }

    double lon_noise_rad = lon_m_noise / (SENSORS_SIM_EARTH_RADIUS_M * cos_lat2);

    double vn_meas = vn + sensors_noise(ss, ss->cfg.noise.gps_vel_noise_std_ms);
    double ve_meas = ve + sensors_noise(ss, ss->cfg.noise.gps_vel_noise_std_ms);
    double vd_meas = vd + sensors_noise(ss, ss->cfg.noise.gps_vel_noise_std_ms);

    ss->data.gps.valid = true;

    ss->data.gps.lat_deg = rad2deg(ss->lat_rad + lat_noise_rad);
    ss->data.gps.lon_deg = rad2deg(ss->lon_rad + lon_noise_rad);
    ss->data.gps.alt_m   = ss->alt_m + alt_noise;

    ss->data.gps.vn_ms = vn_meas;
    ss->data.gps.ve_ms = ve_meas;
    ss->data.gps.vd_ms = vd_meas;

    ss->data.gps.ground_speed_ms = sqrt(vn_meas*vn_meas + ve_meas*ve_meas);
}

static void SensorsSim_UpdateLaser(SensorsSim *ss)
{
    if (!ss->cfg.laser_enabled) {
        ss->data.laser.valid = false;
        return;
    }

    double range = ss->alt_m - ss->cfg.ground_alt_m;

    range += ss->cfg.noise.laser_bias_m;
    range += sensors_noise(ss, ss->cfg.noise.laser_noise_std_m);

    range = clamp_min(range, 0.0);

    ss->data.laser.valid = true;
    ss->data.laser.range_m = range;
}

/* ============================================================
 * API pública
 * ============================================================ */

void SensorsSim_Init(SensorsSim *ss,
                     const SensorsSimConfig *cfg,
                     const FlightSim *sim)
{
    if (!ss || !cfg || !sim) {
        return;
    }

    memset(ss, 0, sizeof(*ss));

    ss->cfg = *cfg;

    ss->lat_rad = deg2rad(cfg->lat0_deg);
    ss->lon_rad = deg2rad(cfg->lon0_deg);

    /*
     * Si cfg->alt0_m es válido, usar ese.
     * Si no, usar la altitud de FlightSim.
     */
    ss->alt_m = cfg->alt0_m;

    if (fabs(ss->alt_m) < 1.0e-9) {
        ss->alt_m = sim->H_m;
    }

    ss->rng_state = cfg->random_seed;
    if (ss->rng_state == 0u) {
        ss->rng_state = 1u;
    }

    ss->prev_u_ms = sensors_get_u_body_ms(sim);
    ss->prev_w_ms = sensors_get_w_body_ms(sim);
    ss->prev_valid = true;

    ss->data.t_s = sim->t_s;

    ss->initialized = true;
}

void SensorsSim_Reset(SensorsSim *ss,
                      const SensorsSimConfig *cfg,
                      const FlightSim *sim)
{
    SensorsSim_Init(ss, cfg, sim);
}

void SensorsSim_Update(SensorsSim *ss,
                       const FlightSim *sim,
                       double dt_s)
{
    if (!ss || !sim || !ss->initialized) {
        return;
    }

    double u = sensors_get_u_body_ms(sim);
    double w = sensors_get_w_body_ms(sim);

    ss->data.t_s = sim->t_s;

    // TODO calcular SideSlipAngle_deg en SensorSim_UpdateVanes (queda en 0.0 por ahora)
    SensorsSim_UpdatePitot(ss, sim);
    SensorSim_UpdateVanes(ss, sim);
    SensorsSim_UpdateImu(ss, sim, dt_s);
    SensorsSim_UpdateGps(ss, sim, dt_s);
    SensorsSim_UpdateLaser(ss);

    ss->prev_u_ms = u;
    ss->prev_w_ms = w;
    ss->prev_valid = true;
}

void SensorsSim_GetData(const SensorsSim *ss,
                        SensorsSimData *out)
{
    if (!ss || !out) {
        return;
    }

    *out = ss->data;
}

const SensorsSimData* SensorsSim_GetDataPtr(const SensorsSim *ss)
{
    if (!ss) {
        return 0;
    }

    return &ss->data;
}
