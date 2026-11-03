/* ===========================================================
 * test_sensors.c
 * Cubre los modelos de pitot y veleta de sensors_sim.c
 * (teoría: docs/teoria/sensores/ruido_sensores.tex):
 *   - pitot sin ruido: Δp = 1/2 rho(h) V^2, TAS = V, IAS = V sqrt(rho/rho0)
 *   - pitot con ruido: el ruido está en Δp con la sigma configurada
 *   - veleta: alpha = atan2(w, u) en el trim; ruido de 0.5° en alpha y beta
 *   - IMU: verdad sin ruido, sigma del piso y de vuelo, retención a 200 Hz,
 *     misma semilla -> misma salida, y aislamiento de los demás sensores
 * =========================================================== */
#include <math.h>
#include <string.h>

#include "atmosphere.h"
#include "sensors_sim.h"
#include "test_util.h"

#define N_MUESTRAS 20000

static Params base(void)
{
    Params p = {
        .name = "sens",
        .Alt_m = 1000.0f, .V0_ms = 25.25f,
        .u0_ms = 25.1154f, .w0_ms = 2.60375f, .theta0_deg = 5.9188f,
    };
    return p;
}

static void init(FlightSim *sim, SensorsSim *ss, const Params *p, bool ruido)
{
    FlightSim_Init(sim, p, NULL);

    SensorsSimConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.alt0_m = p->Alt_m;
    cfg.pitot_enabled = true;
    cfg.random_seed = 1234u;
    cfg.noise.enable_noise = ruido;
    cfg.noise.pitot_dp_noise_std_pa = 5.6;
    cfg.noise.vane_noise_std_deg = 0.5;

    SensorsSim_Init(ss, &cfg, sim);
}

static bool cerca(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

/* T1: sin ruido, en el trim, a 1000 m */
static void test_pitot_sin_ruido(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim ss;
    init(&sim, &ss, &p, false);
    SensorsSim_Update(&ss, &sim, 0.0);

    const double V   = sqrt(p.u0_ms * (double)p.u0_ms + p.w0_ms * (double)p.w0_ms);
    const double rho = Atmosphere_IsaDensity(1000.0);
    const SensorPitotData *d = &SensorsSim_GetDataPtr(&ss)->pitot;

    EXPECT_TRUE(d->valid);
    EXPECT_TRUE(cerca(d->diff_pressure_pa, 0.5 * rho * V * V, 1e-9));
    EXPECT_TRUE(cerca(d->tas_ms, V, 1e-9));
    EXPECT_TRUE(cerca(d->ias_ms, V * sqrt(rho / ISA_RHO0_KG_M3), 1e-9));
    EXPECT_TRUE(d->ias_ms < d->tas_ms);
}

/* T2: más altitud -> menos Δp e IAS a la misma TAS */
static void test_pitot_altitud(void)
{
    Params p = base();
    FlightSim sim;
    SensorsSim ss;

    init(&sim, &ss, &p, false);
    SensorsSim_Update(&ss, &sim, 0.0);
    const SensorPitotData a1000 = SensorsSim_GetDataPtr(&ss)->pitot;

    p.Alt_m = 5000.0f;
    init(&sim, &ss, &p, false);
    SensorsSim_Update(&ss, &sim, 0.0);
    const SensorPitotData a5000 = SensorsSim_GetDataPtr(&ss)->pitot;

    EXPECT_TRUE(a5000.diff_pressure_pa < a1000.diff_pressure_pa);
    EXPECT_TRUE(a5000.ias_ms < a1000.ias_ms);
    EXPECT_TRUE(cerca(a5000.tas_ms, a1000.tas_ms, 1e-9));
}

/* T3: con ruido, la desviación estándar de Δp es la configurada (±5 %) y
 * la de TAS es sigma_dp / (rho V) */
static void test_pitot_ruido(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim ss;
    init(&sim, &ss, &p, true);

    double s1 = 0.0, s2 = 0.0, t1 = 0.0, t2 = 0.0;
    for (int k = 0; k < N_MUESTRAS; ++k) {
        SensorsSim_Update(&ss, &sim, 0.0);
        const SensorPitotData *d = &SensorsSim_GetDataPtr(&ss)->pitot;
        s1 += d->diff_pressure_pa;  s2 += d->diff_pressure_pa * d->diff_pressure_pa;
        t1 += d->tas_ms;            t2 += d->tas_ms * d->tas_ms;
    }
    const double sd_dp  = sqrt(s2 / N_MUESTRAS - (s1 / N_MUESTRAS) * (s1 / N_MUESTRAS));
    const double sd_tas = sqrt(t2 / N_MUESTRAS - (t1 / N_MUESTRAS) * (t1 / N_MUESTRAS));
    const double rho = Atmosphere_IsaDensity(1000.0);

    EXPECT_TRUE(cerca(sd_dp, 5.6, 0.05 * 5.6));
    EXPECT_TRUE(cerca(sd_tas, 5.6 / (rho * p.V0_ms), 0.05 * 0.2));
}

/* T4: veleta sin ruido -> alpha de trim, beta 0; con ruido -> sigma 0.5° y
 * alpha y beta sin correlación (dos muestras distintas de la secuencia) */
static void test_veleta(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim ss;

    init(&sim, &ss, &p, false);
    SensorsSim_Update(&ss, &sim, 0.0);
    const SensorVane *v0 = &SensorsSim_GetDataPtr(&ss)->vane;
    EXPECT_TRUE(v0->valid);
    EXPECT_TRUE(cerca(v0->AngleOfAttack_deg,
                      atan2(p.w0_ms, p.u0_ms) * 180.0 / SENSORS_SIM_PI, 1e-9));
    EXPECT_TRUE(v0->SideSlipAngle_deg == 0.0);

    init(&sim, &ss, &p, true);
    double a1 = 0, a2 = 0, b1 = 0, b2 = 0, ab = 0;
    for (int k = 0; k < N_MUESTRAS; ++k) {
        SensorsSim_Update(&ss, &sim, 0.0);
        const SensorVane *v = &SensorsSim_GetDataPtr(&ss)->vane;
        const double a = v->AngleOfAttack_deg, b = v->SideSlipAngle_deg;
        a1 += a; a2 += a * a; b1 += b; b2 += b * b; ab += a * b;
    }
    const double n = N_MUESTRAS;
    const double sa = sqrt(a2 / n - (a1 / n) * (a1 / n));
    const double sb = sqrt(b2 / n - (b1 / n) * (b1 / n));
    const double corr = (ab / n - (a1 / n) * (b1 / n)) / (sa * sb);

    EXPECT_TRUE(cerca(sa, 0.5, 0.025));
    EXPECT_TRUE(cerca(sb, 0.5, 0.025));
    EXPECT_TRUE(fabs(corr) < 0.05);
}

/* Configuración de la IMU sin sesgo aleatorio: la media es la verdad */
static void init_imu(FlightSim *sim, SensorsSim *ss, const Params *p,
                     bool ruido, bool vuelo, uint32_t semilla)
{
    FlightSim_Init(sim, p, NULL);

    SensorsSimConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.alt0_m = p->Alt_m;
    cfg.imu_enabled = true;
    cfg.random_seed = semilla;
    cfg.noise.enable_noise = ruido;
    cfg.noise.imu_profile = IMU_PROFILE_FLOOR;
    cfg.noise.imu_flight = vuelo;
    cfg.noise.imu_vib_env = 1.0;
    cfg.noise.imu_noise_scale = 1.0;
    cfg.noise.imu_white_scale = 1.0;
    cfg.noise.imu_vib_scale = 1.0;
    cfg.noise.imu_bias_scale = 0.0;

    SensorsSim_Init(ss, &cfg, sim);
}

static void imu_a_vector(const SensorImuData *d, double y[IMU_N])
{
    y[0] = d->gyro_radps.x;  y[1] = d->gyro_radps.y;  y[2] = d->gyro_radps.z;
    y[3] = d->accel_mps2.x;  y[4] = d->accel_mps2.y;  y[5] = d->accel_mps2.z;
}

/* Desviación estándar por canal de n muestras de la IMU a 200 Hz */
static void imu_sigmas(SensorsSim *ss, const FlightSim *sim, int n, double sd[IMU_N])
{
    double s1[IMU_N] = {0}, s2[IMU_N] = {0}, y[IMU_N];

    for (int k = 0; k < n; ++k) {
        SensorsSim_Update(ss, sim, SENSORS_SIM_IMU_DT_S);
        imu_a_vector(&SensorsSim_GetDataPtr(ss)->imu, y);
        for (int j = 0; j < IMU_N; ++j) {
            s1[j] += y[j];
            s2[j] += y[j] * y[j];
        }
    }
    for (int j = 0; j < IMU_N; ++j) {
        sd[j] = sqrt(s2[j] / n - (s1[j] / n) * (s1[j] / n));
    }
}

/* T5: sin ruido, la IMU da la verdad en el trim más el sesgo determinista */
static void test_imu_sin_ruido(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim ss;
    init_imu(&sim, &ss, &p, false, true, 1234u);
    ss.cfg.noise.gyro_bias_x_radps = 0.01;
    SensorsSim_Update(&ss, &sim, 0.001);

    const SensorImuData *d = &SensorsSim_GetDataPtr(&ss)->imu;
    const double th = p.theta0_deg * SENSORS_SIM_PI / 180.0;

    EXPECT_TRUE(d->valid);
    EXPECT_TRUE(d->gyro_radps.x == 0.01);
    EXPECT_TRUE(d->gyro_radps.y == 0.0);
    EXPECT_TRUE(d->gyro_radps.z == 0.0);
    EXPECT_TRUE(cerca(d->accel_mps2.x, 9.81 * sin(th), 1e-9));
    EXPECT_TRUE(d->accel_mps2.y == 0.0);
    EXPECT_TRUE(cerca(d->accel_mps2.z, -9.81 * cos(th), 1e-9));
}

/* T6: en reposo, sigma por canal ±5 % del piso */
static void test_imu_piso(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim ss;
    double sd[IMU_N];
    init_imu(&sim, &ss, &p, true, false, 1234u);
    imu_sigmas(&ss, &sim, 40000, sd);

    for (int j = 0; j < IMU_N; ++j) {
        const ImuStaticParam *e = &IMU_STATIC[IMU_PROFILE_FLOOR][j];
        const double esperado = sqrt(e->sigma_w * e->sigma_w + e->sigma_c * e->sigma_c);
        EXPECT_TRUE(cerca(sd[j], esperado, 0.05 * esperado));
    }
}

/* T7: en vuelo con m = 1, sigma por canal ±10 % de piso + vibración */
static void test_imu_vuelo(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim ss;
    double sd[IMU_N];
    init_imu(&sim, &ss, &p, true, true, 1234u);
    imu_sigmas(&ss, &sim, 120000, sd);

    for (int j = 0; j < IMU_N; ++j) {
        const ImuStaticParam *e = &IMU_STATIC[IMU_PROFILE_FLOOR][j];
        const ImuVibParam *v = &IMU_VIB[j];
        const double esperado = sqrt(e->sigma_w * e->sigma_w + e->sigma_c * e->sigma_c
                                     + v->sigma_flat * v->sigma_flat
                                     + v->res[0].sigma * v->res[0].sigma
                                     + v->res[1].sigma * v->res[1].sigma);
        EXPECT_TRUE(cerca(sd[j], esperado, 0.10 * esperado));
    }
}

/* T8: a 1 ms por paso, la salida se retiene 5 ms y cambia en el 5.º paso */
static void test_imu_retencion(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim ss;
    init_imu(&sim, &ss, &p, true, true, 1234u);

    for (int muestra = 0; muestra < 200; ++muestra) {
        SensorsSim_Update(&ss, &sim, 0.001f);
        const double y0 = SensorsSim_GetDataPtr(&ss)->imu.gyro_radps.x;

        for (int k = 1; k < 5; ++k) {
            SensorsSim_Update(&ss, &sim, 0.001f);
            EXPECT_TRUE(SensorsSim_GetDataPtr(&ss)->imu.gyro_radps.x == y0);
        }

        /* El siguiente paso, ya en la próxima vuelta, trae muestra nueva */
        SensorsSim sig = ss;
        SensorsSim_Update(&sig, &sim, 0.001f);
        EXPECT_TRUE(SensorsSim_GetDataPtr(&sig)->imu.gyro_radps.x != y0);
    }
}

/* T9: misma semilla -> misma salida; otra semilla -> otra salida */
static void test_imu_semilla(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim a, b, c;
    double ya[IMU_N], yb[IMU_N], yc[IMU_N];
    bool todas_iguales = true;

    init_imu(&sim, &a, &p, true, true, 1234u);
    init_imu(&sim, &b, &p, true, true, 1234u);
    init_imu(&sim, &c, &p, true, true, 4321u);

    for (int k = 0; k < 1000; ++k) {
        SensorsSim_Update(&a, &sim, SENSORS_SIM_IMU_DT_S);
        SensorsSim_Update(&b, &sim, SENSORS_SIM_IMU_DT_S);
        SensorsSim_Update(&c, &sim, SENSORS_SIM_IMU_DT_S);
        imu_a_vector(&SensorsSim_GetDataPtr(&a)->imu, ya);
        imu_a_vector(&SensorsSim_GetDataPtr(&b)->imu, yb);
        imu_a_vector(&SensorsSim_GetDataPtr(&c)->imu, yc);
        for (int j = 0; j < IMU_N; ++j) {
            EXPECT_TRUE(ya[j] == yb[j]);
            todas_iguales = todas_iguales && (ya[j] == yc[j]);
        }
    }
    EXPECT_TRUE(!todas_iguales);
}

/* T10: encender la IMU no mueve ni un bit de Δp */
static void test_imu_aislada(void)
{
    const Params p = base();
    FlightSim sim;
    SensorsSim con, sin_imu;

    init(&sim, &con, &p, true);
    init(&sim, &sin_imu, &p, true);
    con.cfg.imu_enabled = true;
    con.cfg.noise.imu_flight = true;
    con.cfg.noise.imu_vib_env = 1.0;
    con.imu.noise_scale = 1.0;

    for (int k = 0; k < 1000; ++k) {
        SensorsSim_Update(&con, &sim, 0.001);
        SensorsSim_Update(&sin_imu, &sim, 0.001);
        EXPECT_TRUE(SensorsSim_GetDataPtr(&con)->imu.valid);
        EXPECT_TRUE(SensorsSim_GetDataPtr(&con)->pitot.diff_pressure_pa
                    == SensorsSim_GetDataPtr(&sin_imu)->pitot.diff_pressure_pa);
    }
}

int main(void)
{
    test_pitot_sin_ruido();
    test_pitot_altitud();
    test_pitot_ruido();
    test_veleta();
    test_imu_sin_ruido();
    test_imu_piso();
    test_imu_vuelo();
    test_imu_retencion();
    test_imu_semilla();
    test_imu_aislada();
    TEST_RETURN();
}
