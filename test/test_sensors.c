/* ===========================================================
 * test_sensors.c
 * Cubre los modelos de pitot y veleta de sensors_sim.c
 * (teoría: docs/teoria/sensores/ruido_sensores.tex):
 *   - pitot sin ruido: Δp = 1/2 rho(h) V^2, TAS = V, IAS = V sqrt(rho/rho0)
 *   - pitot con ruido: el ruido está en Δp con la sigma configurada
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

int main(void)
{
    test_pitot_sin_ruido();
    test_pitot_altitud();
    test_pitot_ruido();
    TEST_RETURN();
}
