/* ===========================================================
 * test_flight_sim_trim.c
 * Cubre FlightSim_SetActuatorsFromFM (flight_sim.c): la planta recibe la
 * perturbación respecto al trim, d_delta = delta - delta_trim [rad].
 *   - superficies en trim -> deltas 0 y el estado no se mueve
 *   - superficies fuera de trim -> la diferencia en rad
 * =========================================================== */
#define _DEFAULT_SOURCE  /* M_PI */
#include <math.h>

#include "flight_sim.h"
#include "test_util.h"

#define D2R (M_PI / 180.0)

static Params base(void)
{
    Params p = {
        .name = "trim",
        .X_u = -0.05f, .X_w = 0.18f, .X_theta = -9.76f,
        .Z_u_hat = -0.78f, .Z_w_hat = -3.10f, .u0_ms = 25.1f, .Z_theta = -1.0f,
        .Z_de = -8.3f,
        .M_u_hat = 0.046f, .M_w_hat = -0.42f, .M_q_hat = -5.3f, .M_de = -50.8f,
        .L_p = -13.0f, .L_da = 50.0f, .N_r = -1.0f, .N_dr = -10.0f,
        .delta_a_trim_deg = 0.3f, .delta_e_trim_deg = -0.49f,
        .delta_r_trim_deg = 1.2f,
    };
    return p;
}

/* T1: actuadores inicializados en trim -> perturbación nula, avion quieto */
static void test_en_trim(void)
{
    const Params p = base();
    FlightSim sim;
    FlightSim_Init(&sim, &p, NULL);

    FM_Actuators act;
    FlightSim_Actuators_init(&act, &p);
    FlightSim_SetActuatorsFromFM(&sim, &act);

    EXPECT_TRUE(sim.delta_elv == 0.0);
    EXPECT_TRUE(sim.delta_ail == 0.0);
    EXPECT_TRUE(sim.delta_rud == 0.0);

    for (int k = 0; k < 100; ++k) FlightSim_Step(&sim, 0.01);
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(sim.X_lon[i] == 0.0);
        EXPECT_TRUE(sim.X_lat[i] == 0.0);
    }
}

/* T2: superficies fuera de trim -> la planta ve delta - delta_trim */
static void test_fuera_de_trim(void)
{
    const Params p = base();
    FlightSim sim;
    FlightSim_Init(&sim, &p, NULL);

    FM_Actuators act;
    FlightSim_GetActuators(&act, 2.3f, 1.51f, -0.8f, 0.0f);
    FlightSim_SetActuatorsFromFM(&sim, &act);

    EXPECT_FLOAT_EQ(sim.delta_elv, (1.51 - (-0.49)) * D2R);
    EXPECT_FLOAT_EQ(sim.delta_ail, (2.3 - 0.3) * D2R);
    EXPECT_FLOAT_EQ(sim.delta_rud, (-0.8 - 1.2) * D2R);
}

int main(void)
{
    test_en_trim();
    test_fuera_de_trim();
    TEST_RETURN();
}
