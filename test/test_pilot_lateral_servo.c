/* ===========================================================
 * test_pilot_lateral_servo.c  (TODO 3.2)
 * Cadena lateral piloto -> RC -> servo -> planta:
 *   - ruteo: roll -> alerón, yaw -> timón (signo de Nelson), trim sumado
 *   - escalón: 1er orden, 63.2 % en tau, estado estacionario k*u
 *   - saturación mecánica y limitador de velocidad de alerón y timón
 *   - signo de punta a punta: stick derecho -> p > 0, pedal derecho -> r > 0
 * Servos con la tabla compilada (Servos_Init_All):
 *   alerón tau 0.05 s, ±20°, 300 °/s; timón tau 0.07 s, ±30°, 250 °/s;
 *   ambos k = 0.18 °/us.
 * =========================================================== */
#include <math.h>
#include <string.h>

#include "flight_sim.h"
#include "pilot_sim.h"
#include "servo_sim.h"
#include "test_util.h"

#define K_DEG_US   0.18f
#define DT_S       0.001

static bool cerca(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

/* Pasa un piloto por rc_source + rc_to_servo_cmds_us y manda los comandos */
static void mandar(pilotcommand_s pilot, const float trim_us[SERVO_SIM_N_SERVOS])
{
    float cmd[SERVO_SIM_N_SERVOS];
    SimRc_s rc = rc_source(pilot);
    rc_to_servo_cmds_us(&rc, trim_us, cmd);
    for (unsigned i = 0u; i < SERVO_SIM_N_SERVOS; ++i) {
        Servos_SetCmdDeltaUs(i, cmd[i]);
    }
}

static void correr_ms(unsigned n)
{
    for (unsigned k = 0u; k < n; ++k) Servos_Update_1ms();
}

/* T1: cada stick va solo a su servo; yaw invertido; el trim se suma */
static void test_ruteo(void)
{
    float cmd[SERVO_SIM_N_SERVOS];
    pilotcommand_s pilot;
    SimRc_s rc;

    memset(&pilot, 0, sizeof(pilot));
    pilot.roll = 0.1f;
    rc = rc_source(pilot);
    rc_to_servo_cmds_us(&rc, NULL, cmd);
    EXPECT_FLOAT_EQ(cmd[SERVO_AILERON], 50.0f);
    EXPECT_FLOAT_EQ(cmd[SERVO_RUDDER], 0.0f);
    EXPECT_FLOAT_EQ(cmd[SERVO_ELEVATOR], 0.0f);
    EXPECT_FLOAT_EQ(cmd[SERVO_THROTTLE], 0.0f);

    memset(&pilot, 0, sizeof(pilot));
    pilot.yaw = 0.1f;
    rc = rc_source(pilot);
    rc_to_servo_cmds_us(&rc, NULL, cmd);
    EXPECT_FLOAT_EQ(cmd[SERVO_RUDDER], -50.0f);   // Nelson: pedal derecho -> delta_r < 0
    EXPECT_FLOAT_EQ(cmd[SERVO_AILERON], 0.0f);
    EXPECT_FLOAT_EQ(cmd[SERVO_ELEVATOR], 0.0f);

    const float trim[SERVO_SIM_N_SERVOS] = {
        [SERVO_ELEVATOR] = -12.0f, [SERVO_AILERON] = 3.0f,
        [SERVO_RUDDER] = 5.0f,     [SERVO_THROTTLE] = 0.0f,
    };
    memset(&pilot, 0, sizeof(pilot));
    pilot.roll = 0.1f;
    pilot.yaw  = 0.1f;
    rc = rc_source(pilot);
    rc_to_servo_cmds_us(&rc, trim, cmd);
    EXPECT_FLOAT_EQ(cmd[SERVO_AILERON], 53.0f);
    EXPECT_FLOAT_EQ(cmd[SERVO_RUDDER], -45.0f);
    EXPECT_FLOAT_EQ(cmd[SERVO_ELEVATOR], -12.0f);
}

/* T2: escalón chico (sin saturar ni limitar velocidad): 1er orden.
 * Alerón: 0.1 de stick -> 50 us -> 9°, pendiente inicial 180 °/s < 300.
 * Timón: 0.1 de pedal -> -50 us -> -9°, pendiente 129 °/s < 250. */
static void test_escalon(void)
{
    const double y_ss = K_DEG_US * 50.0;   // 9°
    pilotcommand_s pilot;

    Servos_Init_All();
    memset(&pilot, 0, sizeof(pilot));
    pilot.roll = 0.1f;
    mandar(pilot, NULL);
    correr_ms(50);                                        // una tau del alerón
    EXPECT_TRUE(cerca(Servos_GetOutDeg(SERVO_AILERON), 0.632 * y_ss, 0.02));
    correr_ms(450);                                       // 10 tau
    EXPECT_TRUE(cerca(Servos_GetOutDeg(SERVO_AILERON), y_ss, 1e-3));
    EXPECT_TRUE(Servos_GetOutDeg(SERVO_RUDDER) == 0.0f);  // el timón no se movió

    Servos_Init_All();
    memset(&pilot, 0, sizeof(pilot));
    pilot.yaw = 0.1f;
    mandar(pilot, NULL);
    correr_ms(70);                                        // una tau del timón
    EXPECT_TRUE(cerca(Servos_GetOutDeg(SERVO_RUDDER), -0.632 * y_ss, 0.02));
    correr_ms(630);
    EXPECT_TRUE(cerca(Servos_GetOutDeg(SERVO_RUDDER), -y_ss, 1e-3));
    EXPECT_TRUE(Servos_GetOutDeg(SERVO_AILERON) == 0.0f);
}

/* Stick a fondo en un canal: la salida nunca pasa el tope, cada paso cambia
 * a lo más rate*dt y termina en el tope. */
static void revisar_saturacion(unsigned canal, float stick, float tope_deg, float rate_dps)
{
    pilotcommand_s pilot;
    memset(&pilot, 0, sizeof(pilot));
    if (canal == SERVO_AILERON) pilot.roll = stick;
    else                        pilot.yaw  = stick;

    Servos_Init_All();
    mandar(pilot, NULL);

    const float paso_max = rate_dps * (float)DT_S + 1e-4f;
    float y_prev = Servos_GetOutDeg(canal);
    bool rate_ok = true, tope_ok = true;
    for (unsigned k = 0u; k < 2000u; ++k) {
        Servos_Update_1ms();
        const float y = Servos_GetOutDeg(canal);
        if (fabsf(y - y_prev) > paso_max) rate_ok = false;
        if (fabsf(y) > tope_deg + 1e-4f)  tope_ok = false;
        y_prev = y;
    }
    EXPECT_TRUE(rate_ok);
    EXPECT_TRUE(tope_ok);
    EXPECT_TRUE(cerca(fabsf(y_prev), tope_deg, 1e-4));
}

/* T3: a fondo (500 us -> 90° pedidos) satura en el tope mecánico, con el
 * limitador de velocidad activo; también con stick > 1 (comando > 500 us) */
static void test_saturacion_y_rate(void)
{
    revisar_saturacion(SERVO_AILERON,  1.0f, 20.0f, 300.0f);
    revisar_saturacion(SERVO_AILERON, -1.0f, 20.0f, 300.0f);
    revisar_saturacion(SERVO_AILERON,  1.5f, 20.0f, 300.0f);
    revisar_saturacion(SERVO_RUDDER,   1.0f, 30.0f, 250.0f);
    revisar_saturacion(SERVO_RUDDER,  -1.0f, 30.0f, 250.0f);

    /* signo en saturación: pedal derecho -> tope negativo */
    pilotcommand_s pilot;
    memset(&pilot, 0, sizeof(pilot));
    pilot.yaw = 1.0f;
    Servos_Init_All();
    mandar(pilot, NULL);
    correr_ms(2000);
    EXPECT_TRUE(cerca(Servos_GetOutDeg(SERVO_RUDDER), -30.0f, 1e-4));
}

/* Derivadas laterales del Ugly Stick (aircraft/ugly_stick.json, ejes cuerpo) */
static Params ugly_lat(void)
{
    Params p = {
        .name = "ugly_lat",
        .u0_ms = 25.1154f, .w0_ms = 2.60375f, .theta0_deg = 5.9188f,
        .I_x = 1.84528f, .I_z = 4.9975f, .I_xz = 0.0f,
        .Y_v = -0.1392f, .Y_p = -0.0133494f, .Y_r = 0.128766f, .Y_dr = 2.6231f,
        .L_v = -0.656701f, .L_p = -13.4567f, .L_r = 1.87639f,
        .L_da = 109.9f, .L_dr = 3.29206f,
        .N_v = 0.599895f, .N_p = -1.03899f, .N_r = -0.46948f,
        .N_da = -2.51477f, .N_dr = -9.68911f,
    };
    return p;
}

/* Lleva la salida de los servos laterales a la planta durante n ms */
static void volar_ms(FlightSim *sim, unsigned n)
{
    FM_Actuators act;
    for (unsigned k = 0u; k < n; ++k) {
        Servos_Update_1ms();
        FlightSim_GetActuators(&act, Servos_GetOutDeg(SERVO_AILERON), 0.0f,
                               Servos_GetOutDeg(SERVO_RUDDER), 0.0f);
        FlightSim_SetActuatorsFromFM(sim, &act);
        FlightSim_Step(sim, DT_S);
    }
}

/* T4: signo de punta a punta (0.2 s, antes de que el dutch roll invierta):
 * stick derecho -> alabeo a la derecha (p > 0);
 * pedal derecho -> nariz a la derecha (r > 0) */
static void test_signo_planta(void)
{
    const Params p = ugly_lat();
    FlightSim sim;
    pilotcommand_s pilot;

    FlightSim_Init(&sim, &p, NULL);
    Servos_Init_All();
    memset(&pilot, 0, sizeof(pilot));
    pilot.roll = 0.05f;
    mandar(pilot, NULL);
    volar_ms(&sim, 200);
    EXPECT_TRUE(sim.X_lat[1] > 0.0);   // dp

    FlightSim_Init(&sim, &p, NULL);
    Servos_Init_All();
    memset(&pilot, 0, sizeof(pilot));
    pilot.yaw = 0.05f;
    mandar(pilot, NULL);
    volar_ms(&sim, 200);
    EXPECT_TRUE(Servos_GetOutDeg(SERVO_RUDDER) < 0.0f);
    EXPECT_TRUE(sim.X_lat[2] > 0.0);   // dr
}

int main(void)
{
    test_ruteo();
    test_escalon();
    test_saturacion_y_rate();
    test_signo_planta();
    TEST_RETURN();
}
