/* ===========================================================
 * test_rk4_n.c
 * Cubre el integrador RK4 generico de dynamics.c:
 *   - rk4_step_n + linear_dynamics_n con n=4, m=1 == rk4_step (bit a bit)
 *   - alabeo puro contra la solucion analitica de 1er orden
 *   - guardas de tamano (n == 0, n > RK4_N_MAX)
 * =========================================================== */
#include "dynamics.h"
#include "test_util.h"

#define EXPECT_NEAR(a, b, tol) do {                                           \
    double _a = (double)(a);                                                  \
    double _b = (double)(b);                                                  \
    if (fabs(_a - _b) > (tol)) {                                              \
        printf("FAIL %s:%d: %s (%.12g) != %s (%.12g)\n",                      \
               __FILE__, __LINE__, #a, _a, #b, _b);                           \
        g_test_fail++;                                                        \
    }                                                                         \
} while (0)

/* Derivadas longitudinales de aircraft/aircraft_a.json */
static const Params kAircraftA = {
    .name = "aircraft_a",
    .X_u = -0.0498f, .X_w = -0.0169f, .X_theta = -9.724f,
    .Z_u_hat = -0.5484f, .Z_w_hat = -2.557f, .u0_ms = 35.6997f, .w0_ms = 4.71f,
    .Z_theta = 1.2963f, .Z_de = 7.1336f,
    .M_u_hat = 0.0213f, .M_w_hat = -7.8773f, .M_q_hat = -2.6347f, .M_de = -64.202f,
};

/* T1: con la A_lon/B_lon de aircraft_a y un escalon de elevador,
   rk4_step_n debe coincidir EXACTAMENTE con rk4_step en 2000 pasos. */
static void test_equivalencia_rk4_step(void)
{
    double A[4][4], B[4];
    build_state_space_matrices(&kAircraftA, A, B);

    const double dt = 0.001;
    double X_ref[4] = {0.0, 0.0, 0.0, 0.0};
    double X_n[4]   = {0.0, 0.0, 0.0, 0.0};
    double Xnext_ref[4], Xnext_n[4];
    int mismatches = 0;

    for (int k = 0; k < 2000; ++k) {
        const double t = k * dt;
        const double U = step_signal(t, 0.1, 0.05);

        rk4_step(longitudinal_dynamics, X_ref, U, dt, A, B, Xnext_ref);
        EXPECT_TRUE(rk4_step_n(linear_dynamics_n, 4u, 1u, X_n, &U, dt,
                               &A[0][0], B, Xnext_n));

        for (int i = 0; i < 4; ++i) {
            if (Xnext_ref[i] != Xnext_n[i]) mismatches++;
            X_ref[i] = Xnext_ref[i];
            X_n[i]   = Xnext_n[i];
        }
    }
    EXPECT_TRUE(mismatches == 0);
    /* sanidad: el escalon realmente movio el estado */
    EXPECT_TRUE(fabs(X_ref[2]) > 1e-6);
}

/* T2: alabeo puro. Solo A[1][1] = L_p y A[3][1] = 1 (phi_dot = p), B[1][0] = L_da.
   p(t) = -(L_da/L_p) * da * (1 - e^{L_p t}) */
static void test_alabeo_puro_analitico(void)
{
    const double L_p = -8.4, L_da = 28.98, da = 0.05, dt = 0.001;
    double A[4][4] = {{0}};
    double B[4][2] = {{0}};
    A[1][1] = L_p;
    A[3][1] = 1.0;
    B[1][0] = L_da;

    const double U[2] = {da, 0.0};
    double X[4] = {0.0, 0.0, 0.0, 0.0};
    double Xn[4];

    const int checks_ms[3] = {500, 1000, 3000};
    int c = 0;
    for (int k = 1; k <= 3000; ++k) {
        EXPECT_TRUE(rk4_step_n(linear_dynamics_n, 4u, 2u, X, U, dt,
                               &A[0][0], &B[0][0], Xn));
        for (int i = 0; i < 4; ++i) X[i] = Xn[i];

        if (c < 3 && k == checks_ms[c]) {
            const double t = k * dt;
            const double p_exact = -(L_da / L_p) * da * (1.0 - exp(L_p * t));
            EXPECT_NEAR(X[1], p_exact, 1e-9);
            c++;
        }
    }
    EXPECT_TRUE(c == 3);
}

/* T3: guardas de tamano; X_next no se toca. */
static void test_guardas(void)
{
    double X[RK4_N_MAX + 1u] = {0};
    double A[1] = {0};
    double B[1] = {0};
    double U[1] = {0};
    double Xn[RK4_N_MAX + 1u];
    for (size_t i = 0; i < RK4_N_MAX + 1u; ++i) Xn[i] = 123.0;

    EXPECT_FALSE(rk4_step_n(linear_dynamics_n, RK4_N_MAX + 1u, 1u, X, U, 0.001, A, B, Xn));
    EXPECT_FALSE(rk4_step_n(linear_dynamics_n, 0u, 1u, X, U, 0.001, A, B, Xn));
    EXPECT_TRUE(Xn[0] == 123.0);
}

int main(void)
{
    test_equivalencia_rk4_step();
    test_alabeo_puro_analitico();
    test_guardas();
    TEST_RETURN();
}
