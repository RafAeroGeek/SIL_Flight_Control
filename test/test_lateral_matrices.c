/* ===========================================================
 * test_lateral_matrices.c
 * Cubre build_lateral_matrices (dynamics.c):
 *   - Navion con I_xz = 0 == forma de Nelson (PLAN_LATERAL.md 1.1)
 *   - correccion por I_xz != 0 contra calculo a mano (1.2)
 *   - I_x = 0 no produce NaN/Inf
 *   - respuesta libre acotada y espiral estable (rk4_step_n)
 * =========================================================== */
#include "dynamics.h"
#include "test_util.h"

#define EXPECT_NEAR(a, b, tol) do {                                           \
    double _a = (double)(a);                                                  \
    double _b = (double)(b);                                                  \
    if (!(fabs(_a - _b) <= (tol))) {                                          \
        printf("FAIL %s:%d: %s (%.12g) != %s (%.12g)\n",                      \
               __FILE__, __LINE__, #a, _a, #b, _b);                           \
        g_test_fail++;                                                        \
    }                                                                         \
} while (0)

/* Navion (Nelson), valores de PLAN_LATERAL.md 1.4 */
static Params navion(void)
{
    Params p = {
        .name = "navion",
        .u0_ms = 53.6448f, .theta0_deg = 0.0f,
        .I_x = 1420.9f, .I_z = 4786.0f, .I_xz = 0.0f,
        .Y_v = -0.254f, .Y_p = 0.0f, .Y_r = 0.0f, .Y_dr = 3.80f,
        .L_v = -0.29863f, .L_p = -8.40f, .L_r = 2.19f, .L_da = 28.98f, .L_dr = 2.548f,
        .N_v = 0.08370f, .N_p = -0.35f, .N_r = -0.76f, .N_da = -0.2218f, .N_dr = -4.597f,
    };
    return p;
}

/* T1: I_xz = 0 -> matriz 1.1 entrada por entrada.
   Los esperados se castean a float: es la precision con que vive Params. */
static void test_navion_ixz_cero(void)
{
    const Params p = navion();
    double A[4][4], B[4][2];
    build_lateral_matrices(&p, A, B);

    const double tol = 1e-6;
    const double A_exp[4][4] = {
        { -0.254f,   0.0f,   -(53.6448f - 0.0f), 9.81 },
        { -0.29863f, -8.40f,  2.19f,             0.0  },
        {  0.08370f, -0.35f, -0.76f,             0.0  },
        {  0.0,       1.0,    0.0,               0.0  },
    };
    const double B_exp[4][2] = {
        {  0.0,      3.80f   },
        { 28.98f,    2.548f  },
        { -0.2218f, -4.597f  },
        {  0.0,      0.0     },
    };
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) EXPECT_NEAR(A[i][j], A_exp[i][j], tol);
        for (int k = 0; k < 2; ++k) EXPECT_NEAR(B[i][k], B_exp[i][k], tol);
    }
    EXPECT_TRUE(A[0][2] == -((double)53.6448f - 0.0));
    EXPECT_TRUE(A[0][3] == 9.81);
}

/* T2: I_x = 1000, I_z = 2000, I_xz = 100 -> D = 1 - 100^2/(1000*2000) = 0.995 */
static void test_correccion_ixz(void)
{
    Params p = navion();
    p.I_x = 1000.0f; p.I_z = 2000.0f; p.I_xz = 100.0f;

    double A[4][4], B[4][2];
    build_lateral_matrices(&p, A, B);

    const double D = 0.995;
    /* fila L: (L_x + (I_xz/I_x) N_x)/D ; fila N: (N_x + (I_xz/I_z) L_x)/D */
    const double A11 = ((double)-8.40f    + 0.1  * (double)-0.35f)    / D;
    const double A20 = ((double)0.08370f  + 0.05 * (double)-0.29863f) / D;
    const double B21 = ((double)-4.597f   + 0.05 * (double)2.548f)    / D;

    EXPECT_NEAR(A[1][1], A11, 1e-12);
    EXPECT_NEAR(A[2][0], A20, 1e-12);
    EXPECT_NEAR(B[2][1], B21, 1e-12);

    /* Filas Y y phi no cambian */
    EXPECT_NEAR(A[0][0], (double)-0.254f, 1e-12);
    EXPECT_NEAR(B[0][1], (double)3.80f,   1e-12);
    EXPECT_TRUE(A[3][1] == 1.0);
}

/* T3: I_x = 0 -> se trata como I_xz = 0, sin NaN/Inf. */
static void test_ix_cero(void)
{
    Params p = navion();
    p.I_x = 0.0f; p.I_xz = 100.0f;

    double A[4][4], B[4][2];
    build_lateral_matrices(&p, A, B);

    int bad = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) if (!isfinite(A[i][j])) bad++;
        for (int k = 0; k < 2; ++k) if (!isfinite(B[i][k])) bad++;
    }
    EXPECT_TRUE(bad == 0);
    EXPECT_NEAR(A[1][1], (double)-8.40f, 1e-12);
}

/* T4: respuesta libre desde dphi0 = 0.1 rad, 30 s, dt = 1 ms. */
static void test_respuesta_libre_navion(void)
{
    const Params p = navion();
    double A[4][4], B[4][2];
    build_lateral_matrices(&p, A, B);

    const double U[2] = {0.0, 0.0};
    const double dt = 0.001;
    double X[4] = {0.0, 0.0, 0.0, 0.1};
    double Xn[4];
    double max_v = 0.0, max_p = 0.0, max_r = 0.0;

    for (int k = 0; k < 30000; ++k) {
        EXPECT_TRUE(rk4_step_n(linear_dynamics_n, 4u, 2u, X, U, dt,
                               &A[0][0], &B[0][0], Xn));
        for (int i = 0; i < 4; ++i) X[i] = Xn[i];
        if (fabs(X[0]) > max_v) max_v = fabs(X[0]);
        if (fabs(X[1]) > max_p) max_p = fabs(X[1]);
        if (fabs(X[2]) > max_r) max_r = fabs(X[2]);
    }

    EXPECT_TRUE(max_v < 50.0);
    EXPECT_TRUE(max_p < 2.0);
    EXPECT_TRUE(max_r < 2.0);
    /* espiral estable y lento: phi decae pero sigue positivo */
    EXPECT_TRUE(X[3] > 0.0 && X[3] < 0.1);
    printf("T4: dphi(30 s) = %.6f rad, max|dv| = %.4f, max|dp| = %.4f, max|dr| = %.4f\n",
           X[3], max_v, max_p, max_r);
}

int main(void)
{
    test_navion_ixz_cero();
    test_correccion_ixz();
    test_ix_cero();
    test_respuesta_libre_navion();
    TEST_RETURN();
}
