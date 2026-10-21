/* ===========================================================
 * test_lon_matrices.c
 * Cubre build_state_space_matrices (dynamics.c) en ejes cuerpo:
 *   - w0 = 0, X_de = 0 == forma de Nelson
 *   - w0 != 0 -> A[0][2] = -w0 ; X_de -> B[0]
 * =========================================================== */
#include "dynamics.h"
#include "test_util.h"

static Params base(void)
{
    Params p = {
        .name = "lon",
        .X_u = -0.05f, .X_w = 0.18f, .X_theta = -9.76f,
        .Z_u_hat = -0.78f, .Z_w_hat = -3.10f, .u0_ms = 25.1f, .Z_theta = -1.0f,
        .Z_de = -8.3f,
        .M_u_hat = 0.046f, .M_w_hat = -0.42f, .M_q_hat = -5.3f, .M_de = -50.8f,
    };
    return p;
}

/* T1: w0 = 0 y X_de = 0 -> sin acoplamiento X-q ni elevador en X */
static void test_forma_nelson(void)
{
    const Params p = base();
    double A[4][4], B[4];
    build_state_space_matrices(&p, A, B);

    EXPECT_TRUE(A[0][2] == 0.0);
    EXPECT_TRUE(B[0] == 0.0);
    EXPECT_TRUE(A[1][2] == (double)25.1f);
    EXPECT_TRUE(A[0][3] == (double)-9.76f);
    EXPECT_TRUE(B[1] == (double)-8.3f);
    EXPECT_TRUE(B[2] == (double)-50.8f);
    EXPECT_TRUE(A[3][2] == 1.0);
}

/* T2: ejes cuerpo con w0 = 2.6 m/s -> du/dt lleva -w0*dq */
static void test_ejes_cuerpo(void)
{
    Params p = base();
    p.w0_ms = 2.6f;
    p.X_de  = 0.85f;
    double A[4][4], B[4];
    build_state_space_matrices(&p, A, B);

    EXPECT_TRUE(A[0][2] == -(double)2.6f);
    EXPECT_TRUE(B[0] == (double)0.85f);
    EXPECT_TRUE(A[1][2] == (double)25.1f);
}

int main(void)
{
    test_forma_nelson();
    test_ejes_cuerpo();
    TEST_RETURN();
}
