#define DYNAMICS_C_INCLUDED

#include "dynamics.h"
#include "sim_settings.h"
#include <math.h>

#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
#include <stdio.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void build_state_space_matrices(const Params *p, double A[4][4], double B[4])
{
    // Matriz A
    A[0][0] = p->X_u;      A[0][1] = p->X_w;      A[0][2] = 0.0;       A[0][3] = p->X_theta;
    A[1][0] = p->Z_u_hat;  A[1][1] = p->Z_w_hat;  A[1][2] = p->u0_ms;  A[1][3] = p->Z_theta;
    A[2][0] = p->M_u_hat;  A[2][1] = p->M_w_hat;  A[2][2] = p->M_q_hat;A[2][3] = 0.0;
    A[3][0] = 0.0;         A[3][1] = 0.0;         A[3][2] = 1.0;       A[3][3] = 0.0;

    // Matriz B (4x1) – usando Z_de y M_de directamente (como en tu ejemplo)
    B[0] = 0.0;
    B[1] = p->Z_de;
    B[2] = p->M_de;
    B[3] = 0.0;
}

/* Misma g que el literal 9.81 de sensors_sim.c */
#define DYN_GRAVITY_MPS2   9.81

/*
 * Supuestos del modelo lateral-direccional:
 *  - Ejes de estabilidad (w0 = 0 en la ecuacion lateral). El longitudinal
 *    esta en ejes cuerpo (w0_ms != 0); el termino +w0*dp de la ecuacion de
 *    fuerza lateral en ejes cuerpo se desprecia en esta fase.
 *  - u0 = u0_ms ; theta0 = theta0_deg (cabeceo de trim).
 *  - Canales longitudinal y lateral desacoplados (modelo lineal).
 *  - Sin dpsi: 4 estados.
 *
 * El JSON trae L_x, N_x sin asterisco. Con D = 1 - I_xz^2/(I_x*I_z):
 *   L*x = L_x/D ; N*x = N_x/D
 *   fila L: L*x + (I_xz/I_x)*N*x ; fila N: N*x + (I_xz/I_z)*L*x
 * para x en {v, p, r, da, dr}. Con I_xz = 0 queda la forma de Nelson 5.x.
 */
void build_lateral_matrices(const Params *p, double A[4][4], double B[4][2])
{
    double Ix  = (double)p->I_x;
    double Iz  = (double)p->I_z;
    double Ixz = (double)p->I_xz;

    if (Ix <= 0.0 || Iz <= 0.0 || (1.0 - Ixz*Ixz / (Ix*Iz)) <= 0.0) {
#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)
        fprintf(stderr, "build_lateral_matrices: inercias invalidas "
                        "(I_x=%g, I_z=%g, I_xz=%g); se usa I_xz = 0\n", Ix, Iz, Ixz);
#endif
        Ix = 1.0; Iz = 1.0; Ixz = 0.0;
    }

    const double D  = 1.0 - Ixz*Ixz / (Ix*Iz);
    const double kL = Ixz / Ix;
    const double kN = Ixz / Iz;

    /* L*, N* sin acoplar */
    const double Ls[5] = { p->L_v/D, p->L_p/D, p->L_r/D, p->L_da/D, p->L_dr/D };
    const double Ns[5] = { p->N_v/D, p->N_p/D, p->N_r/D, p->N_da/D, p->N_dr/D };
    double Lc[5], Nc[5];
    for (int k = 0; k < 5; ++k) {
        Lc[k] = Ls[k] + kL * Ns[k];
        Nc[k] = Ns[k] + kN * Ls[k];
    }

    const double u0     = (double)p->u0_ms;
    const double theta0 = (double)p->theta0_deg * M_PI / 180.0;

    // Matriz A
    A[0][0] = p->Y_v;  A[0][1] = p->Y_p;  A[0][2] = -(u0 - p->Y_r); A[0][3] = DYN_GRAVITY_MPS2 * cos(theta0);
    A[1][0] = Lc[0];   A[1][1] = Lc[1];   A[1][2] = Lc[2];          A[1][3] = 0.0;
    A[2][0] = Nc[0];   A[2][1] = Nc[1];   A[2][2] = Nc[2];          A[2][3] = 0.0;
    A[3][0] = 0.0;     A[3][1] = 1.0;     A[3][2] = 0.0;            A[3][3] = 0.0;

    // Matriz B (4x2): columnas [da, dr]
    B[0][0] = 0.0;     B[0][1] = p->Y_dr;
    B[1][0] = Lc[3];   B[1][1] = Lc[4];
    B[2][0] = Nc[3];   B[2][1] = Nc[4];
    B[3][0] = 0.0;     B[3][1] = 0.0;
}

void longitudinal_dynamics(const double X[4], double U,
                           const double A[4][4], const double B[4],
                           double Xdot[4])
{
    // Xdot = A*X + B*U
    for (int i = 0; i < 4; ++i) {
        double sum = 0.0;
        for (int j = 0; j < 4; ++j) sum += A[i][j] * X[j];
        Xdot[i] = sum + B[i] * U;
    }
}


void first_order_dynamics(const double y, const double u,
                   const double a, const double b,
                   double *ydot)
{
    // ecuacion del roll -> pdot = L_p * p + L_da * delta_a
    *ydot = a * y + b * u;
}


/*
void servo_dynamics(const double y, const double u,
                   const double a, const double b,
                   double *ydot)
{
    // La dinamica del servo supuesto a un sistema de 1er Orden
    *ydot = a * y + b * u;
}
*/

void rk4_step(DynFunc f,
              const double X[4], double U, double dt,
              const double A[4][4], const double B[4],
              double X_next[4])
{
    double k1[4], k2[4], k3[4], k4[4];
    double Xt[4];

    f(X, U, A, B, k1);

    for (int i = 0; i < 4; ++i) Xt[i] = X[i] + 0.5 * dt * k1[i];
    f(Xt, U, A, B, k2);

    for (int i = 0; i < 4; ++i) Xt[i] = X[i] + 0.5 * dt * k2[i];
    f(Xt, U, A, B, k3);

    for (int i = 0; i < 4; ++i) Xt[i] = X[i] + dt * k3[i];
    f(Xt, U, A, B, k4);

    for (int i = 0; i < 4; ++i) {
        X_next[i] = X[i] + (dt/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    }
}

void linear_dynamics_n(size_t n, size_t m,
                       const double *X, const double *U,
                       const double *A, const double *B,
                       double *Xdot)
{
    // Xdot = A*X + B*U  (mismo orden de operaciones que longitudinal_dynamics:
    // con m = 1 el resultado es bit a bit identico)
    for (size_t i = 0; i < n; ++i) {
        double sum = 0.0;
        for (size_t j = 0; j < n; ++j) sum += A[i*n + j] * X[j];
        for (size_t k = 0; k < m; ++k) sum += B[i*m + k] * U[k];
        Xdot[i] = sum;
    }
}

bool rk4_step_n(DynFuncN f, size_t n, size_t m,
                const double *X, const double *U, double dt,
                const double *A, const double *B,
                double *X_next)
{
    if (n == 0u || n > RK4_N_MAX) return false;

    double k1[RK4_N_MAX], k2[RK4_N_MAX], k3[RK4_N_MAX], k4[RK4_N_MAX];
    double Xt[RK4_N_MAX];

    f(n, m, X, U, A, B, k1);

    for (size_t i = 0; i < n; ++i) Xt[i] = X[i] + 0.5 * dt * k1[i];
    f(n, m, Xt, U, A, B, k2);

    for (size_t i = 0; i < n; ++i) Xt[i] = X[i] + 0.5 * dt * k2[i];
    f(n, m, Xt, U, A, B, k3);

    for (size_t i = 0; i < n; ++i) Xt[i] = X[i] + dt * k3[i];
    f(n, m, Xt, U, A, B, k4);

    for (size_t i = 0; i < n; ++i) {
        X_next[i] = X[i] + (dt/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    }
    return true;
}

double step_signal(double tiempo, double t0, double valor)
{
    return (tiempo >= t0) ? valor : 0.0;
}

double doublet_signal(double tiempo, double t0, double amplitud, double duracion)
{
    if (tiempo >= t0 && tiempo < t0 + duracion) return amplitud;
    if (tiempo >= t0 + duracion && tiempo < t0 + 2.0*duracion) return -amplitud;
    return 0.0;
}

double gust_signal(double tiempo, double t0, double amplitud, double frequencia_hz)
{
    double duracion = 1/frequencia_hz;

    if (tiempo >= t0 && tiempo < t0 + duracion)
    {
        return (amplitud / 2) * (1  - cos(2 * M_PI * frequencia_hz * (tiempo - t0))) ;
    }
    return 0.0;
}

double compute_alpha(const Params *p, double du, double dw)
{
    return atan2(dw, p->u0_ms + du);
}

double saturacion(double dato, double sat_min, double sat_max)
{
    //
    if(dato < sat_min)
    {

        return sat_min;
    }
    else if(dato > sat_max)
    {

        return sat_max;
    }
    else
    {

        //
        return dato;
    }
}


void rk4_step_1st(DynFunc_1st f_st,
                  double y, double u, double dt,
                  double a, double b,
                  double *y_next)
{
    double k1, k2, k3, k4;
    double yt;

    f_st(y, u, a, b, &k1);

    yt = y + 0.5 * dt * k1;
    f_st(yt, u, a, b, &k2);

    yt = y + 0.5 * dt * k2;
    f_st(yt, u, a, b, &k3);

    yt = y + dt * k3;
    f_st(yt, u, a, b, &k4);

    *y_next = y + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}
