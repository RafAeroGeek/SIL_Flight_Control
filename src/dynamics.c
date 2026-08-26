#define DYNAMICS_C_INCLUDED

#include "dynamics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void build_state_space_matrices(const Params *p, double A[4][4], double B[4])
{
    // Matriz A
    A[0][0] = p->X_u;      A[0][1] = p->X_w;      A[0][2] = 0.0;       A[0][3] = p->X_theta;
    A[1][0] = p->Z_u_hat;  A[1][1] = p->Z_w_hat;  A[1][2] = p->u0;     A[1][3] = p->Z_theta;
    A[2][0] = p->M_u_hat;  A[2][1] = p->M_w_hat;  A[2][2] = p->M_q_hat;A[2][3] = 0.0;
    A[3][0] = 0.0;         A[3][1] = 0.0;         A[3][2] = 1.0;       A[3][3] = 0.0;

    // Matriz B (4x1) – usando Z_de y M_de directamente (como en tu ejemplo)
    B[0] = 0.0;
    B[1] = p->Z_de;
    B[2] = p->M_de;
    B[3] = 0.0;
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
    return atan2(dw, p->u0 + du);
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
