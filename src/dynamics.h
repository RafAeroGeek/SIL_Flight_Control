#ifndef DYNAMICS_H_INCLUDED
#define DYNAMICS_H_INCLUDED

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Meta
    const char *name;
    // Condición trim
    float Alt_m;        // Altitud [m]
    float V0_ms;        // Velocidad trim [m/s]
    float gamma0_deg;   // ángulo de trayectoria inicial [deg]
    float masa_kg;      // masa [kg]

    // Derivadas dimensionales (longitudinal)
    float X_u;
    float X_w;
    float X_theta;

    float Z_u_hat;
    float Z_w_hat;
    float u0_ms;
    float w0_ms;
    float Z_theta;
    float Z_de;

    float M_u_hat;
    float M_w_hat;
    float M_q_hat;
    float M_de;
    // Derivadas dimensionales (lateral)
    // Nelson cap. 5, ejes de estabilidad. L_x = Q*S*b*C_lx/I_x y
    // N_x = Q*S*b*C_nx/I_z (sin asterisco: la correccion por I_xz se hace en C).
    float theta0_deg;   // angulo de cabeceo de trim [deg]
    float I_x;          // [kg*m^2]
    float I_z;          // [kg*m^2]
    float I_xz;         // [kg*m^2]

    float Y_v;          // [1/s]
    float Y_p;          // [m/s]
    float Y_r;          // [m/s]
    float Y_dr;         // [m/s^2]

    float L_v;          // [1/(m*s)]
    float L_p;          // [1/s]
    float L_r;          // [1/s]
    float L_da;         // [1/s^2]
    float L_dr;         // [1/s^2]

    float N_v;          // [1/(m*s)]
    float N_p;          // [1/s]
    float N_r;          // [1/s]
    float N_da;         // [1/s^2]
    float N_dr;         // [1/s^2]

    float delta_a_trim_deg;
    float delta_e_trim_deg;
    float delta_r_trim_deg;
    float delta_thr_trim;
} Params;

// Construye A (4x4) y B (4x1) a partir de Params
void build_state_space_matrices(const Params *p, double A[4][4], double B[4]);

// Construye A (4x4) y B (4x2) lateral-direccionales (Nelson cap. 5) con
// correccion por producto de inercia I_xz.
// x = [dv, dp, dr, dphi] ; u = [da, dr] (rad)
void build_lateral_matrices(const Params *p, double A[4][4], double B[4][2]);

// xdot = A*x + B*u  (x, xdot de tamaño 4; B es 4x1; u escalar)
void longitudinal_dynamics(const double X[4], double U,
                           const double A[4][4], const double B[4],
                           double Xdot[4]);

// Integrador RK4 de un paso
typedef void (*DynFunc)(const double X[4], double U,
                        const double A[4][4], const double B[4],
                        double Xdot[4]);

void rk4_step(DynFunc f,
              const double X[4], double U, double dt,
              const double A[4][4], const double B[4],
              double X_next[4]);

// Integrador RK4 generico: n estados / m entradas.
// Buffers internos en stack (sin malloc): n <= RK4_N_MAX.
#define RK4_N_MAX 12u

// Xdot = A*X + B*U ; A row-major n x n, B row-major n x m
typedef void (*DynFuncN)(size_t n, size_t m,
                         const double *X, const double *U,
                         const double *A, const double *B,
                         double *Xdot);

void linear_dynamics_n(size_t n, size_t m,
                       const double *X, const double *U,
                       const double *A, const double *B,
                       double *Xdot);

// Devuelve false (y no toca X_next) si n > RK4_N_MAX o n == 0
bool rk4_step_n(DynFuncN f, size_t n, size_t m,
                const double *X, const double *U, double dt,
                const double *A, const double *B,
                double *X_next);


// Señales de control
double step_signal(double tiempo, double t0, double valor);
double doublet_signal(double tiempo, double t0, double amplitud, double duracion);
// Señal de rafaga
double gust_signal(double tiempo, double t0, double amplitud, double frequencia_hz);

// Utilidad: ángulo de ataque (rad) con atan2(w, u0_ms + du)
double compute_alpha(const Params *p, double du, double dw);

// saturacion de señales
double saturacion(double dato, double sat_min, double sat_max);


// ydot = a*y + b*u
void first_order_dynamics(double y, double u, double a, double b, double *ydot);

/*
void servo_dynamics(const double y, const double u,
                   const double a, const double b,
                   double *ydot);
*/

//Modelo funcion dinamica de 1er orden.
typedef void (*DynFunc_1st)(double y, double u, double a, double b, double *ydot);

//Solucion a 1er orden
void rk4_step_1st(DynFunc_1st f_st,
                  double y, double u, double dt,
                  double a, double b,
                  double *y_next);

#ifdef __cplusplus
}
#endif

#endif // DYNAMICS_H_INCLUDED
