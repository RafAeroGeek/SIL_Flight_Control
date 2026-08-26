#ifndef DYNAMICS_H_INCLUDED
#define DYNAMICS_H_INCLUDED

#include <stddef.h>

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
    float u0;
    float Z_theta;
    float Z_de;

    float M_u_hat;
    float M_w_hat;
    float M_q_hat;
    float M_de;
    // Derivadas dimensionales (lateral)
    float L_da;
    float L_p;

    float delta_a_trim_deg;
    float delta_e_trim_deg;
    float delta_r_trim_deg;
    float delta_thr_trim;
} Params;

// Construye A (4x4) y B (4x1) a partir de Params
void build_state_space_matrices(const Params *p, double A[4][4], double B[4]);

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


// Señales de control
double step_signal(double tiempo, double t0, double valor);
double doublet_signal(double tiempo, double t0, double amplitud, double duracion);
// Señal de rafaga
double gust_signal(double tiempo, double t0, double amplitud, double frequencia_hz);

// Utilidad: ángulo de ataque (rad) con atan2(w, u0 + du)
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
