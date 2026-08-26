#ifndef FLIGHT_SIM_H
#define FLIGHT_SIM_H

#include <stdint.h>
#include <stdbool.h>

#include "dynamics.h"          // Params, build_state_space_matrices, rk4_step (si aplica)
#include "flight_management.h" // FM_Actuators (opcional, si quieres usarlo aquí)

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * Estado de simulación (planta)
 * ========================= */
typedef struct
{
    /* Tiempo */
    double t_s;

    /* Estados longitudinales (ejemplo Nelson): [du, dw, dq, dtheta] */
    double X[4];
    double Xn[4];

    /* Altitud / otras variables auxiliares */
    double H_m;

    /* Actuadores actuales (ZOH) */
    double delta_elv;   // elevador (rad o norm) - define tu convención
    double delta_ail;   // aileron (rad o norm)
    double delta_rud;   // por si luego lo agregas
    double throttle;    // [0..1] o físico

    /* Viento / perturbaciones */
    double w_g;         // gust / vertical wind u otra perturbación
    double theta_set;   // set interno si lo usas

    /* Ejemplo lateral simple (si lo quieres aquí) */
    double p;           // roll rate
    double p_next;

    /* Matrices lineales (cache) para acelerar si usas SS */
    double A[4][4];
    double B[4];

    /* Copia de parámetros */
    Params params;

    /* Flags */
    bool initialized;
} FlightSim;


/* =========================
 * Config de init (opcional)
 * ========================= */
typedef struct
{
    /* Estado inicial */
    double t0_s;
    double X0[4];
    double H0_m;

    /* Actuadores iniciales */
    double delta_elv0;
    double delta_ail0;
    double delta_rud0;
    double throttle0;

    /* Perturbaciones iniciales */
    double w_g0;

    /* Si true: construye A,B al iniciar */
    bool build_ss_matrices;

    /* Si true: reinicia todo a cero excepto lo dado */
    bool zero_all_first;
} FlightSimInit;




/* API */
void FlightSim_Init(FlightSim *sim, const Params *p, const FlightSimInit *ic);
void FlightSim_Reset(FlightSim *sim, const FlightSimInit *ic);
void FlightSim_Actuators_init(FM_Actuators *act, const Params *p) ;

/* Asigna actuadores */
void FlightSim_GetActuators(FM_Actuators *act, float delta_ail, float delta_elv, float delta_rud, float throttle);

/* Aplica actuadores desde tu manager/control (ZOH) */
void FlightSim_SetActuators(FlightSim *sim, double delta_elv, double delta_ail, double delta_rud, double throttle);

/* Alternativa: si quieres usar FM_Actuators directamente */
void FlightSim_SetActuatorsFromFM(FlightSim *sim, const FM_Actuators *act);

/* Imprimir las matrices A y B del sistema.*/
void imprimir_matriz_4x4(double A[4][4], double B[4]);

/* Paso de simulación (ej. 1 ms) */
void FlightSim_Step(FlightSim *sim, double dt_s);

/* Obtener estado */
double FlightSim_GetTime(const FlightSim *sim);
void   FlightSim_GetX(const FlightSim *sim, double out_X4[4]);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_SIM_H */
