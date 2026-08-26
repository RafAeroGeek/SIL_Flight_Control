#include <stdio.h>  // Para printf
#include "flight_sim.h"

#include <string.h>

/* Helper local */
static void copy4(double dst[4], const double src[4])
{
    for (int i = 0; i < 4; ++i) dst[i] = src[i];
}

void FlightSim_Init(FlightSim *sim, const Params *p, const FlightSimInit *ic)
{
    if (!sim || !p) return;

    if (ic && ic->zero_all_first) {
        memset(sim, 0, sizeof(*sim));
    } else {
        /* mínimo: limpiar lo necesario */
        memset(sim, 0, sizeof(*sim));
    }

    sim->params = *p;
    sim->initialized = true;

    /* Defaults si ic es NULL */
    FlightSimInit def = {0};
    def.t0_s = 0.0;
    def.X0[0] = 0.0; def.X0[1] = 0.0; def.X0[2] = 0.0; def.X0[3] = 0.0;
    def.H0_m = (double)p->Alt_m;

    def.delta_elv0 = 0.0;
    def.delta_ail0 = 0.0;
    def.delta_rud0 = 0.0;
    def.throttle0  = 0.0;

    def.w_g0 = 0.0;
    def.build_ss_matrices = true;
    def.zero_all_first = true;

    const FlightSimInit *cfg = ic ? ic : &def;

    sim->t_s = cfg->t0_s;
    copy4(sim->X, cfg->X0);
    sim->H_m = cfg->H0_m;

    sim->delta_elv = cfg->delta_elv0;
    sim->delta_ail = cfg->delta_ail0;
    sim->delta_rud = cfg->delta_rud0;
    sim->throttle  = cfg->throttle0;

    sim->w_g = cfg->w_g0;

    sim->theta_set = 0.0; /* por claridad */
    sim->p = 0.0;
    sim->p_next = 0.0;

    if (cfg->build_ss_matrices) {
        build_state_space_matrices(&sim->params, sim->A, sim->B);
    }
}

void FlightSim_Reset(FlightSim *sim, const FlightSimInit *ic)
{
    if (!sim || !sim->initialized) return;
    FlightSim_Init(sim, &sim->params, ic);
}


void FlightSim_Actuators_init(FM_Actuators *act, const Params *p) {
    // Validar punteros
    if (act == NULL || p == NULL) {
        // Manejar error o usar valores por defecto
        if (act != NULL) {
            act->aileron  = 0.0f;
            act->elevator = 0.0f;
            act->rudder   = 0.0f;
            act->throttle = 0.0f;
        }
        return;
    }

    // Inicializar con valores trim del modelo
    act->aileron  = p->delta_a_trim_deg;   // Usar p->, no p.
    act->elevator = p->delta_e_trim_deg;   // Usar p->, no p.
    act->rudder   = p->delta_r_trim_deg;   // Usar p->, no p.
    act->throttle = p->delta_thr_trim;     // Probablemente sin _deg
}

void FlightSim_GetActuators(FM_Actuators *act, float delta_ail, float delta_elv, float delta_rud, float throttle)
{
    //
    act->elevator   = delta_elv;
    act->aileron    = delta_ail;
    act->rudder     = delta_rud;
    act->throttle   = throttle;
}

void FlightSim_SetActuators(FlightSim *sim, double delta_elv, double delta_ail, double delta_rud, double throttle)
{
    if (!sim || !sim->initialized) return;

    sim->delta_elv = delta_elv;
    sim->delta_ail = delta_ail;
    sim->delta_rud = delta_rud;
    sim->throttle  = throttle;
}

void FlightSim_SetActuatorsFromFM(FlightSim *sim, const FM_Actuators *act)
{
    if (!sim || !sim->initialized || !act) return;

    /* Aquí tú defines tu convención:
       si act->elevator es [-1,1] conviértelo a rad o a delta_e físico. */
    sim->delta_elv = (double)act->elevator;
    sim->delta_ail = (double)act->aileron;
    sim->delta_rud = (double)act->rudder;
    sim->throttle  = (double)act->throttle;
}


void imprimir_matriz_4x4(double A[4][4], double B[4]) {
    printf("Matriz A (4x4):\n");
    for (int i = 0; i < 4; ++i) {
        printf("  [ ");
        for (int j = 0; j < 4; ++j) {
            printf("%10.5f ", A[i][j]);
        }
        printf("]\n");
    }

    printf("\nMatriz B (4x1):\n  [ ");
    for (int i = 0; i < 4; ++i) {
        printf("%10.5f ", B[i]);
    }
    printf("]^T\n\n");
}

void FlightSim_Step(FlightSim *sim, double dt_s)
{
    if (!sim || !sim->initialized) return;
    if (dt_s <= 0.0) return;

    rk4_step(longitudinal_dynamics, sim->X, sim->delta_elv, dt_s, sim->A , sim->B, sim->Xn);
    copy4(sim->X, sim->Xn);

    sim->t_s += dt_s;
}

double FlightSim_GetTime(const FlightSim *sim)
{
    if (!sim) return 0.0;
    return sim->t_s;
}

void FlightSim_GetX(const FlightSim *sim, double out_X4[4])
{
    if (!sim || !out_X4) return;
    copy4(out_X4, sim->X);
}
