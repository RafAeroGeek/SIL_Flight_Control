#include "dynamic_models.h"

Params get_dynamic_model(DynamicModelId id)
{
    // Modelo por defecto (fallback seguro)
    const Params def = {
        .name = "DEFAULT",
        .Alt_m = 0.0f, .V0_ms = 1.0f, .gamma0_deg = 0.0f, .masa_kg = 1.0f,
        .X_u = 0.0f, .X_w = 0.0f, .X_theta = 0.0f,
        .Z_u_hat = 0.0f, .Z_w_hat = 0.0f, .u0 = 1.0f, .Z_theta = 0.0f, .Z_de = 0.0f,
        .M_u_hat = 0.0f, .M_w_hat = 0.0f, .M_q_hat = 0.0f, .M_de = 0.0f,
        .L_da = 0.0f, .L_p = 0.0f, .delta_a_trim_deg = 0.0f, .delta_e_trim_deg = 0.0f,
        .delta_r_trim_deg = 0.0f, .delta_thr_trim = 0.0f
    };

    switch (id)
    {
    case DYN_MODEL_SS_V1:
        return (Params){
            .name       = "SS_v1",
            .Alt_m      = 1000.0f,
            .V0_ms      = 36.0f,
            .gamma0_deg = -15.0f,
            .masa_kg    = 4.7f,
            .X_u        = -0.0498f,
            .X_w        = -0.0169f,
            .X_theta    = -9.724f,
            .Z_u_hat    = -0.5484f,
            .Z_w_hat    = -2.557f,
            .u0         = 35.6997f,
            .Z_theta    = 1.2963f,
            .Z_de       = 7.1336f,
            .M_u_hat    = 0.0213f,
            .M_w_hat    = -7.8773f,
            .M_q_hat    = -2.6347f,
            .M_de       = -64.202f,
            .L_da       = 4.66f,
            .L_p        = -1.30f,
            .delta_a_trim_deg = 0.0f,
            .delta_e_trim_deg = 0.0f,
            .delta_r_trim_deg = 0.0f,
            .delta_thr_trim = 0.0f
        };

    case DYN_MODEL_SS_V2:
        return (Params){
            .name = "SS_v2",
            // ... otro set
        };

    default:
        return def;
    }
}
