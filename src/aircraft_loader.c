#include "aircraft_loader.h"
#include "sim_settings.h"

#include <string.h>
#include <stddef.h>

/* Modelo por defecto (fallback seguro) -- mismos valores neutros que antes
 * tenia "def" en dynamic_models.c.
 */
static const Params kDefaultParams = {
    .name = "DEFAULT",
    .Alt_m = 0.0f, .V0_ms = 1.0f, .gamma0_deg = 0.0f, .masa_kg = 1.0f,
    .X_u = 0.0f, .X_w = 0.0f, .X_theta = 0.0f,
    .Z_u_hat = 0.0f, .Z_w_hat = 0.0f, .u0_ms = 1.0f, .w0_ms = 0.0f,
    .Z_theta = 0.0f, .Z_de = 0.0f,
    .M_u_hat = 0.0f, .M_w_hat = 0.0f, .M_q_hat = 0.0f, .M_de = 0.0f,
    .L_da = 0.0f, .L_p = 0.0f,
    .delta_a_trim_deg = 0.0f, .delta_e_trim_deg = 0.0f,
    .delta_r_trim_deg = 0.0f, .delta_thr_trim = 0.0f
};

#if (SYSTEM_SIM_ENV == SIM_PLATFORM_PC)

#include <stdio.h>
#include <stdlib.h>

#define AIRCRAFT_JSON_BUF_SIZE   4096u
#define AIRCRAFT_NAME_BUF_SIZE   32u

static char s_name_buf[AIRCRAFT_NAME_BUF_SIZE];

typedef struct {
    const char *key;
    size_t offset;
} FloatField;

/* Fuente unica de verdad clave-json <-> campo de Params. */
static const FloatField kFloatFields[] = {
    { "Alt_m",            offsetof(Params, Alt_m) },
    { "V0_ms",            offsetof(Params, V0_ms) },
    { "gamma0_deg",       offsetof(Params, gamma0_deg) },
    { "masa_kg",          offsetof(Params, masa_kg) },
    { "X_u",              offsetof(Params, X_u) },
    { "X_w",              offsetof(Params, X_w) },
    { "X_theta",          offsetof(Params, X_theta) },
    { "Z_u_hat",          offsetof(Params, Z_u_hat) },
    { "Z_w_hat",          offsetof(Params, Z_w_hat) },
    { "u0_ms",            offsetof(Params, u0_ms) },
    { "w0_ms",            offsetof(Params, w0_ms) },
    { "Z_theta",          offsetof(Params, Z_theta) },
    { "Z_de",             offsetof(Params, Z_de) },
    { "M_u_hat",          offsetof(Params, M_u_hat) },
    { "M_w_hat",          offsetof(Params, M_w_hat) },
    { "M_q_hat",          offsetof(Params, M_q_hat) },
    { "M_de",             offsetof(Params, M_de) },
    { "L_da",             offsetof(Params, L_da) },
    { "L_p",              offsetof(Params, L_p) },
    { "delta_a_trim_deg", offsetof(Params, delta_a_trim_deg) },
    { "delta_e_trim_deg", offsetof(Params, delta_e_trim_deg) },
    { "delta_r_trim_deg", offsetof(Params, delta_r_trim_deg) },
    { "delta_thr_trim",   offsetof(Params, delta_thr_trim) },
};

#define AIRCRAFT_NUM_FLOAT_FIELDS (sizeof(kFloatFields) / sizeof(kFloatFields[0]))

/* Este modulo solo entiende objetos JSON planos: "clave": numero o
 * "clave": "texto", sin anidar ni arrays -- suficiente para los .json de aircraft/.
 * Busca "key" y devuelve un puntero justo despues del ':' que le sigue
 * (y de los espacios en blanco), o NULL si la clave no aparece.
 */
static const char *find_value_start(const char *json, const char *key)
{
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);

    const char *p = strstr(json, needle);
    if (p == NULL) {
        return NULL;
    }
    p += strlen(needle);

    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        p++;
    }
    if (*p != ':') {
        return NULL;
    }
    p++;

    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        p++;
    }
    return p;
}

static bool find_number(const char *json, const char *key, float *out)
{
    const char *p = find_value_start(json, key);
    if (p == NULL) {
        return false;
    }

    char *end = NULL;
    float value = strtof(p, &end);
    if (end == p) {
        return false;
    }

    *out = value;
    return true;
}

static bool find_string(const char *json, const char *key, char *out, size_t out_sz)
{
    const char *p = find_value_start(json, key);
    if (p == NULL || *p != '"') {
        return false;
    }
    p++;

    size_t i = 0;
    while (p[i] != '"' && p[i] != '\0' && i < (out_sz - 1u)) {
        out[i] = p[i];
        i++;
    }
    out[i] = '\0';
    return true;
}

bool Aircraft_LoadParams(const char *json_path, Params *out)
{
    *out = kDefaultParams;

    FILE *fp = fopen(json_path, "r");
    if (fp == NULL) {
        perror("Aircraft_LoadParams: fopen");
        return false;
    }

    char buf[AIRCRAFT_JSON_BUF_SIZE];
    size_t n = fread(buf, 1u, sizeof(buf) - 1u, fp);
    fclose(fp);
    buf[n] = '\0';

    if (n == 0u) {
        fprintf(stderr, "Aircraft_LoadParams: %s esta vacio\n", json_path);
        return false;
    }

    for (size_t i = 0; i < AIRCRAFT_NUM_FLOAT_FIELDS; i++) {
        float value;
        if (find_number(buf, kFloatFields[i].key, &value)) {
            *(float *)((char *)out + kFloatFields[i].offset) = value;
        }
    }

    if (find_string(buf, "name", s_name_buf, sizeof(s_name_buf))) {
        out->name = s_name_buf;
    }

    return true;
}

typedef struct {
    const char *key;
    unsigned    servo_idx;   /* ServoChannel_e */
    size_t      offset;      /* offsetof(ServoCfg_s, campo) */
} ServoField;

/* Fuente unica de verdad clave-json <-> (canal, campo) de ServoCfg_s.
 * dt_s se omite a proposito: es del arnes SIL (paso == tick del scheduler),
 * no del airframe; se queda en el default compilado de servo_sim.c. */
static const ServoField kServoFields[] = {
    { "servo_elevator_tau_s",          SERVO_ELEVATOR, offsetof(ServoCfg_s, tau_s)          },
    { "servo_elevator_k_deg_per_us",   SERVO_ELEVATOR, offsetof(ServoCfg_s, k_deg_per_us)   },
    { "servo_elevator_y0_deg",         SERVO_ELEVATOR, offsetof(ServoCfg_s, y0_deg)         },
    { "servo_elevator_y_min_deg",      SERVO_ELEVATOR, offsetof(ServoCfg_s, y_min_deg)      },
    { "servo_elevator_y_max_deg",      SERVO_ELEVATOR, offsetof(ServoCfg_s, y_max_deg)      },
    { "servo_elevator_rate_limit_dps", SERVO_ELEVATOR, offsetof(ServoCfg_s, rate_limit_dps) },

    { "servo_aileron_tau_s",           SERVO_AILERON,  offsetof(ServoCfg_s, tau_s)          },
    { "servo_aileron_k_deg_per_us",    SERVO_AILERON,  offsetof(ServoCfg_s, k_deg_per_us)   },
    { "servo_aileron_y0_deg",          SERVO_AILERON,  offsetof(ServoCfg_s, y0_deg)         },
    { "servo_aileron_y_min_deg",       SERVO_AILERON,  offsetof(ServoCfg_s, y_min_deg)      },
    { "servo_aileron_y_max_deg",       SERVO_AILERON,  offsetof(ServoCfg_s, y_max_deg)      },
    { "servo_aileron_rate_limit_dps",  SERVO_AILERON,  offsetof(ServoCfg_s, rate_limit_dps) },

    { "servo_rudder_tau_s",            SERVO_RUDDER,   offsetof(ServoCfg_s, tau_s)          },
    { "servo_rudder_k_deg_per_us",     SERVO_RUDDER,   offsetof(ServoCfg_s, k_deg_per_us)   },
    { "servo_rudder_y0_deg",           SERVO_RUDDER,   offsetof(ServoCfg_s, y0_deg)         },
    { "servo_rudder_y_min_deg",        SERVO_RUDDER,   offsetof(ServoCfg_s, y_min_deg)      },
    { "servo_rudder_y_max_deg",        SERVO_RUDDER,   offsetof(ServoCfg_s, y_max_deg)      },
    { "servo_rudder_rate_limit_dps",   SERVO_RUDDER,   offsetof(ServoCfg_s, rate_limit_dps) },

    { "servo_throttle_tau_s",          SERVO_THROTTLE, offsetof(ServoCfg_s, tau_s)          },
    { "servo_throttle_k_deg_per_us",   SERVO_THROTTLE, offsetof(ServoCfg_s, k_deg_per_us)   },
    { "servo_throttle_y0_deg",         SERVO_THROTTLE, offsetof(ServoCfg_s, y0_deg)         },
    { "servo_throttle_y_min_deg",      SERVO_THROTTLE, offsetof(ServoCfg_s, y_min_deg)      },
    { "servo_throttle_y_max_deg",      SERVO_THROTTLE, offsetof(ServoCfg_s, y_max_deg)      },
    { "servo_throttle_rate_limit_dps", SERVO_THROTTLE, offsetof(ServoCfg_s, rate_limit_dps) },
};

#define AIRCRAFT_NUM_SERVO_FIELDS (sizeof(kServoFields) / sizeof(kServoFields[0]))

bool Aircraft_LoadServoCfg(const char *json_path, ServoCfg_s out[SERVO_SIM_N_SERVOS])
{
    if (out == NULL) {
        return false;
    }

    FILE *fp = fopen(json_path, "r");
    if (fp == NULL) {
        perror("Aircraft_LoadServoCfg: fopen");
        return false;
    }

    char buf[AIRCRAFT_JSON_BUF_SIZE];
    size_t n = fread(buf, 1u, sizeof(buf) - 1u, fp);
    fclose(fp);
    buf[n] = '\0';

    if (n == 0u) {
        fprintf(stderr, "Aircraft_LoadServoCfg: %s esta vacio\n", json_path);
        return false;
    }

    for (size_t i = 0; i < AIRCRAFT_NUM_SERVO_FIELDS; i++) {
        float value;
        if (find_number(buf, kServoFields[i].key, &value)) {
            ServoCfg_s *row = &out[kServoFields[i].servo_idx];
            *(float *)((char *)row + kServoFields[i].offset) = value;
        }
    }

    return true;
}

#else /* SYSTEM_SIM_ENV == SIM_PLATFORM_RTOS (u otra): sin filesystem garantizado */

bool Aircraft_LoadParams(const char *json_path, Params *out)
{
    (void)json_path;
    *out = kDefaultParams;
    return false;
}

bool Aircraft_LoadServoCfg(const char *json_path, ServoCfg_s out[SERVO_SIM_N_SERVOS])
{
    (void)json_path;
    (void)out;               /* out[] se queda con los defaults del llamador */
    return false;
}

#endif
