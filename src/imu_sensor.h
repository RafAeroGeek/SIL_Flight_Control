/* imu_sensor.h — ruido de IMU (3 giros + 3 acelerómetros) para el SIL. Ajustado a un log real de 50 Hz.
 * Modelo (por canal, un paso de dt):
 *   y = u + b0 + b_rw + noise_scale * ( n_w + n_ou + m * vib )
 * Orden de canales: GyrX GyrY GyrZ AccX AccY AccZ. Unidades: rad/s y m/s^2 (igual que el log).
 * Los parámetros viven en imu_params.h (generado por imu_export_c.py). Ver docs/teoria/sensores/imu_noise/Reporte_Ruido_IMU.md. */
#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H
#include <math.h>
#include <stdint.h>
#include "imu_params.h"

#define IMU_PI 3.14159265358979323846   /* M_PI no existe con -std=c11 */
#define IMU_N 6
#define IMU_PROFILE_FLOOR 0   /* en reposo, motor apagado (el más silencioso) */
#define IMU_PROFILE_IDLE  1   /* en reposo, motor en marcha (menos fiable) */

typedef struct {
    double phi_c, sig_w, sig_c, ou;                 /* OU + blanco */
    double a1[2], a2[2], g[2], x1[2], x2[2];        /* resonadores AR(2) de vibración */
    double vib_flat;
    double b0, b_rw;                                /* sesgo */
} ImuCh;

typedef struct {
    ImuCh ch[IMU_N];
    double dt, noise_scale, white_scale, vib_scale, bias_scale, bias_rw_k[IMU_N];
    double env_phi, env_g;                          /* envolvente interna AR(1) */
    uint64_t s[2]; int have_spare; double spare;
} ImuSensor;

static inline uint64_t imu_rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
static inline uint64_t imu_next(ImuSensor *p) {      /* xoroshiro128+ */
    uint64_t s0 = p->s[0], s1 = p->s[1], r = s0 + s1;
    s1 ^= s0; p->s[0] = imu_rotl(s0, 24) ^ s1 ^ (s1 << 16); p->s[1] = imu_rotl(s1, 37); return r;
}
static inline double imu_unif(ImuSensor *p) { return ((imu_next(p) >> 11) + 0.5) * (1.0 / 9007199254740992.0); }
static inline double imu_randn(ImuSensor *p) {
    if (p->have_spare) { p->have_spare = 0; return p->spare; }
    double u = imu_unif(p), v = imu_unif(p), r = sqrt(-2.0 * log(u));
    p->spare = r * sin(6.283185307179586 * v); p->have_spare = 1; return r * cos(6.283185307179586 * v);
}

/* Varianza de un AR(2) x = a1 x1 + a2 x2 + g w  ->  ganancia g para varianza sigma^2 */
static inline double imu_ar2_gain(double a1, double a2, double sigma) {
    return sigma * sqrt(((1.0 + a2) * ((1.0 - a2) * (1.0 - a2) - a1 * a1)) / (1.0 - a2));
}

static inline void imu_init(ImuSensor *p, int profile, double dt, uint64_t seed) {
    p->dt = dt; p->noise_scale = 1.0; p->white_scale = 1.0; p->vib_scale = 1.0; p->bias_scale = 1.0;
    uint64_t z = seed + 0x9E3779B97F4A7C15ULL;      /* splitmix64 para sembrar */
    for (int i = 0; i < 2; i++) { z += 0x9E3779B97F4A7C15ULL; uint64_t t = z;
        t = (t ^ (t >> 30)) * 0xBF58476D1CE4E5B9ULL; t = (t ^ (t >> 27)) * 0x94D049BB133111EBULL; p->s[i] = t ^ (t >> 31); }
    p->have_spare = 0;
    p->env_phi = exp(-dt / IMU_ENV_TAU_S);
    p->env_g = imu_randn(p);
    for (int j = 0; j < IMU_N; j++) {
        const ImuStaticParam *sp = &IMU_STATIC[profile][j]; const ImuVibParam *vp = &IMU_VIB[j]; ImuCh *c = &p->ch[j];
        c->sig_w = sp->sigma_w; c->sig_c = sp->sigma_c;
        c->phi_c = sp->tau_c > 0.0 ? exp(-dt / sp->tau_c) : 0.0;
        c->ou = sp->tau_c > 0.0 ? sp->sigma_c * imu_randn(p) : 0.0;
        c->vib_flat = vp->sigma_flat;
        for (int k = 0; k < 2; k++) {
            double r = exp(-IMU_PI * (vp->res[k].f0_hz / vp->res[k].Q) * dt), th = 2.0 * IMU_PI * vp->res[k].f0_hz * dt;
            c->a1[k] = 2.0 * r * cos(th); c->a2[k] = -r * r; c->g[k] = imu_ar2_gain(c->a1[k], c->a2[k], vp->res[k].sigma);
            c->x1[k] = c->x2[k] = 0.0;
        }
        c->b0 = IMU_BIAS_SIGMA[j] * imu_randn(p); c->b_rw = 0.0; p->bias_rw_k[j] = 0.0;
    }
}

/* Un paso. u[6] = valor verdadero (p,q,r y fuerza específica en ejes cuerpo). y[6] = salida del sensor.
 * flight: 1 suma la vibración. env: envolvente externa (>=0, p. ej. función de la potencia del motor, con E[env^2]~1 en
 * crucero) o <0 para usar la envolvente log-normal interna. */
static inline void imu_step(ImuSensor *p, const double u[IMU_N], double y[IMU_N], int flight, double env) {
    double m = 0.0;
    if (flight) {
        if (env >= 0.0) m = env;
        else {
            p->env_g = p->env_phi * p->env_g + sqrt(1.0 - p->env_phi * p->env_phi) * imu_randn(p);
            m = exp(IMU_ENV_S_LOG * p->env_g - IMU_ENV_S_LOG * IMU_ENV_S_LOG);
        }
    }
    for (int j = 0; j < IMU_N; j++) {
        ImuCh *c = &p->ch[j];
        if (c->phi_c > 0.0) c->ou = c->phi_c * c->ou + c->sig_c * sqrt(1.0 - c->phi_c * c->phi_c) * imu_randn(p);
        double n = p->white_scale * c->sig_w * imu_randn(p) + c->ou;
        if (flight) {
            double v = c->vib_flat * imu_randn(p);
            for (int k = 0; k < 2; k++) {
                double x = c->a1[k] * c->x1[k] + c->a2[k] * c->x2[k] + c->g[k] * imu_randn(p);
                c->x2[k] = c->x1[k]; c->x1[k] = x; v += x;
            }
            n += p->vib_scale * m * v;
        }
        if (p->bias_rw_k[j] > 0.0) c->b_rw += p->bias_rw_k[j] * sqrt(p->dt) * imu_randn(p);
        y[j] = u[j] + p->bias_scale * (c->b0 + c->b_rw) + p->noise_scale * n;
    }
}
#endif
