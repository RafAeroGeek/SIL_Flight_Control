/* Uso: ./imu_test perfil(0|1) vuelo(0|1) segundos semilla [envolvente] > salida.csv   (200 Hz, solo ruido: u=0, sin sesgo)
 * envolvente: >= 0 fija, < 0 aleatoria interna (por defecto) */
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include "imu_sensor.h"
int main(int argc, char **argv) {
    if (argc < 5) { fprintf(stderr, "uso: %s perfil(0|1) vuelo(0|1) segundos semilla\n", argv[0]); return 1; }
    int prof = atoi(argv[1]), fl = atoi(argv[2]); double secs = atof(argv[3]); uint64_t seed = strtoull(argv[4], 0, 10);
    double env = (argc > 5) ? atof(argv[5]) : -1.0;
    ImuSensor s; imu_init(&s, prof, 0.005, seed); s.bias_scale = 0.0;
    long n = (long)(secs * 200.0); double u[6] = {0}, y[6];
    for (long i = 0; i < n; i++) { imu_step(&s, u, y, fl, env);
        printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", y[0], y[1], y[2], y[3], y[4], y[5]); }
    return 0;
}
