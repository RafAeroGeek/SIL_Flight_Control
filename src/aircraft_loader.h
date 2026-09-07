#ifndef AIRCRAFT_LOADER_H_INCLUDED
#define AIRCRAFT_LOADER_H_INCLUDED

#include <stdbool.h>
#include "dynamics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Carga los parametros de una aeronave desde un archivo .json plano
 * (claves 1:1 con los miembros de Params, ver aircraft/ss_v1.json).
 *
 * *out queda SIEMPRE en un estado valido: arranca en los parametros por
 * defecto seguros y solo se sobreescriben los campos presentes en el JSON.
 *
 * Devuelve true si pudo abrir y parsear json_path; false si no (archivo
 * faltante, demasiado grande, etc. -- el motivo ya se reporta por stderr
 * en la plataforma PC). En la plataforma RTOS no se toca disco: siempre
 * devuelve false con *out en los valores por defecto.
 */
bool Aircraft_LoadParams(const char *json_path, Params *out);

#ifdef __cplusplus
}
#endif

#endif // AIRCRAFT_LOADER_H_INCLUDED
