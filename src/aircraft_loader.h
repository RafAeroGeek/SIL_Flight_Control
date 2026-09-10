#ifndef AIRCRAFT_LOADER_H_INCLUDED
#define AIRCRAFT_LOADER_H_INCLUDED

#include <stdbool.h>
#include "dynamics.h"
#include "servo_sim.h"   /* ServoCfg_s, SERVO_SIM_N_SERVOS, ServoChannel_e */

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

/* Superpone la config de servos leida de un JSON plano por-aeronave sobre un
 * arreglo que el llamador YA relleno con los defaults compilados (via
 * Servos_GetDefaultCfg). Solo se sobreescriben los campos cuya clave
 * "servo_<canal>_<campo>" aparece en el archivo. dt_s NUNCA se toca: es un
 * parametro del arnes SIL (paso de integracion == tick del scheduler), no del
 * airframe.
 *
 * A diferencia de Aircraft_LoadParams, NO resetea out[] a un default interno:
 * los defaults de servo son propiedad de servo_sim.c (g_cfg) y duplicarlos aqui
 * seria una segunda fuente de verdad sin proteccion de _Static_assert. En
 * cualquier fallo (archivo faltante/vacio, plataforma RTOS) out[] queda intacto
 * y se devuelve false.
 */
bool Aircraft_LoadServoCfg(const char *json_path,
                           ServoCfg_s out[SERVO_SIM_N_SERVOS]);

#ifdef __cplusplus
}
#endif

#endif // AIRCRAFT_LOADER_H_INCLUDED
