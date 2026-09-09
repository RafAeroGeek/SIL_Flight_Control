#ifndef SIM_SETTINGS_H_INCLUDED
#define SIM_SETTINGS_H_INCLUDED


#define     SIMULATION_TIME_s               10.0f    // Tiempo de simulacion.
#define     SIMULATION_TIME_ms             (SIMULATION_TIME_s * 1000)

/**
 * Definición del entorno de simulacion para PC
 */
#define 	SIM_PLATFORM_PC			    1		/// Definición de aeronave
/**
 * Definición el entorno de simulacion en el RTOS
 */
#define 	SIM_PLATFORM_RTOS			2		/// Definición de simulador


#define 	Coro_sim						0		/// Definición de aeronave CoroStick
#define 	XPlane_sim						1		/// Definición del simulador
#define		Hell_Hawk_sim					48		/// Definición de aeronave M1 (Bluefish) Hell_Hawk
#define     SS_V1_sim                       49

/*------------------------------------------*/
/*********** Seleccion de entorno ***********/
#define 	SYSTEM_SIM_ENV		SIM_PLATFORM_PC			/// Sistema seleccionado
#define 	AIRCRAFT_SIM		Coro_sim	/// Tipo de aeronave seleccionada

/* Ruta del JSON de la aeronave activa, relativa al directorio de trabajo
   (misma convencion que csv_path en main.c: correr siempre desde la raiz
   del repo). Para cambiar de aeronave: edita este string y recompila. */
<<<<<<< HEAD
#define 	AIRCRAFT_JSON_PATH	"aircraft/ugly_stick.json"
=======
#define 	AIRCRAFT_JSON_PATH	"aircraft/ss_v1.json"
>>>>>>> origin/main

/* Rutina de piloto activa (enum RoutineID en pilot_sim.h).
   ROUTINE_LONGITUDINAL = rutina de regresion; ROUTINE_LAT_DIR ejercita yaw y throttle. */
#define 	SIM_PILOT_ROUTINE	ROUTINE_LONGITUDINAL

/* Ganancia stick normalizado ([-1,1] ejes, [0,1] throttle) -> delta PWM [us].
   El piloto emite valores normalizados; main.c los convierte a us antes de los servos. */
#define 	RC_STICK_TO_US		500.0f

/********************************************/
/*------------------------------------------*/


/**
 * Definición de Entrada de Radio
 */

#define 	PWM_Futaba			0		/// Definición de futaba por PWM
#define 	SBUS_Futaba			1		/// Definición de futaba por PWM





#endif // SIM_SETTINGS_H_INCLUDED
