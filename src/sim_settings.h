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

/* Rutina de piloto activa (enum RoutineID en pilot_sim.h).
   ROUTINE_LONGITUDINAL = rutina de regresion; ROUTINE_LAT_DIR ejercita yaw y throttle. */
#define 	SIM_PILOT_ROUTINE	ROUTINE_LONGITUDINAL

/********************************************/
/*------------------------------------------*/


/**
 * Definición de Entrada de Radio
 */

#define 	PWM_Futaba			0		/// Definición de futaba por PWM
#define 	SBUS_Futaba			1		/// Definición de futaba por PWM





#endif // SIM_SETTINGS_H_INCLUDED
