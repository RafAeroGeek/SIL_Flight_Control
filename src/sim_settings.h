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
#define     Aircraft_A_sim                  49

/*------------------------------------------*/
/*********** Seleccion de entorno ***********/
#define 	SYSTEM_SIM_ENV		SIM_PLATFORM_PC			/// Sistema seleccionado
#define 	AIRCRAFT_SIM		Coro_sim	/// Tipo de aeronave seleccionada

/* Ruta del JSON de la aeronave activa, relativa al directorio de trabajo
   (misma convencion que csv_path en main.c: correr siempre desde la raiz
   del repo). Para cambiar de aeronave: edita este string y recompila. */
#define 	AIRCRAFT_JSON_PATH	"aircraft/ugly_stick.json"

/* Ganancias del FCS, misma convencion de ruta que AIRCRAFT_JSON_PATH */
#define 	FCS_GAINS_JSON_PATH	"config/FCS_gains.json"


/* 1 = integra dinamica lateral-direccional ; 0 = solo longitudinal */
#define 	SIL_CONFIG_LATERAL	1


/* Rutina de piloto activa (enum RoutineID en pilot_sim.h).
   ROUTINE_LAT_DIR (por defecto): pitch identico a ROUTINE_LONGITUDINAL mas
   dobletes pequenos de aleron y timon, dentro del rango lineal.
   ROUTINE_LONGITUDINAL: da un escalon de roll = 1.0 -> 500 us -> aleron
   saturado a 20 deg; con el lateral activo lleva phi muy fuera del rango
   lineal. Queda SOLO para la regresion longitudinal.
   ROUTINE_YAW_DAMPER: mismo doblete de timon en MANUAL y con el yaw damper.
   Se puede elegir al compilar:
   cmake -DCMAKE_C_FLAGS=-DSIM_PILOT_ROUTINE=ROUTINE_YAW_DAMPER ... */
#ifndef SIM_PILOT_ROUTINE
#define 	SIM_PILOT_ROUTINE	ROUTINE_LAT_DIR
#endif

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
