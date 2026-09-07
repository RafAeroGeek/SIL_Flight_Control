cmake_minimum_required(VERSION 3.16)

project(SIL_Flight_Control
        VERSION 0.1.0
        DESCRIPTION "Simulador SIL de control de vuelo (dinamica longitudinal + sensores)"
        LANGUAGES C)

# El .cbp original usaba gcc con -Wall, -g en Debug y -O2 en Release.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Si no se pide un tipo de build, usar Debug (equivalente al target por defecto de Code::Blocks).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Tipo de build" FORCE)
endif()

# Genera compile_commands.json: KDevelop y los editores lo usan para
# autocompletado y para saltar a definiciones.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(SIL_Flight_Control
    src/main.c
    src/dynamics.c
    src/aircraft_loader.c
    src/flight_management.c
    src/flight_sim.c
    src/pilot_sim.c
    src/sensors_sim.c
    src/servo_sim.c
    src/sim_PWM_processing.c
    src/sim_task_management.c
)

target_include_directories(SIL_Flight_Control PRIVATE src)

target_compile_options(SIL_Flight_Control PRIVATE -Wall)

# libm: el codigo usa sin/cos/sqrt/fabs. En Windows venia implicita,
# en Linux hay que enlazarla a mano.
target_link_libraries(SIL_Flight_Control PRIVATE m)

# El programa escribe en "data/SIL_sim_servos2.csv", ruta relativa al
# directorio de trabajo. Ejecutar siempre desde la raiz del proyecto.
