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

# ---------------------------------------------------------------------------
# Tests unitarios en C (opt-in). El build por defecto NO cambia.
#   cmake -S . -B build -DSIL_BUILD_TESTS=ON
#   ctest --test-dir build --output-on-failure
# ---------------------------------------------------------------------------
option(SIL_BUILD_TESTS "Compilar los tests unitarios en C" OFF)

if(SIL_BUILD_TESTS)
    enable_testing()

    add_executable(test_servo_cfg
        test/test_servo_cfg.c
        src/servo_sim.c
        src/dynamics.c
    )
    target_include_directories(test_servo_cfg PRIVATE src test)
    target_compile_options(test_servo_cfg PRIVATE -Wall)
    target_link_libraries(test_servo_cfg PRIVATE m)
    add_test(NAME test_servo_cfg COMMAND test_servo_cfg)

    add_executable(test_aircraft_loader_servo
        test/test_aircraft_loader_servo.c
        src/aircraft_loader.c
        src/servo_sim.c
        src/dynamics.c
    )
    target_include_directories(test_aircraft_loader_servo PRIVATE src test)
    target_compile_options(test_aircraft_loader_servo PRIVATE -Wall)
    target_link_libraries(test_aircraft_loader_servo PRIVATE m)
    target_compile_definitions(test_aircraft_loader_servo PRIVATE
        AIRCRAFT_JSON_REAL="${CMAKE_SOURCE_DIR}/aircraft/ss_v1.json"
        TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}"
    )
    add_test(NAME test_aircraft_loader_servo COMMAND test_aircraft_loader_servo)
endif()
