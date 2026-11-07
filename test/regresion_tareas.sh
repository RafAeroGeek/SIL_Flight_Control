#!/usr/bin/env bash
# Regresion bit a bit del CSV de la simulacion, todas las columnas, en las
# dos rutinas del piloto:
#   - ROUTINE_LAT_DIR    (default: MANUAL todo el vuelo)
#   - ROUTINE_YAW_DAMPER (MANUAL -> GYRO_STAB en t = 5 s)
#
# Uso (desde la raiz del repo):
#     bash test/regresion_tareas.sh
#
# Las lineas base son locales (data/*.csv esta en .gitignore); ver la
# seccion 4 de docs/PLAN_FLIGHT_TASKS.md para generarlas.
# Sale con 0 si las dos rutinas son identicas y con 1 si alguna difiere.
set -euo pipefail

BASE_LAT=data/baseline_f140925_latdir.csv
BASE_YD=data/baseline_f140925_yd.csv
CAND_LAT=data/cand_latdir.csv
CAND_YD=data/cand_yd.csv
SIM_CSV=data/SIL_sim_servos2.csv

if [[ ! -f CMakeLists.txt || ! -f test/compare_csv_cols.py || ! -d data ]]; then
    echo "ERROR: correr desde la raiz del repo (bash test/regresion_tareas.sh)" >&2
    exit 1
fi
for f in "$BASE_LAT" "$BASE_YD"; do
    if [[ ! -f "$f" ]]; then
        echo "ERROR: falta la linea base $f (ver docs/PLAN_FLIGHT_TASKS.md, seccion 4)" >&2
        exit 1
    fi
done

# Configura, compila y corre una rutina; copia el CSV al candidato.
#   $1 = directorio de build, $2 = CSV candidato, $3.. = argumentos extra de cmake
correr_rutina() {
    local build_dir=$1 cand=$2
    shift 2
    cmake -S . -B "$build_dir" "$@" > /dev/null
    cmake --build "$build_dir" > /dev/null
    "./$build_dir/SIL_Flight_Control" > /dev/null
    cp "$SIM_CSV" "$cand"
}

correr_rutina build_reg_lat "$CAND_LAT"
correr_rutina build_reg_yd "$CAND_YD" -DCMAKE_C_FLAGS=-DSIM_PILOT_ROUTINE=ROUTINE_YAW_DAMPER

# Compara una rutina; imprime el detalle si falla y una linea de resumen.
#   $1 = nombre, $2 = baseline, $3 = candidato
comparar() {
    local nombre=$1 base=$2 cand=$3 salida rc=0
    salida=$(python3 test/compare_csv_cols.py "$base" "$cand" --all) || rc=$?
    if [[ $rc -eq 0 ]]; then
        echo "$nombre: $salida"
    else
        echo "$salida"
        echo "$nombre: FALLA ($cand vs $base)"
    fi
    return $rc
}

fallo=0
comparar latdir "$BASE_LAT" "$CAND_LAT" || fallo=1
comparar yd "$BASE_YD" "$CAND_YD" || fallo=1
exit $fallo
