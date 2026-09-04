# -*- coding: utf-8 -*-
"""Configuracion central de flightreview: alias de columnas, tabla de modos de
vuelo y constantes de rendimiento.

`FIELD_ALIASES` esta tomado del mapeo flexible de `python/plot_state_exp_v3.py`
(`build_column_map`), reexpresado como {nombre_canonico: [candidatos...]} para
que el parser resuelva un CSV aunque las columnas tengan nombres alternativos.
"""

from __future__ import annotations

import os

# ---------------------------------------------------------------------------
# Nombres canonicos -> posibles nombres reales en el CSV (primer match gana).
# El nombre canonico es el que usan schema.ColumnMap y las graficas.
# ---------------------------------------------------------------------------
FIELD_ALIASES: dict[str, list[str]] = {
    # Eje temporal
    "t": ["t_s", "time_s", "t", "time", "timestamp"],

    # Logica de modo / armado
    "mode": ["mode", "flight_mode", "pilot_mode", "mode_switch"],
    "arm": ["arm", "armed", "arm_switch"],

    # --- Actitud estimada (rad) ---
    # El simulador es longitudinal: hoy solo existe pitch como dtheta_rad.
    "roll": ["roll_rad", "roll", "phi_rad", "phi"],
    "pitch": ["dtheta_rad", "dtheta", "pitch_rad", "pitch", "theta_rad", "theta"],
    "yaw": ["yaw_rad", "yaw", "psi_rad", "psi"],

    # --- Setpoints de actitud (rad) --- (aun no se registran; se dejan cableados)
    "roll_sp": ["roll_sp_rad", "roll_setpoint_rad", "roll_sp", "phi_sp_rad"],
    "pitch_sp": ["pitch_sp_rad", "pitch_setpoint_rad", "dtheta_sp_rad", "theta_sp_rad"],
    "yaw_sp": ["yaw_sp_rad", "yaw_setpoint_rad", "psi_sp_rad"],

    # --- Rapidez angular (rad/s) ---
    "gyro_x": ["gyro_x_radps", "gyro_x", "gyro_p", "imu_gyro_x", "gyr_x", "p_radps"],
    "gyro_y": ["gyro_y_radps", "gyro_y", "gyro_q", "imu_gyro_y", "gyr_y", "q_radps"],
    "gyro_z": ["gyro_z_radps", "gyro_z", "gyro_r", "imu_gyro_z", "gyr_z", "r_radps"],
    "pitch_rate": ["dq_radps", "dq", "q_radps", "pitch_rate_radps"],

    # --- Vibracion / acelerometros (m/s^2) ---
    "acc_x": ["acc_x_mps2", "accel_x", "acc_x", "imu_accel_x", "ax_mps2"],
    "acc_y": ["acc_y_mps2", "accel_y", "acc_y", "imu_accel_y", "ay_mps2"],
    "acc_z": ["acc_z_mps2", "accel_z", "acc_z", "imu_accel_z", "az_mps2"],

    # --- Extras utiles (no usados por el MVP, pero el parser los resuelve) ---
    "airspeed": ["pitot_ms", "airspeed_ms", "ARSP", "pitot_airspeed_ms"],
    "gps_alt": ["gps_alt_m", "gps_alt", "alt_m"],
    "laser_alt": ["laser_alt_m", "laser_range_m", "range_m", "laser_range"],

    # --- Estados longitudinales SIL (m/s) ---
    "du": ["du_mps"],
    "dw": ["dw_mps"],

    # --- Mando del piloto (normalizado) ---
    "pilot_roll": ["pilot_roll"],
    "pilot_pitch": ["pilot_pitch"],
    "pilot_yaw": ["pilot_yaw"],
    "pilot_throttle": ["pilot_throttle"],

    # --- Comandos crudos de radiocontrol (us) ---
    "elev_cmd": ["elev_cmd_us"],
    "ail_cmd": ["ail_cmd_us"],
    "rud_cmd": ["rud_cmd_us"],
    "thro_cmd": ["thro_cmd_us"],

    # --- Canales RC ---
    "rc_pitch": ["rc_pitch"],
    "rc_roll": ["rc_roll"],
    "rc_yaw": ["rc_yaw"],
    "rc_throttle": ["rc_throttle"],
    "rc_flaps": ["rc_flaps"],

    # --- Salida a actuadores (us) ---
    "u_elev": ["u_elev_us"],
    "u_ail": ["u_ail_us"],
    "u_rud": ["u_rud_us"],
    "u_thro": ["u_thro_us"],

    # --- Superficies/throttle resultantes ---
    "y_elev": ["y_elev_deg"],
    "y_ail": ["y_ail_deg"],
    "y_rud": ["y_rud_deg"],
    "y_thro": ["y_thro_norm"],

    # --- GPS: posicion y velocidad ---
    "gps_lat": ["gps_lat_deg"],
    "gps_lon": ["gps_lon_deg"],
    "gps_vn": ["gps_vn_ms"],
    "gps_ve": ["gps_ve_ms"],
    "gps_vd": ["gps_vd_ms"],
}

# ---------------------------------------------------------------------------
# Modos de vuelo: codigo entero -> (nombre corto, color de fondo).
# Alineado con enum FM_Mode de src/flight_management.h.
# ---------------------------------------------------------------------------
FLIGHT_MODES: dict[int, tuple[str, str]] = {
    0: ("MANUAL", "#9e9e9e"),
    1: ("GYRO_STAB", "#4caf50"),
    2: ("ATTITUDE", "#2196f3"),
    3: ("FLY_BY_WIRE", "#3f51b5"),
    4: ("ALT_HOLD", "#ff9800"),
    5: ("ARSP_HOLD", "#795548"),
    6: ("FAILSAFE", "#f44336"),
}

# Paleta ciclica para codigos de modo no listados en FLIGHT_MODES.
UNKNOWN_MODE_COLORS: list[str] = [
    "#8e24aa", "#00acc1", "#c0ca33", "#d81b60", "#5e35b1", "#43a047",
]


def flight_mode_info(code: int) -> tuple[str, str]:
    """Devuelve (nombre, color) para un codigo de modo, con fallback estable."""
    if code in FLIGHT_MODES:
        return FLIGHT_MODES[code]
    color = UNKNOWN_MODE_COLORS[abs(int(code)) % len(UNKNOWN_MODE_COLORS)]
    return (f"MODE_{code}", color)


# ---------------------------------------------------------------------------
# Rendimiento
# ---------------------------------------------------------------------------
CACHE_DIR: str = os.path.join(
    os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")),
    "flightreview",
)

# Por encima de este numero de muestras se grafica una version decimada.
MAX_PLOT_POINTS: int = 200_000

# Umbral de |aceleracion| considerado "normal" para la grafica de vibracion.
VIBRATION_OK_MPS2: float = 3.0
VIBRATION_WARN_MPS2: float = 6.0
