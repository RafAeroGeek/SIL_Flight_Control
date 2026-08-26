# -*- coding: utf-8 -*-
"""
Visualización extendida de simulación longitudinal / sensores desde CSV

Compatible con el CSV generado desde el logger en C con columnas esperadas:
    t_s,
    elev_cmd_us, ail_cmd_us, rud_cmd_us, thro_cmd_us,
    rc_pitch, rc_roll, rc_yaw, rc_throttle, rc_flaps,
    y_elev_deg, y_ail_deg, y_rud_deg, y_thro_perc,
    u_elev_us, u_ail_us, u_rud_us, u_thro_us,
    pilot_roll, pilot_pitch, pilot_yaw, pilot_throttle,
    mode, arm,
    du_mps, dw_mps, dq_radps, dtheta_rad,
    pitot_ms,
    gyro_x_radps, gyro_y_radps, gyro_z_radps,
    acc_x_mps2, acc_y_mps2, acc_z_mps2,
    gps_lat_deg, gps_lon_deg, gps_alt_m,
    gps_vn_ms, gps_ve_ms, gps_vd_ms,
    laser_alt_m

También intenta adaptarse a nombres alternativos.

Autor base: Rafael Trujillo Torres
Versión extendida: ChatGPT
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots


# ============================================================
# CONFIGURACIÓN
# ============================================================
import os
_BASE = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(_BASE, "..", "data", "SIL_sim_servos2.csv")
SHOW_MATPLOTLIB = True
SHOW_PLOTLY = True
SAVE_PLOTLY_HTML = True
PLOTLY_HTML_FILE = os.path.join(_BASE, "..", "data", "sim_plots_interactivos.html")


# ============================================================
# HELPERS
# ============================================================
def find_column(df, candidates):
    """Regresa la primera columna existente dentro de candidates."""
    for c in candidates:
        if c in df.columns:
            return c
    return None


def require_time_column(df):
    t_col = find_column(df, ["t_s", "time_s", "t", "time"])
    if t_col is None:
        raise ValueError("No se encontró columna de tiempo. Se esperaba: t_s, time_s, t o time")
    return t_col


def print_basic_info(df):
    print("=" * 80)
    print("INFORMACIÓN DEL DATASET")
    print("=" * 80)
    print("Archivo cargado correctamente")
    print(f"Dimensiones: {df.shape[0]} filas x {df.shape[1]} columnas")
    print("\nColumnas disponibles:")
    for c in df.columns:
        print(f"  - {c}")


def describe_series(name, s):
    s = pd.to_numeric(s, errors="coerce").dropna()
    if len(s) == 0:
        print(f"\n{name}\n  Sin datos válidos")
        return
    print(f"\n{name}")
    print(f"  Máximo : {s.max():.6f}")
    print(f"  Mínimo : {s.min():.6f}")
    print(f"  Media  : {s.mean():.6f}")
    print(f"  Std    : {s.std():.6f}")


def normalize_signal(x):
    x = np.asarray(x)
    xmax = np.max(np.abs(x))
    if xmax < 1e-12:
        return x
    return x / xmax


def add_zero_line(ax):
    ax.axhline(0.0, linestyle="--", linewidth=0.8, alpha=0.5)


def plot_if_exists(ax, df, t, col, ylabel, title, label=None, zero_line=False):
    if col is not None and col in df.columns:
        ax.plot(t, df[col], linewidth=1.4, label=(label or col))
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()
        if zero_line:
            add_zero_line(ax)
    else:
        ax.text(0.5, 0.5, f"{title}\n(no disponible)", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)


# ============================================================
# MAPEO FLEXIBLE DE COLUMNAS
# ============================================================
def build_column_map(df):
    cmap = {}

    # Tiempo
    cmap["t"] = require_time_column(df)

    # Comandos PWM directos
    cmap["elev_cmd_us"] = find_column(df, ["elev_cmd_us", "elev_cmd", "cmd_elev_us"])
    cmap["ail_cmd_us"] = find_column(df, ["ail_cmd_us", "aile_cmd_us", "aile_cmd", "ail_cmd"])
    cmap["rud_cmd_us"] = find_column(df, ["rud_cmd_us", "rud_cmd"])
    cmap["thro_cmd_us"] = find_column(df, ["thro_cmd_us", "thro_cmd", "thr_cmd_us"])

    # RC simulada
    cmap["rc_pitch"] = find_column(df, ["rc_pitch"])
    cmap["rc_roll"] = find_column(df, ["rc_roll"])
    cmap["rc_yaw"] = find_column(df, ["rc_yaw"])
    cmap["rc_throttle"] = find_column(df, ["rc_throttle"])
    cmap["rc_flaps"] = find_column(df, ["rc_flaps"])

    # Actuadores / superficies
    cmap["y_elev_deg"] = find_column(df, ["y_elev_deg", "y_elev", "elevator_deg"])
    cmap["y_ail_deg"] = find_column(df, ["y_ail_deg", "y_ail", "aileron_deg"])
    cmap["y_rud_deg"] = find_column(df, ["y_rud_deg", "y_rud", "rudder_deg"])
    cmap["y_thro_perc"] = find_column(df, ["y_thro_perc", "y_thro", "throttle_perc"])

    cmap["u_elev_us"] = find_column(df, ["u_elev_us", "u_elev"])
    cmap["u_ail_us"] = find_column(df, ["u_ail_us", "u_ail"])
    cmap["u_rud_us"] = find_column(df, ["u_rud_us", "u_rud"])
    cmap["u_thro_us"] = find_column(df, ["u_thro_us", "u_thro"])

    # Comandos del piloto
    cmap["pilot_roll"] = find_column(df, ["pilot_roll"])
    cmap["pilot_pitch"] = find_column(df, ["pilot_pitch"])
    cmap["pilot_yaw"] = find_column(df, ["pilot_yaw"])
    cmap["pilot_throttle"] = find_column(df, ["pilot_throttle"])
    cmap["mode"] = find_column(df, ["mode", "pilot_mode", "mode_switch"])
    cmap["arm"] = find_column(df, ["arm", "arm_switch"])

    # Estados longitudinales
    cmap["du"] = find_column(df, ["du", "du_mps"])
    cmap["dw"] = find_column(df, ["dw", "dw_mps"])
    cmap["dq"] = find_column(df, ["dq", "dq_radps"])
    cmap["dtheta"] = find_column(df, ["dtheta", "dtheta_rad"])
    cmap["ARSP"] = find_column(df, ["ARSP", "pitot_ms", "airspeed_ms", "pitot_airspeed_ms"])

    # IMU giroscopio
    cmap["gyro_x"] = find_column(df, ["gyro_x_radps", "gyro_x", "gyro_p", "imu_gyro_x", "gyr_x"])
    cmap["gyro_y"] = find_column(df, ["gyro_y_radps", "gyro_y", "gyro_q", "imu_gyro_y", "gyr_y"])
    cmap["gyro_z"] = find_column(df, ["gyro_z_radps", "gyro_z", "gyro_r", "imu_gyro_z", "gyr_z"])

    # IMU acelerómetros
    cmap["accel_x"] = find_column(df, ["acc_x_mps2", "accel_x", "acc_x", "imu_accel_x"])
    cmap["accel_y"] = find_column(df, ["acc_y_mps2", "accel_y", "acc_y", "imu_accel_y"])
    cmap["accel_z"] = find_column(df, ["acc_z_mps2", "accel_z", "acc_z", "imu_accel_z"])

    # GPS
    cmap["gps_lat"] = find_column(df, ["gps_lat_deg", "gps_lat", "lat_deg"])
    cmap["gps_lon"] = find_column(df, ["gps_lon_deg", "gps_lon", "lon_deg"])
    cmap["gps_alt"] = find_column(df, ["gps_alt_m", "gps_alt", "alt_m"])

    cmap["gps_vn"] = find_column(df, ["gps_vn_ms", "gps_vn", "vn_ms"])
    cmap["gps_ve"] = find_column(df, ["gps_ve_ms", "gps_ve", "ve_ms"])
    cmap["gps_vd"] = find_column(df, ["gps_vd_ms", "gps_vd", "vd_ms"])

    # Láser
    cmap["laser_range"] = find_column(df, ["laser_alt_m", "laser_range", "laser", "range_m", "laser_range_m"])

    return cmap


# ============================================================
# CARGA DE DATOS
# ============================================================
df = pd.read_csv(CSV_FILE)
print_basic_info(df)
cols = build_column_map(df)
t = df[cols["t"]]

# Columnas internas normalizadas / conversiones
if cols["dq"] is not None:
    df["dq_plot_radps"] = df[cols["dq"]]
    df["dq_plot_degps"] = np.rad2deg(df[cols["dq"]])

if cols["dtheta"] is not None:
    df["dtheta_plot_rad"] = df[cols["dtheta"]]
    df["dtheta_plot_deg"] = np.rad2deg(df[cols["dtheta"]])

print("\n" + "=" * 80)
print("ESTADÍSTICAS PRINCIPALES")
print("=" * 80)
for key, label in [
    ("du", "du [m/s]"),
    ("dw", "dw [m/s]"),
    ("ARSP", "ARSP [m/s]"),
    ("gyro_x", "gyro_x [rad/s]"),
    ("gyro_y", "gyro_y [rad/s]"),
    ("gyro_z", "gyro_z [rad/s]"),
    ("accel_x", "accel_x [m/s²]"),
    ("accel_y", "accel_y [m/s²]"),
    ("accel_z", "accel_z [m/s²]"),
    ("gps_alt", "gps_alt [m]"),
    ("laser_range", "laser_range [m]"),
]:
    if cols[key] is not None:
        describe_series(label, df[cols[key]])

if "dq_plot_degps" in df.columns:
    describe_series("dq [deg/s]", df["dq_plot_degps"])
if "dtheta_plot_deg" in df.columns:
    describe_series("dtheta [deg]", df["dtheta_plot_deg"])


# ============================================================
# FIGURA 1: ESTADOS LONGITUDINALES
# ============================================================
if SHOW_MATPLOTLIB:
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Estados longitudinales de la aeronave", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["du"], "du [m/s]", "Velocidad axial perturbada", zero_line=True)
    plot_if_exists(axes[1], df, t, cols["dw"], "dw [m/s]", "Velocidad vertical perturbada", zero_line=True)
    plot_if_exists(axes[2], df, t, "dq_plot_degps" if "dq_plot_degps" in df.columns else None,
                   "dq [deg/s]", "Velocidad angular de pitch", zero_line=True)
    plot_if_exists(axes[3], df, t, "dtheta_plot_deg" if "dtheta_plot_deg" in df.columns else None,
                   "dtheta [deg]", "Ángulo de pitch perturbado", zero_line=True)

    axes[3].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()


# ============================================================
# FIGURA 2: COMANDOS PWM ENTRADA
# ============================================================
if SHOW_MATPLOTLIB:
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Comandos PWM de entrada", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["elev_cmd_us"], "[us]", "Elevator command")
    plot_if_exists(axes[1], df, t, cols["ail_cmd_us"], "[us]", "Aileron command")
    plot_if_exists(axes[2], df, t, cols["rud_cmd_us"], "[us]", "Rudder command")
    plot_if_exists(axes[3], df, t, cols["thro_cmd_us"], "[us]", "Throttle command")

    axes[3].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()


# ============================================================
# FIGURA 3: ENTRADAS RC + PILOTO
# ============================================================
if SHOW_MATPLOTLIB:
    fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    fig.suptitle("Entradas RC simuladas y canales auxiliares", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["rc_pitch"], "norm", "RC pitch")
    plot_if_exists(axes[1], df, t, cols["rc_roll"], "norm", "RC roll")
    plot_if_exists(axes[2], df, t, cols["rc_yaw"], "norm", "RC yaw")
    plot_if_exists(axes[3], df, t, cols["rc_throttle"], "norm", "RC throttle")
    plot_if_exists(axes[4], df, t, cols["rc_flaps"], "norm", "RC flaps")

    axes[4].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()

    # Comandos piloto + modo/arm en una figura separada
    fig, axes = plt.subplots(6, 1, figsize=(12, 12), sharex=True)
    fig.suptitle("Comandos del piloto / lógica de modo", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["pilot_roll"], "norm", "Pilot roll", zero_line=True)
    plot_if_exists(axes[1], df, t, cols["pilot_pitch"], "norm", "Pilot pitch", zero_line=True)
    plot_if_exists(axes[2], df, t, cols["pilot_yaw"], "norm", "Pilot yaw", zero_line=True)
    plot_if_exists(axes[3], df, t, cols["pilot_throttle"], "norm", "Pilot throttle")
    plot_if_exists(axes[4], df, t, cols["mode"], "mode", "Flight mode")
    plot_if_exists(axes[5], df, t, cols["arm"], "arm", "Arm switch")

    axes[5].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()


# ============================================================
# FIGURA 4: ACTUADORES / SUPERFICIES + SEÑALES U
# ============================================================
if SHOW_MATPLOTLIB:
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Salidas de actuadores / superficies de control", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["y_elev_deg"], "[deg]", "Elevator")
    plot_if_exists(axes[1], df, t, cols["y_ail_deg"], "[deg]", "Aileron")
    plot_if_exists(axes[2], df, t, cols["y_rud_deg"], "[deg]", "Rudder")
    plot_if_exists(axes[3], df, t, cols["y_thro_perc"], "[% o frac]", "Throttle actuator")

    axes[3].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Señales internas u hacia servos / actuadores", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["u_elev_us"], "[us]", "u_elev")
    plot_if_exists(axes[1], df, t, cols["u_ail_us"], "[us]", "u_ail")
    plot_if_exists(axes[2], df, t, cols["u_rud_us"], "[us]", "u_rud")
    plot_if_exists(axes[3], df, t, cols["u_thro_us"], "[us]", "u_thro")

    axes[3].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()


# ============================================================
# FIGURA 5: COMPARACIÓN ELEVATOR cmd -> u -> y
# ============================================================
if SHOW_MATPLOTLIB:
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Cadena del canal elevator", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["elev_cmd_us"], "[us]", "elev_cmd_us")
    plot_if_exists(axes[1], df, t, cols["u_elev_us"], "[us]", "u_elev_us")
    plot_if_exists(axes[2], df, t, cols["y_elev_deg"], "[deg]", "y_elev_deg")

    axes[2].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()


# ============================================================
# FIGURA 6: AIRSPEED + GIROS + ACELERÓMETROS
# ============================================================
if SHOW_MATPLOTLIB:
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Pitot + IMU giroscopios", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["ARSP"], "[m/s]", "Airspeed (pitot)")
    plot_if_exists(axes[1], df, t, cols["gyro_x"], "[rad/s]", "Gyro X", zero_line=True)
    plot_if_exists(axes[2], df, t, cols["gyro_y"], "[rad/s]", "Gyro Y", zero_line=True)
    plot_if_exists(axes[3], df, t, cols["gyro_z"], "[rad/s]", "Gyro Z", zero_line=True)

    axes[3].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("IMU acelerómetros", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["accel_x"], "[m/s²]", "Accel X", zero_line=True)
    plot_if_exists(axes[1], df, t, cols["accel_y"], "[m/s²]", "Accel Y", zero_line=True)
    plot_if_exists(axes[2], df, t, cols["accel_z"], "[m/s²]", "Accel Z", zero_line=True)

    axes[2].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()


# ============================================================
# FIGURA 7: GPS + LÁSER
# ============================================================
if SHOW_MATPLOTLIB:
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle("GPS posición / altitud", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["gps_lat"], "[deg]", "GPS latitude")
    plot_if_exists(axes[1], df, t, cols["gps_lon"], "[deg]", "GPS longitude")
    plot_if_exists(axes[2], df, t, cols["gps_alt"], "[m]", "GPS altitude")

    axes[2].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("GPS velocidades + láser", fontsize=15, fontweight="bold")

    plot_if_exists(axes[0], df, t, cols["gps_vn"], "[m/s]", "GPS Vn")
    plot_if_exists(axes[1], df, t, cols["gps_ve"], "[m/s]", "GPS Ve")
    plot_if_exists(axes[2], df, t, cols["gps_vd"], "[m/s]", "GPS Vd")
    plot_if_exists(axes[3], df, t, cols["laser_range"], "[m]", "Laser range")

    axes[3].set_xlabel("Tiempo [s]")
    plt.tight_layout()
    plt.show()


# ============================================================
# FIGURA 8: ESTADOS NORMALIZADOS
# ============================================================
if SHOW_MATPLOTLIB:
    fig2, ax2 = plt.subplots(figsize=(12, 6))
    ax2.set_title("Comparación de estados longitudinales (normalizados)")
    ax2.set_xlabel("Tiempo [s]")
    ax2.set_ylabel("Amplitud normalizada")
    ax2.grid(True, alpha=0.3)

    if cols["du"] is not None:
        ax2.plot(t, normalize_signal(df[cols["du"]]), linewidth=1.5, label="du / max|du|")

    if cols["dw"] is not None:
        ax2.plot(t, normalize_signal(df[cols["dw"]]), linewidth=1.5, label="dw / max|dw|")

    if "dq_plot_degps" in df.columns:
        ax2.plot(t, normalize_signal(df["dq_plot_degps"]), linewidth=1.5, label="dq / max|dq|")

    if "dtheta_plot_deg" in df.columns:
        ax2.plot(t, normalize_signal(df["dtheta_plot_deg"]), linewidth=1.5, label="dtheta / max|dtheta|")

    ax2.legend()
    plt.tight_layout()
    plt.show()


# ============================================================
# FIGURA 9: RETRATO DE FASE dq vs dtheta
# ============================================================
if SHOW_MATPLOTLIB and "dq_plot_degps" in df.columns and "dtheta_plot_deg" in df.columns:
    fig3, ax3 = plt.subplots(figsize=(7, 6))
    ax3.plot(df["dtheta_plot_deg"], df["dq_plot_degps"], linewidth=1.2)
    ax3.set_title("Retrato de fase: dq vs dtheta")
    ax3.set_xlabel("dtheta [deg]")
    ax3.set_ylabel("dq [deg/s]")
    ax3.grid(True, alpha=0.3)
    ax3.axhline(0.0, linestyle="--", linewidth=0.8, alpha=0.5)
    ax3.axvline(0.0, linestyle="--", linewidth=0.8, alpha=0.5)
    plt.tight_layout()
    plt.show()


# ============================================================
# PLOTLY INTERACTIVO
# ============================================================
if SHOW_PLOTLY:
    fig_int = make_subplots(
        rows=6, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.03,
        subplot_titles=(
            "Estados longitudinales",
            "Actuadores / superficies",
            "Airspeed + giroscopios",
            "Acelerómetros",
            "GPS velocidades",
            "Modo / arm / láser"
        )
    )

    # Row 1
    if cols["du"] is not None:
        fig_int.add_trace(go.Scatter(x=t, y=df[cols["du"]], mode="lines", name="du [m/s]"), row=1, col=1)
    if cols["dw"] is not None:
        fig_int.add_trace(go.Scatter(x=t, y=df[cols["dw"]], mode="lines", name="dw [m/s]"), row=1, col=1)
    if "dq_plot_degps" in df.columns:
        fig_int.add_trace(go.Scatter(x=t, y=df["dq_plot_degps"], mode="lines", name="dq [deg/s]"), row=1, col=1)
    if "dtheta_plot_deg" in df.columns:
        fig_int.add_trace(go.Scatter(x=t, y=df["dtheta_plot_deg"], mode="lines", name="dtheta [deg]"), row=1, col=1)

    # Row 2
    for key, name in [("y_elev_deg", "y_elev [deg]"), ("y_ail_deg", "y_ail [deg]"),
                      ("y_rud_deg", "y_rud [deg]"), ("y_thro_perc", "y_thro")]:
        if cols[key] is not None:
            fig_int.add_trace(go.Scatter(x=t, y=df[cols[key]], mode="lines", name=name), row=2, col=1)

    # Row 3
    for key, name in [("ARSP", "ARSP [m/s]"), ("gyro_x", "gyro_x [rad/s]"),
                      ("gyro_y", "gyro_y [rad/s]"), ("gyro_z", "gyro_z [rad/s]")]:
        if cols[key] is not None:
            fig_int.add_trace(go.Scatter(x=t, y=df[cols[key]], mode="lines", name=name), row=3, col=1)

    # Row 4
    for key, name in [("accel_x", "acc_x [m/s²]"), ("accel_y", "acc_y [m/s²]"), ("accel_z", "acc_z [m/s²]")]:
        if cols[key] is not None:
            fig_int.add_trace(go.Scatter(x=t, y=df[cols[key]], mode="lines", name=name), row=4, col=1)

    # Row 5
    for key, name in [("gps_vn", "gps_vn [m/s]"), ("gps_ve", "gps_ve [m/s]"), ("gps_vd", "gps_vd [m/s]")]:
        if cols[key] is not None:
            fig_int.add_trace(go.Scatter(x=t, y=df[cols[key]], mode="lines", name=name), row=5, col=1)

    # Row 6
    for key, name in [("mode", "mode"), ("arm", "arm"), ("laser_range", "laser [m]")]:
        if cols[key] is not None:
            fig_int.add_trace(go.Scatter(x=t, y=df[cols[key]], mode="lines", name=name), row=6, col=1)

    fig_int.update_xaxes(title_text="Tiempo [s]", row=6, col=1)
    fig_int.update_layout(
        title="Simulación longitudinal + sensores - Visualización interactiva",
        height=1200,
        showlegend=True
    )

    if SAVE_PLOTLY_HTML:
        fig_int.write_html(PLOTLY_HTML_FILE)
        print(f"\nHTML interactivo guardado en: {os.path.abspath(PLOTLY_HTML_FILE)}")

    fig_int.show()


print("\n" + "=" * 80)
print("FIN DEL SCRIPT")
print("=" * 80)
