# -*- coding: utf-8 -*-
"""
Visualización de estados longitudinales desde CSV
Estados esperados:
    X = [du, dw, dq, dtheta]

Compatible con columnas:
    du, dw, dq, dtheta
o bien:
    du_mps, dw_mps, dq_radps, dtheta_rad

Autor: Rafael Trujillo Torres
"""

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


# ============================================================
# HELPERS
# ============================================================
def find_column(df, candidates):
    """Regresa la primera columna existente dentro de candidates."""
    for c in candidates:
        if c in df.columns:
            return c
    return None


def print_basic_info(df):
    print("=" * 70)
    print("INFORMACIÓN DEL DATASET")
    print("=" * 70)
    print(f"Archivo cargado correctamente")
    print(f"Dimensiones: {df.shape[0]} filas x {df.shape[1]} columnas")
    print("\nColumnas disponibles:")
    for c in df.columns:
        print(f"  - {c}")


def describe_series(name, s):
    print(f"\n{name}")
    print(f"  Máximo : {s.max():.6f}")
    print(f"  Mínimo : {s.min():.6f}")
    print(f"  Media  : {s.mean():.6f}")
    print(f"  Std    : {s.std():.6f}")


# ============================================================
# CARGA DE DATOS
# ============================================================
df = pd.read_csv(CSV_FILE)
print_basic_info(df)

# Tiempo
t_col = find_column(df, ["t_s", "time_s", "t"])
if t_col is None:
    raise ValueError("No se encontró columna de tiempo. Se esperaba: t_s, time_s o t")

# Estados en distintas convenciones de nombre
du_col     = find_column(df, ["du", "du_mps"])
dw_col     = find_column(df, ["dw", "dw_mps"])
dq_col     = find_column(df, ["dq", "dq_radps"])
dtheta_col = find_column(df, ["dtheta", "dtheta_rad"])

if not any([du_col, dw_col, dq_col, dtheta_col]):
    raise ValueError("No se encontró ninguna columna de estados longitudinales en el CSV.")

t = df[t_col]

# Crear columnas normalizadas internas
if du_col is not None:
    df["du_plot"] = df[du_col]

if dw_col is not None:
    df["dw_plot"] = df[dw_col]

if dq_col is not None:
    df["dq_plot_radps"] = df[dq_col]
    df["dq_plot_degps"] = np.rad2deg(df[dq_col])

if dtheta_col is not None:
    df["dtheta_plot_rad"] = df[dtheta_col]
    df["dtheta_plot_deg"] = np.rad2deg(df[dtheta_col])

print("\n" + "=" * 70)
print("ESTADÍSTICAS DE LOS ESTADOS")
print("=" * 70)

if "du_plot" in df.columns:
    describe_series("du [m/s]", df["du_plot"])

if "dw_plot" in df.columns:
    describe_series("dw [m/s]", df["dw_plot"])

if "dq_plot_radps" in df.columns:
    describe_series("dq [rad/s]", df["dq_plot_radps"])
    describe_series("dq [deg/s]", df["dq_plot_degps"])

if "dtheta_plot_rad" in df.columns:
    describe_series("dtheta [rad]", df["dtheta_plot_rad"])
    describe_series("dtheta [deg]", df["dtheta_plot_deg"])


# ============================================================
# FIGURA 1: ESTADOS EN 4x1
# ============================================================
fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Estados longitudinales de la aeronave", fontsize=15, fontweight="bold")

# du
if "du_plot" in df.columns:
    axes[0].plot(t, df["du_plot"], linewidth=1.5, label="du")
    axes[0].set_ylabel("du [m/s]")
    axes[0].set_title("Velocidad axial perturbada")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[0].axhline(0.0, linestyle="--", linewidth=0.8, alpha=0.5)

else:
    axes[0].text(0.5, 0.5, "du no disponible", ha="center", va="center", transform=axes[0].transAxes)
    axes[0].set_title("du no disponible")
    axes[0].grid(True, alpha=0.3)

# dw
if "dw_plot" in df.columns:
    axes[1].plot(t, df["dw_plot"], linewidth=1.5, label="dw")
    axes[1].set_ylabel("dw [m/s]")
    axes[1].set_title("Velocidad vertical perturbada")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    axes[1].axhline(0.0, linestyle="--", linewidth=0.8, alpha=0.5)

else:
    axes[1].text(0.5, 0.5, "dw no disponible", ha="center", va="center", transform=axes[1].transAxes)
    axes[1].set_title("dw no disponible")
    axes[1].grid(True, alpha=0.3)

# dq
if "dq_plot_degps" in df.columns:
    axes[2].plot(t, df["dq_plot_degps"], linewidth=1.5, label="dq")
    axes[2].set_ylabel("dq [deg/s]")
    axes[2].set_title("Velocidad angular de pitch")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()
    axes[2].axhline(0.0, linestyle="--", linewidth=0.8, alpha=0.5)

else:
    axes[2].text(0.5, 0.5, "dq no disponible", ha="center", va="center", transform=axes[2].transAxes)
    axes[2].set_title("dq no disponible")
    axes[2].grid(True, alpha=0.3)

# dtheta
if "dtheta_plot_deg" in df.columns:
    axes[3].plot(t, df["dtheta_plot_deg"], linewidth=1.5, label="dtheta")
    axes[3].set_ylabel("dtheta [deg]")
    axes[3].set_title("Ángulo de pitch perturbado")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend()
    axes[3].axhline(0.0, linestyle="--", linewidth=0.8, alpha=0.5)

else:
    axes[3].text(0.5, 0.5, "dtheta no disponible", ha="center", va="center", transform=axes[3].transAxes)
    axes[3].set_title("dtheta no disponible")
    axes[3].grid(True, alpha=0.3)

axes[3].set_xlabel("Tiempo [s]")
plt.tight_layout()
plt.show()


# ============================================================
# FIGURA 2: TODOS LOS ESTADOS EN UNA SOLA FIGURA NORMALIZADA
# ============================================================
fig2, ax2 = plt.subplots(figsize=(12, 6))
ax2.set_title("Comparación de estados longitudinales (normalizados)")
ax2.set_xlabel("Tiempo [s]")
ax2.set_ylabel("Amplitud normalizada")
ax2.grid(True, alpha=0.3)

def normalize_signal(x):
    xmax = np.max(np.abs(x))
    if xmax < 1e-12:
        return x
    return x / xmax

if "du_plot" in df.columns:
    ax2.plot(t, normalize_signal(df["du_plot"]), linewidth=1.5, label="du / max|du|")

if "dw_plot" in df.columns:
    ax2.plot(t, normalize_signal(df["dw_plot"]), linewidth=1.5, label="dw / max|dw|")

if "dq_plot_degps" in df.columns:
    ax2.plot(t, normalize_signal(df["dq_plot_degps"]), linewidth=1.5, label="dq / max|dq|")

if "dtheta_plot_deg" in df.columns:
    ax2.plot(t, normalize_signal(df["dtheta_plot_deg"]), linewidth=1.5, label="dtheta / max|dtheta|")

ax2.legend()
plt.tight_layout()
plt.show()


# ============================================================
# FIGURA 3: RETRATO DE FASE dq vs dtheta
# ============================================================
if "dq_plot_degps" in df.columns and "dtheta_plot_deg" in df.columns:
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
fig_int = make_subplots(
    rows=4, cols=1,
    shared_xaxes=True,
    vertical_spacing=0.05,
    subplot_titles=(
        "du - velocidad axial",
        "dw - velocidad vertical",
        "dq - velocidad angular de pitch",
        "dtheta - ángulo de pitch"
    )
)

if "du_plot" in df.columns:
    fig_int.add_trace(
        go.Scatter(x=t, y=df["du_plot"], mode="lines", name="du [m/s]"),
        row=1, col=1
    )

if "dw_plot" in df.columns:
    fig_int.add_trace(
        go.Scatter(x=t, y=df["dw_plot"], mode="lines", name="dw [m/s]"),
        row=2, col=1
    )

if "dq_plot_degps" in df.columns:
    fig_int.add_trace(
        go.Scatter(x=t, y=df["dq_plot_degps"], mode="lines", name="dq [deg/s]"),
        row=3, col=1
    )

if "dtheta_plot_deg" in df.columns:
    fig_int.add_trace(
        go.Scatter(x=t, y=df["dtheta_plot_deg"], mode="lines", name="dtheta [deg]"),
        row=4, col=1
    )

fig_int.update_xaxes(title_text="Tiempo [s]", row=4, col=1)
fig_int.update_yaxes(title_text="m/s", row=1, col=1)
fig_int.update_yaxes(title_text="m/s", row=2, col=1)
fig_int.update_yaxes(title_text="deg/s", row=3, col=1)
fig_int.update_yaxes(title_text="deg", row=4, col=1)

fig_int.update_layout(
    title="Estados longitudinales - Visualización interactiva",
    height=900,
    showlegend=True
)

fig_int.show()

print("\n" + "=" * 70)
print("FIN DEL SCRIPT")
print("=" * 70)
