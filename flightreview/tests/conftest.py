# -*- coding: utf-8 -*-
"""Fixtures compartidas: un CSV sintetico con el header real del simulador y un
puntero opcional al CSV real de ``data/``.
"""

from __future__ import annotations

import os

import numpy as np
import pandas as pd
import pytest

# Header real del logger en C (src/main.c). Ojo: la ultima columna lleva un
# espacio final, que el loader debe limpiar.
REAL_COLUMNS = [
    "t_s", "elev_cmd_us", "ail_cmd_us", "rud_cmd_us", "thro_cmd_us",
    "rc_pitch", "rc_roll", "rc_yaw", "rc_throttle", "rc_flaps",
    "y_elev_deg", "y_ail_deg", "y_rud_deg", "y_thro_norm",
    "u_elev_us", "u_ail_us", "u_rud_us", "u_thro_us",
    "pilot_roll", "pilot_pitch", "pilot_yaw", "pilot_throttle",
    "mode", "arm",
    "du_mps", "dw_mps", "dq_radps", "dtheta_rad",
    "pitot_ms",
    "gyro_x_radps", "gyro_y_radps", "gyro_z_radps",
    "acc_x_mps2", "acc_y_mps2", "acc_z_mps2",
    "gps_lat_deg", "gps_lon_deg", "gps_alt_m",
    "gps_vn_ms", "gps_ve_ms", "gps_vd_ms",
    "laser_alt_m ",  # <-- espacio final deliberado
]

N_ROWS = 60
DT = 0.01
# Transicion de modo: 0 (filas 0-19) -> 1 (20-39) -> 0 (40-59)
MODE_PATTERN = [0] * 20 + [1] * 20 + [0] * 20


@pytest.fixture(autouse=True)
def _isolated_cache(tmp_path, monkeypatch):
    """Aisla la cache de parquet en tmp para no ensuciar ~/.cache durante los tests."""
    monkeypatch.setattr(
        "flightreview.parser.loader.CACHE_DIR", str(tmp_path / "cache"), raising=False
    )


@pytest.fixture
def synthetic_df() -> pd.DataFrame:
    t = np.arange(N_ROWS) * DT
    data = {c: np.zeros(N_ROWS, dtype=float) for c in REAL_COLUMNS}
    data["t_s"] = t
    data["mode"] = np.array(MODE_PATTERN, dtype=float)
    data["arm"] = np.where(t >= 0.10, 1.0, 0.0)
    # Senales con forma conocida para las graficas.
    data["dtheta_rad"] = 0.10 * np.sin(2 * np.pi * 1.0 * t)      # pitch
    data["dq_radps"] = 0.20 * np.cos(2 * np.pi * 1.0 * t)        # pitch rate
    data["gyro_x_radps"] = 0.01 * t
    data["gyro_y_radps"] = data["dq_radps"]
    data["gyro_z_radps"] = -0.01 * t
    data["acc_x_mps2"] = 0.5 * np.sin(2 * np.pi * 5.0 * t)
    data["acc_y_mps2"] = 0.3 * np.cos(2 * np.pi * 7.0 * t)
    data["acc_z_mps2"] = -9.81 + 0.2 * np.sin(2 * np.pi * 11.0 * t)
    return pd.DataFrame(data, columns=REAL_COLUMNS)


@pytest.fixture
def synthetic_csv(tmp_path, synthetic_df) -> str:
    p = tmp_path / "log_sintetico.csv"
    synthetic_df.to_csv(p, index=False)
    return str(p)


@pytest.fixture
def real_csv() -> str:
    """Ruta al CSV real del simulador; salta el test si no existe."""
    here = os.path.dirname(__file__)
    path = os.path.normpath(os.path.join(here, "..", "..", "data", "SIL_sim_servos2.csv"))
    if not os.path.exists(path):
        pytest.skip("data/SIL_sim_servos2.csv no presente (correr el simulador)")
    return path
