# -*- coding: utf-8 -*-
"""Carga de un log de vuelo desde CSV a una estructura ``FlightLog``.

Incluye:
- limpieza de nombres de columna (el logger en C deja un espacio final en
  ``laser_alt_m``);
- reduccion a ``float32`` de las columnas numericas (mitad de memoria);
- cache en parquet keyed por ruta absoluta, invalidada por mtime del CSV;
- decimado por stride para graficar cuando el log supera ``MAX_PLOT_POINTS``.
"""

from __future__ import annotations

import hashlib
import os
import warnings
from dataclasses import dataclass

import numpy as np
import pandas as pd

from flightreview.config import CACHE_DIR, MAX_PLOT_POINTS
from flightreview.parser.flight_modes import ModeInterval, mode_intervals
from flightreview.parser.schema import ColumnMap, resolve_columns


@dataclass
class FlightLog:
    """Log de vuelo cargado y estructurado."""

    df: pd.DataFrame                 # datos completos, columnas limpias, float32
    cols: ColumnMap                  # nombre canonico -> columna real
    path: str
    duration_s: float
    n_samples: int
    sample_rate_hz: float
    mode_intervals: list[ModeInterval]
    df_plot: pd.DataFrame            # version (posiblemente) decimada para graficar

    @property
    def t(self) -> pd.Series:
        return self.df[self.cols.t]


# ---------------------------------------------------------------------------
# Cache
# ---------------------------------------------------------------------------
def _cache_path(csv_path: str) -> str:
    key = hashlib.sha1(os.path.abspath(csv_path).encode("utf-8")).hexdigest()
    return os.path.join(CACHE_DIR, f"{key}.parquet")


def _read_from_cache(csv_path: str) -> pd.DataFrame | None:
    cache = _cache_path(csv_path)
    try:
        if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(csv_path):
            return pd.read_parquet(cache)
    except Exception as exc:  # parquet no disponible / corrupto
        warnings.warn(f"flightreview: cache no utilizable ({exc}); se reparsea el CSV")
    return None


def _write_to_cache(csv_path: str, df: pd.DataFrame) -> None:
    try:
        os.makedirs(CACHE_DIR, exist_ok=True)
        df.to_parquet(_cache_path(csv_path))
    except Exception as exc:  # pyarrow ausente, disco lleno, ...
        warnings.warn(f"flightreview: no se pudo escribir la cache ({exc})")


# ---------------------------------------------------------------------------
# Parseo
# ---------------------------------------------------------------------------
def _parse_csv(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df.columns = df.columns.str.strip()
    # Reducir a float32 lo que sea numerico de coma flotante.
    for c in df.columns:
        if pd.api.types.is_float_dtype(df[c]):
            df[c] = df[c].astype("float32")
    return df


def _decimate(df: pd.DataFrame, max_points: int) -> pd.DataFrame:
    if len(df) <= max_points:
        return df
    stride = int(np.ceil(len(df) / max_points))
    return df.iloc[::stride].reset_index(drop=True)


def load_log(csv_path: str, use_cache: bool = True) -> FlightLog:
    """Carga ``csv_path`` y devuelve un ``FlightLog``."""
    if not os.path.exists(csv_path):
        raise FileNotFoundError(csv_path)

    df = _read_from_cache(csv_path) if use_cache else None
    if df is None:
        df = _parse_csv(csv_path)
        if use_cache:
            _write_to_cache(csv_path, df)

    cols = resolve_columns(df)
    t = pd.to_numeric(df[cols.t], errors="coerce")
    n = int(len(df))
    duration = float(t.iloc[-1] - t.iloc[0]) if n else 0.0
    rate = float((n - 1) / duration) if (n > 1 and duration > 0) else 0.0

    return FlightLog(
        df=df,
        cols=cols,
        path=os.path.abspath(csv_path),
        duration_s=duration,
        n_samples=n,
        sample_rate_hz=rate,
        mode_intervals=mode_intervals(df, cols.t, cols.mode),
        df_plot=_decimate(df, MAX_PLOT_POINTS),
    )
