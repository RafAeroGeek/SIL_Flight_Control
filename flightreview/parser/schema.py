# -*- coding: utf-8 -*-
"""Resolucion de columnas: del CSV crudo a un mapa de nombres canonicos.

El nombre canonico (p. ej. ``pitch``, ``gyro_y``, ``acc_z``) es el que usan las
graficas; ``resolve_columns`` lo asocia al primer nombre real que exista en el
DataFrame segun ``config.FIELD_ALIASES``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from flightreview.config import FIELD_ALIASES

_TIME_CANDIDATES = FIELD_ALIASES["t"]


def find_column(df: pd.DataFrame, candidates: list[str]) -> str | None:
    """Primera columna de ``candidates`` presente en ``df`` (o None).

    Mismo criterio que ``build_column_map`` de python/plot_state_exp_v3.py.
    """
    for c in candidates:
        if c in df.columns:
            return c
    return None


def require_time_column(df: pd.DataFrame) -> str:
    """Nombre de la columna temporal; error claro si no hay ninguna."""
    t_col = find_column(df, _TIME_CANDIDATES)
    if t_col is None:
        raise ValueError(
            "No se encontro columna de tiempo. Se esperaba una de: "
            + ", ".join(_TIME_CANDIDATES)
        )
    return t_col


@dataclass
class ColumnMap:
    """Nombre canonico -> nombre real en el CSV (None si la columna no esta)."""

    resolved: dict[str, str | None] = field(default_factory=dict)

    def __getattr__(self, name: str) -> str | None:
        # Acceso comodo: cmap.pitch, cmap.gyro_y, ...
        try:
            return self.__dict__["resolved"][name]
        except KeyError as exc:  # nombre canonico inexistente en FIELD_ALIASES
            raise AttributeError(name) from exc

    def __getitem__(self, name: str) -> str | None:
        return self.resolved.get(name)

    def has(self, *names: str) -> bool:
        """True si TODAS las columnas canonicas dadas estan resueltas."""
        return all(self.resolved.get(n) is not None for n in names)

    def present(self) -> dict[str, str]:
        """Solo las entradas resueltas."""
        return {k: v for k, v in self.resolved.items() if v is not None}


def resolve_columns(df: pd.DataFrame) -> ColumnMap:
    """Construye el ColumnMap aplicando FIELD_ALIASES sobre ``df``."""
    resolved: dict[str, str | None] = {}
    for canonical, candidates in FIELD_ALIASES.items():
        resolved[canonical] = find_column(df, candidates)
    resolved["t"] = require_time_column(df)  # obligatoria
    return ColumnMap(resolved=resolved)


# ---------------------------------------------------------------------------
# Conversiones de unidad
# ---------------------------------------------------------------------------
def to_deg(series_rad: pd.Series | np.ndarray) -> np.ndarray:
    """rad -> deg."""
    return np.rad2deg(np.asarray(series_rad, dtype=float))


def accel_magnitude(x, y, z) -> np.ndarray:
    """Modulo de un vector aceleracion muestra a muestra."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    z = np.asarray(z, dtype=float)
    return np.sqrt(x * x + y * y + z * z)
