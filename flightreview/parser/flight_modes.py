# -*- coding: utf-8 -*-
"""Extraccion de tramos de modo de vuelo a partir de la columna ``mode``.

Codifica por longitud de racha (run-length) la senal escalonada del modo y
devuelve intervalos contiguos ``[t0, t1)`` con nombre y color, listos para
pintar el fondo de las graficas.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

from flightreview.config import flight_mode_info


@dataclass(frozen=True)
class ModeInterval:
    t0: float
    t1: float
    code: int
    name: str
    color: str


def _intervals_from_series(
    t: np.ndarray, codes: np.ndarray, info=flight_mode_info
) -> list[ModeInterval]:
    n = len(t)
    if n == 0:
        return []
    # Indices donde cambia el codigo -> fronteras de tramo.
    change = np.flatnonzero(np.diff(codes) != 0) + 1
    starts = np.concatenate(([0], change))
    ends = np.concatenate((change, [n]))

    out: list[ModeInterval] = []
    for s, e in zip(starts, ends):
        code = int(codes[s])
        name, color = info(code)
        # t1 = inicio de la siguiente muestra si existe (tramo semiabierto),
        # si es el ultimo tramo se extiende hasta la ultima marca temporal.
        t1 = float(t[e]) if e < n else float(t[-1])
        out.append(ModeInterval(float(t[s]), t1, code, name, color))
    return out


def mode_intervals(
    df: pd.DataFrame, t_col: str, mode_col: str | None
) -> list[ModeInterval]:
    """Tramos de modo de vuelo. Lista vacia si no hay columna de modo."""
    if mode_col is None or mode_col not in df.columns:
        return []
    t = np.asarray(df[t_col], dtype=float)
    codes = np.asarray(pd.to_numeric(df[mode_col], errors="coerce").fillna(0)).astype(int)
    return _intervals_from_series(t, codes)


def arm_intervals(
    df: pd.DataFrame, t_col: str, arm_col: str | None
) -> list[ModeInterval]:
    """Tramos armado/desarmado (mismo formato). Solo se calcula en el MVP."""
    if arm_col is None or arm_col not in df.columns:
        return []
    t = np.asarray(df[t_col], dtype=float)
    codes = (
        np.asarray(pd.to_numeric(df[arm_col], errors="coerce").fillna(0)) > 0.5
    ).astype(int)

    def _arm_info(code: int) -> tuple[str, str]:
        return ("ARMED", "#e53935") if code else ("DISARMED", "#bdbdbd")

    return _intervals_from_series(t, codes, info=_arm_info)
