# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd

from flightreview.parser.flight_modes import arm_intervals, mode_intervals


def _df(t, mode=None, arm=None):
    d = {"t_s": np.asarray(t, dtype=float)}
    if mode is not None:
        d["mode"] = np.asarray(mode, dtype=float)
    if arm is not None:
        d["arm"] = np.asarray(arm, dtype=float)
    return pd.DataFrame(d)


def test_rle_tres_tramos():
    # codigos [0,0,1,1,1,0] en t=0..5 -> 3 tramos
    df = _df([0, 1, 2, 3, 4, 5], mode=[0, 0, 1, 1, 1, 0])
    iv = mode_intervals(df, "t_s", "mode")
    assert [x.code for x in iv] == [0, 1, 0]
    assert [x.name for x in iv] == ["MANUAL", "GYRO_STAB", "MANUAL"]
    # fronteras: primer tramo [0,2), segundo [2,5), tercero [5,5]
    assert (iv[0].t0, iv[0].t1) == (0.0, 2.0)
    assert (iv[1].t0, iv[1].t1) == (2.0, 5.0)
    assert iv[2].t0 == 5.0


def test_sin_columna_modo_lista_vacia():
    df = _df([0, 1, 2])
    assert mode_intervals(df, "t_s", None) == []
    assert mode_intervals(df, "t_s", "mode") == []


def test_un_solo_modo_un_tramo():
    df = _df([0, 1, 2, 3], mode=[0, 0, 0, 0])
    iv = mode_intervals(df, "t_s", "mode")
    assert len(iv) == 1
    assert iv[0].code == 0 and iv[0].t0 == 0.0


def test_codigo_desconocido_fallback():
    df = _df([0, 1], mode=[42, 42])
    iv = mode_intervals(df, "t_s", "mode")
    assert iv[0].name == "MODE_42"
    assert iv[0].color.startswith("#")


def test_arm_intervals():
    df = _df([0, 1, 2, 3], arm=[0, 0, 1, 1])
    iv = arm_intervals(df, "t_s", "arm")
    assert [x.name for x in iv] == ["DISARMED", "ARMED"]


def test_synthetic_df_modos(synthetic_df):
    iv = mode_intervals(synthetic_df, "t_s", "mode")
    assert [x.name for x in iv] == ["MANUAL", "GYRO_STAB", "MANUAL"]
