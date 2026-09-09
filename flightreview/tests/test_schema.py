# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import pytest

from flightreview.parser.schema import (
    ColumnMap,
    accel_magnitude,
    find_column,
    require_time_column,
    resolve_columns,
    to_deg,
)


def test_resolve_columns_mapea_canonicos(synthetic_df):
    cmap = resolve_columns(synthetic_df)
    assert cmap.t == "t_s"
    assert cmap.pitch == "dtheta_rad"
    assert cmap.pitch_rate == "dq_radps"
    assert cmap.gyro_y == "gyro_y_radps"
    assert cmap.acc_z == "acc_z_mps2"
    assert cmap.mode == "mode"
    assert cmap.arm == "arm"


def test_resolve_columns_faltantes_son_none(synthetic_df):
    cmap = resolve_columns(synthetic_df)
    # El simulador es longitudinal: no hay roll ni yaw ni setpoints.
    assert cmap.roll is None
    assert cmap.yaw is None
    assert cmap.pitch_sp is None
    assert cmap.has("pitch", "gyro_y") is True
    assert cmap.has("pitch", "roll") is False


def test_require_time_column_error_claro():
    df = pd.DataFrame({"foo": [1, 2, 3]})
    with pytest.raises(ValueError, match="columna de tiempo"):
        require_time_column(df)


def test_find_column_primer_match():
    df = pd.DataFrame({"b": [1], "c": [2]})
    assert find_column(df, ["a", "b", "c"]) == "b"
    assert find_column(df, ["x", "y"]) is None


def test_columnmap_getitem_y_present():
    cmap = ColumnMap(resolved={"t": "t_s", "pitch": None})
    assert cmap["t"] == "t_s"
    assert cmap["pitch"] is None
    assert cmap.present() == {"t": "t_s"}


def test_conversiones_unidad():
    assert to_deg(np.pi) == pytest.approx(180.0)
    assert accel_magnitude([3.0], [4.0], [0.0])[0] == pytest.approx(5.0)
