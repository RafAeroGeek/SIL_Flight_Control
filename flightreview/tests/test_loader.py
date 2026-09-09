# -*- coding: utf-8 -*-
import os

import pytest

from flightreview.parser.loader import _cache_path, load_log


def test_carga_basica_y_limpieza(synthetic_csv):
    log = load_log(synthetic_csv)
    assert log.n_samples == 60
    assert log.df.shape[1] == 42
    # El espacio final de 'laser_alt_m ' debe quedar limpio.
    assert "laser_alt_m" in log.df.columns
    assert "laser_alt_m " not in log.df.columns
    # Columnas de coma flotante reducidas a float32.
    assert log.df["dtheta_rad"].dtype == "float32"


def test_metadatos_tiempo(synthetic_csv):
    log = load_log(synthetic_csv)
    assert log.duration_s == pytest.approx(0.59, abs=1e-4)
    # 60 muestras, 0.59 s de span -> ~100 Hz
    assert log.sample_rate_hz == pytest.approx(100.0, rel=1e-2)
    assert log.cols.pitch == "dtheta_rad"
    assert [iv.name for iv in log.mode_intervals] == ["MANUAL", "GYRO_STAB", "MANUAL"]


def test_cache_se_crea_y_se_reutiliza(synthetic_csv, monkeypatch):
    cache = _cache_path(synthetic_csv)
    if os.path.exists(cache):
        os.remove(cache)

    load_log(synthetic_csv)
    assert os.path.exists(cache), "la primera carga debe escribir el parquet"

    # La segunda carga debe leer del parquet, no reparsear el CSV.
    import flightreview.parser.loader as loader_mod

    called = {"parse": False}
    orig = loader_mod._parse_csv

    def spy(path):
        called["parse"] = True
        return orig(path)

    monkeypatch.setattr(loader_mod, "_parse_csv", spy)
    log2 = load_log(synthetic_csv)
    assert called["parse"] is False
    assert log2.n_samples == 60


def test_use_cache_false_ignora_parquet(synthetic_csv):
    load_log(synthetic_csv)  # crea cache
    import flightreview.parser.loader as loader_mod

    hits = {"n": 0}
    orig = loader_mod._parse_csv

    def spy(path):
        hits["n"] += 1
        return orig(path)

    loader_mod._parse_csv = spy
    try:
        load_log(synthetic_csv, use_cache=False)
    finally:
        loader_mod._parse_csv = orig
    assert hits["n"] == 1


def test_real_csv_si_existe(real_csv):
    log = load_log(real_csv)
    assert log.n_samples > 1000
    assert log.cols.t == "t_s"
    assert log.cols.acc_z == "acc_z_mps2"
    assert log.duration_s == pytest.approx(10.0, abs=0.01)
