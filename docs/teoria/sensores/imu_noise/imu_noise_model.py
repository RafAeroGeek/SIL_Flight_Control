#!/usr/bin/env python3
"""
Modelo de ruido de IMU (giroscopio + acelerómetro) para un SIL a 200 Hz, ajustado a un log real de ~50 Hz.

Uso:
    python3 imu_noise_model.py sensors_data.csv [carpeta_salida]

Requiere imu_noise_analysis.py en la misma carpeta (reutiliza carga y clasificación reposo/dinámico).

Modelo discreto a dt = 1/200 s, por canal:
    y = verdad + b0 + b_rw(t) + n_blanco + n_OU + m(t) * vib
  n_blanco : N(0, sigma_w^2)                       (se ajusta a la PSD del log, ver FOLDING)
  n_OU     : AR(1) con varianza sigma_c^2, constante de tiempo tau_c
  vib      : suma de K=2 resonadores AR(2) + blanco, solo en vuelo (exceso de PSD sobre el reposo)
  m(t)     : envolvente log-normal AR(1), COMPARTIDA por los 6 canales, E[m^2] = 1

FOLDING: el log está a ~50 Hz, el SIL a 200 Hz. Se asume que el log es un submuestreo (1 de cada 4) de un sensor
cuyo ruido llega hasta 100 Hz. Con esa hipótesis la varianza por muestra NO cambia con la tasa, y la PSD que se ve en el
log es la del modelo de 200 Hz plegada (aliasing) a 0-25 Hz. El ajuste se hace sobre esa PSD plegada.

Salidas: imu_noise_model_params.json, imu_noise_model_validation.png
"""
import sys, os, json
import numpy as np
from scipy import signal as sg
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_noise_analysis as A

CH, GYR, ACC = A.CH, A.GYR, A.ACC
FS_SIL = 200.0
DECIM = 4
WIN_S = 1.0           # s, ventana para detectar transitorios en reposo
GATE_FLOOR = 2.0      # ventana buena si std < GATE * mediana (todos los canales)
GATE_IDLE = 2.5
MIN_CHUNK_S = 4.0
VIB_BAND = (5.0, 24.5)
rng0 = np.random.default_rng(12345)


# ───────────────────────── datos ─────────────────────────
def gated_chunks(df, fs, runs, thr):
    """Divide cada tramo en ventanas de 1 s, descarta las que tienen algún canal > thr * mediana del perfil
    y devuelve los trozos contiguos buenos (>= MIN_CHUNK_S) como índices (i0, i1)."""
    n = int(WIN_S * fs)
    stds = {c: [] for c in CH}
    pos = []
    for ri, (a, b) in enumerate(runs):
        for i in range(a, b - n + 1, n):
            pos.append((ri, i))
            for c in CH:
                stds[c].append(df[c].values[i:i + n].std())
    S = np.array([stds[c] for c in CH])
    med = np.median(S, axis=1, keepdims=True)
    good = (S / med).max(axis=0) < thr
    chunks, cur = [], None
    for g, (ri, i) in zip(good, pos):
        if g and cur is not None and cur[0] == ri and cur[2] == i:
            cur[2] = i + n
        elif g:
            if cur is not None:
                chunks.append(cur)
            cur = [ri, i, i + n]
        else:
            if cur is not None:
                chunks.append(cur)
            cur = None
    if cur is not None:
        chunks.append(cur)
    chunks = [(i0, i1) for _, i0, i1 in chunks if (i1 - i0) / fs >= MIN_CHUNK_S]
    return chunks, float(good.mean())


def welch_chunks(df, c, chunks, fs, nper):
    Ps, w = [], []
    for a, b in chunks:
        x = df[c].values[a:b]
        f, P = sg.welch(x - x.mean(), fs=fs, nperseg=min(nper, len(x)), noverlap=min(nper, len(x)) // 2, detrend="linear")
        if len(f) == nper // 2 + 1:
            Ps.append(P); w.append(b - a)
    return f, np.average(Ps, axis=0, weights=w)


def adev_chunks(df, c, chunks, fs, taus):
    """ADEV solapada agrupada: promedia varianzas de todos los trozos ponderando por número de términos."""
    out = {}
    for tau in taus:
        m = max(1, int(round(tau * fs)))
        num, den = 0.0, 0
        for a, b in chunks:
            x = df[c].values[a:b] - df[c].values[a:b].mean()
            if 3 * m >= len(x):
                continue
            th = np.cumsum(x) / fs
            d = th[2 * m:] - 2 * th[m:-m] + th[:-2 * m]
            num += 0.5 * np.sum(d ** 2) / (m / fs) ** 2; den += len(d)
        if den:
            out[tau] = float(np.sqrt(num / den))
    return out


# ───────────────────────── PSD de los modelos (plegada) ─────────────────────────
def ar1_coef(tau_c):
    return float(np.exp(-1.0 / (FS_SIL * tau_c)))


def ar2_coef(f0, Q):
    r = np.exp(-np.pi * (f0 / Q) / FS_SIL); th = 2 * np.pi * f0 / FS_SIL
    return 2 * r * np.cos(th), -r * r


def ar2_gain(a1, a2, sigma):
    """g tal que x_n = a1 x_{n-1} + a2 x_{n-2} + g w_n tiene varianza sigma^2."""
    return sigma * np.sqrt(((1 + a2) * ((1 - a2) ** 2 - a1 ** 2)) / (1 - a2))


def fold(S2_fun, f, fs_lo):
    """PSD unilateral vista a fs_lo al submuestrear un proceso de FS_SIL: 2 * sum_k S2(f + k fs_lo)."""
    return 2 * sum(S2_fun(f + k * fs_lo) for k in range(DECIM))


def S2_white(sig):
    return lambda nu: np.full_like(nu, sig ** 2 / FS_SIL, dtype=float)


def S2_ar1(sig, tau):
    phi = ar1_coef(tau)
    return lambda nu: sig ** 2 * (1 - phi ** 2) / np.abs(1 - phi * np.exp(-2j * np.pi * nu / FS_SIL)) ** 2 / FS_SIL


def S2_ar2(sig, f0, Q):
    a1, a2 = ar2_coef(f0, Q); g = ar2_gain(a1, a2, sig)
    return lambda nu: g ** 2 / np.abs(1 - a1 * np.exp(-2j * np.pi * nu / FS_SIL) - a2 * np.exp(-4j * np.pi * nu / FS_SIL)) ** 2 / FS_SIL


def log_bins(f, P, lo, hi, nb):
    edges = np.geomspace(lo, hi, nb + 1)
    fc, pc = [], []
    for i in range(nb):
        m = (f >= edges[i]) & (f < edges[i + 1])
        if m.any():
            fc.append(np.sqrt(f[m].mean() * 0) + f[m].mean()); pc.append(P[m].mean())
    return np.array(fc), np.array(pc)


# ───────────────────────── ajuste del ruido en reposo ─────────────────────────
def fit_static(f, P, fs, std_meas):
    """Ajusta blanco (+ OU opcional) a la PSD plegada. Devuelve dict."""
    fc, pc = log_bins(f, P, 0.45, 24.5, 22)
    lt = np.log(pc)
    var_hi = float(np.median(pc[fc > 12]))                  # densidad en banda alta -> sigma_w inicial
    sw0 = np.sqrt(var_hi * fs / 2)

    def model(p, with_ou):
        sw = np.exp(p[0])
        S = fold(S2_white(sw), fc, fs)
        if with_ou:
            S = S + fold(S2_ar1(np.exp(p[1]), np.exp(p[2])), fc, fs)
        return S

    W_STD = 3.0   # peso del término que fuerza la varianza total a coincidir con la medida (incluye <0.45 Hz)

    def res(p, with_ou):
        tot = np.sqrt(np.exp(p[0]) ** 2 + (np.exp(p[1]) ** 2 if with_ou else 0.0))
        return np.append(np.log(model(p, with_ou)) - lt, W_STD * (np.log(tot) - np.log(std_meas)))

    r0 = least_squares(lambda p: res(p, False), [np.log(sw0)])
    best = dict(sigma_w=float(np.exp(r0.x[0])), sigma_c=0.0, tau_c=None, cost=float(r0.cost), with_ou=False)
    bestc = r0.cost
    for tau0 in (0.1, 0.5, 2.0):
        r = least_squares(lambda p: res(p, True), [np.log(sw0 * 0.7), np.log(sw0), np.log(tau0)],
                          bounds=([-20, -20, np.log(0.03)], [0, 0, np.log(8.0)]))
        if r.cost < bestc and r.cost < 0.8 * r0.cost:      # solo se acepta el OU si mejora >20 %
            bestc = r.cost
            best = dict(sigma_w=float(np.exp(r.x[0])), sigma_c=float(np.exp(r.x[1])), tau_c=float(np.exp(r.x[2])),
                        cost=float(r.cost), with_ou=True)
    best["rms_log10_err"] = float(np.sqrt(2 * best["cost"] / (len(fc) + 1)) / np.log(10))
    return best


# ───────────────────────── ajuste de la vibración ─────────────────────────
def fit_vib(f, Pexc, K=2):
    m = (f >= VIB_BAND[0]) & (f <= VIB_BAND[1])
    fb, pb = f[m], np.maximum(Pexc[m], 1e-18)
    # bins de 0.5 Hz
    e = np.arange(VIB_BAND[0], VIB_BAND[1] + 0.5, 0.5)
    fc = np.array([fb[(fb >= e[i]) & (fb < e[i + 1])].mean() for i in range(len(e) - 1) if ((fb >= e[i]) & (fb < e[i + 1])).any()])
    pc = np.array([pb[(fb >= e[i]) & (fb < e[i + 1])].mean() for i in range(len(e) - 1) if ((fb >= e[i]) & (fb < e[i + 1])).any()])
    lt = np.log(pc)
    tot = np.sqrt(np.trapezoid(pc, fc))
    fs_lo = FS_SIL / DECIM

    def model(p):
        S = fold(S2_white(np.exp(p[0])), fc, fs_lo)
        for k in range(K):
            f0, lq, ls = p[1 + 3 * k: 4 + 3 * k]
            S = S + fold(S2_ar2(np.exp(ls), f0, np.exp(lq)), fc, fs_lo)
        return S

    best = None
    for starts in ([9, 17], [12, 19], [10, 15], [10, 21], [14, 20], [9, 13]):
        p0 = [np.log(tot / 4)]
        lo = [-25]; hi = [3]
        for f0 in starts[:K]:
            p0 += [f0, np.log(3.0), np.log(tot / 2)]
            lo += [8.0, np.log(0.7), -25]; hi += [24.5, np.log(15.0), 3]
        r = least_squares(lambda p: np.log(model(p) + 1e-30) - lt, p0, bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r
    p = best.x
    out = dict(sigma_flat=float(np.exp(p[0])), resonators=[
        dict(f0_hz=float(p[1 + 3 * k]), Q=float(np.exp(p[2 + 3 * k])), sigma=float(np.exp(p[3 + 3 * k]))) for k in range(K)])
    out["rms_log10_err"] = float(np.sqrt(2 * best.cost / len(fc)) / np.log(10))
    out["fit_freqs"] = fc.tolist(); out["fit_meas"] = pc.tolist(); out["fit_model"] = model(p).tolist()
    return out


# ───────────────────────── simulador (numpy, mismas ecuaciones que el C) ─────────────────────────
def simulate(par, n, mode, rng, flight=False, bias_sigma=None):
    """Devuelve array (n, 6) con ruido a 200 Hz. mode: 'floor' | 'idle'. Sin verdad ni sesgo."""
    out = np.zeros((n, 6))
    prof = par["static"][mode]
    if flight:
        s_log = par["vibration"]["envelope"]["s_log"]; tau_m = par["vibration"]["envelope"]["tau_s"]
        phi = np.exp(-1 / (FS_SIL * tau_m))
        g = sg.lfilter([np.sqrt(1 - phi ** 2)], [1, -phi], rng.standard_normal(n))
        m = np.exp(s_log * g - s_log ** 2)
    for j, c in enumerate(CH):
        p = prof[c]
        y = p["sigma_w"] * rng.standard_normal(n)
        if p["sigma_c"] > 0:
            phi1 = ar1_coef(p["tau_c"])
            y += p["sigma_c"] * sg.lfilter([np.sqrt(1 - phi1 ** 2)], [1, -phi1], rng.standard_normal(n))
        if flight:
            v = par["vibration"]["channels"][c]
            vib = v["sigma_flat"] * rng.standard_normal(n)
            for r in v["resonators"]:
                a1, a2 = ar2_coef(r["f0_hz"], r["Q"]); gg = ar2_gain(a1, a2, r["sigma"])
                vib += sg.lfilter([gg], [1, -a1, -a2], rng.standard_normal(n))
            y += m * vib
        out[:, j] = y
    return out


# ───────────────────────── métricas de validación ─────────────────────────
def static_metrics(X, fs, taus):
    """X (n,6) a fs. Devuelve por canal: std, bandas de PSD y ADEV."""
    res = {}
    for j, c in enumerate(CH):
        x = X[:, j] - X[:, j].mean()
        f, P = sg.welch(x, fs=fs, nperseg=128, noverlap=64, detrend="linear")
        ad = {}
        th = np.cumsum(x) / fs
        for tau in taus:
            m = int(round(tau * fs))
            if 3 * m < len(x):
                d = th[2 * m:] - 2 * th[m:-m] + th[:-2 * m]
                ad[tau] = float(np.sqrt(0.5 * np.mean(d ** 2)) / (m / fs))
        res[c] = dict(std=float(x.std()), psd_0p5_5=A.band_mean(f, P, .5, 5), psd_5_12=A.band_mean(f, P, 5, 12),
                      psd_12_24=A.band_mean(f, P, 12, 24), adev=ad)
    return res


def envelope_stats(y, fs, W):
    n = int(W * fs)
    amp = np.array([np.sqrt(np.mean(y[i:i + n] ** 2)) for i in range(0, len(y) - n, n)])
    la = np.log(amp); la = la - la.mean()
    return float(la.std()), float(np.corrcoef(la[:-1], la[1:])[0, 1])


def flight_metrics(X, fs):
    sos_bp = sg.butter(4, [5, 24], btype="band", fs=fs, output="sos")
    res = {}
    for j, c in enumerate(CH):
        x = X[:, j]
        f, P = sg.welch(x - x.mean(), fs=fs, nperseg=512, noverlap=256, detrend="linear")
        y = sg.sosfiltfilt(sos_bp, x)
        res[c] = dict(hp5_rms=A.hp_rms(x, fs), psd_5_12=A.band_mean(f, P, 5, 12), psd_12_24=A.band_mean(f, P, 12, 24),
                      env={str(W): envelope_stats(y, fs, W) for W in (2, 4, 8)})
    return res


# ───────────────────────── principal ─────────────────────────
def main(path, outdir="."):
    os.makedirs(outdir, exist_ok=True)
    df, t = A.load(path)
    dt = np.diff(t); fs = float(1.0 / np.median(dt[dt < 0.1]))
    blocks = A.classify_blocks(df, fs)
    sruns = [r for r in A.merge_runs(blocks, True) if (r[1] - r[0]) / fs >= 30]
    druns = [r for r in A.merge_runs(blocks, False) if (r[1] - r[0]) / fs >= 60]

    # perfiles: floor = tramos 1 y 3 (reposo más silencioso), idle = tramo 2 (reposo con más ruido)
    # (la asignación se verifica comparando la std robusta de cada tramo)
    stdr = [np.mean([np.std(df[c].values[a:b]) / np.median([np.std(df[c].values[i:i + int(fs)]) for i in range(a, b - int(fs), int(fs))])
                     for c in CH]) for a, b in sruns]
    lvl = [np.mean([np.median([df[c].values[i:i + int(fs)].std() for i in range(a, b - int(fs), int(fs))]) / ref
                    for c, ref in zip(CH, [1e-3, 5e-4, 5e-4, 7e-3, 2e-2, 8e-3])]) for a, b in sruns]
    order = np.argsort(lvl)                                  # el más silencioso primero
    idle_i = int(order[-1]); floor_runs = [sruns[i] for i in range(len(sruns)) if i != idle_i]
    prof_runs = {"floor": floor_runs, "idle": [sruns[idle_i]]}
    prof_thr = {"floor": GATE_FLOOR, "idle": GATE_IDLE}
    info = dict(run_levels_rel=[float(x) for x in lvl], idle_run_index=idle_i)

    static, meas_static, meta = {"floor": {}, "idle": {}}, {}, {}
    taus = [0.1, 0.2, 0.5, 1.0, 2.0, 4.0]
    for name in ("floor", "idle"):
        chunks, frac = gated_chunks(df, fs, prof_runs[name], prof_thr[name])
        meta[name] = dict(chunks_s=[[float(t[a]), float((b - a) / fs)] for a, b in chunks], good_window_fraction=frac,
                          total_s=float(sum(b - a for a, b in chunks) / fs))
        meas_static[name] = {}
        for c in CH:
            f, P = welch_chunks(df, c, chunks, fs, 128)
            x = np.concatenate([df[c].values[a:b] - df[c].values[a:b].mean() for a, b in chunks])
            fit = fit_static(f, P, fs, float(x.std()))
            ad = adev_chunks(df, c, chunks, fs, taus)
            static[name][c] = dict(sigma_w=fit["sigma_w"], sigma_c=fit["sigma_c"], tau_c=fit["tau_c"],
                                   fit_rms_log10_err=fit["rms_log10_err"], std_measured=float(x.std()),
                                   std_robust_mad=float(1.4826 * np.median(np.abs(x - np.median(x)))),
                                   kurtosis=float(((x - x.mean()) ** 4).mean() / x.var() ** 2 - 3))
            meas_static[name][c] = dict(std=float(x.std()), adev=ad, psd_0p5_5=A.band_mean(f, P, .5, 5),
                                        psd_5_12=A.band_mean(f, P, 5, 12), psd_12_24=A.band_mean(f, P, 12, 24), f=f, P=P)

    # sesgos por tramo (media de los trozos buenos) y norma de la gravedad
    bias = {}
    for name, runs in (("floor", prof_runs["floor"]), ("idle", prof_runs["idle"])):
        for k, run in enumerate(runs):
            chunks, _ = gated_chunks(df, fs, [run], prof_thr[name])
            bias[f"{name}_{k}"] = {c: float(np.mean(np.concatenate([df[c].values[a:b] for a, b in chunks]))) for c in CH}
            bias[f"{name}_{k}"]["acc_norm"] = float(np.sqrt(sum(bias[f"{name}_{k}"][c] ** 2 for c in ACC)))
            bias[f"{name}_{k}"]["t0_s"] = float(t[run[0]])
    gyro_bias_vals = {c: [bias[k][c] for k in bias] for c in GYR}
    gyro_bias_sigma = {c: float(np.std(v, ddof=1)) for c, v in gyro_bias_vals.items()}

    # cota superior de random walk del sesgo (ADEV a tau largo del piso)
    adl = {}
    for c in GYR:
        ad_long = adev_chunks(df, c, gated_chunks(df, fs, prof_runs["floor"], GATE_FLOOR)[0], fs, [4.0])
        adl[c] = float(ad_long[4.0] * np.sqrt(3.0 / 4.0)) if ad_long else None

    # ───── vuelo ─────
    fl_chunks = druns
    Pdyn, floor_model_psd = {}, {}
    vib = {}
    for c in CH:
        f, Pd = A.welch(df[c].values[druns[0][0]:druns[0][1]], fs, 512)
        Ps, w = [], []
        for a, b in druns:
            f, P = A.welch(df[c].values[a:b], fs, 512); Ps.append(P); w.append(b - a)
        Pdyn[c] = (f, np.average(Ps, axis=0, weights=w))
        p = static["floor"][c]
        Pfl = fold(S2_white(p["sigma_w"]), f, fs / 1.0 * 1.0) if False else fold(S2_white(p["sigma_w"]), f, FS_SIL / DECIM)
        if p["sigma_c"] > 0:
            Pfl = Pfl + fold(S2_ar1(p["sigma_c"], p["tau_c"]), f, FS_SIL / DECIM)
        Pexc = np.maximum(Pdyn[c][1] - Pfl, 1e-3 * Pdyn[c][1])
        vib[c] = fit_vib(f, Pexc)

    # envolvente compartida: ventanas de 2 s, banda 5-24 Hz
    sos = sg.butter(4, [5, 24], btype="band", fs=fs, output="sos")
    env = {}
    for W in (2, 4, 8):
        sl, l1 = [], []
        for c in CH:
            amp = []
            for a, b in druns:
                y = sg.sosfiltfilt(sos, df[c].values[a:b]); n = int(W * fs)
                amp += [np.sqrt(np.mean(y[i:i + n] ** 2)) for i in range(0, len(y) - n, n)]
            la = np.log(amp); la -= la.mean()
            sl.append(la.std()); l1.append(np.corrcoef(la[:-1], la[1:])[0, 1])
        env[W] = (float(np.mean(sl)), float(np.mean(l1)))
    s_log = env[2][0]
    # tau_m: se busca el valor que mejor reproduce (std_log, lag1) medidos en ventanas de 2, 4 y 8 s
    best_tau, best_err = None, 1e9
    for tau_c_ in (6, 8, 10.7, 14, 18, 24, 32):
        phi_ = np.exp(-1.0 / (fs * tau_c_)); err = 0.0
        for rep in range(4):
            g_ = sg.lfilter([np.sqrt(1 - phi_ ** 2)], [1, -phi_], np.random.default_rng(rep).standard_normal(int(900 * fs)))
            y_ = np.exp(s_log * g_ - s_log ** 2) * np.random.default_rng(100 + rep).standard_normal(len(g_))
            for W in (2, 4, 8):
                sd_, l1_ = envelope_stats(y_, fs, W)
                err += ((sd_ - env[W][0]) / env[W][0]) ** 2 + (l1_ - env[W][1]) ** 2
        if err < best_err:
            best_tau, best_err = tau_c_, err
    tau_m = float(best_tau)
    envelope = dict(s_log=s_log, tau_s=tau_m, measured={str(W): dict(std_log=v[0], lag1=v[1]) for W, v in env.items()})

    params = dict(
        source=os.path.basename(path), fs_log_hz=fs, fs_sil_hz=FS_SIL, decimation_assumed=DECIM,
        units=dict(gyro="rad/s", accel="m/s^2"),
        profiles_info=info, profiles_meta=meta,
        static=static, bias_runs=bias, gyro_bias_sigma_across_runs=gyro_bias_sigma,
        gyro_bias_rw_upper_bound_rad_s_per_sqrt_s=adl,
        vibration=dict(channels={c: {k: v for k, v in vib[c].items() if not k.startswith("fit_")} for c in CH},
                       envelope=envelope, band_hz=VIB_BAND))

    # ───── validación ─────
    rng = np.random.default_rng(2024)
    val = {"static": {}, "flight": {}}
    for name in ("floor", "idle"):
        acc = {c: dict(std=[], psd_0p5_5=[], psd_5_12=[], psd_12_24=[], adev={str(t_): [] for t_ in taus}) for c in CH}
        for _ in range(20):
            X = simulate(params, int(120 * FS_SIL), name, rng)[::DECIM]
            M = static_metrics(X, FS_SIL / DECIM, taus)
            for c in CH:
                for k in ("std", "psd_0p5_5", "psd_5_12", "psd_12_24"):
                    acc[c][k].append(M[c][k])
                for t_ in taus:
                    if t_ in M[c]["adev"]:
                        acc[c]["adev"][str(t_)].append(M[c]["adev"][t_])
        val["static"][name] = {}
        for c in CH:
            ms = meas_static[name][c]
            val["static"][name][c] = dict(
                std=(ms["std"], float(np.mean(acc[c]["std"]))),
                psd_0p5_5=(ms["psd_0p5_5"], float(np.mean(acc[c]["psd_0p5_5"]))),
                psd_5_12=(ms["psd_5_12"], float(np.mean(acc[c]["psd_5_12"]))),
                psd_12_24=(ms["psd_12_24"], float(np.mean(acc[c]["psd_12_24"]))),
                adev={str(t_): (ms["adev"].get(t_), float(np.mean(acc[c]["adev"][str(t_)])) if acc[c]["adev"][str(t_)] else None) for t_ in taus})
    # vuelo medido
    meas_fl = {}
    for c in CH:
        f, P = Pdyn[c]
        hp = float(np.mean([A.hp_rms(df[c].values[a:b], fs) for a, b in druns]))
        envm = {}
        for W in (2, 4, 8):
            y = np.concatenate([sg.sosfiltfilt(sos, df[c].values[a:b]) for a, b in druns])
            envm[str(W)] = None
        meas_fl[c] = dict(hp5_rms=hp, psd_5_12=A.band_mean(f, P, 5, 12), psd_12_24=A.band_mean(f, P, 12, 24))
    accf = {c: dict(hp5_rms=[], psd_5_12=[], psd_12_24=[], env={str(W): [] for W in (2, 4, 8)}) for c in CH}
    for _ in range(6):
        X = simulate(params, int(600 * FS_SIL), "floor", rng, flight=True)[::DECIM]
        M = flight_metrics(X, FS_SIL / DECIM)
        for c in CH:
            for k in ("hp5_rms", "psd_5_12", "psd_12_24"):
                accf[c][k].append(M[c][k])
            for W in ("2", "4", "8"):
                accf[c]["env"][W].append(M[c]["env"][W])
    for c in CH:
        val["flight"][c] = dict(
            hp5_rms=(meas_fl[c]["hp5_rms"], float(np.mean(accf[c]["hp5_rms"]))),
            psd_5_12=(meas_fl[c]["psd_5_12"], float(np.mean(accf[c]["psd_5_12"]))),
            psd_12_24=(meas_fl[c]["psd_12_24"], float(np.mean(accf[c]["psd_12_24"]))),
            env_model={W: [float(np.mean([e[0] for e in accf[c]["env"][W]])), float(np.mean([e[1] for e in accf[c]["env"][W]]))] for W in ("2", "4", "8")})
    params["validation"] = val
    params["envelope_measured_vs_model_std_lag1"] = {
        W: dict(measured=[envelope["measured"][W]["std_log"], envelope["measured"][W]["lag1"]],
                model=[float(np.mean([val["flight"][c]["env_model"][W][0] for c in CH])),
                       float(np.mean([val["flight"][c]["env_model"][W][1] for c in CH]))]) for W in ("2", "4", "8")}

    # errores relativos resumidos
    def rel(a, b):
        return abs(b - a) / abs(a) if a else float("nan")
    summ = {}
    for name in ("floor", "idle"):
        e = []
        for c in CH:
            v = val["static"][name][c]
            e += [rel(*v["std"]), rel(*v["psd_12_24"])]
        summ[name] = dict(median_rel_err=float(np.median(e)), max_rel_err=float(np.max(e)))
    e = []
    for c in CH:
        v = val["flight"][c]
        e += [rel(*v["hp5_rms"]), rel(*v["psd_5_12"]), rel(*v["psd_12_24"])]
    summ["flight"] = dict(median_rel_err=float(np.median(e)), max_rel_err=float(np.max(e)))
    params["validation_summary"] = summ

    with open(os.path.join(outdir, "imu_noise_model_params.json"), "w") as fh:
        json.dump(params, fh, indent=2, default=lambda o: None)

    # ───── figura ─────
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 3, figsize=(16, 8.5))
        for k, c in enumerate(["GyrY", "AccZ"]):
            ms = meas_static["floor"][c]; p = static["floor"][c]; ff = ms["f"][1:]
            S = fold(S2_white(p["sigma_w"]), ff, FS_SIL / DECIM)
            if p["sigma_c"] > 0:
                S = S + fold(S2_ar1(p["sigma_c"], p["tau_c"]), ff, FS_SIL / DECIM)
            ax[0, k].loglog(ff, ms["P"][1:], label="medido (piso)"); ax[0, k].loglog(ff, S, "--", label="modelo plegado")
            ax[0, k].set(title=f"PSD en reposo {c}", xlabel="Hz"); ax[0, k].legend()
        for k, (nm, grp) in enumerate([("giroscopio", GYR), ("acelerómetro", ACC)]):
            for c in grp:
                ms = meas_static["floor"][c]["adev"]; mo = val["static"]["floor"][c]["adev"]
                ln, = ax[0, 2].loglog([], [])
            ax[0, 2].cla()
        for c in CH:
            ms = meas_static["floor"][c]["adev"]
            mo = {float(k_): v[1] for k_, v in val["static"]["floor"][c]["adev"].items() if v[1]}
            l, = ax[0, 2].loglog(list(ms), list(ms.values()), "o", ms=4, label=c)
            ax[0, 2].loglog(list(mo), list(mo.values()), "-", color=l.get_color(), lw=1)
        ax[0, 2].set(title="ADEV piso (puntos = medido, línea = modelo)", xlabel="τ (s)"); ax[0, 2].legend(ncol=2, fontsize=7)
        for k, c in enumerate(["AccY", "AccZ", "GyrX"]):
            f, P = Pdyn[c]; v = vib[c]
            ax[1, k].loglog(f[1:], P[1:], label="medido (vuelo)")
            ax[1, k].loglog(v["fit_freqs"], v["fit_model"], "--", label="piso + vibración modelo")
            ax[1, k].set(title=f"PSD en vuelo {c}", xlabel="Hz"); ax[1, k].legend()
        plt.tight_layout(); plt.savefig(os.path.join(outdir, "imu_noise_model_validation.png"), dpi=120)
    except Exception as ex:
        print("figura omitida:", ex)
    print(json.dumps(dict(profiles=info, meta=meta, static=static, gyro_bias_sigma=gyro_bias_sigma,
                          bias_rw_ub=adl, envelope=envelope, summary=summ), indent=1, default=lambda o: None))
    return params


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else ".")
