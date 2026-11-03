#!/usr/bin/env python3
"""Lee imu_noise_model_params.json y escribe imu_params.h (tablas const para imu_sensor.h)."""
import sys, json
src = sys.argv[1] if len(sys.argv) > 1 else "imu_noise_model_params.json"
dst = sys.argv[2] if len(sys.argv) > 2 else "imu_params.h"
d = json.load(open(src)); CH = ["GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"]
o = ["/* GENERADO por imu_export_c.py desde %s — no editar a mano. Canales: GyrX GyrY GyrZ AccX AccY AccZ */" % src,
     "#ifndef IMU_PARAMS_H\n#define IMU_PARAMS_H",
     "typedef struct { double sigma_w, sigma_c, tau_c; } ImuStaticParam;   /* tau_c = 0 -> sin componente OU */",
     "typedef struct { double f0_hz, Q, sigma; } ImuRes;",
     "typedef struct { double sigma_flat; ImuRes res[2]; } ImuVibParam;",
     "static const ImuStaticParam IMU_STATIC[2][6] = {  /* [0]=floor  [1]=idle */"]
for name in ("floor", "idle"):
    rows = []
    for c in CH:
        p = d["static"][name][c]
        rows.append("  {%.9g, %.9g, %.9g}" % (p["sigma_w"], p["sigma_c"], p["tau_c"] or 0.0))
    o.append(" {  /* %s */\n%s\n }," % (name, ",\n".join(rows)))
o.append("};")
o.append("static const ImuVibParam IMU_VIB[6] = {")
for c in CH:
    v = d["vibration"]["channels"][c]; r = v["resonators"]
    o.append("  {%.9g, {{%.9g, %.9g, %.9g}, {%.9g, %.9g, %.9g}}}," % (v["sigma_flat"], r[0]["f0_hz"], r[0]["Q"], r[0]["sigma"], r[1]["f0_hz"], r[1]["Q"], r[1]["sigma"]))
o.append("};")
e = d["vibration"]["envelope"]
o.append("#define IMU_ENV_S_LOG %.9g\n#define IMU_ENV_TAU_S %.9g" % (e["s_log"], e["tau_s"]))
gb = d["gyro_bias_sigma_across_runs"]
o.append("static const double IMU_BIAS_SIGMA[6] = {%.9g, %.9g, %.9g, 0.05, 0.05, 0.05}; /* acel: SUPUESTO, no identificable */" % (gb["GyrX"], gb["GyrY"], gb["GyrZ"]))
o.append("#endif")
open(dst, "w").write("\n".join(o) + "\n"); print("escrito", dst)
