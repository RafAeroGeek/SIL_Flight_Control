/* GENERADO por imu_export_c.py desde docs/teoria/sensores/imu_noise/imu_noise_model_params.json — no editar a mano. Canales: GyrX GyrY GyrZ AccX AccY AccZ */
#ifndef IMU_PARAMS_H
#define IMU_PARAMS_H
typedef struct { double sigma_w, sigma_c, tau_c; } ImuStaticParam;   /* tau_c = 0 -> sin componente OU */
typedef struct { double f0_hz, Q, sigma; } ImuRes;
typedef struct { double sigma_flat; ImuRes res[2]; } ImuVibParam;
static const ImuStaticParam IMU_STATIC[2][6] = {  /* [0]=floor  [1]=idle */
 {  /* floor */
  {0.000624597549, 0.000739850613, 0.0594682622},
  {0.00045370013, 0, 0},
  {0.000301966962, 0.000371134435, 0.048854419},
  {0.0065933829, 0, 0},
  {0.0178635504, 0, 0},
  {0.00704776715, 0, 0}
 },
 {  /* idle */
  {0.000500636993, 0.00214946665, 0.198630108},
  {0.000454677894, 0.000356832734, 0.037936976},
  {2.06170539e-09, 0.0020605187, 0.384656436},
  {7.56957964e-07, 0.0212321466, 0.0560512529},
  {0.0240185696, 0, 0},
  {0.010278633, 0, 0}
 },
};
static const ImuVibParam IMU_VIB[6] = {
  {0.00331636372, {{11.3982436, 1.92378816, 0.0611648354}, {18.2307956, 7.83146281, 0.0583752475}}},
  {2.39569313e-07, {{15.6936636, 0.986568338, 0.0301560906}, {16.7354462, 7.78477888, 0.019139659}}},
  {0.0150300662, {{12.1516157, 2.4312711, 0.00837763469}, {18.8063675, 4.59530513, 0.0151320386}}},
  {0.411940718, {{11.7334025, 15, 0.096551207}, {18.079385, 6.09098711, 0.359846806}}},
  {0.444720167, {{11.94067, 1.49272492, 0.667452724}, {18.4782611, 9.20713726, 0.855172612}}},
  {0.824882591, {{11.0091137, 1.64422608, 0.957880762}, {17.498162, 4.83151093, 0.394661308}}},
};
#define IMU_ENV_S_LOG 0.836874237
#define IMU_ENV_TAU_S 14
static const double IMU_BIAS_SIGMA[6] = {0.000729803553, 0.000424504034, 9.77619349e-05, 0.05, 0.05, 0.05}; /* acel: SUPUESTO, no identificable */
#endif
