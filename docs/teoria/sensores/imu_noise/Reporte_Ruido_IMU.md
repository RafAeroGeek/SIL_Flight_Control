# Reporte de ruido de IMU (giroscopios + acelerómetros) para el SIL a 200 Hz

**Proyecto:** Simulacion sensores · **Fuente:** `sensors_data.csv` (vuelo real, IMU, ~50.075 Hz) · **Destino:** SIL en C (RK4 de N estados, parámetros en cJSON)
**Alcance:** solo giroscopios y acelerómetros. El log **no es** el mismo vuelo que `Airspeed_M1.csv` (confirmado por el usuario); no comparten reloj, así que nada de aquí se puede cruzar con el Pitot.

Etiquetas: **[M]** medido en el log · **[I]** inferido (razonable, no comprobado) · **[S]** supuesto mío, cámbialo si sabes algo mejor.

---

## 0. Resumen para quien implemente

1. Hay un modelo discreto a **dt = 1/200 s** listo para usar: `imu_sensor.h` + `imu_params.h` (apéndices A y B). Salida = verdad + sesgo + ruido; **no** modela escala ni desalineación (no son identificables con este log).
2. El ruido tiene **tres regímenes** [M]: *floor* (reposo silencioso), *idle* (reposo con más ruido, probablemente motor en marcha [I]) y *vuelo* = floor + **vibración de 5-24.5 Hz** con una envolvente lenta compartida por los 6 canales.
3. La vibración domina en vuelo: el RMS por encima de 5 Hz es de 0.02 a 0.08 rad/s en giros y de 0.5 a 1.2 m/s² en acelerómetros, entre 40 y 130 veces el σ del piso (tabla 8.3 frente a la tabla 5.2). Si el SIL solo mete el ruido de reposo, la IMU simulada saldrá demasiado limpia.
4. **El log es de 50 Hz y el SIL correrá a 200 Hz.** Todo lo que está entre 25 y 100 Hz es invisible en el log. El modelo asume una hipótesis explícita (sección 6) y deja una perilla (`white_scale`) para la hipótesis alternativa. Esta es la mayor incertidumbre del modelo.
5. Dos advertencias de uso: (a) el contenido de vuelo por debajo de ~5 Hz del log es movimiento real de la aeronave y turbulencia, **no se añade como ruido** (el SIL ya lo simula); (b) no apiles este modelo sobre turbulencia/vibración que el SIL ya genere sin revisar el doble conteo.

---

## 1. Datos y segmentación

| Concepto | Valor |
|---|---|
| Filas / duración | 53 556 filas, 1096.1 s [M] |
| Tasa del log | mediana de dt = 20 ms (≈50.075 Hz); percentiles 5-95 % de dt = 16.9-24.0 ms (jitter de ~±4 ms) [M] |
| Huecos | un hueco de 25 s al inicio (t = 14.8 s) [M] |
| Columnas | `TimeUS` (µs, reloj del autopiloto), `GyrX/Y/Z` (rad/s), `AccX/Y/Z` (m/s²) [M] |
| Resolución | ~53 500 valores distintos por canal: no se ve cuantización; el sensor o el autopiloto entregan valores ya filtrados/convertidos en coma flotante [M] |
| Convención | `AccZ ≈ -9.8` en reposo (Z hacia abajo, el acelerómetro mide fuerza específica) [M] |

Clasificación en bloques de 5 s (reposo si σ_giro < 0.01 rad/s y σ_acel < 0.1 m/s²): 53 bloques en reposo (265 s) y 160 dinámicos (800 s). Tramos en reposo continuos ≥ 30 s: **t = 45 s (50 s), t = 370 s (45 s), t = 1005 s (35 s)**. Tramos dinámicos ≥ 60 s: t = 215 s (155 s) y t = 440 s (564 s) [M].

---

## 2. Corrección de lo dicho antes (importante)

En el primer análisis del IMU (script `imu_noise_analysis.py`) reporté un ruido en reposo mucho mayor que el real. **Esas cifras estaban contaminadas por transitorios** al inicio y al final de cada tramo en reposo: durante los primeros ~6 s la desviación estándar en ventanas de 2 s es de 6 a 13 veces la del régimen estable (la aeronave se asienta), y al final del tramo vuelve a subir cuando empieza el movimiento.

| Canal | σ que reporté antes | σ corregida (piso, sin transitorios) | Factor |
|---|---|---|---|
| GyrX | 2.7e-3 | 1.09e-3 | 2.5× |
| GyrY | 1.1e-3 | 4.6e-4 | 2.4× |
| GyrZ | 2.1e-3 | 5.1e-4 | 4.1× |
| AccX | 2.3e-2 | 6.8e-3 | 3.4× |
| AccY | 3.2e-2 | 2.0e-2 | 1.6× |
| AccZ | 1.7e-2 | 7.7e-3 | 2.2× |

Además, el «reposo» no era un solo régimen: el tramo de t = 370 s es sistemáticamente más ruidoso que los otros dos (sección 3.2). Usa `imu_noise_model.py` y el JSON `imu_noise_model_params.json`; **no** uses las cifras estáticas de `imu_noise_params.json`. Los ratios «dinámico/reposo» que di antes (12× a 66×) usaban ese reposo contaminado como referencia y subestiman el cociente real frente al piso.

---

## 3. Qué muestran los datos

### 3.1 Transitorios en los bordes de reposo [M]
Se descartan ventanas de 1 s cuyo σ (en cualquier canal) supere 2× la mediana del perfil (2.5× en *idle*) y se conservan solo trozos contiguos ≥ 4 s. Quedan **54.9 s** para *floor* (33 s en t = 52 s y 22 s en t = 1010 s) y **38.9 s** para *idle*. Poco material: las estimaciones de baja frecuencia (< 0.45 Hz) y de deriva a más de ~4 s **no son fiables**.

### 3.2 Dos niveles de reposo [M], motor encendido o no [I]
El tramo de t = 370 s tiene σ entre 1.3 y 4.3 veces la de los otros dos tramos (más en GyrX, GyrZ y AccX; poco en GyrY y AccY) y más contenido de 15-25 Hz, especialmente en AccZ. Lo más probable es que ahí el motor estuviera encendido en tierra (el log no trae RPM ni estado del motor, así que **no lo puedo comprobar**). Se modelan como dos perfiles: `floor` (tramos de t = 45 s y t = 1005 s) e `idle` (t = 370 s). El perfil *idle* tiene un solo tramo y su ajuste es peor (sección 8.2).

### 3.3 Forma del ruido en reposo [M]
- GyrY, AccX, AccY, AccZ: **blanco** (PSD plana entre 0.45 y 24.5 Hz; curtosis ≈ 0 → gaussiano).
- GyrX y GyrZ: blanco **más una componente correlacionada corta** (Ornstein-Uhlenbeck con τ ≈ 0.05 s, esquina ~3 Hz). El ajuste de GyrX es el peor del perfil *floor* (error de ~0.36 dex en PSD); probablemente su forma real es más parecida a 1/f que a un OU simple.
- No se ven ráfagas ni colas pesadas una vez recortados los transitorios (curtosis < 0.9 en *floor*).

### 3.4 Correlación entre canales [M]
En reposo, GyrX y GyrZ tienen correlación ρ ≈ 0.5 (tramos de t = 370 y 1005 s; 0.26 en el de t = 45 s). El resto de pares están por debajo de |0.25|. **El modelo supone canales independientes**; la correlación GyrX-GyrZ queda sin modelar. En vuelo, las señales de banda 5-24 Hz entre canales tienen |ρ| ≤ 0.35 (casi independientes en fase).

### 3.5 Sesgos [M] y su límite [I]
| Tramo (inicio) | GyrX | GyrY | GyrZ | AccX | AccY | AccZ | ‖acel‖ |
|---|---|---|---|---|---|---|---|
| floor_0 (t=45 s) | -0.00073 | +0.00040 | -0.00016 | -0.38321 | -0.14863 | -9.81855 | 9.827 |
| floor_1 (t=1005 s) | +0.00029 | -0.00018 | -0.00015 | -2.68093 | -0.10028 | -9.54033 | 9.910 |
| idle_0 (t=370 s) | -0.00112 | +0.00065 | +0.00001 | -0.38562 | -0.16408 | -9.81362 | 9.823 |

- Giros: la dispersión del sesgo entre los tres tramos (σ, n = 3) es **0.73 / 0.42 / 0.10 mrad/s** (X/Y/Z). Es del orden del σ del piso. Muestra pequeña: es un orden de magnitud, no una estadística.
- Acelerómetros: el sesgo **no es identificable** (no se conoce la actitud exacta ni la gravedad local). La norma del vector medido, 9.82-9.91 m/s², supera 9.80665 por 0.016, 0.016 y 0.104 m/s²; eso es una cota gruesa de sesgo+escala+gravedad local, no una medición. El `0.05 m/s²` por eje que uso es un **[S]**.
- Deriva del sesgo: el ADEV a τ = 4 s del piso da una **cota superior** del random walk del sesgo de giro: 8.0e-05 / 3.2e-05 / 6.0e-05 rad/s/√s (X/Y/Z). No se resuelve la inestabilidad de sesgo: el mínimo de la ADEV está en el τ más largo disponible. El modelo trae `bias_rw_k = 0` por defecto.

### 3.6 Vibración en vuelo [M]
- Banda observable: 5-24.5 Hz (Nyquist 25 Hz). El contenido de 0-5 Hz es movimiento real (la PSD de AccZ tiene un pico de hasta 2.2 m/s² RMS por debajo de 2.5 Hz) y **se excluye**.
- Forma del espectro de exceso (vuelo menos piso): una joroba ancha en ~11-12 Hz y otra más estrecha en ~17-19 Hz, ambas presentes en casi todos los canales. Se ajustan con **dos resonadores AR(2) + un piso plano** por canal (tabla 5.4); el error de ajuste es de 0.05-0.15 dex.
- **Envolvente no estacionaria compartida:** el log-RMS de la banda en ventanas de 2 s tiene σ = 0.84 y las seis envolventes están correlacionadas entre sí a 0.85-0.96 [M]. Se modela con **una sola** envolvente log-normal AR(1) para todos los canales (probablemente refleja potencia del motor/maniobra [I]).
- Frecuencia dominante en ventanas de 4 s (14-24.5 Hz): mediana 16-19 Hz, P10-P90 de 14.5 a 23.8 Hz en AccY. Es decir, el pico **se mueve**; con RPM del motor se podría atar la frecuencia a las revoluciones, sin RPM se deja como resonador de banda ancha.
- **Aliasing [I]:** con 50 Hz de muestreo, una vibración real a 68 Hz aparecería en 18 Hz. No puedo saber si los picos de 17-19 Hz son fundamentales o alias de armónicos mayores. Si tienes la RPM del motor y el número de palas, compara.

---

## 4. Modelo

Por canal *j* y paso *n* (dt = 0.005 s):

```
y[n] = u[n] + bias_scale·(b0 + b_rw[n]) + noise_scale·( white_scale·σ_w·N(0,1) + ou[n] + flight·vib_scale·m[n]·v[n] )

ou[n]  = φ_c·ou[n-1] + σ_c·sqrt(1-φ_c²)·N(0,1),     φ_c = exp(-dt/τ_c)          (varianza σ_c²)
v[n]   = σ_flat·N(0,1) + Σ_{k=1,2} x_k[n]
x_k[n] = a1_k·x_k[n-1] + a2_k·x_k[n-2] + g_k·N(0,1)
         r = exp(-π·(f0/Q)·dt),  θ = 2π·f0·dt,  a1 = 2r·cosθ,  a2 = -r²
         g = σ_k·sqrt( (1+a2)·((1-a2)² - a1²) / (1-a2) )                          (varianza σ_k²)
m[n]   = exp(s·g_m[n] - s²),  g_m = AR(1) estándar con φ_m = exp(-dt/τ_m)          (E[m²] = 1)
b0     ~ N(0, σ_b²) por canal, fijo en cada inicialización;  b_rw: paseo aleatorio (opcional)
```

`u` es la verdad del SIL: velocidades angulares (p, q, r) y **fuerza específica** en ejes cuerpo.

**Ajuste.** Los parámetros del reposo se ajustan a la PSD de Welch (ventana 2.5 s) en 22 bandas logarítmicas entre 0.45 y 24.5 Hz, más un término que obliga a la varianza total a coincidir con la medida (porque la PSD no cubre < 0.45 Hz). El OU solo se acepta si mejora el ajuste más de 20 %. La vibración se ajusta a (PSD de vuelo − PSD del piso) en 5-24.5 Hz, con 6 inicializaciones y resonadores limitados a f₀ ∈ [8, 24.5] Hz, Q ∈ [0.7, 15]. La envolvente toma σ = std(log RMS) en ventanas de 2 s y busca τ_m que mejor reproduce el σ y la autocorrelación lag-1 medidos con ventanas de 2, 4 y 8 s.

**Plegado (aliasing).** El log es 1 de cada 4 muestras de un proceso a 200 Hz. La PSD que se ve a 50 Hz es la del modelo de 200 Hz **plegada**: `S50(f) = 2·Σ_{k=0..3} S2(f + 50k)`. El ajuste se hace sobre esa PSD plegada, no sobre la PSD continua.

---

## 5. Parámetros

### 5.1 Bloque de parámetros (copiable)

```yaml
# Unidades: giro rad/s, acel m/s^2. Perfil 'floor' = reposo silencioso; 'idle' = reposo con ruido mayor.
imu_noise_model:
  dt_sil_s: 0.005
  channels_order: [GyrX, GyrY, GyrZ, AccX, AccY, AccZ]
  static:
    floor:
      GyrX: {sigma_w: 6.2460e-04, sigma_c: 7.3985e-04, tau_c_s: 0.0595}
      GyrY: {sigma_w: 4.5370e-04, sigma_c: 0.0000e+00, tau_c_s: 0.0000}
      GyrZ: {sigma_w: 3.0197e-04, sigma_c: 3.7113e-04, tau_c_s: 0.0489}
      AccX: {sigma_w: 6.5934e-03, sigma_c: 0.0000e+00, tau_c_s: 0.0000}
      AccY: {sigma_w: 1.7864e-02, sigma_c: 0.0000e+00, tau_c_s: 0.0000}
      AccZ: {sigma_w: 7.0478e-03, sigma_c: 0.0000e+00, tau_c_s: 0.0000}
    idle:
      GyrX: {sigma_w: 5.0064e-04, sigma_c: 2.1495e-03, tau_c_s: 0.1986}
      GyrY: {sigma_w: 4.5468e-04, sigma_c: 3.5683e-04, tau_c_s: 0.0379}
      GyrZ: {sigma_w: 2.0617e-09, sigma_c: 2.0605e-03, tau_c_s: 0.3847}
      AccX: {sigma_w: 7.5696e-07, sigma_c: 2.1232e-02, tau_c_s: 0.0561}
      AccY: {sigma_w: 2.4019e-02, sigma_c: 0.0000e+00, tau_c_s: 0.0000}
      AccZ: {sigma_w: 1.0279e-02, sigma_c: 0.0000e+00, tau_c_s: 0.0000}
  vibration:   # solo en vuelo; se multiplica por la envolvente m(t) y se SUMA al perfil floor
    envelope: {s_log: 0.8369, tau_s: 14.0}   # m = exp(s*g - s^2), E[m^2]=1
    GyrX: {sigma_flat: 3.3164e-03, res: [{f0_hz: 11.40, Q: 1.92, sigma: 6.1165e-02}, {f0_hz: 18.23, Q: 7.83, sigma: 5.8375e-02}]}
    GyrY: {sigma_flat: 2.3957e-07, res: [{f0_hz: 15.69, Q: 0.99, sigma: 3.0156e-02}, {f0_hz: 16.74, Q: 7.78, sigma: 1.9140e-02}]}
    GyrZ: {sigma_flat: 1.5030e-02, res: [{f0_hz: 12.15, Q: 2.43, sigma: 8.3776e-03}, {f0_hz: 18.81, Q: 4.60, sigma: 1.5132e-02}]}
    AccX: {sigma_flat: 4.1194e-01, res: [{f0_hz: 11.73, Q: 15.00, sigma: 9.6551e-02}, {f0_hz: 18.08, Q: 6.09, sigma: 3.5985e-01}]}
    AccY: {sigma_flat: 4.4472e-01, res: [{f0_hz: 11.94, Q: 1.49, sigma: 6.6745e-01}, {f0_hz: 18.48, Q: 9.21, sigma: 8.5517e-01}]}
    AccZ: {sigma_flat: 8.2488e-01, res: [{f0_hz: 11.01, Q: 1.64, sigma: 9.5788e-01}, {f0_hz: 17.50, Q: 4.83, sigma: 3.9466e-01}]}
  bias: {gyro_sigma_rad_s: [7.298e-04, 4.245e-04, 9.776e-05], accel_sigma_m_s2: 0.05}   # accel = SUPUESTO
  bias_random_walk: 0.0   # no identificable; cotas superiores en la tabla de la sección 5.3
```

### 5.2 Perfil *floor* (reposo silencioso; de 54.9 s de datos)

| Canal | σ_w (blanco) | σ_c (OU) | τ_c (s) | σ total modelo | σ medida | σ robusta (MAD) | curtosis |
|---|---|---|---|---|---|---|---|
| GyrX | 6.246e-04 | 7.399e-04 | 0.059 | 9.682e-04 | 1.091e-03 | 1.074e-03 | 0.33 |
| GyrY | 4.537e-04 | 0 | — | 4.537e-04 | 4.592e-04 | 4.613e-04 | -0.03 |
| GyrZ | 3.020e-04 | 3.711e-04 | 0.049 | 4.785e-04 | 5.069e-04 | 4.755e-04 | 0.83 |
| AccX | 6.593e-03 | 0 | — | 6.593e-03 | 6.829e-03 | 6.796e-03 | 0.02 |
| AccY | 1.786e-02 | 0 | — | 1.786e-02 | 1.983e-02 | 2.027e-02 | -0.16 |
| AccZ | 7.048e-03 | 0 | — | 7.048e-03 | 7.711e-03 | 7.575e-03 | 0.27 |

σ total del modelo = √(σ_w² + σ_c²). Las curtosis cerca de cero y la σ robusta (MAD) ≈ σ confirman que no hay colas pesadas.

### 5.3 Perfil *idle* (reposo con ruido mayor; 38.9 s de datos, **confianza baja**)

| Canal | σ_w (blanco) | σ_c (OU) | τ_c (s) | σ total modelo | σ medida | σ robusta (MAD) | curtosis |
|---|---|---|---|---|---|---|---|
| GyrX | 5.006e-04 | 2.149e-03 | 0.199 | 2.207e-03 | 2.472e-03 | 2.260e-03 | 0.65 |
| GyrY | 4.547e-04 | 3.568e-04 | 0.038 | 5.780e-04 | 5.838e-04 | 5.630e-04 | 0.98 |
| GyrZ | 2.062e-09 | 2.061e-03 | 0.385 | 2.061e-03 | 1.967e-03 | 1.663e-03 | 1.51 |
| AccX | 7.570e-07 | 2.123e-02 | 0.056 | 2.123e-02 | 2.223e-02 | 1.895e-02 | 0.86 |
| AccY | 2.402e-02 | 0 | — | 2.402e-02 | 2.737e-02 | 2.547e-02 | 0.40 |
| AccZ | 1.028e-02 | 0 | — | 1.028e-02 | 1.166e-02 | 1.035e-02 | 3.61 |

En GyrZ y AccX el ajuste da σ_w ≈ 0 y todo el ruido en la componente OU: significa que el ajuste no distingue bien las dos partes con tan pocos datos; usa la σ total, no la descomposición.

### 5.4 Vibración (solo en vuelo; se suma al perfil *floor*)

Envolvente compartida: **s = 0.84**, **τ_m = 14 s**.

| Canal | σ_flat | f₀₁ (Hz) | Q₁ | σ₁ | f₀₂ (Hz) | Q₂ | σ₂ | error de ajuste (dex) |
|---|---|---|---|---|---|---|---|---|
| GyrX | 3.316e-03 | 11.4 | 1.9 | 6.116e-02 | 18.2 | 7.8 | 5.838e-02 | 0.11 |
| GyrY | 2.396e-07 | 15.7 | 1.0 | 3.016e-02 | 16.7 | 7.8 | 1.914e-02 | 0.07 |
| GyrZ | 1.503e-02 | 12.2 | 2.4 | 8.378e-03 | 18.8 | 4.6 | 1.513e-02 | 0.07 |
| AccX | 4.119e-01 | 11.7 | 15.0 | 9.655e-02 | 18.1 | 6.1 | 3.598e-01 | 0.15 |
| AccY | 4.447e-01 | 11.9 | 1.5 | 6.675e-01 | 18.5 | 9.2 | 8.552e-01 | 0.10 |
| AccZ | 8.249e-01 | 11.0 | 1.6 | 9.579e-01 | 17.5 | 4.8 | 3.947e-01 | 0.05 |

(σ en rad/s para giros y m/s² para acelerómetros; es la desviación estándar de cada componente antes de multiplicar por m(t), con E[m²] = 1.)

### 5.5 Densidad de ruido del piso, tal como se ve en el log de 50 Hz

| Canal | densidad de ruido (/√Hz) | ARW o VRW (por √hr) |
|---|---|---|
| GyrX | 1.935e-04 rad/s | 0.470 °/√hr |
| GyrY | 9.067e-05 rad/s | 0.220 °/√hr |
| GyrZ | 9.562e-05 rad/s | 0.232 °/√hr |
| AccX | 1.318e-03 m/s² | 0.056 m/s/√hr |
| AccY | 3.570e-03 m/s² | 0.151 m/s/√hr |
| AccZ | 1.408e-03 m/s² | 0.060 m/s/√hr |

Solo informativa (se calcula con σ total y la tasa del log, y asume ruido blanco hasta 25 Hz; GyrX y GyrZ incluyen su parte correlacionada). Referencia para comparar con una hoja de datos.

---

## 6. De 50 Hz a 200 Hz: la hipótesis y sus consecuencias

El log solo ve hasta 25 Hz, así que el comportamiento entre 25 y 100 Hz es una **extrapolación**. El modelo adopta:

- **H2 (por defecto): el log es un submuestreo** (sin promediar) de la salida del sensor. Entonces la **varianza por muestra no depende de la tasa**: σ_w a 200 Hz = σ_w medido, y el espectro sigue plano hasta 100 Hz. `white_scale = 1.0`.
- **H1 (alternativa): el log fue promediado/filtrado antes de registrarse** y el sensor tiene densidad plana hasta 100 Hz. Entonces la σ a 200 Hz es el doble: `white_scale = 2.0` (σ₂₀₀ = σ_densidad·√(200/2) = 2·σ₅₀ para ruido blanco). La parte OU y la vibración no cambian.

Indicios a favor de H2 [I]: la PSD en reposo no cae hacia 25 Hz (la banda 22.5-25 Hz tiene la misma densidad que 12-22 Hz), lo que descarta un filtro anti-alias fuerte por debajo de 25 Hz; con H1 y un filtro suave tampoco se vería caída. **No hay evidencia que separe H1 de H2.** Efecto práctico: en un filtro de estimación (EKF) con R = σ², la diferencia entre H1 y H2 es un factor 4 en R para los canales blancos. Si tienes la hoja de datos del IMU (ancho de banda del filtro interno, tasa de salida) o un log a 200 Hz, úsalo para elegir.

Otras consecuencias:
- **Vibración por encima de 25 Hz:** el modelo no tiene energía de vibración arriba de ~25 Hz salvo el piso plano `σ_flat` y las colas de los resonadores. La PSD simulada a 200 Hz en AccY es 0.065 en 12-24 Hz y 0.003 en 25-50 Hz (m/s²)²/Hz. Con un motor real a >25 Hz, esto subestima la vibración que vería el SIL a 200 Hz. **[S]**
- **Cambiar dt:** `imu_init` recalcula φ, a1, a2 con el dt que le pases, pero deja σ por muestra constante (coherente con H2). Los parámetros se ajustaron para 200 Hz; para otro dt, revisa la validación.

---

## 7. Integración en el SIL (C)

Archivos: `imu_params.h` (generado, apéndice A), `imu_sensor.h` (apéndice B), `imu_test.c` (apéndice C), `imu_export_c.py` (regenera `imu_params.h` desde el JSON si vuelves a ajustar).

```c
#include "imu_sensor.h"

ImuSensor imu;
imu_init(&imu, IMU_PROFILE_FLOOR, 0.005, 42);   // dt del SIL = 1/200 s, semilla para repetir corridas

// En cada paso del SIL (200 Hz), con la verdad en ejes cuerpo:
//   u[0..2] = p, q, r  (rad/s)           u[3..5] = fuerza específica = a_cuerpo - g_cuerpo (m/s²)
//   en reposo con Z hacia abajo:  u[3..5] = {0, 0, -9.81}
double u[6] = { p, q, r, fx, fy, fz }, y[6];
imu_step(&imu, u, y, en_vuelo /*0|1*/, -1.0 /*envolvente interna*/);
```

| Perilla | Efecto | Valor por defecto |
|---|---|---|
| `profile` en `imu_init` | `IMU_PROFILE_FLOOR` o `IMU_PROFILE_IDLE` | usa FLOOR; IDLE solo para pruebas en tierra con motor |
| `flight` en `imu_step` | suma la vibración (con envolvente) | 1 en vuelo, 0 en tierra |
| `env` en `imu_step` | `<0` usa la envolvente interna; `≥0` fija m (p. ej. función de la potencia del motor con E[m²] ≈ 1 en crucero) | -1 |
| `noise_scale` | multiplica todo el ruido (0 desactiva) | 1.0 |
| `white_scale` | 2.0 = hipótesis H1 (sección 6) | 1.0 |
| `vib_scale` | escala solo la vibración | 1.0 |
| `bias_scale` | escala b0 y b_rw (0 = sin sesgo) | 1.0 |
| `bias_rw_k[j]` | coeficiente de random walk del sesgo (unidad/√s); cotas en 3.5 | 0 |

Reglas para quien integre:
1. **Perfil por fase:** `floor` con motor apagado, `floor` + `flight=1` en vuelo. No cambies a `idle` en vuelo; la vibración ya contiene lo que el motor añade.
2. **Doble conteo:** no sumes encima el ruido de turbulencia o vibración que el SIL genere por su cuenta en la banda 5-25 Hz.
3. **Un `ImuSensor` por IMU físico.** Para varias IMU, usa semillas distintas.
4. **Reproducibilidad:** el generador es xoroshiro128+ con semilla explícita; misma semilla → misma salida.
5. **Sin escala ni desalineación:** si el SIL las necesita, añádelas fuera de este modelo con valores de hoja de datos **[S]**.

---

## 8. Validación

### 8.1 Perfil *floor* (modelo de 200 Hz submuestreado a 50 Hz frente a lo medido; 20 realizaciones de 120 s)

| Canal | σ medida → modelo | PSD 12-24 Hz medida → modelo | ADEV τ=0.1 s | ADEV τ=1 s | ADEV τ=4 s |
|---|---|---|---|---|---|
| GyrX | 1.091e-03 → 9.689e-04 (-11 %) | 1.516e-08 → 2.045e-08 | 0.573 → 0.544 | 0.191 → 0.259 | 0.0925 → 0.135 |
| GyrY | 4.592e-04 → 4.539e-04 (-1 %) | 8.113e-09 → 8.302e-09 | 0.197 → 0.202 | 0.0606 → 0.0645 | 0.0371 → 0.033 |
| GyrZ | 5.069e-04 → 4.802e-04 (-5 %) | 4.811e-09 → 5.112e-09 | 0.296 → 0.272 | 0.0868 → 0.121 | 0.069 → 0.0605 |
| AccX | 6.829e-03 → 6.593e-03 (-3 %) | 1.722e-06 → 1.737e-06 | 2.98 → 2.95 | 0.857 → 0.925 | 0.395 → 0.439 |
| AccY | 1.983e-02 → 1.792e-02 (-10 %) | 1.288e-05 → 1.283e-05 | 7.43 → 7.96 | 2.34 → 2.55 | 2.57 → 1.29 |
| AccZ | 7.711e-03 → 7.044e-03 (-9 %) | 2.316e-06 → 1.986e-06 | 3.27 → 3.15 | 0.88 → 0.992 | 0.561 → 0.494 |

(ADEV en unidades ×10⁻³; rad/s para giros, m/s² para acelerómetros.)

La σ del modelo queda entre 1 % y 11 % por debajo de la medida (más en GyrX, AccY y AccZ); la PSD de 12-24 Hz coincide dentro de ~16 %. **Discrepancia conocida:** la ADEV de AccY a τ = 4 s es 2.6e-3 medida frente a 1.3e-3 del modelo; el piso medido tiene una deriva lenta que el modelo no reproduce (solo hay ~55 s de datos).

### 8.2 Perfil *idle* (**confianza baja**)

| Canal | σ medida → modelo | PSD 12-24 Hz medida → modelo | ADEV τ=0.1 s | ADEV τ=1 s | ADEV τ=4 s |
|---|---|---|---|---|---|
| GyrX | 2.472e-03 → 2.202e-03 (-11 %) | 2.432e-08 → 2.252e-08 | 1.53 → 1.08 | 0.541 → 1.13 | 0.116 → 0.639 |
| GyrY | 5.838e-04 → 5.764e-04 (-1 %) | 1.089e-08 → 9.976e-09 | 0.304 → 0.3 | 0.106 → 0.115 | 0.048 → 0.0566 |
| GyrZ | 1.967e-03 → 2.060e-03 (+5 %) | 5.137e-09 → 5.852e-09 | 1.29 → 0.794 | 0.523 → 1.27 | 0.109 → 0.825 |
| AccX | 2.223e-02 → 2.120e-02 (-5 %) | 3.528e-06 → 4.168e-06 | 18.1 → 13.4 | 2.75 → 6.7 | 0.783 → 3.59 |
| AccY | 2.737e-02 → 2.407e-02 (-12 %) | 1.877e-05 → 2.305e-05 | 10.5 → 10.7 | 2.73 → 3.35 | 0.782 → 1.72 |
| AccZ | 1.166e-02 → 1.026e-02 (-12 %) | 7.710e-06 → 4.245e-06 | 4.7 → 4.58 | 1.18 → 1.41 | 0.742 → 0.717 |

(ADEV en unidades ×10⁻³; rad/s para giros, m/s² para acelerómetros.)

La σ coincide dentro de ~12 %, pero la ADEV a τ ≥ 1 s del modelo es de 2 a 8 veces la medida en GyrX, GyrZ y AccX (los OU ajustados sobreestiman la parte lenta). No uses *idle* para estudiar deriva de actitud.

### 8.3 Vuelo (piso + vibración; 6 realizaciones de 600 s)

| Canal | RMS >5 Hz medido → modelo | PSD 5-12 Hz | PSD 12-24 Hz |
|---|---|---|---|
| GyrX | 0.0794 → 0.0819 (+3 %) | 3.171e-04 → 3.503e-04 (+10 %) | 3.695e-04 → 3.969e-04 (+7 %) |
| GyrY | 0.0266 → 0.0335 (+26 %) | 4.320e-05 → 4.511e-05 (+4 %) | 6.628e-05 → 7.112e-05 (+7 %) |
| GyrZ | 0.0205 → 0.0217 (+6 %) | 1.691e-05 → 1.791e-05 (+6 %) | 2.788e-05 → 3.013e-05 (+8 %) |
| AccX | 0.4751 → 0.5346 (+13 %) | 1.109e-02 → 1.038e-02 (-6 %) | 1.653e-02 → 1.891e-02 (+14 %) |
| AccY | 1.1688 → 1.1257 (-4 %) | 4.317e-02 → 4.658e-02 (+8 %) | 7.939e-02 → 8.397e-02 (+6 %) |
| AccZ | 1.0255 → 1.2057 (+18 %) | 1.022e-01 → 1.107e-01 (+8 %) | 5.923e-02 → 6.435e-02 (+9 %) |

Mediana de error ≈ 8 %, máximo ≈ 26 %. **Esta validación es dentro de la muestra** (los mismos ~700 s de vuelo ajustaron y validan el modelo). Además, con una envolvente log-normal de σ = 0.84 y correlación de ~14 s, la potencia media sobre 700 s tiene una incertidumbre estadística del orden de ±50 %: no esperes que otro vuelo tenga el mismo nivel de vibración.

Envolvente (ventanas de 2, 4, 8 s; promedio de los 6 canales):

| Ventana | std(log amp) medida | modelo | autocorr. lag-1 medida | modelo |
|---|---|---|---|---|
| 2 s | 0.84 | 0.79 | 0.83 | 0.88 |
| 4 s | 0.78 | 0.77 | 0.80 | 0.79 |
| 8 s | 0.74 | 0.74 | 0.72 | 0.64 |

La envolvente AR(1) reproduce el σ pero decae más rápido que el dato a ventanas largas (lag-1 a 8 s: 0.64 modelo frente a 0.72 medido): los datos tienen varias escalas de tiempo y el modelo una sola.

### 8.4 Verificación de la versión en C
La implementación C (`imu_sensor.h`) se compiló con `gcc -O2 -Wall -Wextra` sin advertencias y se corrió a 200 Hz (600 s en *floor*, 1800 s en vuelo), submuestreando 1 de cada 4 para compararla con el log:
- *floor*: σ de C = 9.7e-4 / 4.5e-4 / 4.8e-4 rad/s y 6.6e-3 / 1.8e-2 / 7.0e-3 m/s² (GyrX…AccZ); coincide con el modelo en Python y con los valores medidos dentro del mismo margen de la tabla 8.1.
- vuelo: error relativo de C frente a lo medido (RMS >5 Hz y PSD 5-12 y 12-24 Hz, 18 métricas): **mediana 7 %, máximo 21 %**.
- PSD a 200 Hz: en reposo es plana de 0 a 100 Hz (GyrY ≈ 2.0e-9 (rad/s)²/Hz en 1-25, 25-50 y 50-100 Hz), como exige H2.

---

## 9. Limitaciones y datos que faltarían

1. **Un solo log, un solo avión:** 55 s de piso, 39 s de *idle*, ~700 s de vuelo. Sin réplicas, sin temperatura.
2. **50 Hz de muestreo:** sin visión de 25-100 Hz; aliasing posible; H1 vs H2 sin resolver (sección 6).
3. **Sin RPM ni estado del motor:** *idle* vs *floor* y los picos de 17-19 Hz son inferencias. Con RPM se podría atar la frecuencia de vibración a las revoluciones y la envolvente a la potencia.
4. **Sin referencia de verdad en vuelo:** el residuo de 5-24.5 Hz contiene vibración y ruido, además de una parte pequeña de dinámica real de alta frecuencia; no se separan.
5. **Inestabilidad de sesgo, random walk y efecto de temperatura:** no identificables con tramos de 35-50 s. Para eso haría falta un registro estático de varias horas.
6. **Escala, desalineación y sensibilidad cruzada:** no hay referencia para medirlas.
7. **Canales independientes:** la correlación GyrX-GyrZ (≈ 0.5) en reposo no se modela.
8. **Sin cuantización:** el log no la muestra; si el SIL alimenta un controlador que espera enteros del sensor, habrá que añadirla desde la hoja de datos.

Siguientes datos útiles, en este orden: (1) un log del IMU a la tasa interna (≥ 200 Hz, idealmente 1 kHz) para cerrar la cuestión 25-100 Hz; (2) RPM del motor sincronizada; (3) 30-60 min de IMU inmóvil con motor apagado para sesgo y random walk.

---

## 10. Cómo reproducir

```bash
python3 imu_noise_model.py sensors_data.csv salida/      # requiere imu_noise_analysis.py en la misma carpeta
python3 imu_export_c.py salida/imu_noise_model_params.json imu_params.h
gcc -O2 -Wall -Wextra -o imu_test imu_test.c -lm
./imu_test 0 1 600 1 > vuelo.csv                          # perfil floor, vuelo, 600 s, semilla 1 -> 200 Hz, 6 columnas
```

---

## Apéndice A. `imu_params.h` (generado)

```c
/* GENERADO por imu_export_c.py desde imu_model_out/imu_noise_model_params.json — no editar a mano. Canales: GyrX GyrY GyrZ AccX AccY AccZ */
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
```

## Apéndice B. `imu_sensor.h`

```c
/* imu_sensor.h — ruido de IMU (3 giros + 3 acelerómetros) para el SIL. Ajustado a un log real de 50 Hz.
 * Modelo (por canal, un paso de dt):
 *   y = u + b0 + b_rw + noise_scale * ( n_w + n_ou + m * vib )
 * Orden de canales: GyrX GyrY GyrZ AccX AccY AccZ. Unidades: rad/s y m/s^2 (igual que el log).
 * Los parámetros viven en imu_params.h (generado por imu_export_c.py). Ver Reporte_Ruido_IMU.md. */
#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H
#include <math.h>
#include <stdint.h>
#include "imu_params.h"

#define IMU_N 6
#define IMU_PROFILE_FLOOR 0   /* en reposo, motor apagado (el más silencioso) */
#define IMU_PROFILE_IDLE  1   /* en reposo, motor en marcha (menos fiable) */

typedef struct {
    double phi_c, sig_w, sig_c, ou;                 /* OU + blanco */
    double a1[2], a2[2], g[2], x1[2], x2[2];        /* resonadores AR(2) de vibración */
    double vib_flat;
    double b0, b_rw;                                /* sesgo */
} ImuCh;

typedef struct {
    ImuCh ch[IMU_N];
    double dt, noise_scale, white_scale, vib_scale, bias_scale, bias_rw_k[IMU_N];
    double env_phi, env_g;                          /* envolvente interna AR(1) */
    uint64_t s[2]; int have_spare; double spare;
} ImuSensor;

static inline uint64_t imu_rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
static inline uint64_t imu_next(ImuSensor *p) {      /* xoroshiro128+ */
    uint64_t s0 = p->s[0], s1 = p->s[1], r = s0 + s1;
    s1 ^= s0; p->s[0] = imu_rotl(s0, 24) ^ s1 ^ (s1 << 16); p->s[1] = imu_rotl(s1, 37); return r;
}
static inline double imu_unif(ImuSensor *p) { return ((imu_next(p) >> 11) + 0.5) * (1.0 / 9007199254740992.0); }
static inline double imu_randn(ImuSensor *p) {
    if (p->have_spare) { p->have_spare = 0; return p->spare; }
    double u = imu_unif(p), v = imu_unif(p), r = sqrt(-2.0 * log(u));
    p->spare = r * sin(6.283185307179586 * v); p->have_spare = 1; return r * cos(6.283185307179586 * v);
}

/* Varianza de un AR(2) x = a1 x1 + a2 x2 + g w  ->  ganancia g para varianza sigma^2 */
static inline double imu_ar2_gain(double a1, double a2, double sigma) {
    return sigma * sqrt(((1.0 + a2) * ((1.0 - a2) * (1.0 - a2) - a1 * a1)) / (1.0 - a2));
}

static inline void imu_init(ImuSensor *p, int profile, double dt, uint64_t seed) {
    p->dt = dt; p->noise_scale = 1.0; p->white_scale = 1.0; p->vib_scale = 1.0; p->bias_scale = 1.0;
    uint64_t z = seed + 0x9E3779B97F4A7C15ULL;      /* splitmix64 para sembrar */
    for (int i = 0; i < 2; i++) { z += 0x9E3779B97F4A7C15ULL; uint64_t t = z;
        t = (t ^ (t >> 30)) * 0xBF58476D1CE4E5B9ULL; t = (t ^ (t >> 27)) * 0x94D049BB133111EBULL; p->s[i] = t ^ (t >> 31); }
    p->have_spare = 0;
    p->env_phi = exp(-dt / IMU_ENV_TAU_S);
    p->env_g = imu_randn(p);
    for (int j = 0; j < IMU_N; j++) {
        const ImuStaticParam *sp = &IMU_STATIC[profile][j]; const ImuVibParam *vp = &IMU_VIB[j]; ImuCh *c = &p->ch[j];
        c->sig_w = sp->sigma_w; c->sig_c = sp->sigma_c;
        c->phi_c = sp->tau_c > 0.0 ? exp(-dt / sp->tau_c) : 0.0;
        c->ou = sp->tau_c > 0.0 ? sp->sigma_c * imu_randn(p) : 0.0;
        c->vib_flat = vp->sigma_flat;
        for (int k = 0; k < 2; k++) {
            double r = exp(-M_PI * (vp->res[k].f0_hz / vp->res[k].Q) * dt), th = 2.0 * M_PI * vp->res[k].f0_hz * dt;
            c->a1[k] = 2.0 * r * cos(th); c->a2[k] = -r * r; c->g[k] = imu_ar2_gain(c->a1[k], c->a2[k], vp->res[k].sigma);
            c->x1[k] = c->x2[k] = 0.0;
        }
        c->b0 = IMU_BIAS_SIGMA[j] * imu_randn(p); c->b_rw = 0.0; p->bias_rw_k[j] = 0.0;
    }
}

/* Un paso. u[6] = valor verdadero (p,q,r y fuerza específica en ejes cuerpo). y[6] = salida del sensor.
 * flight: 1 suma la vibración. env: envolvente externa (>=0, p. ej. función de la potencia del motor, con E[env^2]~1 en
 * crucero) o <0 para usar la envolvente log-normal interna. */
static inline void imu_step(ImuSensor *p, const double u[IMU_N], double y[IMU_N], int flight, double env) {
    double m = 0.0;
    if (flight) {
        if (env >= 0.0) m = env;
        else {
            p->env_g = p->env_phi * p->env_g + sqrt(1.0 - p->env_phi * p->env_phi) * imu_randn(p);
            m = exp(IMU_ENV_S_LOG * p->env_g - IMU_ENV_S_LOG * IMU_ENV_S_LOG);
        }
    }
    for (int j = 0; j < IMU_N; j++) {
        ImuCh *c = &p->ch[j];
        if (c->phi_c > 0.0) c->ou = c->phi_c * c->ou + c->sig_c * sqrt(1.0 - c->phi_c * c->phi_c) * imu_randn(p);
        double n = p->white_scale * c->sig_w * imu_randn(p) + c->ou;
        if (flight) {
            double v = c->vib_flat * imu_randn(p);
            for (int k = 0; k < 2; k++) {
                double x = c->a1[k] * c->x1[k] + c->a2[k] * c->x2[k] + c->g[k] * imu_randn(p);
                c->x2[k] = c->x1[k]; c->x1[k] = x; v += x;
            }
            n += p->vib_scale * m * v;
        }
        if (p->bias_rw_k[j] > 0.0) c->b_rw += p->bias_rw_k[j] * sqrt(p->dt) * imu_randn(p);
        y[j] = u[j] + p->bias_scale * (c->b0 + c->b_rw) + p->noise_scale * n;
    }
}
#endif
```

## Apéndice C. `imu_test.c`

```c
/* Uso: ./imu_test perfil(0|1) vuelo(0|1) segundos semilla > salida.csv   (200 Hz, solo ruido: u=0, sin sesgo) */
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include "imu_sensor.h"
int main(int argc, char **argv) {
    if (argc < 5) { fprintf(stderr, "uso: %s perfil(0|1) vuelo(0|1) segundos semilla\n", argv[0]); return 1; }
    int prof = atoi(argv[1]), fl = atoi(argv[2]); double secs = atof(argv[3]); uint64_t seed = strtoull(argv[4], 0, 10);
    ImuSensor s; imu_init(&s, prof, 0.005, seed); s.bias_scale = 0.0;
    long n = (long)(secs * 200.0); double u[6] = {0}, y[6];
    for (long i = 0; i < n; i++) { imu_step(&s, u, y, fl, -1.0);
        printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", y[0], y[1], y[2], y[3], y[4], y[5]); }
    return 0;
}
```
