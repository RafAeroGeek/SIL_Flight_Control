# -*- coding: utf-8 -*-
"""
Created on Tue Mar 18 11:20:33 2025

@author: rtrujillo
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import control

input_path = "ugly_longitudinal.json"
# Cargar los valores constantes desde el archivo JSON
with open(input_path, 'r') as file:
    constantes = json.load(file)

# Asignar los valores a variables (opcional, para mayor claridad)

#Condiciones Iniciales
Alt         = constantes['Alt[m]']                  #Altitud
V0          = constantes['V_0[m/s]']                #Velocidad
gamma0      = constantes['gamma_0[deg]']            #Angulo de Trayectoria

#Parámetros Geométricos
x_cg        = constantes['x_cg[m]']                 #Posición CG desde Datum
x_ca        = constantes['x_ca[m]']                 #Posición CA desde Datum
b           = constantes['b[m]']                    #Envergadura
S           = constantes['S[m2]']                   #Superficie alar
cam         = constantes['cam[m]']                  #Cuerda media aerodinamica. 
l_ht        = constantes['l_ht[m]']                 #Longitud de CA_ala y CA_empenaje
l_f         = constantes['l_f[m]']                  #Longitud del fuselaje
S_ht        = constantes['S_ht[m2]']                #Superficie empenaje horizontal
S_e         = constantes['S_e[m2]']                 #Superficie elevador
#Datos Aerodinamicos
alpha_0     = constantes['alpha_0[deg]']        #Ángulo de ataque a cero levantamiento
CL_alpha    = constantes['CL_alpha']            #Derivada de levantamiento CL vs alpha
CD_0par     = constantes['CD_0']                #Coeficiente de arrastre parasito
k           = constantes['k']                   #Constante polar
CL_u        = constantes['CL_u'] 
CD_u        = constantes['CD_u']                #Derivada de arrastres respecto a velocidad.
CM_u        = constantes['CM_u']
CM_0        = constantes['CM_0']                #Coeficiente de momentos a 0 levantamiento.
CL_alpha_ht = constantes['CL_alpha_ht']         #Derivada de levatamiento CL vs alpha empenaje horizontal con interferencia,
CL_alpha_w  = constantes['CL_alpha_w']          #Derivada de levatamiento CL vs alpha ala con interferencia.
#Datos Inerciales
Iy          = constantes['Iy']
m           = constantes['masa[Kg]']                #masa 


rho = 1.225 * np.exp(-Alt / 8400)
PH = 101325 * ((16.6 - Alt/1000) / (16.6 + Alt/1000))
w = m * 9.81


AR      = b ** 2 / S
V_ht    = S_ht * l_ht / (S * cam)
#I_x = 0.01 * m * b ** 2
#I_z = 0.07 * m * ((b + lf) / 2) ** 2
#Iy          =   0.04 * m * l_f**2

CL_trim = w / (0.5 * rho * (V0**2) * S)
CD_trim = CD_0par + k * CL_trim ** 2

CD_alpha = 2 * k * CL_trim * CL_alpha 
# ==============================================
# Derivadas longitudinales
# ==============================================

CM_alpha    =  - CL_alpha * (x_ca - x_cg)/cam

CM_de       =  - CL_alpha_ht * V_ht * (S_e / S_ht)**0.5
CL_de       =  CL_alpha_ht * S_ht / S *(S_e / S_ht)**0.5

d_e_trim    =   -(CM_0 *CL_alpha + CM_alpha * CL_trim)/(CM_de * CL_alpha - CM_alpha*CL_de)
alpha_trim  =   (CL_trim - CL_de * d_e_trim) / CL_alpha + alpha_0 / 57.3;

u0          = V0 * np.cos(alpha_trim)

Q           =   0.5 * rho * V0**2


epsilon_alpha         = 2 * CL_alpha_w / (np.pi * AR) 

eta_ht    =   1.0
CM_q      = -2*eta_ht * (S_ht/S) * (l_ht/cam)**2 * CL_alpha_ht #-CL_alpha_ht * S_ht * l_ht**2 / (S * cam**2) * eta_ht
CM_adot   = -2 * eta_ht * V_ht * CL_alpha_ht * epsilon_alpha * (l_ht/cam)
CZ_adot   = -2 * eta_ht * V_ht * CL_alpha_ht * epsilon_alpha

'''
%===========================================================================
%----------------   CALCULO DE COEFICIENTES       ----------------
%===========================================================================
'''
X_u         =   -( CD_u + 2 * CD_trim ) * Q * S / (m * u0)
X_w         =   -( CD_alpha - CL_trim ) * Q * S / (m * u0)
Z_u         =   -( CL_u + 2 * CL_trim ) * Q * S / (m * u0)
M_u         =   Q * S * cam / (u0*Iy) * CM_u  
Z_w         =   -(CL_alpha + CD_trim) * Q * S /(u0 * m)
Z_w_dot     =   Q * S * cam * CZ_adot  / ( 2 * m * u0**2)
Z_alpha     =   Z_w * u0
M_w         =   Q* S * cam * (CM_alpha) / (Iy * u0) #Q* S * cam * (CM_alpha  - cam * CM_adot * Z_w/V0) / (Iy * u0) 
M_q         =   Q* S * cam**2 * (CM_q) / (Iy * V0 * 2) # Q* S * cam**2 * (CM_q + CM_adot) / (Iy * V0)
M_alpha     =   Q* S * cam * (CM_alpha ) / Iy
M_w_dot     =   Q* S * cam**2 * (CM_adot) / ( Iy * u0**2) #2
M_alpha_dot =   u0 * M_w_dot

M_de        =   Q* S * cam * (CM_de) / Iy
Z_de        =   -CL_de * Q *S/m

omega_sp = (M_q * Z_w - M_alpha)**0.5
zeta_sp = - (M_q + Z_w + M_alpha_dot)/(2 * omega_sp)

T_theta_2 = -u0 / Z_alpha
n_a = -Z_alpha/(9.81)
CAP = omega_sp**2 / n_a

omega_sp_hertz = omega_sp/(2*np.pi)
periodo_sp  = 1/omega_sp_hertz 

zeta_ph     = (1/2**0.5) * (CD_trim/CL_trim)
omega_ph    = 9.81 * 2**0.5 / V0

#Eigenvalores

from sympy import I, symbols, pretty

# Definir un número complejo simbólico
z_1 = round(-(zeta_sp * omega_sp),4) + round((omega_sp * (1-zeta_sp**2)**0.5),4)*I
z_2 = round(-(zeta_sp * omega_sp),4) - round((omega_sp * (1-zeta_sp**2)**0.5),4)*I
print()

print("##############################################################")
print("######## *******  R E S U L T A D O S  *****  ################")
print("##############################################################")
print("**************  L O N G I T U D I N A L  *********************")
print("Trim:")
print("delta_e_trim",d_e_trim * 57.3)
print("alpha_trim",alpha_trim * 57.3)
print("*****************  Periodo Corto  *********************")
print("omega_sp",omega_sp)
print("zeta_sp",zeta_sp)
print("omega_n_Hertz:", omega_sp_hertz)
print("T:", 1 / omega_sp_hertz, "s")
print('n_a',n_a)
print("T_\u03B8 2  (Incidence lag):",T_theta_2 )
print('CAP (control Anticioation Parameter:', CAP)
print('Eigenvalores :')
print('λ 1 : ', pretty(z_1))
print('λ 2 : ', pretty(z_2))
print("*****************  P H U G O I D E   *********************")
print("*Periodo Largo")
print("omega_ph",omega_ph)
print("zeta_ph",zeta_ph)
print("omega_ph_Hertz:", omega_ph / (2*np.pi) )
print("T_ph:", (2*np.pi) / omega_ph, "s")
print("##########################################################")




#Definimos el rango del tiempo para el plot de la simulación
T0 = 0.0
Tf = round(1 / omega_sp_hertz * 4 , 1)
ts = round(periodo_sp / 25,3) 
N = int((Tf - T0)/ts)

Delta_alpha_0 = 10 #Condiciones iniciales simulación

T=[]
Alpha=[]
N_Z = []
for n in range(N+1):
    T.append(n * ts)
    t = n * ts
    #Obtener la respuesta de la aeronave.
    adt = Delta_alpha_0 * (np.exp(-(zeta_sp * omega_sp) * t)) * ((zeta_sp/(1-zeta_sp**0.5)**0.5) * np.sin((omega_sp *(1-zeta_sp**0.5)**0.5)* t )+np.cos((omega_sp *(1-zeta_sp**0.5)**0.5)* t ))
    Alpha.append(adt)
    N_Z.append(adt*n_a)
    

# Crear la figura y el gráfico
plt.close('all')
fig = plt.figure(num=1, figsize=(15, 8))
ax1 = plt.subplot(1, 1, 1)
ax1.plot(T, Alpha, '.-', linewidth=1.0, label='$\Delta \\alpha (t)$')

# Añadir etiquetas a los ejes
ax1.set_xlabel("Tiempo (s)", fontsize=12)  # Etiqueta del eje X
ax1.set_ylabel("$\Delta \\alpha (t)$", fontsize=12)  # Etiqueta del eje Y

# Añadir leyenda, cuadrícula y mostrar el gráfico
ax1.legend()
ax1.minorticks_on()
ax1.grid(True, which='major', linestyle='-', linewidth='0.5', color='gray')
ax1.grid(True, which='minor', linestyle=':', linewidth='0.5', color='slategray')
plt.show()

'''
%===========================================================================
%----------------   STEP       ----------------
%===========================================================================
'''
# 1. Definir correctamente la variable 's' como variable de Laplace
s = control.TransferFunction.s

# 2. Crear la función de transferencia correctamente
numerator = [M_de*n_a]  # (M_de/T_theta_2)*s + M_de
denominator = [1, 2*zeta_sp*omega_sp, omega_sp**2]  # s² + 2*zeta_sp*omega_sp*s + omega_sp²
sys_sp = control.TransferFunction(numerator, denominator)

numerator_ph = [omega_ph**2]  
denominator_ph = [1, 2*zeta_ph*omega_ph, omega_ph**2]
sys_ph = control.TransferFunction(numerator_ph,denominator_ph)

# 3. Calcular la respuesta escalón
T, yout = control.step_response(sys_sp)

# 4. Calcular la respuesta en estado estacionario
SS_response = M_de * n_a/ (omega_sp**2)

# 5. Graficar la respuesta normalizada


# Configuración del tiempo
t = np.linspace(0, T[-1], 1000)  # De 0 a 12 segundos
tp = np.pi/(omega_sp * (1-zeta_sp**2)**0.5)

# Definición de las líneas según tus especificaciones
def line1(t):
    return np.piecewise(t,
                       [t < tp/3, (t >= tp/3) & (t < tp), t >= tp],
                       [-0.25, lambda t: -0.25 + (0.5/(tp/3*2))*(t-tp/3), 0.25])

def line2(t):
    return np.piecewise(t,
                       [t < tp/3, (t >= tp/3) & (t < tp), t >= tp],
                       [0, lambda t: 0 + (0.85/(tp/3*2))*(t-tp/3), 0.85])

def line3(t):
    return np.piecewise(t,
                       [t < tp/3, t >= tp/3],
                       [lambda t: 0.5 + (0.7/(tp/3))*t, 1.2])

def line4(t):
    return np.piecewise(t,
                       [t < tp/3, t >= tp/3],
                       [lambda t: 1 + (0.7/(tp/3))*t, 1.7])

# Calculamos las líneas
y1 = line1(t)
y2 = line2(t)
y3 = line3(t)
y4 = line4(t)

# Creamos la figura
plt.figure(figsize=(12, 7))

# Dibujamos las líneas
plt.plot(t, y1, 'k-', linewidth=2, label='Línea 1 (y=-0.25 a 0.25)')
plt.plot(t, y2, 'k-', linewidth=2, label='Línea 2 (y=0 a 0.75)')
plt.plot(t, y3, 'k-', linewidth=2, label='Línea 3 (y=0.5 a 1.2)')
plt.plot(t, y4, 'k-', linewidth=2, label='Línea 4 (y=1 a 1.7)')

# Rellenamos las áreas con colores según tu imagen
plt.fill_between(t,-10,y1, where=(-10<y1),     color = 'red', alpha = 0.3)
plt.fill_between(t, y1, y2, where=(y1<y2), color='yellow', alpha=0.3)
plt.fill_between(t, y2, y3, where=(y2<y3), color='green', alpha=0.3)
plt.fill_between(t, y3, y4, where=(y3<y4), color='yellow', alpha=0.3)
plt.fill_between(t, y4, 10, where=(y4<10), color='red', alpha=0.3)

# Ploteamos la respuesta
plt.plot(T, yout/SS_response)

# Añadimos líneas verticales en t=3 y t=9
plt.axvline(x=3, color='gray', linestyle='--', alpha=0.7)
plt.axvline(x=9, color='gray', linestyle='--', alpha=0.7)

# Etiquetas y título
plt.xlabel('Tiempo [s]', fontsize=12)
plt.ylabel('Amplitud', fontsize=12)
plt.title('Envolvente', fontsize=14)

# Leyenda
plt.legend(loc='upper left')

# Límites de los ejes
plt.xlim(0, T[-1])
plt.ylim(-0.5, 2.0)

# Cuadrícula
plt.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
plt.show()


# Graficar el root locus
sys_total = sys_sp * sys_ph
rlist, klist = control.root_locus(sys_total, plot=False)
plt.figure()
control.root_locus(
    sys_total, 
    kvect=np.logspace(0, 1, 1),
    grid=True,
    plot=True)#,
    #xlim=(-2, 1), 
    #ylim=(-10, 10)
    #)
# # Ocultar leyendas del Root Locus
# ax = plt.gca()
# ax.get_legend().remove()

# Sombrear la región positiva de x (Re(s) > 0) en rojo
plt.axvspan(0, 1, facecolor='red', alpha=0.3, label='Región inestable (Re(s) > 0)')

plt.title('Lugar de las Raíces (sys_sp * sys_ph)')
#plt.legend()  # Mostrar leyenda
plt.show()

# -------------------------------
# Guardar JSON con derivadas LON
# -------------------------------
import os

# Asegurar tipos "serializables" (float nativo)
def f(x): 
    return float(x)

# Clonar el diccionario original y añadir derivadas
salida = constantes.copy()
salida.update({
    "X_u":         f(round(X_u,4)),
    "X_w":         f(round(X_w,4)),
    "Z_u":         f(round(Z_u,4)),
    "Z_w":         f(round(Z_w,4)),
    "u0" :         f(round(u0,4)),
    "M_u":         f(round(M_u,4)),
    "M_w":         f(round(M_w,4)),
    "Z_alpha":     f(round(Z_alpha,4)),
    "M_q":         f(round(M_q,4)),
    "M_alpha":     f(round(M_alpha,4)),
    "M_w_dot":     f(round(M_w_dot,4)),
    "M_alpha_dot": f(round(M_alpha_dot,4)),
    "M_de":        f(round(M_de,4)),
    "Z_de":        f(round(Z_de,4)),
})

# Construir nombre de archivo de salida

base, ext = os.path.splitext(input_path)
output_path = f"{base}_LonDerivatives{ext}"

# Escribir JSON con identación y sin notación científica excesiva
with open(output_path, "w", encoding="utf-8") as fout:
    json.dump(salida, fout, ensure_ascii=False, indent=2)

print(f"\nArchivo JSON generado: {output_path}")

# ================================
#  Construcción de A y B (Long)
#  Estados: x = [u, w, q, theta]
#  Entrada: δe (elevador)
# ================================
g = 9.81
theta0 = alpha_trim  + gamma0/57.3 # 

# --- "Hats" (corrección por Z_wdot) ---


den = (1.0 - Z_w_dot) if abs(1.0 - Z_w_dot) > 1e-9 else 1e-9

Z_u_hat  = Z_u  / den
Z_w_hat  = Z_w  / den
M_u_hat  = (M_u + M_w_dot * Z_u) / den
M_w_hat  = (M_w + M_w_dot* Z_w) / den #(M_alpha + M_alpha_dot * Z_w)
M_q_hat  = (M_q + M_alpha_dot) / den

Z_de_hat = Z_de / den
M_de_hat = (M_de + M_w_dot * Z_de) / den

X_theta = -g * np.cos(theta0)
Z_theta = -g * np.sin(theta0)

A = np.array([
    [X_u,        X_w,       0.0,     X_theta],
    [Z_u_hat,    Z_w_hat,   u0,      Z_theta],
    [M_u_hat,    M_w_hat,   M_q_hat, 0.0    ],
    [0.0,        0.0,       1.0,     0.0    ]
], dtype=float)

B = np.array([
    [0.0      ],
    [Z_de_hat ],
    [M_de_hat ],
    [0.0      ]
], dtype=float)

# A = np.array([ [Z_w_hat, 1], [M_w_hat, M_q_hat] ], dtype=float) 

# B = np.array([ [Z_de_hat ], [M_de_hat ] ], dtype=float)

# --- Autovalores y clasificación de modos ---
eigvals, eigvecs = np.linalg.eig(A)
idx = np.argsort(np.real(eigvals))
eigvals = eigvals[idx]

print("\n=== A y B longitudinales ===")
print("A =\n", A)
print("B =\n", B)

# -------------------------------
# Guardar JSON con derivadas LON
# -------------------------------
import os

# Asegurar tipos "serializables" (float nativo)
def f(x): 
    return float(x)

# Clonar el diccionario original y añadir derivadas
salida = constantes.copy()
salida.update({
    "X_u":         f(round(X_u,4)),
    "X_w":         f(round(X_w,4)),
    "X_theta":     f(round(X_theta,4)),
    "Z_u":         f(round(Z_u,4)),
    "Z_u_hat":     f(round(Z_u_hat,4)),
    "Z_w":         f(round(Z_w,4)),
    "Z_w_hat":     f(round(Z_w_hat,4)),
    "u0" :         f(round(u0,4)),
    "Z_theta":     f(round(Z_theta,4)),
    "M_u":         f(round(M_u,4)),
    "M_u_hat":     f(round(M_u_hat,4)),
    "M_w":         f(round(M_w,4)),
    "M_w_hat":     f(round(M_w_hat,4)),
    "Z_alpha":     f(round(Z_alpha,4)),
    "M_q":         f(round(M_q,4)),
    "M_q_hat":     f(round(M_q_hat,4)),
    "M_alpha":     f(round(M_alpha,4)),
    "M_w_dot":     f(round(M_w_dot,4)),
    "M_alpha_dot": f(round(M_alpha_dot,4)),
    "M_de":        f(round(M_de,4)),
    "Z_de":        f(round(Z_de,4)),
})

# Construir nombre de archivo de salida

base, ext = os.path.splitext(input_path)
output_path = f"{base}_LonDerivatives{ext}"

# Escribir JSON con identación y sin notación científica excesiva
with open(output_path, "w", encoding="utf-8") as fout:
    json.dump(salida, fout, ensure_ascii=False, indent=2)

print(f"\nArchivo JSON generado: {output_path}")

# ================================
#  Construcción de A y B (Long)
#  Estados: x = [u, w, q, theta]
#  Entrada: δe (elevador)
# ================================
g = 9.81
theta0 = alpha_trim  + gamma0/57.3 # 

def mode_metrics(lam):
    wn   = np.sqrt(np.real(lam)**2 + np.imag(lam)**2)
    zeta = -np.real(lam)/wn if wn > 1e-12 else np.nan
    T    = 2*np.pi/abs(np.imag(lam)) if abs(np.imag(lam))>1e-9 else np.inf
    return wn, zeta, T

# Identificar pares complejos (SP/Phugoid)
pairs, used = [], set()
for i in range(len(eigvals)):
    if i in used: 
        continue
    lam = eigvals[i]
    if abs(np.imag(lam)) > 1e-7:
        for j in range(i+1, len(eigvals)):
            if j in used: 
                continue
            if abs(eigvals[j] - np.conj(lam)) < 1e-6:
                pairs.append((lam, eigvals[j]))
                used.add(i); used.add(j)
                break

print("\nAutovalores(A):")
for i, lam in enumerate(eigvals, 1):
    wn, zeta, T = mode_metrics(lam)
    print(f"  λ{i}: {lam:.6f}   ->  ω_n={wn:.5f}  ζ={zeta:.3f}  T≈{T:.2f}s")

if pairs:
    metrics = [mode_metrics(p[0]) for p in pairs]
    idx_sp = int(np.argmax([m[0] for m in metrics]))  # mayor ω_n = SP
    for k, (p, m) in enumerate(zip(pairs, metrics)):
        tag = "SHORT-PERIOD" if k == idx_sp else "PHUGOID (probable)"
        print(f"\n[{tag}] ω_n={m[0]:.4f} rad/s, ζ={m[1]:.3f}, T≈{m[2]:.2f} s  (λ={p[0]:.6f}, {p[1]:.6f})")
else:
    print("\nNo se detectaron pares complejos claros (revisa signos/derivadas).")


import pandas as pd

# Convertir a DataFrames con nombres de filas y columnas
df_A = pd.DataFrame(A, 
                   index=['F_x', 'F_z', 'M_y', 'Pos_z'],
                   columns=['u', 'w', 'q', 'θ'])
df_B = pd.DataFrame(B,
                   index=['F_x', 'F_z', 'M_y', 'Pos_z'],
                   columns=['δ_e'])

print("Matriz A:")
print(df_A)
print("\n" + "="*50 + "\n")
print("Matriz B:")
print(df_B)
