# -*- coding: utf-8 -*-
"""
SIL Simulation Visualization - Complete System with Flight States
Author: rtrujillo
Date: 2026-03-30
"""

import pandas as pd
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np

# ============================================
# CARGA DE DATOS
# ============================================

# Cargar el archivo CSV completo
import os
_BASE = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(_BASE, "..", "data", "SIL_sim_servos2.csv")

df = pd.read_csv(CSV_FILE)

# Mostrar información básica
print("="*60)
print("INFORMACIÓN DEL DATASET")
print("="*60)
print(f"Forma: {df.shape}")
print(f"Columnas disponibles: {df.columns.tolist()}")
print(f"\nPrimeras 5 filas:")
print(df.head())
print(f"\nÚltimas 5 filas:")
print(df.tail())
print(f"\nEstadísticas básicas:")
print(df.describe())

# ============================================
# VISUALIZACIÓN 1: ESTADOS DEL SISTEMA (du, dw, dq, dtheta)
# ============================================
print("\n" + "="*60)
print("VISUALIZACIÓN 1: Estados del Sistema (Variables de Estado)")
print("="*60)

# Verificar qué estados están disponibles
state_vars = ['du', 'dw', 'dq', 'dtheta']
available_states = [var for var in state_vars if var in df.columns]

if available_states:
    fig_states, axes_states = plt.subplots(len(available_states), 1, figsize=(12, 3*len(available_states)), sharex=True)
    
    # Si solo hay un estado, axes_states no es una lista
    if len(available_states) == 1:
        axes_states = [axes_states]
    
    state_labels = {
        'du': 'du - Velocidad axial (m/s)',
        'dw': 'dw - Velocidad vertical (m/s)',
        'dq': 'dq - Velocidad angular de pitch (rad/s)',
        'dtheta': 'dtheta - Ángulo de pitch (rad)'
    }
    
    state_colors = {
        'du': 'blue',
        'dw': 'green',
        'dq': 'red',
        'dtheta': 'orange'
    }
    
    for idx, state in enumerate(available_states):
        axes_states[idx].plot(df['t_s'], df[state], state_colors.get(state, 'black'), linewidth=1.5, label=state_labels.get(state, state))
        axes_states[idx].set_ylabel(state_labels.get(state, state))
        axes_states[idx].set_title(f'Evolución de {state_labels.get(state, state)}')
        axes_states[idx].grid(True, alpha=0.3)
        axes_states[idx].legend()
        
        # Agregar línea en cero para referencia
        axes_states[idx].axhline(y=0, color='k', linestyle='--', alpha=0.3, linewidth=0.5)
    
    axes_states[-1].set_xlabel('Tiempo (s)')
    plt.tight_layout()
    plt.show()
else:
    print("No se encontraron variables de estado en el archivo")

# ============================================
# VISUALIZACIÓN 2: SERVOS COMPLETOS
# ============================================
print("\n" + "="*60)
print("VISUALIZACIÓN 2: Respuesta de todos los servos")
print("="*60)

# Definir los servos y sus columnas
servos = [
    {'name': 'Elevator', 'cmd': 'elev_cmd_us', 'u': 'u_elev_us', 'y': 'y_elev_deg', 'color_cmd': 'blue', 'color_u': 'green', 'color_y': 'red'},
    {'name': 'Aileron',  'cmd': 'ail_cmd_us',  'u': 'u_ail_us',  'y': 'y_ail_deg',  'color_cmd': 'blue', 'color_u': 'green', 'color_y': 'orange'},
    {'name': 'Rudder',   'cmd': 'rud_cmd_us',  'u': 'u_rud_us',  'y': 'y_rud_deg',  'color_cmd': 'blue', 'color_u': 'green', 'color_y': 'purple'},
    {'name': 'Throttle', 'cmd': 'thro_cmd_us', 'u': 'u_thro_us', 'y': 'y_thro_perc', 'color_cmd': 'blue', 'color_u': 'green', 'color_y': 'brown'}
]

# Verificar qué servos están presentes
available_servos = []
for servo in servos:
    if servo['cmd'] in df.columns:
        available_servos.append(servo)

if available_servos:
    n_servos = len(available_servos)
    fig1, axes = plt.subplots(n_servos, 3, figsize=(15, 4 * n_servos))
    fig1.suptitle('Sistema Completo de Servos - SIL Simulation', fontsize=16, fontweight='bold')
    
    # Si solo hay un servo, axes no es una matriz 2D
    if n_servos == 1:
        axes = axes.reshape(1, -1)
    
    for idx, servo in enumerate(available_servos):
        # Comando
        axes[idx, 0].plot(df['t_s'], df[servo['cmd']], servo['color_cmd'], linewidth=1.5, label='Comando')
        axes[idx, 0].set_ylabel(servo['cmd'])
        axes[idx, 0].set_title(f'{servo["name"]} - Comando')
        axes[idx, 0].grid(True, alpha=0.3)
        axes[idx, 0].legend()
        
        # Señal PWM (si existe)
        if servo['u'] in df.columns:
            axes[idx, 1].plot(df['t_s'], df[servo['u']], servo['color_u'], linewidth=1.5, label='PWM')
            axes[idx, 1].set_ylabel(servo['u'])
            axes[idx, 1].set_title(f'{servo["name"]} - PWM')
            axes[idx, 1].grid(True, alpha=0.3)
            axes[idx, 1].legend()
        
        # Salida (si existe)
        if servo['y'] in df.columns:
            axes[idx, 2].plot(df['t_s'], df[servo['y']], servo['color_y'], linewidth=1.5, label='Salida')
            axes[idx, 2].set_ylabel(servo['y'])
            axes[idx, 2].set_title(f'{servo["name"]} - Salida')
            axes[idx, 2].grid(True, alpha=0.3)
            axes[idx, 2].legend()
    
    axes[-1, 0].set_xlabel('Tiempo (s)')
    axes[-1, 1].set_xlabel('Tiempo (s)')
    axes[-1, 2].set_xlabel('Tiempo (s)')
    
    plt.tight_layout()
    plt.show()
else:
    print("No se encontraron datos de servos en el archivo")

# ============================================
# VISUALIZACIÓN 3: SEÑALES RC
# ============================================
print("\n" + "="*60)
print("VISUALIZACIÓN 3: Señales RC")
print("="*60)

fig2, axes2 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# Verificar qué columnas RC están disponibles
rc_columns = ['rc_pitch', 'rc_roll', 'rc_yaw', 'rc_throttle', 'rc_flaps']
available_rc = [col for col in rc_columns if col in df.columns]

if available_rc:
    for col in available_rc:
        axes2[0].plot(df['t_s'], df[col], linewidth=1.5, label=col)
    axes2[0].set_ylabel('Señal RC')
    axes2[0].set_title('Señales RC - Comandos del Piloto')
    axes2[0].legend()
    axes2[0].grid(True, alpha=0.3)
    
    # Modos si están disponibles
    if 'mode' in df.columns:
        axes2[1].plot(df['t_s'], df['mode'], 'b-', linewidth=1.5, label='Mode Switch')
        axes2[1].set_ylabel('Modo')
        axes2[1].set_title('Modo de Vuelo')
        axes2[1].grid(True, alpha=0.3)
        axes2[1].legend()
    
    if 'arm' in df.columns:
        axes2[2].plot(df['t_s'], df['arm'], 'r-', linewidth=1.5, label='Arm Switch')
        axes2[2].set_xlabel('Tiempo (s)')
        axes2[2].set_ylabel('Armado')
        axes2[2].set_title('Estado de Armado')
        axes2[2].grid(True, alpha=0.3)
        axes2[2].legend()
else:
    axes2[0].text(0.5, 0.5, 'No hay datos RC disponibles', 
                  horizontalalignment='center', verticalalignment='center',
                  transform=axes2[0].transAxes)
    axes2[0].set_title('Señales RC no encontradas')

plt.tight_layout()
plt.show()

# ============================================
# VISUALIZACIÓN 4: COMPARACIÓN COMANDOS PILOTO VS SALIDAS
# ============================================
print("\n" + "="*60)
print("VISUALIZACIÓN 4: Comandos del Piloto")
print("="*60)

pilot_columns = ['pilot_roll', 'pilot_pitch', 'pilot_yaw', 'pilot_throttle']
available_pilot = [col for col in pilot_columns if col in df.columns]

if available_pilot:
    fig_pilot, axes_pilot = plt.subplots(len(available_pilot), 1, figsize=(12, 3*len(available_pilot)), sharex=True)
    
    if len(available_pilot) == 1:
        axes_pilot = [axes_pilot]
    
    pilot_labels = {
        'pilot_roll': 'Roll Command',
        'pilot_pitch': 'Pitch Command',
        'pilot_yaw': 'Yaw Command',
        'pilot_throttle': 'Throttle Command'
    }
    
    for idx, col in enumerate(available_pilot):
        axes_pilot[idx].plot(df['t_s'], df[col], 'b-', linewidth=1.5, label=pilot_labels.get(col, col))
        axes_pilot[idx].set_ylabel(pilot_labels.get(col, col))
        axes_pilot[idx].set_title(f'{pilot_labels.get(col, col)} del Piloto')
        axes_pilot[idx].grid(True, alpha=0.3)
        axes_pilot[idx].legend()
    
    axes_pilot[-1].set_xlabel('Tiempo (s)')
    plt.tight_layout()
    plt.show()

# ============================================
# VISUALIZACIÓN 5: COMPARACIÓN COMANDOS VS SALIDAS DE SERVOS
# ============================================
print("\n" + "="*60)
print("VISUALIZACIÓN 5: Comparación Comando vs Salida por Servo")
print("="*60)

servos_comp = [
    {'name': 'Elevator', 'cmd': 'elev_cmd_us', 'out': 'y_elev_deg', 'pos': (0, 0)},
    {'name': 'Aileron',  'cmd': 'ail_cmd_us',  'out': 'y_ail_deg',  'pos': (0, 1)},
    {'name': 'Rudder',   'cmd': 'rud_cmd_us',  'out': 'y_rud_deg',  'pos': (1, 0)},
    {'name': 'Throttle', 'cmd': 'thro_cmd_us', 'out': 'y_thro_perc', 'pos': (1, 1)}
]

# Contar cuántos servos tienen datos completos
valid_servos = [s for s in servos_comp if s['cmd'] in df.columns and s['out'] in df.columns]
n_valid = len(valid_servos)

if n_valid > 0:
    fig3, axes3 = plt.subplots(2, 2, figsize=(14, 10))
    fig3.suptitle('Comparación Comando vs Salida por Servo', fontsize=14, fontweight='bold')
    
    for servo in valid_servos:
        ax = axes3[servo['pos'][0], servo['pos'][1]]
        ax.plot(df['t_s'], df[servo['cmd']], 'b-', linewidth=1.5, label='Comando')
        ax.plot(df['t_s'], df[servo['out']], 'r-', linewidth=1.5, label='Salida')
        ax.set_xlabel('Tiempo (s)')
        ax.set_ylabel('Valor')
        ax.set_title(servo['name'])
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

# ============================================
# VISUALIZACIÓN 6: VISUALIZACIÓN INTERACTIVA CON PLOTLY
# ============================================
print("\n" + "="*60)
print("VISUALIZACIÓN INTERACTIVA CON PLOTLY")
print("="*60)

# Determinar cuántos subplots crear (servos + estados)
n_subplots = len(available_servos) + len(available_states)
if n_subplots > 0:
    subplot_titles = [f"{s['name']}" for s in available_servos] + [state_labels.get(s, s) for s in available_states]
    
    fig_plotly = make_subplots(
        rows=n_subplots, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.05,
        subplot_titles=subplot_titles,
        x_title='Tiempo (s)'
    )
    
    # Agregar servos
    for idx, servo in enumerate(available_servos):
        row = idx + 1
        if servo['cmd'] in df.columns:
            fig_plotly.add_trace(
                go.Scatter(x=df['t_s'], y=df[servo['cmd']], 
                          mode='lines', name=f'{servo["name"]} - Cmd', 
                          line=dict(color='blue', width=1)),
                row=row, col=1
            )
        if servo['u'] in df.columns:
            fig_plotly.add_trace(
                go.Scatter(x=df['t_s'], y=df[servo['u']], 
                          mode='lines', name=f'{servo["name"]} - PWM', 
                          line=dict(color='green', width=1, dash='dot')),
                row=row, col=1
            )
        if servo['y'] in df.columns:
            fig_plotly.add_trace(
                go.Scatter(x=df['t_s'], y=df[servo['y']], 
                          mode='lines', name=f'{servo["name"]} - Out', 
                          line=dict(color='red', width=1.5)),
                row=row, col=1
            )
    
    # Agregar estados del sistema
    for idx, state in enumerate(available_states):
        row = len(available_servos) + idx + 1
        fig_plotly.add_trace(
            go.Scatter(x=df['t_s'], y=df[state], 
                      mode='lines', name=state_labels.get(state, state), 
                      line=dict(color=state_colors.get(state, 'black'), width=1.5)),
            row=row, col=1
        )
    
    # Actualizar layout
    fig_plotly.update_layout(
        height=300 * n_subplots,
        showlegend=True,
        title_text="Sistema Completo - SIL Simulation (Interactivo)"
    )
    
    for i in range(1, n_subplots+1):
        fig_plotly.update_yaxes(title_text="Valor", row=i, col=1)
    
    fig_plotly.show()

# ============================================
# ANÁLISIS DE RESPUESTA AL ESCALÓN
# ============================================
print("\n" + "="*60)
print("ANÁLISIS DE RESPUESTA AL ESCALÓN")
print("="*60)

if available_states:
    fig_step, axes_step = plt.subplots(len(available_states), 1, figsize=(12, 3*len(available_states)))
    
    if len(available_states) == 1:
        axes_step = [axes_step]
    
    step_analysis = []
    
    for idx, state in enumerate(available_states):
        # Encontrar el primer escalón significativo
        state_data = df[state]
        threshold = state_data.max() * 0.1 if state_data.max() != 0 else 0.01
        
        # Buscar puntos donde hay cambio significativo
        state_diff = state_data.diff().abs()
        step_points = state_diff[state_diff > threshold].index
        
        if len(step_points) > 0:
            step_time = df.loc[step_points[0], 't_s']
            step_data = df[df['t_s'] > step_time].copy()
            step_data = step_data.reset_index(drop=True)
            
            # Graficar respuesta
            axes_step[idx].plot(step_data['t_s'], step_data[state], 'r-', linewidth=2, label='Respuesta')
            axes_step[idx].axvline(x=step_time, color='b', linestyle='--', alpha=0.5, label=f'Escalón en t={step_time:.3f}s')
            axes_step[idx].axhline(y=0, color='k', linestyle='-', alpha=0.2, linewidth=0.5)
            
            # Calcular métricas
            y_final = step_data[state].iloc[-100:].mean()
            y_max = step_data[state].max()
            y_min = step_data[state].min()
            overshoot = ((y_max - y_final) / abs(y_final)) * 100 if y_final != 0 else 0
            
            axes_step[idx].set_xlabel('Tiempo (s)')
            axes_step[idx].set_ylabel(state_labels.get(state, state))
            axes_step[idx].set_title(f'{state_labels.get(state, state)} - Respuesta al Escalón')
            axes_step[idx].grid(True, alpha=0.3)
            axes_step[idx].legend()
            
            step_analysis.append({
                'Estado': state_labels.get(state, state),
                'Tiempo_escalón_s': step_time,
                'Valor_final': f'{y_final:.6f}',
                'Valor_max': f'{y_max:.6f}',
                'Valor_min': f'{y_min:.6f}',
                'Sobrepico_%': f'{overshoot:.2f}'
            })
    
    plt.tight_layout()
    plt.show()
    
    # Mostrar análisis en tabla
    if step_analysis:
        print("\n" + "="*60)
        print("RESUMEN DE RESPUESTA AL ESCALÓN - ESTADOS")
        print("="*60)
        analysis_df = pd.DataFrame(step_analysis)
        print(analysis_df.to_string(index=False))

# ============================================
# ANÁLISIS ESTADÍSTICO COMPLETO
# ============================================
print("\n" + "="*60)
print("ANÁLISIS ESTADÍSTICO POR VARIABLE")
print("="*60)

# Análisis de servos
stats_data = []
for servo in servos_comp:
    if servo['cmd'] in df.columns and servo['out'] in df.columns:
        stats_data.append({
            'Variable': f"{servo['name']}_cmd",
            'Máximo': df[servo['cmd']].max(),
            'Mínimo': df[servo['cmd']].min(),
            'Media': df[servo['cmd']].mean(),
            'Desv_Std': df[servo['cmd']].std()
        })
        stats_data.append({
            'Variable': f"{servo['name']}_out",
            'Máximo': df[servo['out']].max(),
            'Mínimo': df[servo['out']].min(),
            'Media': df[servo['out']].mean(),
            'Desv_Std': df[servo['out']].std()
        })

# Análisis de estados
for state in available_states:
    stats_data.append({
        'Variable': state_labels.get(state, state),
        'Máximo': df[state].max(),
        'Mínimo': df[state].min(),
        'Media': df[state].mean(),
        'Desv_Std': df[state].std()
    })

stats_df = pd.DataFrame(stats_data)
print(stats_df.to_string(index=False))

print("\n" + "="*60)
print("FIN DEL ANÁLISIS")
print("="*60)
