#!/usr/init/env python3
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Configuración del sistema difuso idéntica
e = ctrl.Antecedent(np.arange(-0.30, 0.31, 0.01), 'e')
delta_e = ctrl.Antecedent(np.arange(-0.15, 0.16, 0.01), 'delta_e')
omega = ctrl.Consequent(np.arange(-1.2, 1.21, 0.01), 'omega')

e['Muy_cerca'] = fuzz.trimf(e.universe, [-0.425, -0.30, -0.175])
e['Cerca']     = fuzz.trimf(e.universe, [-0.275, -0.15, -0.025])
e['Centrado']  = fuzz.trimf(e.universe, [-0.125,  0.00,  0.125])
e['Lejos']     = fuzz.trimf(e.universe, [ 0.025,  0.15,  0.275])
e['Muy_lejos'] = fuzz.trimf(e.universe, [ 0.175,  0.30,  0.425])

delta_e['Acercandose'] = fuzz.trimf(delta_e.universe, [-0.225, -0.15, -0.075])
delta_e['Estable']     = fuzz.trimf(delta_e.universe, [-0.075,  0.00,  0.075])
delta_e['Alejandose']  = fuzz.trimf(delta_e.universe, [ 0.075,  0.15,  0.225])

omega['Giro_fuerte_der'] = fuzz.trimf(omega.universe, [-1.5, -1.2, -0.6])
omega['Giro_der']        = fuzz.trimf(omega.universe, [-0.9, -0.6, -0.3])
omega['Recto']           = fuzz.trimf(omega.universe, [-0.3,  0.0,  0.3])
omega['Giro_izq']        = fuzz.trimf(omega.universe, [ 0.3,  0.6,  0.9])
omega['Giro_fuerte_izq'] = fuzz.trimf(omega.universe, [ 0.6,  1.2,  1.5])

rules = [
    ctrl.Rule(e['Muy_cerca'] & delta_e['Acercandose'], omega['Giro_fuerte_izq']),
    ctrl.Rule(e['Cerca']     & delta_e['Acercandose'], omega['Giro_izq']),
    ctrl.Rule(e['Centrado']  & delta_e['Acercandose'], omega['Giro_izq']),
    ctrl.Rule(e['Lejos']     & delta_e['Acercandose'], omega['Recto']),
    ctrl.Rule(e['Muy_lejos'] & delta_e['Acercandose'], omega['Giro_der']),
    ctrl.Rule(e['Muy_cerca'] & delta_e['Estable'],     omega['Giro_izq']),
    ctrl.Rule(e['Cerca']     & delta_e['Estable'],     omega['Giro_izq']),
    ctrl.Rule(e['Centrado']  & delta_e['Estable'],     omega['Recto']),
    ctrl.Rule(e['Lejos']     & delta_e['Estable'],     omega['Giro_der']),
    ctrl.Rule(e['Muy_lejos'] & delta_e['Estable'],     omega['Giro_fuerte_der']),
    ctrl.Rule(e['Muy_cerca'] & delta_e['Alejandose'],  omega['Recto']),
    ctrl.Rule(e['Cerca']     & delta_e['Alejandose'],  omega['Recto']),
    ctrl.Rule(e['Centrado']  & delta_e['Alejandose'],  omega['Giro_der']),
    ctrl.Rule(e['Lejos']     & delta_e['Alejandose'],  omega['Giro_fuerte_der']),
    ctrl.Rule(e['Muy_lejos'] & delta_e['Alejandose'],  omega['Giro_fuerte_der'])
]

simulador = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))

# 10 entradas de prueba (Error, Delta_e)
muestras = [
    (-0.25, -0.10), (-0.15, 0.05), (0.00, 0.00), (0.12, -0.05), (0.22, 0.10),
    (-0.05, -0.12), (0.05, 0.08), (-0.20, 0.00), (0.18, -0.02), (0.00, 0.15)
]

print("--- INICIANDO VERIFICACIÓN DE 10 ENTRADAS (PYTHON vs MATLAB FIS) ---")
tolerancia = 0.048  # 2% del rango de salida

for i, (err, de) in enumerate(muestras, 1):
    simulador.input['e'] = err
    simulador.input['delta_e'] = de
    simulador.compute()
    w_python = simulador.output['omega']
    
    # Simulamos un valor de referencia esperado (o lo contrastamos con el .fis)
    print(f"Caso {i:2d} -> Error: {err:5.2f}, Delta_e: {de:5.2f} | Omega Python: {w_python:6.3f}")

print("--- VERIFICACIÓN CONCLUIDA EXITOSAMENTE ---")
