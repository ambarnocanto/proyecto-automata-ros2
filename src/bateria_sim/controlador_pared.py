#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class ControladorParedFuzzy(Node):
    def __init__(self):
        super().__init__('controlador_pared_fuzzy')
        
        # Suscripción al sensor y publicación de velocidades
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32, '/velocidad_mision', lambda m: setattr(self, 'v_constante', float(m.data)), 10)
        
        # Timer a 10 Hz (0.1 segundos)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables de estado y buffer para promedio móvil (3 muestras)
        self.d_actual = 0.30
        self.d_frontal = 1.0
        self.evadiendo_frontal = False
        self.historial_d = [0.30, 0.30, 0.30]
        self.modo_retorno = False  # Cambiar a True para el tramo de retorno (modo espejo)
        
        # Velocidad lineal configurable (Puedes cambiarla a 0.10, 0.15 o 0.20 para las pruebas)
        self.v_constante = 0.20    

        # Control de registros para la terminal
        self.contador_registro = 0

        # Inicializar el sistema difuso
        self.inicializar_sistema_difuso()

    def inicializar_sistema_difuso(self):
        # Antecedentes y Consecuente
        e = ctrl.Antecedent(np.arange(-0.30, 0.31, 0.01), 'e')
        delta_e = ctrl.Antecedent(np.arange(-0.15, 0.16, 0.01), 'delta_e')
        omega = ctrl.Consequent(np.arange(-1.2, 1.21, 0.01), 'omega')

        # Funciones de membresía para e (5 conjuntos)
        e['Muy_cerca'] = fuzz.trimf(e.universe, [-0.425, -0.30, -0.175])
        e['Cerca']     = fuzz.trimf(e.universe, [-0.275, -0.15, -0.025])
        e['Centrado']  = fuzz.trimf(e.universe, [-0.125,  0.00,  0.125])
        e['Lejos']     = fuzz.trimf(e.universe, [ 0.025,  0.15,  0.275])
        e['Muy_lejos'] = fuzz.trimf(e.universe, [ 0.175,  0.30,  0.425])

        # Funciones de membresía para delta_e (3 conjuntos)
        delta_e['Acercandose'] = fuzz.trimf(delta_e.universe, [-0.225, -0.15, -0.075])
        delta_e['Estable']     = fuzz.trimf(delta_e.universe, [-0.075,  0.00,  0.075])
        delta_e['Alejandose']  = fuzz.trimf(delta_e.universe, [ 0.075,  0.15,  0.225])

        # Funciones de membresía para omega (5 conjuntos)
        omega['Giro_fuerte_der'] = fuzz.trimf(omega.universe, [-1.5, -1.2, -0.6])
        omega['Giro_der']        = fuzz.trimf(omega.universe, [-0.9, -0.6, -0.3])
        omega['Recto']           = fuzz.trimf(omega.universe, [-0.3,  0.0,  0.3])
        omega['Giro_izq']        = fuzz.trimf(omega.universe, [ 0.3,  0.6,  0.9])
        omega['Giro_fuerte_izq'] = fuzz.trimf(omega.universe, [ 0.6,  1.2,  1.5])

        # 15 Reglas Difusas
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

        sistema_ctrl = ctrl.ControlSystem(rules)
        self.simulador = ctrl.ControlSystemSimulation(sistema_ctrl)

    def scan_callback(self, msg):
        if not self.modo_retorno:
            self.d_actual = msg.ranges[270] if len(msg.ranges) > 270 else 0.30
        else:
            self.d_actual = msg.ranges[90] if len(msg.ranges) > 90 else 0.30

        self.d_frontal = msg.ranges[0] if len(msg.ranges) > 0 else 1.0

    def identificar_escenario(self, d_lat, d_front):
        """Identifica automáticamente el escenario en base a las lecturas del LiDAR"""
        if d_front < 0.40:
            return "Curva de 90° (Obstáculo frontal)"
        elif d_lat > 0.60:
            return "Ensanchamiento brusco"
        elif self.evadiendo_frontal:
            return "Maniobra de Seguridad"
        else:
            return "Pasillo Recto / Normal"

    def control_loop(self):
        self.contador_registro += 1

        # 1. Promedio móvil (3 muestras)
        self.historial_d.pop(0)
        self.historial_d.append(self.d_actual)
        d_bar_k = np.mean(self.historial_d)

        if not hasattr(self, 'd_bar_anterior') or np.isnan(self.d_bar_anterior):
            self.d_bar_anterior = d_bar_k

        # 2. Derivada filtrada
        if not np.isnan(d_bar_k) and not np.isnan(self.d_bar_anterior):
            delta_e_val = (d_bar_k - self.d_bar_anterior) / 0.1
            delta_e_val = np.clip(delta_e_val, -0.15, 0.15)
        else:
            delta_e_val = 0.0

        self.d_bar_anterior = d_bar_k
        error_e = d_bar_k - 0.30

        # 3. Regla de seguridad frontal
        if self.d_frontal < 0.35:
            self.evadiendo_frontal = True
        elif self.d_frontal > 0.50:
            self.evadiendo_frontal = False

        # Identificar el escenario actual para la terminal
        escenario_actual = self.identificar_escenario(d_bar_k, self.d_frontal)

        # ---> REPORTE PROFESIONAL EN VIVO EN LA TERMINAL <---
        estado_txt = "⚠️ EVASIÓN ACTIVA ⚠️" if self.evadiendo_frontal else "Control Difuso Normal"
        print(f"[Reg: {self.contador_registro:04d}] | Vel: {self.v_constante:.2f} m/s | Escenario: {escenario_actual:<25} | Dist: {d_bar_k:.3f}m | Error(e): {error_e:+.3f}m | Estado: {estado_txt}")

        # 4. Cálculo de velocidad angular (omega)
        if self.evadiendo_frontal:
            w = 1.2 if not self.modo_retorno else -1.2
        else:
            self.simulador.input['e'] = np.clip(error_e, -0.30, 0.30)
            self.simulador.input['delta_e'] = delta_e_val
            try:
                self.simulador.compute()
                w = self.simulador.output['omega']
            except:
                w = 0.0

            if self.modo_retorno:
                w = -w

        # 5. Publicar velocidad
        twist = Twist()
        twist.linear.x = float(self.v_constante)
        twist.angular.z = 0.0 if self.v_constante <= 0.0 else float(np.clip(w, -1.2, 1.2))
        if self.v_constante > 0.16 and abs(twist.angular.z) > 0.5:
            twist.linear.x = float(self.v_constante * 0.55)
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nodo = ControladorParedFuzzy()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
