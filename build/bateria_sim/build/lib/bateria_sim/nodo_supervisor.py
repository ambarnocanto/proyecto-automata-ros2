#!/usr/bin/env python3
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('nodo_supervisor')
        self.declare_parameter('modo', 'lstm')       # 'lstm' o 'fijo'
        self.declare_parameter('velocidad', 0.15)    # 0.10 a 0.20 m/s
        self.declare_parameter('carga', 0.0)         # 0.0 a 0.50 kg

        self.modo = str(self.get_parameter('modo').value).lower()
        self.v_mision = float(self.get_parameter('velocidad').value)
        self.carga_kg = float(self.get_parameter('carga').value)

        ruta_yaml = os.path.expanduser('~/ros2_ws/retorno.yaml')
        with open(ruta_yaml, 'r') as f:
            self.tabla_retorno = yaml.safe_load(f)['retorno_tabla']

        self.base_coords = (1.0, 0.4)
        self.soc = 1.0
        self.autonomia_pred = 9999.0
        self.autonomia_recibida = False
        self.pos_x, self.pos_y = 1.0, 0.4
        self.en_retorno = False
        self.mision_terminada = False
        self.puntos_cubiertos = 0
        self.ultimo_cp = None
        self.descarga_completa = False

        self.create_subscription(BatteryState, '/battery_state', self.cb_bat, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(Float32, '/autonomia_restante', self.cb_auto, 10)
        self.pub_carga = self.create_publisher(Float32, '/carga_kg', 10)
        self.pub_vel = self.create_publisher(Float32, '/velocidad_mision', 10)
        self.create_timer(0.2, self.ciclo_supervisor)
        self.get_logger().info(
            f"Supervisor activo | Modo={self.modo.upper()} | v={self.v_mision}m/s | carga={self.carga_kg}kg"
        )

    def cb_bat(self, msg):
        self.soc = float(msg.percentage)
        if self.soc <= 0.10 and not self.mision_terminada:
            self.descarga_completa = True
            self.finalizar("FALLA: Batería agotada (SoC <= 10%) antes de llegar a la base")

    def cb_odom(self, msg):
        self.pos_x = float(msg.pose.pose.position.x)
        self.pos_y = float(msg.pose.pose.position.y)

    def cb_auto(self, msg):
        self.autonomia_pred = float(msg.data)
        self.autonomia_recibida = True

    def finalizar(self, motivo):
        self.mision_terminada = True
        v_stop = Float32()
        v_stop.data = 0.0
        self.pub_vel.publish(v_stop)
        self.get_logger().warn(
            f"\n=== FIN MISIÓN ({self.modo.upper()}) ===\n"
            f"Motivo: {motivo}\n"
            f"Puntos cubiertos: {self.puntos_cubiertos} | Descarga completa: {self.descarga_completa} | SoC final: {self.soc*100:.1f}%"
        )

    def ciclo_supervisor(self):
        m_c, m_v = Float32(), Float32()
        m_c.data = self.carga_kg
        m_v.data = 0.0 if self.mision_terminada else self.v_mision
        self.pub_carga.publish(m_c)
        self.pub_vel.publish(m_v)
        if self.mision_terminada:
            return

        if self.en_retorno:
            d_base_real = math.hypot(self.pos_x - self.base_coords[0], self.pos_y - self.base_coords[1])
            if d_base_real < 0.35:
                self.finalizar("ÉXITO: Robot regresó a la base de carga")
            return

        for cp in self.tabla_retorno:
            cp_id = cp['punto_id']
            cx, cy = cp['coordenadas']
            if math.hypot(self.pos_x - cx, self.pos_y - cy) < 0.45 and self.ultimo_cp != cp_id:
                self.ultimo_cp = cp_id
                self.puntos_cubiertos += 1
                d_base = float(cp['d_base'])
                t_req = 1.2 * (d_base / max(self.v_mision, 0.05))
                self.get_logger().info(
                    f"[CP_{cp_id}] Cubiertos:{self.puntos_cubiertos} | SoC:{self.soc*100:.1f}% | "
                    f"AutoPred:{self.autonomia_pred:.1f}s vs Req:{t_req:.1f}s"
                )
                if self.modo == 'fijo' and self.soc < 0.30:
                    self.en_retorno = True
                    self.get_logger().warn(f"Retorno FIJO activado en CP_{cp_id} (SoC < 30%)")
                elif self.modo == 'lstm' and self.autonomia_recibida and self.autonomia_pred <= t_req:
                    self.en_retorno = True
                    self.get_logger().warn(f"Retorno LSTM activado en CP_{cp_id} ({self.autonomia_pred:.1f}s <= {t_req:.1f}s)")

def main(args=None):
    rclpy.init(args=args)
    nodo = SupervisorNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
