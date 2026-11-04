import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
import math

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('nodo_supervisor')
        
        # Variables de estado del robot
        self.soc = 1.0
        self.en_retorno = False
        self.punto_actual_idx = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        
        # Definición de waypoints del circuito (puntos de control CP1 a CP6)
        self.waypoints = [
            (1.0, 0.4),  # CP1
            (2.5, 0.4),  # CP2
            (2.5, 2.0),  # CP3
            # ... agrega las coordenadas de tu circuito en Gazebo
        ]
        self.base_coords = (0.0, 0.0)  # Coordenadas de la base de carga
        
        # Suscripciones a los tópicos de batería y odometría
        self.create_subscription(BatteryState, '/battery_state', self.cb_bateria, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        
        # Publicador de comandos de velocidad hacia el robot
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Temporizador de control a 10 Hz
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Nodo supervisor de línea base iniciado correctamente.")

    def cb_bateria(self, msg):
        # Actualiza el porcentaje de batería recibido del nodo bateria_sim
        self.soc = msg.percentage

    def cb_odom(self, msg):
        # Actualiza la posición actual del robot desde /odom
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

    def control_loop(self):
        cmd = Twist()
        
        # Regla fija del supervisor: Si SoC < 30%, activar el retorno a la base
        if self.soc < 0.30 and not self.en_retorno:
            self.en_retorno = True
            self.get_logger().warn(f"¡SoC crítico ({self.soc*100:.1f}%)! Activando regla de retorno fijo a la base.")
        
        # Seleccionar el objetivo actual (base o siguiente punto de control)
        if self.en_retorno:
            meta_x, meta_y = self.base_coords
        else:
            meta_x, meta_y = self.waypoints[self.punto_actual_idx]

        # Control proporcional provisional básico hacia el objetivo (meta_x, meta_y)
        error_x = meta_x - self.pos_x
        error_y = meta_y - self.pos_y
        distancia = math.sqrt(error_x**2 + error_y**2)
        
        # Si llega al waypoint actual, pasa al siguiente (en modo patrullaje)
        if distancia < 0.2 and not self.en_retorno:
            self.punto_actual_idx = (self.punto_actual_idx + 1) % len(self.waypoints)
            self.get_logger().info(f"Punto de control alcanzado. Avanzando al siguiente.")

        # Generar velocidades lineales y angulares proporcionales
        if distancia > 0.1:
            cmd.linear.x = min(0.15, 0.5 * distancia)
            angulo_meta = math.atan2(error_y, error_x)
            cmd.angular.z = 1.0 * angulo_meta
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
