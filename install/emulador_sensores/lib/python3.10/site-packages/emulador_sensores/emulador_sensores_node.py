
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

class EmuladorSensores(Node):

    def __init__(self):
        super().__init__('emulador_sensores')

        # Ruido gaussiano: 5 mm
        self.ruido_std = 0.005
        self.contador = 0

        # Publicadores de los sensores simulados
        self.pub_izq = self.create_publisher(
            Range, '/rango_izq', 10
        )

        self.pub_der = self.create_publisher(
            Range, '/rango_der', 10
        )

        self.pub_frontal = self.create_publisher(
            Range, '/rango_frontal', 10
        )

        # Suscripcion al LiDAR
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback_scan,
            10
        )

        self.get_logger().info(
            'Emulador de sensores iniciado correctamente'
        )

    # Obtener la distancia minima de un sector
    def calcular_distancia(
        self, scan, centro_grados, apertura_grados,
        alcance_maximo
    ):

        centro = math.radians(centro_grados)
        mitad = math.radians(apertura_grados / 2.0)

        distancias = []

        for i, distancia in enumerate(scan.ranges):

            angulo = (
                scan.angle_min
                + i * scan.angle_increment
            )

            # Diferencia angular normalizada
            diferencia = math.atan2(
                math.sin(angulo - centro),
                math.cos(angulo - centro)
            )

            if abs(diferencia) > mitad:
                continue

            # Descartar lecturas invalidas
            if not math.isfinite(distancia):
                continue

            if not (
                scan.range_min
                <= distancia
                <= scan.range_max
            ):
                continue

            distancias.append(distancia)

        # Si no hay lectura valida, usar alcance maximo
        if not distancias:
            return alcance_maximo

        # Distancia minima y limite del sensor
        distancia = min(
            min(distancias),
            alcance_maximo
        )

        # Agregar ruido gaussiano
        distancia += random.gauss(
            0.0, self.ruido_std
        )

        # Limitar al rango permitido
        distancia = max(
            0.03,
            min(alcance_maximo, distancia)
        )

        return distancia

    # Crear mensaje de distancia
    def crear_mensaje(
        self, scan, distancia, apertura,
        alcance_maximo, tipo
    ):

        mensaje = Range()

        mensaje.header = scan.header
        mensaje.radiation_type = tipo
        mensaje.field_of_view = math.radians(
            apertura
        )
        mensaje.min_range = 0.03
        mensaje.max_range = alcance_maximo
        mensaje.range = float(distancia)

        return mensaje

    # Procesar las lecturas del LiDAR
    def callback_scan(self, scan):

        # Sensor lateral izquierdo
        distancia_izq = self.calcular_distancia(
            scan, 90.0, 25.0, 1.2
        )

        # Sensor lateral derecho
        distancia_der = self.calcular_distancia(
            scan, -90.0, 25.0, 1.2
        )

        # Sensor frontal
        distancia_frontal = self.calcular_distancia(
            scan, 0.0, 15.0, 2.0
        )

        # Mensajes sensor_msgs/Range
        msg_izq = self.crear_mensaje(
            scan,
            distancia_izq,
            25.0,
            1.2,
            Range.INFRARED
        )

        msg_der = self.crear_mensaje(
            scan,
            distancia_der,
            25.0,
            1.2,
            Range.INFRARED
        )

        msg_frontal = self.crear_mensaje(
            scan,
            distancia_frontal,
            15.0,
            2.0,
            Range.ULTRASOUND
        )

        # Publicar las tres distancias
        self.pub_izq.publish(msg_izq)
        self.pub_der.publish(msg_der)
        self.pub_frontal.publish(msg_frontal)

        # Mostrar valores cada diez escaneos
        self.contador += 1

        if self.contador % 10 == 0:
            self.get_logger().info(
                f'Izq: {distancia_izq:.3f} m | '
                f'Der: {distancia_der:.3f} m | '
                f'Frontal: {distancia_frontal:.3f} m'
            )

def main(args=None):

    rclpy.init(args=args)
    nodo = EmuladorSensores()

    try:
        rclpy.spin(nodo)

    except KeyboardInterrupt:
        pass

    finally:
        nodo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
