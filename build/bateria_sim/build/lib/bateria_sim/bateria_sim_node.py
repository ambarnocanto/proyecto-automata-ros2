import math
import random
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState


class BateriaSim(Node):

    def __init__(self):
        super().__init__('bateria_sim')

        # Variables del robot
        self.velocidad = 0.0
        self.carga_kg = 0.0

        # Parametros del modelo de bateria
        self.P_base = 4.5
        self.k1 = 15.0
        self.k2 = 6.0
        self.C_sim = 0.20
        self.R_int = 0.15

        # Estado inicial de la bateria
        self.soc = 1.0
        self.soc_minimo = 0.10

        # Ruido de potencia en W
        self.ruido_std = 0.05

        # Control del tiempo
        self.ultimo_tiempo = time.monotonic()

        # Suscripcion a la velocidad
        self.sub_velocidad = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback_velocidad,
            10
        )

        # Suscripcion a la carga
        self.sub_carga = self.create_subscription(
            Float32,
            '/carga_kg',
            self.callback_carga,
            10
        )

        # Publicador del estado de bateria
        self.pub_bateria = self.create_publisher(
            BatteryState,
            '/battery_state',
            10
        )

        # Temporizador de 1 segundo
        self.timer = self.create_timer(
            1.0,
            self.actualizar_bateria
        )

        self.get_logger().info(
            'Nodo bateria_sim iniciado correctamente'
        )

    # Recibir velocidad del robot
    def callback_velocidad(self, msg):
        self.velocidad = msg.linear.x

    # Recibir carga del robot
    def callback_carga(self, msg):
        self.carga_kg = max(0.0, msg.data)

    # Calcular voltaje OCV segun el SoC
    def calcular_ocv(self, soc):

        puntos = [
            (0.0, 6.0),
            (0.1, 6.8),
            (0.2, 7.1),
            (0.5, 7.4),
            (0.8, 7.8),
            (1.0, 8.4)
        ]

        soc = max(0.0, min(1.0, soc))

        for i in range(len(puntos) - 1):

            s0, v0 = puntos[i]
            s1, v1 = puntos[i + 1]

            if s0 <= soc <= s1:
                return v0 + (
                    (v1 - v0) * (soc - s0) / (s1 - s0)
                )

        return 8.4

    # Calcular corriente y voltaje
    def calcular_consumo(self, potencia, soc):

        ocv = self.calcular_ocv(soc)

        # Resolver P = V*I y V = OCV - I*R
        discriminante = max(
            0.0,
            ocv**2 - 4.0 * self.R_int * potencia
        )

        corriente = (
            2.0 * potencia /
            (ocv + math.sqrt(discriminante))
        )

        voltaje = ocv - corriente * self.R_int

        return corriente, voltaje

    # Actualizar la bateria cada segundo
    def actualizar_bateria(self):

        ahora = time.monotonic()
        dt = ahora - self.ultimo_tiempo
        self.ultimo_tiempo = ahora

        # Comprobar limite de bateria
        if self.soc <= self.soc_minimo:

            corriente = 0.0
            voltaje = self.calcular_ocv(self.soc)

        else:

            # Velocidad absoluta
            v = abs(self.velocidad)

            # Ruido gaussiano de potencia
            ruido = random.gauss(
                0.0,
                self.ruido_std
            )

            # Calculo de potencia
            potencia = (
                self.P_base
                + self.k1 * v**2
                + self.k2 * self.carga_kg * v
                + ruido
            )

            potencia = max(0.0, potencia)

            # Calculo de corriente y voltaje
            corriente, voltaje = self.calcular_consumo(
                potencia,
                self.soc
            )

            # Conteo de coulombs
            consumo_ah = corriente * dt / 3600.0

            # Actualizar SoC
            self.soc = max(
                self.soc_minimo,
                self.soc - consumo_ah / self.C_sim
            )

            # Aviso de bateria baja
            if self.soc <= self.soc_minimo:

                self.get_logger().warn(
                    'Bateria al 10 por ciento'
                )

        # Crear mensaje de bateria
        msg = BatteryState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.voltage = float(voltaje)

        # Corriente negativa indica descarga en ROS
        msg.current = float(-corriente)

        msg.percentage = float(self.soc)

        msg.charge = float(self.soc * self.C_sim)

        msg.capacity = float(self.C_sim)

        msg.design_capacity = float(self.C_sim)

        msg.temperature = float('nan')

        msg.present = True

        msg.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        )

        msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_GOOD
        )

        if self.soc <= self.soc_minimo:
            msg.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            )
        else:
            msg.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            )

        # Publicar estado de bateria
        self.pub_bateria.publish(msg)

        # Mostrar informacion en terminal
        self.get_logger().info(
            f'Voltaje: {voltaje:.2f} V | '
            f'Corriente consumida: {corriente:.3f} A | '
            f'SoC: {self.soc * 100:.2f}%'
        )


def main(args=None):

    rclpy.init(args=args)

    nodo = BateriaSim()

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
