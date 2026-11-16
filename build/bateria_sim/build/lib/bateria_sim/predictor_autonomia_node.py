#!/usr/bin/env python3
import os
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import tensorflow as tf

class PredictorAutonomiaNode(Node):
    def __init__(self):
        super().__init__('predictor_autonomia')
        base = os.path.expanduser('~/ros2_ws/modelos')
        self.modelo = tf.keras.models.load_model(f'{base}/modelo_lstm.h5', compile=False)
        sc = np.load(f'{base}/scaler_lstm.npz', allow_pickle=True)
        self.mean = sc['mean'].astype(np.float32)
        self.std = sc['std'].astype(np.float32)
        self.corr_pos = bool(self.mean[1] > 0.0)
        self.soc_100 = bool(self.mean[4] > 1.5)
        self.v_bat, self.i_bat, self.vel, self.carga = 8.4, 0.6, 0.0, 0.0
        self.soc = 100.0 if self.soc_100 else 1.0
        self.listo = False
        self.ventana = deque(maxlen=30)
        self.ciclos = 0
        self.create_subscription(BatteryState, '/battery_state', self.cb_bat, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_vel, 10)
        self.create_subscription(Float32, '/carga_kg', self.cb_carga, 10)
        self.pub = self.create_publisher(Float32, '/autonomia_restante', 10)
        self.create_timer(1.0, self.predecir)
        self.get_logger().info('predictor_autonomia iniciado a 1 Hz')

    def cb_bat(self, msg):
        self.v_bat = float(msg.voltage)
        c = float(msg.current)
        self.i_bat = abs(c) if self.corr_pos else c
        s = float(msg.percentage)
        if self.soc_100 and s <= 1.05:
            self.soc = s * 100.0
        elif not self.soc_100 and s > 1.5:
            self.soc = s / 100.0
        else:
            self.soc = s
        self.listo = True

    def cb_vel(self, msg):
        self.vel = abs(float(msg.linear.x))

    def cb_carga(self, msg):
        self.carga = max(0.0, float(msg.data))

    def predecir(self):
        if not self.listo:
            return
        m = np.array([self.v_bat, self.i_bat, self.vel, self.carga, self.soc], dtype=np.float32)
        if len(self.ventana) == 0:
            for _ in range(30):
                self.ventana.append(m)
        else:
            self.ventana.append(m)
        self.ciclos += 1
        seq = (np.array(self.ventana, dtype=np.float32) - self.mean) / self.std
        pred = max(0.0, float(self.modelo.predict(seq[None, :, :], verbose=0)[0][0]))
        out = Float32()
        out.data = pred
        self.pub.publish(out)
        self.get_logger().info(f'[{min(self.ciclos,30)}/30] SoC:{self.soc:.1f}% | Pred:{pred:.1f}s')

def main(args=None):
    rclpy.init(args=args)
    nodo = PredictorAutonomiaNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
