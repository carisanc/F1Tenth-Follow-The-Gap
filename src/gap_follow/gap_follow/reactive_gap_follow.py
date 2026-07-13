#!/usr/bin/env python3

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveGapFollower(Node):
    def __init__(self):
        super().__init__('reactive_gap_follower')

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # PARÁMETROS
        self.vel_recta = 7.5
        self.vel_curva = 2.5        

        self.rango_max = 6.8
        self.radio_burbuja = 52
        self.ventana_suavizado = 3
        self.umbral_gap = 1.7

        self.zona_muerta = math.radians(1.5)
        self.alpha_suavizado = 0.50
        self.steering_previo = 0.0

        self.idx_inicio = None
        self.idx_fin = None
        self.fov_recorte = math.radians(85)

        # Contador de vueltas
        self.x = None
        self.y = None
        self.start_x = None
        self.start_y = None
        self.finished          = False
        self.armed_lap_counter = False
        self.lap_count         = 0
        self.start_time    = None
        self.last_lap_time = None
        self.best_lap_time = None

        self.get_logger().info("¡Reactive Gap Follower iniciado!")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x       = self.x
            self.start_y       = self.y
            self.start_time    = time.time()
            self.last_lap_time = self.start_time
            self.get_logger().info(
                f"Punto de salida: x={self.start_x:.2f}  y={self.start_y:.2f} | ¡Cronómetro iniciado!"
            )
            return

        dist = math.hypot(self.x - self.start_x, self.y - self.start_y)

        if not self.armed_lap_counter and dist > 8.0:
            self.armed_lap_counter = True
            self.get_logger().info("Contador de vuelta armado.")

        if self.armed_lap_counter and dist < 1.5:
            self.lap_count        += 1
            self.armed_lap_counter = False
            now        = time.time()
            lap_time   = now - self.last_lap_time
            total_time = now - self.start_time
            self.last_lap_time = now
            if self.best_lap_time is None or lap_time < self.best_lap_time:
                self.best_lap_time = lap_time
            self.get_logger().info(
                f"Vuelta {self.lap_count}/10 | "
                f"Tiempo vuelta: {lap_time:.2f} s | "
                f"Mejor vuelta: {self.best_lap_time:.2f} s | "
                f"Tiempo total: {total_time:.2f} s"
            )
            if self.lap_count >= 10:
                self.finished = True
                self.get_logger().info(
                    f"¡10 vueltas completadas! "
                    f"Mejor tiempo: {self.best_lap_time:.2f} s | "
                    f"Tiempo total de carrera: {total_time:.2f} s"
                )
                self._stop()

    def preprocess_lidar(self, ranges):
        proc = np.array(ranges, dtype=np.float64)
        proc[np.isinf(proc)] = 0.0
        proc[np.isnan(proc)] = 0.0
        proc[proc > self.rango_max] = self.rango_max
        if self.ventana_suavizado > 1:
            kernel = np.ones(self.ventana_suavizado) / self.ventana_suavizado
            proc = np.convolve(proc, kernel, mode='same')
        return proc

    def find_max_gap(self, free_space_ranges):
        libre = free_space_ranges > self.umbral_gap
        mejor_inicio, mejor_fin = 0, 0
        mejor_largo = 0
        inicio_actual = None
        for i, es_libre in enumerate(libre):
            if es_libre:
                if inicio_actual is None:
                    inicio_actual = i
            else:
                if inicio_actual is not None:
                    largo = i - inicio_actual
                    if largo > mejor_largo:
                        mejor_largo = largo
                        mejor_inicio, mejor_fin = inicio_actual, i - 1
                    inicio_actual = None
        if inicio_actual is not None:
            largo = len(libre) - inicio_actual
            if largo > mejor_largo:
                mejor_inicio, mejor_fin = inicio_actual, len(libre) - 1
        return mejor_inicio, mejor_fin

    def find_best_point(self, start_i, end_i):
        return (start_i + end_i) // 2

    def lidar_callback(self, data):
        if self.finished:
            self._stop()
            return

        angle_min = data.angle_min
        angle_increment = data.angle_increment

        if self.idx_inicio is None:
            centro = len(data.ranges) // 2
            n_rayos = int(self.fov_recorte / angle_increment)
            self.idx_inicio = max(0, centro - n_rayos)
            self.idx_fin = min(len(data.ranges) - 1, centro + n_rayos)

        proc = self.preprocess_lidar(data.ranges)
        frente = proc[self.idx_inicio:self.idx_fin + 1]

        valid_indices = np.where(frente > 0.1)[0]
        if len(valid_indices) > 0:
            idx_cercano = valid_indices[np.argmin(frente[valid_indices])]
            ini_burbuja = max(0, idx_cercano - self.radio_burbuja)
            fin_burbuja = min(len(frente), idx_cercano + self.radio_burbuja)
            frente[ini_burbuja:fin_burbuja] = 0.0

        gap_inicio, gap_fin = self.find_max_gap(frente)
        idx_objetivo = self.find_best_point(gap_inicio, gap_fin)
        idx_global   = idx_objetivo + self.idx_inicio
        steering_angle = angle_min + idx_global * angle_increment

        if abs(steering_angle) < self.zona_muerta:
            steering_angle = 0.0

        steering_angle = (self.alpha_suavizado * steering_angle
                          + (1.0 - self.alpha_suavizado) * self.steering_previo)
        self.steering_previo = steering_angle
        steering_angle = max(-0.41, min(0.41, steering_angle))

        # Velocidad continua — igual que el compañero que funciona
        factor_giro = abs(steering_angle) / 0.41
        speed = self.vel_recta - (self.vel_recta - self.vel_curva) * factor_giro

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar'
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def _stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveGapFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
