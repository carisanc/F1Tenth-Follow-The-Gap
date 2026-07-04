#!/usr/bin/env python3

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


PI = math.pi

# ── Parámetros tunables ───────────────────────────────────────────────
ALPHA               = 0.30    # EMA: fracción del ángulo nuevo por scan
MAX_ANGLE_DELTA_DEG = 3.0     # rate limiter °/scan
DEAD_ZONE_DEG       = 2.0     # ángulos < esto → 0 en recta
CAR_HALF_WIDTH      = 0.35    # metros, para la burbuja
FOV_DEG             = 100.0   # campo de visión ±°
MAX_STEERING        = 0.42    # rad, clamp máximo de dirección
BUBBLE_RADIUS       = 13      # índices fijos de burbuja mínima
SMOOTHING_WINDOW    = 5       # ventana media móvil

# Velocidades [m/s]
SPEED_STRAIGHT      = 8.5
SPEED_MEDIUM        = 4.8
SPEED_CURVE         = 2.2
SPEED_DANGER        = 1.2

# Contador de vueltas
MAX_LAPS            = 10
REARM_DISTANCE      = 8.0    # metros — distancia mínima antes de rearmar el contador
FINISH_RADIUS       = 1.5    # metros — radio de la zona de meta
# ─────────────────────────────────────────────────────────────────────


class ReactiveGapFollower(Node):

    def __init__(self):
        super().__init__('reactive_gap_follower')

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Filtro de ángulo
        self.smoothed_angle = 0.0

        # Odometría y vueltas
        self.x = None
        self.y = None
        self.start_x = None
        self.start_y = None

        self.finished           = False
        self.armed_lap_counter  = False
        self.lap_count          = 0

        self.start_time    = None
        self.last_lap_time = None
        self.best_lap_time = None

        self.get_logger().info("Reactive Gap Follower iniciado — esperando odometría.")

    # ------------------------------------------------------------------
    # Odometría y contador de vueltas
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Primera lectura → fijar punto de salida
        if self.start_x is None:
            self.start_x       = self.x
            self.start_y       = self.y
            self.start_time    = time.time()
            self.last_lap_time = self.start_time
            self.get_logger().info(
                f"Punto de salida: x={self.start_x:.2f}  y={self.start_y:.2f}"
            )
            return

        dist = math.hypot(self.x - self.start_x, self.y - self.start_y)

        # Armar el contador una vez que el carro se alejó lo suficiente
        if not self.armed_lap_counter and dist > REARM_DISTANCE:
            self.armed_lap_counter = True
            self.get_logger().info("Contador de vuelta armado.")

        # Detectar cruce de meta
        if self.armed_lap_counter and dist < FINISH_RADIUS:
            self.lap_count        += 1
            self.armed_lap_counter = False

            now       = time.time()
            lap_time  = now - self.last_lap_time
            total_time = now - self.start_time
            self.last_lap_time = now

            if self.best_lap_time is None or lap_time < self.best_lap_time:
                self.best_lap_time = lap_time

            self.get_logger().info(
                f"Vuelta {self.lap_count}/{MAX_LAPS} | "
                f"Tiempo vuelta: {lap_time:.2f} s | "
                f"Mejor vuelta: {self.best_lap_time:.2f} s | "
                f"Tiempo total: {total_time:.2f} s"
            )

            if self.lap_count >= MAX_LAPS:
                self.finished = True
                total = time.time() - self.start_time
                self.get_logger().info(
                    f"¡{MAX_LAPS} vueltas completadas! "
                    f"Mejor vuelta: {self.best_lap_time:.2f} s | "
                    f"Tiempo total: {total:.2f} s"
                )
                self._stop()

    # ------------------------------------------------------------------
    # Utilidades de índice/ángulo
    # ------------------------------------------------------------------

    @staticmethod
    def _angle_at(scan, idx):
        return scan.angle_min + idx * scan.angle_increment

    @staticmethod
    def _idx_for_angle(scan, angle_rad):
        return int(round(
            (angle_rad - scan.angle_min) / scan.angle_increment
        ))

    # ------------------------------------------------------------------
    # Callback principal de LiDAR
    # ------------------------------------------------------------------

    def scan_callback(self, scan):

        if self.finished:
            self._stop()
            return

        # ── 1. Preprocesar ────────────────────────────────────────────
        ranges = np.array(scan.ranges, dtype=np.float64)
        ranges = np.where(
            np.isfinite(ranges),
            np.clip(ranges, 0.0, scan.range_max),
            0.0
        )

        # ── 2. FOV ───────────────────────────────────────────────────
        fov_rad = math.radians(FOV_DEG / 2.0)
        lo = max(2,             self._idx_for_angle(scan, -fov_rad))
        hi = min(len(ranges)-3, self._idx_for_angle(scan,  fov_rad))

        # ── 3. Suavizado (media móvil) ────────────────────────────────
        kernel = np.ones(SMOOTHING_WINDOW) / SMOOTHING_WINDOW
        ranges_smooth = ranges.copy()
        pad = SMOOTHING_WINDOW // 2
        ranges_smooth[lo:hi+1] = np.convolve(
            ranges[lo-pad:hi+pad+1], kernel, mode='valid'
        )[:hi-lo+1]

        # ── 4. Obstáculo más cercano ──────────────────────────────────
        closest_idx = int(np.argmin(ranges_smooth[lo:hi+1])) + lo

        # ── 5. Burbuja adaptativa ─────────────────────────────────────
        obs_dist   = max(ranges_smooth[closest_idx], 0.05)
        half_angle = math.atan2(CAR_HALF_WIDTH, obs_dist)
        bub_radius = int(half_angle / scan.angle_increment)
        bub_radius = max(BUBBLE_RADIUS, min(bub_radius, 150))

        bub_start = max(0,           closest_idx - bub_radius)
        bub_end   = min(len(ranges), closest_idx + bub_radius + 1)
        ranges_smooth[bub_start:bub_end] = 0.0

        # ── 6. Gap más largo ──────────────────────────────────────────
        best_start, best_end = lo, lo
        cur_start = -1
        best_len  = 0

        for i in range(lo, hi + 1):
            if ranges_smooth[i] > 0.0:
                if cur_start < 0:
                    cur_start = i
            else:
                if cur_start >= 0:
                    length = i - cur_start
                    if length > best_len:
                        best_len   = length
                        best_start = cur_start
                        best_end   = i - 1
                    cur_start = -1

        if cur_start >= 0 and (hi + 1 - cur_start) > best_len:
            best_start = cur_start
            best_end   = hi

        # ── 7. Mejor punto: mezcla centroide + punto más lejano ───────
        #    80% centro del gap + 20% punto más lejano (igual que referencia)
        gap_r      = ranges_smooth[best_start:best_end + 1]
        gap_center = (best_start + best_end) // 2
        farthest   = best_start + int(np.argmax(gap_r))
        best_idx   = int(0.80 * gap_center + 0.20 * farthest)

        raw_angle = self._angle_at(scan, best_idx)
        raw_angle = max(-MAX_STEERING, min(MAX_STEERING, raw_angle))

        # ── 8. Dead-zone ──────────────────────────────────────────────
        if abs(math.degrees(raw_angle)) < DEAD_ZONE_DEG:
            raw_angle = 0.0

        # ── 9. Filtro EMA ─────────────────────────────────────────────
        ema = ALPHA * raw_angle + (1.0 - ALPHA) * self.smoothed_angle

        # ── 10. Rate limiter ──────────────────────────────────────────
        max_delta = math.radians(MAX_ANGLE_DELTA_DEG)
        delta     = float(np.clip(ema - self.smoothed_angle, -max_delta, max_delta))
        self.smoothed_angle += delta

        self._publish_drive()

    # ------------------------------------------------------------------
    # Control de velocidad y publicación
    # ------------------------------------------------------------------

    def _publish_drive(self):

        angle = self.smoothed_angle
        abs_steer = abs(angle)

        # Distancia frontal (índice central del scan — se recalcula aquí
        # solo para la lógica de velocidad; no requiere otro sub)
        # Usamos el ángulo suavizado como proxy: a más ángulo, más peligro.
        # La lógica de velocidad es idéntica a la del código de referencia.
        if abs_steer < 0.06:
            speed = SPEED_STRAIGHT          # 8.2 m/s
        elif abs_steer < 0.10:
            speed = 7.2
        elif abs_steer < 0.16:
            speed = 5.6
        elif abs_steer < 0.25:
            speed = SPEED_MEDIUM            # 4.8 m/s
        else:
            speed = SPEED_CURVE             # 2.4 m/s

        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = 'base_link'
        msg.drive.steering_angle = float(angle)
        msg.drive.speed          = float(speed)
        self.drive_pub.publish(msg)


    def _stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed          = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)


# ──────────────────────────────────────────────────────────────────────
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
