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

# Parámetros del oponente (más lento que el carro principal)
ALPHA               = 0.30
MAX_ANGLE_DELTA_DEG = 3.0
DEAD_ZONE_DEG       = 2.0
CAR_HALF_WIDTH      = 0.50
FOV_DEG             = 100.0
MAX_STEERING        = 0.42
BUBBLE_RADIUS       = 20
SMOOTHING_WINDOW    = 7

# Velocidades reducidas — este carro es el obstáculo dinámico
SPEED_STRAIGHT      = 5.0    
SPEED_MEDIUM        = 2.0
SPEED_CURVE         = 1.8
SPEED_DANGER        = 0.8

FRONT_DANGER_DIST   = 1.5
FRONT_CAUTION_DIST  = 3.5

MAX_LAPS            = 10
REARM_DISTANCE      = 8.0
FINISH_RADIUS       = 1.5


class OpponentGapFollower(Node):

    def __init__(self):
        super().__init__('opponent_gap_follower')

        # Topics del carro 2 (oponente)
        # En F1Tenth con num_agent=2 el segundo carro usa:
        #   /opp_scan        → LiDAR del oponente
        #   /opp_drive       → comandos del oponente
        #   /opp_racecar/odom → odometría del oponente
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/opp_drive', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/opp_scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/opp_racecar/odom', self.odom_callback, 10)

        self.smoothed_angle = 0.0
        self.front_distance = 999.0

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

        self.get_logger().info("Opponent Gap Follower iniciado.")

    # Odometría y contador de vueltas

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x       = self.x
            self.start_y       = self.y
            self.start_time    = time.time()
            self.last_lap_time = self.start_time
            self.get_logger().info(
                f"[OPP] Punto de salida: x={self.start_x:.2f}  y={self.start_y:.2f}"
            )
            return

        dist = math.hypot(self.x - self.start_x, self.y - self.start_y)

        if not self.armed_lap_counter and dist > REARM_DISTANCE:
            self.armed_lap_counter = True

        if self.armed_lap_counter and dist < FINISH_RADIUS:
            self.lap_count        += 1
            self.armed_lap_counter = False

            now        = time.time()
            lap_time   = now - self.last_lap_time
            total_time = now - self.start_time
            self.last_lap_time = now

            if self.best_lap_time is None or lap_time < self.best_lap_time:
                self.best_lap_time = lap_time

            self.get_logger().info(
                f"[OPP] Vuelta {self.lap_count}/{MAX_LAPS} | "
                f"Tiempo vuelta: {lap_time:.2f} s | "
                f"Mejor vuelta: {self.best_lap_time:.2f} s | "
                f"Tiempo total: {total_time:.2f} s"
            )

            if self.lap_count >= MAX_LAPS:
                self.finished = True
                total = time.time() - self.start_time
                self.get_logger().info(
                    f"[OPP] ¡{MAX_LAPS} vueltas completadas! "
                    f"Mejor vuelta: {self.best_lap_time:.2f} s | "
                    f"Tiempo total: {total:.2f} s"
                )
                self._stop()

    # Utilidades
    @staticmethod
    def _angle_at(scan, idx):
        return scan.angle_min + idx * scan.angle_increment

    @staticmethod
    def _idx_for_angle(scan, angle_rad):
        return int(round(
            (angle_rad - scan.angle_min) / scan.angle_increment
        ))

    # Callback LiDAR

    def scan_callback(self, scan):

        if self.finished:
            self._stop()
            return

        ranges = np.array(scan.ranges, dtype=np.float64)
        ranges = np.where(
            np.isfinite(ranges),
            np.clip(ranges, 0.0, scan.range_max),
            0.0
        )

        # Distancia frontal ±10°
        center_idx   = self._idx_for_angle(scan, 0.0)
        front_window = int(math.radians(10) / scan.angle_increment)
        f_lo = max(0, center_idx - front_window)
        f_hi = min(len(ranges) - 1, center_idx + front_window)
        front_slice  = ranges[f_lo:f_hi + 1]
        front_slice  = front_slice[front_slice > 0.0]
        self.front_distance = float(np.min(front_slice)) \
                              if len(front_slice) > 0 else 999.0

        fov_rad = math.radians(FOV_DEG / 2.0)
        lo = max(2,             self._idx_for_angle(scan, -fov_rad))
        hi = min(len(ranges)-3, self._idx_for_angle(scan,  fov_rad))

        kernel = np.ones(SMOOTHING_WINDOW) / SMOOTHING_WINDOW
        ranges_smooth = ranges.copy()
        pad = SMOOTHING_WINDOW // 2
        conv = np.convolve(ranges[lo-pad:hi+pad+1], kernel, mode='valid')
        ranges_smooth[lo:lo + len(conv)] = conv[:hi - lo + 1]

        closest_idx = int(np.argmin(ranges_smooth[lo:hi+1])) + lo

        obs_dist   = max(ranges_smooth[closest_idx], 0.05)
        half_angle = math.atan2(CAR_HALF_WIDTH, obs_dist)
        bub_radius = int(half_angle / scan.angle_increment)
        bub_radius = max(BUBBLE_RADIUS, min(bub_radius, 180))

        bub_start = max(0,           closest_idx - bub_radius)
        bub_end   = min(len(ranges), closest_idx + bub_radius + 1)
        ranges_smooth[bub_start:bub_end] = 0.0

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

        gap_r      = ranges_smooth[best_start:best_end + 1]
        gap_center = (best_start + best_end) // 2
        farthest   = best_start + int(np.argmax(gap_r))
        best_idx   = int(0.80 * gap_center + 0.20 * farthest)

        raw_angle = self._angle_at(scan, best_idx)
        raw_angle = max(-MAX_STEERING, min(MAX_STEERING, raw_angle))

        if abs(math.degrees(raw_angle)) < DEAD_ZONE_DEG:
            raw_angle = 0.0

        ema = ALPHA * raw_angle + (1.0 - ALPHA) * self.smoothed_angle

        max_delta = math.radians(MAX_ANGLE_DELTA_DEG)
        delta     = float(np.clip(ema - self.smoothed_angle, -max_delta, max_delta))
        self.smoothed_angle += delta

        self._publish_drive()

    # Velocidad y publicación

    def _publish_drive(self):

        angle     = self.smoothed_angle
        abs_steer = abs(angle)
        front     = self.front_distance

        if abs_steer < 0.06:
            speed = SPEED_STRAIGHT
        elif abs_steer < 0.10:
            speed = 3.2
        elif abs_steer < 0.16:
            speed = 2.8
        elif abs_steer < 0.25:
            speed = SPEED_MEDIUM
        else:
            speed = SPEED_CURVE

        if front < FRONT_DANGER_DIST:
            speed = SPEED_DANGER
        elif front < FRONT_CAUTION_DIST:
            t     = (front - FRONT_DANGER_DIST) / (FRONT_CAUTION_DIST - FRONT_DANGER_DIST)
            speed = SPEED_DANGER + (speed - SPEED_DANGER) * t

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

def main(args=None):
    rclpy.init(args=args)
    node = OpponentGapFollower()

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
