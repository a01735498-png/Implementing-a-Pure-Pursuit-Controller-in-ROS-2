#!/usr/bin/env python3
"""Nodo que graba la trayectoria de referencia para Pure Pursuit.

Se suscribe a /odom, guarda un punto (x, y) solo si se alejó más de
`threshold` metros del último punto guardado, y al cerrarse (Ctrl+C)
escribe todos los puntos acumulados en waypoints.csv.
"""
import csv
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PathRecorder(Node):
    def __init__(self):
        super().__init__("path_recorder")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("output_file", "waypoints.csv")
        self.declare_parameter("threshold", 0.2)  # [m] distancia mínima entre puntos guardados

        self.output_file = self.get_parameter("output_file").value
        self.threshold = self.get_parameter("threshold").value

        self.points = []       # lista de (x, y) guardados
        self.last_point = None  # último punto guardado, para medir el desplazamiento

        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(
            f"Grabando trayectoria desde '{odom_topic}' "
            f"(umbral={self.threshold} m). Ctrl+C para detener y guardar en "
            f"'{self.output_file}'.")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_point is None:
            self._add_point(x, y)
            return

        dx = x - self.last_point[0]
        dy = y - self.last_point[1]
        dist = math.hypot(dx, dy)

        if dist >= self.threshold:
            self._add_point(x, y)

    def _add_point(self, x, y):
        self.points.append((x, y))
        self.last_point = (x, y)
        self.get_logger().info(
            f"Punto #{len(self.points)} guardado: x={x:.3f}, y={y:.3f}",
            throttle_duration_sec=0.0)

    def save_to_csv(self):
        """Escribe los puntos acumulados en el CSV. Se llama al cerrar el nodo."""
        if not self.points:
            self.get_logger().warn("No se guardó ningún punto, no se escribe archivo.")
            return

        with open(self.output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            writer.writerows(self.points)

        self.get_logger().info(
            f"Guardados {len(self.points)} puntos en '{self.output_file}'.")


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # "on_shutdown hook" para volcar los puntos aunque se corte con Ctrl+C
        node.save_to_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()