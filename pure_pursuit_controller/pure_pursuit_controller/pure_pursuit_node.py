#!/usr/bin/env python3
"""Nodo de control Pure Pursuit.

Arquitectura (segun el paso 2):
  - Publisher:  /cmd_vel  (geometry_msgs/msg/Twist)
  - Subscriber: /odom     (nav_msgs/msg/Odometry)
  - Timer callback a frecuencia fija (10-20 Hz) para desacoplar el control
    de la frecuencia real de /odom.

Formulas:
  yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
  P     = k * v            (look-ahead distance, proporcional a la velocidad)
  delta = atan2(2*L*sin(psi), P)     (psi = error de rumbo al punto objetivo)
  theta_dot = (v / L) * tan(delta)   (modelo de bicicleta cinematico)

  cmd.linear.x  = v
  cmd.angular.z = theta_dot
"""
import csv
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def load_waypoints(path):
    """Lee un CSV con columnas x, y (el que genera path_recorder.py)."""
    pts = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"])))
    if len(pts) < 2:
        raise ValueError(f"'{path}' tiene menos de 2 puntos, no hay trayectoria que seguir.")
    return pts


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def angle_diff(a, b):
    """a - b normalizado a [-pi, pi]."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        # --- Parametros ---
        self.declare_parameter("waypoints_file", "waypoints.csv")
        self.declare_parameter("wheelbase", 2.7)           # [m] L, del plugin AckermannSteering (model.sdf)
        self.declare_parameter("steering_limit", 0.5)       # [rad] limite mecanico del volante (mismo SDF)
        self.declare_parameter("v_ref", 0.605)             # [m/s] velocidad de crucero comandada
        self.declare_parameter("lookahead_gain", 0.5)      # k en  P = k * v
        self.declare_parameter("min_lookahead", 0.5)       # [m] piso de P para que no sea 0
        self.declare_parameter("goal_tolerance", 0.3)      # [m] radio para considerar meta alcanzada
        self.declare_parameter("rate_hz", 20.0)             # frecuencia del timer (10-20 Hz)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        p = self.get_parameter
        self.L = p("wheelbase").value
        self.steering_limit = p("steering_limit").value
        if self.L <= 0.0:
            self.get_logger().error(
                "Parametro 'wheelbase' no configurado (<=0). El controlador no "
                "funcionara correctamente hasta que lo fijes con "
                "-p wheelbase:=<metros>.")

        self.path = load_waypoints(p("waypoints_file").value)
        self.v_ref = p("v_ref").value
        self.k = p("lookahead_gain").value
        self.min_lookahead = p("min_lookahead").value
        self.goal_tolerance = p("goal_tolerance").value

        self.pose = None            # (x, y, yaw) mas reciente de /odom
        self.last_stamp_ns = -1     # para descartar mensajes de /odom fuera de orden
        self.closest_idx = 0
        self.done = False

        self.create_subscription(Odometry, p("odom_topic").value, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, p("cmd_topic").value, 10)
        self.create_timer(1.0 / p("rate_hz").value, self.control_loop)

        self.get_logger().info(
            f"{len(self.path)} waypoints cargados | L={self.L} m | "
            f"steering_limit={self.steering_limit} rad | v_ref={self.v_ref} m/s | "
            f"k={self.k} | min_lookahead={self.min_lookahead} m")

    # ---------- Suscripcion a /odom: estado y conversiones ----------
    def odom_callback(self, msg: Odometry):
        # /odom puede llegar desordenado; se descartan mensajes con stamp menor al ultimo.
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp < self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.pose = (x, y, yaw)

    # ---------- Utilidades de la trayectoria ----------
    def find_closest_index(self, x, y, window=40):
        """Busca el punto mas cercano solo hacia adelante del ultimo indice conocido,
        para no saltar de tramo si la ruta se acerca a si misma."""
        start = self.closest_idx
        end = min(len(self.path), start + window)
        best_i, best_d = start, float("inf")
        for i in range(start, end):
            px, py = self.path[i]
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def find_target_index(self, start_idx, x, y, lookahead):
        """Camina hacia adelante desde start_idx acumulando distancia de arco
        hasta alcanzar 'lookahead'. Devuelve el indice del punto objetivo."""
        acc = 0.0
        px, py = self.path[start_idx]
        i = start_idx
        while i < len(self.path) - 1:
            nx, ny = self.path[i + 1]
            acc += math.hypot(nx - px, ny - py)
            if acc >= lookahead:
                return i + 1
            px, py = nx, ny
            i += 1
        return len(self.path) - 1  # no alcanzo el lookahead: usa el ultimo punto

    # ---------- Loop de control (timer, frecuencia fija) ----------
    def control_loop(self):
        if self.pose is None:
            return  # aun no llega odometria
        if self.done:
            self.publish_cmd(0.0, 0.0)
            return

        x, y, yaw = self.pose

        # Meta alcanzada
        gx, gy = self.path[-1]
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal < self.goal_tolerance and self.closest_idx >= len(self.path) - 3:
            self.done = True
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(f"Meta alcanzada (a {dist_goal:.2f} m). Auto detenido.")
            return

        v = self.v_ref

        # P = k * v  (look-ahead distance), con piso minimo
        P = max(self.k * abs(v), self.min_lookahead)

        self.closest_idx = self.find_closest_index(x, y)
        target_idx = self.find_target_index(self.closest_idx, x, y, P)
        tx, ty = self.path[target_idx]

        # psi: error de rumbo hacia el punto objetivo
        heading_to_target = math.atan2(ty - y, tx - x)
        psi = angle_diff(heading_to_target, yaw)

        # delta = atan2(2L*sin(psi), P)
        if self.L > 0.0:
            delta = math.atan2(2.0 * self.L * math.sin(psi), P)
            # Saturamos al limite mecanico del volante (steering_limit del SDF).
            # El plugin AckermannSteering tambien satura internamente, pero lo
            # hacemos aqui tambien para que nuestros logs reflejen lo real.
            delta = max(-self.steering_limit, min(self.steering_limit, delta))
            theta_dot = (v / self.L) * math.tan(delta)
        else:
            delta = 0.0
            theta_dot = 0.0  # sin wheelbase valido no se puede calcular

        self.publish_cmd(v, theta_dot)

        self.get_logger().info(
            f"idx={target_idx}/{len(self.path)-1} dist_meta={dist_goal:.2f} m "
            f"P={P:.2f} psi={math.degrees(psi):+.1f} delta={math.degrees(delta):+.1f} "
            f"w={theta_dot:+.3f}",
            throttle_duration_sec=1.0)

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()