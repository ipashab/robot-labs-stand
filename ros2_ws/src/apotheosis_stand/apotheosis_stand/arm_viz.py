#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Визуализатор руки: суставы и звенья как MarkerArray (ЛР_05 + ЛР_06).

Цилиндры и сферы строятся той же FK, что и оси TF (stand_math.arm_fk):
та же геометрия, что у URDF и панели ноутбука. Старые клиенты
(Foxglove v1.87 / Trillium) рендерят MarkerArray надёжно, а полную
URDF-модель - не всегда, так что это основной запасной вид.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from .stand_math import arm_fk, D_BASE

# цвета как в URDF стенда
STEEL = (0.25, 0.28, 0.33, 1.0)
ACCENT = (0.88, 0.54, 0.18, 1.0)


def z_to_quat(d):
    """Кватернион (x,y,z,w) поворота оси +z в направление d."""
    d = np.asarray(d, float)
    n = np.linalg.norm(d)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    d = d / n
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, d))
    if c < -1 + 1e-9:                    # противоположно: поворот на pi вокруг x
        return (1.0, 0.0, 0.0, 0.0)
    ax = np.cross(z, d)
    s = float(np.linalg.norm(ax))
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    ax = ax / s
    half = np.arccos(np.sqrt(max(0.0, (1 + c) / 2)))
    sin_half = np.sin(half)
    return (float(ax[0] * sin_half), float(ax[1] * sin_half),
            float(ax[2] * sin_half), float(np.cos(half)))


class ArmViz(Node):

    def __init__(self):
        super().__init__('arm_viz')
        self.js_sub = self.create_subscription(JointState, '/joint_states',
                                               self.on_joints, 10)
        self.pub = self.create_publisher(MarkerArray, '/stand/arm_markers',
                                         QoSProfile(depth=5))
        self.q = np.array([0.5, -0.5, 0.9, 0.6])
        self.timer = self.create_timer(0.05, self.tick)   # 20 Гц
        self.get_logger().info('визуализатор руки: MarkerArray на '
                               '/stand/arm_markers')

    def on_joints(self, msg):
        if len(msg.position) == 4:
            self.q = np.array(msg.position, float)

    def _mv(self, mid, eid, ax_, ay_, az_, quat, sx, sy, sz, color):
        m = Marker()
        m.header.frame_id = 'world'
        m.ns, m.id = 'arm', eid
        m.action = Marker.ADD
        m.pose.position.x = float(ax_)
        m.pose.position.y = float(ay_)
        m.pose.position.z = float(az_)
        m.pose.orientation.x, m.pose.orientation.y = quat[0], quat[1]
        m.pose.orientation.z, m.pose.orientation.w = quat[2], quat[3]
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def cylinder(self, eid, a, b, radius, color):
        """Цилиндр между точками a и b: середина + ориентация оси z вдоль."""
        d = b - a
        length = float(np.linalg.norm(d))
        mid = (a + b) / 2
        return self._mv(mid, eid, mid[0], mid[1], mid[2], z_to_quat(d),
                        2 * radius, 2 * radius, length, color)

    def sphere(self, eid, p, radius, color):
        return self._mv(p, eid, p[0], p[1], p[2], (0.0, 0.0, 0.0, 1.0),
                        2 * radius, 2 * radius, 2 * radius, color)

    def tick(self):
        points = arm_fk(self.q)[0]
        _, p_base, p_elbow, p_wrist, p_ee = points
        floor = np.zeros(3)
        base_top = np.array([0.0, 0.0, D_BASE])
        m = MarkerArray()
        # стойка: цилиндр от пола до базы
        m.markers.append(self.cylinder(0, floor, base_top, 0.035, STEEL))
        # суставы: сферы в плечо/локоть/запястье
        m.markers.append(self.sphere(1, p_base, 0.045, ACCENT))
        m.markers.append(self.sphere(2, p_elbow, 0.030, STEEL))
        m.markers.append(self.sphere(3, p_wrist, 0.024, STEEL))
        # звенья: цилиндры между суставами (длины 0.45/0.35/0.15, как URDF)
        m.markers.append(self.cylinder(4, p_base, p_elbow, 0.028, STEEL))
        m.markers.append(self.cylinder(5, p_elbow, p_wrist, 0.022, STEEL))
        m.markers.append(self.cylinder(6, p_wrist, p_ee, 0.016, STEEL))
        # схват: акцентный шарик
        m.markers.append(self.sphere(7, p_ee, 0.014, ACCENT))
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ArmViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
