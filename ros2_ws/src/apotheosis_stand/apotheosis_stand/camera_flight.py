#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Перелёт камеры world->camera по slerp (ЛР_02, задание 5) в живом TF.

Публикует TF world->camera по геодезической между видами A и B:
параметр anim переключает качание туда-обратно, period задаёт период.
Тот же полёт, что на ползунке «камера: перелёт A->B» в ноутбуке.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64

from .stand_math import cam_pose, R_to_quat, quat_to_params


class CameraFlight(Node):

    def __init__(self):
        super().__init__('camera_flight')
        self.declare_parameter('anim', True)          # качание A<->B
        self.declare_parameter('period', 20.0)        # сек на полный цикл
        self.tfl = TransformBroadcaster(self)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.tick)   # 20 Гц
        # мост из ноутбука может задать t напрямую (панель -> перелёт камеры)
        self.bridge_t = None
        self.sub = self.create_subscription(Float64, '/stand/cam_t',
                                            self.on_cam_t, 10)
        self.get_logger().info('перелёт камеры: slerp между видами A и B')

    def on_cam_t(self, msg):
        self.bridge_t = float(msg.data)

    def tick(self):
        anim = self.get_parameter('anim').value
        period = float(self.get_parameter('period').value)
        now = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if self.bridge_t is not None:
            # ноутбук задал перелёт: слушаем его, пока пишут (масштаб 0..1
            # из панели карты прямо на slerp; выход за границы - клип)
            t = float(np.clip(self.bridge_t, 0.0, 1.0))
        elif anim:
            # ping-pong по треугольной волне: без рывка на концах
            ph = (now % period) / period
            t = 1 - abs(2 * ph - 1)
        else:
            t = 0.0
        R, t_vec, _ = cam_pose(t)
        qwxyz = R_to_quat(R)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'camera'
        # порядок полей сообщения: (x, y, z, w) - как в ROS
        xyzw = quat_to_params(qwxyz)
        msg.transform.rotation.x = float(xyzw[0])
        msg.transform.rotation.y = float(xyzw[1])
        msg.transform.rotation.z = float(xyzw[2])
        msg.transform.rotation.w = float(xyzw[3])
        # camera frame: точка кадра = R @ X_world + t_vec, а TF хранит
        # позу камеры в мире: p_cam_world = -R^T t
        p_world = -R.T @ t_vec
        msg.transform.translation.x = float(p_world[0])
        msg.transform.translation.y = float(p_world[1])
        msg.transform.translation.z = float(p_world[2])
        self.tfl.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
