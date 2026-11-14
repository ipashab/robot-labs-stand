#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Мост «ноутбук -> ROS» (вариант 2): TCP-сервер на порту 8977.

Ноутбук «Апофеоз» (ipywidgets-панель на хосте, без ROS) отправляет сюда
JSON-сообщения:

    {"q":  [q1, q2, q3, q4]}                   - суставы (ручной режим)
    {"goal": [x, y, z], "seed": [q1..q4]}       - цель для IK
    {"t_cam": 0.35}                            - перелёт камеры (slerp)

Сервер превращает их в ROS-публикации:
- q        -> /joint_states (рука в rviz/Foxglove повторяет панель);
- goal     -> /goal_pose (решает ik_server);
- t_cam    -> параметр t узла camera_flight (перелёт замирает на t).

Единственная зависимость ноутбука - модуль socket стандартной библиотеки:
никакого ROS на хосте macOS не нужно.
"""
import json
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from .stand_math import arm_fk

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
PORT = 8977


class BridgeServer(Node):

    def __init__(self):
        super().__init__('nb_bridge')
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # перелёт камеры по команде из ноутбука (см. camera_flight.py)
        self.cam_pub = self.create_publisher(Float64, '/stand/cam_t', 10)
        self.declare_parameter('port', PORT)
        port = int(self.get_parameter('port').value)

        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(('0.0.0.0', port))
        self.srv.listen(4)
        self.srv.settimeout(0.5)
        self.get_logger().info('мост: слушаю TCP :%d (открыт наружу '
                               'в docker-compose)' % port)

        self._alive = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    # ---- приём ----
    def loop(self):
        while self._alive:
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            with conn:
                try:
                    data = conn.recv(4096)
                    reply = self.on_message(data)
                except Exception as e:              # noqa: BLE001 - на связи
                    reply = {'err': str(e)}
                    self.get_logger().warn('мост: %s' % e)
                conn.sendall(json.dumps(reply).encode())

    def on_message(self, data):
        msg = json.loads(data.decode())
        if 'ping' in msg:
            return {'ok': True, 'node': 'nb_bridge'}
        if 'q' in msg:
            q = [float(v) for v in msg['q']]
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = JOINTS[:len(q)]
            m.position = q
            self.js_pub.publish(m)
            ee = arm_fk(q)[0][-1]
            return {'ok': True, 'ee': ee.tolist()}
        if 'goal' in msg:
            g = PoseStamped()
            g.header.stamp = self.get_clock().now().to_msg()
            g.header.frame_id = 'world'
            g.pose.position.x, g.pose.position.y, g.pose.position.z \
                = (float(v) for v in msg['goal'])
            self.goal_pub.publish(g)
            return {'ok': True}
        if 't_cam' in msg:
            self.cam_pub.publish(Float64(data=float(msg['t_cam'])))
            return {'ok': True}
        return {'err': 'неизвестное сообщение: %s' % sorted(msg)}

    def destroy_node(self):
        self._alive = False
        self.srv.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
