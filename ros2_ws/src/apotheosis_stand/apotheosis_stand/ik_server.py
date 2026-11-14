#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IK-сервер руки: цель PoseStamped -> суставы (ЛР_06, задания 3-5).

Кто публикует цель:
- в Foxglove: инструмент «Publish pose» на 3D-панели (топик /goal_pose);
- из ноутбука: мост (см. bridge_server.py) пересылает цель из панели;
- из консоли: ros2 topic pub /goal_pose ...

Солвер - ньютоновский IK с pinv, тот же, что в ноутбуке (з.3);
на каждой итерации следим cond(J) (з.5). Результат -> /joint_states,
что и рисует руку в rviz/Foxglove. Остаточная невязка - в /stand/ik_residual.
У недостижимой цели солвер честно упирается в границу рабочей зоны (з.6).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from .stand_math import arm_fk, ik3d_newton

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


class IKServer(Node):

    def __init__(self):
        super().__init__('ik_server')
        self.declare_parameter('seed_q', [0.5, -0.5, 0.9, 0.6])
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.res_pub = self.create_publisher(Float64, '/stand/ik_residual', 10)
        self.sub = self.create_subscription(PoseStamped, '/goal_pose',
                                            self.on_goal, 10)
        seed = [float(v) for v in self.get_parameter('seed_q').value]
        self.q = np.array(seed, float)
        self.publish_joint_state(self.q)
        self.get_logger().info(
            'IK-сервер: цель - /goal_pose (PoseStamped), суставы - /joint_states')

    def on_goal(self, msg):
        target = np.array([msg.pose.position.x, msg.pose.position.y,
                           msg.pose.position.z])
        q, hist, conds = ik3d_newton(target, self.q)
        self.q = q
        self.publish_joint_state(q)
        self.res_pub.publish(Float64(data=float(hist[-1])))
        state = ('сошёлся' if hist[-1] < 1e-9
                 else 'цель вне рабочей зоны - остановился у границы')
        self.get_logger().info(
            'цель %s: %s за %d итераций, cond(J) %.1f..%.1f, невязка %.2e'
            % (np.round(target, 3).tolist(), state, len(hist) - 1,
               conds.min(), conds.max(), hist[-1]))

    def publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [float(v) for v in q]
        self.js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
