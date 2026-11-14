#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Перерепубликатор URDF на /robot_description (для просмотрщиков).

robot_state_publisher отдаёт URDF с durability=TRANSIENT_LOCAL, а rosbridge
подписывается с volatile QoS - DDS их не спарит, и клиенты (Trillium/Foxglove)
модель не получают. Здесь URDF публикуется обычным потоком 1 Гц -
такой подписчик получает его максимум через секунду.
"""
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RATE_HZ = 1.0


class URDFRepublisher(Node):

    def __init__(self):
        super().__init__('urdf_republisher')
        path = get_package_share_directory('apotheosis_stand') \
            + '/urdf/stand_arm.urdf'
        self.urdf = open(path).read()
        self.pub = self.create_publisher(String, '/robot_description', 1)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info('URDF руки перепубликуется на /robot_description '
                               '(1 Гц)')


    def tick(self):
        self.pub.publish(String(data=self.urdf))


def main(args=None):
    rclpy.init(args=args)
    node = URDFRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
