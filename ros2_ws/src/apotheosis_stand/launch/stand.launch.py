# -*- coding: utf-8 -*-
"""Полный запуск стенда: рука + перелёт камеры + трекинг маркера +
IK-сервер + мост из ноутбука + rosbridge (для Foxglove).

Компоненты и их лабы:
- robot_state_publisher + URDF      - ЛР_01 (кадры), ЛР_05 (DH-геометрия)
- camera_flight (slerp)             - ЛР_02 з.5
- marker_tracker (RANSAC + H + поза) - ЛР_04 з.1-3, 5; ЛР_03 з.1, 3-4
- static tf world->marker           - ЛР_01 з.4 (дерево кадров)
- ik_server (Ньютон + pinv)         - ЛР_06 з.3-5
- bridge_server (TCP 8977)           - мост из ноутбука (вариант 2)
- rosbridge (порт 9090)             - подключение Foxglove в браузере
"""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf = PathJoinSubstitution([FindPackageShare('apotheosis_stand'),
                                 'urdf', 'stand_arm.urdf'])
    robot_desc = ParameterValue(Command([FindExecutable(name='cat'), ' ', urdf]),
                                value_type=str)

    return LaunchDescription([
        # --- рука: кадры из URDF (ЛР_01 + ЛР_05) ---
        # суставы публикует ik_server (стартовая поза + каждый решение),
        # а в режиме моста - наоборот, панель ноутбука. Никакого
        # joint_state_publisher: два клона /joint_states дрались бы.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}]),
        # --- перелёт камеры по slerp (ЛР_02) ---
        Node(package='apotheosis_stand', executable='camera_flight',
             name='camera_flight', parameters=[{'anim': True}]),
        # --- трекинг маркера: RANSAC + гомография (ЛР_04, ЛР_03) ---
        Node(package='apotheosis_stand', executable='marker_tracker',
             name='marker_tracker'),
        # --- IK-сервер: /goal_pose -> /joint_states (ЛР_06) ---
        Node(package='apotheosis_stand', executable='ik_server',
             name='ik_server'),
        # --- мост из ноутбука (вариант 2) ---
        Node(package='apotheosis_stand', executable='bridge_server',
             name='nb_bridge'),
        # --- статический кадр маркера: world -> marker (ЛР_01 з.4) ---
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='marker_static_tf',
             arguments=['0.4', '0.4', '0.0', '0', '0', '0',
                        'world', 'marker']),
        # --- rosbridge: Foxglove подключается на ws://localhost:9090 ---
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge'),
    ])
