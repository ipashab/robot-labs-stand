#!/usr/bin/env bash
# Полный живой стенд: launch поднимает robot_state_publisher, перелёт
# камеры (slerp), трекинг маркера (RANSAC + гомография), IK-сервер,
# мост из ноутбука, статический кадр маркера и rosbridge.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.sh
source /stand/ros2_ws/install/setup.bash
exec ros2 launch apotheosis_stand stand.launch.py
