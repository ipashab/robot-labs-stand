#!/usr/bin/env bash
# Вариант «мост»: рука слушается панель ноутбука (TCP 8977), камера
# летает сама, rosbridge открыт для Foxglove. Трекер маркера и IK-сервер
# не запускаются (full:=false) - суставы задаёт только панель.
# URDF передаётся через launch (ParameterValue): через CLI -p Jazzy
# не парсит многострочный XML как YAML.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.sh
source /stand/ros2_ws/install/setup.bash
exec ros2 launch apotheosis_stand stand.launch.py full:=false
