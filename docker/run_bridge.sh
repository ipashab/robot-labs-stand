#!/usr/bin/env bash
# Вариант «мост»: рука слушается панель ноутбука (TCP 8977),
# плюс rosbridge, чтобы Foxglove показывал ту же сцену.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.sh
source /stand/ros2_ws/install/setup.bash

# кадры руки из URDF; суставы придут от ноутбука (bridge_server)
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description="$(cat /stand/ros2_ws/src/apotheosis_stand/urdf/stand_arm.urdf)" &
ros2 run apotheosis_stand camera_flight --ros-args -p anim:=true &
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 &

# мост -前台: слушает TCP 8977 и переводит в топики
ros2 run apotheosis_stand bridge_server
