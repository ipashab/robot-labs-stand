#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Трекинг маркера на синтетическом кадре: гомография + RANSAC -> TF
(ЛР_04, задания 1-3 и 5): полный конвейер «пиксели -> поза» в ROS.

Каждый тик:
1) строит мировые углы маркера и проецирует их текущей позой летящей
   камеры (та же K и дисторсия, что в ноутбуке);
2) сыпет шум и выбросы на наблюдения;
3) RANSAC-гомография (минимальная четвёрка + нормировка Хартли);
4) поза из H с SVD-проекцией в SO(3);
5) публикует TF camera->marker_est, кадр (Image, rgb8) для Foxglove,
   ошибку оценки позы и стерео-глубину (d = fB/Z, ЛР_03 задание 4).

Кадр честно синтетический: стенд воспроизводит математику
лабораторных, а не съёмку реальной камеры.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from .stand_math import (BASE_STEREO, F_PX, K, cam_pose, distort_px,
                         marker_grid, pose_from_homography, project,
                         quat_to_params, ransac_homography, R_to_quat)


class MarkerTracker(Node):

    def __init__(self):
        super().__init__('marker_tracker')
        self.declare_parameter('rate', 5.0)            # оценок позы в секунду
        self.declare_parameter('noise_px', 0.2)       # шум наблюдений
        self.declare_parameter('outliers_frac', 0.2)   # доля мусора
        self.declare_parameter('seed', 7)
        self.declare_parameter('distortion', False)
        self.declare_parameter('publish_image', True)
        self.tfl = TransformBroadcaster(self)
        self.img_pub = self.create_publisher(Image, '/stand/camera_frame', 2)
        self.err_pub = self.create_publisher(Float64, '/stand/marker_pose_err',
                                             10)
        self.depth_pub = self.create_publisher(Float64, '/stand/stereo_depth',
                                               10)
        rate = float(self.get_parameter('rate').value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.t = 0.0
        self.grid = marker_grid()          # 81 угол маркера в мире
        g = np.linspace(-0.10, 0.10, 9)
        self.M2 = np.array([[a, b] for a in g for b in g])   # план маркера
        self.get_logger().info('трекер: 81 угол, RANSAC + нормировка Хартли')

    def tick(self):
        # 1) наблюдения: проекция летящей камерой (+ опц. дисторсия)
        R, t_vec, _ = cam_pose(self.t)
        self.t = (self.t + 0.02) % 1.0
        obs, z = project(K, R, t_vec, self.grid)
        if self.get_parameter('distortion').value:
            obs = distort_px(obs)

        # 2) шум и выбросы
        r = np.random.default_rng(int(self.get_parameter('seed').value) + 1)
        s = float(self.get_parameter('noise_px').value)
        obs = obs + r.normal(0, max(s, 1e-9), obs.shape)
        inl_true = np.ones(len(obs), bool)
        k = int(len(obs) * float(self.get_parameter('outliers_frac').value))
        if k:
            bad = r.choice(len(obs), k, replace=False)
            obs[bad] += r.uniform(-150, 150, (k, 2))
            inl_true[bad] = False

        # 3) RANSAC-гомография (один прогон: и H, и inliers)
        try:
            H, inl = ransac_homography(self.M2, obs, seed=42)
        except np.linalg.LinAlgError:
            self.get_logger().warn('RANSAC не нашёл гомографию',
                                   throttle_duration_sec=2.0)
            return

        # 4) поза из H (та же SVD-проекция в SO(3), что в ЛР_01 задание 7):
        # точки маркера -> кадр камеры; поза маркера в кадре = (R_est, t_est)
        R_est, t_est = pose_from_homography(H, K)
        err = float(np.linalg.norm(
            t_est - (R @ np.array([0.4, 0.4, 0.0]) + t_vec)))
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.child_frame_id = 'marker_est'
        xyzw = quat_to_params(R_to_quat(R_est))
        msg.transform.rotation.x = float(xyzw[0])
        msg.transform.rotation.y = float(xyzw[1])
        msg.transform.rotation.z = float(xyzw[2])
        msg.transform.rotation.w = float(xyzw[3])
        msg.transform.translation.x = float(t_est[0])
        msg.transform.translation.y = float(t_est[1])
        msg.transform.translation.z = float(t_est[2])
        self.tfl.sendTransform(msg)
        self.err_pub.publish(Float64(data=err))

        # 5) стерео-глубина до центра маркера: d = fB/Z -> Z = fB/d (ЛР_03 з.4)
        z_marker = float(z[len(z) // 2])          # центр сетки ~ центр маркера
        disp = F_PX * BASE_STEREO / z_marker
        z_back = F_PX * BASE_STEREO / disp
        self.depth_pub.publish(Float64(data=z_back))

        # 6) кадр в топик: наблюдения и инлаеры поверх
        if self.get_parameter('publish_image').value:
            self.publish_frame(obs, inl)

    def publish_frame(self, obs, inl):
        img = np.full((720, 1280, 3), 24, np.uint8)
        ix = np.clip(obs[:, 0].astype(int), 4, 1275)
        iy = np.clip(obs[:, 1].astype(int), 4, 715)
        # наблюдения пятнами 11x11 px: одиночные пиксели в просмотрщике не видно
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                img[iy + dy, ix + dx] = (60, 150, 255)   # все наблюдения (синий)
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                img[iy[inl] + dy, ix[inl] + dx] = (255, 190, 70)  # инлаеры (оранж.)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = 720, 1280
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = 1280 * 3
        msg.data = np.ascontiguousarray(img).tobytes()
        self.img_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
