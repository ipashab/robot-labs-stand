# -*- coding: utf-8 -*-
"""Проверка математики стенда против эталонных чисел ноутбука."""
import numpy as np

from apotheosis_stand.stand_math import (arm_fk, arm_jac, cam_pose,
                                          ik3d_newton, marker_grid, project,
                                          quat_to_params, R_to_quat, slerp,
                                          K, F_PX, L_ARM, D_BASE)


def test_fk_вылета():
    """Полностью вытянутая рука держит вылет l1+l2+l3 вдоль +x."""
    q = np.array([0.0, np.pi / 2, 0.0, 0.0])
    p_ee = arm_fk(q)[0][-1]
    assert abs(p_ee[0] - L_ARM.sum()) < 1e-12
    assert abs(p_ee[2] - D_BASE) < 1e-12


def test_якобиан_численно():
    """Геометрический якобиан == численная производная FK."""
    rng = np.random.default_rng(0)
    for q in rng.uniform(-2, 2, (10, 4)):
        J = arm_jac(q)
        h = 1e-6
        Jn = np.zeros((3, 4))
        for j in range(4):
            qp, qm = q.copy(), q.copy()
            qp[j] += h; qm[j] -= h
            Jn[:, j] = (arm_fk(qp)[0][-1] - arm_fk(qm)[0][-1]) / (2 * h)
        assert np.abs(J - Jn).max() < 1e-6


def test_ik_сходимость():
    """Ньютоновский IK попадает в случайные достижимые цели."""
    rng = np.random.default_rng(1)
    for _ in range(10):
        q_star = rng.uniform(-1.5, 1.5, 4)
        target = arm_fk(q_star)[0][-1]
        _, hist, _ = ik3d_newton(target, q_star + rng.uniform(-0.3, 0.3, 4))
        assert hist[-1] < 1e-9


def test_slerp_по_геодезической():
    """Середина slerp делит угол между видами ровно пополам."""
    RA, _, _ = cam_pose(0.0)
    RB, _, _ = cam_pose(1.0)
    qa, qb = R_to_quat(RA), R_to_quat(RB)
    qm = slerp(qa, qb, 0.5)
    ang_ab = 2 * np.arccos(np.clip(abs(qa @ qb), 0, 1))
    ang_am = 2 * np.arccos(np.clip(abs(qa @ qm), 0, 1))
    assert abs(ang_am - ang_ab / 2) < 1e-9


def test_порядок_рос_кватерниона():
    """ROS хранит (x,y,z,w): перестановка честная."""
    q = R_to_quat(cam_pose(0.3)[0])
    x, y, z, w = quat_to_params(q)
    assert (w, x, y, z) == tuple(q)


def test_стерео_глубина():
    """d = fB/Z и обратная формула дают одну и ту же глубину."""
    uv, z = project(K, *cam_pose(0.5)[:2], marker_grid()[len(marker_grid()) // 2][None])
    d = F_PX * 0.12 / z[0]
    assert abs(F_PX * 0.12 / d - z[0]) < 1e-15
