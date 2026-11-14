# -*- coding: utf-8 -*-
"""Общая математика ROS-стенда: та же, что в ноутбуке «Апофеоз».

Функции перенесены 1:1 из ноутбучного ядра, чтобы числа в ROS
совпадали с числами панели до машинной точности.
"""
import numpy as np

# ---------- камера (ЛР_03) ----------
F_PX = 800.0
CX, CY = 640.0, 360.0
W_IMG, H_IMG = 1280, 720
K = np.array([[F_PX, 0.0, CX], [0.0, F_PX, CY], [0.0, 0.0, 1.0]])
BASE_STEREO = 0.12

# ---------- сцена ----------
MARKER_C = np.array([0.4, 0.4, 0.0])
MARKER_H = 0.10
EYE_A = np.array([1.4, -1.6, 1.3])
EYE_B = np.array([-1.8, -0.8, 1.9])

# ---------- рука (сводная сцена, ЛР_05/06) ----------
D_BASE = 0.30
L_ARM = np.array([0.45, 0.35, 0.15])
REACH = float(np.sum(L_ARM))


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def R_to_quat(R):
    """Матрица -> кватернион (w, x, y, z)."""
    w = np.sqrt(max(0.0, 1.0 + np.trace(R))) / 2.0
    if w > 1e-6:
        x = (R[2, 1] - R[1, 2]) / (4.0 * w)
        y = (R[0, 2] - R[2, 0]) / (4.0 * w)
        z = (R[1, 0] - R[0, 1]) / (4.0 * w)
    else:                   # угол ~ pi: другой след
        i = np.argmax(np.diag(R))
        j, k2 = (i + 1) % 3, (i + 2) % 3
        q = np.zeros(4)
        q[1 + i] = np.sqrt(max(0.0, 1 + 2 * R[i, i] - np.trace(R))) / 2.0
        q[1 + j] = (R[j, i] + R[i, j]) / (4 * q[1 + i])
        q[1 + k2] = (R[k2, i] + R[i, k2]) / (4 * q[1 + i])
        return q
    return np.array([w, x, y, z])


def slerp(q0, q1, t):
    """Кратчайшая дуга на S^3 (ЛР_02, задание 5)."""
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    if q0 @ q1 < 0:
        q1 = -q1                      # кратчайшая дуга (двойное накрытие)
    d = np.clip(q0 @ q1, -1.0, 1.0)
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    return (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)


def quat_to_params(q):
    """Кватернион (w,x,y,z) -> (x,y,z,w): порядок полей ROS."""
    w, x, y, z = q
    return np.array([x, y, z, w])


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Кадр камеры: (R, t), 3D-точка кадра = R @ X_world + t (ЛР_03)."""
    eye = np.asarray(eye, float)
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(np.asarray(up, float), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    return R, (-R @ eye)


def project(Km, R, t, X):
    """Мировые точки -> пиксели (ЛР_03, задание 1)."""
    X = np.atleast_2d(np.asarray(X, float))
    Xc = X @ R.T + np.asarray(t, float)
    uvw = Xc @ np.asarray(Km).T
    return uvw[:, :2] / uvw[:, 2:3], Xc[:, 2]


def distort_px(uv, k1=-0.28, k2=0.09, p1=1e-3, p2=-8e-4):
    """Радиальная + тангенциальная дисторсия в пикселях (ЛР_03, задание 3)."""
    uv = np.atleast_2d(np.asarray(uv, float))
    x = (uv[:, 0] - CX) / F_PX
    y = (uv[:, 1] - CY) / F_PX
    r2 = x * x + y * y
    rad = 1 + k1 * r2 + k2 * r2 * r2
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.column_stack([CX + F_PX * xd, CY + F_PX * yd])


def normalize_pts(p):
    """Нормировка Хартли: centred + sqrt(2)/mean_dist (ЛР_04, задание 2)."""
    p = np.atleast_2d(np.asarray(p, float))
    c = p.mean(axis=0)
    s = np.sqrt(2) / np.sqrt(((p - c) ** 2).sum(axis=1)).mean()
    T = np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1.0]])
    pp = np.column_stack([p, np.ones(len(p))]) @ T.T
    return pp[:, :2], T


def apply_H(H, pts):
    pts = np.atleast_2d(np.asarray(pts, float))
    p = np.column_stack([pts, np.ones(len(pts))]) @ np.asarray(H).T
    return p[:, :2] / p[:, 2:3]


def dlt_homography(src, dst):
    """Сырой DLT гомографии (ЛР_04, задание 1)."""
    A = []
    for (x, y), (u, v) in zip(src, dst):
        A += [[-x, -y, -1, 0, 0, 0, u * x, u * y, u],
              [0, 0, 0, -x, -y, -1, v * x, v * y, v]]
    h = np.linalg.svd(np.asarray(A, float))[2][-1]
    H = h.reshape(3, 3)
    return H / H[2, 2]


def dlt_normalized(src, dst):
    """DLT + нормировка Хартли (cond плана ~1e6 -> ~20) (ЛР_04, задание 2)."""
    sn, Ts = normalize_pts(src)
    dn, Td = normalize_pts(dst)
    Hn = dlt_homography(sn, dn)
    H = np.linalg.inv(Td) @ Hn @ Ts
    return H / H[2, 2]


def ransac_homography(src, dst, thr=3.0, iters=2000, seed=0):
    """RANSAC гомографии (ЛР_04, задание 5)."""
    r = np.random.default_rng(seed)
    best_in, best_H = None, None
    n = len(src)
    for _ in range(iters):
        idx = r.choice(n, 4, replace=False)
        with np.errstate(all='ignore'):
            try:
                Hh = dlt_normalized(src[idx], dst[idx])
            except np.linalg.LinAlgError:
                continue
            d = np.linalg.norm(apply_H(Hh, src) - dst, axis=1)
        inl = d < thr
        if best_in is None or inl.sum() > best_in.sum():
            best_in, best_H = inl, Hh
    if best_in.sum() >= 4:
        best_H = dlt_normalized(src[best_in], dst[best_in])
    return best_H, best_in


def pose_from_homography(H, K):
    """Поза из H с SVD-проекцией в SO(3) (ЛР_04, задание 3 + ЛР_01, з.7)."""
    Ki = np.linalg.inv(np.asarray(K))
    h1, h2, h3 = (Ki @ H).T
    lam = 1.0 / np.linalg.norm(h1)
    r1, r2, t = lam * h1, lam * h2, lam * h3
    R = np.column_stack([r1, r2, np.cross(r1, r2)])
    U, _, Vt = np.linalg.svd(R)
    R = U @ np.diag([1.0, 1.0, np.linalg.det(U @ Vt)]) @ Vt
    if t[2] < 0:
        R, t = -R, -t
        R[:, 2] *= -1
    return R, t


# ---------- рука ----------
def arm_fk(q):
    """FK пространственной руки: точки, кадры, оси, начала (ЛР_05)."""
    q1, q2, q3, q4 = q
    p_base = np.array([0.0, 0.0, D_BASE])
    Rz1 = Rz(q1)
    Rs = Rz1 @ Ry(q2 - np.pi / 2)
    p_elbow = p_base + Rs @ np.array([L_ARM[0], 0, 0])
    Re = Rs @ Ry(q3)
    p_wrist = p_elbow + Re @ np.array([L_ARM[1], 0, 0])
    Rw = Re @ Ry(q4)
    p_ee = p_wrist + Rw @ np.array([L_ARM[2], 0, 0])
    points = np.array([[0, 0, 0], p_base, p_elbow, p_wrist, p_ee], float)
    frames = []
    for p, R in ((p_base, Rs), (p_elbow, Re), (p_wrist, Rw)):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, p
        frames.append(T)
    axes = [np.array([0.0, 0.0, 1.0]), Rz1 @ np.array([0.0, 1.0, 0.0]),
            Rz1 @ np.array([0.0, 1.0, 0.0]), Rz1 @ np.array([0.0, 1.0, 0.0])]
    origins = [np.array([0.0, 0.0, 0.0]), p_base, p_elbow, p_wrist]
    return points, frames, axes, origins


def arm_jac(q):
    """Геометрический якобиан 3x4 (ЛР_06, задание 1)."""
    points, _, axes, origins = arm_fk(q)
    p_ee = points[-1]
    return np.column_stack([np.cross(a, p_ee - o)
                            for a, o in zip(axes, origins)])


def ik3d_newton(target, q0, iters=120, tol=1e-12):
    """Ньютоновский IK с pinv (ЛР_06, задание 3)."""
    q = np.asarray(q0, float).copy()
    hist, conds = [], []
    for _ in range(iters):
        r = arm_fk(q)[0][-1] - np.asarray(target, float)
        J = arm_jac(q)
        conds.append(np.linalg.cond(J))
        hist.append(np.linalg.norm(r))
        if hist[-1] < tol:
            break
        q = (q - np.linalg.pinv(J) @ r + np.pi) % (2 * np.pi) - np.pi
    hist.append(np.linalg.norm(arm_fk(q)[0][-1] - np.asarray(target, float)))
    return q, np.array(hist), np.array(conds)


def cam_pose(t):
    """Перелёт камеры A->B по slerp (ЛР_02, задание 5)."""
    RA, _ = look_at(EYE_A, MARKER_C)
    RB, _ = look_at(EYE_B, MARKER_C)
    q = slerp(R_to_quat(RA), R_to_quat(RB), t)
    # кватернион (w,x,y,z) -> R
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    eye = (1 - t) * EYE_A + t * EYE_B
    return R, -R @ eye, eye


def marker_grid():
    """81 угол маркера в мире (9x9, шаг 0.02 м) - как в ноутбуке."""
    g = np.linspace(-MARKER_H, MARKER_H, 9)
    return np.array([[a + MARKER_C[0], b + MARKER_C[1], 0.0]
                     for a in g for b in g])
