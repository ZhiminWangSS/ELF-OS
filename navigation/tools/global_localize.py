#!/usr/bin/env python3
"""Offline global localization: grid-search + ICP refine against the prior map.

Read-only. Captures one live lidar frame, seeds candidate poses by 2D band FFT
correlation over free cells, then refines each seed with the same two-stage
ICP schedule prior_initializer uses (single frame, full 3D map, voxel 0.2,
correspondence 0.8->0.3). Prints ranked base poses for tools/set_initial_pose.py."""
import json, math, time
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.spatial import cKDTree
from scipy.signal import correlate
import rclpy
from livox_ros_driver2.msg import CustomMsg

NAV = Path(__file__).resolve().parents[1]
MAPDIR = NAV / 'maps/floor_1789552084236'
HPI6 = math.pi / 6
RY_UNDO = np.array([[math.cos(-HPI6), 0, math.sin(-HPI6)],
                    [0, 1, 0],
                    [-math.sin(-HPI6), 0, math.cos(-HPI6)]])
SENSOR_T = np.array([-0.011, -0.02329, 0.04412])
PITCH = 0.5235987756


def capture_one():
    rclpy.init()
    n = rclpy.create_node('global_localize_capture')
    got = []
    def cb(m):
        if len(got) < 4:
            got.append(np.array([[p.x, p.y, p.z] for p in m.points], dtype=np.float32))
    n.create_subscription(CustomMsg, '/livox/lidar', cb, 10)
    end = time.monotonic() + 4.0
    while time.monotonic() < end and len(got) < 4:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node(); rclpy.shutdown()
    if not got:
        raise SystemExit('no lidar frames received')
    return got[-1]


def voxel(pts, size):
    keys = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def load_map_all():
    raw = (MAPDIR / 'map.pcd').read_bytes()
    header, body = [], raw
    for _ in range(11):
        line, body = body.split(b'\n', 1)
        header.append(line.decode())
    count = int([l for l in header if l.startswith('POINTS')][0].split()[1])
    data = np.frombuffer(body[:count * 16], dtype=np.float32).reshape(count, 4)[:, :3]
    return np.asarray(data, dtype=np.float64)


def icp3d(src, tree, T, max_dist, iters=40, eps=1e-7):
    T = T.copy()
    for _ in range(iters):
        q = src @ T[:3, :3].T + T[:3, 3]
        d, j = tree.query(q, k=1, distance_upper_bound=max_dist)
        ok = np.isfinite(d)
        if ok.sum() < 50:
            break
        a = q[ok]
        b = np.asarray(tree.data)[j[ok]]
        ca, cb = a.mean(0), b.mean(0)
        H = (a - ca).T @ (b - cb)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = cb - R @ ca
        step = np.eye(4)
        step[:3, :3] = R
        step[:3, 3] = t
        T_new = step @ T
        if np.abs(T_new - T).max() < eps:
            T = T_new
            break
        T = T_new
    return T


def score_exact(src, T, tree):
    q = src @ T[:3, :3].T + T[:3, 3]
    d, _ = tree.query(q, k=1)
    inlier = d < 0.3
    overlap = float(inlier.mean())
    rmse = float(np.sqrt((d[inlier] ** 2).mean())) if inlier.any() else 9.9
    return overlap, rmse


def main():
    frame = capture_one()
    nrm = np.linalg.norm(frame, axis=1)
    imu_pts = frame[(nrm > 0.5) & (nrm < 25.0)] @ RY_UNDO.T + SENSOR_T
    src = voxel(imu_pts.astype(np.float64), 0.2)
    map_raw = load_map_all()
    map3d = voxel(map_raw, 0.2)
    tree = cKDTree(map3d)
    print(f'frame {len(frame)} raw -> {len(src)} voxel; map {len(map3d)} voxel', flush=True)

    band_map = map_raw[(map_raw[:, 2] >= 0.15) & (map_raw[:, 2] <= 1.2)][:, :2]
    band_scan = imu_pts[(imu_pts[:, 2] >= 0.15) & (imu_pts[:, 2] <= 1.2)][:, :2]
    cfg = yaml.safe_load((MAPDIR / 'map.yaml').read_text())
    grid_img = np.flipud(np.array(Image.open(MAPDIR / cfg['image'])))
    res, pad = 0.25, 1.5
    bounds = (band_map[:, 0].min() - pad, band_map[:, 1].min() - pad,
              band_map[:, 0].max() + pad, band_map[:, 1].max() + pad)
    x0, y0, _, _ = bounds
    H_, W_ = int((bounds[3] - bounds[1]) / res) + 1, int((bounds[2] - bounds[0]) / res) + 1
    M = np.zeros((H_, W_), dtype=np.float32)
    jj = np.round((band_map[:, 0] - x0) / res).astype(int)
    ii = np.round((band_map[:, 1] - y0) / res).astype(int)
    M[ii, jj] = 1.0
    wx = x0 + np.arange(W_) * res
    wy = y0 + np.arange(H_) * res
    px = np.floor((wx - cfg['origin'][0]) / cfg['resolution']).astype(int)
    py = np.floor((wy - cfg['origin'][1]) / cfg['resolution']).astype(int)
    PX, PY = np.meshgrid(px, py)
    okc = (PX >= 0) & (PX < grid_img.shape[1]) & (PY >= 0) & (PY < grid_img.shape[0])
    free_abs = np.zeros((H_, W_), dtype=bool)
    free_abs[okc] = grid_img[PY[okc], PX[okc]] == 254
    s_ci, s_cj = H_ // 2, W_ // 2

    S0 = np.zeros((H_, W_), dtype=np.float32)
    S0[s_ci - 1:s_ci + 2, s_cj - 1:s_cj + 2] = np.eye(3)
    c0 = correlate(np.roll(S0, (17, -9), axis=(0, 1)), S0, mode='same', method='fft')
    i0, j0 = np.unravel_index(np.argmax(c0), c0.shape)
    row_k, col_k = i0 - 17, j0 + 9
    free_lag = np.zeros((H_, W_), dtype=bool)
    src_r = np.arange(H_) - row_k + s_ci
    src_c = np.arange(W_) - col_k + s_cj
    vr, vc = (src_r >= 0) & (src_r < H_), (src_c >= 0) & (src_c < W_)
    free_lag[np.ix_(vr, vc)] = free_abs[np.ix_(src_r[vr], src_c[vc])]

    seeds = []
    S = np.zeros((H_, W_), dtype=np.float32)
    sv = voxel(band_scan, 0.2)
    for deg in range(0, 360, 10):
        a = math.radians(deg)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        rot = sv @ R.T
        S[:] = 0
        gj = s_cj + np.round(rot[:, 0] / res).astype(int)
        gi = s_ci + np.round(rot[:, 1] / res).astype(int)
        o = (gi >= 0) & (gi < H_) & (gj >= 0) & (gj < W_)
        S[gi[o], gj[o]] = 1.0
        c = correlate(M, S, mode='same', method='fft') * free_lag
        if c.max() <= 0:
            continue
        for _ in range(4):
            i2, j2 = np.unravel_index(np.argmax(c), c.shape)
            seeds.append({'x': float(x0 + (j2 - col_k + s_cj) * res),
                          'y': float(y0 + (i2 - row_k + s_ci) * res),
                          'yaw_deg': float(deg), 'grid': float(c[i2, j2])})
            c[max(0, i2 - 4):i2 + 5, max(0, j2 - 4):j2 + 5] = 0
    seeds.sort(key=lambda s: s['grid'], reverse=True)
    kept = []
    for s in seeds:
        if all(math.hypot(s['x'] - k['x'], s['y'] - k['y']) > 1.5 or
               min(abs(s['yaw_deg'] - k['yaw_deg']) % 360, 360 - abs(s['yaw_deg'] - k['yaw_deg']) % 360) > 20
               for k in kept):
            kept.append(s)
        if len(kept) >= 30:
            break
    print(f'{len(kept)} distinct seeds; exact 3D refine', flush=True)

    Ry = np.array([[math.cos(PITCH), 0, math.sin(PITCH)], [0, 1, 0], [-math.sin(PITCH), 0, math.cos(PITCH)]])
    B_imu = np.eye(4); B_imu[:3, :3] = Ry; B_imu[:3, 3] = [0, 0, 0.46]

    results = []
    for s in kept[:20]:
        for dyaw in (-5, 0, 5):
            yaw = math.radians(s['yaw_deg'] + dyaw)
            Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            T0 = np.eye(4); T0[:3, :3] = Rz @ Ry
            T0[:3, 3] = Rz @ np.array([0., 0., 0.46]) + np.array([s['x'], s['y'], 0.])
            T = T0
            for md in (0.8, 0.3):
                T = icp3d(src, tree, T, md)
            ov, rm = score_exact(src, T, tree)
            Tb = T @ np.linalg.inv(B_imu)
            byaw = math.degrees(math.atan2(Tb[1, 0], Tb[0, 0])) % 360
            results.append({'base_x': round(float(Tb[0, 3]), 3), 'base_y': round(float(Tb[1, 3]), 3),
                            'base_yaw_deg': round(byaw, 1), 'overlap': round(ov, 3), 'rmse': round(rm, 3),
                            'seed': [round(s['x'], 1), round(s['y'], 1), s['yaw_deg']]})
    results.sort(key=lambda r: (r['overlap'] >= 0.65 and r['rmse'] < 0.15, r['overlap']), reverse=True)
    print(json.dumps(results[:10], indent=1))


if __name__ == '__main__':
    main()
