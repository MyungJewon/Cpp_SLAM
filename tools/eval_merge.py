#!/usr/bin/env python3
"""병합 맵 품질 평가: 로버스트 span + 레퍼런스 대비 정렬 RMSE.

사용법:
    python tools/eval_merge.py <merged.ply> [reference.ply|reference.pcd]

- reference 없으면 span만 출력.
- reference 있으면 yaw-sweep ICP로 정렬 후 RMSE/inlier 비율 출력.
  (두 점군의 좌표계가 달라도 수직축 yaw + centroid 정렬로 맞춘다.)
"""
import sys
import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


def load_ply(path, sub=1):
    started = False
    pts = []
    for line in open(path):
        if not started:
            if line.startswith("end_header"):
                started = True
            continue
        p = line.split()
        if len(p) >= 3:
            pts.append((float(p[0]), float(p[1]), float(p[2])))
    return np.array(pts)[::sub]


def load_pcd_bin(path, sub=1):
    data = open(path, "rb").read()
    h = data.find(b"DATA binary\n") + len(b"DATA binary\n")
    body = data[h:]
    n = len(body) // 16  # x y z intensity (float32)
    arr = np.frombuffer(body[: n * 16], dtype=np.float32).reshape(n, 4)[:, :3]
    arr = arr[np.isfinite(arr).all(1)]
    return arr[::sub].astype(np.float64)


def load_any(path, sub=1):
    if path.lower().endswith(".pcd"):
        return load_pcd_bin(path, sub)
    return load_ply(path, sub)


def trim_outliers(P):
    med = np.median(P, 0)
    mad = np.median(np.abs(P - med), 0) * 1.4826 + 1e-6
    return P[(np.abs(P - med) < 6 * mad).all(1)]


def robust_span(P):
    out = {}
    for i, t in enumerate("XYZ"):
        v = np.sort(P[:, i])
        n = len(v)
        out[t] = v[int(0.995 * n)] - v[int(0.005 * n)]
    return out


def yaw_R(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def align_rmse(ours, ref):
    if cKDTree is None:
        print("scipy 미설치 — RMSE 평가 생략 (pip install scipy)")
        return
    tree = cKDTree(ref)
    cs, cr = ours.mean(0), ref.mean(0)
    best = None
    for deg in range(0, 360, 15):
        R = yaw_R(np.radians(deg))
        t = cr - R @ cs
        for _ in range(25):
            P = (R @ ours.T).T + t
            d, idx = tree.query(P)
            thr = max(np.percentile(d, 70), 0.4)
            m = d < thr
            if m.sum() < 100:
                break
            Pm, Qm = ours[m], ref[idx[m]]
            cp, cq = Pm.mean(0), Qm.mean(0)
            H = (Pm - cp).T @ (Qm - cq)
            U, _, Vt = np.linalg.svd(H)
            Rr = Vt.T @ U.T
            if np.linalg.det(Rr) < 0:
                Vt[2] *= -1
                Rr = Vt.T @ U.T
            R, t = Rr, cq - Rr @ cp
        P = (R @ ours.T).T + t
        d, _ = tree.query(P)
        inl50 = (d < 0.5).mean()
        rmse_in = np.sqrt(np.mean(d[d < 0.5] ** 2)) if (d < 0.5).any() else 9
        if best is None or inl50 > best[0]:
            best = (inl50, deg, (d < 0.2).mean(), rmse_in)
    print(
        f"정렬: yaw={best[1]}deg | <20cm={best[2]*100:.1f}% "
        f"<50cm={best[0]*100:.1f}% RMSE(<50cm)={best[3]:.3f}m"
    )


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    ours = trim_outliers(load_any(sys.argv[1], sub=2))
    sp = robust_span(ours)
    print(f"{sys.argv[1]}  점={len(ours)}")
    print(f"  robust span(0.5-99.5%): X={sp['X']:.1f} Y={sp['Y']:.1f} Z={sp['Z']:.1f}")
    if len(sys.argv) >= 3:
        ref = trim_outliers(load_any(sys.argv[2], sub=8))
        rsp = robust_span(ref)
        print(f"{sys.argv[2]} (ref) span: X={rsp['X']:.1f} Y={rsp['Y']:.1f} Z={rsp['Z']:.1f}")
        align_rmse(ours, ref)


if __name__ == "__main__":
    main()
