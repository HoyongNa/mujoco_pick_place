# view_saved_map.py
# 사용법:
#   python view_saved_map.py lidar_map_20250826_215447.npz
# (파일 인자를 생략하면 현재 폴더에서 가장 최근 .npz를 찾으려 시도합니다)

import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt

def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))

def load_npz(path):
    if not os.path.isfile(path):
        raise FileNotFoundError(f"NPZ file not found: {path}")
    npz = np.load(path, allow_pickle=False)
    files = set(npz.files)

    log_odds = npz["log_odds"].astype(np.float32)
    h, w = log_odds.shape

    # 안전하게 읽기 (없을 수도 있으니 기본값 준비)
    resolution = float(npz["resolution"]) if "resolution" in files else 0.05
    if "origin" in files:
        origin_raw = npz["origin"]
        origin = (float(origin_raw[0]), float(origin_raw[1]))
    else:
        # 기본: 맵 중앙이 (0,0)
        origin = (-w * resolution / 2.0, -h * resolution / 2.0)

    return log_odds, resolution, origin

def compute_stats(log_odds):
    known = int(np.count_nonzero(log_odds))
    occ   = int(np.count_nonzero(log_odds > 0))
    free  = int(np.count_nonzero(log_odds < 0))
    total = int(log_odds.size)
    cov   = 100.0 * known / max(total, 1)
    return {
        "height": log_odds.shape[0],
        "width": log_odds.shape[1],
        "occupied_cells": occ,
        "free_cells": free,
        "coverage_percent": cov,
    }

def visualize(log_odds, resolution, origin, title_suffix=""):
    h, w = log_odds.shape
    xmin = origin[0]
    xmax = origin[0] + w * resolution
    ymin = origin[1]
    ymax = origin[1] + h * resolution
    extent = (xmin, xmax, ymin, ymax)

    prob = sigmoid(log_odds)

    # RGB occupancy 이미지 (unknown: gray, free: white, occupied: black)
    img = np.full((h, w, 3), 128, np.uint8)
    free = prob < 0.5
    occ  = prob >= 0.5
    img[free] = (255, 255, 255)
    img[occ]  = (0, 0, 0)

    stats = compute_stats(log_odds)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    fig.suptitle(f"Saved Lidar Map {title_suffix}".strip())

    ax1.imshow(img, extent=extent, origin="lower", interpolation="nearest")
    ax1.set_title(
        f"Occupancy (occ: {stats['occupied_cells']}, free: {stats['free_cells']}, "
        f"coverage: {stats['coverage_percent']:.1f}%)"
    )
    ax1.set_xlabel("X (meters)")
    ax1.set_ylabel("Y (meters)")
    ax1.grid(True, color=(0.8, 0.8, 0.8), lw=0.5, alpha=0.4)

    im = ax2.imshow(prob, extent=extent, origin="lower",
                    interpolation="nearest", vmin=0.0, vmax=1.0)
    ax2.set_title("Probability Map")
    ax2.set_xlabel("X (meters)")
    ax2.set_ylabel("Y (meters)")
    fig.colorbar(im, ax=ax2, fraction=0.046, pad=0.04)

    plt.tight_layout()
    plt.show()

def find_latest_npz():
    files = sorted(glob.glob("*.npz"), key=os.path.getmtime, reverse=True)
    return files[0] if files else None

def main():
    if len(sys.argv) >= 2:
        path = sys.argv[1]
    else:
        latest = find_latest_npz()
        if latest is None:
            print("❌ NPZ 파일 경로를 인자로 주거나, 현재 폴더에 .npz 파일이 있어야 합니다.")
            print("   예) python view_saved_map.py lidar_map_20250826_215447.npz")
            sys.exit(1)
        print(f"ℹ️ 경로 인자가 없어 최신 파일을 엽니다: {latest}")
        path = latest

    log_odds, res, origin = load_npz(path)
    print(f"Loaded: {path}")
    print(f"  shape: {log_odds.shape} (HxW)")
    print(f"  resolution: {res} m/cell")
    print(f"  origin: {origin}")

    stats = compute_stats(log_odds)
    print(f"  occupied: {stats['occupied_cells']}")
    print(f"  free: {stats['free_cells']}")
    print(f"  coverage: {stats['coverage_percent']:.1f}%")

    visualize(log_odds, res, origin, title_suffix=os.path.basename(path))

if __name__ == "__main__":
    main()
