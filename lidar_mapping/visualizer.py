"""
Map Visualizer (Matplotlib)
- origin='lower' + extent 로 월드 좌표와 정렬
- 왼쪽: 점유맵(RGB), 오른쪽: 확률맵(0~1)
- 로봇 포즈와 라이다 히트 포인트(빨간 점) 오버레이
"""

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from typing import Optional


class MapVisualizer:
    def __init__(self, mapping_system, model: Optional[object] = None, data: Optional[object] = None):
        """
        mapping_system: LidarMappingSystem 인스턴스
        model/data가 전달되지 않으면 mapping_system의 model/data를 사용합니다.
        """
        self.mapping_system = mapping_system
        self.model = model if model is not None else getattr(mapping_system, "model", None)
        self.data = data if data is not None else getattr(mapping_system, "data", None)

        self.fig: Optional[plt.Figure] = None
        self.ax_map = None
        self.ax_prob = None
        self.im_map = None
        self.im_prob = None

        # overlays
        self.robot_dot = None
        self.robot_arrow = None
        self.scan_plot = None

        # path trace (optional)
        self.path_x, self.path_y = [], []
        self.path_line = None

    # ---------------- Figure ----------------
    def setup_figure(self):
        self.fig, (self.ax_map, self.ax_prob) = plt.subplots(1, 2, figsize=(12, 6))
        try:
            self.fig.canvas.manager.set_window_title("Lidar Mapping Visualization")
        except Exception:
            pass

        extent = self.mapping_system.grid_map.get_extent()

        # Occupancy map (RGB)
        img = self.mapping_system.get_map_image()
        self.im_map = self.ax_map.imshow(
            img, extent=extent, origin="lower", interpolation="nearest"
        )
        self.ax_map.set_title("Occupancy Grid Map")
        self.ax_map.set_xlabel("X (meters)")
        self.ax_map.set_ylabel("Y (meters)")

        # Probability map
        prob = self.mapping_system.get_probability_map()
        self.im_prob = self.ax_prob.imshow(
            prob, extent=extent, origin="lower", interpolation="nearest", vmin=0.0, vmax=1.0
        )
        self.ax_prob.set_title("Probability Map")
        self.ax_prob.set_xlabel("X (meters)")
        self.ax_prob.set_ylabel("Y (meters)")
        try:
            self.fig.colorbar(self.im_prob, ax=self.ax_prob, fraction=0.046, pad=0.04)
        except Exception:
            pass

        # overlays: robot & scan hits
        self.robot_dot, = self.ax_map.plot([], [], "blue", marker="o", markersize=6, linestyle="None", label="Robot")
        self.robot_arrow = None
        self.scan_plot, = self.ax_map.plot([], [], "r.", markersize=2, label="Scan")
        self.path_line, = self.ax_map.plot([], [], "b.", markersize=1, alpha=0.5)

        self.ax_map.legend(loc="upper right")

        print("[Visualizer] Figure ready")

    # ---------------- Update ----------------
    def update_frame(self):
        if self.fig is None:
            self.setup_figure()

        # images
        img = self.mapping_system.get_map_image()
        prob = self.mapping_system.get_probability_map()
        self.im_map.set_data(img)
        self.im_prob.set_data(prob)

        # robot pose
        rx = float(self.data.qpos[0]) if (self.data is not None and self.data.qpos.size >= 1) else 0.0
        ry = float(self.data.qpos[1]) if (self.data is not None and self.data.qpos.size >= 2) else 0.0
        self.robot_dot.set_data([rx], [ry])
        self.path_x.append(rx); self.path_y.append(ry)
        self.path_line.set_data(self.path_x, self.path_y)

        # heading (간단 yaw 가정: qpos[2])
        if (self.data is not None) and (self.data.qpos.size >= 3):
            yaw = float(self.data.qpos[2])
            dx, dy = 0.3 * np.cos(yaw), 0.3 * np.sin(yaw)
        else:
            dx, dy = 0.3, 0.0

        if self.robot_arrow:
            try:
                self.robot_arrow.remove()
            except Exception:
                pass
        self.robot_arrow = self.ax_map.arrow(rx, ry, dx, dy, color="b", width=0.0)

        # scan hits (hit 포인트만 표시; no-hit는 생략)
        scan = self.mapping_system.lidar.get_scan()
        if scan["points"].size:
            xs = scan["points"][:, 0]; ys = scan["points"][:, 1]
            self.scan_plot.set_data(xs, ys)
        else:
            self.scan_plot.set_data([], [])

        # 제목/통계
        stats = self.mapping_system.get_statistics()
        title = (
            f"Scans: {stats.get('total_scans', 0)} | "
            f"Coverage: {stats.get('coverage_percent', 0.0):.1f}% | "
            f"Occupied: {stats.get('occupied_cells', 0)} | "
            f"Free: {stats.get('free_cells', 0)}"
        )
        self.ax_map.set_title(f"Occupancy Grid Map   {title}")

        self.fig.canvas.draw_idle()

    # ---------------- Public helpers ----------------
    def update_once(self):
        if self.fig is None:
            self.setup_figure()
        self.update_frame()
        plt.pause(0.01)

    def close(self):
        if self.fig:
            plt.close(self.fig)
            self.fig = None
        print("[Visualizer] Closed")
