"""
Lidar Mapping System
- hit: occupied 끝점 포함
- no-hit: 경로만 free 로 누적(끝점 occupied 금지)
- visualizer 호환: get_map_image(), get_statistics() 등 제공
"""

import threading, time
from typing import Optional
import mujoco, numpy as np

from .lidar_sensor import LidarSensor
from .occupancy_grid import OccupancyGridMap


class LidarMappingSystem:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 map_size=(200, 200), resolution=0.05):
        self.model, self.data = model, data
        self.grid_map = OccupancyGridMap(map_size=map_size, resolution=resolution)
        self.lidar = LidarSensor(model, data)

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._running = False

        self.total_updates = 0
        self.total_scans = 0
        print("[MappingSystem] Initialized")

    # ----- loop -----
    def start_mapping(self, update_rate: float = 10.0, **kwargs):
        """과거 호환: hz=... 인자도 허용"""
        if self._running: return
        if "hz" in kwargs and kwargs["hz"] is not None:
            try: update_rate = float(kwargs["hz"])
            except: pass
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, args=(float(update_rate),), daemon=True)
        self._thread.start(); self._running = True
        print(f"[MappingSystem] started ({update_rate} Hz)")

    def stop_mapping(self):
        if not self._running: return
        self._stop.set()
        if self._thread: self._thread.join(timeout=2.0)
        self._running = False
        print("[MappingSystem] stopped")

    def _loop(self, rate: float):
        dt = 1.0 / max(rate, 1e-3)
        while not self._stop.is_set():
            t0 = time.time()
            try: self.update_once()
            except Exception as e: print(f"[MappingSystem] update error: {e}")
            d = time.time() - t0
            if d < dt: time.sleep(dt - d)

    # ----- single update -----
    def update_once(self) -> dict:
        scan = self.lidar.get_scan()
        self.total_scans += 1
        added_hit = added_free = 0

        # 1) hit -> 끝점 occupied
        if scan["points"].size:
            for s, p in zip(scan["origins"], scan["points"]):
                self.grid_map.update_scan((float(s[0]), float(s[1])),
                                          np.asarray([p]),
                                          mark_end_as_occupied=True)
                added_hit += 1

        # 2) no-hit -> 끝점 occupied 금지 (경로만 free)
        if scan["free_points"].size:
            for s, p in zip(scan["free_origins"], scan["free_points"]):
                self.grid_map.update_scan((float(s[0]), float(s[1])),
                                          np.asarray([p]),
                                          mark_end_as_occupied=False)
                added_free += 1

        self.total_updates += 1
        return {
            "total_updates": self.total_updates,
            "hit_added": added_hit,
            "free_only_added": added_free,
            "num_valid": int(scan["num_valid"]),
            "num_nohit": int(scan["num_nohit"]),
            "num_beams": int(scan["num_beams"]),
        }

    # ----- compat / utils -----
    def get_map_image(self) -> np.ndarray:
        return self.grid_map.get_visualization_image()

    def get_probability_map(self) -> np.ndarray:
        return self.grid_map.get_probability_map()

    def get_log_odds(self) -> np.ndarray:
        return self.grid_map.log_odds

    def get_statistics(self) -> dict:
        m = self.grid_map.get_statistics()
        s = self.lidar.get_scan_statistics()
        # Visualizer가 기대하는 키들을 최상위로 포함
        return {**m, **s, "total_scans": self.total_scans, "total_updates": self.total_updates}

    def clear_map(self):
        self.grid_map.clear(); self.total_updates = self.total_scans = 0
        print("[MappingSystem] Map cleared")

    def save_map(self, filename: str = "lidar_map") -> str:
        import time as _t
        ts = _t.strftime("%Y%m%d_%H%M%S")
        full = f"{filename}_{ts}.npz"
        self.grid_map.save(full); return full

    def __del__(self): self.stop_mapping()
