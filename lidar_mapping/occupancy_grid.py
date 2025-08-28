"""
Occupancy Grid (log-odds)
- log_odds 누적 기반 2D 점유맵
- visualizer/test 호환을 위해 map_size / map_origin / get_visualization_image / get_extent 제공
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np
import cv2


@dataclass
class _Params:
    l_free: float = -0.40
    l_occ:  float = +0.85
    l_min:  float = -4.0
    l_max:  float = +4.0
    prob_thresh: float = 0.5  # 시각화 임계


class OccupancyGridMap:
    def __init__(self,
                 map_size: Tuple[int, int] = (200, 200),
                 resolution: float = 0.05,
                 origin: Optional[Tuple[float, float]] = None):
        """
        map_size: (height, width) in cells
        resolution: meters per cell
        origin: world (x,y) of map cell (0,0). None이면 맵을 중심(-w/2,-h/2)로 둠.
        """
        self.h, self.w = int(map_size[0]), int(map_size[1])
        self.resolution = float(resolution)

        if origin is None:
            ox = -self.w * self.resolution / 2.0
            oy = -self.h * self.resolution / 2.0
            origin = (ox, oy)
        self.origin = (float(origin[0]), float(origin[1]))

        # ---- 호환 alias ----
        self.map_size   = (self.h, self.w)   # visualizer가 참조
        self.map_origin = self.origin

        self.params = _Params()
        self.log_odds = np.zeros((self.h, self.w), dtype=np.float32)

    # ------------- coords -------------
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        j = int((x - self.origin[0]) / self.resolution)
        i = int((y - self.origin[1]) / self.resolution)
        return i, j

    def map_to_world(self, i: int, j: int) -> Tuple[float, float]:
        x = self.origin[0] + (j + 0.5) * self.resolution
        y = self.origin[1] + (i + 0.5) * self.resolution
        return x, y

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.h and 0 <= j < self.w

    # ------------- ray tracing -------------
    def _bresenham(self, i0: int, j0: int, i1: int, j1: int):
        di = abs(i1 - i0)
        dj = abs(j1 - j0)
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1
        err = dj - di
        i, j = i0, j0
        out = []
        while True:
            out.append((i, j))
            if i == i1 and j == j1:
                break
            e2 = 2 * err
            if e2 > -di:
                err -= di; j += sj
            if e2 < dj:
                err += dj; i += si
        return out

    def update_scan(self,
                    origin_xy: Tuple[float, float],
                    points_xy: np.ndarray,
                    mark_end_as_occupied: bool = True):
        """
        origin -> 각각의 point까지 레이로 업데이트.
        - mark_end_as_occupied=True  : 끝점을 occupied로 누적(실제 hit)
        - mark_end_as_occupied=False : 끝점도 free로 누적(no-hit; 경로만 free)
        """
        if points_xy is None or len(points_xy) == 0:
            return

        oi, oj = self.world_to_map(origin_xy[0], origin_xy[1])

        for p in points_xy:
            pi, pj = self.world_to_map(float(p[0]), float(p[1]))
            cells = self._bresenham(oi, oj, pi, pj)
            if not cells:
                continue

            # 1) 경로 free (끝점 제외)
            for (ci, cj) in cells[:-1]:
                if self.in_bounds(ci, cj):
                    self.log_odds[ci, cj] = np.clip(
                        self.log_odds[ci, cj] + self.params.l_free,
                        self.params.l_min, self.params.l_max)

            # 2) 끝점 처리
            ei, ej = cells[-1]
            if self.in_bounds(ei, ej):
                inc = self.params.l_occ if mark_end_as_occupied else self.params.l_free
                self.log_odds[ei, ej] = np.clip(
                    self.log_odds[ei, ej] + inc,
                    self.params.l_min, self.params.l_max)

    # ------------- viz / stats -------------
    def get_probability_map(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def get_visualization_image(self) -> np.ndarray:
        """
        RGB 시각화 이미지 (H,W,3) uint8
        - unknown: gray(128), free: white(255), occupied: black(0)
        - flip 하지 않음 (imshow에서 origin='lower' 사용)
        """
        p = self.get_probability_map()
        img = np.full((self.h, self.w, 3), 128, np.uint8)  # unknown
        free = p < self.params.prob_thresh
        occ  = p >= self.params.prob_thresh
        img[free] = (255, 255, 255)
        img[occ]  = (0, 0, 0)
        return img

    def get_extent(self) -> Tuple[float, float, float, float]:
        """matplotlib imshow extent = (xmin, xmax, ymin, ymax)"""
        xmin = self.origin[0]
        xmax = self.origin[0] + self.w * self.resolution
        ymin = self.origin[1]
        ymax = self.origin[1] + self.h * self.resolution
        return (xmin, xmax, ymin, ymax)

    def get_statistics(self) -> dict:
        known = int(np.count_nonzero(self.log_odds))
        occ   = int(np.count_nonzero(self.log_odds > 0))
        free  = int(np.count_nonzero(self.log_odds < 0))
        total = int(self.h * self.w)
        cov   = 100.0 * known / max(total, 1)
        return {
            "coverage_percent": float(cov),
            "occupied_cells": occ,
            "free_cells": free,
            "total_cells": total,
            "height": self.h,
            "width": self.w,
            "resolution": self.resolution,
        }

    def clear(self):
        self.log_odds.fill(0)

    def save(self, path: str):
        np.savez_compressed(
            path,
            log_odds=self.log_odds,
            resolution=self.resolution,
            origin=np.asarray(self.origin, dtype=np.float32),
            h=self.h, w=self.w
        )

    def load(self, path: str):
        arr = np.load(path, allow_pickle=False)
        self.log_odds = arr["log_odds"]
        self.resolution = float(arr["resolution"])
        o = arr["origin"]
        self.origin = (float(o[0]), float(o[1]))
        self.h, self.w = int(arr["h"]), int(arr["w"])
        # alias 갱신
        self.map_size = (self.h, self.w)
        self.map_origin = self.origin
