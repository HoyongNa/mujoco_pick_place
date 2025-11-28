"""
Occupancy Grid (log-odds)
- log_odds 누적 기반 2D 점유맵
- visualizer/test 호환을 위해 map_size / map_origin / get_visualization_image / get_extent 제공
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, Optional, List
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
        """world (x,y) -> map cell (row, col)"""
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return (my, mx)

    def map_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """map cell (row, col) -> world (x,y)"""
        x = col * self.resolution + self.origin[0]
        y = row * self.resolution + self.origin[1]
        return (x, y)

    def is_inside(self, row: int, col: int) -> bool:
        return 0 <= row < self.h and 0 <= col < self.w

    # ------------- update -------------
    def update_free(self, row: int, col: int):
        if self.is_inside(row, col):
            self.log_odds[row, col] = np.clip(
                self.log_odds[row, col] + self.params.l_free,
                self.params.l_min, self.params.l_max
            )

    def update_occupied(self, row: int, col: int):
        if self.is_inside(row, col):
            self.log_odds[row, col] = np.clip(
                self.log_odds[row, col] + self.params.l_occ,
                self.params.l_min, self.params.l_max
            )

    def get_probability(self, row: int, col: int) -> float:
        if not self.is_inside(row, col):
            return 0.5
        odds = np.exp(self.log_odds[row, col])
        return odds / (1.0 + odds)

    def get_probability_map(self) -> np.ndarray:
        odds = np.exp(self.log_odds)
        return odds / (1.0 + odds)

    # ------------- visualization -------------
    def get_visualization_image(self) -> np.ndarray:
        prob = self.get_probability_map()
        img = (255 * (1 - prob)).astype(np.uint8)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def get_extent(self) -> Tuple[float, float, float, float]:
        """Returns (xmin, xmax, ymin, ymax) for matplotlib imshow"""
        xmin = self.origin[0]
        xmax = self.origin[0] + self.w * self.resolution
        ymin = self.origin[1]
        ymax = self.origin[1] + self.h * self.resolution
        return (xmin, xmax, ymin, ymax)

    def reset(self):
        self.log_odds.fill(0)
        
    def clear(self):
        """Alias for reset for compatibility"""
        self.reset()
        
    def save(self, filename: str):
        """Save map to file"""
        np.savez(filename, 
                 log_odds=self.log_odds,
                 resolution=self.resolution,
                 origin=np.array(self.origin),
                 map_size=np.array(self.map_size))
        print(f"[OccupancyGrid] Map saved to {filename}")
        
    def get_statistics(self) -> dict:
        """Get map statistics"""
        prob_map = self.get_probability_map()
        return {
            "occupied_cells": int(np.sum(prob_map > self.params.prob_thresh)),
            "free_cells": int(np.sum(prob_map < (1 - self.params.prob_thresh))),
            "unknown_cells": int(np.sum((prob_map >= (1 - self.params.prob_thresh)) & 
                                       (prob_map <= self.params.prob_thresh))),
            "total_cells": int(self.h * self.w)
        }
    
    def update_scan(self, sensor_pos: Tuple[float, float], 
                    endpoints: np.ndarray,
                    mark_end_as_occupied: bool = True):
        """
        라이다 스캔 데이터로 맵 업데이트
        Bresenham 알고리즘을 사용하여 레이 트레이싱
        
        Args:
            sensor_pos: 센서의 월드 좌표 (x, y)
            endpoints: 레이저 끝점들의 월드 좌표 배열 (N, 2)
            mark_end_as_occupied: True면 끝점을 occupied로, False면 free로만
        """
        if endpoints.size == 0:
            return
            
        # 센서 위치를 맵 좌표로 변환
        sensor_row, sensor_col = self.world_to_map(sensor_pos[0], sensor_pos[1])
        
        for endpoint in endpoints:
            end_row, end_col = self.world_to_map(float(endpoint[0]), float(endpoint[1]))
            
            # Bresenham line algorithm으로 레이 경로 상의 모든 셀 구하기
            cells = self._bresenham_line(sensor_row, sensor_col, end_row, end_col)
            
            # 레이 경로 상의 셀들을 free로 업데이트 (끝점 제외)
            for i, (row, col) in enumerate(cells[:-1]):
                self.update_free(row, col)
            
            # 끝점 처리
            if len(cells) > 0:
                last_row, last_col = cells[-1]
                if mark_end_as_occupied:
                    # Hit: 끝점을 occupied로
                    self.update_occupied(last_row, last_col)
                else:
                    # No-hit: 끝점도 free로 (최대 거리에서 아무것도 없음)
                    self.update_free(last_row, last_col)
    
    def _bresenham_line(self, r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
        """
        Bresenham's line algorithm
        두 셀 사이의 직선 경로 상의 모든 셀 반환
        """
        cells = []
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        
        r, c = r0, c0
        
        while True:
            if self.is_inside(r, c):
                cells.append((r, c))
            
            if r == r1 and c == c1:
                break
                
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc
                
        return cells


# LidarIntegratedController와의 호환성을 위한 래퍼 클래스
class OccupancyGrid:
    """LidarIntegratedController와 호환되는 OccupancyGrid 클래스"""
    
    def __init__(self, size: Tuple[int, int] = (200, 200), resolution: float = 0.05):
        """
        Args:
            size: 그리드 크기 (width, height)
            resolution: 셀 해상도 (meters)
        """
        # 내부적으로 OccupancyGridMap 사용
        self._map = OccupancyGridMap(map_size=(size[1], size[0]), resolution=resolution)
        
        # 호환성을 위한 속성
        self.size = size
        self.resolution = resolution
        self.origin = self._map.origin
        self.grid = self._map.log_odds  # log-odds 맵 직접 참조
        
    def update_cell_free(self, col: int, row: int):
        """셀을 free로 업데이트 (인터페이스 맞춤)"""
        self._map.update_free(row, col)
        
    def update_cell_occupied(self, col: int, row: int):
        """셀을 occupied로 업데이트 (인터페이스 맞춤)"""
        self._map.update_occupied(row, col)
        
    def get_probability(self, col: int, row: int) -> float:
        """특정 셀의 점유 확률 반환"""
        return self._map.get_probability(row, col)
        
    def get_probability_map(self) -> np.ndarray:
        """전체 확률 맵 반환"""
        return self._map.get_probability_map()
        
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """월드 좌표를 맵 좌표로 변환"""
        row, col = self._map.world_to_map(x, y)
        return (col, row)  # (x, y) 순서로 반환
        
    def map_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """맵 좌표를 월드 좌표로 변환"""
        return self._map.map_to_world(row, col)
        
    def is_inside(self, col: int, row: int) -> bool:
        """좌표가 맵 내부인지 확인"""
        return self._map.is_inside(row, col)
        
    def update_scan(self, sensor_pos: Tuple[float, float], endpoints: np.ndarray, mark_end_as_occupied: bool = True):
        """라이다 스캔 데이터로 맵 업데이트 (래퍼 메서드)"""
        return self._map.update_scan(sensor_pos, endpoints, mark_end_as_occupied)
    
    def reset(self):
        """맵 초기화"""
        self._map.reset()
