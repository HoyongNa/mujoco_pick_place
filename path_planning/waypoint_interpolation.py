"""
Waypoint Interpolation Module for A* + MPC Navigation
경로점 보간 모듈 - A* 결과를 MPC에 적합하게 변환
"""

import numpy as np
from typing import List, Tuple, Optional, Union
from enum import Enum


class InterpolationType(Enum):
    """보간 방법 타입"""
    LINEAR = "linear"
    CUBIC_SPLINE = "cubic_spline"
    ADAPTIVE = "adaptive"


class WaypointInterpolator:
    """
    Simplified path의 waypoint를 보간하는 클래스
    MPC의 성능 향상을 위한 최적화
    """
    
    def __init__(self, min_spacing: float = 0.1, max_spacing: float = 0.5):
        """
        Args:
            min_spacing: waypoint 간 최소 간격 (m) - 곡선 구간
            max_spacing: waypoint 간 최대 간격 (m) - 직선 구간
        """
        self.min_spacing = min_spacing
        self.max_spacing = max_spacing
        
    def interpolate(self, 
                   waypoints: Union[List[Tuple[float, float]], np.ndarray],
                   method: InterpolationType = InterpolationType.ADAPTIVE,
                   target_spacing: Optional[float] = None) -> List[Tuple[float, float]]:
        """
        메인 인터폴레이션 함수
        
        Args:
            waypoints: 원본 waypoint 리스트 [(x,y), ...] 또는 numpy array
            method: 보간 방법
            target_spacing: 목표 간격 (None이면 자동 계산)
            
        Returns:
            보간된 waypoint 리스트 [(x,y), ...]
        """
        # 입력 데이터 정규화
        points = self._normalize_input(waypoints)
        
        if len(points) < 2:
            return self._to_tuple_list(points)
        
        # 보간 방법 선택
        if method == InterpolationType.LINEAR:
            result = self._linear_interpolation(points, target_spacing)
        elif method == InterpolationType.CUBIC_SPLINE:
            result = self._cubic_spline_interpolation(points, target_spacing)
        elif method == InterpolationType.ADAPTIVE:
            result = self._adaptive_interpolation(points)
        else:
            result = self._linear_interpolation(points, target_spacing)
        
        return self._to_tuple_list(result)
    
    def _normalize_input(self, waypoints: Union[List[Tuple[float, float]], np.ndarray]) -> np.ndarray:
        """입력 데이터를 numpy array로 변환"""
        if isinstance(waypoints, np.ndarray):
            return waypoints
        elif isinstance(waypoints, list):
            return np.array(waypoints)
        else:
            raise TypeError("waypoints must be list or numpy array")
    
    def _to_tuple_list(self, points: np.ndarray) -> List[Tuple[float, float]]:
        """numpy array를 tuple 리스트로 변환"""
        return [(float(p[0]), float(p[1])) for p in points]
    
    def _linear_interpolation(self, points: np.ndarray, 
                             target_spacing: Optional[float] = None) -> np.ndarray:
        """
        선형 보간 (가장 간단하고 빠름)
        
        Args:
            points: N x 2 numpy array
            target_spacing: waypoint 간 목표 간격
            
        Returns:
            보간된 waypoints
        """
        if target_spacing is None:
            target_spacing = self.min_spacing
            
        interpolated = []
        
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            
            # 두 점 사이의 거리 계산
            distance = np.linalg.norm(p2 - p1)
            
            # 필요한 보간점 개수 계산
            n_segments = max(1, int(np.ceil(distance / target_spacing)))
            
            # 선형 보간
            for j in range(n_segments):
                t = j / n_segments
                interpolated_point = p1 + t * (p2 - p1)
                interpolated.append(interpolated_point)
        
        # 마지막 점 추가
        interpolated.append(points[-1])
        
        return np.array(interpolated)
    
    def _cubic_spline_interpolation(self, points: np.ndarray,
                                   target_spacing: Optional[float] = None) -> np.ndarray:
        """
        Cubic Spline을 사용한 부드러운 보간
        
        Args:
            points: N x 2 numpy array
            target_spacing: waypoint 간 목표 간격
            
        Returns:
            부드럽게 보간된 waypoints
        """
        try:
            from scipy.interpolate import CubicSpline
        except ImportError:
            print("[WaypointInterpolator] scipy not installed. Using linear interpolation.")
            return self._linear_interpolation(points, target_spacing)
        
        if target_spacing is None:
            target_spacing = self.min_spacing
        
        # 누적 거리를 매개변수로 사용
        distances = self._calculate_cumulative_distances(points)
        
        # 각 차원에 대해 Cubic Spline 생성
        cs_x = CubicSpline(distances, points[:, 0])
        cs_y = CubicSpline(distances, points[:, 1])
        
        # 새로운 샘플링 포인트 생성
        total_distance = distances[-1]
        n_points = max(2, int(np.ceil(total_distance / target_spacing)) + 1)
        s_new = np.linspace(0, total_distance, n_points)
        
        # 보간
        interpolated = np.column_stack([cs_x(s_new), cs_y(s_new)])
        
        return interpolated
    
    def _adaptive_interpolation(self, points: np.ndarray) -> np.ndarray:
        """
        곡률 기반 적응형 보간
        곡선 구간은 촘촘하게, 직선 구간은 듬성듬성하게
        
        Args:
            points: N x 2 numpy array
            
        Returns:
            적응적으로 보간된 waypoints
        """
        interpolated = []
        
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            
            # 곡률 추정 (앞뒤 점을 이용)
            curvature = self._estimate_local_curvature(points, i)
            
            # 곡률에 따른 간격 결정
            if curvature > 0.5:  # 급커브
                target_spacing = self.min_spacing
            elif curvature > 0.1:  # 완만한 커브
                target_spacing = (self.min_spacing + self.max_spacing) / 2
            else:  # 직선
                target_spacing = self.max_spacing
            
            # 두 점 사이 거리
            distance = np.linalg.norm(p2 - p1)
            
            # 보간점 개수 계산
            n_segments = max(1, int(np.ceil(distance / target_spacing)))
            
            # 보간
            for j in range(n_segments):
                t = j / n_segments
                interpolated_point = p1 + t * (p2 - p1)
                interpolated.append(interpolated_point)
        
        # 마지막 점 추가
        interpolated.append(points[-1])
        
        return np.array(interpolated)
    
    def _calculate_cumulative_distances(self, points: np.ndarray) -> np.ndarray:
        """누적 거리 계산"""
        distances = [0]
        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i-1])
            distances.append(distances[-1] + dist)
        return np.array(distances)
    
    def _estimate_local_curvature(self, points: np.ndarray, index: int) -> float:
        """
        지역 곡률 추정 (3점을 이용한 Menger 곡률)
        
        Args:
            points: 전체 경로점
            index: 현재 인덱스
            
        Returns:
            추정된 곡률 값
        """
        # 경계 처리
        if index == 0:
            if len(points) < 3:
                return 0
            p1, p2, p3 = points[0], points[1], points[2]
        elif index >= len(points) - 2:
            if len(points) < 3:
                return 0
            p1, p2, p3 = points[-3], points[-2], points[-1]
        else:
            p1, p2, p3 = points[index], points[index + 1], points[index + 2]
        
        # Menger 곡률 계산
        return self._menger_curvature(p1, p2, p3)
    
    def _menger_curvature(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
        """
        세 점으로부터 Menger 곡률 계산
        
        Returns:
            곡률 (0: 직선, 큰 값: 급커브)
        """
        # 삼각형 변의 길이
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)
        
        # 거의 일직선인 경우
        if a < 1e-10 or b < 1e-10 or c < 1e-10:
            return 0
        
        # 헤론의 공식으로 면적 계산
        s = (a + b + c) / 2  # 반둘레
        area_squared = s * (s - a) * (s - b) * (s - c)
        
        if area_squared <= 0:
            return 0
        
        area = np.sqrt(area_squared)
        
        # Menger 곡률 = 4 * area / (a * b * c)
        curvature = 4 * area / (a * b * c)
        
        return curvature
    
    def interpolate_with_velocity_profile(self, 
                                         waypoints: np.ndarray,
                                         max_velocity: float = 2.0,
                                         max_acceleration: float = 1.0) -> Tuple[List[Tuple[float, float]], np.ndarray]:
        """
        속도 프로파일을 고려한 보간
        
        Args:
            waypoints: 원본 waypoints
            max_velocity: 최대 속도 (m/s)
            max_acceleration: 최대 가속도 (m/s^2)
            
        Returns:
            (interpolated_waypoints, velocity_profile)
        """
        # 먼저 적응형 보간 수행
        points = self._normalize_input(waypoints)
        interpolated = self._adaptive_interpolation(points)
        
        # 각 점에서의 속도 프로파일 계산
        velocities = []
        
        for i in range(len(interpolated)):
            # 곡률 기반 속도 제한
            curvature = self._estimate_local_curvature(interpolated, i)
            
            # 곡률이 클수록 속도 감소
            if curvature > 0.5:
                v_max_curve = max_velocity * 0.3
            elif curvature > 0.2:
                v_max_curve = max_velocity * 0.6
            else:
                v_max_curve = max_velocity
            
            velocities.append(v_max_curve)
        
        # 가속도 제약을 고려한 속도 스무딩
        velocities = self._smooth_velocity_profile(
            velocities, 
            interpolated,
            max_acceleration
        )
        
        return self._to_tuple_list(interpolated), np.array(velocities)
    
    def _smooth_velocity_profile(self, velocities: List[float], 
                                waypoints: np.ndarray,
                                max_acceleration: float) -> List[float]:
        """가속도 제약을 고려한 속도 프로파일 스무딩"""
        smoothed = velocities.copy()
        dt = 0.1  # 가정: 제어 주기
        
        # Forward pass: 가속 제약
        for i in range(1, len(smoothed)):
            dist = np.linalg.norm(waypoints[i] - waypoints[i-1])
            if dist > 0:
                dt = dist / max(smoothed[i-1], 0.1)
                max_next_vel = smoothed[i-1] + max_acceleration * dt
                smoothed[i] = min(smoothed[i], max_next_vel)
        
        # Backward pass: 감속 제약
        for i in range(len(smoothed) - 2, -1, -1):
            dist = np.linalg.norm(waypoints[i+1] - waypoints[i])
            if dist > 0:
                dt = dist / max(smoothed[i], 0.1)
                max_prev_vel = smoothed[i+1] + max_acceleration * dt
                smoothed[i] = min(smoothed[i], max_prev_vel)
        
        return smoothed