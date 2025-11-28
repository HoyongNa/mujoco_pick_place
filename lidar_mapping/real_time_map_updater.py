"""
실시간 맵 업데이터 모듈 (최적화 버전)
- lidar_mapping 모듈의 기존 기능 최대한 활용
- 중복 코드 제거
- 맵 병합 기능에만 집중
"""

import threading
import time
import numpy as np
from typing import Optional, Tuple, Dict, Any
from datetime import datetime
import mujoco

# LidarMappingSystem 사용 (항상 사용)
from .mapping_system import LidarMappingSystem
from .occupancy_grid import OccupancyGrid, OccupancyGridMap


class RealTimeMapUpdater:
    """실시간 맵 업데이터 - 최적화된 버전
    
    기존 lidar_mapping 모듈의 기능을 최대한 활용하여
    중복 코드를 제거하고 맵 병합 기능에 집중
    """
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 map_size: Tuple[int, int] = (200, 200),
                 resolution: float = 0.05,
                 update_rate: float = 10.0):
        """
        Args:
            model: MuJoCo 모델
            data: MuJoCo 데이터
            base_lock: 베이스 제어 락 (위치 읽기용)
            map_size: 맵 크기 (height, width)
            resolution: 맵 해상도 (meters/cell)
            update_rate: 업데이트 주기 (Hz)
        """
        self.model = model
        self.data = data
        
        # LidarMappingSystem 사용 (모든 기능 포함)
        self.mapping_system = LidarMappingSystem(
            model, data, 
            map_size=map_size, 
            resolution=resolution
        )
        
        # 원본 맵 저장용 (병합 기능)
        self.original_map: Optional[np.ndarray] = None
        self.original_resolution: float = resolution
        self.original_origin: Tuple[float, float] = (-5.0, -5.0)
        
        # 업데이트 관리
        self.is_updating = False
        self.update_count = 0
        self.update_rate = update_rate
        self.last_saved_filename: Optional[str] = None
        
        print(f"[RealTimeMapUpdater] 초기화 완료 (최적화 버전)")
        print("[RealTimeMapUpdater] LidarMappingSystem 활용 모드")
    
    def load_base_map(self, map_data: dict) -> bool:
        """기존 맵을 베이스로 로드
        
        Args:
            map_data: {'occupancy_grid': np.ndarray, 'resolution': float, 'origin': tuple}
        
        Returns:
            성공 여부
        """
        try:
            # 원본 맵 저장 (병합용)
            self.original_map = map_data.get('occupancy_grid', None)
            self.original_resolution = map_data.get('resolution', 0.05)
            self.original_origin = map_data.get('origin', (-5.0, -5.0))
            
            if self.original_map is not None:
                # 원본 맵을 log-odds로 변환
                if self.original_map.dtype == np.uint8:
                    prob_map = self.original_map.astype(np.float32) / 255.0
                else:
                    prob_map = self.original_map.astype(np.float32)
                
                # 0과 1 값 클리핑 (log 계산 안정성)
                prob_map = np.clip(prob_map, 0.01, 0.99)
                log_odds = np.log(prob_map / (1.0 - prob_map))
                log_odds = np.clip(log_odds, -4.0, 4.0)
                
                # LidarMappingSystem의 grid_map에 직접 설정
                self.mapping_system.grid_map.log_odds = log_odds
                self.mapping_system.grid_map.origin = self.original_origin
                self.mapping_system.grid_map.resolution = self.original_resolution
                
                print(f"[RealTimeMapUpdater] 베이스 맵 로드 완료 - 크기: {self.original_map.shape}")
                return True
            else:
                print("[RealTimeMapUpdater] 베이스 맵 데이터가 없습니다.")
                return False
                
        except Exception as e:
            print(f"[RealTimeMapUpdater] 베이스 맵 로드 실패: {e}")
            return False
    
    def start_updating(self):
        """맵 업데이트 시작"""
        if self.is_updating:
            print("[RealTimeMapUpdater] 이미 업데이트 중입니다.")
            return
        
        # LidarMappingSystem의 내장 스레드 사용
        self.mapping_system.start_mapping(update_rate=self.update_rate)
        self.is_updating = True
        print(f"[RealTimeMapUpdater] LidarMappingSystem 맵핑 시작 ({self.update_rate}Hz)")
        
        # 주기적 상태 출력을 위한 모니터링 스레드 시작
        self._monitor_thread = threading.Thread(
            target=self._monitor_updates,
            daemon=True
        )
        self._monitor_thread.start()
    
    def stop_updating(self):
        """맵 업데이트 중지"""
        if not self.is_updating:
            return
        
        self.is_updating = False  # 모니터링 종료 신호
        
        # LidarMappingSystem 정지
        self.mapping_system.stop_mapping()
        self.update_count = self.mapping_system.total_updates
        # 모니터링 스레드 종료 대기
        if hasattr(self, '_monitor_thread'):
            self._monitor_thread.join(timeout=1.0)
        
        print(f"[RealTimeMapUpdater] 맵 업데이트 중지 (총 {self.get_update_count()}회 업데이트)")
    
    def _monitor_updates(self):
        """주기적으로 업데이트 상태를 출력하는 모니터링 스레드"""
        last_count = 0
        milestones = [50, 100, 200, 300, 500, 1000]  # 출력할 마일스톤
        milestone_idx = 0
        
        while self.is_updating:
            time.sleep(1.0)  # 1초마다 체크
            
            current_count = self.get_update_count()
            
            # 마일스톤 도달 체크
            if milestone_idx < len(milestones) and current_count >= milestones[milestone_idx]:
                self._print_progress(current_count, milestones[milestone_idx])
                milestone_idx += 1
            # 50회마다 계속 출력 (1000회 이후)
            elif current_count > 1000 and current_count % 50 == 0 and current_count != last_count:
                self._print_progress(current_count)
                last_count = current_count
    
    def _print_progress(self, update_count: int, milestone: Optional[int] = None):
        """진행 상황 출력"""
        stats = self.get_statistics()
        
        if milestone:
            print(f"\n{'='*60}")
            print(f"🎯 마일스톤 달성: {milestone}회 업데이트")
        else:
            print(f"\n{'-'*60}")
            print(f"📊 업데이트 진행: {update_count}회")
        
        print(f"   • 점유 셀 (장애물): {stats.get('occupied_cells', 0):,}개")
        print(f"   • 자유 셀 (통행가능): {stats.get('free_cells', 0):,}개")
        print(f"   • 미탐색 셀: {stats.get('unknown_cells', 0):,}개")
        
        # 탐색률 계산
        total = stats.get('total_cells', 1)
        explored = stats.get('occupied_cells', 0) + stats.get('free_cells', 0)
        coverage = (explored / total) * 100 if total > 0 else 0
        print(f"   • 탐색률: {coverage:.1f}% ({explored:,}/{total:,} 셀)")
        
        if milestone:
            print(f"{'='*60}")
        else:
            print(f"{'-'*60}")
    
    def get_update_count(self) -> int:
        """현재까지의 업데이트 횟수"""
        return self.mapping_system.total_updates
    
    def get_current_map(self) -> np.ndarray:
        """현재 맵 가져오기 (병합 없이)"""
        return self.mapping_system.get_probability_map()
    
    def get_merged_map(self) -> np.ndarray:
        """원본 맵과 업데이트된 맵을 병합하여 반환
        
        Returns:
            병합된 점유 격자 맵 (0-255 uint8)
        """
        # 현재 확률 맵
        current_prob_map = self.get_current_map()
        
        if self.original_map is not None:
            # 원본과 병합
            merged = self._merge_maps(self.original_map, current_prob_map)
        else:
            merged = current_prob_map
        
        # uint8로 변환
        if merged.dtype != np.uint8:
            merged_uint8 = np.clip(merged * 255, 0, 255).astype(np.uint8)
        else:
            merged_uint8 = merged
            
        return merged_uint8
    
    def _merge_maps(self, original: np.ndarray, updated: np.ndarray) -> np.ndarray:
        """원본 맵과 업데이트된 맵을 지능적으로 병합
        
        이 메서드는 RealTimeMapUpdater의 고유 기능으로 유지
        """
        # 원본 맵 정규화
        if original.dtype == np.uint8:
            original_norm = original.astype(np.float32) / 255.0
        else:
            original_norm = original.astype(np.float32)
        
        # 크기 맞추기
        if original_norm.shape != updated.shape:
            from scipy import ndimage
            scale_y = updated.shape[0] / original_norm.shape[0]
            scale_x = updated.shape[1] / original_norm.shape[1]
            original_norm = ndimage.zoom(original_norm, (scale_y, scale_x), order=1)
        
        # 가중 병합 (고유 로직)
        merged = original_norm.copy()
        
        # 확실한 영역만 업데이트
        high_occupied = updated > 0.8
        merged[high_occupied] = updated[high_occupied]
        
        high_free = updated < 0.2
        merged[high_free] = updated[high_free]
        
        # 불확실한 영역은 가중 평균
        uncertain = ~(high_occupied | high_free)
        alpha = 0.7
        merged[uncertain] = alpha * updated[uncertain] + (1 - alpha) * original_norm[uncertain]
        
        return merged
    
    def sync_to_map_processor(self, map_processor) -> dict:
        """MapProcessor에 업데이트된 맵을 동기화"""
        # 병합된 맵
        merged_map = self.get_merged_map()
        
        # MapProcessor 업데이트
        map_processor.occupancy_grid = merged_map
        map_processor.resolution = self.mapping_system.grid_map.resolution
        map_processor.origin = self.mapping_system.grid_map.origin
        
        # 장애물 팽창
        map_processor.dilate_obstacles(radius=6)
        
        return {
            'occupancy_grid': merged_map,
            'resolution': map_processor.resolution,
            'origin': map_processor.origin
        }
    
    def save_updated_map(self, filename: Optional[str] = None) -> str:
        """업데이트된 맵을 파일로 저장"""
        # LidarMappingSystem의 save_map 활용
        if filename is None:
            saved_path = self.mapping_system.save_map("realtime_map")
        else:
            saved_path = self.mapping_system.save_map(filename.replace('.npz', ''))
        
        self.last_saved_filename = saved_path
        print(f"[RealTimeMapUpdater] 맵 저장 완료: {saved_path} "
              f"(업데이트 {self.get_update_count()}회)")
        
        return saved_path
    
    def get_statistics(self) -> dict:
        """맵 업데이트 통계 반환"""
        # LidarMappingSystem의 통계 사용
        stats = self.mapping_system.get_statistics()
        
        # 공통 통계 추가
        update_count = self.get_update_count()
        stats.update({
            'update_count': update_count,
            'updates': update_count,  # 호환성
            'is_updating': self.is_updating,
            'has_base_map': self.original_map is not None,
            'last_saved': self.last_saved_filename
        })
        
        return stats
    
    def reset(self):
        """맵을 원본 상태로 리셋"""
        self.mapping_system.clear_map()
        self.update_count = 0
        
        # 원본 맵이 있으면 다시 로드
        if self.original_map is not None:
            self.load_base_map({
                'occupancy_grid': self.original_map,
                'resolution': self.original_resolution,
                'origin': self.original_origin
            })
        
        print("[RealTimeMapUpdater] 맵 리셋 완료")
    
    def get_map_for_planning(self) -> dict:
        """경로 계획용 맵 데이터 반환"""
        merged_map = self.get_merged_map()
        
        return {
            'occupancy_grid': merged_map,
            'resolution': self.mapping_system.grid_map.resolution,
            'origin': self.mapping_system.grid_map.origin
        }
    
    def update_lidar_obstacles(self, lidar_sensor, map_processor, 
                               robot_base_qpos_idx, max_obstacles: int = 30,
                               min_dist: float = 0.3, max_dist: float = 3.0):
        """실시간 라이다 장애물 업데이트 (동적 장애물만)
        
        Args:
            lidar_sensor: LidarSensor 인스턴스
            map_processor: MapProcessor 인스턴스 (정적 맵 확인용)
            robot_base_qpos_idx: 로봇 베이스 위치 인덱스 (slice object)
            max_obstacles: 최대 장애물 개수
            min_dist: 최소 감지 거리 (m)
            max_dist: 최대 감지 거리 (m)
        
        Returns:
            동적 장애물 리스트 [(x, y), ...]
        """
        scan_data = lidar_sensor.get_scan()
        
        if scan_data is None or scan_data["num_valid"] == 0:
            return []
        
        # 로봇 위치
        base_pos = self.data.qpos[robot_base_qpos_idx]
        robot_x = base_pos[0]
        robot_y = base_pos[1]
        
        obstacles = []
        hit_points = scan_data["points"]
        
        # 현재 업데이트된 맵 가져오기
        current_map = self.get_current_map()
        
        if hit_points.size > 0:
            for point in hit_points:
                px = point[0]
                py = point[1]
                
                dist = np.sqrt((px - robot_x)**2 + (py - robot_y)**2)
                
                # 거리 필터링
                if min_dist < dist < max_dist:
                    # 업데이트된 맵에 없는 장애물만 추가
                    if not self._is_in_static_map(px, py, current_map, map_processor):
                        obstacles.append((px, py))
            
            if obstacles:
                # 로봇과의 거리 기준으로 정렬
                obstacles.sort(key=lambda p: (p[0]-robot_x)**2 + (p[1]-robot_y)**2)
                return obstacles[:max_obstacles]
        
        return []
    
    def _is_in_static_map(self, x: float, y: float, current_map: np.ndarray, 
                          map_processor) -> bool:
        """현재 업데이트된 맵에 장애물이 있는지 확인
        
        Args:
            x: 월드 x 좌표
            y: 월드 y 좌표
            current_map: 현재 점유 맵
            map_processor: MapProcessor 인스턴스 (origin과 resolution 사용)
        
        Returns:
            정적 맵에 장애물이 있으면 True
        """
        # map_processor의 origin과 resolution 사용
        map_x = int((x - map_processor.origin[0]) / map_processor.resolution)
        map_y = int((y - map_processor.origin[1]) / map_processor.resolution)
        
        # 맵 범위 체크
        if (0 <= map_x < current_map.shape[0] and 
            0 <= map_y < current_map.shape[1]):
            # 확장된 영역도 고려 (장애물 주변 팽창 영역)
            # 정확한 점이 아니라 주변 셀도 확인
            check_radius = 2  # 주변 2셀 확인
            
            for dx in range(-check_radius, check_radius + 1):
                for dy in range(-check_radius, check_radius + 1):
                    nx, ny = map_x + dx, map_y + dy
                    if (0 <= nx < current_map.shape[0] and 
                        0 <= ny < current_map.shape[1]):
                        # 맵에 장애물로 표시되어 있으면 정적 장애물
                        if current_map[nx, ny] > 0.5:
                            return True
            
            return False
        
        return False
