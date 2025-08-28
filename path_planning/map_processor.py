"""
맵 처리 모듈 - 수정 버전
Lidar 맵을 로드하고 경로 계획을 위한 occupancy grid로 변환
"""

import numpy as np
import cv2
import os


class MapProcessor:
    """Lidar 맵 데이터 처리"""
    
    def __init__(self, map_path=None):
        """
        Args:
            map_path: .npz 맵 파일 경로. None이면 기본 파일 사용
        """
        self.map_path = map_path
        self.log_odds = None
        self.resolution = 0.05  # 기본 해상도 5cm/cell
        self.origin = (0, 0)  # 기본 원점
        self.occupancy_grid = None
        self.dilated_map = None  # 팽창된 맵 (장애물 회피용)
        self.map_shape = None
        
        # 맵 파일이 지정되었으면 자동 로드
        if self.map_path:
            self.load_map(self.map_path)
    
    def load_map(self, map_path=None):
        """맵 파일 로드
        
        Args:
            map_path: .npz 파일 경로. None이면 초기화 시 설정된 경로 사용
        
        Returns:
            성공 여부
        """
        if map_path:
            self.map_path = map_path
            
        if not self.map_path:
            # 기본 맵 파일 찾기
            default_maps = [
                "lidar_map_20250826_215447.npz",
                "default_map.npz",
                "default_4room_map.npz",
                "simple_test_map.npz",
                "test_map.npz"
            ]
            
            for default_map in default_maps:
                if os.path.exists(default_map):
                    self.map_path = default_map
                    print(f"[MapProcessor] 기본 맵 파일 발견: {default_map}")
                    break
            else:
                print(f"[MapProcessor] 맵 파일을 찾을 수 없습니다.")
                return False
        
        if not os.path.exists(self.map_path):
            print(f"[MapProcessor] 맵 파일이 존재하지 않습니다: {self.map_path}")
            return False
            
        try:
            # NPZ 파일 로드
            npz = np.load(self.map_path, allow_pickle=False)
            
            # 파일에 있는 키 확인
            available_keys = list(npz.keys())
            print(f"[MapProcessor] 맵 파일 키: {available_keys}")
            
            # occupancy_grid가 직접 저장된 경우 (간단한 맵)
            if "occupancy_grid" in available_keys:
                self.occupancy_grid = npz["occupancy_grid"].astype(np.uint8)
                
                # 해상도와 원점 정보
                if "resolution" in available_keys:
                    self.resolution = float(npz["resolution"])
                else:
                    self.resolution = 0.1  # 기본값
                    
                if "origin" in available_keys:
                    origin_raw = npz["origin"]
                    self.origin = (float(origin_raw[0]), float(origin_raw[1]))
                else:
                    # 기본: 맵 중앙이 (0,0)
                    h, w = self.occupancy_grid.shape
                    self.origin = (-w * self.resolution / 2.0, -h * self.resolution / 2.0)
                    
                self.map_shape = self.occupancy_grid.shape
                
            # log_odds가 저장된 경우 (lidar 맵)
            elif "log_odds" in available_keys:
                self.log_odds = npz["log_odds"].astype(np.float32)
                
                # 해상도와 원점 정보
                if "resolution" in available_keys:
                    self.resolution = float(npz["resolution"])
                else:
                    self.resolution = 0.05  # 기본값
                
                if "origin" in available_keys:
                    origin_raw = npz["origin"]
                    self.origin = (float(origin_raw[0]), float(origin_raw[1]))
                else:
                    # 기본: 맵 중앙이 (0,0)
                    h, w = self.log_odds.shape
                    self.origin = (-w * self.resolution / 2.0, -h * self.resolution / 2.0)
                
                # Occupancy grid 생성
                self._create_occupancy_grid()
                
            else:
                print(f"[MapProcessor] 맵 데이터를 찾을 수 없습니다. 사용 가능한 키: {available_keys}")
                return False
            
            # 장애물 맵 팽창
            self._dilate_obstacles()
            
            print(f"[MapProcessor] 맵 로드 완료: {self.map_path}")
            print(f"  - Shape: {self.map_shape} (H x W)")
            print(f"  - Resolution: {self.resolution} m/cell")
            print(f"  - Origin: {self.origin}")
            
            return True
            
        except Exception as e:
            print(f"[MapProcessor] 맵 로드 실패: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _create_occupancy_grid(self):
        """Log-odds를 occupancy grid로 변환"""
        if self.log_odds is None:
            return
            
        # Sigmoid 함수로 확률 변환
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))
        
        # 0: free (흰색), 255: occupied (검은색)
        # 간단한 이진화
        self.occupancy_grid = np.zeros_like(prob, dtype=np.uint8)
        
        # 확률 > 0.5 → 장애물
        self.occupancy_grid[prob >= 0.5] = 255
        
        self.map_shape = self.occupancy_grid.shape
        
    def _dilate_obstacles(self):
        """장애물 팽창 (로봇 크기 고려)"""
        if self.occupancy_grid is None:
            return
            
        # 로봇 반경 + 안전 마진을 셀 단위로 변환
        robot_radius = 0.35  # meters (로봇 실제 크기 - 조금 줄임)
        safety_margin = 0.4  # meters (추가 안전 마진 - 줄임)
        total_radius = robot_radius + safety_margin  # 총 0.4m
        dilation_radius = int(total_radius / self.resolution)
        
        print(f"[MapProcessor] 장애물 팽창 설정:")
        print(f"  - 로봇 반경: {robot_radius}m")
        print(f"  - 안전 마진: {safety_margin}m")
        print(f"  - 총 팽창 반경: {total_radius}m ({dilation_radius} cells)")
        
        # 커널 생성 (원형)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (2*dilation_radius+1, 2*dilation_radius+1)
        )
        
        # 팽창 (1번만 적용)
        self.dilated_map = cv2.dilate(self.occupancy_grid, kernel, iterations=1)
        
    def world_to_grid(self, x, y):
        """월드 좌표를 그리드 좌표로 변환
        
        Args:
            x, y: 월드 좌표 (meters)
            
        Returns:
            (grid_x, grid_y): 그리드 좌표
        """
        grid_x = int((x - self.origin[0]) / self.resolution)
        grid_y = int((y - self.origin[1]) / self.resolution)
        return (grid_x, grid_y)
        
    def grid_to_world(self, grid_x, grid_y):
        """그리드 좌표를 월드 좌표로 변환
        
        Args:
            grid_x, grid_y: 그리드 좌표
            
        Returns:
            (x, y): 월드 좌표 (meters)
        """
        x = grid_x * self.resolution + self.origin[0]
        y = grid_y * self.resolution + self.origin[1]
        return (x, y)
        
    def is_valid(self, grid_x, grid_y):
        """그리드 좌표가 맵 범위 내인지 확인"""
        if self.map_shape is None:
            return False
        h, w = self.map_shape
        return 0 <= grid_y < h and 0 <= grid_x < w
        
    def is_free(self, grid_x, grid_y, use_dilated=True):
        """그리드 좌표가 자유 공간인지 확인
        
        Args:
            grid_x, grid_y: 그리드 좌표
            use_dilated: 팽창된 맵 사용 여부
        """
        if not self.is_valid(grid_x, grid_y):
            return False
            
        if use_dilated and self.dilated_map is not None:
            return self.dilated_map[grid_y, grid_x] == 0
        elif self.occupancy_grid is not None:
            return self.occupancy_grid[grid_y, grid_x] == 0
        else:
            return False
            
    def visualize(self, path=None, start=None, goal=None):
        """맵 시각화
        
        Args:
            path: 경로 (옵션)
            start: 시작점 (옵션)
            goal: 목표점 (옵션)
        """
        import matplotlib.pyplot as plt
        
        if self.occupancy_grid is None:
            print("[MapProcessor] 시각화할 맵이 없습니다.")
            return
            
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # 맵 표시 (검은색: 장애물, 흰색: 자유 공간)
        ax.imshow(self.occupancy_grid, cmap='gray_r', origin='lower')
        
        # 시작점과 목표점 표시
        if start:
            start_grid = self.world_to_grid(start[0], start[1])
            ax.plot(start_grid[0], start_grid[1], 'go', markersize=10, label='Start')
            
        if goal:
            goal_grid = self.world_to_grid(goal[0], goal[1])
            ax.plot(goal_grid[0], goal_grid[1], 'ro', markersize=10, label='Goal')
            
        # 경로 표시
        if path:
            path_x = []
            path_y = []
            for point in path:
                grid_point = self.world_to_grid(point[0], point[1])
                path_x.append(grid_point[0])
                path_y.append(grid_point[1])
            ax.plot(path_x, path_y, 'b-', linewidth=2, label='Path')
            
        ax.set_title('Occupancy Grid Map')
        ax.set_xlabel('X (grid cells)')
        ax.set_ylabel('Y (grid cells)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
