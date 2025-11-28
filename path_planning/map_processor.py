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
        self.map_path = map_path
        self.log_odds = None
        self.resolution = 0.05
        self.origin = (0, 0)
        self.occupancy_grid = None
        self.dilated_map = None        # Keep for backward compatibility
        self.dilated_grid = None       # ✅ AsyncMPCPlanner expects this name
        self.map_shape = None
        self.width = 0                 # ✅ AsyncMPCPlanner needs width
        self.height = 0                # ✅ AsyncMPCPlanner needs height
        self.loaded = False            # ✅ AsyncMPCPlanner checks this
        
        if self.map_path:
            self.load_map(self.map_path)
    
    def process_map(self, map_array, resolution=0.1, origin=None):
        """numpy 배열로 주어진 맵 데이터를 처리
        
        Args:
            map_array: numpy 배열 (0: free, 1 or 255: occupied)
            resolution: 해상도 (meters/cell)
            origin: 맵 원점 (x, y). None이면 중앙을 원점으로
            
        Returns:
            성공 여부
        """
        try:
            # 입력 배열 정규화
            map_array = np.asarray(map_array, dtype=np.float32)
            
            # 0-1 범위로 정규화
            if map_array.max() > 1.0:
                map_array = map_array / 255.0
            
            # occupancy grid 생성 (0: free, 255: occupied)
            self.occupancy_grid = np.zeros_like(map_array, dtype=np.uint8)
            self.occupancy_grid[map_array >= 0.5] = 255
            
            # 메타데이터 설정
            self.resolution = float(resolution)
            self.map_shape = self.occupancy_grid.shape
            
            if origin is None:
                # 맵 중앙을 원점으로
                h, w = self.map_shape
                self.origin = (-w * self.resolution / 2.0, -h * self.resolution / 2.0)
            else:
                self.origin = (float(origin[0]), float(origin[1]))
            
            # 장애물 맵 팽창
            # 주의: load_map에서 이미 팽창하므로 여기서는 스킵
            # self._dilate_obstacles()
            pass  # 팽창은 LidarIntegratedController에서 한 번만 수행
            
            # print(f"[MapProcessor] 맵 처리 완료")
            # print(f"  - Shape: {self.map_shape} (H x W)")
            # print(f"  - Resolution: {self.resolution} m/cell")
            # print(f"  - Origin: {self.origin}")
            
            return True
            
        except Exception as e:
            print(f"[MapProcessor] 맵 처리 실패: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def load_map(self, map_path=None, auto_dilate=True):
        """맵 파일 로드
        
        Args:
            map_path: .npz 파일 경로. None이면 초기화 시 설정된 경로 사용
            auto_dilate: 자동으로 장애물 팽창 수행 여부 (기본: True)
        
        Returns:
            성공 여부
        """
        if map_path:
            self.map_path = map_path
            
        if not os.path.exists(self.map_path):
            print(f"[MapProcessor] 맵 파일이 존재하지 않습니다: {self.map_path}")
            return False
            
        try:
            # NPZ 파일 로드
            npz = np.load(self.map_path, allow_pickle=False)
            
            # 파일에 있는 키 확인
            available_keys = list(npz.keys())
            # print(f"[MapProcessor] 맵 파일 키: {available_keys}")
            
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
            
            # 장애물 맵 팽창 (auto_dilate가 True일 때만)
            if auto_dilate:
                self._dilate_obstacles()
            
            # ✅ Set AsyncMPCPlanner compatibility attributes
            if self.map_shape is not None:
                self.height, self.width = self.map_shape  # map_shape is (H, W)
            
            # ✅ Sync dilated_grid with dilated_map
            if self.dilated_map is not None:
                self.dilated_grid = self.dilated_map
            
            # ✅ Mark map as successfully loaded
            self.loaded = True
            
            # print(f"[MapProcessor] 맵 로드 완료: {self.map_path}")
            # print(f"  - Shape: {self.map_shape} (H x W)")
            # print(f"  - Width x Height: {self.width} x {self.height}")
            # print(f"  - Resolution: {self.resolution} m/cell")
            # print(f"  - Origin: {self.origin}")
            # print(f"  - Loaded: {self.loaded}")
            
            # 맵 통계 출력
            free_cells = np.sum(self.occupancy_grid == 0)
            occupied_cells = np.sum(self.occupancy_grid == 255)
            total_cells = self.occupancy_grid.size
            # print(f"  - 자유 공간: {free_cells} 셀 ({free_cells/total_cells*100:.1f}%)")
            # print(f"  - 장애물: {occupied_cells} 셀 ({occupied_cells/total_cells*100:.1f}%)")
            
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
            
        # 로봇 반경 + 안전 마진을 셀 단위로 변환 (크게 증가)
        robot_radius = 0.3  # meters (로봇 실제 크기 - 증가)
        safety_margin = 0.3  # meters (추가 안전 마진 - 크게 증가)
        total_radius = robot_radius + safety_margin  # 총 1.0m
        dilation_radius = int(total_radius / self.resolution)
        
        # print(f"[MapProcessor] 장애물 팽창 설정:")
        # print(f"  - 로봇 반경: {robot_radius}m")
        # print(f"  - 안전 마진: {safety_margin}m")
        # print(f"  - 총 팽창 반경: {total_radius}m ({dilation_radius} cells)")
        
        # 커널 생성 (원형)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (2*dilation_radius+1, 2*dilation_radius+1)
        )
        
        # 팽창 (1번만 적용)
        self.dilated_map = cv2.dilate(self.occupancy_grid, kernel, iterations=1)
        self.dilated_grid = self.dilated_map  # ✅ Sync for AsyncMPCPlanner
    
    
    def visualize_dilated_map(self):
        """팽창된 맵을 시각화하여 문 영역 확인"""
        if self.dilated_map is None:
            print("[MapProcessor] 팽창된 맵이 없습니다")
            return
            
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(1, 2, figsize=(12, 6))
        
        # 원본 맵
        axes[0].imshow(self.occupancy_grid, cmap='gray')
        axes[0].set_title('원본 맵')
        axes[0].axis('off')
        
        # 팽창된 맵
        axes[1].imshow(self.dilated_map, cmap='gray')
        axes[1].set_title('팽창된 맵 (문 영역 보호)')
        axes[1].axis('off')
        
        # 문 위치 표시
        door_world_positions = [
            (0.0, -1.0, '아래'),
            (0.0, 1.0, '위'),
            (-1.0, 0.0, '왼쪽'),
            (1.0, 0.0, '오른쪽')
        ]
        
        for wx, wy, label in door_world_positions:
            gx, gy = self.world_to_grid(wx, wy)
            axes[1].plot(gx, gy, 'r*', markersize=15)
            axes[1].text(gx, gy-5, label, color='red', fontsize=10, ha='center')
        
        plt.tight_layout()
        plt.show()
        
        # 통계 출력
        free_orig = np.sum(self.occupancy_grid < 127)
        free_dilated = np.sum(self.dilated_map < 127)
        # print(f"\n[MapProcessor] 맵 통계:")
        # print(f"  - 원본 자유 공간: {free_orig} ({free_orig/self.occupancy_grid.size*100:.1f}%)")
        # print(f"  - 팽창 후 자유 공간: {free_dilated} ({free_dilated/self.dilated_map.size*100:.1f}%)")
        # print(f"  - 감소량: {free_orig - free_dilated} cells")
    
    def dilate_obstacles(self, radius=None):
        """장애물 팽창 (public 메서드)
        
        Args:
            radius: 팽창 반경 (픽셀 단위). None이면 기본값 사용
        """
        if self.occupancy_grid is None:
            print("[MapProcessor] 경고: occupancy_grid가 없습니다")
            return
            
        if radius is not None:
            # 지정된 반경 사용
            dilation_radius = int(radius)
            # print(f"[MapProcessor] 장애물 팽창: 반경 {dilation_radius} cells")
            
            # 커널 생성 (원형)
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, 
                (2*dilation_radius+1, 2*dilation_radius+1)
            )
            
            # 팽창 수행
            self.dilated_map = cv2.dilate(self.occupancy_grid, kernel, iterations=1)
            
            # 팽창 결과 확인
            original_obstacles = np.sum(self.occupancy_grid >= 127)
            dilated_obstacles = np.sum(self.dilated_map >= 127)
            # print(f"[MapProcessor] 팽창 결과:")
            # print(f"  - 원본 장애물: {original_obstacles} cells")
            # print(f"  - 팽창 후 장애물: {dilated_obstacles} cells")
            # print(f"  - 증가량: {dilated_obstacles - original_obstacles} cells")
        else:
            # 기본 설정 사용
            self._dilate_obstacles()
        
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
            
        # 임계값 기반 판단 (50 이하를 free로 간주 - 더 엄격하게)
        threshold = 50
        
        if use_dilated and self.dilated_map is not None:
            return self.dilated_map[grid_y, grid_x] < threshold
        elif self.occupancy_grid is not None:
            return self.occupancy_grid[grid_y, grid_x] < threshold
        else:
            return False
    
    def get_dilated_map(self):
        """팽창된 맵 반환"""
        return self.dilated_map if self.dilated_map is not None else self.occupancy_grid
    
    def visualize(self, path=None, start=None, goal=None):
        """맵과 경로 시각화
        
        Args:
            path: 경로 좌표 리스트 [(x,y), ...]
            start: 시작점 (x, y)
            goal: 목표점 (x, y)
        """
        import matplotlib.pyplot as plt
        
        if self.occupancy_grid is None:
            print("[MapProcessor] 시각화할 맵이 없습니다.")
            return
            
        # 그림 생성
        fig, axes = plt.subplots(1, 2, figsize=(12, 6))
        
        # 원본 맵
        ax = axes[0]
        ax.imshow(self.occupancy_grid, cmap='gray_r', origin='lower')
        ax.set_title("Original Map")
        ax.set_xlabel("X (cells)")
        ax.set_ylabel("Y (cells)")
        
        # 팽창된 맵
        ax = axes[1]
        if self.dilated_map is not None:
            ax.imshow(self.dilated_map, cmap='gray_r', origin='lower')
        else:
            ax.imshow(self.occupancy_grid, cmap='gray_r', origin='lower')
        ax.set_title("Dilated Map (for path planning)")
        ax.set_xlabel("X (cells)")
        ax.set_ylabel("Y (cells)")
        
        # 경로 표시
        if path:
            path_array = np.array(path)
            # 월드 좌표를 그리드 좌표로 변환
            grid_path = []
            for x, y in path:
                grid_x, grid_y = self.world_to_grid(x, y)
                grid_path.append([grid_x, grid_y])
            grid_path = np.array(grid_path)
            
            for a in axes:
                a.plot(grid_path[:, 0], grid_path[:, 1], 'b-', linewidth=2, label='Path')
                a.plot(grid_path[:, 0], grid_path[:, 1], 'bo', markersize=3)
        
        # 시작점과 목표점 표시
        if start:
            start_grid = self.world_to_grid(start[0], start[1])
            for a in axes:
                a.plot(start_grid[0], start_grid[1], 'go', markersize=10, label='Start')
                
        if goal:
            goal_grid = self.world_to_grid(goal[0], goal[1])
            for a in axes:
                a.plot(goal_grid[0], goal_grid[1], 'ro', markersize=10, label='Goal')
        
        for a in axes:
            a.legend()
            a.grid(True, alpha=0.3)
            
        plt.tight_layout()
        plt.show()
