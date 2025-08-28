"""
Pure Pursuit 경로 추종 알고리즘
경로를 따라 로봇을 제어하는 알고리즘
"""

import numpy as np
from typing import List, Tuple, Optional


class PurePursuit:
    """Pure Pursuit 경로 추종 컨트롤러"""
    
    def __init__(self, lookahead_distance=0.5, max_linear_vel=0.5, max_angular_vel=1.0):
        """
        Args:
            lookahead_distance: 전방 주시 거리 (meters)
            max_linear_vel: 최대 선속도 (m/s)
            max_angular_vel: 최대 각속도 (rad/s)
        """
        self.lookahead_distance = lookahead_distance
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        
        # 경로 관련
        self.path = None
        self.current_waypoint_idx = 0
        
        # 제어 파라미터
        self.goal_tolerance = 0.1  # 목표점 도달 허용 오차 (meters)
        self.min_lookahead = 0.3   # 최소 전방 주시 거리
        self.max_lookahead = 0.5   # 최대 전방 주시 거리
        
    def set_path(self, path: List[Tuple[float, float]]):
        """경로 설정
        
        Args:
            path: 경로 좌표 리스트 [(x1,y1), (x2,y2), ...]
        """
        if not path:
            print("[PurePursuit] 빈 경로가 입력되었습니다.")
            return
            
        self.path = path
        self.current_waypoint_idx = 0
        print(f"[PurePursuit] 경로 설정 완료: {len(path)}개 웨이포인트")
        
    def compute_control(self, robot_pose: Tuple[float, float, float]) -> Tuple[float, float, bool]:
        """제어 명령 계산
        
        Args:
            robot_pose: 현재 로봇 위치와 방향 (x, y, theta)
            
        Returns:
            (linear_vel, angular_vel, is_finished): 선속도, 각속도, 완료 여부
        """
        if self.path is None or len(self.path) == 0:
            return 0.0, 0.0, True
            
        x, y, theta = robot_pose
        
        # 목표점 도달 체크
        goal = self.path[-1]
        dist_to_goal = np.sqrt((x - goal[0])**2 + (y - goal[1])**2)
        
        if dist_to_goal < self.goal_tolerance:
            print(f"[PurePursuit] 목표점 도달! (오차: {dist_to_goal:.3f}m)")
            return 0.0, 0.0, True
        
        # Lookahead 포인트 찾기
        lookahead_point = self._find_lookahead_point(robot_pose)
        
        if lookahead_point is None:
            # 마지막 웨이포인트를 lookahead point로 사용
            lookahead_point = self.path[-1]
        
        # Pure Pursuit 제어 계산
        linear_vel, angular_vel = self._calculate_velocities(robot_pose, lookahead_point)
        
        # 속도 제한
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        return linear_vel, angular_vel, False
    
    def _find_lookahead_point(self, robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """Lookahead 포인트 찾기
        
        Args:
            robot_pose: 현재 로봇 위치와 방향 (x, y, theta)
            
        Returns:
            Lookahead 포인트 (x, y) or None
        """
        x, y, theta = robot_pose
        
        # 가장 가까운 웨이포인트 찾기
        min_dist = float('inf')
        closest_idx = self.current_waypoint_idx
        
        for i in range(self.current_waypoint_idx, len(self.path)):
            wx, wy = self.path[i]
            dist = np.sqrt((x - wx)**2 + (y - wy)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.current_waypoint_idx = closest_idx
        
        # 적응적 lookahead 거리 조정
        # 속도에 따라 lookahead 거리 조정 가능
        adaptive_lookahead = self.lookahead_distance
        
        # Lookahead 거리를 만족하는 첫 번째 웨이포인트 찾기
        for i in range(self.current_waypoint_idx, len(self.path)):
            wx, wy = self.path[i]
            dist = np.sqrt((x - wx)**2 + (y - wy)**2)
            
            if dist >= adaptive_lookahead:
                # 선형 보간으로 정확한 lookahead point 계산
                if i > 0:
                    prev_wx, prev_wy = self.path[i-1]
                    prev_dist = np.sqrt((x - prev_wx)**2 + (y - prev_wy)**2)
                    
                    if prev_dist < adaptive_lookahead <= dist:
                        # 보간
                        t = (adaptive_lookahead - prev_dist) / (dist - prev_dist)
                        lookahead_x = prev_wx + t * (wx - prev_wx)
                        lookahead_y = prev_wy + t * (wy - prev_wy)
                        return (lookahead_x, lookahead_y)
                
                return (wx, wy)
        
        # 경로의 끝에 도달한 경우
        return None
    
    def _calculate_velocities(self, robot_pose: Tuple[float, float, float], 
                            lookahead_point: Tuple[float, float]) -> Tuple[float, float]:
        """Pure Pursuit 속도 계산
        
        Args:
            robot_pose: 현재 로봇 위치와 방향 (x, y, theta)
            lookahead_point: 전방 주시점 (x, y)
            
        Returns:
            (linear_vel, angular_vel): 선속도, 각속도
        """
        x, y, theta = robot_pose
        lx, ly = lookahead_point
        
        # 로봇 좌표계에서 lookahead point의 위치
        dx = lx - x
        dy = ly - y
        
        # 로봇 좌표계로 변환
        local_x = dx * np.cos(theta) + dy * np.sin(theta)
        local_y = -dx * np.sin(theta) + dy * np.cos(theta)
        
        # Lookahead 거리
        L = np.sqrt(local_x**2 + local_y**2)
        
        if L < 0.01:  # 거의 도달한 경우
            return 0.0, 0.0
        
        # 곡률 계산 (2 * y / L^2)
        curvature = 2.0 * local_y / (L**2)
        
        # 속도 계산
        linear_vel = self.max_linear_vel
        
        # 곡률이 클수록 속도 감소
        if abs(curvature) > 0.1:
            linear_vel *= (0.1 / abs(curvature))
            linear_vel = max(linear_vel, 0.1)  # 최소 속도 유지
        
        # 각속도 = 선속도 * 곡률
        angular_vel = linear_vel * curvature
        
        # 후진이 필요한 경우 (목표가 뒤에 있음)
        if local_x < 0:
            linear_vel = -linear_vel
            
        return linear_vel, angular_vel
    
    def get_progress(self) -> float:
        """경로 진행률 반환
        
        Returns:
            진행률 (0.0 ~ 1.0)
        """
        if self.path is None or len(self.path) == 0:
            return 1.0
            
        return self.current_waypoint_idx / max(len(self.path) - 1, 1)
    
    def visualize_lookahead(self, robot_pose: Tuple[float, float, float]):
        """Lookahead 포인트 시각화 (디버깅용)
        
        Args:
            robot_pose: 현재 로봇 위치와 방향 (x, y, theta)
        """
        import matplotlib.pyplot as plt
        
        if self.path is None:
            return
            
        x, y, theta = robot_pose
        
        # 경로 그리기
        path_x = [p[0] for p in self.path]
        path_y = [p[1] for p in self.path]
        plt.plot(path_x, path_y, 'b-', label='Path')
        
        # 로봇 위치
        plt.plot(x, y, 'ro', markersize=10, label='Robot')
        
        # 로봇 방향
        arrow_len = 0.3
        plt.arrow(x, y, arrow_len * np.cos(theta), arrow_len * np.sin(theta),
                 head_width=0.1, head_length=0.1, fc='r', ec='r')
        
        # Lookahead 포인트
        lookahead_point = self._find_lookahead_point(robot_pose)
        if lookahead_point:
            plt.plot(lookahead_point[0], lookahead_point[1], 'go', 
                    markersize=8, label='Lookahead')
            
            # Lookahead 원
            circle = plt.Circle((x, y), self.lookahead_distance, 
                               color='g', fill=False, linestyle='--')
            plt.gca().add_patch(circle)
        
        # 현재 웨이포인트
        if self.current_waypoint_idx < len(self.path):
            wx, wy = self.path[self.current_waypoint_idx]
            plt.plot(wx, wy, 'yo', markersize=6, label='Current Waypoint')
        
        plt.axis('equal')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.title('Pure Pursuit Visualization')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.show()