"""
A* 경로 계획 알고리즘
occupancy grid 상에서 최단 경로 탐색
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional


class AStarPlanner:
    """A* 경로 계획기"""
    
    def __init__(self, map_processor):
        """
        Args:
            map_processor: MapProcessor 인스턴스
        """
        self.map_processor = map_processor
        
        # 8방향 이동 (dx, dy, cost)
        # 대각선 이동은 √2 비용
        self.moves = [
            (-1, 0, 1.0),   # up
            (1, 0, 1.0),    # down
            (0, -1, 1.0),   # left
            (0, 1, 1.0),    # right
            (-1, -1, 1.414), # up-left
            (-1, 1, 1.414),  # up-right
            (1, -1, 1.414),  # down-left
            (1, 1, 1.414)    # down-right
        ]
        
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float], 
             use_dilated=True) -> Optional[List[Tuple[float, float]]]:
        """A* 경로 계획
        
        Args:
            start: 시작 위치 (x, y) in meters
            goal: 목표 위치 (x, y) in meters
            use_dilated: 팽창된 맵 사용 여부
            
        Returns:
            경로 좌표 리스트 [(x1,y1), (x2,y2), ...] or None if no path
        """
        # 그리드 좌표로 변환
        start_grid = self.map_processor.world_to_grid(start[0], start[1])
        goal_grid = self.map_processor.world_to_grid(goal[0], goal[1])
        
        # 유효성 체크
        if not self._is_valid_point(start_grid, use_dilated):
            print(f"[AStarPlanner] 시작점이 유효하지 않습니다: {start} → {start_grid}")
            # 팽창된 맵에서는 막혀있지만 원본 맵에서는 가능한지 확인
            if self._is_valid_point(start_grid, use_dilated=False):
                print("  -> 원본 맵에서는 유효함. 팽창 반경이 너무 큽니다.")
            return None
            
        if not self._is_valid_point(goal_grid, use_dilated):
            print(f"[AStarPlanner] 목표점이 유효하지 않습니다: {goal} → {goal_grid}")
            if self._is_valid_point(goal_grid, use_dilated=False):
                print("  -> 원본 맵에서는 유효함. 팽창 반경이 너무 큽니다.")
            return None
        
        print(f"[AStarPlanner] 경로 계획 시작: {start} → {goal}")
        
        # A* 알고리즘
        grid_path = self._astar(start_grid, goal_grid, use_dilated)
        
        if grid_path is None:
            print("[AStarPlanner] 경로를 찾을 수 없습니다.")
            return None
        
        # 월드 좌표로 변환
        world_path = []
        for row, col in grid_path:
            x, y = self.map_processor.grid_to_world(row, col)
            world_path.append((x, y))
        
        # 경로 단순화 (선택적)
        world_path = self._simplify_path(world_path, use_dilated)
        
        print(f"[AStarPlanner] 경로 생성 완료: {len(world_path)}개 웨이포인트")
        return world_path
    
    def _astar(self, start_grid: Tuple[int, int], goal_grid: Tuple[int, int], 
               use_dilated=True) -> Optional[List[Tuple[int, int]]]:
        """A* 알고리즘 구현
        
        Args:
            start_grid: 시작 그리드 좌표 (row, col)
            goal_grid: 목표 그리드 좌표 (row, col)
            use_dilated: 팽창된 맵 사용 여부
            
        Returns:
            그리드 경로 [(row1,col1), (row2,col2), ...] or None
        """
        # 초기화
        open_set = []  # (f_score, counter, node)
        counter = 0  # tie-breaker
        
        # 시작 노드
        start_node = start_grid
        g_score = {start_node: 0}
        f_score = self._heuristic(start_node, goal_grid)
        
        heapq.heappush(open_set, (f_score, counter, start_node))
        counter += 1
        
        # 부모 노드 추적
        came_from = {}
        
        # 방문한 노드
        closed_set = set()
        
        while open_set:
            current_f, _, current = heapq.heappop(open_set)
            
            # 목표 도달
            if current == goal_grid:
                return self._reconstruct_path(came_from, current)
            
            # 이미 방문한 노드 스킵
            if current in closed_set:
                continue
                
            closed_set.add(current)
            
            # 이웃 노드 탐색
            for dr, dc, move_cost in self.moves:
                neighbor = (current[0] + dr, current[1] + dc)
                
                # 유효성 체크
                if not self._is_valid_point(neighbor, use_dilated):
                    continue
                    
                # 이미 방문한 노드 스킵
                if neighbor in closed_set:
                    continue
                
                # 벽과의 거리를 고려한 추가 비용
                wall_penalty = self._wall_proximity_cost(neighbor, use_dilated)
                
                # g 스코어 계산 (벽 근접 패널티 포함)
                tentative_g = g_score[current] + move_cost + wall_penalty
                
                # 더 나은 경로 발견
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h_score = self._heuristic(neighbor, goal_grid)
                    f = tentative_g + h_score
                    
                    heapq.heappush(open_set, (f, counter, neighbor))
                    counter += 1
        
        return None  # 경로 없음
    
    def _wall_proximity_cost(self, node: Tuple[int, int], use_dilated=True) -> float:
        """벽과의 근접도에 따른 비용 계산
        
        Args:
            node: 그리드 좌표 (row, col)
            use_dilated: 팽창된 맵 사용 여부
            
        Returns:
            벽 근접 패널티 비용
        """
        # 주변 5x5 영역에서 벽까지의 최소 거리 계산
        min_distance = float('inf')
        check_radius = 5  # 검사 반경 (셀 단위)
        
        for dr in range(-check_radius, check_radius + 1):
            for dc in range(-check_radius, check_radius + 1):
                check_node = (node[0] + dr, node[1] + dc)
                
                # 맵 범위 내인지 확인
                if not self.map_processor.is_valid(check_node[1], check_node[0]):
                    continue
                    
                # 장애물인지 확인
                if not self.map_processor.is_free(check_node[1], check_node[0], use_dilated=False):
                    distance = np.sqrt(dr**2 + dc**2)
                    min_distance = min(min_distance, distance)
        
        # 거리에 따른 패널티 계산
        if min_distance < 2:  # 2셀 이내에 벽이 있으면 (줄임)
            # 가까울수록 높은 패널티 (계수도 줄임)
            return (2 - min_distance) * 1.0
        else:
            return 0.0
    
    def _heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """휴리스틱 함수 (유클리드 거리)
        
        Args:
            node: 현재 노드 (row, col)
            goal: 목표 노드 (row, col)
            
        Returns:
            휴리스틱 값
        """
        return np.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)
    
    def _is_valid_point(self, point: Tuple[int, int], use_dilated=True) -> bool:
        """그리드 좌표가 유효하고 자유 공간인지 확인
        
        Args:
            point: 그리드 좌표 (row, col)
            use_dilated: 팽창된 맵 사용 여부
            
        Returns:
            유효 여부
        """
        # is_free 메서드가 is_valid 체크도 내부적으로 수행함
        return self.map_processor.is_free(point[0], point[1], use_dilated)
    
    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """경로 재구성
        
        Args:
            came_from: 부모 노드 딕셔너리
            current: 현재 노드
            
        Returns:
            경로 리스트
        """
        path = [current]
        
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path
    
    def _simplify_path(self, path: List[Tuple[float, float]], 
                       use_dilated=True) -> List[Tuple[float, float]]:
        """경로 단순화 (불필요한 중간점 제거)
        
        Args:
            path: 원본 경로
            use_dilated: 팽창된 맵 사용 여부
            
        Returns:
            단순화된 경로
        """
        if len(path) <= 2:
            return path
            
        simplified = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            
            # 가장 먼 직선 연결 가능한 점 찾기
            while j > i + 1:
                if self._is_line_clear(path[i], path[j], use_dilated):
                    break
                j -= 1
            
            simplified.append(path[j])
            i = j
        
        return simplified
    
    def _is_line_clear(self, p1: Tuple[float, float], p2: Tuple[float, float], 
                       use_dilated=True) -> bool:
        """두 점 사이의 직선 경로가 장애물 없이 clear한지 확인
        
        Args:
            p1: 시작점 (x, y) in meters
            p2: 끝점 (x, y) in meters
            use_dilated: 팽창된 맵 사용 여부
            
        Returns:
            직선 경로 가능 여부
        """
        # 그리드 좌표로 변환
        grid1 = self.map_processor.world_to_grid(p1[0], p1[1])
        grid2 = self.map_processor.world_to_grid(p2[0], p2[1])
        
        # Bresenham's line algorithm
        points = self._bresenham_line(grid1[0], grid1[1], grid2[0], grid2[1])
        
        # 모든 점이 자유 공간인지 확인
        for point in points:
            if not self.map_processor.is_free(point[0], point[1], use_dilated):
                return False
                
        return True
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm
        
        Args:
            x0, y0: 시작점
            x1, y1: 끝점
            
        Returns:
            라인 상의 점들
        """
        points = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            
            if e2 > -dy:
                err -= dy
                x0 += sx
                
            if e2 < dx:
                err += dx
                y0 += sy
        
        return points
