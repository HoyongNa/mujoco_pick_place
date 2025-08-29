# Path Planning Documentation

## 목차
1. [개요](#개요)
2. [시스템 아키텍처](#시스템-아키텍처)
3. [핵심 컴포넌트](#핵심-컴포넌트)
4. [알고리즘 상세](#알고리즘-상세)
5. [사용 예시](#사용-예시)
6. [파라미터 튜닝](#파라미터-튜닝)

## 개요

이 프로젝트의 Path Planning 시스템은 LiDAR 기반 occupancy grid map에서 안전하고 효율적인 경로를 생성하는 역할을 합니다. A* 알고리즘을 사용하여 최적 경로를 탐색하고, 웨이포인트 기반 경로 추종을 통해 목표점까지 이동합니다.

### 주요 특징
- **A* 기반 경로 탐색**: 휴리스틱을 활용한 최적 경로 생성
- **맵 팽창(Dilation)**: 장애물 주변 안전 마진 확보
- **웨이포인트 추종**: MuJoCo position actuator를 활용한 직접 위치 제어
- **실시간 경로 재계획**: 동적 환경 대응

## 시스템 아키텍처

```
┌──────────────────┐     ┌──────────────────┐
│   LiDAR Mapping  │────▶│   Map Processor  │
└──────────────────┘     └──────────────────┘
                                │
                                ▼
                        ┌──────────────────┐
                        │   A* Planner     │
                        └──────────────────┘
                                │
                                ▼
                    ┌────────────────────────────┐
                    │  PathFollowingController   │
                    │  (Waypoint Following)      │
                    └────────────────────────────┘
                                │
                                ▼
                        ┌──────────────────┐
                        │  Robot Control   │
                        └──────────────────┘
```

## 핵심 컴포넌트

### 1. MapProcessor (`map_processor.py`)

Occupancy grid map을 처리하고 경로 계획에 필요한 전처리를 수행합니다.

#### 주요 메서드

##### `__init__(self, occupancy_grid, origin, resolution, obstacle_threshold=0.5)`
맵 프로세서 초기화
- **occupancy_grid**: 2D numpy array (0: free, 1: occupied)
- **origin**: 맵의 원점 좌표 (x, y)
- **resolution**: 그리드 셀 크기 (meters/cell)
- **obstacle_threshold**: 장애물 판정 임계값

##### `dilate_map(self, kernel_size=5)`
장애물 주변을 팽창시켜 안전 마진을 생성합니다.
```python
# 모폴로지 팽창 연산으로 장애물 확장
kernel = np.ones((kernel_size, kernel_size))
dilated = cv2.dilate(binary_map, kernel, iterations=1)
```

##### `world_to_grid(self, x, y)`
월드 좌표를 그리드 인덱스로 변환
```python
grid_x = int((x - self.origin[0]) / self.resolution)
grid_y = int((y - self.origin[1]) / self.resolution)
return (grid_x, grid_y)
```

##### `grid_to_world(self, grid_x, grid_y)`
그리드 인덱스를 월드 좌표로 변환
```python
x = grid_x * self.resolution + self.origin[0]
y = grid_y * self.resolution + self.origin[1]
return (x, y)
```

### 2. AStarPlanner (`astar_planner.py`)

A* 알고리즘을 구현하여 최단 경로를 탐색합니다.

#### 핵심 알고리즘

##### A* 경로 탐색
```python
def plan(self, start, goal, use_dilated=True):
    """
    A* 알고리즘 구현
    f(n) = g(n) + h(n)
    - g(n): 시작점에서 현재 노드까지의 실제 비용
    - h(n): 현재 노드에서 목표점까지의 휴리스틱 비용 (유클리드 거리)
    """
    
    # 우선순위 큐 초기화 (f_score 기준)
    open_set = [(0, start_grid)]
    
    # 각 노드까지의 최소 비용
    g_score = {start_grid: 0}
    
    # 경로 추적을 위한 부모 노드
    came_from = {}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal_grid:
            # 경로 재구성
            return self.reconstruct_path(came_from, current)
        
        # 8방향 이웃 탐색
        for dx, dy, move_cost in self.moves:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # 장애물 체크
            if self.is_occupied(neighbor):
                continue
                
            # 비용 계산
            tentative_g = g_score[current] + move_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                # 경로 업데이트
                g_score[neighbor] = tentative_g
                f_score = tentative_g + self.heuristic(neighbor, goal_grid)
                heapq.heappush(open_set, (f_score, neighbor))
                came_from[neighbor] = current
```

##### 휴리스틱 함수
```python
def heuristic(self, node, goal):
    """유클리드 거리 기반 휴리스틱"""
    return np.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)
```

#### 주요 특징

1. **8방향 이동**: 대각선 이동을 포함하여 더 자연스러운 경로 생성
   - 직선 이동: 비용 1.0
   - 대각선 이동: 비용 √2 (1.414)

2. **경로 최적화**: 불필요한 중간점을 제거하여 경로 단순화

3. **안전성 보장**: 팽창된 맵 사용으로 장애물과의 충돌 방지

### 3. PathFollowingController (`path_following_controller.py`)

경로 계획과 웨이포인트 기반 경로 추종을 통합 관리하는 컨트롤러입니다.

#### 핵심 기능

##### 웨이포인트 추종 알고리즘
```python
def _control_loop(self):
    """웨이포인트 기반 경로 추종"""
    
    # 현재 로봇 포즈
    x = self.data.qpos[0]
    y = self.data.qpos[1]
    theta = self.data.qpos[2]
    
    # 현재 목표 웨이포인트
    target_waypoint = self.current_path[self.waypoint_index]
    
    # 목표까지의 거리와 방향
    dx = target_waypoint[0] - x
    dy = target_waypoint[1] - y
    distance = np.sqrt(dx**2 + dy**2)
    
    # 웨이포인트 도달 확인
    if distance < self.waypoint_threshold:
        self.waypoint_index += 1
    
    # 단계적 이동 (부드럽게)
    move_step = min(self.step_size, distance)
    
    # 다음 위치 계산
    next_x = x + (dx / distance) * move_step
    next_y = y + (dy / distance) * move_step
    
    # 목표 방향 계산
    target_theta = np.arctan2(dy, dx)
    
    # MuJoCo position actuator에 직접 명령
    self.data.ctrl[0] = next_x  # x 위치
    self.data.ctrl[1] = next_y  # y 위치
    self.data.ctrl[2] = target_theta  # theta 각도
```

#### 주요 파라미터

- **waypoint_threshold**: 웨이포인트 도달 거리 (0.1m)
- **final_threshold**: 최종 목표 도달 거리 (0.1m)
- **step_size**: 한 번에 이동하는 거리 (0.5m)
- **control_frequency**: 제어 주기 (1000Hz)

##### 경로 계획 및 실행
```python
def navigate_to(self, target, visualize=False):
    """목표점까지의 경로 계획 및 실행"""
    
    # 1. 경로 계획
    path = self.planner.plan(start, target, use_dilated=True)
    
    if path is None:
        print("경로를 찾을 수 없습니다")
        return False
    
    # 2. 경로 설정
    self.current_path = path
    self.target_position = target
    self.waypoint_index = 0
    
    # 3. 네비게이션 시작
    self.is_navigating = True
    self.navigation_complete = False
    
    return True
```

## 알고리즘 상세

### A* 알고리즘의 장점

1. **최적성 보장**: 일관된 휴리스틱 사용시 최단 경로 보장
2. **효율성**: 휴리스틱으로 불필요한 탐색 감소
3. **완전성**: 경로가 존재하면 반드시 찾음

### 웨이포인트 추종의 특징

1. **직접 위치 제어**: MuJoCo position actuator 활용
2. **단계적 이동**: step_size를 통한 부드러운 이동
3. **회전 제어**: 목표 방향으로 점진적 회전
4. **구현 간단**: 복잡한 제어 이론 불필요

## 사용 예시

### 기본 사용법

```python
# 1. 맵 프로세서 생성
map_processor = MapProcessor(
    occupancy_grid=lidar_map,
    origin=(-5.0, -5.0),
    resolution=0.05
)

# 2. 맵 팽창 (안전 마진)
map_processor.dilate_map(kernel_size=7)

# 3. A* 플래너 생성
planner = AStarPlanner(map_processor)

# 4. 경로 계획
start_pos = (0.0, 0.0)
goal_pos = (4.0, 3.0)
path = planner.plan(start_pos, goal_pos, use_dilated=True)

# 5. PathFollowingController 생성 및 사용
path_controller = PathFollowingController(
    model, data, base_cmd_ref, base_lock
)
path_controller.initialize("map.npz")

# 6. 자율 네비게이션
path_controller.navigate_to((4.0, 3.0))

# 7. 완료 대기
while not path_controller.is_navigation_complete():
    time.sleep(0.1)
```

### 고급 사용법

```python
# 통합 시스템에서 사용
class NavigationSystem:
    def __init__(self, model, data):
        self.controller = PathFollowingController(
            model, data, base_cmd_ref, base_lock
        )
        
    def execute_mission(self, waypoints):
        for waypoint in waypoints:
            # 각 웨이포인트로 이동
            success = self.controller.navigate_to(waypoint)
            
            if not success:
                print(f"Failed to reach {waypoint}")
                continue
                
            # 도달 대기
            while not self.controller.is_navigation_complete():
                time.sleep(0.1)
```

## 파라미터 튜닝

### MapProcessor 파라미터

| 파라미터 | 기본값 | 설명 | 조정 가이드 |
|---------|-------|------|------------|
| kernel_size | 5 | 팽창 커널 크기 | 크게: 안전하지만 좁은 통로 통과 어려움 |
| obstacle_threshold | 0.5 | 장애물 판정 임계값 | 낮게: 보수적, 높게: 공격적 |

### A* Planner 파라미터

| 파라미터 | 설명 | 영향 |
|---------|------|------|
| use_dilated | 팽창 맵 사용 여부 | True: 안전, False: 최단 거리 |
| move_cost | 이동 비용 | 대각선 vs 직선 선호도 조절 |

### PathFollowingController 파라미터

| 파라미터 | 기본값 | 설명 | 조정 효과 |
|---------|-------|------|------------|
| waypoint_threshold | 0.1m | 웨이포인트 도달 거리 | 작게: 정밀, 크게: 빠른 진행 |
| final_threshold | 0.1m | 최종 목표 도달 거리 | 목표 도달 정밀도 |
| step_size | 0.5m | 이동 스텝 크기 | 작게: 부드러움, 크게: 빠름 |
| control_frequency | 1000Hz | 제어 주기 | 시스템 반응성 |
| max_rotation | 0.2 rad | 최대 회전 속도 | 회전 속도 제한 |

## 성능 최적화

### 1. 경로 단순화
불필요한 중간점을 제거하여 더 효율적인 경로 생성:
```python
def simplify_path(path, tolerance=0.1):
    """Bresenham 알고리즘으로 직선 연결 가능한 점 제거"""
    # 구현 코드
```

### 2. 적응적 스텝 크기
상황에 따라 step_size 조절:
```python
# 좁은 통로에서는 작게
if in_narrow_passage:
    step_size = 0.1
# 열린 공간에서는 크게
else:
    step_size = 0.5
```

### 3. 경로 캐싱
자주 사용되는 경로를 캐싱하여 재계산 시간 단축

## 트러블슈팅

### 문제: 로봇이 목표점에 도달하지 못함
- **원인**: waypoint_threshold가 너무 작음
- **해결**: waypoint_threshold 증가 (0.1m → 0.2m)

### 문제: 로봇이 회전을 못함
- **원인**: max_rotation이 너무 작음
- **해결**: max_rotation 증가 또는 회전 제어 게인 조정

### 문제: 좁은 통로를 통과하지 못함
- **원인**: 맵 팽창이 과도함
- **해결**: kernel_size 감소 또는 adaptive dilation 적용

### 문제: 경로가 비효율적
- **원인**: 웨이포인트가 너무 많음
- **해결**: 경로 단순화 알고리즘 적용

## 향후 개선 사항

1. **Pure Pursuit 통합**: 더 부드러운 경로 추종
2. **RRT* 통합**: 복잡한 환경에서 더 나은 경로 생성
3. **Dynamic Window Approach**: 동적 장애물 회피
4. **경로 스무딩**: B-spline 또는 Bezier 곡선 적용
5. **학습 기반 최적화**: 강화학습으로 파라미터 자동 튜닝
