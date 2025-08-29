# Quick Reference Guide

## 프로젝트 빠른 시작

### 1. 환경 설정
```bash
# 가상환경 생성 및 활성화
python -m venv venv
venv\Scripts\activate  # Windows
source venv/bin/activate  # Linux/Mac

# 의존성 설치
pip install -r requirements.txt
```

### 2. 기본 실행
```bash
# 메인 시뮬레이션 실행
python main.py

# LiDAR 매핑 테스트
python test_lidar_interactive.py

# 저장된 맵 확인
python view_saved_map.py
```

## 주요 클래스 빠른 참조

### LidarMapper
```python
from lidar_mapping.lidar_mapper import LidarMapper

# 초기화
mapper = LidarMapper(
    map_size=(10.0, 10.0),      # 맵 크기 (m)
    resolution=0.05,             # 해상도 (m/cell)
    lidar_range=5.0,            # LiDAR 범위 (m)
    lidar_resolution=360         # 각도 해상도
)

# 맵 업데이트
mapper.update_map(robot_x, robot_y, robot_theta, scan_data)

# 맵 저장/로드
mapper.save_map("map.npz")
loaded_map = mapper.load_map("map.npz")
```

### AStarPlanner
```python
from path_planning.astar_planner import AStarPlanner
from path_planning.map_processor import MapProcessor

# 맵 프로세서 설정
processor = MapProcessor(occupancy_grid, origin, resolution)
processor.dilate_map(kernel_size=5)

# 경로 계획
planner = AStarPlanner(processor)
path = planner.plan(
    start=(0, 0), 
    goal=(4, 3),
    use_dilated=True
)
```

### PathFollowingController
```python
from path_planning.path_following_controller import PathFollowingController

# 초기화 (웨이포인트 기반 경로 추종)
path_controller = PathFollowingController(
    model, data,
    base_cmd_ref,
    base_lock
)

# 맵 로드 및 초기화
path_controller.initialize("map.npz")

# 목표점으로 네비게이션
success = path_controller.navigate_to((4.0, 3.0))

# 완료 확인
if path_controller.is_navigation_complete():
    print("목표 도달!")
```

### SimulationManager
```python
from simulation.simulation_manager import SimulationManager

# 시뮬레이션 초기화
sim = SimulationManager(
    model_path="model/robot.xml",
    render_mode="human"
)

# 시뮬레이션 스텝
sim.step()
robot_state = sim.get_robot_state()
sim.set_control(linear_vel, angular_vel)
```

## 자주 사용하는 코드 패턴

### 1. 맵 생성 및 경로 계획
```python
# LiDAR 데이터로 맵 생성
mapper = LidarMapper(map_size=(10, 10), resolution=0.05)
for _ in range(100):
    scan = get_lidar_scan()
    pose = get_robot_pose()
    mapper.update_map(pose[0], pose[1], pose[2], scan)

# 생성된 맵으로 경로 계획
occupancy_grid = mapper.get_occupancy_grid()
processor = MapProcessor(occupancy_grid, mapper.origin, mapper.resolution)
planner = AStarPlanner(processor)
path = planner.plan(start_pos, goal_pos)
```

### 2. 경로 추종 제어 루프
```python
# PathFollowingController 사용
path_controller = PathFollowingController(
    model, data, base_cmd_ref, base_lock
)

# 맵 초기화 및 네비게이션
path_controller.initialize("map.npz")
path_controller.start()  # 제어 스레드 시작

# 목표점으로 이동
path_controller.navigate_to((4.0, 3.0))

# 완료 대기
while not path_controller.is_navigation_complete():
    time.sleep(0.1)
```

### 3. 실시간 매핑과 네비게이션
```python
# 통합 시스템
class NavigationSystem:
    def __init__(self):
        self.mapper = LidarMapper()
        self.planner = None
        self.path_controller = None
        
    def update(self, lidar_data, robot_pose):
        # 맵 업데이트
        self.mapper.update_map(
            robot_pose[0], robot_pose[1], 
            robot_pose[2], lidar_data
        )
        
        # 필요시 재계획
        if self.should_replan():
            self.replan_path()
        
        # 웨이포인트 추종
        return self.get_control_command()
```

## 주요 파라미터 체크리스트

### LiDAR 매핑
- [ ] `map_size`: (10.0, 10.0) - 환경 크기에 맞게
- [ ] `resolution`: 0.05 - 정밀도 vs 성능
- [ ] `lidar_range`: 5.0 - 센서 사양
- [ ] `prob_hit`: 0.7 - 측정 신뢰도
- [ ] `prob_miss`: 0.3 - 빈 공간 신뢰도

### 경로 계획
- [ ] `kernel_size`: 5 - 장애물 팽창
- [ ] `obstacle_threshold`: 0.5 - 장애물 판정
- [ ] `use_dilated`: True - 안전 마진

### 경로 추종 (웨이포인트 기반)
- [ ] `waypoint_threshold`: 0.1 - 웨이포인트 도달 거리
- [ ] `final_threshold`: 0.1 - 최종 목표 도달 거리
- [ ] `step_size`: 0.5 - 한 번에 이동하는 거리
- [ ] `control_frequency`: 1000 - 제어 주기 (Hz)
- [ ] `max_rotation`: 0.2 - 최대 회전 속도

## 디버깅 팁

### 1. 맵이 제대로 생성되지 않을 때
```python
# 맵 시각화로 확인
import matplotlib.pyplot as plt
plt.imshow(mapper.get_occupancy_grid(), cmap='gray')
plt.colorbar()
plt.show()

# 로그 확률 확인
print(f"Max log-odds: {mapper.log_odds_map.max()}")
print(f"Min log-odds: {mapper.log_odds_map.min()}")
```

### 2. 경로가 이상할 때
```python
# 팽창된 맵 확인
plt.subplot(1, 2, 1)
plt.imshow(processor.occupancy_grid, cmap='gray')
plt.title("Original")

plt.subplot(1, 2, 2)
plt.imshow(processor.dilated_grid, cmap='gray')
plt.title("Dilated")
plt.show()

# 경로 시각화
for point in path:
    plt.plot(point[0], point[1], 'ro')
```

### 3. 로봇이 경로를 따라가지 못할 때
```python
# 현재 웨이포인트 확인
current_waypoint = path_controller.current_path[path_controller.waypoint_index]
print(f"Current waypoint: {current_waypoint}")
print(f"Robot position: {path_controller.get_current_position()}")

# 제어 출력 확인
with path_controller.base_lock:
    cmd = path_controller.base_cmd_ref.copy()
print(f"Control command: x={cmd[0]:.2f}, y={cmd[1]:.2f}, theta={cmd[2]:.2f}")
```

## 성능 최적화 체크리스트

### 계산 최적화
- [ ] NumPy 벡터화 연산 사용
- [ ] 불필요한 맵 업데이트 방지
- [ ] 경로 캐싱 활용
- [ ] 적절한 업데이트 주기 설정

### 메모리 최적화
- [ ] 맵 크기 최적화
- [ ] 히스토리 데이터 제한
- [ ] 불필요한 복사 방지

### 실시간 성능
- [ ] 제어 주기: 1000 Hz (PathFollowingController)
- [ ] 매핑 주기: 5-10 Hz
- [ ] 계획 주기: 1-2 Hz

## 자주 발생하는 에러와 해결

### ImportError
```python
# mujoco 관련
# 해결: pip install mujoco

# OpenCV 관련
# 해결: pip install opencv-python
```

### ValueError: Path not found
```python
# 원인: 시작/목표점이 장애물 내부
# 해결:
if processor.is_occupied(start):
    start = find_nearest_free_point(start)
```

### IndexError in grid conversion
```python
# 원인: 좌표가 맵 범위 초과
# 해결:
grid_x = np.clip(grid_x, 0, map_width-1)
grid_y = np.clip(grid_y, 0, map_height-1)
```

## 유용한 유틸리티 함수

### 좌표 변환
```python
def polar_to_cartesian(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def cartesian_to_polar(x, y):
    r = np.hypot(x, y)
    theta = np.arctan2(y, x)
    return r, theta
```

### 각도 정규화
```python
def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle
```

### 거리 계산
```python
def euclidean_distance(p1, p2):
    return np.hypot(p1[0]-p2[0], p1[1]-p2[1])

def manhattan_distance(p1, p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])
```

## 테스트 명령어

```bash
# 단위 테스트
python -m pytest tests/

# 특정 모듈 테스트
python -m pytest tests/test_path_planning.py

# 커버리지 확인
python -m pytest --cov=path_planning tests/
```

## Git 워크플로우

```bash
# 기능 브랜치 생성
git checkout -b feature/improved-path-planning

# 변경사항 커밋
git add .
git commit -m "feat: 웨이포인트 추종 알고리즘 개선"

# 메인 브랜치 병합
git checkout main
git merge feature/improved-path-planning
```

## 추가 리소스

- [프로젝트 문서](./README.md)
- [코드 문서](./CODE_DOCUMENTATION.md)
- [LiDAR 매핑 상세](./LIDAR_MAPPING.md)
- [경로 계획 상세](./PATH_PLANNING.md)
