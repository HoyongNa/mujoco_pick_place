# 🤖 MuJoCo Tidybot: Autonomous Navigation & Manipulation System

> **Stanford Tidybot 기반 자율 주행 및 Pick & Place 시뮬레이션 프레임워크**

[![MuJoCo](https://img.shields.io/badge/MuJoCo-2.3.0+-blue)](https://mujoco.org)
[![Python](https://img.shields.io/badge/Python-3.8+-green)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Stable-success)](https://github.com)

## 📖 개요

이 프로젝트는 Stanford Tidybot 로봇을 이용한 자율 주행 및 물체 조작 시뮬레이션 프레임워크입니다. 4개의 방으로 구성된 10m × 10m 환경에서 로봇이 LiDAR 기반 매핑, A* 경로 계획, 웨이포인트 기반 경로 추종을 통해 자율적으로 이동하며 Pick & Place 작업을 수행합니다.

### 🎯 주요 특징

- **🗺️ LiDAR 기반 실시간 환경 매핑**: 360도 레이저 스캔으로 정밀한 점유 격자 맵 생성
- **🧭 A* 경로 계획**: 8방향 탐색 기반 최적 경로 생성 및 장애물 회피
- **🚗 웨이포인트 경로 추종**: MuJoCo position actuator를 활용한 직접 위치 제어
- **🦾 7-DOF 로봇 팔 제어**: Ruckig 라이브러리 기반 Jerk-limited 궤적 생성
- **📦 Pick & Place 작업**: 4개 방에서 8개 박스 자동 조작 및 이송
- **⌨️ 하이브리드 제어**: 자율 네비게이션 + 텔레오퍼레이션 모드 전환

## 🚀 빠른 시작

### 1. 요구사항

```bash
# 필수 패키지
pip install -r requirements.txt
```

**필수 구성요소:**
- Python 3.8 이상
- MuJoCo 2.3.0 이상
- NumPy, OpenCV, Matplotlib
- pynput (크로스 플랫폼 키보드 입력)
- ruckig (궤적 생성)

### 2. 실행

```bash
# 기본 실행 (통합 시스템)
python main.py

# LiDAR 매핑 테스트
python test_lidar_interactive.py

# 저장된 맵 시각화
python view_saved_map.py
```

## 🎮 조작법

### 🎹 키보드 명령

| 키 | 기능 | 설명 |
|:---:|------|------|
| **Space** | 작업 시작/진행 | Pick & Place 작업 실행 |
| **1-4** | 방 이동 | 각 방으로 자동 네비게이션 |
| **8** | 전진 | 로봇 기준 앞으로 |
| **5** | 후진 | 로봇 기준 뒤로 |
| **4** | 좌측 이동 | 로봇 기준 왼쪽 |
| **6** | 우측 이동 | 로봇 기준 오른쪽 |
| **7** | 좌회전 | 반시계 방향 |
| **9** | 우회전 | 시계 방향 |
| **2** | 정지 | 원점 복귀 |
| **ESC** | 종료 | 프로그램 종료 |

## 🏗️ 시스템 구조

### 📂 프로젝트 구조

```
code2/
├── 📄 main.py                      # 메인 실행 파일 (통합 시스템)
├── 📄 test_lidar_interactive.py    # LiDAR 매핑 테스트
├── 📄 view_saved_map.py            # 저장된 맵 시각화 툴
├── 📄 requirements.txt             # 의존성 패키지
│
├── 🎯 simulation/                  # 시뮬레이션 코어
│   ├── simulation_manager.py       # 통합 시뮬레이션 관리
│   └── viewer_manager.py           # MuJoCo 뷰어 및 렌더링
│
├── 🤖 controllers/                 # 로봇 제어
│   ├── arm/                       # 7-DOF 팔 제어
│   │   ├── arm_controller.py      # Ruckig 기반 팔 제어
│   │   ├── torque_controller.py   # 토크 기반 정밀 제어
│   │   ├── trajectory_tracker.py  # 궤적 추적 및 보간
│   │   └── arm_holder.py          # 위치 유지 제어
│   ├── base/                      # 모바일 베이스 제어  
│   │   ├── mobility_controller.py # 베이스 이동 통합 제어
│   │   ├── base_teleop.py         # 텔레오퍼레이션 제어
│   │   └── keyboard_handler.py    # 키보드 입력 처리
│   └── gripper/                   # 그리퍼 제어
│       └── grasp_checker.py       # 접촉 및 파지 감지
│
├── 📐 kinematics/                  # 기구학 계산
│   └── ik_solver.py               # 역기구학 솔버 (SLSQP)
│
├── 🗺️ lidar_mapping/              # LiDAR 매핑 시스템
│   ├── lidar_sensor.py            # 360도 LiDAR 센서 시뮬레이션
│   ├── mapping_system.py          # 실시간 매핑 시스템 통합
│   ├── occupancy_grid.py          # Log-odds 기반 점유 격자 맵
│   └── visualizer.py              # 실시간 맵 시각화
│
├── 🧭 path_planning/               # 경로 계획
│   ├── map_processor.py           # 맵 전처리 (팽창/필터링/변환)
│   ├── astar_planner.py           # A* 최단 경로 계획
│   └── path_following_controller.py # 웨이포인트 기반 자율 네비게이션
│
├── 📦 tasks/                       # 작업 정의 및 실행
│   ├── pick_and_place.py          # Pick & Place 통합 작업
│   ├── waypoint_generator.py      # 7단계 웨이포인트 생성
│   └── feasibility_checker.py     # IK 기반 도달 가능성 검사
│
├── ⚙️ config/                      # 설정 파일
│   ├── constants.py               # 시스템 상수
│   └── robot_config.py            # 로봇 파라미터
│
├── 📚 docs/                        # 프로젝트 문서
│   ├── CODE_DOCUMENTATION.md      # 상세 코드 설명서
│   ├── CODE_READING_CHECKLIST.md  # 코드 읽기 체크리스트
│   └── QUICK_REFERENCE.md         # 빠른 참조 가이드
│
└── 🌍 model/                       # MuJoCo 모델
    └── stanford_tidybot/          # Tidybot 로봇 모델
```

## 🖥️ 플랫폼 호환성

이 프로젝트는 Windows, macOS, Linux에서 모두 동작합니다.

### macOS 사용자
- `pynput` 라이브러리를 사용하여 macOS 완벽 지원
- 시스템 환경설정에서 터미널/IDE에 접근성 권한 부여 필요할 수 있음

### Linux 사용자
- sudo 권한 또는 input 그룹 추가 필요할 수 있음
- 자세한 내용은 [PLATFORM_COMPATIBILITY.md](./PLATFORM_COMPATIBILITY.md) 참조

## 🔧 핵심 모듈 설명

### 1. 🗺️ LiDAR 매핑 시스템

**실시간 환경 매핑 및 점유 격자 맵 생성**

```python
# LiDAR 매핑 시스템 초기화
mapping_system = LidarMappingSystem(model, data,
    map_size=(200, 200),      # 200x200 격자
    resolution=0.05            # 5cm 해상도
)

# 실시간 매핑 시작 (10Hz 업데이트)
mapping_system.start_mapping(update_rate=10.0)

# 맵 저장 및 로드
mapping_system.save_map("environment_map")
loaded_map = mapping_system.load_map("environment_map.npz")
```

**핵심 알고리즘:**
- **360도 LiDAR 스캔**: 180개 레이, 최대 10m 범위
- **Log-odds 베이지안 업데이트**: 확률적 점유 격자 맵 생성
- **Hit/No-hit 처리**: 
  - Hit: 끝점을 occupied로 마킹, 경로를 free로 업데이트
  - No-hit: 경로만 free로 업데이트 (끝점 제외)
- **실시간 시각화**: matplotlib 기반 동적 맵 렌더링

### 2. 🧭 경로 계획 시스템

**A* 알고리즘 기반 최적 경로 생성**

```python
# 경로 계획기 초기화
map_processor = MapProcessor()
map_processor.load_map("lidar_map_20250826_215447.npz")

# 장애물 팽창 처리 (안전 마진)
map_processor.dilate_obstacles(radius=3)

# A* 경로 계획
planner = AStarPlanner(map_processor)
path = planner.plan(
    start=(current_x, current_y),
    goal=(target_x, target_y),
    use_dilated=True  # 팽창된 맵 사용
)
```

**핵심 특징:**
- **8방향 탐색**: 대각선 이동 포함 (cost = √2)
- **휴리스틱 함수**: 유클리드 거리 기반
- **경로 단순화**: Bresenham 라인 알고리즘으로 불필요한 웨이포인트 제거
- **장애물 팽창**: 안전 마진을 위한 morphological dilation

### 3. 🚗 경로 추종 제어

**웨이포인트 기반 경로 추종 (직접 위치 제어)**

```python
# 경로 추종 컨트롤러 초기화
controller = PathFollowingController(
    model, data, 
    base_cmd_ref, 
    base_lock
)

# 자율 네비게이션
success = controller.navigate_to((target_x, target_y))

# 실시간 상태 확인
if controller.is_navigation_complete():
    print("목표 도달!")
```

**제어 알고리즘:**
- **웨이포인트 추종**: A* 경로의 각 점을 순차적으로 추종
- **위치 직접 제어**: MuJoCo position actuator 활용
- **단계적 이동**: step_size(0.5m)를 통한 부드러운 이동
- **목표 도달 감지**: 거리 임계값 기반 (0.1m)

### 4. 🦾 로봇 팔 제어

**7-DOF 팔의 정밀 제어**

```python
# Ruckig 기반 부드러운 궤적 생성
arm_controller = ArmController(model, data, arm_joint_ids)

# Jerk-limited 궤적 추적
arm_controller.track_with_ruckig(
    target_joint_angles,
    max_velocity=3.0,      # rad/s
    max_acceleration=7.0,  # rad/s²
    max_jerk=150.0        # rad/s³
)

# 위치 유지 제어
arm_controller.hold_position(
    kp=400.0,  # P 게인
    kd=25.0    # D 게인
)
```

**제어 특징:**
- **Ruckig 라이브러리**: 3차 궤적 생성 (Jerk 제한)
- **토크 기반 제어**: 정밀한 힘 제어
- **SLSQP 역기구학**: 최적화 기반 IK 솔버
- **실시간 궤적 추적**: 2ms 제어 주기

### 5. 📦 Pick & Place 작업

**자동화된 물체 조작 작업**

```python
# Pick & Place 작업 실행
task = PickAndPlaceTask(sim_manager)
task.execute()  # 4개 방 순차 실행

# 단일 작업 실행
task.execute_single_task(
    pick_object="red_box",
    place_object="blue_box"
)
```

**작업 시퀀스 (7단계):**
1. **접근 시작** (Approach Start): 박스 위 15cm
2. **파지 위치** (Grasp Position): 박스 위 2cm  
3. **들어올리기** (Lift): 박스 잡고 20cm 상승
4. **이동** (Transit): 목표 위치로 이동
5. **놓기 위치** (Place Position): 목표 박스 위 15cm
6. **릴리즈** (Release): 그리퍼 열어 박스 놓기
7. **복귀** (Return): 홈 포지션으로 복귀

## 🌍 환경 구조

```
┌─────────┬─────────┐
│  Room 1 │  Room 2 │  각 방: 5m × 5m
│  Red→   │  Green→ │  문 폭: 1.4m
│  Blue   │  Yellow │  벽 두께: 10cm
├─────────┼─────────┤  
│  Room 3 │  Room 4 │  박스: 5cm × 5cm × 5cm
│  Orange→│  Cyan→  │  로봇 베이스: 직경 50cm
│  Purple │  Pink   │  팔 도달 범위: 0.8m
└─────────┴─────────┘
     10m × 10m
```

## ⚙️ 시스템 파라미터

### 베이스 제어
```python
# MuJoCo Position Actuator
BASE_KP = 1000000  # 위치 게인
BASE_KV = 50000    # 속도 게인
THETA_KP = 50000   # 각도 게인
THETA_KV = 1000    # 각속도 게인

# 경로 추종 파라미터
WAYPOINT_THRESHOLD = 0.1  # 웨이포인트 도달 거리 (m)
FINAL_THRESHOLD = 0.1      # 최종 목표 도달 거리 (m)
STEP_SIZE = 0.5           # 단계적 이동 거리 (m)
```

### 팔 제어
```python
# Ruckig 궤적 파라미터
MAX_VELOCITY = 3.0      # rad/s
MAX_ACCELERATION = 7.0  # rad/s²
MAX_JERK = 150.0       # rad/s³

# 위치 유지 제어
ARM_KP_HOLD = 400.0    # P 게인
ARM_KD_HOLD = 25.0     # D 게인
```

### 경로 계획
```python
# A* 파라미터
DILATION_RADIUS = 3    # 장애물 팽창 반경 (격자 단위)

# 웨이포인트 추종
CONTROL_FREQUENCY = 1000  # 제어 주기 (Hz)
MAX_ROTATION = 0.2       # 최대 회전 속도 (rad)
```

### LiDAR 매핑
```python
# 센서 파라미터
NUM_BEAMS = 180         # 레이저 빔 수
MAX_RANGE = 10.0        # 최대 감지 거리 (m)
SENSOR_HEIGHT = 0.2     # 센서 높이 (m)

# 맵 업데이트
HIT_PROB_INC = 0.7     # Hit 확률 증가
MISS_PROB_DEC = 0.4    # Miss 확률 감소
UPDATE_RATE = 10.0     # 업데이트 주기 (Hz)
```

## 🔬 고급 기능

### 맵 저장 및 로드

```python
# NPZ 형식으로 맵 저장
mapping_system.save_map("my_environment")

# 맵 로드 및 처리
map_processor = MapProcessor()
map_processor.load_map("lidar_map_20250826_215447.npz")
```

### 경로 시각화

```python
# 계획된 경로 시각화
map_processor.visualize(
    path=planned_path,
    start=(start_x, start_y),
    goal=(goal_x, goal_y)
)
```

### 실시간 디버깅

```python
# 시뮬레이션 상태 모니터링
print(f"베이스 위치: ({data.qpos[0]:.2f}, {data.qpos[1]:.2f}, {data.qpos[2]:.2f})")
print(f"팔 관절: {np.round(data.qpos[3:10], 2)}")
print(f"그리퍼: {data.ctrl[10]:.0f}")
print(f"접촉력: {grasp_checker.get_gripper_contact_force():.3f}N")
```

## 🐛 문제 해결

### 일반적인 문제와 해결방법

| 문제 | 원인 | 해결 방법 |
|------|------|-----------|
| Import 오류 | 모듈 경로 문제 | `__init__.py` 파일 확인, PYTHONPATH 설정 |
| 경로 계획 실패 | 목표 불가능 | 팽창 반경 조정 또는 목표 위치 변경 |
| 파지 실패 | 접촉력 부족 | `GRIPPER_FORCE_THRESHOLD` 조정 (기본 0.05N) |
| 느린 시뮬레이션 | timestep 설정 | `model.opt.timestep = 0.004` (기본 0.002) |
| 로봇 충돌 | 경로 너무 가까움 | `DILATION_RADIUS` 증가 (기본 3) |
| IK 실패 | 목표 도달 불가 | 베이스 위치 조정 필요 |

## 📊 성능 최적화

### 시뮬레이션 속도 개선
- **Timestep 조정**: `0.002` → `0.004` (정확도 vs 속도)
- **뷰어 업데이트 감소**: 매 2-3 프레임만 렌더링
- **불필요한 print 제거**: 디버그 모드만 활성화

### 경로 계획 최적화
- **맵 해상도 조정**: `0.05` → `0.1` (빠르지만 정밀도 감소)
- **경로 단순화 활성화**: 불필요한 웨이포인트 제거
- **A* 휴리스틱 가중치**: 1.0 → 1.5 (속도 우선)

### 메모리 사용 최적화
- **맵 크기 조정**: (200, 200) → (100, 100)
- **스레드 풀 사용**: 병렬 처리 활성화
- **캐싱 활용**: 반복 계산 결과 저장

## 🚧 알려진 제한사항

- **동적 장애물 미지원**: 정적 환경만 가능
- **단일 로봇만 지원**: 다중 로봇 협업 미구현
- **2D 평면 이동**: 3D 네비게이션 미지원
- **그리퍼 타입 고정**: 2-finger 그리퍼만 사용

## 📚 참고 자료

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Stanford Tidybot Project](https://tidybot.cs.princeton.edu/)
- [Ruckig Motion Library](https://github.com/pantor/ruckig)
- [A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Waypoint Navigation](https://en.wikipedia.org/wiki/Waypoint)
- [Log-odds Occupancy Mapping](https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2005/projects/1aslam_blas_repo.pdf)

## 🤝 기여

버그 리포트, 기능 제안, 풀 리퀘스트를 환영합니다!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 라이센스

MIT License - 자세한 내용은 [LICENSE](LICENSE) 파일 참조

---

**최종 업데이트**: 2025-08-28
**버전**: v3.0  
**작성**: MuJoCo Tidybot Team
