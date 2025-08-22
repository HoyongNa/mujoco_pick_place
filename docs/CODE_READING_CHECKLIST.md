# 📋 코드 읽기 체크리스트

## ✅ 코드를 처음 읽을 때 확인사항

### 1️⃣ 실행 순서 파악
- [ ] `main.py`의 `main()` 함수부터 시작
- [ ] `wait_for_start()` → Space 키 대기
- [ ] `PickAndPlaceTask.execute()` → 작업 실행
- [ ] 4개 방을 순차적으로 처리

### 2️⃣ 핵심 데이터 구조 이해
- [ ] `self.model` - MuJoCo 모델 (환경 정의)
- [ ] `self.data` - 시뮬레이션 상태 (위치, 속도)
- [ ] `self.data.qpos` - 모든 관절 위치 [베이스 3개 + 팔 7개 + 그리퍼 1개]
- [ ] `self.data.ctrl` - 제어 입력 [같은 구조]

### 3️⃣ 주요 클래스 역할
- [ ] `SimulationManager` - 전체 관리자 (모든 컴포넌트 초기화)
- [ ] `PickAndPlaceTask` - 4개 작업 순차 실행
- [ ] `ArmController` - 팔 제어 (Ruckig 궤적)
- [ ] `MobilityController` - 베이스 이동 (키보드 입력)
- [ ] `InverseKinematicsSolver` - IK 계산 (위치→관절각)

---

## 🔍 코드 수정 시 주의사항

### ⚠️ 인덱스 관련
```python
# 항상 이 순서를 기억하세요!
qpos[0:3]   # 베이스 (x, y, theta)
qpos[3:10]  # 팔 7개 관절
qpos[10]    # 그리퍼

# ctrl도 동일한 구조
ctrl[0:3]   # 베이스 제어
ctrl[3:10]  # 팔 제어  
ctrl[10]    # 그리퍼 제어 (0=열림, 255=닫힘)
```

### ⚠️ 스레드 안전성
```python
# base_lock 사용 필수!
with self.base_lock:
    self.base_cmd_ref[:] = new_command
```

### ⚠️ 시뮬레이션 스텝
```python
# 물리 시뮬레이션 진행
mujoco.mj_step(self.model, self.data)
time.sleep(0.002)  # timestep과 동일
```

---

## 🗂️ 파일별 주요 함수

### 📄 main.py
| 함수 | 역할 | 호출 순서 |
|------|------|----------|
| `main()` | 진입점 | 1 |
| `wait_for_start()` | Space 키 대기 | 2 |

### 📄 simulation_manager.py
| 함수 | 역할 | 중요도 |
|------|------|--------|
| `__init__()` | 모든 초기화 | ⭐⭐⭐ |
| `start_mobility_control()` | 베이스 제어 시작 | ⭐⭐ |
| `stop_mobility_control()` | 베이스 제어 정지 | ⭐⭐ |
| `run_viewer()` | 뷰어 실행 루프 | ⭐⭐⭐ |

### 📄 pick_and_place.py
| 함수 | 역할 | 중요도 |
|------|------|--------|
| `execute()` | 전체 작업 실행 | ⭐⭐⭐ |
| `_execute_single_task()` | 단일 작업 | ⭐⭐⭐ |
| `_execute_waypoints()` | 웨이포인트 추적 | ⭐⭐ |
| `_grasp_object()` | 파지 실행 | ⭐ |
| `_release_object()` | 놓기 실행 | ⭐ |

### 📄 arm_controller.py
| 함수 | 역할 | 중요도 |
|------|------|--------|
| `track_with_ruckig()` | 부드러운 궤적 추적 | ⭐⭐⭐ |
| `hold_position()` | 위치 유지 | ⭐⭐ |
| `set_torque()` | 토크 설정 | ⭐ |

---

## 🎨 코드 패턴 인식

### 패턴 1: 박스 위치 조회
```python
# 이 패턴이 보이면 "박스 위치를 가져오는구나"
box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "box_name")
box_pos = self.data.xpos[box_id]
```

### 패턴 2: IK 계산
```python
# 이 패턴이 보이면 "목표 위치에서 관절각 계산"
q_target = self.ik_solver.solve(target_pos, target_rpy)
self.arm_controller.track_with_ruckig(q_target)
```

### 패턴 3: 그리퍼 제어
```python
# 이 패턴이 보이면 "그리퍼 열고 닫기"
self.shared_gripper_ctrl[0] = 255  # 닫기
self.settle(SETTLE_STEPS_GRASP)    # 안정화 대기
success = self.grasp_checker.wait_until_grasped()  # 파지 확인
```

### 패턴 4: 키보드 입력 처리
```python
# 이 패턴이 보이면 "키 입력 대기"
if keyboard.is_pressed('space'):
    if space_armed:  # 디바운싱
        # 실행
        space_armed = False
```

---

## 📝 변수명 규칙

### 일반 규칙
| 패턴 | 의미 | 예시 |
|------|------|------|
| `q_*` | 관절 각도 | `q_target`, `q_current` |
| `*_pos` | 위치 | `pick_pos`, `place_pos` |
| `*_id` | MuJoCo ID | `body_id`, `site_id` |
| `*_ref` | 참조값 | `base_cmd_ref` |
| `*_ctrl` | 제어값 | `gripper_ctrl` |

### 약어
| 약어 | 전체 | 의미 |
|------|------|------|
| `q` | joint angles | 관절 각도 |
| `ee` | end effector | 말단 장치 |
| `rpy` | roll pitch yaw | 회전 각도 |
| `cmd` | command | 명령 |
| `ref` | reference | 참조값 |
| `ctrl` | control | 제어 |
| `idx` | index | 인덱스 |

---

## 🔧 디버깅 포인트

### 중요 체크 위치
1. **작업 시작**: `_execute_single_task()` 시작 부분
2. **IK 계산**: `ik_solver.solve()` 전후
3. **파지 확인**: `wait_until_grasped()` 결과
4. **베이스 이동**: `update_command()` 에서 cmd 값

### 로그 추가 위치
```python
# 1. 작업 시작 시
print(f"\n작업 시작: {task['name']}")
print(f"Pick: {pick_pos}, Place: {place_pos}")

# 2. IK 실패 시
if not feasible:
    print(f"IK 실패: 거리={distance:.2f}m")
    
# 3. 파지 상태
print(f"접촉력: {force:.3f}N, 임계값: {threshold}N")

# 4. 베이스 위치
print(f"베이스: ({self.data.qpos[0]:.2f}, {self.data.qpos[1]:.2f})")
```

---

## 💡 빠른 테스트

### 1. 그리퍼만 테스트
```python
# Python 콘솔에서
sim_manager.shared_gripper_ctrl[0] = 255  # 닫기
time.sleep(1)
sim_manager.shared_gripper_ctrl[0] = 0    # 열기
```

### 2. 베이스만 이동
```python
# constants.py에서 속도 증가
BASE_LIN_STEP = 0.01  # 5배 빠르게
```

### 3. 특정 방만 테스트
```python
# pick_and_place.py에서
self.tasks = [self.tasks[0]]  # 방 1만
```

---

## ✨ 코드 읽기 순서 추천

### 초보자 (2시간)
1. `main.py` 전체
2. `constants.py` 상수 확인
3. `pick_and_place.py`의 `execute()` 메소드

### 중급자 (4시간)
1. 초보자 과정 +
2. `simulation_manager.py` 전체
3. `arm_controller.py`의 `track_with_ruckig()`
4. `mobility_controller.py`의 제어 루프

### 고급자 (전체)
1. 중급자 과정 +
2. `ik_solver.py` 최적화 알고리즘
3. `grasp_checker.py` 접촉 판정
4. 모든 스레드 동기화 부분

---

**팁**: 이 체크리스트를 프린트해서 모니터 옆에 두고 참고하세요! 📌
