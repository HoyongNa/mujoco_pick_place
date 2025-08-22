# 🚀 코드 빠른 참조 가이드 (Quick Reference)

## 📌 자주 사용하는 코드 패턴

### 1. 박스 위치 가져오기
```python
# 박스 이름으로 위치 조회
box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "red_box")
box_pos = self.data.xpos[box_id]  # [x, y, z] 좌표
```

### 2. 로봇 베이스 위치 확인
```python
# 현재 베이스 위치
base_x = self.data.qpos[0]
base_y = self.data.qpos[1]  
base_theta = self.data.qpos[2]  # 회전 각도
```

### 3. 팔 관절 제어
```python
# 목표 관절 각도 설정
target_q = [0, 0, 0, -1.57, 0, 1.57, 0]  # 7개 관절
self.arm_controller.track_with_ruckig(target_q)
```

### 4. 그리퍼 제어
```python
# 그리퍼 열기/닫기
self.shared_gripper_ctrl[0] = 0    # 열기
self.shared_gripper_ctrl[0] = 255  # 닫기
```

### 5. IK 계산
```python
# 목표 위치에서 관절 각도 계산
target_pos = [0.5, 0.0, 0.3]  # [x, y, z]
target_rpy = [3.14, 0, 0]     # [roll, pitch, yaw]
joint_angles = self.ik_solver.solve(target_pos, target_rpy)
```

---

## 🎮 디버깅 치트시트

### 상태 확인 명령어
```python
# 전체 상태 출력
print("="*50)
print(f"시간: {self.data.time:.2f}초")
print(f"베이스: x={self.data.qpos[0]:.2f}, y={self.data.qpos[1]:.2f}")
print(f"팔 관절: {np.round(self.data.qpos[3:10], 2)}")
print(f"그리퍼: {self.data.ctrl[10]}")
print("="*50)
```

### 박스 추적
```python
# 모든 박스 위치 출력
boxes = ["red_box", "blue_box", "green_box", "yellow_box"]
for box_name in boxes:
    box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, box_name)
    pos = self.data.xpos[box_id]
    print(f"{box_name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
```

### 접촉 확인
```python
# 그리퍼 접촉력 확인
for i in range(self.data.ncon):
    contact = self.data.contact[i]
    if contact.geom1 in [left_pad_id, right_pad_id]:
        force = np.linalg.norm(contact.force)
        print(f"접촉력: {force:.3f}N")
```

---

## 📊 중요 인덱스 참조표

### qpos (상태) 인덱스
| 인덱스 | 의미 | 단위 | 범위 |
|--------|------|------|------|
| 0 | 베이스 X | m | -5 ~ 5 |
| 1 | 베이스 Y | m | -5 ~ 5 |
| 2 | 베이스 회전 | rad | -π ~ π |
| 3 | 팔 관절 1 | rad | -2.9 ~ 2.9 |
| 4 | 팔 관절 2 | rad | -2.1 ~ 2.1 |
| 5 | 팔 관절 3 | rad | -2.9 ~ 2.9 |
| 6 | 팔 관절 4 | rad | -2.1 ~ 2.1 |
| 7 | 팔 관절 5 | rad | -2.9 ~ 2.9 |
| 8 | 팔 관절 6 | rad | -2.1 ~ 2.1 |
| 9 | 팔 관절 7 | rad | -2.9 ~ 2.9 |
| 10 | 그리퍼 | - | 0 ~ 1 |

### ctrl (제어) 인덱스
| 인덱스 | 의미 | 값 범위 |
|--------|------|---------|
| 0 | 베이스 X 속도 | -1 ~ 1 |
| 1 | 베이스 Y 속도 | -1 ~ 1 |
| 2 | 베이스 회전 속도 | -1 ~ 1 |
| 3-9 | 팔 관절 토크 | -100 ~ 100 |
| 10 | 그리퍼 제어 | 0 ~ 255 |

---

## 🔄 작업 흐름 순서

### Pick & Place 단계별 실행
```python
# 1단계: 실행 가능성 체크
feasible, msg = self.feasibility_checker.check_pick_and_place_feasibility(
    pick_pos, place_pos
)

# 2단계: 웨이포인트 생성
waypoints = self.waypoint_gen.generate_pick_place_waypoints(
    pick_pos, place_pos
)

# 3단계: 접근
self._move_to_pose(waypoints[0][0], waypoints[0][1], 0)  # 접근 시작
self._move_to_pose(waypoints[1][0], waypoints[1][1], 0)  # 파지 위치

# 4단계: 파지
self.shared_gripper_ctrl[0] = 255
success = self.grasp_checker.wait_until_grasped()

# 5단계: 이동
self._move_to_pose(waypoints[2][0], waypoints[2][1], 255)  # 들어올리기
self._move_to_pose(waypoints[3][0], waypoints[3][1], 255)  # 이동
self._move_to_pose(waypoints[4][0], waypoints[4][1], 255)  # 놓기 위치

# 6단계: 릴리즈
self.shared_gripper_ctrl[0] = 0

# 7단계: 복귀
self._move_to_pose(waypoints[5][0], waypoints[5][1], 0)  # 복귀
```

---

## 🛠️ 커스터마이징 가이드

### 새로운 박스 추가
```xml
<!-- scene.xml에 추가 -->
<body name="new_box" pos="1 1 0.025">
    <geom type="box" size="0.025 0.025 0.025" rgba="1 0 1 1"/>
    <freejoint/>
</body>
```

### 작업 추가
```python
# pick_and_place.py의 tasks 리스트에 추가
self.tasks.append({
    "pick": "new_box",
    "place": "target_box",
    "name": "새로운 작업",
    "room": "방 5",
    "location": "(0, 0)"
})
```

### 이동 속도 조정
```python
# constants.py에서 수정
BASE_LIN_STEP = 0.004  # 기본 0.002 → 2배 빠르게
BASE_YAW_STEP = 0.003  # 회전 속도 증가
```

---

## 💻 자주 발생하는 에러와 해결

### 1. AttributeError: 'NoneType' object
```python
# 원인: 초기화 안 됨
# 해결: 
if self.arm_controller is None:
    self._setup_controllers()
```

### 2. IndexError: index out of bounds
```python
# 원인: 잘못된 인덱스
# 해결: 
# qpos는 0-10, ctrl은 0-10 범위 확인
assert 0 <= index <= 10
```

### 3. ValueError: Body 'xxx_box' not found
```python
# 원인: 박스 이름 오타
# 해결:
try:
    box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, box_name)
except:
    print(f"박스 '{box_name}'을 찾을 수 없습니다")
```

---

## 📈 성능 모니터링

### FPS 확인
```python
# viewer_manager.py에 추가
fps = 1.0 / (time.time() - self.last_time)
print(f"FPS: {fps:.1f}")
```

### 제어 주기 확인
```python
# 제어 루프 시간 측정
start = time.time()
# ... 제어 코드 ...
elapsed = time.time() - start
print(f"제어 시간: {elapsed*1000:.2f}ms")
```

---

## 🎯 최적화 팁

### 1. 빠른 시뮬레이션
```python
# timestep 증가 (정확도는 감소)
self.model.opt.timestep = 0.004  # 기본 0.002
```

### 2. 뷰어 업데이트 감소
```python
# 매 프레임이 아닌 n번째 프레임만 업데이트
if self.frame_count % 2 == 0:
    self.viewer.sync()
```

### 3. 불필요한 계산 제거
```python
# IK는 필요할 때만
if distance_to_target > threshold:
    q_target = self.ik_solver.solve(...)
```

---

## 🔗 유용한 링크

- [MuJoCo 함수 레퍼런스](https://mujoco.readthedocs.io/en/latest/APIreference.html)
- [NumPy 치트시트](https://numpy.org/doc/stable/user/numpy-for-matlab-users.html)
- [Python 디버깅 가이드](https://docs.python.org/3/library/pdb.html)

---

**빠른 도움말**: 코드에서 `Ctrl+F`로 필요한 패턴을 검색하세요!
