# MuJoCo Tidybot Pick-and-Place

Stanford Tidybot 모델로 MuJoCo에서 pick-and-place를 수행하는 로보틱 매니퓰레이션 시뮬레이션.

---

## Contents
- Features
- Project Structure
- Requirements & Installation
- Configuration
- Run
- Controls
- Key Components
- Technical Details
- License

---

## Features
- Inverse Kinematics: 목표 EE 포즈에 대한 실시간 조인트 각도 계산
- Smooth Trajectory (Ruckig): 7-DOF 팔의 jerk-limited 모션 계획
- Base Mobility Control: 키보드 텔레오퍼레이션 + 팔 안정화
- Force-based Grasp Detection: 접촉력 기반 파지 여부 검증
- Interactive GUI: MuJoCo viewer 기반 실시간 3D 시각화

---

## Project Structure
```text
.
├─ model/                      # MuJoCo 모델 및 에셋 (scene.xml 등)
├─ main.py                     # 실행 엔트리(스페이스바 트리거)
├─ simulation.py               # 시뮬레이션 오케스트레이션/상태 관리
├─ arm_controller.py           # Ruckig + 토크 제어 기반 팔 제어
├─ mobility_controller.py      # 베이스 텔레오퍼레이션 & 중력보상
├─ ik_solver.py                # SLSQP 기반 역기구학 솔버
└─ grasp_checker.py            # 접촉력 기반 파지 검증
Requirements & Installation
bash
복사
편집
pip install mujoco numpy scipy keyboard ruckig
Configuration
main.py의 모델 경로를 실제 파일 위치로 수정합니다.

python
복사
편집
# main.py
XML_PATH = "path/to/stanford_tidybot/scene.xml"
Run
bash
복사
편집
python main.py
Controls
Key	Action
Space	pick-and-place 시퀀스 시작
8 / 5	베이스 전진 / 후진
4 / 6	베이스 좌 / 우 이동
7 / 9	베이스 반시계 / 시계 회전
2	베이스 정지
ESC	시뮬레이션 종료

Key Components
Arm Controller: Computed-torque + PD, Ruckig 기반 jerk-limited 트래젝터리

Mobility Controller: 베이스/팔 decoupled 제어, 텔레오퍼레이션 중 gravity compensation

IK Solver: SLSQP로 위치/자세 오차 최소화, 조인트 한계 준수

Grasp Checker: 접촉력/임계값 기반 파지 판정

구현 메모: simulation.py에서 mjData 접근을 직렬화(락)하여 GUI와 제어 루프 충돌을 방지합니다.

Technical Details
Control rate: 모델 타임스텝(기본 2 ms)

Arm limits: 3 rad/s(vel), 7 rad/s²(acc), 150 rad/s³(jerk)

Grasp detection: 0.05 N 임계값

PD gains: Kp = 1500, Kd = 30 (팔 추적)

License
MIT
