# MuJoCo Tidybot Pick-and-Place

> **Stanford Tidybot** 모델을 이용해 MuJoCo에서 **pick-and-place** 조작을 수행하는 로보틱 매니퓰레이션 시뮬레이션

---

## 📚 Table of Contents
- [Features](#-features)
- [Project Structure](#-project-structure)
- [Requirements & Installation](#-requirements--installation)
- [Configuration](#-configuration)
- [Run](#-run)
- [Controls](#-controls)
- [Key Components](#-key-components)
- [Technical Details](#-technical-details)
- [License](#-license)

---

## ✨ Features
- **Inverse Kinematics (IK)**: 목표 EE 포즈에 대한 실시간 조인트 각도 계산  
- **Smooth Trajectory (Ruckig)**: 7-DOF 팔의 **jerk-limited** 모션 계획  
- **Base Mobility Control**: 키보드 텔레오퍼레이션 + 동시 팔 안정화  
- **Force-based Grasp Detection**: 접촉력 기반 파지 여부 검증  
- **Interactive GUI**: MuJoCo viewer 기반 실시간 3D 시각화

---

## 🗂 Project Structure
```text
.
├─ main.py                 # Space-bar 트리거가 있는 엔트리 포인트
├─ simulation.py           # 시뮬레이션 오케스트레이터
├─ arm_controller.py       # Ruckig + 토크 제어 기반 팔 제어
├─ mobility_controller.py  # 베이스 텔레오퍼레이션 & 팔 중력보상
├─ ik_solver.py            # 역기구학(IK) 솔버
└─ grasp_checker.py        # 접촉 기반 파지 검증
📦 Requirements & Installation
bash
복사
편집
pip install mujoco numpy scipy keyboard ruckig
⚙️ Configuration
main.py의 모델 경로를 실제 파일 위치로 설정하세요.

python
복사
편집
# main.py
XML_PATH = "path/to/stanford_tidybot/scene.xml"
▶️ Run
bash
복사
편집
python main.py
🎮 Controls
Key	Action
Space	pick-and-place 시퀀스 시작
8 / 5	베이스 전진 / 후진
4 / 6	베이스 좌 / 우 이동
7 / 9	베이스 반시계 / 시계 방향 회전
2	베이스 정지
ESC	시뮬레이션 종료

🧩 Key Components
Arm Controller
Computed-torque control + PD feedback

선택적 Disturbance Observer (DOB)

Ruckig 기반 jerk-limited 트래젝터리

Mobility Controller
베이스와 팔의 decoupled 제어

텔레오퍼레이션 중 gravity compensation

Thread-safe 커맨드 버퍼 공유

IK Solver
SLSQP 기반 7-DOF redundacy 최적화

위치/자세 오차 최소화, joint limit 준수

🧠 Architecture (Mermaid)
mermaid
복사
편집
flowchart LR
  K[Keyboard] -->|teleop| MC[mobility_controller.py]
  MC --> SIM[simulation.py]
  AC[arm_controller.py] --> SIM
  IK[ik_solver.py] --> AC
  GC[grasp_checker.py] --> AC
  SIM -->|render/step| GUI[MuJoCo Viewer]
🔧 Technical Details
Control Rate: 모델 타임스텝(기본 2 ms)

Arm Limits: 3 rad/s(vel), 7 rad/s²(acc), 150 rad/s³(jerk)

Grasp Detection: 0.05 N 임계값

PD Gains: Kp = 1500, Kd = 30 (팔 추적)

📄 License
MIT

markdown
복사
편집

위 내용은 사용자가 제공하신 기존 README를 구조화하여 재작성한 것입니다. :contentReference[oaicite:0]{index=0}

추가로 원하시면 저장소 상단에 보일 **프로젝트 썸네일/GIF**(예: `docs/demo.gif`)와 **배지(버전/라이선스)**도 넣어 드릴 수 있습