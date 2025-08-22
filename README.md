# 🤖 MuJoCo Tidybot Pick & Place Simulation v2.1

Stanford Tidybot 모델을 사용한 고급 Pick & Place 시뮬레이션 시스템입니다.  
두 개의 연속적인 Pick & Place 작업을 수행하며, 실행 가능성 체크와 로봇 헤딩 기준 조종을 지원합니다.

![MuJoCo](https://img.shields.io/badge/MuJoCo-2.3.0+-blue)
![Python](https://img.shields.io/badge/Python-3.8+-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

---

## 📋 목차
- [주요 기능](#-주요-기능)
- [시스템 요구사항](#-시스템-요구사항)
- [설치 방법](#-설치-방법)
- [실행 방법](#-실행-방법)
- [조작 방법](#-조작-방법)
- [프로젝트 구조](#-프로젝트-구조)
- [기술적 특징](#-기술적-특징)
- [문제 해결](#-문제-해결)

---

## ✨ 주요 기능

### 1. **듀얼 Pick & Place 작업**
- 🔴 → 🔵 작업 1: 빨간색 박스를 파란색 박스 위로 이동
- 🟢 → 🟡 작업 2: 초록색 박스를 노란색 박스 위로 이동
- 각 작업은 독립적으로 실행 가능

### 2. **실행 가능성 체크 시스템**
- 작업 실행 전 도달 가능성 자동 검증
- IK(역기구학) 솔루션 존재 여부 확인
- 실패 시 최적 베이스 위치 제안

### 3. **로봇 헤딩 기준 조종**
- 로봇이 바라보는 방향 기준의 직관적 조작
- 월드 좌표계가 아닌 로봇 좌표계 기준 이동

### 4. **고급 제어 기능**
- Ruckig 기반 부드러운 궤적 생성
- 토크 제어 기반 정밀 제어
- 접촉력 기반 파지 검증

---

## 💻 시스템 요구사항

### 최소 요구사항
- **OS**: Windows 10/11, Ubuntu 20.04+, macOS 11+
- **Python**: 3.8 이상
- **RAM**: 8GB 이상
- **GPU**: OpenGL 3.3+ 지원

### 권장 사양
- **CPU**: Intel i5 또는 AMD Ryzen 5 이상
- **RAM**: 16GB
- **GPU**: 전용 그래픽 카드

---

## 📦 설치 방법

### 1. 저장소 클론
```bash
git clone <repository-url>
cd Mujoco_local_structured/code2
```

### 2. 가상환경 생성 (권장)
```bash
python -m venv venv

# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate
```

### 3. 의존성 설치
```bash
pip install -r requirements.txt
```

### 필수 패키지
```
mujoco>=2.3.0
numpy>=1.20.0
scipy>=1.7.0
keyboard>=0.13.5
ruckig>=0.9.0
```

---

## 🚀 실행 방법

### 기본 실행
```bash
python main.py
```

### 실행 흐름
1. **시뮬레이션 초기화**
   - MuJoCo 환경 로드
   - 로봇 및 박스 초기 배치

2. **작업 실행**
   - `[Space]` 키로 시뮬레이션 시작
   - 각 작업마다 `[Space]` 키로 진행

3. **실행 가능성 체크**
   - 자동으로 도달 가능성 검증
   - 불가능 시 베이스 조정 안내

---

## 🎮 조작 방법

### 시뮬레이션 제어
| 키 | 기능 |
|---|------|
| `Space` | 시뮬레이션 시작 / 다음 작업 진행 |
| `ESC` | 프로그램 종료 |

### 베이스 이동 (로봇 헤딩 기준)
| 키 | 기능 | 설명 |
|---|------|------|
| `8` | 전진 | 로봇이 바라보는 방향으로 이동 |
| `5` | 후진 | 뒤로 이동 |
| `4` | 좌측 이동 | 로봇 기준 왼쪽으로 이동 |
| `6` | 우측 이동 | 로봇 기준 오른쪽으로 이동 |
| `7` | 좌회전 | 반시계 방향 회전 |
| `9` | 우회전 | 시계 방향 회전 |
| `2` | 정지 | 모든 이동 정지 |

> 💡 **팁**: 작업 실패 시 베이스를 조정한 후 재시도할 수 있습니다.

---

## 📁 프로젝트 구조

```
code2/
├── 📄 main.py                    # 메인 실행 파일
├── 📁 model/                     # MuJoCo 모델 파일
│   └── stanford_tidybot/
│       ├── scene.xml            # 메인 씬 정의
│       ├── tidybot.xml          # 로봇 모델
│       └── assets/              # 메시 파일들
├── 📁 config/                    # 설정 파일
│   ├── constants.py             # 상수 정의
│   └── robot_config.py          # 로봇 설정
├── 📁 simulation/                # 시뮬레이션 관리
│   ├── simulation_manager.py    # 통합 매니저
│   └── viewer_manager.py        # 뷰어 관리
├── 📁 controllers/               # 제어기
│   ├── arm/                    # 팔 제어
│   │   ├── arm_controller.py
│   │   ├── trajectory_tracker.py
│   │   └── torque_controller.py
│   ├── base/                   # 베이스 제어
│   │   ├── mobility_controller.py
│   │   ├── keyboard_handler.py
│   │   └── base_teleop.py
│   └── gripper/                # 그리퍼 제어
│       └── grasp_checker.py
├── 📁 kinematics/               # 기구학
│   └── ik_solver.py            # 역기구학 솔버
├── 📁 tasks/                    # 작업 정의
│   ├── pick_and_place.py      # Pick & Place 작업
│   ├── feasibility_checker.py  # 실행 가능성 체크
│   └── waypoint_generator.py   # 경로점 생성
└── 📁 utils/                    # 유틸리티
    └── thread_manager.py        # 스레드 관리
```

---

## 🔧 기술적 특징

### 1. **역기구학 (IK) 솔버**
- SLSQP 최적화 기반
- 위치 및 자세 오차 최소화
- 관절 한계 고려

### 2. **궤적 생성 (Ruckig)**
- Jerk-limited 모션 계획
- 7-DOF 팔 제어
- 부드러운 움직임 보장

### 3. **실행 가능성 체크**
- 도달 범위: 0.2m ~ 0.8m
- 높이 제한: -0.1m ~ 0.6m
- IK 솔루션 검증

### 4. **파지 검증**
- 접촉력 기반 검증
- 임계값: 0.05N
- 실시간 피드백

### 5. **제어 파라미터**
```python
# 베이스 제어
BASE_LIN_STEP = 0.0008  # 선속도 스텝
BASE_YAW_STEP = 0.0010  # 각속도 스텝

# 팔 제어
ARM_KP_HOLD = 400.0     # P 게인
ARM_KD_HOLD = 25.0      # D 게인

# Ruckig 파라미터
RUCKIG_MAX_V = 3.0      # 최대 속도 (rad/s)
RUCKIG_MAX_A = 7.0      # 최대 가속도 (rad/s²)
RUCKIG_MAX_J = 150.0    # 최대 저크 (rad/s³)
```

---

## 🐛 문제 해결

### 1. **"Pick & Place 수행 불가능" 메시지**
- **원인**: 목표가 로봇 팔 도달 범위 밖
- **해결**: 제안된 위치로 베이스 이동 후 재시도

### 2. **키보드 입력이 작동하지 않음**
- **원인**: 포커스 문제
- **해결**: MuJoCo 뷰어 창 클릭 후 조작

### 3. **시뮬레이션이 느림**
- **원인**: 하드웨어 성능 부족
- **해결**: 
  - 뷰어 업데이트 간격 조정
  - 그래픽 드라이버 업데이트

### 4. **모델 로드 실패**
- **원인**: 파일 경로 문제
- **해결**: `DEFAULT_XML_PATH` 확인 및 수정

---

## 📈 성능 지표

- **제어 주기**: 500Hz (2ms)
- **IK 수렴 시간**: < 100ms
- **파지 검증 시간**: < 3초
- **전체 작업 시간**: 작업당 약 20-30초

---

## 🤝 기여 방법

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📝 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

---

## 🙏 감사의 글

- Stanford Tidybot 팀 - 로봇 모델 제공
- MuJoCo 개발팀 - 시뮬레이션 엔진
- Ruckig 라이브러리 - 궤적 생성

---

## 📞 문의

프로젝트 관련 문의사항이 있으시면 Issue를 생성해주세요.

---

**마지막 업데이트**: 2024년 1월
