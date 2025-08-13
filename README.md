# MuJoCo Tidybot Pick-and-Place

Stanford Tidybot 모델을 사용한 MuJoCo 물리 엔진 기반 pick-and-place 로봇 매니퓰레이션 시뮬레이션

## 주요 기능

- **역기구학(IK)**: SLSQP 최적화를 통한 실시간 관절 각도 계산
- **부드러운 궤적 생성**: Ruckig 라이브러리를 활용한 7-DOF 팔의 jerk-limited 모션 계획  
- **베이스 이동 제어**: 키보드 텔레오퍼레이션 및 팔 안정화
- **힘 기반 파지 감지**: 접촉력 기반 파지 검증
- **대화형 GUI**: MuJoCo viewer를 통한 실시간 3D 시각화

## 설치 방법

### 요구사항
- Python 3.8+
- Ubuntu 20.04+ / Windows 10+ / macOS 10.15+
- OpenGL 3.3+ 호환 그래픽

### 의존성 설치

```bash
pip install mujoco numpy scipy keyboard ruckig
```

또는 requirements.txt 사용:

```bash
pip install -r requirements.txt
```

### 설정

`main.py`에서 모델 경로를 수정하세요:

```python
# main.py
XML_PATH = "./model/stanford_tidybot/scene.xml"
```

## 실행

```bash
python main.py
```

## 조작 방법

| 키 | 동작 |
|---|------|
| `Space` | pick-and-place 시퀀스 시작 |
| `8` / `5` | 베이스 전진 / 후진 |
| `4` / `6` | 베이스 좌 / 우 이동 |
| `7` / `9` | 베이스 좌회전 / 우회전 |
| `2` | 베이스 정지 |
| `ESC` | 시뮬레이션 종료 |

## 프로젝트 구조

```
.
├── model/
│   └── stanford_tidybot/
│       └── scene.xml              # MuJoCo 모델 정의
├── main.py                        # 메인 실행 파일 (Space 키 트리거)
├── simulation.py                  # 시뮬레이션 오케스트레이션/상태 관리
├── arm_controller.py              # Ruckig + 토크 제어 기반 팔 제어
├── mobility_controller.py         # 베이스 텔레오퍼레이션 & 중력 보상
├── ik_solver.py                   # SLSQP 기반 역기구학 솔버
├── grasp_checker.py               # 접촉력 기반 파지 검증
└── README.md                      # 이 문서
```

## 핵심 컴포넌트

### Arm Controller (`arm_controller.py`)
- Computed-torque 제어 + PD 피드백
- Ruckig을 사용한 jerk-limited 궤적 생성
- 7-DOF 팔의 실시간 토크 계산

### Mobility Controller (`mobility_controller.py`)
- 베이스/팔 분리 제어 아키텍처
- 텔레오퍼레이션 중 중력 보상
- 스레드 안전 명령 버퍼 관리

### IK Solver (`ik_solver.py`)
- 위치/방향 오차 최소화를 위한 SLSQP 최적화
- 관절 한계 적용
- 실시간 성능 최적화

### Grasp Checker (`grasp_checker.py`)
- 접촉력 임계값 감지 (0.05 N)
- 패드 거리 모니터링
- 실시간 파지 상태 피드백

## 기술 사양

| 파라미터 | 값 |
|---------|-----|
| 제어 주기 | 500 Hz (2ms timestep) |
| 팔 속도 제한 | 3 rad/s |
| 팔 가속도 제한 | 7 rad/s² |
| 팔 저크 제한 | 150 rad/s³ |
| 파지 힘 임계값 | 0.05 N |
| PD 게인 (팔) | Kp = 1500, Kd = 30 |

## 작동 흐름

### 1. 초기화
- MuJoCo 모델 로드 및 컨트롤러 초기화

### 2. 텔레오퍼레이션 모드
- 키보드로 베이스 제어
- 팔은 현재 자세 유지

### 3. Pick 시퀀스 (Space 키)
- 접근 위치 IK 계산
- 파지 자세로 이동
- 그리퍼 닫고 파지 확인
- 물체 들어올리기

### 4. Place 시퀀스
- 목표 위치로 이동
- 물체 내려놓기
- 그리퍼 열기
- 홈 포지션 복귀

### 5. 텔레오퍼레이션 재개

## 구현 세부사항

- **스레드 안전성**: 모든 `mjData` 접근은 락으로 직렬화되어 GUI/제어 루프 충돌 방지
- **실시간 제어**: 모빌리티와 팔 제어를 위한 별도 스레드로 부드러운 작동 보장
- **힘 피드백**: MuJoCo 내장 접촉 솔버를 사용한 접촉력 계산
- **궤적 부드러움**: Ruckig이 C³ 연속 궤적 보장 (연속 저크)

## 문제 해결

### 모델을 찾을 수 없음
```
FileNotFoundError: XML file not found
```
**해결**: `main.py`의 `XML_PATH`가 올바른 모델 파일을 가리키는지 확인

### 키보드 입력이 작동하지 않음
```
PermissionError: Keyboard access denied
```
**해결**: Linux에서 sudo로 실행하거나 Windows에서 관리자 권한으로 실행

### 파지 감지 실패
- `grasp_checker.py`에서 힘 임계값 조정
- 그리퍼 패드 충돌 형상 확인
- 물체 질량 및 마찰 파라미터 검증

## 성능 최적화

- 제어 성능 향상을 위해 viewer 업데이트 빈도 감소
- 특정 로봇 구성에 맞게 PD 게인 조정
- 액추에이터 성능에 따라 Ruckig 제한 조정

- Ruckig 라이브러리 - 궤적 생성
