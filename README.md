# MuJoCo 4-Room Pick & Place Simulation

## 📋 개요
MuJoCo를 사용한 4개 방 Pick & Place 시뮬레이션입니다. Stanford Tidybot 로봇이 10m x 10m 공간의 4개 방을 순차적으로 이동하며 색상별로 매칭된 박스 작업을 수행합니다.

## 🚀 빠른 시작

### 1. 설치
```bash
# 필수 패키지 설치
pip install -r requirements.txt
```

### 2. 실행
```bash
python main.py
```

## 🎮 조작법

### 로봇 이동 (숫자패드 사용)
- **8** - 전진 (로봇이 바라보는 방향)
- **5** - 후진
- **4** - 좌측 이동 (로봇 기준)
- **6** - 우측 이동 (로봇 기준)
- **7** - 좌회전
- **9** - 우회전
- **2** - 정지

### 작업 제어
- **Space** - Pick & Place 시작

### 작업 진행 방식
1. 프로그램 실행 후 Space키로 시작
2. 각 방에서 Space키를 눌러 작업 수행
3. 작업은 자동으로 진행 (Pick & Place)
4. 실패 시 베이스 위치 조정 후 Space로 재시도

## 📍 환경 구조

```
        방 1 (북서)          방 2 (북동)
        [-2.5, 2.5]          [2.5, 2.5]
        빨강 → 파랑          초록 → 노랑
        
                로봇 시작점
                  [0, 0]
                
        방 3 (남서)          방 4 (남동)
        [-2.5, -2.5]         [2.5, -2.5]
        주황 → 보라          청록 → 분홍
```

## 🎯 작업 목표

각 방에서 순차적으로 Pick & Place 작업 수행:

1. **방 1 (북서)**: 빨간 박스(red_box) → 파란 박스(blue_box)
2. **방 2 (북동)**: 초록 박스(green_box) → 노란 박스(yellow_box)
3. **방 3 (남서)**: 주황 박스(orange_box) → 보라 박스(purple_box)
4. **방 4 (남동)**: 청록 박스(cyan_box) → 분홍 박스(pink_box)

## 📁 프로젝트 구조

```
code2/
├── main.py              # 메인 실행 파일
├── requirements.txt     # Python 패키지
├── README.md           # 이 문서
├── model/              # MuJoCo 모델
│   └── stanford_tidybot/
│       ├── scene.xml   # 4개 방 환경
│       ├── tidybot.xml # 로봇 모델
│       └── assets/     # 3D 메시 파일
├── simulation/         # 시뮬레이션 관리
│   ├── simulation_manager.py
│   └── viewer_manager.py
├── controllers/        # 제어기
│   ├── arm/           # 팔 제어
│   ├── base/          # 베이스 제어
│   └── gripper/       # 그리퍼 제어
├── tasks/             # 작업 정의
│   ├── pick_and_place.py
│   ├── feasibility_checker.py
│   └── waypoint_generator.py
├── kinematics/        # 기구학
│   └── ik_solver.py
├── config/            # 설정
│   ├── constants.py
│   └── robot_config.py
└── utils/             # 유틸리티
    └── thread_manager.py
```

## 🔧 문제 해결

### 작업이 실패하는 경우
- 로봇이 박스에서 너무 멀리 있음
- 콘솔에 표시된 제안 위치로 이동
- 일반적으로 0.2m ~ 0.8m 거리가 적절

### 낮은 FPS
- `scene.xml`의 timestep 조정 (기본: 0.002)
- 그래픽 드라이버 업데이트

## 💡 사용 팁

1. **시작하기**: Space키로 시작, 각 방에서 Space로 작업 실행
2. **방 이동**: 중앙 통로를 통해 방 사이 이동
3. **작업 과정**:
   - 해당 방으로 이동
   - Space키로 작업 시작
   - 자동으로 Pick & Place 수행
   - 실패 시 위치 조정 후 재시도
4. **상태 확인**: 콘솔에서 진행 상황 확인

## 📊 기술 사양

- **MuJoCo 버전**: 2.3.0 이상
- **Python 버전**: 3.8 이상
- **환경 크기**: 10m x 10m (4개 방)
- **로봇**: Stanford Tidybot (7-DOF 팔 + 2-finger 그리퍼)
- **제어 주기**: 500Hz (timestep: 0.002초)
- **베이스 속도**: 0.002 m/step
- **회전 속도**: 0.002 rad/step

## ✅ 주요 기능

- 4개 독립된 방과 순차적 작업
- 로봇 헤딩 기준 직관적 조종
- 자동 Pick & Place 실행
- 실행 가능성 자동 체크
- 실패 시 베이스 위치 제안
- 멀티스레딩 베이스/팔 제어
- Ruckig 기반 부드러운 궤적 생성
- 접촉력 기반 파지 확인

## 📝 참고사항

- 작업은 방1→방2→방3→방4 순서로 진행
- 각 작업 전 Space키 대기
- 베이스는 작업 중 자동 정지
- 그리퍼는 자동 제어 (수동 조작 불가)
- IK 솔버로 실시간 궤적 계산

## 🔄 버전 정보

- **현재 버전**: v2.2
- **최종 업데이트**: 2025-08-22
- **주요 업데이트**:
  - 실행 가능성 체크 시스템
  - 로봇 헤딩 기준 조종
  - 베이스 위치 제안 기능

## 📄 라이선스

MIT License

---

**개발자**: Hoyong Na  
**GitHub**: https://github.com/HoyongNa/mujoco_pick_place