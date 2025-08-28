# MuJoCo 4-Room Navigation & Pick-Place System

## ✅ 모든 오류 해결 완료!

### 수정 내역 (2025.01.15)
1. ✅ `path_planning/__init__.py` 생성 - import 오류 해결
2. ✅ `MapProcessor` 개선 - log_odds와 occupancy_grid 모두 지원
3. ✅ `AStarPlanner` 수정 - `is_valid` → `is_free` 메서드 사용
4. ✅ `PathFollowingController` 단순화 - 웨이포인트 직접 추종

## 🚀 실행

```bash
# 1. 시스템 체크
python final_test.py

# 2. 메인 실행
python main.py
```

## 🎮 조작법

| 키 | 기능 |
|---|------|
| **1** | 방 1 (북서, -2.5, 2.5)로 이동 |
| **2** | 방 2 (북동, 2.5, 2.5)로 이동 |
| **3** | 방 3 (남서, -2.5, -2.5)로 이동 |
| **4** | 방 4 (남동, 2.5, -2.5)로 이동 |
| **Space** | Pick & Place 실행 |
| **8/5** | 전진/후진 (로봇 기준) |
| **4/6** | 좌/우 이동 (로봇 기준) |
| **7/9** | 좌/우 회전 |
| **2** | 정지 |
| **ESC** | 종료 |

## 📍 환경 구조

```
10m x 10m 환경 (4개 방)

┌───────┬───────┐
│ 방 1  │ 방 2  │
│ 빨강→ │ 초록→ │
│ 파랑  │ 노랑  │
├───────┼───────┤
│ 방 3  │ 방 4  │
│ 주황→ │ 청록→ │
│ 보라  │ 분홍  │
└───────┴───────┘
```

## 🧪 테스트

```bash
# 시스템 체크
python final_test.py

# 맵 시각화
python view_saved_map.py
```

## 📁 핵심 파일

```
code2/
├── main.py                         # 메인 실행
├── final_test.py                   # 시스템 체크
├── lidar_map_20250826_215447.npz  # 환경 맵
├── path_planning/
│   ├── __init__.py                # ✅ 패키지 초기화
│   ├── map_processor.py           # ✅ 맵 처리
│   ├── astar_planner.py           # ✅ A* 경로 계획
│   ├── pure_pursuit.py            # Pure Pursuit 추종
│   └── path_following_controller.py # ✅ 경로 추종 컨트롤러
├── controllers/                    # 로봇 제어
├── simulation/                     # 시뮬레이션 관리
└── tasks/                         # Pick & Place 작업
```

## ⚙️ 시스템 구조

### 베이스 제어 방식
- **MuJoCo Position Actuator** 사용
- `ctrl[0]`: X 위치 (kp=1000000, kv=50000)
- `ctrl[1]`: Y 위치 (kp=1000000, kv=50000)
- `ctrl[2]`: Theta 각도 (kp=50000, kv=1000)

### 경로 계획 흐름
1. **맵 로드**: `lidar_map_20250826_215447.npz` (log_odds 형식)
2. **A* 경로 생성**: 시작점 → 목표점 최단 경로
3. **웨이포인트 추종**: Pure Pursuit + 직접 위치 제어
4. **도착 감지**: 목표 도달 시 자동 정지

## ⚠️ 필수 요구사항

- Python 3.8+
- MuJoCo 2.3.0+
- NumPy, OpenCV, Matplotlib
- keyboard, ruckig

## ✅ 동작 확인

- ✅ Import 오류 없음
- ✅ 맵 로드 성공
- ✅ 경로 계획 작동
- ✅ 베이스 이동 정상
- ✅ Pick & Place 실행

---

**시스템 준비 완료! `python main.py`로 실행하세요.**
