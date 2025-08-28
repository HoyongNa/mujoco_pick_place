# 🚀 Lidar Mapping System

깔끔하고 모듈화된 라이다 매핑 시스템입니다.

## 📁 구조

```
lidar_mapping/
├── __init__.py          # 패키지 초기화
├── lidar_sensor.py      # 라이다 센서 데이터 처리
├── occupancy_grid.py    # Occupancy Grid Map 구현
├── mapping_system.py    # 통합 매핑 시스템
└── visualizer.py        # 실시간 시각화 (Main Thread)
```

## 🎯 주요 특징

### 1. **모듈화된 설계**
- 각 컴포넌트가 독립적으로 동작
- 쉬운 유지보수 및 확장

### 2. **Thread-Safe 구현**
- Background mapping thread
- Main thread visualization
- Thread-safe map updates

### 3. **Aggressive Parameters**
- 빠른 벽 감지를 위한 파라미터 설정
- 단일 스캔에서도 벽 감지 가능

## 🚀 사용법

### 간단한 테스트
```bash
python test_lidar_simple.py
```

### 인터랙티브 데모
```bash
python test_lidar_interactive.py
```

### Python 코드에서 사용
```python
from lidar_mapping import LidarMappingSystem, MapVisualizer

# 초기화
mapping = LidarMappingSystem(model, data)

# 단일 업데이트
stats = mapping.update_once()

# 연속 매핑 시작
mapping.start_mapping(update_rate=10)

# 시각화
viz = MapVisualizer(mapping)
viz.setup_figure()
viz.update_once()
```

## 🎮 컨트롤 (Interactive Demo)

- **WASD**: 로봇 이동
- **Q/E**: 로봇 회전
- **Space**: 매핑 시작/정지
- **V**: 시각화 토글
- **C**: 맵 초기화
- **R**: 로봇 위치 리셋
- **S**: 맵 저장
- **ESC**: 종료

## ⚙️ 파라미터 조정

### Lidar Sensor (`lidar_sensor.py`)
```python
self.min_range = 0.8  # 최소 거리 (로봇 자체 필터링)
self.max_range = 6.0  # 최대 거리
```

### Occupancy Grid (`occupancy_grid.py`)
```python
self.log_odds_occupied = 3.0    # 벽 감지 강도
self.log_odds_free = -0.5       # Free space 강도
self.prob_threshold_occupied = 0.6  # Occupied 임계값
self.prob_threshold_free = 0.4      # Free 임계값
```

## 📊 통계 정보

매핑 시스템은 다양한 통계 정보를 제공합니다:

- `total_scans`: 총 스캔 횟수
- `valid_readings`: 유효한 센서 읽기 수
- `occupied_cells`: Occupied로 표시된 셀 수
- `free_cells`: Free로 표시된 셀 수
- `coverage_percent`: 맵 커버리지 비율
- `inner_wall_detections`: 내부 벽 감지 수
- `outer_wall_detections`: 외부 벽 감지 수

## 🔧 문제 해결

### 1. 벽이 감지되지 않는 경우
- `log_odds_occupied` 값 증가
- `prob_threshold_occupied` 값 감소
- `min_range` 값 확인 (너무 크면 벽 필터링)

### 2. 노이즈가 많은 경우
- `log_odds_free` 절대값 감소
- 벽 주변 셀 마킹 비활성화

### 3. 매핑이 느린 경우
- `update_rate` 감소
- 맵 크기 감소
- Resolution 증가 (세밀도 감소)

## 📈 성능 최적화

1. **Background Thread**: 매핑은 별도 스레드에서 실행
2. **Bresenham Algorithm**: 효율적인 ray tracing
3. **NumPy Vectorization**: 빠른 배열 연산
4. **Lock-based Synchronization**: Thread-safe 업데이트

## 🎉 예상 결과

성공적인 매핑 시:
- 첫 스캔에서 100+ occupied cells
- 10초 후 500+ occupied cells  
- Coverage 10% 이상
- 내부/외부 벽 명확히 구분

## 📝 라이센스

MIT License
