# Documentation Index

## 프로젝트 문서 가이드

이 문서는 MuJoCo 기반 모바일 매니퓰레이터 시뮬레이션 프로젝트의 전체 문서 구조를 안내합니다.

## 📚 문서 구조

```
docs/
├── INDEX.md                    # 현재 문서 (문서 인덱스)
├── README.md                   # 프로젝트 개요 및 시작 가이드
├── CODE_DOCUMENTATION.md      # 코드 구조 및 API 문서
├── LIDAR_MAPPING.md           # LiDAR 매핑 시스템 상세
├── PATH_PLANNING.md           # 경로 계획 알고리즘 상세
├── ADVANCED_FEATURES.md       # 고급 기능 및 확장
├── QUICK_REFERENCE.md         # 빠른 참조 가이드
└── CODE_READING_CHECKLIST.md  # 코드 리뷰 체크리스트
```

## 🎯 목적별 가이드

### 처음 시작하는 경우
1. [README.md](../README.md) - 프로젝트 개요 확인
2. [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) - 빠른 시작 명령어
3. [CODE_DOCUMENTATION.md](./CODE_DOCUMENTATION.md) - 기본 구조 이해

### 개발자를 위한 가이드
1. [CODE_READING_CHECKLIST.md](./CODE_READING_CHECKLIST.md) - 코드 분석 시작점
2. [CODE_DOCUMENTATION.md](./CODE_DOCUMENTATION.md) - API 상세 문서
3. [ADVANCED_FEATURES.md](./ADVANCED_FEATURES.md) - 고급 기능 구현

### 특정 모듈 이해
- **LiDAR & Mapping**: [LIDAR_MAPPING.md](./LIDAR_MAPPING.md)
- **Path Planning**: [PATH_PLANNING.md](./PATH_PLANNING.md)
- **통합 시스템**: [ADVANCED_FEATURES.md](./ADVANCED_FEATURES.md)

## 📖 문서별 주요 내용

### [README.md](../README.md)
- 프로젝트 소개
- 주요 기능
- 설치 방법
- 기본 사용법
- 프로젝트 구조

### [CODE_DOCUMENTATION.md](./CODE_DOCUMENTATION.md)
- 전체 아키텍처
- 모듈별 상세 설명
- 클래스 다이어그램
- API 레퍼런스
- 데이터 흐름

### [LIDAR_MAPPING.md](./LIDAR_MAPPING.md)
- Occupancy Grid Mapping
- Bresenham 알고리즘
- Log-odds 업데이트
- 맵 저장/로드
- 시각화 도구

### [PATH_PLANNING.md](./PATH_PLANNING.md)
- A* 알고리즘 구현
- 웨이포인트 기반 경로 추종
- 맵 프로세싱
- 경로 최적화
- 파라미터 튜닝

### [ADVANCED_FEATURES.md](./ADVANCED_FEATURES.md)
- 멀티스레드 아키텍처
- 동적 재계획
- 작업 계획 시스템
- 실시간 시각화
- 성능 프로파일링

### [QUICK_REFERENCE.md](./QUICK_REFERENCE.md)
- 자주 사용하는 명령어
- 코드 스니펫
- 파라미터 체크리스트
- 디버깅 팁
- 트러블슈팅

### [CODE_READING_CHECKLIST.md](./CODE_READING_CHECKLIST.md)
- 코드 분석 체크리스트
- 모듈별 핵심 파일
- 알고리즘 이해 포인트
- 리팩토링 기회
- 학습 자료

## 🔄 읽기 순서 추천

### 초급자 경로
```
1. README.md (프로젝트 이해)
   ↓
2. QUICK_REFERENCE.md (빠른 실행)
   ↓
3. CODE_DOCUMENTATION.md (구조 파악)
   ↓
4. LIDAR_MAPPING.md + PATH_PLANNING.md (핵심 기능)
```

### 중급자 경로
```
1. CODE_DOCUMENTATION.md (전체 구조)
   ↓
2. CODE_READING_CHECKLIST.md (코드 분석)
   ↓
3. LIDAR_MAPPING.md + PATH_PLANNING.md (알고리즘 이해)
   ↓
4. ADVANCED_FEATURES.md (고급 기능)
```

### 고급자 경로
```
1. CODE_READING_CHECKLIST.md (빠른 파악)
   ↓
2. ADVANCED_FEATURES.md (확장 가능성)
   ↓
3. 특정 모듈 문서 (필요시 참조)
```

## 💡 활용 팁

### 검색 키워드
각 문서에서 자주 검색되는 키워드:

- **알고리즘**: A*, Waypoint Following, Bresenham, Log-odds
- **클래스**: LidarMapper, AStarPlanner, PathFollowingController
- **파라미터**: waypoint_threshold, step_size, resolution, kernel_size
- **디버깅**: visualization, profiler, logger
- **최적화**: vectorization, threading, caching

### 코드와 함께 읽기
문서를 읽을 때 해당 코드 파일 위치:

| 문서 섹션 | 관련 코드 파일 |
|---------|--------------|
| LiDAR Mapping | `lidar_mapping/lidar_mapper.py` |
| A* Planning | `path_planning/astar_planner.py` |
| Path Following | `path_planning/path_following_controller.py` |
| Map Processing | `path_planning/map_processor.py` |
| Controllers | `controllers/` 디렉토리 |
| Simulation | `simulation/simulation_manager.py` |

### 실습 순서
1. **기본 실행**: `python main.py`
2. **LiDAR 테스트**: `python test_lidar_interactive.py`
3. **맵 확인**: `python view_saved_map.py`
4. **경로 계획 테스트**: 코드 내 예제 실행
5. **파라미터 조정**: 설정 파일 수정

## 📝 문서 관리

### 업데이트 정책
- 코드 변경시 관련 문서 동시 업데이트
- API 변경은 CODE_DOCUMENTATION.md에 반영
- 새 기능은 ADVANCED_FEATURES.md에 추가
- 버그 해결은 QUICK_REFERENCE.md의 트러블슈팅에 추가

### 문서 작성 규칙
- 명확한 제목과 목차 사용
- 코드 예제 포함
- 다이어그램과 시각화 활용
- 파라미터 테이블 제공
- 트러블슈팅 가이드 포함

## 🔗 외부 리소스

### 참고 문헌
- Probabilistic Robotics (Thrun et al., 2005)
- Planning Algorithms (LaValle, 2006)
- Introduction to Autonomous Mobile Robots (Siegwart et al., 2011)

### 온라인 리소스
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [OpenCV Documentation](https://docs.opencv.org/)

### 관련 프로젝트
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Navigation2](https://navigation.ros.org/)
- [MoveIt](https://moveit.ros.org/)

## ❓ FAQ

**Q: 어떤 문서부터 읽어야 하나요?**
A: README.md로 시작하여 QUICK_REFERENCE.md로 빠르게 실행해보세요.

**Q: 알고리즘 세부사항은 어디에 있나요?**
A: LIDAR_MAPPING.md와 PATH_PLANNING.md에 상세히 설명되어 있습니다.

**Q: 코드 구조를 이해하려면?**
A: CODE_DOCUMENTATION.md와 CODE_READING_CHECKLIST.md를 참조하세요.

**Q: 새로운 기능을 추가하려면?**
A: ADVANCED_FEATURES.md의 확장 가이드를 참조하세요.

**Q: 문제가 발생했을 때?**
A: QUICK_REFERENCE.md의 트러블슈팅 섹션을 확인하세요.

## 📮 기여 가이드

문서 개선에 기여하려면:
1. 명확성과 정확성 확인
2. 코드 예제 테스트
3. 일관된 포맷 유지
4. 상호 참조 링크 확인
5. 목차 업데이트

---

*최종 업데이트: 2025년 1월*
*문서 버전: 1.0.0*
