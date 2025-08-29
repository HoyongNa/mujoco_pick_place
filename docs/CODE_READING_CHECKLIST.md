# Code Reading Checklist

## 프로젝트 구조 파악 체크리스트

### 1. 전체 구조 이해
- [ ] 프로젝트 루트의 README.md 읽기
- [ ] requirements.txt로 의존성 확인
- [ ] main.py 진입점 분석
- [ ] 각 모듈의 역할 파악

### 2. 모듈별 핵심 파일

#### LiDAR Mapping 모듈
- [ ] `lidar_mapping/lidar_mapper.py` - 핵심 매핑 클래스
  - [ ] `__init__()` - 초기화 파라미터 이해
  - [ ] `update_map()` - 맵 업데이트 알고리즘
  - [ ] `bresenham_line()` - 레이캐스팅 구현
  - [ ] `save_map()`, `load_map()` - 맵 저장/로드
  
- [ ] `lidar_mapping/occupancy_grid.py` - 그리드 맵 관리
  - [ ] log-odds 업데이트 방식
  - [ ] 확률 변환 메커니즘

#### Path Planning 모듈
- [ ] `path_planning/map_processor.py` - 맵 전처리
  - [ ] `dilate_map()` - 장애물 팽창
  - [ ] `world_to_grid()`, `grid_to_world()` - 좌표 변환
  
- [ ] `path_planning/astar_planner.py` - A* 알고리즘
  - [ ] `plan()` - 메인 경로 탐색
  - [ ] `heuristic()` - 휴리스틱 함수
  - [ ] `reconstruct_path()` - 경로 재구성
  
- [ ] `path_planning/path_following_controller.py` - 경로 추종 통합 제어
  - [ ] 웨이포인트 기반 추종 알고리즘
  - [ ] `navigate_to()` - 자율 네비게이션
  - [ ] `_control_loop()` - 제어 루프
  - [ ] MuJoCo position actuator 직접 제어

#### Simulation 모듈
- [ ] `simulation/simulation_manager.py` - 시뮬레이션 관리
  - [ ] MuJoCo 초기화 및 설정
  - [ ] 센서 데이터 획득
  - [ ] 제어 명령 적용

#### Controllers 모듈
- [ ] `controllers/base_controller.py` - 컨트롤러 인터페이스
- [ ] `controllers/navigation_controller.py` - 네비게이션 통합

### 3. 데이터 흐름 추적

#### LiDAR 데이터 흐름
```
센서 → SimulationManager → LidarMapper → OccupancyGrid
                                ↓
                         MapProcessor → AStarPlanner
```

#### 제어 명령 흐름
```
AStarPlanner → Path → PathFollowingController → Control Command
                              ↓
                      MuJoCo Position Actuator
```

### 4. 핵심 알고리즘 이해

#### Bresenham Line Algorithm
- [ ] 정수 연산만으로 선 그리기
- [ ] 레이캐스팅에 활용
- [ ] 시간 복잡도: O(n)

#### Log-Odds Mapping
- [ ] Bayes 규칙 기반 업데이트
- [ ] 수치 안정성 (log 공간 연산)
- [ ] 업데이트 공식:
  ```python
  log_odds = log_odds + log_odds_hit  # hit
  log_odds = log_odds + log_odds_miss  # miss
  ```

#### A* Search
- [ ] f(n) = g(n) + h(n) 이해
- [ ] 우선순위 큐 사용
- [ ] 8방향 탐색
- [ ] 시간 복잡도: O(b^d) worst case

#### Waypoint Following
- [ ] 단계적 이동 (step_size)
- [ ] 목표 방향 계산 (atan2)
- [ ] 직접 위치 제어 (MuJoCo actuator)

### 5. 주요 설계 패턴

#### Singleton Pattern
- [ ] SimulationManager에서 사용
- [ ] 단일 시뮬레이션 인스턴스 보장

#### Strategy Pattern
- [ ] 다양한 컨트롤러 구현
- [ ] BaseController 인터페이스

#### Observer Pattern
- [ ] 이벤트 기반 업데이트
- [ ] 센서 데이터 콜백

### 6. 성능 관련 코드

#### NumPy 최적화
- [ ] 벡터화 연산 활용
- [ ] Broadcasting 이해
- [ ] In-place 연산 사용

#### 메모리 관리
- [ ] 큰 배열의 효율적 관리
- [ ] 순환 참조 방지
- [ ] 가비지 컬렉션 고려

### 7. 에러 처리 확인

- [ ] Try-except 블록 위치
- [ ] 예외 타입별 처리
- [ ] 로깅 메커니즘
- [ ] Graceful degradation

### 8. 테스트 코드 분석

- [ ] 단위 테스트 커버리지
- [ ] 통합 테스트 시나리오
- [ ] Edge case 처리
- [ ] Mock 객체 사용

### 9. 설정 및 파라미터

#### 설정 파일 확인
- [ ] `config/` 디렉토리 파일들
- [ ] 하드코딩된 값 찾기
- [ ] 환경 변수 사용

#### 주요 파라미터 목록
- [ ] LiDAR 범위 및 해상도
- [ ] 맵 크기 및 그리드 해상도
- [ ] 경로 계획 파라미터
- [ ] 제어 게인 값

### 10. 코드 품질 체크

#### 가독성
- [ ] 변수명이 명확한가?
- [ ] 함수가 단일 책임을 가지는가?
- [ ] 주석이 적절한가?

#### 유지보수성
- [ ] 모듈 간 결합도가 낮은가?
- [ ] 의존성이 명확한가?
- [ ] 확장 가능한 구조인가?

#### 문서화
- [ ] Docstring이 완전한가?
- [ ] 타입 힌트가 있는가?
- [ ] 예제 코드가 있는가?

### 11. 실행 흐름 추적

#### Main 실행 흐름
1. [ ] 초기화 단계
   - [ ] 설정 로드
   - [ ] 시뮬레이션 생성
   - [ ] 모듈 초기화

2. [ ] 메인 루프
   - [ ] 센서 데이터 수집
   - [ ] 맵 업데이트
   - [ ] 경로 계획
   - [ ] 제어 실행

3. [ ] 종료 처리
   - [ ] 리소스 정리
   - [ ] 데이터 저장
   - [ ] 로그 마무리

### 12. 디버깅 포인트

#### 브레이크포인트 설정 위치
- [ ] `update_map()` 진입점
- [ ] `plan()` 경로 생성 후
- [ ] `_control_loop()` 제어 계산 후
- [ ] 예외 발생 지점

#### 로그 확인 사항
- [ ] 타임스탬프
- [ ] 로그 레벨
- [ ] 모듈별 로그
- [ ] 성능 메트릭

### 13. 리팩토링 기회

#### 코드 중복
- [ ] 반복되는 패턴 찾기
- [ ] 공통 유틸리티 추출
- [ ] 상속 vs 컴포지션

#### 성능 개선
- [ ] 병목 지점 식별
- [ ] 알고리즘 최적화
- [ ] 캐싱 기회

#### 구조 개선
- [ ] 순환 의존성 제거
- [ ] 인터페이스 정리
- [ ] 네이밍 일관성

### 14. 보안 및 안전성

#### 입력 검증
- [ ] 센서 데이터 범위 체크
- [ ] 파라미터 유효성 검증
- [ ] Null/None 체크

#### 안전 제약
- [ ] 최대 속도 제한
- [ ] 충돌 방지 로직
- [ ] Fail-safe 메커니즘

### 15. 확장성 고려사항

#### 새 기능 추가 포인트
- [ ] 새로운 센서 타입
- [ ] 다른 경로 계획 알고리즘
- [ ] 추가 제어 전략

#### 스케일링 고려사항
- [ ] 더 큰 맵 지원
- [ ] 다중 로봇 지원
- [ ] 실시간 성능 유지

## 체크리스트 사용법

1. **첫 번째 읽기**: 전체 구조 파악
   - 상위 레벨 이해에 집중
   - 모듈 간 관계 파악

2. **두 번째 읽기**: 핵심 알고리즘 이해
   - 주요 함수 동작 방식
   - 데이터 흐름 추적

3. **세 번째 읽기**: 세부사항 분석
   - 엣지 케이스 처리
   - 성능 최적화 부분
   - 에러 처리 로직

4. **네 번째 읽기**: 개선점 도출
   - 리팩토링 기회
   - 버그 가능성
   - 문서화 필요 부분

## 코드 리뷰 템플릿

```markdown
## 모듈: [모듈명]
### 목적
- 

### 주요 클래스/함수
- 

### 강점
- 

### 개선 필요사항
- 

### 질문사항
- 
```

## 학습 자료

### 알고리즘 참고
- Probabilistic Robotics (Thrun et al.)
- Planning Algorithms (LaValle)
- Introduction to Autonomous Mobile Robots (Siegwart et al.)

### Python 최적화
- High Performance Python (Gorelick & Ozsvald)
- Effective Python (Slatkin)

### MuJoCo 문서
- [공식 문서](https://mujoco.readthedocs.io/)
- [튜토리얼](https://github.com/deepmind/mujoco)
