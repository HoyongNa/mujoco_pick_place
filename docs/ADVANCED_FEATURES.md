# Advanced Features Documentation

## 목차
1. [통합 네비게이션 시스템](#통합-네비게이션-시스템)
2. [동적 맵 업데이트](#동적-맵-업데이트)  
3. [멀티스레드 아키텍처](#멀티스레드-아키텍처)
4. [작업 계획 시스템](#작업-계획-시스템)
5. [고급 경로 최적화](#고급-경로-최적화)
6. [시각화 및 디버깅 도구](#시각화-및-디버깅-도구)

## 통합 네비게이션 시스템

### 전체 파이프라인

```python
# 1. LiDAR 데이터 수집
lidar_data = sensor.get_lidar_scan()

# 2. 맵 업데이트 (SLAM)
mapper.update_map(robot_x, robot_y, robot_theta, lidar_data)

# 3. 경로 계획
processor = MapProcessor(mapper.get_occupancy_grid())
planner = AStarPlanner(processor)
path = planner.plan(current_pos, target_pos)

# 4. 경로 추종
controller = PathFollowingController(model, data)
controller.navigate_to(target_pos)
```

### NavigationSystem 클래스

통합된 네비게이션을 위한 상위 레벨 인터페이스:

```python
class NavigationSystem:
    def __init__(self, model, data, viewer=None):
        # 컴포넌트 초기화
        self.mapper = LidarMapper(map_size=(10, 10), resolution=0.05)
        self.path_controller = PathFollowingController(
            model, data, base_cmd_ref, base_lock, viewer
        )
        self.task_planner = TaskPlanner()
        
    def execute_mission(self, waypoints):
        """미션 실행 - 여러 웨이포인트 순차 방문"""
        for waypoint in waypoints:
            # 실시간 맵 업데이트
            self.update_map_from_sensors()
            
            # 경로 계획 및 실행
            success = self.path_controller.navigate_to(waypoint)
            
            if not success:
                # 실패시 재계획
                alternative = self.find_alternative_target(waypoint)
                self.path_controller.navigate_to(alternative)
            
            # 목표 도달 대기
            self.wait_for_arrival()
```

### 실시간 Replanning

동적 환경에서의 경로 재계획:

```python
class DynamicReplanner:
    def __init__(self, planner, mapper):
        self.planner = planner
        self.mapper = mapper
        self.replan_threshold = 0.7  # 경로 차단 임계값
        
    def check_path_validity(self, path):
        """경로상 장애물 검사"""
        for point in path:
            grid_x, grid_y = self.mapper.world_to_grid(point[0], point[1])
            if self.mapper.is_occupied(grid_x, grid_y):
                return False
        return True
    
    def adaptive_replan(self, current_pos, target):
        """적응적 재계획"""
        # 1. 빠른 로컬 재계획 시도
        local_path = self.local_replan(current_pos, target)
        if local_path:
            return local_path
        
        # 2. 전역 재계획
        return self.planner.plan(current_pos, target, use_dilated=True)
```

## 동적 맵 업데이트

### 실시간 SLAM

```python
class RealtimeSLAM:
    def __init__(self, mapper):
        self.mapper = mapper
        self.scan_buffer = []
        self.update_frequency = 5  # Hz
        
    def process_scan(self, scan_data, robot_pose):
        """스캔 데이터 처리 및 맵 업데이트"""
        # 스캔 필터링
        filtered_scan = self.filter_scan(scan_data)
        
        # 맵 업데이트
        self.mapper.update_map(
            robot_pose[0], robot_pose[1], robot_pose[2],
            filtered_scan
        )
        
        # 루프 클로저 검사
        if self.detect_loop_closure():
            self.correct_map()
    
    def filter_scan(self, scan_data):
        """노이즈 필터링 및 이상치 제거"""
        # 중간값 필터
        filtered = median_filter(scan_data, size=5)
        
        # 이상치 제거
        mean = np.mean(filtered)
        std = np.std(filtered)
        mask = np.abs(filtered - mean) < 3 * std
        
        return filtered[mask]
```

### 맵 병합 및 확장

```python
class MapMerger:
    def __init__(self):
        self.global_map = None
        self.local_maps = []
        
    def merge_maps(self, map1, map2, transform):
        """두 맵을 병합"""
        # 좌표 변환 적용
        transformed_map2 = self.apply_transform(map2, transform)
        
        # 확률적 병합
        merged = np.zeros_like(map1)
        for i in range(merged.shape[0]):
            for j in range(merged.shape[1]):
                # Bayesian 업데이트
                p1 = map1[i, j]
                p2 = transformed_map2[i, j]
                merged[i, j] = self.bayesian_update(p1, p2)
        
        return merged
    
    def extend_map_boundaries(self, current_map, robot_pose):
        """맵 경계 동적 확장"""
        x, y = robot_pose[:2]
        
        # 경계 근처 체크
        if self.near_boundary(x, y, current_map):
            # 새로운 큰 맵 생성
            extended = self.create_extended_map(current_map)
            
            # 기존 맵 복사
            self.copy_map_content(current_map, extended)
            
            return extended
        
        return current_map
```

## 멀티스레드 아키텍처

### ThreadManager 구현

```python
class ThreadManager:
    def __init__(self):
        self.threads = {}
        self.stop_events = {}
        self.locks = {}
        
    def start_thread(self, name, target, args=()):
        """스레드 시작"""
        if name in self.threads:
            self.stop_thread(name)
        
        stop_event = threading.Event()
        self.stop_events[name] = stop_event
        
        thread = threading.Thread(
            target=self._thread_wrapper,
            args=(target, stop_event, args),
            name=name
        )
        thread.daemon = True
        thread.start()
        
        self.threads[name] = thread
        
    def _thread_wrapper(self, target, stop_event, args):
        """스레드 래퍼 - 예외 처리 포함"""
        try:
            target(stop_event, *args)
        except Exception as e:
            print(f"Thread error: {e}")
            traceback.print_exc()
```

### 동시성 제어

```python
class ConcurrentController:
    def __init__(self):
        # 공유 자원
        self.base_cmd = np.zeros(3)  # [x, y, theta]
        self.arm_cmd = np.zeros(7)   # 7-DOF arm
        
        # 락
        self.base_lock = threading.RLock()
        self.arm_lock = threading.RLock()
        
        # 우선순위 큐
        self.command_queue = PriorityQueue()
        
    def set_base_command(self, cmd, priority=0):
        """베이스 명령 설정 (우선순위 고려)"""
        with self.base_lock:
            self.command_queue.put((priority, time.time(), cmd))
            
    def process_commands(self):
        """명령 처리 루프"""
        while True:
            if not self.command_queue.empty():
                priority, timestamp, cmd = self.command_queue.get()
                
                # 타임아웃 체크
                if time.time() - timestamp < 1.0:  # 1초 이내
                    self.apply_command(cmd)
```

## 작업 계획 시스템

### Pick and Place 작업

```python
class PickAndPlaceTask:
    def __init__(self, robot_controller):
        self.controller = robot_controller
        self.gripper = GripperController()
        
    def execute(self, object_pose, target_pose):
        """Pick and Place 실행"""
        # 1. 접근 위치 계산
        approach_pose = self.calculate_approach_pose(object_pose)
        
        # 2. 접근
        self.controller.move_to(approach_pose)
        
        # 3. 정밀 접근
        self.fine_approach(object_pose)
        
        # 4. 파지
        self.gripper.close()
        
        # 5. 들어올리기
        lift_pose = object_pose.copy()
        lift_pose[2] += 0.1  # 10cm 상승
        self.controller.move_to(lift_pose)
        
        # 6. 목표 위치로 이동
        self.controller.move_to(target_pose)
        
        # 7. 놓기
        self.gripper.open()
        
    def calculate_approach_pose(self, object_pose):
        """접근 자세 계산"""
        approach = object_pose.copy()
        approach[2] += 0.05  # 5cm 위
        return approach
```

### 작업 시퀀서

```python
class TaskSequencer:
    def __init__(self):
        self.task_queue = []
        self.completed_tasks = []
        
    def add_task(self, task_type, params):
        """작업 추가"""
        task = {
            'type': task_type,
            'params': params,
            'status': 'pending',
            'priority': self.calculate_priority(task_type)
        }
        self.task_queue.append(task)
        
    def execute_sequence(self):
        """작업 시퀀스 실행"""
        # 우선순위 정렬
        self.task_queue.sort(key=lambda x: x['priority'])
        
        for task in self.task_queue:
            try:
                result = self.execute_task(task)
                task['status'] = 'completed'
                self.completed_tasks.append(task)
            except Exception as e:
                task['status'] = 'failed'
                task['error'] = str(e)
                
                # 실패 처리
                self.handle_failure(task)
```

## 고급 경로 최적화

### 경로 스무딩

```python
class PathSmoother:
    def __init__(self):
        self.smoothing_iterations = 50
        self.weight_smooth = 0.5
        self.weight_data = 0.5
        
    def smooth_path(self, path):
        """Gradient descent 기반 경로 스무딩"""
        smoothed = np.array(path, dtype=float)
        
        for _ in range(self.smoothing_iterations):
            for i in range(1, len(smoothed) - 1):
                # 데이터 항
                data_term = self.weight_data * (path[i] - smoothed[i])
                
                # 스무딩 항
                smooth_term = self.weight_smooth * (
                    smoothed[i-1] + smoothed[i+1] - 2 * smoothed[i]
                )
                
                # 업데이트
                smoothed[i] += data_term + smooth_term
                
                # 충돌 체크
                if self.check_collision(smoothed[i]):
                    smoothed[i] = path[i]  # 원래 경로로 복원
        
        return smoothed.tolist()
```

### RRT* 통합

```python
class RRTStarPlanner:
    def __init__(self, map_processor):
        self.map_processor = map_processor
        self.max_iterations = 5000
        self.step_size = 0.1
        self.goal_sample_rate = 0.1
        
    def plan(self, start, goal):
        """RRT* 경로 계획"""
        tree = {start: {'parent': None, 'cost': 0}}
        
        for _ in range(self.max_iterations):
            # 랜덤 샘플링 또는 목표 샘플링
            if np.random.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self.random_sample()
            
            # 가장 가까운 노드 찾기
            nearest = self.find_nearest(tree, sample)
            
            # 새 노드 생성
            new_node = self.steer(nearest, sample, self.step_size)
            
            if not self.check_collision(nearest, new_node):
                # 근처 노드들 찾기
                near_nodes = self.find_near_nodes(tree, new_node)
                
                # 최소 비용 부모 선택
                min_cost_parent = self.choose_parent(near_nodes, nearest, new_node)
                
                # 트리에 추가
                tree[new_node] = {
                    'parent': min_cost_parent,
                    'cost': tree[min_cost_parent]['cost'] + 
                            self.distance(min_cost_parent, new_node)
                }
                
                # Rewire
                self.rewire(tree, new_node, near_nodes)
                
                # 목표 도달 체크
                if self.distance(new_node, goal) < 0.1:
                    return self.extract_path(tree, new_node)
        
        return None
```

## 시각화 및 디버깅 도구

### 실시간 시각화

```python
class RealtimeVisualizer:
    def __init__(self):
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 10))
        plt.ion()  # 인터랙티브 모드
        
        # 서브플롯 설정
        self.map_ax = self.axes[0, 0]
        self.path_ax = self.axes[0, 1]
        self.sensor_ax = self.axes[1, 0]
        self.status_ax = self.axes[1, 1]
        
    def update(self, data):
        """시각화 업데이트"""
        # 맵 업데이트
        self.map_ax.clear()
        self.map_ax.imshow(data['map'], cmap='gray')
        self.map_ax.set_title('Occupancy Grid Map')
        
        # 경로 표시
        if 'path' in data:
            path = np.array(data['path'])
            self.path_ax.plot(path[:, 0], path[:, 1], 'b-')
            self.path_ax.plot(data['robot_pos'][0], 
                            data['robot_pos'][1], 'ro')
        
        # 센서 데이터
        if 'lidar' in data:
            self.sensor_ax.clear()
            angles = np.linspace(0, 2*np.pi, len(data['lidar']))
            self.sensor_ax.polar(angles, data['lidar'])
            
        # 상태 정보
        self.status_ax.clear()
        self.status_ax.axis('off')
        status_text = f"""
        Position: ({data['robot_pos'][0]:.2f}, {data['robot_pos'][1]:.2f})
        Heading: {data['robot_pos'][2]:.2f} rad
        Velocity: {data.get('velocity', 0):.2f} m/s
        Task: {data.get('current_task', 'None')}
        """
        self.status_ax.text(0.1, 0.5, status_text, fontsize=10)
        
        plt.pause(0.001)
```

### 디버그 로거

```python
class DebugLogger:
    def __init__(self, log_dir="logs"):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        # 로거 설정
        self.logger = logging.getLogger('RobotDebug')
        self.logger.setLevel(logging.DEBUG)
        
        # 파일 핸들러
        fh = logging.FileHandler(
            f"{log_dir}/debug_{datetime.now():%Y%m%d_%H%M%S}.log"
        )
        
        # 포매터
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        fh.setFormatter(formatter)
        self.logger.addHandler(fh)
        
        # 데이터 기록용
        self.data_buffer = []
        
    def log_state(self, state_dict):
        """상태 로깅"""
        timestamp = time.time()
        self.data_buffer.append({
            'timestamp': timestamp,
            **state_dict
        })
        
        # 주기적 저장
        if len(self.data_buffer) >= 1000:
            self.save_data()
    
    def save_data(self):
        """데이터 저장"""
        filename = f"{self.log_dir}/data_{datetime.now():%Y%m%d_%H%M%S}.npz"
        np.savez(filename, data=self.data_buffer)
        self.data_buffer.clear()
```

### 성능 프로파일러

```python
class PerformanceProfiler:
    def __init__(self):
        self.timings = defaultdict(list)
        self.counters = defaultdict(int)
        
    @contextmanager
    def timer(self, name):
        """컨텍스트 매니저로 시간 측정"""
        start = time.perf_counter()
        yield
        end = time.perf_counter()
        self.timings[name].append(end - start)
        
    def profile_function(self, func):
        """함수 프로파일링 데코레이터"""
        @wraps(func)
        def wrapper(*args, **kwargs):
            with self.timer(func.__name__):
                result = func(*args, **kwargs)
            self.counters[func.__name__] += 1
            return result
        return wrapper
    
    def get_statistics(self):
        """통계 반환"""
        stats = {}
        for name, times in self.timings.items():
            stats[name] = {
                'count': len(times),
                'total': sum(times),
                'mean': np.mean(times),
                'std': np.std(times),
                'min': min(times),
                'max': max(times)
            }
        return stats
    
    def print_report(self):
        """프로파일링 리포트 출력"""
        stats = self.get_statistics()
        
        print("\n=== Performance Report ===")
        for name, stat in sorted(stats.items(), 
                                key=lambda x: x[1]['total'], 
                                reverse=True):
            print(f"\n{name}:")
            print(f"  Calls: {stat['count']}")
            print(f"  Total: {stat['total']:.3f}s")
            print(f"  Mean:  {stat['mean']*1000:.3f}ms")
            print(f"  Std:   {stat['std']*1000:.3f}ms")
```

## 고급 설정 및 튜닝

### 파라미터 자동 튜닝

```python
class ParameterTuner:
    def __init__(self, controller):
        self.controller = controller
        self.param_ranges = {
            'lookahead_distance': (0.3, 1.5),
            'max_linear_vel': (0.2, 1.0),
            'max_angular_vel': (0.5, 2.0)
        }
        
    def auto_tune(self, test_paths, metric='time'):
        """자동 파라미터 튜닝"""
        from scipy.optimize import differential_evolution
        
        def objective(params):
            # 파라미터 설정
            self.controller.set_params({
                'lookahead_distance': params[0],
                'max_linear_vel': params[1],
                'max_angular_vel': params[2]
            })
            
            # 테스트 실행
            total_cost = 0
            for path in test_paths:
                result = self.controller.execute_path(path)
                
                if metric == 'time':
                    total_cost += result['time']
                elif metric == 'energy':
                    total_cost += result['energy']
                elif metric == 'smoothness':
                    total_cost += result['jerk']
            
            return total_cost
        
        # 최적화 실행
        bounds = list(self.param_ranges.values())
        result = differential_evolution(objective, bounds)
        
        return dict(zip(self.param_ranges.keys(), result.x))
```

### 적응적 제어

```python
class AdaptiveController:
    def __init__(self):
        self.terrain_detector = TerrainDetector()
        self.param_sets = {
            'smooth': {'max_vel': 1.0, 'lookahead': 1.0},
            'rough': {'max_vel': 0.3, 'lookahead': 0.5},
            'narrow': {'max_vel': 0.2, 'lookahead': 0.3}
        }
        
    def adapt_to_environment(self, sensor_data):
        """환경에 따른 파라미터 적응"""
        # 지형 분류
        terrain_type = self.terrain_detector.classify(sensor_data)
        
        # 통로 폭 감지
        passage_width = self.detect_passage_width(sensor_data)
        
        # 파라미터 선택
        if passage_width < 0.8:
            params = self.param_sets['narrow']
        elif terrain_type == 'rough':
            params = self.param_sets['rough']
        else:
            params = self.param_sets['smooth']
        
        # 부드러운 전환
        self.smooth_transition(params)
```

## 최적화 팁

### 메모리 최적화
- Circular buffer 사용으로 히스토리 제한
- 맵 압축 (Octree, Quadtree)
- Lazy evaluation 활용

### 계산 최적화
- NumPy vectorization 최대 활용
- Numba JIT 컴파일 적용
- Cython으로 병목 구간 가속

### 실시간 성능
- 우선순위 기반 스케줄링
- Lock-free 데이터 구조 사용
- 비동기 I/O 활용

## 트러블슈팅 가이드

### 문제: 맵이 왜곡됨
- **원인**: 오도메트리 드리프트
- **해결**: 
  - IMU 센서 퓨전
  - Loop closure 검출
  - Particle filter 적용

### 문제: 경로가 진동함
- **원인**: 제어 게인 부적절
- **해결**:
  - PID 게인 튜닝
  - Low-pass 필터 적용
  - Adaptive lookahead

### 문제: 좁은 통로 실패
- **원인**: 과도한 맵 팽창
- **해결**:
  - Dynamic dilation
  - Local planner 사용
  - Precise mode 전환
