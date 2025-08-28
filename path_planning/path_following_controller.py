"""
경로 추종 컨트롤러 - 단순 직접 제어 버전
베이스가 확실히 움직이도록 단순화
"""

import threading
import time
import numpy as np
import mujoco
from typing import Tuple, Optional, List
from path_planning.map_processor import MapProcessor
from path_planning.astar_planner import AStarPlanner
from path_planning.pure_pursuit import PurePursuit
from controllers.arm.arm_holder import ArmHolder
from utils.thread_manager import ThreadManager


class PathFollowingController:
    """경로 계획 기반 자율 주행 컨트롤러"""
    
    def __init__(self, model, data, base_cmd_ref, base_lock, viewer=None):
        """
        Args:
            model: MuJoCo 모델
            data: MuJoCo 데이터
            base_cmd_ref: 베이스 명령 참조 (공유)
            base_lock: 베이스 락 (공유)
            viewer: 뷰어 (선택적)
        """
        self.model = model
        self.data = data
        self.viewer = viewer
        
        # 공유 자원
        self.base_cmd_ref = base_cmd_ref
        self.base_lock = base_lock
        
        # 경로 계획 구성요소
        self.map_processor = MapProcessor()
        self.planner = None  # 맵 로드 후 초기화
        self.pure_pursuit = PurePursuit(
            lookahead_distance=0.5,
            max_linear_vel=3, #0.3
            max_angular_vel=2 # 0.5
        )
        
        # 팔 홀더
        self.arm_holder = ArmHolder(model, data)
        
        # 스레드 관리
        self.thread_manager = ThreadManager()
        
        # 상태
        self.current_path = None
        self.target_position = None
        self.is_navigating = False
        self.navigation_complete = False
        self.waypoint_index = 0
        
        # 제어 파라미터
        self.control_frequency = 1000  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # 웨이포인트 도달 거리
        self.waypoint_threshold = 0.1   # 50cm
        self.final_threshold = 0.1      # 30cm
        
        # 부드러운 이동을 위한 스텝 크기
        self.step_size = 0.5  # 한 번에 10cm씩 이동 (더 부드럽게)
        
    def initialize(self, map_path: Optional[str] = None) -> bool:
        """초기화
        
        Args:
            map_path: 맵 파일 경로 (None이면 기본 맵 사용)
            
        Returns:
            성공 여부
        """
        # 맵 로드
        if not self.map_processor.load_map(map_path):
            print("[PathFollowingController] 맵 로드 실패")
            return False
            
        # 플래너 생성
        self.planner = AStarPlanner(self.map_processor)
        
        print("[PathFollowingController] 초기화 완료")
        return True
        
    def navigate_to(self, target: Tuple[float, float], visualize=False) -> bool:
        """목표 위치로 자율 주행
        
        Args:
            target: 목표 위치 (x, y) in meters
            visualize: 경로 시각화 여부
            
        Returns:
            경로 계획 성공 여부
        """
        print(f"[PathFollowingController] navigate_to 호출: target={target}")
        
        if self.planner is None:
            print("[PathFollowingController] 플래너가 초기화되지 않았습니다.")
            return False
            
        # 현재 위치
        with self.base_lock:
            current_x = self.data.qpos[0]
            current_y = self.data.qpos[1]
            
        start = (current_x, current_y)
        
        print(f"\n[PathFollowingController] 경로 계획 요청")
        print(f"  시작: ({current_x:.2f}, {current_y:.2f})")
        print(f"  목표: ({target[0]:.2f}, {target[1]:.2f})")
        
        # A* 경로 계획 (항상 팽창된 맵 사용)
        path = self.planner.plan(start, target, use_dilated=True)
        
        if path is None:
            print("[PathFollowingController] 경로를 찾을 수 없습니다.")
            return False
            
        # 경로 설정
        self.current_path = path
        self.target_position = target
        self.pure_pursuit.set_path(path)
        
        # 상태 초기화
        self.waypoint_index = 0
        
        # 시각화 (선택적)
        if visualize:
            self.map_processor.visualize(path, start, target)
        
        # 네비게이션 상태 설정
        self.is_navigating = True
        self.navigation_complete = False
        
        print(f"[PathFollowingController] 경로 생성 완료: {len(path)}개 웨이포인트")
        print(f"  - is_navigating 설정: {self.is_navigating}")
        print(f"  - 첫 웨이포인트: {path[0] if path else None}")
        print(f"  - 마지막 웨이포인트: {path[-1] if path else None}")
        return True
        
    def _control_loop(self):
        """제어 루프 (스레드에서 실행)"""
        # 팔 자세 홀드 설정
        self.arm_holder.set_current_as_target()
        
        print("[PathFollowingController] 제어 루프 시작")
        print(f"  - is_navigating: {self.is_navigating}")
        print(f"  - navigation_complete: {self.navigation_complete}")
        print(f"  - current_path: {len(self.current_path) if self.current_path else 0} waypoints")
        
        # 상태 추적
        last_log_time = time.time()
        
        while not self.thread_manager.should_stop():
            # 뷰어 체크
            if self.viewer and not self.viewer.is_running():
                break
                
            # 네비게이션 중인 경우에만 경로 추종
            if self.is_navigating and not self.navigation_complete:
                # 현재 로봇 포즈
                with self.base_lock:
                    x = self.data.qpos[0]
                    y = self.data.qpos[1]
                    theta = self.data.qpos[2]
                    
                # 최종 목표 도달 확인
                if self.target_position:
                    final_dx = self.target_position[0] - x
                    final_dy = self.target_position[1] - y
                    final_distance = np.sqrt(final_dx**2 + final_dy**2)
                    
                    if final_distance < self.final_threshold:
                        # 목표 도달
                        self.navigation_complete = True
                        self.is_navigating = False
                        print(f"\n[PathFollowingController] 목표 도달!")
                        print(f"  최종 위치: ({x:.2f}, {y:.2f})")
                        
                        # 현재 위치 유지
                        with self.base_lock:
                            self.base_cmd_ref[0] = x
                            self.base_cmd_ref[1] = y
                            self.base_cmd_ref[2] = theta
                        continue
                
                # 웨이포인트 추종
                if self.current_path and self.waypoint_index < len(self.current_path):
                    # 현재 목표 웨이포인트 (몇 개 앞을 볼 수 있음)
                    lookahead_index = min(self.waypoint_index + 1, len(self.current_path) - 1)
                    target_waypoint = self.current_path[lookahead_index]
                    
                    # 목표까지의 거리와 방향
                    dx = target_waypoint[0] - x
                    dy = target_waypoint[1] - y
                    distance = np.sqrt(dx**2 + dy**2)
                    
                    # 웨이포인트 도달 확인
                    if distance < self.waypoint_threshold:
                        # 다음 웨이포인트로
                        if self.waypoint_index < len(self.current_path) - 1:
                            self.waypoint_index += 1
                            # 새 웨이포인트로 재계산
                            target_waypoint = self.current_path[self.waypoint_index]
                            dx = target_waypoint[0] - x
                            dy = target_waypoint[1] - y
                            distance = np.sqrt(dx**2 + dy**2)
                    
                    # 이동할 거리 계산
                    if distance > 0.01:
                        # 단계적 이동 (부드럽게)
                        move_step = min(self.step_size, distance)
                        
                        # 다음 위치 계산
                        next_x = x + (dx / distance) * move_step
                        next_y = y + (dy / distance) * move_step
                        
                        # 목표 방향 계산
                        target_theta = np.arctan2(dy, dx)
                        
                        # 각도 차이 (최소 경로)
                        theta_diff = target_theta - theta
                        while theta_diff > np.pi:
                            theta_diff -= 2 * np.pi
                        while theta_diff < -np.pi:
                            theta_diff += 2 * np.pi
                        
                        # 부드러운 회전
                        # 회전 속도를 각도 차이에 따라 조정
                        if abs(theta_diff) > np.pi/4:  # 45도 이상이면 천천히 회전
                            max_rotation = 0.05  # 더 작은 회전 스텝
                            move_step = min(move_step * 0.5, 0.02)  # 감속
                        else:
                            max_rotation = 0.08  # 일반 회전 스텝
                        
                        theta_step = np.clip(theta_diff, -max_rotation, max_rotation)
                        next_theta = theta + theta_step
                        
                        # 명령 설정
                        with self.base_lock:
                            self.base_cmd_ref[0] = next_x
                            self.base_cmd_ref[1] = next_y
                            self.base_cmd_ref[2] = next_theta
                            
                        # 이동 중임을 한 번만 로그 (디버깅)
                        if not hasattr(self, '_moving_logged'):
                            print(f"[PathFollowingController] 이동 중: 목표=({next_x:.3f}, {next_y:.3f}), 거리={distance:.3f}m")
                            self._moving_logged = True
                    else:
                        # 현재 위치 유지
                        with self.base_lock:
                            self.base_cmd_ref[0] = x
                            self.base_cmd_ref[1] = y
                            self.base_cmd_ref[2] = theta
                    
                    # 진행 상황 로그 (1초마다)
                    current_time = time.time()
                    if current_time - last_log_time > 1.0:
                        progress = (self.waypoint_index / len(self.current_path)) * 100
                        print(f"  진행: {progress:.0f}% | 위치: ({x:.2f}, {y:.2f}) | "
                              f"웨이포인트: {self.waypoint_index+1}/{len(self.current_path)} | "
                              f"거리: {distance:.2f}m")
                        last_log_time = current_time
                else:
                    # 경로가 없으면 현재 위치 유지
                    with self.base_lock:
                        self.base_cmd_ref[0] = x
                        self.base_cmd_ref[1] = y
                        self.base_cmd_ref[2] = theta
            else:
                # 네비게이션 중이 아니면 현재 위치 유지
                with self.base_lock:
                    x = self.data.qpos[0]
                    y = self.data.qpos[1]
                    theta = self.data.qpos[2]
                    self.base_cmd_ref[0] = x
                    self.base_cmd_ref[1] = y
                    self.base_cmd_ref[2] = theta
            
            # 베이스 제어 적용 (위치 제어)
            with self.base_lock:
                cmd = self.base_cmd_ref.copy()
            
            # MuJoCo position actuator에 직접 명령
            self.data.ctrl[0] = cmd[0]  # x 위치
            self.data.ctrl[1] = cmd[1]  # y 위치
            self.data.ctrl[2] = cmd[2]  # theta 각도
            
            # 처음에만 제어 명령 확인 (디버깅)
            if not hasattr(self, '_first_cmd_logged'):
                print(f"[PathFollowingController] 첫 베이스 명령: x={cmd[0]:.3f}, y={cmd[1]:.3f}, theta={cmd[2]:.3f}")
                self._first_cmd_logged = True
            
            # 팔 홀드
            torque = self.arm_holder.compute_hold_torque()
            self.data.ctrl[3:10] = torque
            
            # 물리 시뮬레이션 스텝
            mujoco.mj_step(self.model, self.data)
            
            # 뷰어 동기화
            if self.viewer:
                self.viewer.sync()
            
            # 주파수 유지
            time.sleep(self.dt)
            
        print("[PathFollowingController] 제어 루프 종료")
        
    def start(self):
        """컨트롤러 시작"""
        if self.planner is None:
            print("[PathFollowingController] 먼저 initialize()를 호출하세요.")
            return
        
        # 현재 위치로 베이스 명령 초기화
        with self.base_lock:
            self.base_cmd_ref[0] = self.data.qpos[0]
            self.base_cmd_ref[1] = self.data.qpos[1]
            self.base_cmd_ref[2] = self.data.qpos[2]
            print(f"[PathFollowingController] 시작 위치: ({self.base_cmd_ref[0]:.2f}, {self.base_cmd_ref[1]:.2f}, {self.base_cmd_ref[2]:.2f})")
            
        self.thread_manager.start(self._control_loop)
        print("[PathFollowingController] 스레드 시작됨")
        
    def stop(self, timeout=1.0):
        """컨트롤러 정지"""
        # 현재 위치 유지 (마지막 명령값 저장)
        with self.base_lock:
            final_pos = self.data.qpos[:3].copy()
            self.base_cmd_ref[:] = final_pos
            # ctrl에도 적용하여 즉시 정지
            self.data.ctrl[0] = final_pos[0]
            self.data.ctrl[1] = final_pos[1]
            self.data.ctrl[2] = final_pos[2]
            
        print(f"[PathFollowingController] 정지 (위치: {final_pos[0]:.2f}, {final_pos[1]:.2f}, 각도: {final_pos[2]:.2f})")
        self.thread_manager.stop(timeout)
        
    def is_navigation_complete(self) -> bool:
        """네비게이션 완료 여부 반환"""
        return self.navigation_complete
        
    def get_current_position(self) -> Tuple[float, float, float]:
        """현재 로봇 위치 반환"""
        with self.base_lock:
            return (self.data.qpos[0], self.data.qpos[1], self.data.qpos[2])
