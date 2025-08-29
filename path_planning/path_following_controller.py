"""
경로 추종 컨트롤러 - 목표 Heading을 경로 추종 중 미리 고려하는 버전
목표점 근처에서부터 목표 heading으로 정렬하면서 접근
"""

import threading
import time
import numpy as np
import mujoco
from typing import Tuple, Optional, List
from path_planning.map_processor import MapProcessor
from path_planning.astar_planner import AStarPlanner
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
        self.final_cmd = None  # 완료 시점의 명령값 저장
        
        # 제어 파라미터
        self.control_frequency = 1000  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # 웨이포인트 도달 거리
        self.waypoint_threshold = 0.1   # 10cm
        self.final_threshold = 0.1      # 10cm
        
        # 부드러운 이동을 위한 스텝 크기
        self.step_size = 0.5  # 한 번에 50cm씩 이동
        
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
        
        # 상태 초기화
        self.waypoint_index = 0
        self.final_cmd = None  # 이전 명령값 초기화
        
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
        last_debug_time = time.time()
        
        while not self.thread_manager.should_stop():
            # 뷰어 체크
            if self.viewer and not self.viewer.is_running():
                break
                
            # 네비게이션 완료 후 처리
            if self.navigation_complete and self.final_cmd is not None:
                # 완료된 경우 저장된 최종 명령값을 계속 적용
                with self.base_lock:
                    self.base_cmd_ref[0] = self.final_cmd[0]
                    self.base_cmd_ref[1] = self.final_cmd[1]
                    self.base_cmd_ref[2] = self.final_cmd[2]
                    
                # 주기적으로 상태 확인 (2초마다)
                current_time = time.time()
                if current_time - last_debug_time > 2.0:
                    print(f"[DEBUG] 완료 후 상태:")
                    print(f"  목표 cmd: theta={np.degrees(self.final_cmd[2]):.1f}°")
                    print(f"  현재 qpos: theta={np.degrees(self.data.qpos[2]):.1f}°")
                    print(f"  ctrl[2]: {np.degrees(self.data.ctrl[2]):.1f}°")
                    last_debug_time = current_time
                    
            # 네비게이션 중인 경우에만 경로 추종
            elif self.is_navigating and not self.navigation_complete:
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
                    
                    # 위치 도달 확인
                    if final_distance < self.final_threshold:
                        # 위치만으로 완료 판단
                        self.navigation_complete = True
                        self.is_navigating = False
                        print(f"\n[PathFollowingController] 목표 도달!")
                        print(f"  최종 위치: ({x:.2f}, {y:.2f})")
                        
                        with self.base_lock:
                            # 현재 위치와 방향 유지
                            self.base_cmd_ref[0] = self.target_position[0] if self.target_position else x
                            self.base_cmd_ref[1] = self.target_position[1] if self.target_position else y
                            # 현재 방향 유지
                            self.base_cmd_ref[2] = theta
                            self.final_cmd = self.base_cmd_ref.copy()
                        continue
                        

                
                # 웨이포인트 추종
                if self.current_path and self.waypoint_index < len(self.current_path):
                    # 현재 목표 웨이포인트
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
                        
                        # 강한 회전 제어
                        # 회전 속도를 각도 차이에 따라 조정 (더 크게 설정)
                        if abs(theta_diff) > np.pi/4:  # 45도 이상이면
                            max_rotation = 0.2  # 매우 빠른 회전 (2배 증가)
                            move_step = min(move_step * 0.3, 0.05)  # 더 많이 감속
                        elif abs(theta_diff) > np.pi/6:  # 30도 이상이면
                            max_rotation = 0.15  # 빠른 회전
                            move_step = min(move_step * 0.5, 0.1)  # 감속
                        else:
                            max_rotation = 0.2  # 일반 회전 속도도 높게
                        
                        theta_step = np.clip(theta_diff, -max_rotation, max_rotation)
                        # 더 강한 heading 제어를 위해 스텝 크기 증가
                        next_theta = theta + 2 * theta_step  # 5배 더 빠른 회전
                        
                        # 명령 설정
                        with self.base_lock:
                            self.base_cmd_ref[0] = next_x
                            self.base_cmd_ref[1] = next_y
                            self.base_cmd_ref[2] = next_theta
                            
                        # 이동 중임을 한 번만 로그 (디버깅)
                        if not hasattr(self, '_moving_logged'):
                            print(f"[PathFollowingController] 이동 시작")
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
                        
                        # 목표 정보
                        if self.target_position:
                            final_dx = self.target_position[0] - x
                            final_dy = self.target_position[1] - y
                            final_distance = np.sqrt(final_dx**2 + final_dy**2)
                            
                            print(f"  진행: {progress:.0f}% | 위치: ({x:.2f}, {y:.2f}) | "
                                  f"heading: {np.degrees(theta):.1f}° | "
                                  f"목표까지: {final_distance:.2f}m")
                            

                        else:
                            print(f"  진행: {progress:.0f}% | 위치: ({x:.2f}, {y:.2f}) | "
                                  f"웨이포인트: {self.waypoint_index+1}/{len(self.current_path)} | "
                                  f"거리: {distance:.2f}m")
                        
                        last_log_time = current_time
                else:
                    # 경로가 없으면 마지막 명령값 유지
                    # (현재 theta로 되돌아가지 않도록 명령값을 변경하지 않음)
                    pass  # base_cmd_ref를 변경하지 않고 유지
            
            # 베이스 제어 적용 (위치 제어)
            with self.base_lock:
                cmd = self.base_cmd_ref.copy()
            
            # MuJoCo position actuator에 직접 명령
            self.data.ctrl[0] = cmd[0]  # x 위치
            self.data.ctrl[1] = cmd[1]  # y 위치
            self.data.ctrl[2] = cmd[2]  # theta 각도
            
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
            
        # 플래그 초기화
        if hasattr(self, '_moving_logged'):
            delattr(self, '_moving_logged')
        if hasattr(self, '_complete_logged'):
            delattr(self, '_complete_logged')
            
        self.thread_manager.start(self._control_loop)
        print("[PathFollowingController] 스레드 시작됨")
        
    def stop(self, timeout=1.0):
        """컨트롤러 정지"""
        # 현재 위치 유지 (마지막 명령값 저장)
        with self.base_lock:
            # navigation_complete인 경우 final_cmd 유지, 아니면 현재 위치
            if self.navigation_complete and self.final_cmd is not None:
                self.base_cmd_ref[:] = self.final_cmd
                self.data.ctrl[0] = self.final_cmd[0]
                self.data.ctrl[1] = self.final_cmd[1]
                self.data.ctrl[2] = self.final_cmd[2]
                print(f"[PathFollowingController] 정지 (최종 명령 유지: theta={np.degrees(self.final_cmd[2]):.1f}°)")
            else:
                final_pos = self.data.qpos[:3].copy()
                self.base_cmd_ref[:] = final_pos
                self.data.ctrl[0] = final_pos[0]
                self.data.ctrl[1] = final_pos[1]
                self.data.ctrl[2] = final_pos[2]
                print(f"[PathFollowingController] 정지 (현재 위치: {final_pos[0]:.2f}, {final_pos[1]:.2f}, 각도: {np.degrees(final_pos[2]):.1f}°)")
            
        self.thread_manager.stop(timeout)
        
    def is_navigation_complete(self) -> bool:
        """네비게이션 완료 여부 반환"""
        return self.navigation_complete
        
    def get_current_position(self) -> Tuple[float, float, float]:
        """현재 로봇 위치 반환 (x, y, theta)"""
        with self.base_lock:
            return (self.data.qpos[0], self.data.qpos[1], self.data.qpos[2])
    