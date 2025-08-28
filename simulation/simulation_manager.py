"""시뮬레이션 매니저 - 리팩토링된 버전"""

import threading
import numpy as np
import mujoco
from config.robot_config import RobotConfig
from controllers.base.mobility_controller import MobilityController
from kinematics.ik_solver import InverseKinematicsSolver
from simulation.viewer_manager import ViewerManager
from tasks.waypoint_generator import WaypointGenerator
from path_planning import PathFollowingController

class SimulationManager:
    """Pick & Place 시뮬레이션 통합 관리"""
    
    def __init__(self, model_path):
        # 모델/데이터
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # 설정
        self.config = RobotConfig(self.model)
        
        # 매니저
        self.viewer_manager = ViewerManager(self.model, self.data)
        self.waypoint_gen = WaypointGenerator(self.data)
        
        # 공유 상태
        self.base_lock = threading.RLock()
        mujoco.mj_forward(self.model, self.data)  # 초기 상태 동기화
        self.base_cmd_ref = np.copy(self.data.qpos[:3])
        self.shared_gripper_ctrl = [0.0]
        
        # 컨트롤러
        self.mobility_controller = None
        self.path_controller = None  # 경로 추종 컨트롤러
        self.arm_controller = None
        self.grasp_checker = None
        
        # IK 솔버
        bounds = self.config.get_arm_joint_bounds()
        self.ik_solver = InverseKinematicsSolver(
            self.model, self.data, 
            list(range(3, 10)), bounds, 
            self.config.ee_site_id
        )
        
        # 홈 자세
        self.arm_home_q = np.copy(self.data.qpos[3:10])
        
    def initialize_viewer(self):
        """뷰어 초기화"""
        self.viewer_manager.initialize()
        self.start_mobility_control()
        
    def start_mobility_control(self):
        """
Mobility 컨트롤 시작 - 현재 베이스 위치 유지
        """
        if self.mobility_controller:
            self.mobility_controller.stop()
        
        # 현재 베이스 위치를 명령값으로 설정 (원점 복귀 방지)
        with self.base_lock:
            # base_cmd_ref가 이미 설정되어 있지 않은 경우만 현재 위치로 설정
            # (외부에서 설정한 경우 그 값 사용)
            if np.allclose(self.base_cmd_ref, 0.0, atol=1e-3):
                self.base_cmd_ref[:] = self.data.qpos[:3]
            
        self.mobility_controller = MobilityController(
            self.model, self.data,
            self.base_cmd_ref, self.base_lock,
            self.viewer_manager.viewer
        )
        self.mobility_controller.start()
        
    def stop_mobility_control(self, zero_on_stop=False, maintain_position=True):
        """Mobility 컨트롤 정지
        
        Args:
            zero_on_stop: True면 베이스 명령을 0으로 설정
            maintain_position: True면 현재 위치를 유지
        """
        if maintain_position and not zero_on_stop:
            # 현재 베이스 위치를 저장
            with self.base_lock:
                self.base_cmd_ref[:] = self.data.qpos[:3]
                
        if self.mobility_controller:
            self.mobility_controller.stop(zero_on_stop=zero_on_stop)
            self.mobility_controller = None
            
    def initialize_path_controller(self, map_path=None):
        """경로 추종 컨트롤러 초기화
        
        Args:
            map_path: 맵 파일 경로 (None이면 기본 맵 사용)
            
        Returns:
            성공 여부
        """
        # 기존 컨트롤러가 있으면 정지
        if self.path_controller:
            self.stop_path_control()
            
        # 새로운 컨트롤러 생성
        self.path_controller = PathFollowingController(
            self.model, self.data,
            self.base_cmd_ref, self.base_lock,
            self.viewer_manager.viewer
        )
        
        # 맵 초기화
        success = self.path_controller.initialize(map_path)
        
        if success:
            print("[SimulationManager] 경로 추종 컨트롤러 초기화 완료")
        else:
            print("[SimulationManager] 경로 추종 컨트롤러 초기화 실패")
            self.path_controller = None
            
        return success
        
    def start_path_control(self):
        """경로 추종 컨트롤 시작"""
        if not self.path_controller:
            print("[SimulationManager] 경로 추종 컨트롤러가 초기화되지 않았습니다.")
            return False
            
        # 기존 모빌리티 컨트롤러 정지
        self.stop_mobility_control(maintain_position=True)
        
        # 경로 추종 컨트롤러 시작 (스레드 시작)
        self.path_controller.start()
        print("[SimulationManager] 경로 추종 컨트롤러 시작")
        return True
        
    def stop_path_control(self):
        """경로 추종 컨트롤 정지"""
        if self.path_controller:
            # 정지 전 현재 위치 저장
            with self.base_lock:
                current_pos = self.data.qpos[:3].copy()
                self.base_cmd_ref[:] = current_pos
            
            self.path_controller.stop()
            print("[SimulationManager] 경로 추종 컨트롤러 정지")
            # 컨트롤러 인스턴스는 유지 (맵 정보 보존)
            # self.path_controller = None
            
    def navigate_to(self, target, visualize=False):
        """목표 위치로 자율 주행
        
        Args:
            target: 목표 위치 (x, y) in meters
            visualize: 경로 시각화 여부
            
        Returns:
            경로 계획 성공 여부
        """
        if not self.path_controller:
            print("[SimulationManager] 경로 추종 컨트롤러가 초기화되지 않았습니다.")
            return False
            
        return self.path_controller.navigate_to(target, visualize)
        
    def is_navigation_complete(self):
        """네비게이션 완료 여부 확인"""
        if not self.path_controller:
            return True
            
        return self.path_controller.is_navigation_complete()
        
    def wait_for_navigation(self, timeout=None):
        """네비게이션 완료 대기
        
        Args:
            timeout: 최대 대기 시간 (초). None이면 무한 대기
            
        Returns:
            완료 여부 (True: 완료, False: 타임아웃 또는 취소)
        """
        import time
        
        if not self.path_controller:
            return True
            
        start_time = time.time()
        
        while not self.path_controller.is_navigation_complete():
            if not self.viewer_manager.is_running():
                return False
                
            if timeout and (time.time() - start_time) > timeout:
                print("[SimulationManager] 네비게이션 타임아웃")
                return False
                
            time.sleep(0.1)
            
        return True