"""시뮬레이션 매니저 - 리팩토링된 버전"""

import threading
import numpy as np
import mujoco
from config.robot_config import RobotConfig
from controllers.base.mobility_controller import MobilityController
from kinematics.ik_solver import InverseKinematicsSolver
from simulation.viewer_manager import ViewerManager
from tasks.waypoint_generator import WaypointGenerator

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
        """Mobility 컨트롤 시작 - 현재 베이스 위치 유지"""
        if self.mobility_controller:
            self.mobility_controller.stop()
        
        # 현재 베이스 위치를 명령값으로 설정 (원점 복귀 방지)
        with self.base_lock:
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