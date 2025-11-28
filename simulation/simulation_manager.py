"""시뮬레이션 매니저 - 리팩토링된 버전 (스레드 제거, 메인 루프 통합)"""

import numpy as np
import mujoco
from config.robot_config import RobotConfig
from controllers.base.velocity_mobility_controller import VelocityMobilityController
from controllers.arm.arm_controller import ArmController  # ✅ ArmController import
from kinematics.async_ik_solver import AsyncIKSolver  # ✅ Async IK solver for non-blocking computation
from path_planning.async_mpc_planner import AsyncMPCPlanner  # ✅ Async MPC planner for non-blocking path planning
from path_planning.async_mpc_controller import AsyncMPCController
from simulation.viewer_manager import ViewerManager


# Import RoboCasa configuration (always available)
try:
    from config.constants import (USE_ROBOCASA, ROBOCASA_LAYOUT, ROBOCASA_STYLE,
                                  ROBOT1_START_POS, ROBOT2_START_POS)
except ImportError:
    # Fallback if constants not updated yet
    USE_ROBOCASA = False
    ROBOCASA_LAYOUT = "G-shaped"
    ROBOCASA_STYLE = "modern"
    ROBOT1_START_POS = (0.0, 0.0, 0.0)
    ROBOT2_START_POS = (2.0, 0.0, 0.0)

# RoboCasa integration (only if module available)
try:
    from robocasa_integration import create_robocasa_kitchen
    ROBOCASA_AVAILABLE = True
except ImportError as e:
    ROBOCASA_AVAILABLE = False
    print(f"[WARNING] RoboCasa not available: {e}")
    print("[INFO] Using default scene instead.")

class SimulationManager:
    """Pick & Place 시뮬레이션 통합 관리"""
    
    def __init__(self, model_path):
        """
        Args:
            model_path: 모델 파일 경로
            controller_type: 'APF' (A* + Potential Field) 또는 'MPC' (A* + MPC)
        """
        self.model_xml_path = model_path
        # 모델/데이터
        # RoboCasa 환경 사용 여부 확인
        if ROBOCASA_AVAILABLE and USE_ROBOCASA:
            print(f"[SimulationManager] Using RoboCasa kitchen: {ROBOCASA_LAYOUT} - {ROBOCASA_STYLE}")
            print("[SimulationManager] ⏳ Generating kitchen fixtures... (10-30 seconds)")
            print("[SimulationManager]    This includes cabinets, appliances, counters, etc.")
            self.model = create_robocasa_kitchen(
                layout=ROBOCASA_LAYOUT,
                style=ROBOCASA_STYLE,
                robot1_pos=ROBOT1_START_POS,
                robot2_pos=ROBOT2_START_POS
            )
            print("[SimulationManager] ✓ RoboCasa environment loaded successfully")
            self.model_xml_path = None  # RoboCasa case
        else:
            if USE_ROBOCASA and not ROBOCASA_AVAILABLE:
                print("[WARNING] RoboCasa requested but not available. Using default scene.")
            print(f"[SimulationManager] Using default scene: {model_path}")
            self.model = mujoco.MjModel.from_xml_path(model_path)
        
        self.data = mujoco.MjData(self.model)
        
        # 설정
        self.config = RobotConfig(self.model)
        
        # 매니저
        self.viewer_manager = ViewerManager(self.model, self.data)
        
        # 공유 상태 (mj_forward 이미 호출됨)
        self.shared_gripper_ctrl = [0.0]  # Robot1 gripper control
        self.shared_gripper_ctrl_robot2 = np.array([0.0])  #  Robot2 gripper control
        
        # 컨트롤러
        self.mobility_controller = None  # Robot1 모빌리티 컨트롤러
        self.mobility_controller_robot2 = None  # Robot2 모빌리티 컨트롤러
        self.path_controller = None  # Robot1 경로 추종 컨트롤러
        self.path_controller_robot2 = None  #  Robot2 경로 추종 컨트롤러
        self.arm_controller = None  # Robot1 arm controller
        self.arm_controller_robot2 = None  #  Robot2 arm controller
        self.grasp_checker = None
        self.is_hybrid_controller = False  # 하이브리드 컨트롤러 사용 여부
        self.is_hybrid_controller_robot2 = False  #  Robot2 하이브리드 컨트롤러
        
        # ✅ General arm holders (for LLM/manual/idle modes - always available)
        self.arm_holder = None  # Robot1 general arm holder
        self.arm_holder_robot2 = None  # Robot2 general arm holder
        self.torque_controller = None  # Robot1 general torque controller
        self.torque_controller_robot2 = None  # Robot2 general torque controller
        
        # ✅ Path following arm holders (only active during navigation)
        self.path_arm_holder = None  # Robot1 path following
        self.path_arm_holder_robot2 = None  # Robot2 path following
        self.path_torque_controller = None  # Robot1 path torque controller
        self.path_torque_controller_robot2 = None  # Robot2 path torque controller
        
        #  홈 자세 정의 (안정화된 후 저장)
        self.arm_home_q = np.array([0.0, 1, 0, 1, 0.0, 1.0, 0.0])  # Robot1 home position
        self.arm_home_q_robot2 = np.array([0.0, 1, 0, 1, 0.0, 1.0, 0.0])  # Robot2 home position
        
        #  Robot1 상태 저장 (홈 포지션으로 초기화)
        self.saved_arm_position = self.arm_home_q.copy()
        self.saved_gripper_state = 0.0  # Open gripper
        
        #  Robot2 상태 저장 (홈 포지션으로 초기화)
        self.saved_arm_position_robot2 = self.arm_home_q_robot2.copy()
        self.saved_gripper_state_robot2 = 0.0  # Open gripper
        
        #  Robot-specific held objects
        self.held_object_robot1 = None
        self.held_object_robot2 = None

        #  IK 솔버 초기화 (Robot1 & Robot2)
        bounds = self.config.get_arm_joint_bounds()
        
        # Robot1 IK solver (Async for non-blocking)
        from config.constants import ARM_Q_IDX
        self.ik_solver = AsyncIKSolver(
            self.model, self.data, 
            np.array(ARM_Q_IDX), bounds,  # ✅ Use correct indices
            self.config.ee_site_id,
            max_workers=1,  # 1 workers for parallel IK solving
            model_xml_path=self.model_xml_path  # ← ADD THIS LINE
        )
        
        # Robot2 IK solver (Async for non-blocking)
        from config.constants import ROBOT2_ARM_Q_IDX
        robot2_ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "robot2/robot2_pinch_site")
        self.ik_solver_robot2 = AsyncIKSolver(
            self.model, self.data,
            np.array(ROBOT2_ARM_Q_IDX), bounds,
            robot2_ee_site_id,
            max_workers=1,  # 1 workers for parallel IK solving
            model_xml_path=self.model_xml_path  # ← ADD THIS LINE
        )
        
    def initialize_viewer(self):
        """뷰어 초기화"""
        self.viewer_manager.initialize()
    

    def initialize_arm_controllers(self):
        """ Robot1과 Robot2의 arm controller 초기화"""
        from config.constants import ARM_Q_IDX, ARM_CTRL_IDX, GRIP_CTRL_IDX,ROBOT2_GRIP_CTRL_IDX,BASE_CTRL_SLICE,ROBOT2_BASE_CTRL_SLICE,ROBOT2_ARM_Q_IDX,ROBOT2_ARM_CTRL_IDX
        
        # Robot1 arm controller
        if not self.arm_controller:
            self.arm_controller = ArmController(
                model=self.model,
                data=self.data,
                joint_idx=np.array(ARM_Q_IDX),
                ctrl_idx=np.array(ARM_CTRL_IDX),
                shared_gripper_ctrl=self.shared_gripper_ctrl,
                viewer=self.viewer_manager.viewer,
                use_dob=True,
                use_eso=True,
                eso_type='linear',
                eso_omega=5.0,
                robot_id='robot1',
                gripper_ctrl_idx=GRIP_CTRL_IDX,
                base_ctrl_idx=BASE_CTRL_SLICE
            )
        
        # Robot2 arm controller
        if not self.arm_controller_robot2:
            self.arm_controller_robot2 = ArmController(
                model=self.model,
                data=self.data,
                joint_idx=np.array(ROBOT2_ARM_Q_IDX),
                ctrl_idx=np.array(ROBOT2_ARM_CTRL_IDX),
                shared_gripper_ctrl=self.shared_gripper_ctrl_robot2,
                viewer=self.viewer_manager.viewer,
                use_dob=True,
                use_eso=True,
                eso_type='linear',
                eso_omega=5.0,
                robot_id='robot2',
                gripper_ctrl_idx=ROBOT2_GRIP_CTRL_IDX,
                base_ctrl_idx=ROBOT2_BASE_CTRL_SLICE
            )
    
    def initialize_arm_holder(self, robot_id=1):
        """
        Initialize general arm holder (for LLM/manual/idle modes)
        
        This is separate from path_arm_holder and can be used anytime,
        not just during navigation.
        
        Args:
            robot_id: 1 for Robot1, 2 for Robot2
        """
        from controllers.arm.torque_controller import TorqueController
        from controllers.arm.arm_holder import ArmHolder
        import numpy as np
        
        if robot_id == 1:
            from config.constants import ARM_Q_IDX
            joint_idx = ARM_Q_IDX
            
            # Create torque controller
            self.torque_controller = TorqueController(
                self.model, 
                self.data, 
                joint_idx=joint_idx, 
                use_dob=False
            )
            
            # Create arm holder
            self.arm_holder = ArmHolder(
                self.model, 
                self.data, 
                self.torque_controller, 
                joint_idx=joint_idx
            )
            
            # Initialize with current position
            current_q = np.copy(self.data.qpos[joint_idx])
            self.arm_holder._init_ruckig(current_q)
            
            print(f"[SimulationManager] Robot1 ✅ General arm holder initialized")
            
        else:  # robot_id == 2
            from config.constants import ROBOT2_ARM_Q_IDX
            joint_idx = ROBOT2_ARM_Q_IDX
            
            # Create torque controller
            self.torque_controller_robot2 = TorqueController(
                self.model, 
                self.data, 
                joint_idx=joint_idx, 
                use_dob=False
            )
            
            # Create arm holder
            self.arm_holder_robot2 = ArmHolder(
                self.model, 
                self.data, 
                self.torque_controller_robot2, 
                joint_idx=joint_idx
            )
            
            # Initialize with current position
            current_q = np.copy(self.data.qpos[joint_idx])
            self.arm_holder_robot2._init_ruckig(current_q)
            
            print(f"[SimulationManager] Robot2 ✅ General arm holder initialized")

    def get_grasp_checker(self, robot_id='robot1'):
        """로봇 ID에 따른 grasp checker 반환 (lazy initialization)"""
        from controllers.gripper.grasp_checker import GraspChecker
        
        if robot_id == 'robot2':
            if not hasattr(self, "grasp_checker_robot2") or self.grasp_checker_robot2 is None:
                ee_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "robot2/end_effector")
                left_pad = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot2/left_pad")
                right_pad = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot2/right_pad")
                self.grasp_checker_robot2 = GraspChecker(
                    self.model, self.data, ee_site, left_pad, right_pad,
                    viewer=self.viewer_manager.viewer
                )
            return self.grasp_checker_robot2
        else:
            if not hasattr(self, "grasp_checker") or self.grasp_checker is None:
                self.grasp_checker = GraspChecker(
                    self.model, self.data, self.config.ee_site_id,
                    self.config.left_pad_body_id, self.config.right_pad_body_id,
                    viewer=self.viewer_manager.viewer
                )
            return self.grasp_checker
        
    def get_feasibility_checker(self, robot_id='robot1'):
        """로봇 ID에 따른 feasibility checker 반환 (lazy initialization)
        
        Args:
            robot_id: 'robot1' or 'robot2'
            
        Returns:
            FeasibilityChecker: Checker instance for the specified robot
        """
        from tasks.feasibility_checker import FeasibilityChecker
        
        # Convert robot_id to number
        robot_num = 1 if robot_id == 'robot1' else 2
        
        # Create instance variable name
        attr_name = f"feasibility_checker_{robot_id}"
        
        # Lazy initialization
        if not hasattr(self, attr_name) or getattr(self, attr_name) is None:
            checker = FeasibilityChecker(
                self.model,
                self.data,
                self.get_ik_solver(robot_id),
                self.config,
                robot_id=robot_num
            )
            setattr(self, attr_name, checker)
        
        return getattr(self, attr_name)
     
    def start_mobility_control(self):
        from config.constants import BASE_CTRL_SLICE, ARM_Q_IDX, ARM_CTRL_IDX, GRIP_CTRL_IDX
        """Robot1 모빌리티 컨트롤러 시작 (초기화만 수행, 스레드 없음)"""
        if self.mobility_controller:
            return
        
        # Robot1 컨트롤러 생성 (✅ INCREASED speed)
        self.mobility_controller = VelocityMobilityController(
            self.model, 
            self.data,
            self.viewer_manager.viewer,
            linear_speed=2.5,  
            angular_speed=2.0,  # ✅ Increased from 1 - faster rotation
            initial_arm_position=self.saved_arm_position if hasattr(self, 'saved_arm_position') else None,
            initial_gripper_state=self.saved_gripper_state if hasattr(self, 'saved_gripper_state') else None,
            robot_id=1,
            base_ctrl_slice=BASE_CTRL_SLICE,
            arm_q_idx=ARM_Q_IDX,
            arm_ctrl_idx=ARM_CTRL_IDX,
            gripper_ctrl_idx=GRIP_CTRL_IDX
        )
        
        # 초기화만 수행 (스레드 시작 안 함)
        self.mobility_controller.start()
    
    def start_mobility_control_robot2(self):
        """Robot2 모빌리티 컨트롤러 시작"""
        if self.mobility_controller_robot2:
            return
        
        from config.constants import ROBOT2_BASE_CTRL_SLICE, ROBOT2_ARM_Q_IDX, ROBOT2_ARM_CTRL_IDX, ROBOT2_GRIP_CTRL_IDX
        
        # Robot2 컨트롤러 생성 (✅ INCREASED speed)
        self.mobility_controller_robot2 = VelocityMobilityController(
            self.model, 
            self.data,
            self.viewer_manager.viewer,
            linear_speed=2.5,  # ✅ Increased from 1 - faster base movement
            angular_speed=2.0,  # ✅ Increased from 1 - faster rotation
            initial_arm_position=self.saved_arm_position_robot2,
            initial_gripper_state=self.saved_gripper_state_robot2,
            robot_id=2,
            base_ctrl_slice=ROBOT2_BASE_CTRL_SLICE,
            arm_q_idx=ROBOT2_ARM_Q_IDX,
            arm_ctrl_idx=ROBOT2_ARM_CTRL_IDX,
            gripper_ctrl_idx=ROBOT2_GRIP_CTRL_IDX
        )
        
        # 초기화만 수행
        self.mobility_controller_robot2.start()
    
    def update_mobility_control(self):
        """모빌리티 컨트롤러 업데이트 (메인 루프에서 호출)"""
        # Robot1 업데이트
        if self.mobility_controller and self.mobility_controller.is_running():
            self.mobility_controller.update()
        
        # Robot2 업데이트
        if self.mobility_controller_robot2 and self.mobility_controller_robot2.is_running():
            self.mobility_controller_robot2.update()
        
    def stop_mobility_control(self):
        """Mobility 컨트롤 정지 (Robot1)"""
        if self.mobility_controller:
            # 현재 로봇팔 위치와 그리퍼 상태 저장
            if hasattr(self.mobility_controller, 'arm_holder'):
                from config.constants import ARM_Q_IDX, GRIP_CTRL_IDX
                self.saved_arm_position = self.data.qpos[ARM_Q_IDX].copy()  # ✅ Use correct indices
            
            # 그리퍼 상태 저장
            if hasattr(self.mobility_controller, 'gripper_value'):
                self.saved_gripper_state = self.mobility_controller.gripper_value
                # print(f"[SimulationManager] Robot1 그리퍼 상태 저장: {self.saved_gripper_state:.0f}")
            else:
                self.saved_gripper_state = self.data.ctrl[GRIP_CTRL_IDX]
                # print(f"[SimulationManager] Robot1 그리퍼 상태 저장 (ctrl): {self.saved_gripper_state:.0f}")
            
            self.mobility_controller.stop()
            self.mobility_controller = None
    
    def stop_mobility_control_robot2(self):
        """Mobility 컨트롤 정지 (Robot2)"""
        if self.mobility_controller_robot2:
            from config.constants import ROBOT2_ARM_Q_IDX, ROBOT2_GRIP_CTRL_IDX
            # Robot2 팔 위치 저장
            if hasattr(self.mobility_controller_robot2, 'arm_holder'):
                self.saved_arm_position_robot2 = self.data.qpos[ROBOT2_ARM_Q_IDX].copy()
            
            # Robot2 그리퍼 상태 저장
            if hasattr(self.mobility_controller_robot2, 'gripper_value'):
                self.saved_gripper_state_robot2 = self.mobility_controller_robot2.gripper_value
            else:
                self.saved_gripper_state_robot2 = self.data.ctrl[ROBOT2_GRIP_CTRL_IDX]
            
            self.mobility_controller_robot2.stop()
            self.mobility_controller_robot2 = None
            
    def initialize_path_controller(self, map_path=None):
        """Robot1 경로 추종 컨트롤러 초기화 (with warm starting!)"""
        if self.path_controller:
            self.stop_path_control()
        
        # ✅ NEW: AsyncMPCController with multiprocessing + warm starting
        print(f"[SimulationManager] Robot1 🚀 Initializing AsyncMPCController...")
        self.path_controller = AsyncMPCController(
            robot_id=1,
            model=self.model,  # ✅ Pass model for state queries
            data=self.data      # ✅ Pass data for state queries
        )
        
        # Load map into worker process
        if map_path:
            success = self.path_controller.load_map(map_path)
            if not success:
                print(f"[SimulationManager] Robot1 ❌ Failed to load map: {map_path}")
                return False
        
        # Robot1 Async MPC Planner (for path planning)
        self.async_planner = AsyncMPCPlanner(
            self.path_controller,
            max_workers=1
        )
        
        self.is_hybrid_controller = True
        
        # ✅ Initialize PATH-SPECIFIC arm holder (separate from general arm_holder)
        from config.constants import ARM_Q_IDX
        from controllers.arm.torque_controller import TorqueController
        from controllers.arm.arm_holder import ArmHolder
        import numpy as np
        
        # Create torque controller for path following
        self.path_torque_controller = TorqueController(
            self.model, 
            self.data, 
            joint_idx=ARM_Q_IDX, 
        )
        
        # Create arm holder for path following
        self.path_arm_holder = ArmHolder(
            self.model, 
            self.data, 
            self.path_torque_controller, 
            joint_idx=ARM_Q_IDX
        )
        
        # Initialize with current position
        current_q = np.copy(self.data.qpos[ARM_Q_IDX])
        self.path_arm_holder._init_ruckig(current_q)
        
        print(f"[SimulationManager] Robot1 ✅ AsyncMPCController initialized - 3-4ms solves!")
        print(f"[SimulationManager] Robot1 ✅ Path arm holder initialized")
        return True
    
    def initialize_path_controller_robot2(self, map_path=None):
        """Robot2 경로 추종 컨트롤러 초기화 (with warm starting!)"""
        if self.path_controller_robot2:
            self.stop_path_control_robot2()
        
        # ✅ NEW: AsyncMPCController with multiprocessing + warm starting
        print(f"[SimulationManager] Robot2 🚀 Initializing AsyncMPCController...")
        self.path_controller_robot2 = AsyncMPCController(
            robot_id=2,
            model=self.model,  # ✅ Pass model for state queries
            data=self.data      # ✅ Pass data for state queries
        )
        
        # Load map into worker process
        if map_path:
            success = self.path_controller_robot2.load_map(map_path)
            if not success:
                print(f"[SimulationManager] Robot2 ❌ Failed to load map: {map_path}")
                return False
        
        # Robot2 Async MPC Planner (for path planning)
        self.async_planner_robot2 = AsyncMPCPlanner(
            self.path_controller_robot2,
            max_workers=1
        )
        
        self.is_hybrid_controller_robot2 = True
        
        # ✅ Initialize PATH-SPECIFIC arm holder (separate from general arm_holder_robot2)
        from config.constants import ROBOT2_ARM_Q_IDX
        from controllers.arm.torque_controller import TorqueController
        from controllers.arm.arm_holder import ArmHolder
        import numpy as np
        
        # Create torque controller for path following
        self.path_torque_controller_robot2 = TorqueController(
            self.model, 
            self.data, 
            joint_idx=ROBOT2_ARM_Q_IDX, 
        )
        
        # Create arm holder for path following
        self.path_arm_holder_robot2 = ArmHolder(
            self.model, 
            self.data, 
            self.path_torque_controller_robot2, 
            joint_idx=ROBOT2_ARM_Q_IDX
        )
        
        # Initialize with current position
        current_q = np.copy(self.data.qpos[ROBOT2_ARM_Q_IDX])
        self.path_arm_holder_robot2._init_ruckig(current_q)
        
        print(f"[SimulationManager] Robot2 ✅ AsyncMPCController initialized - 3-4ms solves!")
        print(f"[SimulationManager] Robot2 ✅ Path arm holder initialized")
        return True
    
    def update_path_control(self, robot_system=None):
        """Robot1 경로 컨트롤러 업데이트 (uses get_velocity)
        
        Args:
            robot_system: Reference to NavigationPickPlaceSystem for accessing arm_busy flags
        """
        if self.path_controller and self.is_hybrid_controller:
            # ✅ Get robot position from body xpos (accurate world position)
            from config.constants import BASE_Q_SLICE, BASE_CTRL_SLICE, ARM_CTRL_IDX
            import numpy as np
            
            # ✅✅ ROBUST Safety check: Always apply zero velocities if navigation not active
            # This handles stop(), shutdown(), or any invalid state
            try:
                is_active = self.path_controller.navigation_active
            except (AttributeError, ReferenceError):
                # Controller in invalid state, apply zero velocities
                is_active = False
            
            if not is_active:
                self.data.ctrl[BASE_CTRL_SLICE.start] = 0.0      # ctrl[0] - world X velocity
                self.data.ctrl[BASE_CTRL_SLICE.start + 1] = 0.0  # ctrl[1] - world Y velocity
                self.data.ctrl[BASE_CTRL_SLICE.start + 2] = 0.0  # ctrl[2] - angular velocity
                
                # ✅ FIX: Check arm busy flag from robot_system
                arm_is_busy = robot_system.is_arm_busy_robot1 if robot_system else False
                if self.path_arm_holder is not None and not arm_is_busy:
                    try:
                        torque = self.path_arm_holder.compute_hold_torque()
                        self.data.ctrl[ARM_CTRL_IDX] = torque
                    except:
                        pass  # Silently handle any errors
                return
            
            body_name = "robot1_base_link"
            body_id = self.model.body(body_name).id
            body_xpos = self.data.xpos[body_id]
            x = body_xpos[0]  # World X position
            y = body_xpos[1]  # World Y position
            
            # Get yaw from qpos
            base_pos = self.data.qpos[BASE_Q_SLICE]
            yaw = base_pos[2]
            
            # Get velocity commands (non-blocking!)
            v, omega = self.path_controller.get_velocity(x, y, yaw)
            
            # ✅ Convert body frame velocity to world frame (CRITICAL!)
            # MPC outputs (v, omega) in body frame
            # Base needs (vx_world, vy_world, omega) in world frame
            vx_world = v * np.cos(yaw)
            vy_world = v * np.sin(yaw)
            
            # ✅ Apply world frame velocities (holonomic base)
            self.data.ctrl[BASE_CTRL_SLICE.start] = vx_world      # ctrl[0] - world X velocity
            self.data.ctrl[BASE_CTRL_SLICE.start + 1] = vy_world  # ctrl[1] - world Y velocity
            self.data.ctrl[BASE_CTRL_SLICE.start + 2] = omega     # ctrl[2] - angular velocity
            
            # ✅ Apply arm holding torque (check busy flag from robot_system)
            arm_is_busy = robot_system.is_arm_busy_robot1 if robot_system else False
            if self.path_arm_holder is not None and not arm_is_busy:
                torque = self.path_arm_holder.compute_hold_torque()
                self.data.ctrl[ARM_CTRL_IDX] = torque
    
    def update_path_control_robot2(self, robot_system=None):
        """Robot2 경로 컨트륤러 업데이트 (uses get_velocity)
        
        Args:
            robot_system: Reference to NavigationPickPlaceSystem for accessing arm_busy flags
        """
        if self.path_controller_robot2 and self.is_hybrid_controller_robot2:
            # ✅ Get Robot2 position from body xpos (accurate world position)
            from config.constants import ROBOT2_BASE_Q_SLICE, ROBOT2_BASE_CTRL_SLICE, ROBOT2_ARM_CTRL_IDX
            import numpy as np
            
            # ✅✅ ROBUST Safety check: Always apply zero velocities if navigation not active
            # This handles stop(), shutdown(), or any invalid state
            try:
                is_active = self.path_controller_robot2.navigation_active
            except (AttributeError, ReferenceError):
                # Controller in invalid state, apply zero velocities
                is_active = False
            
            if not is_active:
                self.data.ctrl[ROBOT2_BASE_CTRL_SLICE.start] = 0.0      # ctrl[11] - world X velocity
                self.data.ctrl[ROBOT2_BASE_CTRL_SLICE.start + 1] = 0.0  # ctrl[12] - world Y velocity
                self.data.ctrl[ROBOT2_BASE_CTRL_SLICE.start + 2] = 0.0  # ctrl[13] - angular velocity
                
                # ✅ FIX: Check arm busy flag from robot_system
                arm_is_busy = robot_system.is_arm_busy_robot2 if robot_system else False
                if self.path_arm_holder_robot2 is not None and not arm_is_busy:
                    try:
                        torque = self.path_arm_holder_robot2.compute_hold_torque()
                        self.data.ctrl[ROBOT2_ARM_CTRL_IDX] = torque
                    except:
                        pass  # Silently handle any errors
                return
            
            body_name = "robot2_base_link"
            body_id = self.model.body(body_name).id
            body_xpos = self.data.xpos[body_id]
            x = body_xpos[0]  # World X position
            y = body_xpos[1]  # World Y position
            
            # Get yaw from qpos
            base_pos = self.data.qpos[ROBOT2_BASE_Q_SLICE]
            yaw = base_pos[2]
            
            # Get velocity commands (non-blocking!)
            v, omega = self.path_controller_robot2.get_velocity(x, y, yaw)
            
            # ✅ Convert body frame velocity to world frame (CRITICAL!)
            # MPC outputs (v, omega) in body frame
            # Base needs (vx_world, vy_world, omega) in world frame
            vx_world = v * np.cos(yaw)
            vy_world = v * np.sin(yaw)
            
            # ✅ Apply world frame velocities (holonomic base)
            self.data.ctrl[ROBOT2_BASE_CTRL_SLICE.start] = vx_world      # ctrl[11] - world X velocity
            self.data.ctrl[ROBOT2_BASE_CTRL_SLICE.start + 1] = vy_world  # ctrl[12] - world Y velocity
            self.data.ctrl[ROBOT2_BASE_CTRL_SLICE.start + 2] = omega     # ctrl[13] - angular velocity
            
            # ✅ Apply arm holding torque (check busy flag from robot_system)
            arm_is_busy = robot_system.is_arm_busy_robot2 if robot_system else False
            if self.path_arm_holder_robot2 is not None and not arm_is_busy:
                torque = self.path_arm_holder_robot2.compute_hold_torque()
                self.data.ctrl[ROBOT2_ARM_CTRL_IDX] = torque
        
    def stop_path_control(self):
        """로봇1 경로 컨트롤러 정지"""
        if self.path_controller is not None:
            print(f"[SimulationManager] Robot1 stopping navigation...")
            self.path_controller.stop()  # ← Change shutdown() to stop()
            print(f"[SimulationManager] Robot1 navigation stopped")
    
    def stop_path_control_robot2(self):
        """로봇2 경로 컨트롤러 정지"""
        if self.path_controller_robot2 is not None:
            print(f"[SimulationManager] Robot2 stopping navigation...")
            self.path_controller_robot2.stop()  # ← Change shutdown() to stop()
            print(f"[SimulationManager] Robot2 navigation stopped")
         
    def reset_simulation(self):
        """시뮬레이션을 초기 상태로 리셋"""
        mujoco.mj_resetData(self.model, self.data)
                
        self.shared_gripper_ctrl[0] = 0.0
        from config.constants import ARM_Q_IDX
        self.data.qpos[ARM_Q_IDX] = self.arm_home_q  
        
        mujoco.mj_forward(self.model, self.data)
        
        print("[SimulationManager] 시뮬레이션이 리셋되었습니다.")
        
    def step(self):
        """단일 물리 스텝 실행 (메인 루프에서 호출)"""
        mujoco.mj_step(self.model, self.data)
        self.viewer_manager.sync()

    def set_arm_busy(self, robot_system, robot_id, busy):
        """로봇별 arm busy 플래그 설정"""
        flag = f"is_arm_busy_{robot_id}"
        if hasattr(robot_system, flag):
            setattr(robot_system, flag, busy)

    def set_arm_hold_position(self, arm_position):
        """LLM 모드에서 저장한 로봇팔 위치 설정"""
        self.saved_arm_position = arm_position
    
    def set_gripper_state(self, gripper_state):
        """LLM 모드에서 저장한 그리퍼 상태 설정"""
        self.saved_gripper_state = gripper_state
    
    def set_arm_and_gripper_state(self, arm_position, gripper_state):
        """LLM 모드에서 저장한 로봇팔 위치와 그리퍼 상태 설정"""
        self.saved_arm_position = arm_position
        self.saved_gripper_state = gripper_state
    
    #  Robot 선택을 위한 헬퍼 메서드들
    def get_arm_controller(self, robot_id='robot1'):
        """로봇 ID에 따른 arm controller 반환"""
        if robot_id == 'robot2':
            return self.arm_controller_robot2
        return self.arm_controller
    
    def get_gripper_control(self, robot_id='robot1'):
        """로봇 ID에 따른 gripper control 배열 반환"""
        if robot_id == 'robot2':
            return self.shared_gripper_ctrl_robot2
        return self.shared_gripper_ctrl
    
    def get_mobility_controller(self, robot_id='robot1'):
        """로봇 ID에 따른 mobility controller 반환"""
        if robot_id == 'robot2':
            return self.mobility_controller_robot2
        return self.mobility_controller
    
    def get_path_controller(self, robot_id='robot1'):
        """로봇 ID에 따른 path controller 반환"""
        if robot_id == 'robot2':
            return self.path_controller_robot2
        return self.path_controller
    
    def get_base_indices(self, robot_id='robot1'):
        """로봇 ID에 따른 base control indices 반환"""
        if robot_id == 'robot2':
            return {'x': 11, 'y': 12, 'th': 13, 'qpos': slice(18, 21)}
        return {'x': 0, 'y': 1, 'th': 2, 'qpos': slice(0, 3)}
    
    def get_arm_indices(self, robot_id='robot1'):
        """로봇 ID에 따른 arm indices 반환"""
        from config.constants import ARM_Q_IDX, ARM_CTRL_IDX, ROBOT2_ARM_Q_IDX, ROBOT2_ARM_CTRL_IDX
        if robot_id == 'robot2':
            return {'qpos': ROBOT2_ARM_Q_IDX, 'ctrl': ROBOT2_ARM_CTRL_IDX}
        return {'qpos': ARM_Q_IDX, 'ctrl': ARM_CTRL_IDX}
    
    def get_ik_solver(self, robot_id='robot1'):
        """로봇 ID에 따른 IK solver 반환"""
        if robot_id == 'robot2':
            return self.ik_solver_robot2
        return self.ik_solver
    
    def clear_navigation(self, robot_system, robot_id):
        """로봇별 navigation 플래그 클리어"""
        nav_flag = "is_navigating_robot2" if robot_id == "robot2" else "is_navigating"
        if hasattr(robot_system, nav_flag) and getattr(robot_system, nav_flag):
            setattr(robot_system, nav_flag, False)

    def cleanup(self):
        """시뮬레이션 정리 및 리소스 해제"""
        print("[SimulationManager] 정리 시작...")
        
        # 1. 컨트롤러 정지
        if self.mobility_controller:
            print("[SimulationManager] Robot1 모빌리티 컨트롤러 정지 중...")
            self.stop_mobility_control()
        
        if self.mobility_controller_robot2:
            print("[SimulationManager] Robot2 모빌리티 컨트롤러 정지 중...")
            self.stop_mobility_control_robot2()
        
        # ✅ Path controllers - shutdown AsyncMPCController worker processes
        if self.path_controller:
            print("[SimulationManager] Robot1 경로 컨트롤러 정지 중...")
            if self.is_hybrid_controller:
                self.path_controller.stop()
                self.path_controller.shutdown(timeout=5.0)
                self.is_hybrid_controller = False
            print("[SimulationManager] Robot1 경로 컨트롤러 정지 완료")
        
        # ✅ Cleanup path arm holders
        if self.path_arm_holder:
            print("[SimulationManager] Robot1 path arm holder 정리")
            self.path_arm_holder = None
            self.path_torque_controller = None

        if self.path_controller_robot2:
            print("[SimulationManager] Robot2 경로 컨트롤러 정지 중...")
            if self.is_hybrid_controller_robot2:
                self.path_controller_robot2.stop()
                self.path_controller_robot2.shutdown(timeout=5.0)
                self.is_hybrid_controller_robot2 = False
            print("[SimulationManager] Robot2 경로 컨트롤러 정지 완료")
        
        if self.path_arm_holder_robot2:
            print("[SimulationManager] Robot2 path arm holder 정리")
            self.path_arm_holder_robot2 = None
            self.path_torque_controller_robot2 = None
        
        # ✅ Cleanup general arm holders
        if self.arm_holder:
            print("[SimulationManager] Robot1 general arm holder 정리")
            self.arm_holder = None
            self.torque_controller = None
        
        if self.arm_holder_robot2:
            print("[SimulationManager] Robot2 general arm holder 정리")
            self.arm_holder_robot2 = None
            self.torque_controller_robot2 = None
        
        #  Arm controllers 정지
        if self.arm_controller:
            if self.arm_controller.is_tracking:
                self.arm_controller.stop_trajectory()
            print("[SimulationManager] Robot1 arm controller 정지")
        
        if self.arm_controller_robot2:
            if self.arm_controller_robot2.is_tracking:
                self.arm_controller_robot2.stop_trajectory()
            print("[SimulationManager] Robot2 arm controller 정지")
        
        # 3. IK 솔버 정리 (worker threads 종료)
        if hasattr(self, 'ik_solver') and self.ik_solver:
            print("[SimulationManager] Robot1 IK solver 종료 중...")
            self.ik_solver.shutdown(timeout=2.0)
        
        if hasattr(self, 'ik_solver_robot2') and self.ik_solver_robot2:
            print("[SimulationManager] Robot2 IK solver 종료 중...")
            self.ik_solver_robot2.shutdown(timeout=2.0)
        
        #  Async planner 정리 (worker threads 종료)
        if hasattr(self, 'async_planner') and self.async_planner:
            print("[SimulationManager] Robot1 Async MPC planner 종료 중...")
            self.async_planner.shutdown(timeout=5.0)
        
        if hasattr(self, 'async_planner_robot2') and self.async_planner_robot2:
            print("[SimulationManager] Robot2 Async MPC planner 종료 중...")
            self.async_planner_robot2.shutdown(timeout=5.0)
        
        # 4. 뷰어 정리
        if self.viewer_manager:
            print("[SimulationManager] 뷰어 정리 중...")
            if hasattr(self.viewer_manager, 'cleanup'):
                self.viewer_manager.cleanup()
        
            print("[SimulationManager]  정리 완료")
            
    def shutdown_all_workers(self):
            """Shutdown all async workers gracefully"""
            print("\n[SimulationManager] Shutting down all async workers...")
            
            # 1. Stop navigation first
            self.stop_path_control()
            self.stop_path_control_robot2()
            
            # 2. Shutdown Robot1 MPC worker
            if self.path_controller:
                print("[SimulationManager] Shutting down Robot1 MPC worker...")
                try:
                    self.path_controller._shutdown_flag.set()
                    if self.path_controller.worker_process.is_alive():
                        self.path_controller.worker_process.join(timeout=2.0)
                        if self.path_controller.worker_process.is_alive():
                            self.path_controller.worker_process.terminate()
                    print("[SimulationManager] ✅ Robot1 MPC worker terminated")
                except Exception as e:
                    print(f"[SimulationManager] ⚠️ Robot1 MPC shutdown error: {e}")
            
            # 3. Shutdown Robot2 MPC worker
            if self.path_controller_robot2:
                print("[SimulationManager] Shutting down Robot2 MPC worker...")
                try:
                    self.path_controller_robot2._shutdown_flag.set()
                    if self.path_controller_robot2.worker_process.is_alive():
                        self.path_controller_robot2.worker_process.join(timeout=2.0)
                        if self.path_controller_robot2.worker_process.is_alive():
                            self.path_controller_robot2.worker_process.terminate()
                    print("[SimulationManager] ✅ Robot2 MPC worker terminated")
                except Exception as e:
                    print(f"[SimulationManager] ⚠️ Robot2 MPC shutdown error: {e}")
            
            # 4. Shutdown IK workers
            if self.ik_solver:
                print("[SimulationManager] Shutting down Robot1 IK workers...")
                try:
                    self.ik_solver.shutdown()
                    print("[SimulationManager] ✅ Robot1 IK workers terminated")
                except Exception as e:
                    print(f"[SimulationManager] ⚠️ Robot1 IK shutdown error: {e}")
            
            if self.ik_solver_robot2:
                print("[SimulationManager] Shutting down Robot2 IK workers...")
                try:
                    self.ik_solver_robot2.shutdown()
                    print("[SimulationManager] ✅ Robot2 IK workers terminated")
                except Exception as e:
                    print(f"[SimulationManager] ⚠️ Robot2 IK shutdown error: {e}")
            
            # 5. Shutdown path planner workers
            if self.async_planner:
                print("[SimulationManager] Shutting down Robot1 path planner...")
                try:
                    self.async_planner.shutdown()
                    print("[SimulationManager] ✅ Robot1 planner terminated")
                except Exception as e:
                    print(f"[SimulationManager] ⚠️ Robot1 planner shutdown error: {e}")
            
            if self.async_planner_robot2:
                print("[SimulationManager] Shutting down Robot2 path planner...")
                try:
                    self.async_planner_robot2.shutdown()
                    print("[SimulationManager] ✅ Robot2 planner terminated")
                except Exception as e:
                    print(f"[SimulationManager] ⚠️ Robot2 planner shutdown error: {e}")
            
            print("[SimulationManager] ✅ All workers shut down")