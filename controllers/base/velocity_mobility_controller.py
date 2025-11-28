"""VelocityMobilityController - 속도 제어 + 팔 홀딩 (스레드 제거, 메인 루프 통합)"""

import numpy as np
from controllers.base.velocity_keyboard_handler import VelocityKeyboardHandler
from controllers.arm.torque_controller import TorqueController
from controllers.arm.arm_holder import ArmHolder
from config.constants import ARM_Q_IDX, BASE_CTRL_SLICE, ARM_CTRL_IDX, ROBOT2_BASE_CTRL_SLICE, ROBOT2_ARM_Q_IDX, ROBOT2_ARM_CTRL_IDX

class VelocityMobilityController:
    """속도 기반 베이스 텔레옵 + 팔 자세 유지 컨트롤러 (메인 루프에서 호출)

    VelocityKeyboardHandler에 BaseVelocityTeleop 기능이 통합되어 단일 핸들러로 동작
    """
    def __init__(self, model, data, viewer=None,
                 linear_speed=0.5, angular_speed=0.5,
                 max_linear_vel=1.0, max_angular_vel=1.0,
                 initial_arm_position=None,
                 initial_gripper_state=None, robot_id=1, base_ctrl_slice=None,
                 arm_q_idx=None, arm_ctrl_idx=None, gripper_ctrl_idx=None):
        """
        Args:
            model: MuJoCo model
            data: MuJoCo data
            viewer: 뷰어 (옵션)
            linear_speed: 초기 선속도 (m/s)
            angular_speed: 초기 각속도 (rad/s)
            max_linear_vel: 최대 선속도 (m/s)
            max_angular_vel: 최대 각속도 (rad/s)
            initial_arm_position: 초기 로봇팔 위치 (None이면 현재 위치 사용)
            initial_gripper_state: 초기 그리퍼 상태 (None이면 열린 상태)
            robot_id: 로봇 번호 (1 or 2)
            base_ctrl_slice: 베이스 제어 슬라이스 (None이면 robot1 기본값 사용)
            arm_q_idx: 팔 조인트 qpos 인덱스 (None이면 robot1 기본값 사용)
            arm_ctrl_idx: 팔 제어 인덱스 (None이면 robot1 기본값 사용)
            gripper_ctrl_idx: 그리퍼 제어 인덱스 (None이면 자동 계산)
        """
        self.model = model
        self.data = data
        self.viewer = viewer
        self.robot_id = robot_id

        # 로봇별 인덱스 설정
        if robot_id == 1:
            self.base_ctrl_slice = base_ctrl_slice if base_ctrl_slice is not None else BASE_CTRL_SLICE
            self.joint_idx = arm_q_idx if arm_q_idx is not None else ARM_Q_IDX
            self.arm_ctrl_idx = arm_ctrl_idx if arm_ctrl_idx is not None else ARM_CTRL_IDX
            self.gripper_ctrl_idx = gripper_ctrl_idx if gripper_ctrl_idx is not None else 10
        else:  # robot_id == 2
            self.base_ctrl_slice = base_ctrl_slice if base_ctrl_slice is not None else ROBOT2_BASE_CTRL_SLICE
            self.joint_idx = arm_q_idx if arm_q_idx is not None else ROBOT2_ARM_Q_IDX
            self.arm_ctrl_idx = arm_ctrl_idx if arm_ctrl_idx is not None else ROBOT2_ARM_CTRL_IDX
            self.gripper_ctrl_idx = gripper_ctrl_idx if gripper_ctrl_idx is not None else 21

        # 통합된 키보드 핸들러 (BaseVelocityTeleop 기능 포함)
        self.keyboard_handler = VelocityKeyboardHandler(
            data=data,
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            robot_id=robot_id,
            base_ctrl_slice=self.base_ctrl_slice
        )
        
        # 토크 컨트롤러 + 팔 홀더 (robot2도 지원하도록 수정)
        self.torque_controller = TorqueController(model, data, joint_idx=self.joint_idx, use_dob=True)
        self.arm_holder = ArmHolder(model, data, self.torque_controller, joint_idx=self.joint_idx)
        
        # 초기 상태
        self.initial_arm_position = initial_arm_position
        self.initial_gripper_state = initial_gripper_state
        
        # 활성화 상태
        self._is_active = False
        self.gripper_toggle_pressed = False
        self.gripper_value = 0.0
        
    def start(self):
        """컨트롤러 시작 (초기화만 수행)"""
        # 팔 초기화
        if self.initial_arm_position is not None:
            self.arm_holder._init_ruckig(self.initial_arm_position)
        else:
            current_q = np.copy(self.data.qpos[self.joint_idx])
            self.arm_holder._init_ruckig(current_q)
        
        # 그리퍼 초기화
        if self.initial_gripper_state is not None:
            self.gripper_value = self.initial_gripper_state
        else:
            self.gripper_value = 0.0
            
        
        # 속도 초기화
        self.data.ctrl[self.base_ctrl_slice] = 0.0
        
        self._is_active = True

        
    def update(self):
        """
        단일 제어 스텝 실행 (메인 루프에서 호출)
        
        메인 루프에서 이 메서드를 반복 호출하여 제어 명령을 업데이트합니다.
        물리 스텝은 메인 루프에서 별도로 처리합니다.
        """
        if not self._is_active:
            return
            
        # 뷰어 체크
        if self.viewer and not self.viewer.is_running():
            self._is_active = False
            return
            
        # 1. 베이스 속도 제어 (통합된 keyboard_handler.update() 사용)
        # Robot1: qpos[2] (joint_th), Robot2: qpos[20] (robot2_joint_th)
        from config.constants import USE_ROBOCASA, BASE_Q_SLICE, ROBOT2_BASE_QPOS_IDX

        if USE_ROBOCASA:
            # RoboCasa: heading is at the end of base slice (x, y, theta)
            heading_idx = (BASE_Q_SLICE.stop - 1) if self.robot_id == 1 else (ROBOT2_BASE_QPOS_IDX.stop - 1)
        else:
            # Simple scene
            heading_idx = 2 if self.robot_id == 1 else 20

        robot_heading = self.data.qpos[heading_idx]

        # 키보드 입력을 읽고 속도를 자동으로 적용 (통합된 기능 사용)
        self.keyboard_handler.update(robot_heading)
        
        # 2. 팔 홀드 (ArmHolder 사용)
        torque = self.arm_holder.compute_hold_torque()
        self.data.ctrl[self.arm_ctrl_idx] = torque
        
        # 3. 그리퍼 제어
        self.data.ctrl[self.gripper_ctrl_idx] = self.gripper_value
        
        # 그리퍼 토글
        g_pressed = self.keyboard_handler.is_key_pressed('g')
        if g_pressed and not self.gripper_toggle_pressed:
            if self.gripper_value < 128.0:
                self.gripper_value = 255.0
                print(f"[Robot{self.robot_id} 그리퍼 닫기]")
            else:
                self.gripper_value = 0.0
                print(f"[Robot{self.robot_id} 그리퍼 열기]")
        self.gripper_toggle_pressed = g_pressed
        
    def stop(self, timeout=1.0):
        """컨트롤러 정지"""
        self._is_active = False

        # 통합된 keyboard_handler.stop()이 베이스 정지와 리스너 정지를 모두 처리
        if hasattr(self, 'keyboard_handler'):
            self.keyboard_handler.stop()

        print(f"[VelocityMobilityController Robot{self.robot_id}] 종료")
    
    def is_running(self):
        """컨트롤러 실행 상태 확인"""
        return self._is_active
    
    def __del__(self):
        """소멸자"""
        if hasattr(self, 'keyboard_handler'):
            self.keyboard_handler.stop()
