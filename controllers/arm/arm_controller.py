"""ArmController - 비동기(Async) 버전 (For Loop 제거) - Multi-Robot Support"""

import numpy as np
import time
from ruckig import Result
from controllers.arm.trajectory_initializer import TrajectoryInitializer
from controllers.arm.torque_controller import TorqueController
from controllers.arm.arm_holder import ArmHolder  #  Import ArmHolder

class ArmController:
    """팔 제어 통합 클래스 - 비동기 모드 (다중 로봇 지원)"""
    
    def __init__(self, model, data, joint_idx, ctrl_idx, 
                 shared_gripper_ctrl, use_dob=True, use_eso=True, 
                 eso_type='linear', eso_omega=10.0, viewer=None,
                use_velocity_filter=True,  #  속도 필터 활성화
                velocity_filter_alpha=0.3,  #  필터 계수
                robot_id='robot1',  #  로봇 ID (robot1, robot2)
                gripper_ctrl_idx=10,  #  그리퍼 제어 인덱스
                base_ctrl_idx=None):  #  베이스 제어 인덱스 (slice or list)
        """
        Args:
            model: MuJoCo model
            data: MuJoCo data
            joint_idx: 팔 관절 인덱스 (numpy array or list)
            ctrl_idx: 팔 제어 인덱스 (numpy array or list)
            shared_gripper_ctrl: 그리퍼 제어 공유 변수
            use_dob: DOB 사용 여부
            use_eso: ESO 사용 여부
            eso_type: ESO 타입 ('linear' or 'nonlinear')
            eso_omega: ESO 대역폭
            viewer: Viewer 객체
            step_manager: 중앙 스텝 관리자
            use_velocity_filter: 속도 필터 활성화
            velocity_filter_alpha: 필터 계수
            robot_id: 로봇 식별자 ('robot1', 'robot2', etc.)
            gripper_ctrl_idx: 그리퍼 제어 인덱스
            base_ctrl_idx: 베이스 제어 인덱스 (기본값: [0,1,2] for robot1)
        """
        
        self.model = model
        self.data = data
        self.joint_idx = joint_idx
        self.ctrl_idx = ctrl_idx
        self.shared_gripper_ctrl = shared_gripper_ctrl
        self.viewer = viewer
        self.robot_id = robot_id  #  로봇 ID 저장
        
        #  그리퍼 및 베이스 제어 인덱스 설정
        self.gripper_ctrl_idx = gripper_ctrl_idx
        if base_ctrl_idx is None:
            # 기본값: robot1은 [0,1,2], robot2는 [11,12,13]
            if robot_id == 'robot2':
                self.base_ctrl_idx = [11, 12, 13]
            else:
                self.base_ctrl_idx = [0, 1, 2]
        else:
            self.base_ctrl_idx = base_ctrl_idx
        
        
        # 구성 요소
        self.trajectory_initializer = TrajectoryInitializer(model, data, joint_idx)
        self.torque_controller = TorqueController(
            model, data, joint_idx, 
            use_dob=use_dob,
            use_eso=use_eso,
            eso_type=eso_type,
            eso_omega=eso_omega,
            use_velocity_filter=use_velocity_filter,  #  필터 설정 전달
            velocity_filter_alpha=velocity_filter_alpha  #  필터 계수 전달
        )
        
        #  비동기 모드를 위한 상태 변수들
        self.is_tracking = False
        self.target_q = None
        self.ruckig = None
        self.inp = None
        self.out = None
        self.result = None
        self.step_count = 0
        self.max_step = 100000
        self.last_print_time = time.time()
        self.print_interval = 1.0
        self.position_tolerance = 0.0001
        
        #  ArmHolder for smooth holding (reusing existing implementation)
        self.arm_holder = ArmHolder(
            model=self.model,
            data=self.data,
            torque_controller=self.torque_controller,
            joint_idx=self.joint_idx
        )
        
    def start_trajectory(self, target_q, max_step=100000, position_tolerance=0.0001):
        """
        새로운 궤적 추종 시작 (비동기 모드)
        
        Args:
            target_q: 목표 관절 각도 (numpy array)
            max_step: 최대 스텝 수
            position_tolerance: 위치 허용 오차
            
        Returns:
            bool: 성공 여부
        """
        if self.is_tracking:
            print(f"  [{self.robot_id}] 이미 궤적 추종 중입니다. 기존 궤적을 중지합니다.")
            self.stop_trajectory()
        
        # 궤적 초기화
        self.target_q = target_q.copy()
        self.ruckig, self.inp, self.out = self.trajectory_initializer.initialize_trajectory(target_q)
        self.result = Result.Working
        self.step_count = 0
        self.max_step = max_step
        self.position_tolerance = position_tolerance
        self.last_print_time = time.time()
        self.is_tracking = True
        
        print(f" [{self.robot_id}] 궤적 추종 시작 (비동기 모드)")
        return True
    
    def update(self):
        """
        궤적 추종 업데이트 (매 스텝마다 호출)
         FIX: is_tracking=False일 때도 팔 자세 유지(Hold)
        
        Returns:
            dict: 상태 정보
                - 'completed': bool, 궤적 완료 여부
                - 'error': float, 현재 위치 오차
                - 'step': int, 현재 스텝 수
                - 'status': str, 상태 메시지
        """
        if not self.is_tracking:
            #  FIX: 추종 중이 아닐 때는 현재 위치 유지 (Passive Hold)
            # self._hold_current_position()
            return {
                'completed': True,
                'error': 0.0,
                'step': self.step_count,
                'status': 'holding'
            }
        
        #  FIX: 현재 위치 오차를 먼저 계산 (stop_trajectory 호출 전에!)
        q = self.data.qpos[self.joint_idx]
        error_norm = np.linalg.norm(self.target_q - q)
        
        # 최대 스텝 확인
        if self.step_count >= self.max_step:
            self.stop_trajectory()
            return {
                'completed': True,
                'error': error_norm,
                'step': self.step_count,
                'status': 'max_step_reached'
            }
        
        # Ruckig 완료 확인
        if self.result != Result.Working:
            print(f"✅ [{self.robot_id}] 목표 trajectory 완료. Step {self.step_count}")
            self.stop_trajectory()
            return {
                'completed': True,
                'error': error_norm,
                'step': self.step_count,
                'status': 'ruckig_finished'
            }
        
        # Ruckig 업데이트
        self.result = self.ruckig.update(self.inp, self.out)
        q_des = np.array(self.out.new_position)
        qd_des = np.array(self.out.new_velocity)
        qdd_des = np.array(self.out.new_acceleration)
        
        # 토크 계산 및 적용
        torque = self.torque_controller.compute_torque(q_des, qd_des, qdd_des)
        self.data.ctrl[self.ctrl_idx] = torque
        
        #  베이스 속도 0으로 설정 (로봇별 인덱스 사용)
        self.data.ctrl[self.base_ctrl_idx] = 0.0
        
        #  그리퍼 (로봇별 인덱스 사용)
        self.data.ctrl[self.gripper_ctrl_idx] = self.shared_gripper_ctrl[0]
        
        # 진행 상황 출력
        if time.time() - self.last_print_time > self.print_interval:
            print(f"  [{self.robot_id}] 진행중... Step: {self.step_count}, Error: {error_norm:.4f}")
            self.last_print_time = time.time()
        
        # Ruckig 상태 업데이트
        self.inp.current_position = self.out.new_position
        self.inp.current_velocity = self.out.new_velocity
        self.inp.current_acceleration = self.out.new_acceleration
        
        # 스텝 카운트 증가
        self.step_count += 1
        
        # 목표 도달 확인
        if error_norm < self.position_tolerance:
            print(f" [{self.robot_id}] 목표 도달! Step {self.step_count}, Error: {error_norm:.6f}")
            self.stop_trajectory()
            return {
                'completed': True,
                'error': error_norm,
                'step': self.step_count,
                'status': 'target_reached'
            }
        
        # 뷰어 종료 확인
        if self.viewer and not self.viewer.is_running():
            print(f"  [{self.robot_id}] 사용자가 시뮬레이션을 중단했습니다.")
            self.stop_trajectory()
            return {
                'completed': True,
                'error': error_norm,
                'step': self.step_count,
                'status': 'viewer_closed'
            }
        
        return {
            'completed': False,
            'error': error_norm,
            'step': self.step_count,
            'status': 'tracking'
        }
    
    def _hold_current_position(self):
        """
         Hold current arm position using ArmHolder
        Reuses arm_holder.py implementation (Ruckig + ESO/DOB)
        Called when is_tracking=False
        """
        # Compute holding torque using ArmHolder (Ruckig-based smooth holding)
        torque = self.arm_holder.compute_hold_torque()
        
        # Apply torque to arm joints
        self.data.ctrl[self.ctrl_idx] = torque
        
        #  베이스 속도 0으로 설정
        self.data.ctrl[self.base_ctrl_idx] = 0.0
        
        #  그리퍼 유지
        self.data.ctrl[self.gripper_ctrl_idx] = self.shared_gripper_ctrl[0]
    
    def stop_trajectory(self):
        """궤적 추종 중지"""
        if self.is_tracking:  # Only log if actually stopping
            print(f"⏹  [{self.robot_id}] 궤적 추종 중지됨 - is_tracking: True -> False")
        self.is_tracking = False
        self.target_q = None
        self.ruckig = None
        self.inp = None
        self.out = None
        self.result = None
    
    def cleanup(self):
        """
         Cleanup resources (prevents Ruckig memory leaks)
        Call this before destroying the controller
        """
        # Clean up tracking Ruckig
        self.stop_trajectory()
        
        # Clean up ArmHolder's Ruckig
        if hasattr(self, 'arm_holder') and self.arm_holder is not None:
            self.arm_holder.ruckig = None
            self.arm_holder.rinp = None
            self.arm_holder.rout = None
        
        # print(f" [{self.robot_id}] ArmController cleanup complete")
    
    def get_status(self):
        """
        현재 상태 조회 (업데이트 없이)
        
        Returns:
            dict: 상태 정보
                - 'completed': bool, 궤적 완료 여부
                - 'is_tracking': bool, 추종 중인지 여부
                - 'step': int, 현재 스텝 수
                - 'status': str, 상태 메시지
        """
        if not self.is_tracking:
            return {
                'completed': True,
                'is_tracking': False,
                'step': self.step_count,
                'status': 'idle'
            }
        
        # 현재 오차 계산
        q = self.data.qpos[self.joint_idx]
        error_norm = np.linalg.norm(self.target_q - q) if self.target_q is not None else 0.0
        
        return {
            'completed': False,
            'is_tracking': True,
            'step': self.step_count,
            'error': error_norm,
            'status': 'tracking'
        }
    