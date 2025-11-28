"""팔 자세 유지(Holding) 전용 클래스
velocity_mobility_controller와 mpc_hybrid_controller에서 공통으로 사용
"""

import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, Result
from config.constants import (
    ARM_Q_IDX,
    RUCKIG_MAX_V, RUCKIG_MAX_A, RUCKIG_MAX_J
)


class ArmHolder:
    """팔 자세 유지 전용 클래스 (Ruckig + Torque Controller)
    
    TorqueController로부터 독립적으로 holding 로직을 구현하여
    각 컨트롤러가 필요에 따라 사용할 수 있도록 함
    """
    
    def __init__(self, model, data, torque_controller, joint_idx=None):
        """
        Args:
            model: MuJoCo model
            data: MuJoCo data
            torque_controller: TorqueController 인스턴스 (순수 토크 계산용)
            joint_idx: 제어할 관절 인덱스 (None이면 ARM_Q_IDX 사용)
        """
        self.model = model
        self.data = data
        self.torque_controller = torque_controller
        
        self.joint_idx = joint_idx if joint_idx is not None else ARM_Q_IDX
        self.num_joints = len(self.joint_idx)
        
        # Ruckig 초기화
        self.ruckig = None
        self.rinp = None
        self.rout = None
        self.target_q = None
        
        current_q = np.copy(self.data.qpos[self.joint_idx])
        self._init_ruckig(current_q)
    
    def _init_ruckig(self, q_target):
        """Ruckig 초기화 - 부드러운 궤적 생성"""
        q_now = np.copy(self.data.qpos[self.joint_idx])
        dt = float(self.model.opt.timestep)
        
        self.ruckig = Ruckig(self.num_joints, dt)
        self.rinp = InputParameter(self.num_joints)
        self.rout = OutputParameter(self.num_joints)
        
        # 현재 상태
        self.rinp.current_position = q_now.tolist()
        self.rinp.current_velocity = [0.0] * self.num_joints
        self.rinp.current_acceleration = [0.0] * self.num_joints
        
        # 목표 상태
        self.rinp.target_position = q_target.tolist()
        self.rinp.target_velocity = [0.0] * self.num_joints
        self.rinp.target_acceleration = [0.0] * self.num_joints
        
        # 제한값 (constants에서 가져옴)
        self.rinp.max_velocity = [RUCKIG_MAX_V] * self.num_joints
        self.rinp.max_acceleration = [RUCKIG_MAX_A] * self.num_joints
        self.rinp.max_jerk = [RUCKIG_MAX_J] * self.num_joints
        
        self.target_q = q_target
    
    def compute_hold_torque(self):
        """홀드 토크 계산
        
        Returns:
            토크 명령 (numpy array)
        """
        # Ruckig 업데이트 (부드러운 궤적 생성)
        result = self.ruckig.update(self.rinp, self.rout)
        
        # 참조 궤적 추출
        q_des = np.array(self.rout.new_position)
        qd_des = np.array(self.rout.new_velocity)
        qdd_des = np.array(self.rout.new_acceleration)
        
        # 토크 계산 (TorqueController 사용)
        torque = self.torque_controller.compute_torque(q_des, qd_des, qdd_des)
        
        # Ruckig 상태 업데이트
        self.rinp.current_position = self.rout.new_position
        self.rinp.current_velocity = self.rout.new_velocity
        self.rinp.current_acceleration = self.rout.new_acceleration
        
        # 목표 도달 시 현재 위치로 재설정
        if result != Result.Working:
            current_q = np.copy(self.data.qpos[self.joint_idx])
            self._init_ruckig(current_q)
        
        return torque
    
    def get_current_position(self):
        """현재 팔 관절 위치 반환"""
        return np.copy(self.data.qpos[self.joint_idx])
