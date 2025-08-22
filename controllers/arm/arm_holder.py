"""팔 자세 유지 제어"""

import numpy as np
import mujoco
from ruckig import Ruckig, InputParameter, OutputParameter, Result
from config.constants import *

class ArmHolder:
    """팔 자세 유지 기능"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.num_joints = len(ARM_Q_IDX)
        
        self.ruckig = None
        self.rinp = None
        self.rout = None
        self.hold_q = None
        
    def init_ruckig(self, q_target):
        """Ruckig 초기화"""
        q_now = np.copy(self.data.qpos[ARM_Q_IDX])
        dt = float(self.model.opt.timestep)
        
        self.ruckig = Ruckig(self.num_joints, dt)
        self.rinp = InputParameter(self.num_joints)
        self.rout = OutputParameter(self.num_joints)
        
        self.rinp.current_position = q_now.tolist()
        self.rinp.current_velocity = [0.0] * self.num_joints
        self.rinp.current_acceleration = [0.0] * self.num_joints
        
        self.rinp.target_position = q_target.tolist()
        self.rinp.target_velocity = [0.0] * self.num_joints
        self.rinp.target_acceleration = [0.0] * self.num_joints
        
        self.rinp.max_velocity = [RUCKIG_MAX_V] * self.num_joints
        self.rinp.max_acceleration = [RUCKIG_MAX_A] * self.num_joints
        self.rinp.max_jerk = [RUCKIG_MAX_J] * self.num_joints
        
    def set_current_as_target(self):
        """현재 자세를 목표로 설정"""
        self.hold_q = np.copy(self.data.qpos[ARM_Q_IDX])
        self.init_ruckig(self.hold_q)
        
    def compute_hold_torque(self):
        """홀드 토크 계산"""
        if self.ruckig is None:
            return np.zeros(self.num_joints)
            
        # Ruckig 업데이트
        result = self.ruckig.update(self.rinp, self.rout)
        
        # 현재 상태
        q = self.data.qpos[ARM_Q_IDX]
        qd = self.data.qvel[ARM_Q_IDX]
        
        # 참조 궤적
        q_des = np.array(self.rout.new_position)
        qd_des = np.array(self.rout.new_velocity)
        qdd_des = np.array(self.rout.new_acceleration)
        
        # 오차
        pos_err = q_des - q
        vel_err = qd_des - qd
        
        # 동역학 계산
        M_full = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, M_full, self.data.qM)
        M = M_full[np.ix_(ARM_Q_IDX, ARM_Q_IDX)]
        
        mujoco.mj_rnePostConstraint(self.model, self.data)
        bias = self.data.qfrc_bias[ARM_Q_IDX]
        
        # 토크 계산
        torque = M @ (qdd_des + ARM_KP_HOLD * pos_err + ARM_KD_HOLD * vel_err) + bias
        torque = np.clip(torque, -ARM_TORQUE_LIMIT, ARM_TORQUE_LIMIT)
        
        # Ruckig 상태 업데이트
        self.rinp.current_position = self.rout.new_position
        self.rinp.current_velocity = self.rout.new_velocity
        self.rinp.current_acceleration = self.rout.new_acceleration
        
        # 목표 도달 시 재설정
        if result != Result.Working:
            self.set_current_as_target()
            
        return torque * ARM_TORQUE_SCALE