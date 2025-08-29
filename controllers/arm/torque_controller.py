"""토크 제어 및 DOB"""

import numpy as np
import mujoco

class TorqueController:
    """토크 계산 및 외란 관측기"""
    
    def __init__(self, model, data, joint_idx):
        self.model = model
        self.data = data
        self.joint_idx = joint_idx
        self.num_joints = len(joint_idx)
        
        # 제어 게인
        self.Kp = np.eye(self.num_joints) * 1500
        self.Kd = np.eye(self.num_joints) * 30
        self.Ki = np.eye(self.num_joints) * 100
        
        # 적분 항
        self.integral_error = np.zeros(self.num_joints)
        self.max_integral = 0.3
        
        # DOB
        self.use_dob = True
        self.dob_gain = 5.0
        self.qd_prev = np.zeros(self.num_joints)
        self.filtered_disturbance = np.zeros(self.num_joints)
        
    def compute_torque(self, q_des, qd_des, qdd_des):
        """PID + 피드포워드 토크 계산"""
        q = self.data.qpos[self.joint_idx]
        qd = self.data.qvel[self.joint_idx]
        dt = self.model.opt.timestep
        
        # 오차
        pos_err = q_des - q
        vel_err = qd_des - qd
        
        # 적분 항 업데이트
        self.integral_error += pos_err * dt
        self.integral_error = np.clip(
            self.integral_error, -self.max_integral, self.max_integral
        )
        
        # 동역학 항
        M_full = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, M_full, self.data.qM)
        M = M_full[np.ix_(self.joint_idx, self.joint_idx)]
        
        mujoco.mj_rnePostConstraint(self.model, self.data)
        bias = self.data.qfrc_bias[self.joint_idx]
        
        # 토크 계산
        torque = M @ (
            qdd_des + self.Kp @ pos_err + self.Kd @ vel_err + 
            self.Ki @ self.integral_error
        ) + bias
        
        # DOB 적용
        if self.use_dob:
            torque = self._apply_dob(torque, q, qd, M, bias, dt)
            
        return np.clip(torque, -1000, 1000) / 100.0
        
    def _apply_dob(self, torque_nominal, qd, M, bias, dt):
        """외란 관측기 적용"""
        qdd_est = (qd - self.qd_prev) / dt
        self.qd_prev = np.copy(qd)
        
        disturbance = torque_nominal - (M @ qdd_est + bias)
        self.filtered_disturbance += self.dob_gain * dt * (
            disturbance - self.filtered_disturbance
        )
        
        return torque_nominal - self.filtered_disturbance