"""Ruckig 기반 궤적 추종"""

import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter
from config.constants import *

class TrajectoryTracker:
    """Ruckig을 사용한 부드러운 궤적 추종"""
    
    def __init__(self, model, data, joint_idx):
        self.model = model
        self.data = data
        self.joint_idx = joint_idx
        self.num_joints = len(joint_idx)
        
    def create_trajectory(self, target_q, current_q=None):
        """목표 관절각까지의 궤적 생성"""
        if current_q is None:
            current_q = np.copy(self.data.qpos[self.joint_idx])
            
        dt = self.model.opt.timestep
        ruckig = Ruckig(self.num_joints, dt)
        inp = InputParameter(self.num_joints)
        
        inp.current_position = current_q.tolist()
        inp.current_velocity = [0.0] * self.num_joints
        inp.current_acceleration = [0.0] * self.num_joints
        
        inp.target_position = target_q.tolist()
        inp.target_velocity = [0.0] * self.num_joints
        inp.target_acceleration = [0.0] * self.num_joints
        
        inp.max_velocity = [RUCKIG_MAX_V] * self.num_joints
        inp.max_acceleration = [RUCKIG_MAX_A] * self.num_joints
        inp.max_jerk = [RUCKIG_MAX_J] * self.num_joints
        
        return ruckig, inp, OutputParameter(self.num_joints)