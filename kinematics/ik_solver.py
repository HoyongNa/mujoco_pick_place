"""역기구학 솔버 - 기존 코드 유지"""

import numpy as np
import mujoco
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

class InverseKinematicsSolver:
    """목표 위치와 회전을 기반으로 역기구학 해결"""
    
    def __init__(self, model, data, joint_idx, bounds, ee_site_id):
        self.model = model
        self.data = data
        self.joint_idx = joint_idx
        self.bounds = bounds
        self.ee_site_id = ee_site_id
        
    def solve(self, target_pos, target_rpy):
        """목표 위치와 회전을 위한 관절 값을 계산"""
        target_rot_mat = R.from_euler('xyz', target_rpy).as_matrix()
        
        def ik_cost(q):
            q_backup = np.copy(self.data.qpos)
            self.data.qpos[self.joint_idx] = q
            mujoco.mj_forward(self.model, self.data)
            ee_pos = self.data.site_xpos[self.ee_site_id]
            ee_rot = self.data.site_xmat[self.ee_site_id].reshape(3, 3)
            self.data.qpos[:] = q_backup
            
            pos_error = np.linalg.norm(target_pos - ee_pos)
            rot_error = 0.5 * np.linalg.norm(ee_rot - target_rot_mat, ord='fro')
            return pos_error + rot_error
            
        result = minimize(
            ik_cost,
            np.copy(self.data.qpos[self.joint_idx]),
            bounds=self.bounds,
            method='SLSQP',
            options={'ftol': 1e-6, 'maxiter': 500}
        )
        
        return result.x if result.success else self.data.qpos[self.joint_idx]