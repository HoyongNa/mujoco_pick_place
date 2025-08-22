"""파지 체크 - 기존 코드 유지"""

import mujoco
import numpy as np
import time

class GraspChecker:
    """MuJoCo 시뮬레이션 상에서 로봇 그리퍼의 파지 성공 여부를 판단"""
    
    def __init__(self, model, data, ee_site_id, left_pad_body_id, 
                 right_pad_body_id, viewer=None):
        self.model = model
        self.data = data
        self.ee_site_id = ee_site_id
        self.left_pad_body_id = left_pad_body_id
        self.right_pad_body_id = right_pad_body_id
        self.viewer = viewer
        
    def update_viewer(self):
        """viewer가 있으면 업데이트"""
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()
            
    def wait_until_grasped(self, threshold=1.0, max_time=2.0, min_pad_distance=0.02):
        """일정 시간 동안 시뮬레이션을 반복하면서 파지 여부를 실시간 확인"""
        steps = int(max_time / self.model.opt.timestep)
        viewer_update_interval = 5
        last_print_time = time.time()
        
        for step in range(steps):
            mujoco.mj_step(self.model, self.data)
            
            if step % viewer_update_interval == 0:
                self.update_viewer()
                
            pad_left = self.data.xipos[self.left_pad_body_id]
            pad_right = self.data.xipos[self.right_pad_body_id]
            pad_distance = np.linalg.norm(pad_left - pad_right)
            
            if pad_distance < min_pad_distance:
                continue
                
            # 접촉 검사
            for i in range(self.data.ncon):
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, force)
                
                if force[0] > threshold:
                    print(f"파지 성공 (force: {force[0]:.2f}N, pad_dist: {pad_distance:.3f}m)")
                    self.update_viewer()
                    return True
                    
            if self.viewer is not None and time.time() - last_print_time > 1.0:
                print(f"  파지 확인 중... (pad_dist: {pad_distance:.3f}m)")
                last_print_time = time.time()
                
            if self.viewer is not None and not self.viewer.is_running():
                print(" 사용자가 시뮬레이션을 중단했습니다.")
                return False
                
        print("파지 실패 (시간 초과)")
        return False