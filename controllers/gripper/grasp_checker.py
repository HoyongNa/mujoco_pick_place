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
            
    
    def check_grasp_state(self, force_threshold=1.0, min_pad_distance=0.02, max_pad_distance=0.15):
        """현재 파지 상태를 실시간으로 체크 (토크 제어기 연동용)
        
        Args:
            force_threshold: 파지 판단을 위한 최소 접촉력 (N)
            min_pad_distance: 최소 패드 간격 (m) - 이보다 작으면 물체 없음
            max_pad_distance: 최대 패드 간격 (m) - 이보다 크면 파지 실패
            
        Returns:
            tuple: (is_grasping, max_contact_force, pad_distance)
                - is_grasping: 파지 여부 (bool)
                - max_contact_force: 최대 접촉력 (float, N)
                - pad_distance: 패드 간 거리 (float, m)
        """
        # 패드 거리 계산
        pad_left = self.data.xipos[self.left_pad_body_id]
        pad_right = self.data.xipos[self.right_pad_body_id]
        pad_distance = np.linalg.norm(pad_left - pad_right)
        
        # 패드가 너무 가까우면 물체가 없는 것
        if pad_distance < min_pad_distance:
            return False, 0.0, pad_distance
        
        # 패드가 너무 멀면 파지 실패
        if pad_distance > max_pad_distance:
            return False, 0.0, pad_distance
        
        # 접촉력 확인
        max_force = 0.0
        for i in range(self.data.ncon):
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            force_magnitude = np.linalg.norm(force[:3])
            max_force = max(max_force, force_magnitude)
        
        # 파지 판단: 적절한 패드 간격 + 충분한 접촉력
        is_grasping = (min_pad_distance < pad_distance < max_pad_distance) and (max_force > force_threshold)
        
        return is_grasping, max_force, pad_distance