"""로봇 관련 설정 및 초기화"""

import mujoco

class RobotConfig:
    """로봇 구성 및 ID 관리"""
    
    def __init__(self, model):
        self.model = model
        self._initialize_ids()
        
    def _initialize_ids(self):
        """모델에서 필요한 ID들을 초기화"""
        self.ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "pinch_site"
        )
        self.left_pad_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "left_pad"
        )
        self.right_pad_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "right_pad"
        )
        
    def get_arm_joint_bounds(self):
        """팔 관절 범위 반환"""
        arm_joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")
            for i in range(1, 8)
        ]
        joint_ranges = self.model.jnt_range[arm_joint_ids]
        return [(low, high) for (low, high) in joint_ranges]