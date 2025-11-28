"""로봇 관련 설정 및 초기화 - 다중 로봇 지원"""

import mujoco

class RobotConfig:
    """로봇 구성 및 ID 관리 - 다중 로봇 지원"""
    
    def __init__(self, model, robot_prefix=""):
        """
        Args:
            model: MuJoCo 모델
            robot_prefix: 로봇 이름 prefix (예: "", "robot2_")
        """
        self.model = model
        self.prefix = robot_prefix
        self._initialize_ids()
        
    def _initialize_ids(self):
        """모델에서 필요한 ID들을 초기화"""
        try:
            self.ee_site_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, f"{self.prefix}pinch_site"
            )
            self.left_pad_body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{self.prefix}left_pad"
            )
            self.right_pad_body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{self.prefix}right_pad"
            )
        except Exception as e:
            print(f"Warning: Failed to initialize IDs for robot '{self.prefix}': {e}")
        
    def get_arm_joint_bounds(self):
        """팔 관절 범위 반환"""
        arm_joint_ids = []
        for i in range(1, 8):
            try:
                joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{self.prefix}joint_{i}"
                )
                arm_joint_ids.append(joint_id)
            except:
                print(f"Warning: Could not find joint {self.prefix}joint_{i}")
                
        if not arm_joint_ids:
            return []
            
        joint_ranges = self.model.jnt_range[arm_joint_ids]
        return [(low, high) for (low, high) in joint_ranges]
    
    def get_base_joint_ids(self):
        """베이스 조인트 ID 반환"""
        try:
            return {
                'x': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{self.prefix}joint_x"),
                'y': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{self.prefix}joint_y"),
                'th': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{self.prefix}joint_th")
            }
        except Exception as e:
            print(f"Warning: Failed to get base joint IDs for robot '{self.prefix}': {e}")
            return {}
    
    def get_arm_joint_ids(self):
        """팔 조인트 ID 리스트 반환"""
        joint_ids = []
        for i in range(1, 8):
            try:
                joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{self.prefix}joint_{i}"
                )
                joint_ids.append(joint_id)
            except:
                print(f"Warning: Could not find joint {self.prefix}joint_{i}")
        return joint_ids


class MultiRobotConfig:
    """다중 로봇 관리 클래스"""
    
    def __init__(self, model, robot_prefixes=["", "robot2_"]):
        """
        Args:
            model: MuJoCo 모델
            robot_prefixes: 로봇 이름 prefix 리스트
        """
        self.model = model
        self.robots = {}
        
        for prefix in robot_prefixes:
            robot_name = prefix.rstrip('_') if prefix else 'robot1'
            self.robots[robot_name] = RobotConfig(model, prefix)
    
    def get_robot(self, robot_name='robot1'):
        """특정 로봇 설정 반환"""
        return self.robots.get(robot_name)
    
    def get_all_robots(self):
        """모든 로봇 설정 반환"""
        return self.robots
