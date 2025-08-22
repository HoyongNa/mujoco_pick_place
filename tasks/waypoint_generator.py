"""웨이포인트 생성"""

import numpy as np

class WaypointGenerator:
    """Pick & Place 웨이포인트 생성"""
    
    def __init__(self, data):
        self.data = data
        self.default_rpy = np.array([np.pi, 0, -np.pi / 2])
        
    def generate_pick_place_waypoints(self, source_pos, target_pos):
        """픽앤플레이스 웨이포인트 생성"""
        waypoints = [
            # 접근
            (np.array([source_pos[0], source_pos[1], source_pos[2] + 0.15]), 
             self.default_rpy),
            # 파지
            (np.array([source_pos[0], source_pos[1], source_pos[2] + 0.03]), 
             self.default_rpy),
            # 상승
            (np.array([source_pos[0], source_pos[1], source_pos[2] + 0.25]), 
             self.default_rpy),
            # 이송
            (np.array([target_pos[0], target_pos[1], target_pos[2] + 0.25]), 
             self.default_rpy),
            # 드롭
            (np.array([target_pos[0], target_pos[1], target_pos[2] + 0.20]), 
             self.default_rpy),
            # 상승
            (np.array([target_pos[0], target_pos[1], target_pos[2] + 0.25]), 
             self.default_rpy),
            # 중앙
            (np.array([0.3, 0.0, 0.5]), self.default_rpy),
        ]
        return waypoints