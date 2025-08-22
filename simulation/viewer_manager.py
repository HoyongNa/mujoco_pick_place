"""MuJoCo 뷰어 관리"""

import mujoco
import mujoco.viewer
import numpy as np
from config.constants import *

class ViewerManager:
    """뷰어 초기화 및 관리"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.viewer = None
        
    def initialize(self):
        """뷰어 초기화"""
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._setup_camera()
            print("MuJoCo GUI viewer 초기화 완료")
            
    def _setup_camera(self):
        """카메라 설정"""
        if self.viewer:
            self.viewer.cam.azimuth = CAM_AZIMUTH
            self.viewer.cam.elevation = CAM_ELEVATION
            self.viewer.cam.distance = CAM_DISTANCE
            self.viewer.cam.lookat = np.array(CAM_LOOKAT)
            
    def sync(self):
        """뷰어 동기화"""
        if self.viewer and self.viewer.is_running():
            self.viewer.sync()
            
    def is_running(self):
        """뷰어 실행 중 확인"""
        return self.viewer is not None and self.viewer.is_running()
        
    def close(self):
        """뷰어 종료"""
        if self.viewer:
            self.viewer.close()
            self.viewer = None
            print("MuJoCo GUI viewer 종료")