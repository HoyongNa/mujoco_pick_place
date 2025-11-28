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
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                if self.viewer:
                    self._setup_camera()
                    # print("MuJoCo GUI viewer 초기화 완료")
                else:
                    print(" MuJoCo GUI viewer 생성 실패")
            except Exception as e:
                print(f" MuJoCo GUI viewer 초기화 오류: {e}")
                self.viewer = None
            
    def _setup_camera(self):
        """카메라 설정"""
        if self.viewer:
            try:
                self.viewer.cam.azimuth = CAM_AZIMUTH
                self.viewer.cam.elevation = CAM_ELEVATION
                self.viewer.cam.distance = CAM_DISTANCE
                self.viewer.cam.lookat = np.array(CAM_LOOKAT)
            except Exception as e:
                print(f"⚠️ 카메라 설정 오류: {e}")
            
    def sync(self):
        """뷰어 동기화"""
        if self.viewer and self.is_running():
            try:
                self.viewer.sync()
            except Exception as e:
                print(f"⚠️ 뷰어 동기화 오류: {e}")
                # 오류 발생 시 viewer 무효화
                self.viewer = None
            
    def is_running(self):
        """뷰어 실행 중 확인"""
        if self.viewer is None:
            return False
        try:
            return self.viewer.is_running()
        except Exception as e:
            print(f"⚠️ 뷰어 상태 확인 오류: {e}")
            return False
        
    def close(self):
        """뷰어 종료"""
        if self.viewer:
            try:
                self.viewer.close()
                print("MuJoCo GUI viewer 종료")
            except Exception as e:
                print(f"⚠️ 뷰어 종료 오류: {e}")
            finally:
                self.viewer = None
