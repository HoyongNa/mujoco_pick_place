"""키보드 입력 처리"""

import keyboard
import numpy as np
from config.constants import *

class KeyboardHandler:
    """키보드 입력을 베이스 명령으로 변환"""
    
    def __init__(self):
        self.key_map = {
            KEY_FORWARD: (0, BASE_LIN_STEP),
            KEY_BACKWARD: (0, -BASE_LIN_STEP),
            KEY_LEFT: (1, BASE_LIN_STEP),
            KEY_RIGHT: (1, -BASE_LIN_STEP),
            KEY_ROTATE_LEFT: (2, BASE_YAW_STEP),
            KEY_ROTATE_RIGHT: (2, -BASE_YAW_STEP),
        }
        
    def update_command(self, cmd_ref, robot_heading=None):
        """키 입력을 읽어 명령 업데이트
        
        Args:
            cmd_ref: 베이스 명령 참조 [x, y, theta]
            robot_heading: 로봇의 현재 헤딩 각도 (라디안). None이면 월드 좌표계 사용
        """
        if robot_heading is not None:
            # 로봇 좌표계 기준 조종
            return self._update_command_robot_frame(cmd_ref, robot_heading)
        else:
            # 월드 좌표계 기준 조종 (기존 방식)
            return self._update_command_world_frame(cmd_ref)
    
    def _update_command_world_frame(self, cmd_ref):
        """월드 좌표계 기준 명령 업데이트 (기존 방식)"""
        for key, (idx, delta) in self.key_map.items():
            if keyboard.is_pressed(key):
                cmd_ref[idx] += delta
                
        if keyboard.is_pressed(KEY_STOP):
            cmd_ref[:] = 0.0
            
        return cmd_ref.copy()
    
    def _update_command_robot_frame(self, cmd_ref, robot_heading):
        """로봇 좌표계 기준 명령 업데이트 (헤딩 기준)"""
        # 로봇 좌표계에서의 속도 명령
        robot_vx = 0.0  # 전진/후진
        robot_vy = 0.0  # 좌/우
        robot_vtheta = 0.0  # 회전
        
        # 키 입력 처리 (로봇 좌표계)
        if keyboard.is_pressed(KEY_FORWARD):
            robot_vx += BASE_LIN_STEP
        if keyboard.is_pressed(KEY_BACKWARD):
            robot_vx -= BASE_LIN_STEP
        if keyboard.is_pressed(KEY_LEFT):
            robot_vy += BASE_LIN_STEP
        if keyboard.is_pressed(KEY_RIGHT):
            robot_vy -= BASE_LIN_STEP
        if keyboard.is_pressed(KEY_ROTATE_LEFT):
            robot_vtheta += BASE_YAW_STEP
        if keyboard.is_pressed(KEY_ROTATE_RIGHT):
            robot_vtheta -= BASE_YAW_STEP
        
        # 로봇 좌표계 -> 월드 좌표계 변환
        cos_theta = np.cos(robot_heading)
        sin_theta = np.sin(robot_heading)
        
        # 회전 변환
        world_vx = cos_theta * robot_vx - sin_theta * robot_vy
        world_vy = sin_theta * robot_vx + cos_theta * robot_vy
        
        # 명령 업데이트
        cmd_ref[0] += world_vx
        cmd_ref[1] += world_vy
        cmd_ref[2] += robot_vtheta
        
        if keyboard.is_pressed(KEY_STOP):
            cmd_ref[:] = 0.0
            
        return cmd_ref.copy()
    
    def log_controls(self, robot_frame=True):
        """제어 키 안내 출력"""
        if robot_frame:
            print("[키보드 제어 - 로봇 헤딩 기준]")
            print(f"  전진/후진: {KEY_FORWARD}/{KEY_BACKWARD} (로봇이 바라보는 방향)")
            print(f"  좌/우 이동: {KEY_LEFT}/{KEY_RIGHT} (로봇 기준 좌우)")
        else:
            print("[키보드 제어 - 월드 좌표계 기준]")
            print(f"  전진/후진: {KEY_FORWARD}/{KEY_BACKWARD}")
            print(f"  좌/우: {KEY_LEFT}/{KEY_RIGHT}")
        print(f"  회전: {KEY_ROTATE_LEFT}/{KEY_ROTATE_RIGHT}")
        print(f"  정지: {KEY_STOP}")