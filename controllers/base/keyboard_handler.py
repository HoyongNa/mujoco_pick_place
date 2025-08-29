"""키보드 입력 처리 (pynput 사용 - 크로스 플랫폼)"""

from pynput import keyboard
import numpy as np
from config.constants import *
import threading


class KeyboardHandler:
    """키보드 입력을 베이스 명령으로 변환 (pynput 기반)"""
    
    def __init__(self):
        # 키 매핑 (pynput 키 형식)
        self.key_map = {
            KEY_FORWARD: (0, BASE_LIN_STEP),
            KEY_BACKWARD: (0, -BASE_LIN_STEP),
            KEY_LEFT: (1, BASE_LIN_STEP),
            KEY_RIGHT: (1, -BASE_LIN_STEP),
            KEY_ROTATE_LEFT: (2, BASE_YAW_STEP),
            KEY_ROTATE_RIGHT: (2, -BASE_YAW_STEP),
        }
        
        # 현재 눌린 키들을 추적
        self.pressed_keys = set()
        self._lock = threading.Lock()
        
        # 키보드 리스너 시작
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        
    def _on_press(self, key):
        """키 눌림 이벤트 처리"""
        try:
            # 일반 키
            if hasattr(key, 'char') and key.char:
                with self._lock:
                    self.pressed_keys.add(key.char)
            # 특수 키 (숫자패드 등)
            elif hasattr(key, 'value'):
                # pynput에서 숫자패드 키 처리
                if hasattr(key.value, 'vk'):
                    # Windows 가상 키코드 기반
                    vk = key.value.vk
                    # 숫자패드 매핑 (Windows VK codes)
                    numpad_map = {
                        96: '0',   # VK_NUMPAD0
                        97: '1',   # VK_NUMPAD1
                        98: '2',   # VK_NUMPAD2
                        99: '3',   # VK_NUMPAD3
                        100: '4',  # VK_NUMPAD4
                        101: '5',  # VK_NUMPAD5
                        102: '6',  # VK_NUMPAD6
                        103: '7',  # VK_NUMPAD7
                        104: '8',  # VK_NUMPAD8
                        105: '9',  # VK_NUMPAD9
                    }
                    if vk in numpad_map:
                        with self._lock:
                            self.pressed_keys.add(numpad_map[vk])
                # Mac/Linux 처리
                elif hasattr(key.value, 'char'):
                    with self._lock:
                        self.pressed_keys.add(key.value.char)
        except AttributeError:
            pass
            
    def _on_release(self, key):
        """키 릴리즈 이벤트 처리"""
        try:
            # 일반 키
            if hasattr(key, 'char') and key.char:
                with self._lock:
                    self.pressed_keys.discard(key.char)
            # 특수 키 (숫자패드 등)
            elif hasattr(key, 'value'):
                # Windows 가상 키코드 기반
                if hasattr(key.value, 'vk'):
                    vk = key.value.vk
                    numpad_map = {
                        96: '0',   # VK_NUMPAD0
                        97: '1',   # VK_NUMPAD1
                        98: '2',   # VK_NUMPAD2
                        99: '3',   # VK_NUMPAD3
                        100: '4',  # VK_NUMPAD4
                        101: '5',  # VK_NUMPAD5
                        102: '6',  # VK_NUMPAD6
                        103: '7',  # VK_NUMPAD7
                        104: '8',  # VK_NUMPAD8
                        105: '9',  # VK_NUMPAD9
                    }
                    if vk in numpad_map:
                        with self._lock:
                            self.pressed_keys.discard(numpad_map[vk])
                # Mac/Linux 처리
                elif hasattr(key.value, 'char'):
                    with self._lock:
                        self.pressed_keys.discard(key.value.char)
        except AttributeError:
            pass
    
    def is_pressed(self, key):
        """특정 키가 눌렸는지 확인"""
        with self._lock:
            return key in self.pressed_keys
    
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
            if self.is_pressed(key):
                cmd_ref[idx] += delta
                
        if self.is_pressed(KEY_STOP):
            cmd_ref[:] = 0.0
            
        return cmd_ref.copy()
    
    def _update_command_robot_frame(self, cmd_ref, robot_heading):
        """로봇 좌표계 기준 명령 업데이트 (헤딩 기준)"""
        # 로봇 좌표계에서의 속도 명령
        robot_vx = 0.0  # 전진/후진
        robot_vy = 0.0  # 좌/우
        robot_vtheta = 0.0  # 회전
        
        # 키 입력 처리 (로봇 좌표계)
        if self.is_pressed(KEY_FORWARD):
            robot_vx += BASE_LIN_STEP
        if self.is_pressed(KEY_BACKWARD):
            robot_vx -= BASE_LIN_STEP
        if self.is_pressed(KEY_LEFT):
            robot_vy += BASE_LIN_STEP
        if self.is_pressed(KEY_RIGHT):
            robot_vy -= BASE_LIN_STEP
        if self.is_pressed(KEY_ROTATE_LEFT):
            robot_vtheta += BASE_YAW_STEP
        if self.is_pressed(KEY_ROTATE_RIGHT):
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
        
        if self.is_pressed(KEY_STOP):
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
    
    def stop(self):
        """키보드 리스너 정지"""
        if hasattr(self, 'listener'):
            self.listener.stop()
    
    def __del__(self):
        """소멸자 - 리스너 정리"""
        self.stop()
