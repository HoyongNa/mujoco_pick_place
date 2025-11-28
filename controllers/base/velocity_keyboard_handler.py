"""속도 제어용 키보드 핸들러"""

import numpy as np
from pynput import keyboard
import threading
import time
from config.constants import (
    KEY_FORWARD, KEY_BACKWARD, KEY_LEFT, KEY_RIGHT,
    KEY_ROTATE_LEFT, KEY_ROTATE_RIGHT, KEY_STOP,
    KEY_FORWARD_robot2, KEY_BACKWARD_robot2, KEY_LEFT_robot2, KEY_RIGHT_robot2,
    KEY_ROTATE_LEFT_robot2, KEY_ROTATE_RIGHT_robot2, KEY_STOP_robot2,
    BASE_CTRL_SLICE
)

class VelocityKeyboardHandler:
    """속도 제어를 위한 키보드 핸들러 (pynput 사용)

    BaseVelocityTeleop 기능이 통합됨:
    - 키보드 입력 처리
    - 속도 명령 생성
    - MuJoCo ctrl에 속도 적용
    """

    def __init__(self, data=None, linear_speed=0.5, angular_speed=0.5,
                 max_linear_vel=1.0, max_angular_vel=1.0,
                 robot_id=1, base_ctrl_slice=None):
        """
        Args:
            data: MuJoCo data (속도 적용을 위해 필요)
            linear_speed: 기본 선속도 (m/s)
            angular_speed: 기본 각속도 (rad/s)
            max_linear_vel: 최대 선속도 (m/s)
            max_angular_vel: 최대 각속도 (rad/s)
            robot_id: 로봇 번호 (1 or 2)
            base_ctrl_slice: 베이스 제어 슬라이스 (None이면 robot1 기본값 사용)
        """
        self.data = data
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.robot_id = robot_id
        self.base_ctrl_slice = base_ctrl_slice if base_ctrl_slice is not None else BASE_CTRL_SLICE
        
        # 로봇별 키 매핑 설정
        if robot_id == 1:
            self.key_forward = KEY_FORWARD
            self.key_backward = KEY_BACKWARD
            self.key_left = KEY_LEFT
            self.key_right = KEY_RIGHT
            self.key_rotate_left = KEY_ROTATE_LEFT
            self.key_rotate_right = KEY_ROTATE_RIGHT
            self.key_stop = KEY_STOP
        else:  # robot_id == 2
            self.key_forward = KEY_FORWARD_robot2
            self.key_backward = KEY_BACKWARD_robot2
            self.key_left = KEY_LEFT_robot2
            self.key_right = KEY_RIGHT_robot2
            self.key_rotate_left = KEY_ROTATE_LEFT_robot2
            self.key_rotate_right = KEY_ROTATE_RIGHT_robot2
            self.key_stop = KEY_STOP_robot2
        
        # 현재 눌려있는 키 상태
        self.keys_pressed = set()
        self.lock = threading.Lock()
        
        # pynput 리스너
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        
        # 속도 명령 초기화
        self.vel_cmd = np.zeros(3)  # [vx, vy, vtheta]
        
    def _on_press(self, key):
        """키 누름 이벤트 처리"""
        try:
            # 일반 키
            if hasattr(key, 'char') and key.char:
                with self.lock:
                    self.keys_pressed.add(key.char)
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
                        with self.lock:
                            self.keys_pressed.add(numpad_map[vk])
        except:
            pass
            
    def _on_release(self, key):
        """키 뗌 이벤트 처리"""
        try:
            # 일반 키
            if hasattr(key, 'char') and key.char:
                with self.lock:
                    self.keys_pressed.discard(key.char)
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
                        with self.lock:
                            self.keys_pressed.discard(numpad_map[vk])
        except:
            pass
            
    def get_velocity_command(self, robot_heading=0.0):
        """현재 키 상태에 따른 속도 명령 반환
        
        Args:
            robot_heading: 로봇 헤딩 각도 (로봇 프레임 기준 제어시 사용)
            
        Returns:
            [vx, vy, vtheta] 속도 명령
        """
        with self.lock:
            # 속도 명령 초기화
            vx_robot = 0.0
            vy_robot = 0.0
            vtheta = 0.0
            
            # 선속도 제어 (로봇 프레임 기준) - 동적 키 매핑 사용
            if self.key_forward in self.keys_pressed:
                vx_robot = self.linear_speed  # 전진
            elif self.key_backward in self.keys_pressed:
                vx_robot = -self.linear_speed  # 후진
                
            if self.key_left in self.keys_pressed:
                vy_robot = self.linear_speed  # 좌측 이동
            elif self.key_right in self.keys_pressed:
                vy_robot = -self.linear_speed  # 우측 이동
                
            # 각속도 제어
            if self.key_rotate_left in self.keys_pressed:
                vtheta = self.angular_speed  # 반시계 회전
            elif self.key_rotate_right in self.keys_pressed:
                vtheta = -self.angular_speed  # 시계 회전
            
            # 정지 키
            if self.key_stop in self.keys_pressed:
                vx_robot = 0.0
                vy_robot = 0.0
                vtheta = 0.0
                
            # 로봇 프레임에서 월드 프레임으로 변환
            cos_h = np.cos(robot_heading)
            sin_h = np.sin(robot_heading)
            
            vx_world = cos_h * vx_robot - sin_h * vy_robot
            vy_world = sin_h * vx_robot + cos_h * vy_robot
            
            self.vel_cmd[0] = vx_world
            self.vel_cmd[1] = vy_world
            self.vel_cmd[2] = vtheta
            
            return self.vel_cmd.copy()

    def apply_velocity(self, vel_cmd):
        """베이스 속도 명령을 ctrl에 적용 (BaseVelocityTeleop 통합 기능)

        Args:
            vel_cmd: [vx, vy, vtheta] 속도 명령
        """
        if self.data is None:
            return

        # 속도 제한 적용
        vel_cmd = np.copy(vel_cmd)
        vel_cmd[0] = np.clip(vel_cmd[0], -self.max_linear_vel, self.max_linear_vel)
        vel_cmd[1] = np.clip(vel_cmd[1], -self.max_linear_vel, self.max_linear_vel)
        vel_cmd[2] = np.clip(vel_cmd[2], -self.max_angular_vel, self.max_angular_vel)

        # MuJoCo ctrl에 속도 명령 적용
        self.data.ctrl[self.base_ctrl_slice] = vel_cmd

    def update(self, robot_heading=0.0):
        """키보드 입력을 읽고 속도를 자동으로 적용

        Args:
            robot_heading: 로봇 헤딩 각도 (로봇 프레임 기준 제어시 사용)
        """
        vel_cmd = self.get_velocity_command(robot_heading)
        self.apply_velocity(vel_cmd)

    def is_key_pressed(self, key):
        """특정 키가 눌려 있는지 확인
        
        Args:
            key: 확인할 키 (문자열)
            
        Returns:
            bool: 키가 눌려 있으면 True
        """
        with self.lock:
            return key in self.keys_pressed
    
    def stop_velocity(self):
        """베이스 정지 - 속도를 0으로 설정 (BaseVelocityTeleop 통합 기능)"""
        if self.data is not None:
            self.data.ctrl[self.base_ctrl_slice] = 0.0

    def stop(self):
        """키보드 리스너 정지 및 베이스 정지"""
        self.stop_velocity()
        if hasattr(self, 'listener'):
            self.listener.stop()
            
    def log_controls(self):
        """제어 키 안내 출력"""
        robot_name = f"Robot {self.robot_id}"
        print(f"\n=== {robot_name} 속도 제어 모드 ===")
        print(f"{self.key_forward}/{self.key_backward}: 전진/후진")
        print(f"{self.key_left}/{self.key_right}: 좌/우 이동")
        print(f"{self.key_rotate_left}/{self.key_rotate_right}: 좌/우 회전")
        print(f"{self.key_stop}: 정지")
        print("키를 떼면 자동으로 정지합니다")
        print("=====================\n")
        