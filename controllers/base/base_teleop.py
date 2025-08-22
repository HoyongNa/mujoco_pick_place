"""베이스 텔레옵 제어"""

from config.constants import BASE_CTRL_SLICE

class BaseTeleop:
    """베이스 텔레옵 기능"""
    
    def __init__(self, data, base_cmd_ref, base_lock):
        self.data = data
        self.base_cmd_ref = base_cmd_ref
        self.base_lock = base_lock
        
    def apply_command(self, cmd):
        """베이스 명령을 ctrl에 적용"""
        self.data.ctrl[BASE_CTRL_SLICE] = cmd
        
    def reset_command(self):
        """베이스 명령 초기화"""
        with self.base_lock:
            self.base_cmd_ref[:] = 0.0
            self.data.ctrl[BASE_CTRL_SLICE] = 0.0