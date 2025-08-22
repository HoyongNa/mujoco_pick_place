"""MobilityController - 리팩토링된 버전"""

import mujoco
from controllers.base.keyboard_handler import KeyboardHandler
from controllers.base.base_teleop import BaseTeleop
from controllers.arm.arm_holder import ArmHolder
from utils.thread_manager import ThreadManager

class MobilityController:
    """베이스 텔레옵 + 팔 자세 유지 + 물리 스텝 통합 컨트롤러"""
    
    def __init__(self, model, data, base_cmd_ref, base_lock, viewer=None):
        self.model = model
        self.data = data
        self.viewer = viewer
        
        # 공유 자원 (외부에서 전달받은 참조 사용)
        self.base_cmd_ref = base_cmd_ref  # 외부와 공유되는 명령값
        self.base_lock = base_lock
        
        # 구성 요소
        self.keyboard_handler = KeyboardHandler()
        self.base_teleop = BaseTeleop(data, base_cmd_ref, base_lock)
        self.arm_holder = ArmHolder(model, data)
        self.thread_manager = ThreadManager()
        
    def _loop(self):
        """메인 제어 루프"""
        # 초기화
        self.arm_holder.set_current_as_target()
        self.keyboard_handler.log_controls(robot_frame=True)  # 로봇 프레임 기준
        
        # 시작 시 현재 베이스 명령을 ctrl에 적용
        with self.base_lock:
            initial_cmd = self.base_cmd_ref.copy()
        self.data.ctrl[:3] = initial_cmd
        print(f"[MobilityController] 시작 베이스 위치: {initial_cmd}")
        print("[로봇 헤딩 기준 조종 활성화]")
        
        while not self.thread_manager.should_stop():
            # 뷰어 체크
            if self.viewer and not self.viewer.is_running():
                break
                
            # 1. 베이스 제어 (키보드 입력 반영 - 로봇 헤딩 기준)
            robot_heading = self.data.qpos[2]  # 현재 로봇의 헤딩 각도 (theta)
            with self.base_lock:
                cmd = self.keyboard_handler.update_command(self.base_cmd_ref, robot_heading)
            self.base_teleop.apply_command(cmd)
            
            # 2. 팔 홀드
            torque = self.arm_holder.compute_hold_torque()
            self.data.ctrl[3:10] = torque
            
            # 3. 물리 스텝
            mujoco.mj_step(self.model, self.data)
            
            # 4. 뷰어 동기화
            if self.viewer:
                self.viewer.sync()
                
        print("[MobilityController] 종료")
        
    def start(self):
        """컨트롤러 시작"""
        self.thread_manager.start(self._loop)
        
    def stop(self, timeout=1.0, zero_on_stop=True):
        """컨트롤러 정지
        
        Args:
            timeout: 스레드 종료 대기 시간
            zero_on_stop: True면 정지 시 베이스 명령을 0으로 설정
        """
        if zero_on_stop:
            # 베이스를 0으로 설정
            self.base_teleop.reset_command()
            self.data.ctrl[3:10] = 0.0
        else:
            # 현재 위치 유지
            with self.base_lock:
                final_cmd = self.base_cmd_ref.copy()
            print(f"[MobilityController] 종료 시 베이스 위치 유지: {final_cmd}")
            
        self.thread_manager.stop(timeout)