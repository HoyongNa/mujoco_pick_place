"""ArmController - 리팩토링된 버전"""

import numpy as np
import time
import threading
import mujoco
from ruckig import Result
from controllers.arm.trajectory_tracker import TrajectoryTracker
from controllers.arm.torque_controller import TorqueController

class ArmController:
    """팔 제어 통합 클래스"""
    
    def __init__(self, model, data, joint_idx, ctrl_idx, 
                 shared_gripper_ctrl, use_dob=False, viewer=None,
                 base_cmd_ref=None, base_lock=None):
        self.model = model
        self.data = data
        self.joint_idx = joint_idx
        self.ctrl_idx = ctrl_idx
        self.shared_gripper_ctrl = shared_gripper_ctrl
        self.viewer = viewer
        
        # 구성 요소
        self.trajectory_tracker = TrajectoryTracker(model, data, joint_idx)
        self.torque_controller = TorqueController(model, data, joint_idx)
        self.torque_controller.use_dob = use_dob
        
        # 베이스 명령 (옵션)
        self.base_lock = base_lock if base_lock is not None else threading.RLock()
        self.base_cmd_ref = base_cmd_ref if base_cmd_ref is not None else np.copy(data.qpos[:3])
        
    def track_with_ruckig(self, target_q, max_step=100000):
        """Ruckig 궤적 추종"""
        # 궤적 생성
        ruckig, inp, out = self.trajectory_tracker.create_trajectory(target_q)
        result = Result.Working
        
        viewer_update_interval = 5
        last_print_time = time.time()
        print_interval = 1.0
        
        for step in range(max_step):
            if result != Result.Working:
                print(f" 목표 trajectory 종료. Step {step}")
                break
                
            # Ruckig 업데이트
            result = ruckig.update(inp, out)
            q_des = np.array(out.new_position)
            qd_des = np.array(out.new_velocity)
            qdd_des = np.array(out.new_acceleration)
            
            # 토크 계산
            torque = self.torque_controller.compute_torque(q_des, qd_des, qdd_des)
            self.data.ctrl[self.ctrl_idx] = torque
            
            # 베이스 명령
            with self.base_lock:
                self.data.ctrl[:3] = self.base_cmd_ref.copy()
                
            # 그리퍼
            self.data.ctrl[10] = self.shared_gripper_ctrl[0]
            
            # 물리 스텝
            mujoco.mj_step(self.model, self.data)
            
            # 뷰어 업데이트
            if step % viewer_update_interval == 0 and self.viewer:
                self.viewer.sync()
                
            # 진행 상황 출력
            if self.viewer and time.time() - last_print_time > print_interval:
                q = self.data.qpos[self.joint_idx]
                error_norm = np.linalg.norm(target_q - q)
                print(f"  진행중... Step: {step}, Error: {error_norm:.4f}")
                last_print_time = time.time()
                
            # Ruckig 상태 업데이트
            inp.current_position = out.new_position
            inp.current_velocity = out.new_velocity
            inp.current_acceleration = out.new_acceleration
            
            # 목표 도달 확인
            q = self.data.qpos[self.joint_idx]
            if np.linalg.norm(target_q - q) < 0.001:
                print(f" 목표 도달! Step {step}, Error: {np.linalg.norm(target_q - q):.6f}")
                if self.viewer:
                    self.viewer.sync()
                break
                
            # 뷰어 종료 확인
            if self.viewer and not self.viewer.is_running():
                print(" 사용자가 시뮬레이션을 중단했습니다.")
                break