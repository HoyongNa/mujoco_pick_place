import numpy as np
import mujoco
import time
import threading

from ruckig import Ruckig, InputParameter, OutputParameter, Result


class ArmController:
    """
    - 팔: Ruckig로 궤적 추종, 토크 계산 → data.ctrl[joint_idx]에 반영
    - 베이스: 외부 주입 base_cmd_ref를 읽어 data.ctrl[:3]에 반영 (스레드는 mobility_controller가 담당)
    """
    def __init__(self, model, data, joint_idx, ctrl_idx, shared_gripper_ctrl,
                 use_dob=False, viewer=None,
                 base_cmd_ref=None, base_lock=None):
        self.model = model
        self.data = data
        self.joint_idx = joint_idx
        self.ctrl_idx = ctrl_idx
        self.shared_gripper_ctrl = shared_gripper_ctrl
        self.num_joints = len(joint_idx)
        self.viewer = viewer

        # ---- 외부 주입 베이스 명령 ----
        self.base_lock = base_lock or threading.RLock()
        if base_cmd_ref is None:
            # 기본: qpos[:3]을 복사해서 시작 (사용자 요청 반영)
            self.base_cmd_ref = np.copy(self.data.qpos[:3])
        else:
            self.base_cmd_ref = base_cmd_ref  # shape (3,)

        # 제어 게인
        self.Kp = np.eye(self.num_joints) * 1500
        self.Kd = np.eye(self.num_joints) * 30
        self.Ki = np.eye(self.num_joints) * 0
        self.Kii = np.eye(self.num_joints) * 0

        # 적분 항
        self.integral_error = np.zeros(self.num_joints)
        self.double_integral_error = np.zeros(self.num_joints)
        self.max_integral = 0.3
        self.max_double_integral = 0.1

        # DOB
        self.use_dob = use_dob
        self.dob_gain = 5.0
        self.qd_prev = np.zeros(self.num_joints)
        self.filtered_disturbance = np.zeros(self.num_joints)


    def update_viewer(self):
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def track_with_ruckig(self, target_q, ruckig_dt=0.002, max_step=100000):
        ruckig = Ruckig(self.num_joints, ruckig_dt)
        inp = InputParameter(self.num_joints)

        q_start = np.copy(self.data.qpos[self.joint_idx])
        inp.current_position = q_start.tolist()
        inp.current_velocity = [0.0] * self.num_joints
        inp.current_acceleration = [0.0] * self.num_joints
        inp.target_position = target_q.tolist()
        inp.target_velocity = [0.0] * self.num_joints
        inp.target_acceleration = [0.0] * self.num_joints
        inp.max_velocity = [3] * self.num_joints
        inp.max_acceleration = [7.0] * self.num_joints
        inp.max_jerk = [150.0] * self.num_joints

        out = OutputParameter(self.num_joints)
        result = Result.Working

        viewer_update_interval = 5
        last_print_time = time.time()
        print_interval = 1.0

        for step in range(max_step):
            q = self.data.qpos[self.joint_idx]
            qd = self.data.qvel[self.joint_idx]
            dt = self.model.opt.timestep

            if result != Result.Working:
                print(f" 목표 trajectory 종료. Step {step}")
                break

            result = ruckig.update(inp, out)
            q_des = np.array(out.new_position)
            qd_des = np.array(out.new_velocity)
            qdd_des = np.array(out.new_acceleration)

            pos_err = q_des - q
            vel_err = qd_des - qd

            self.integral_error += pos_err * dt
            self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
            self.double_integral_error += self.integral_error * dt
            self.double_integral_error = np.clip(self.double_integral_error, -self.max_double_integral, self.max_double_integral)

            # 동역학 항
            M_full = np.zeros((self.model.nv, self.model.nv))
            mujoco.mj_fullM(self.model, M_full, self.data.qM)
            M = M_full[np.ix_(self.joint_idx, self.joint_idx)]
            mujoco.mj_rnePostConstraint(self.model, self.data)
            bias = self.data.qfrc_bias[self.joint_idx]

            torque_nominal = M @ (
                qdd_des + self.Kp @ pos_err + self.Kd @ vel_err +
                self.Ki @ self.integral_error + self.Kii @ self.double_integral_error
            ) + bias

            if self.use_dob:
                qdd_est = (qd - self.qd_prev) / dt
                self.qd_prev = np.copy(qd)
                disturbance = torque_nominal - (M @ qdd_est + bias)
                self.filtered_disturbance += self.dob_gain * dt * (disturbance - self.filtered_disturbance)
                torque = torque_nominal - self.filtered_disturbance
            else:
                torque = torque_nominal

            torque = np.clip(torque, -1000, 1000)
            self.data.ctrl[self.joint_idx] = torque / 100.0

            # ---- 베이스 명령 반영 (외부 스레드에서 갱신됨) ----
            with self.base_lock:
                base_cmd = self.base_cmd_ref.copy()
            self.data.ctrl[:3] = base_cmd

            # 그리퍼
            self.data.ctrl[10] = self.shared_gripper_ctrl[0]

            # 단일 mj_step
            mujoco.mj_step(self.model, self.data)

            if step % viewer_update_interval == 0:
                self.update_viewer()

            if self.viewer is not None and time.time() - last_print_time > print_interval:
                error_norm = np.linalg.norm(target_q - q)
                print(f"  진행중... Step: {step}, Error: {error_norm:.4f}")
                last_print_time = time.time()

            inp.current_position = out.new_position
            inp.current_velocity = out.new_velocity
            inp.current_acceleration = out.new_acceleration

            if np.linalg.norm(target_q - q) < 0.001:
                print(f" 목표 도달! Step {step}, Error: {np.linalg.norm(target_q - q):.6f}")
                self.update_viewer()
                break

            if self.viewer is not None and not self.viewer.is_running():
                print(" 사용자가 시뮬레이션을 중단했습니다.")
                break
