"""
MobilityController
- 최소 인자: model, data, base_cmd_ref, base_lock, viewer(optional)
- 가정:
  • 베이스 제어는 data.ctrl[0:3]
  • 팔 관절/제어 인덱스는 3..9 (7 DOF)
  • gravity_comp = True (항상)
  • 키보드는 항상 사용 가능
  • Ruckig/PD/토크/샘플링 파라미터는 내부 상수 사용
- 역할:
  (1) 베이스 텔레옵, (2) 팔 자세 유지(항상 ON), (3) mj_step + viewer.sync
"""

from __future__ import annotations

import time
import threading
from typing import Optional

import numpy as np
import mujoco
import keyboard  # 설치되어 있다고 가정
from ruckig import Ruckig, InputParameter, OutputParameter, Result


class MobilityController:
    """베이스 텔레옵 + 팔 자세 유지(홀드) + 물리 스텝을 담당하는 컨트롤 루프."""

    # ---- 고정 파라미터(모델 전제) -------------------------------------------------
    _BASE_CTRL_SLICE = slice(0, 3)          # 베이스가 쓰는 data.ctrl 범위
    _ARM_Q_IDX = np.arange(3, 10)           # 팔 qpos/qvel 인덱스
    _ARM_CTRL_IDX = np.arange(3, 10)        # 팔 ctrl 인덱스 (actuator 매핑 가정)

    # 키 입력 증분
    _LIN_STEP = 0.0008   # 8/5, 4/6
    _YAW_STEP = 0.0010   # 7/9

    # 팔 홀드 제어 파라미터
    _KP_HOLD = 400.0
    _KD_HOLD = 25.0
    _TORQUE_SCALE = 0.01
    _TORQUE_LIMIT = 800.0

    # Ruckig 제한
    _MAX_V = 3.0
    _MAX_A = 7.0
    _MAX_J = 150.0

    # -----------------------------------------------------------------------------

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 base_cmd_ref: np.ndarray, base_lock: threading.RLock,
                 viewer: Optional[object] = None) -> None:
        assert base_cmd_ref.shape == (3,), "base_cmd_ref must have shape (3,)."

        # 핸들/공유자원
        self.model = model
        self.data = data
        self.base_cmd_ref = base_cmd_ref    # 베이스 목표 공유 버퍼
        self.base_lock = base_lock          # 공유 버퍼 락
        self.viewer = viewer

        # 주기(물리 timestep)
        self._dt = float(self.model.opt.timestep)
        self._period = self._dt

        # Ruckig/홀드 상태
        self._num_arm = len(self._ARM_Q_IDX)
        self._ruckig: Optional[Ruckig] = None
        self._rinp: Optional[InputParameter] = None
        self._rout: Optional[OutputParameter] = None
        self._hold_q: Optional[np.ndarray] = None  # 목표 관절각(없으면 현재 각도)

        # 스레드 제어
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ─────────────────────────────── 내부 유틸 ───────────────────────────────

    def _log_once_on_start(self) -> None:
        print("[MobilityController] 키: 8/5 전후, 4/6 좌우, 7/9 회전, 2 정지")

    def _maybe_sync_viewer(self) -> None:
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    # ─────────────────────────────── Ruckig / 홀드 ───────────────────────────────

    def _init_ruckig_to_target(self, q_target: np.ndarray) -> None:
        """현재 상태에서 q_target으로 부드럽게 이동하도록 Ruckig 세팅."""
        q_now = np.copy(self.data.qpos[self._ARM_Q_IDX])

        self._ruckig = Ruckig(self._num_arm, self._dt)
        self._rinp = InputParameter(self._num_arm)
        self._rout = OutputParameter(self._num_arm)

        self._rinp.current_position = q_now.tolist()
        self._rinp.current_velocity = [0.0] * self._num_arm
        self._rinp.current_acceleration = [0.0] * self._num_arm

        q_target = np.asarray(q_target, dtype=float)
        self._rinp.target_position = q_target.tolist()
        self._rinp.target_velocity = [0.0] * self._num_arm
        self._rinp.target_acceleration = [0.0] * self._num_arm

        self._rinp.max_velocity = [self._MAX_V] * self._num_arm
        self._rinp.max_acceleration = [self._MAX_A] * self._num_arm
        self._rinp.max_jerk = [self._MAX_J] * self._num_arm

    def _set_hold_to_current(self) -> None:
        """현재 팔 관절각을 그대로 홀드 목표로 고정."""
        self._hold_q = np.copy(self.data.qpos[self._ARM_Q_IDX])
        self._init_ruckig_to_target(self._hold_q)

    def _arm_hold_step(self) -> None:
        """Ruckig 참조를 따라 팔을 토크 제어로 유지."""
        if self._ruckig is None or self._rinp is None or self._rout is None:
            return  # 아직 준비 전

        # 1) Ruckig 진행
        result = self._ruckig.update(self._rinp, self._rout)

        # 2) 현재 상태
        q  = self.data.qpos[self._ARM_Q_IDX]
        qd = self.data.qvel[self._ARM_Q_IDX]

        # 3) 참조
        q_des   = np.array(self._rout.new_position)
        qd_des  = np.array(self._rout.new_velocity)
        qdd_des = np.array(self._rout.new_acceleration)

        # 4) 오차
        pos_err = q_des - q
        vel_err = qd_des - qd

        # 5) 동역학 항(M, bias)
        M_full = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, M_full, self.data.qM)
        M = M_full[np.ix_(self._ARM_Q_IDX, self._ARM_Q_IDX)]

        # gravity/바이어스(항상 사용)
        mujoco.mj_rnePostConstraint(self.model, self.data)
        bias = self.data.qfrc_bias[self._ARM_Q_IDX]

        # 6) 토크 계산: M*(qdd_des + Kp*e + Kd*e_dot) + bias
        torque = M @ (qdd_des + self._KP_HOLD * pos_err + self._KD_HOLD * vel_err) + bias
        torque = np.clip(torque, -self._TORQUE_LIMIT, self._TORQUE_LIMIT)
        self.data.ctrl[self._ARM_CTRL_IDX] = torque * self._TORQUE_SCALE

        # 7) Ruckig 상태 갱신
        self._rinp.current_position = self._rout.new_position
        self._rinp.current_velocity = self._rout.new_velocity
        self._rinp.current_acceleration = self._rout.new_acceleration

        # 8) 목표점에 도달했으면 현재를 타겟으로 다시 고정(잔진동 억제)
        if result != Result.Working:
            self._set_hold_to_current()

    # ─────────────────────────────── 입력 / 베이스 ───────────────────────────────

    def _update_base_from_keys(self) -> np.ndarray:
        """키 입력을 읽어 base_cmd_ref를 갱신하고 복사본을 반환."""
        if keyboard.is_pressed('8'):
            self.base_cmd_ref[0] += self._LIN_STEP
        if keyboard.is_pressed('5'):
            self.base_cmd_ref[0] -= self._LIN_STEP
        if keyboard.is_pressed('4'):
            self.base_cmd_ref[1] += self._LIN_STEP
        if keyboard.is_pressed('6'):
            self.base_cmd_ref[1] -= self._LIN_STEP
        if keyboard.is_pressed('7'):
            self.base_cmd_ref[2] += self._YAW_STEP
        if keyboard.is_pressed('9'):
            self.base_cmd_ref[2] -= self._YAW_STEP
        if keyboard.is_pressed('2'):
            self.base_cmd_ref[:] = 0.0
        return self.base_cmd_ref.copy()

    # ─────────────────────────────── 메인 루프 ───────────────────────────────

    def _loop(self) -> None:
        # 홀드 목표 초기화(항상 ON)
        if self._hold_q is None:
            self._set_hold_to_current()
        else:
            self._init_ruckig_to_target(self._hold_q)

        self._log_once_on_start()
        next_t = time.perf_counter()

        while not self._stop.is_set():
            # 뷰어 종료 시 안전 탈출
            if self.viewer is not None and not self.viewer.is_running():
                break

            # 1) 베이스 명령(키 입력 → 공유 버퍼 → ctrl 반영)
            with self.base_lock:
                base_cmd = self._update_base_from_keys()
            self.data.ctrl[self._BASE_CTRL_SLICE] = base_cmd

            # 2) 팔 홀드(토크 제어)
            self._arm_hold_step()

            # 3) 물리 스텝 + 렌더 동기화
            mujoco.mj_step(self.model, self.data)
            self._maybe_sync_viewer()

            # 4) 주기 유지
            next_t += self._period
            sleep_dt = next_t - time.perf_counter()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                # 프레임 드롭 시 기준 시각 재설정(스파이크 누적 방지)
                next_t = time.perf_counter()

        print("[MobilityController] 종료")

    # ─────────────────────────────── 수명주기 ───────────────────────────────

    def start(self) -> None:
        """컨트롤 루프 시작(백그라운드 스레드)."""
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 1.0, zero_on_stop: bool = True) -> None:
        """컨트롤 루프 정지. 필요 시 ctrl을 0으로 초기화하고 Ruckig 상태를 정리."""
        if self._thread is not None and self._thread.is_alive():
            if zero_on_stop:
                with self.base_lock:
                    self.base_cmd_ref[:] = 0.0
                    self.data.ctrl[self._BASE_CTRL_SLICE] = 0.0
                # 팔 토크 0
                self.data.ctrl[self._ARM_CTRL_IDX] = 0.0

            self._stop.set()
            self._thread.join(timeout=timeout)

        # 다음 start를 위해 Ruckig 상태 정리
        self._ruckig = None
        self._rinp = None
        self._rout = None
        self._hold_q = None
        self._thread = None