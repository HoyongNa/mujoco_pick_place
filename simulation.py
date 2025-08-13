import time
import threading
from typing import Optional, Tuple, List

import numpy as np
import mujoco
import mujoco.viewer

from ik_solver import InverseKinematicsSolver
from arm_controller import ArmController
from grasp_checker import GraspChecker
from mobility_controller import MobilityController


class PickAndPlaceSimulation:
    """Pick & Place 시나리오를 간단하고 읽기 좋게 감싼 시뮬레이션 래퍼."""

    # ── 카메라 / 타이밍 상수 ----------------------------------------------------
    _CAM_AZIMUTH = 135
    _CAM_ELEVATION = -30
    _CAM_DISTANCE = 3.0
    _CAM_LOOKAT = np.array([0.3, 0.0, 0.3])

    _SETTLE_STEPS_GRASP = 10    # 파지 전/후 관성 정착용 스텝 수
    _SETTLE_STEPS_RELEASE = 25  # 릴리즈 후 정착 스텝 수
    _SETTLE_DT = 0.002          # 정착 단계에서의 sleep 간격

    # ───────────────────────────────────────────────────────────────────────────

    def __init__(self, model_path: str):
        # 모델/데이터
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 인덱스 및 공유 상태
        self.joint_idx = np.arange(3, 10)     # 팔 7DOF
        self.shared_gripper_ctrl = [0.0]      # 리스트로 참조 공유
        self.viewer: Optional[mujoco.viewer.Handle] = None

        # 주요 ID
        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "pinch_site")
        self.left_pad_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_pad")
        self.right_pad_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_pad")
        self.red_box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "obstacle1")
        self.blue_box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "obstacle2")

        # IK 경계 (joint_1..joint_7 범위 사용)
        arm_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(1, 8)]
        joint_ranges = self.model.jnt_range[arm_joint_ids]
        bounds = [(low, high) for (low, high) in joint_ranges]

        # 구성 요소
        self.ik_solver = InverseKinematicsSolver(self.model, self.data, self.joint_idx, bounds, self.ee_site_id)

        # Mobility 공유 버퍼/락 (qpos[:3]로 초기화)
        mujoco.mj_forward(self.model, self.data)  # 상태-유도 값 먼저 정합화
        self.base_lock = threading.RLock()
        self.base_cmd_ref = np.copy(self.data.qpos[:3])  # shape (3,)
        self.mobility: Optional[MobilityController] = None

        # 팔 홈 자세 저장
        self.arm_home_q = np.copy(self.data.qpos[self.joint_idx])
        print(f"초기 관절 각도: {self.data.qpos[self.joint_idx]}")

        # Arm/Grasp는 pick_and_place에서 생성
        self.arm_controller: Optional[ArmController] = None
        self.grasp_checker: Optional[GraspChecker] = None

    # ─────────────────────────────── Mobility ───────────────────────────────

    def start_mobility_control(self) -> None:
        """MobilityController를 항상 깨끗하게 재시작."""
        self.stop_mobility_control(zero_on_stop=False, wait=False)
        with self.base_lock:
            self.base_cmd_ref[:] = self.data.qpos[:3]
        self.mobility = MobilityController(
            self.model,
            self.data,
            base_cmd_ref=self.base_cmd_ref,
            base_lock=self.base_lock,
            viewer=self.viewer,
        )
        self.mobility.start()

    def stop_mobility_control(self, zero_on_stop: bool = False, wait: bool = False) -> None:
        """Mobility 정지(간단화: 대기 루프는 옵션)."""
        if self.mobility is not None:
            self.mobility.stop(zero_on_stop=zero_on_stop)
        self.mobility = None

    # ─────────────────────────────── Viewer ────────────────────────────────

    def initialize_viewer(self) -> None:
        """뷰어를 띄우고 Mobility를 시작."""
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.azimuth = self._CAM_AZIMUTH
            self.viewer.cam.elevation = self._CAM_ELEVATION
            self.viewer.cam.distance = self._CAM_DISTANCE
            self.viewer.cam.lookat = self._CAM_LOOKAT
            print("MuJoCo GUI viewer 초기화 완료")
        self.start_mobility_control()

    def close_viewer(self) -> None:
        """Mobility와 뷰어 정리."""
        self.stop_mobility_control(zero_on_stop=True)
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
            print("MuJoCo GUI viewer 종료")

    def _sync_viewer(self) -> None:
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    # ───────────────────────────── 기본 동작 ───────────────────────────────

    def move_to_pose(self, pos: np.ndarray, rpy: np.ndarray, gripper_state: float = 0.0) -> np.ndarray:
        """IK로 관절 목표를 구해 Ruckig 궤적으로 이동(ArmController가 mj_step 수행)."""
        q_opt = self.ik_solver.solve(pos, rpy)
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)  # 상태 갱신 후 유도 값 갱신
        self.shared_gripper_ctrl[0] = gripper_state
        self.arm_controller.track_with_ruckig(q_opt)
        return q_opt

    def return_arm_home(self, gripper_state: float = 0.0) -> None:
        """팔을 홈 관절각으로 복귀."""
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self.shared_gripper_ctrl[0] = gripper_state
        self.arm_controller.track_with_ruckig(self.arm_home_q)

    def _settle(self, steps: int) -> None:
        """간단한 정착 단계: 몇 스텝 돌리고 렌더를 갱신."""
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            self._sync_viewer()
            time.sleep(self._SETTLE_DT)

    def grasp_object(self) -> bool:
        print("\n 물체 파지 시도...")
        self.shared_gripper_ctrl[0] = 255
        self._settle(self._SETTLE_STEPS_GRASP)
        return self.grasp_checker.wait_until_grasped(threshold=0.05, max_time=3.0)

    def release_object(self) -> None:
        print("\n 물체 놓기...")
        self.shared_gripper_ctrl[0] = 0
        self._settle(self._SETTLE_STEPS_RELEASE)

    def get_red_box_position(self) -> np.ndarray:
        mujoco.mj_forward(self.model, self.data)
        return self.data.xpos[self.red_box_id]

    def get_blue_box_position(self) -> np.ndarray:
        mujoco.mj_forward(self.model, self.data)
        return self.data.xpos[self.blue_box_id]

    # ───────────────────────────── Pick & Place ─────────────────────────────

    def _waypoints(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """빨간 상자 → 파란 상자 이동 시나리오의 7개 웨이포인트 생성."""
        red = self.get_red_box_position()
        blue = self.get_blue_box_position()
        rpy_down = np.array([np.pi, 0, -np.pi / 2])

        wp = [
            (np.array([red[0],  red[1],  red[2]  + 0.15]), rpy_down),  # 접근(상)
            (np.array([red[0],  red[1],  red[2]  + 0.03]), rpy_down),  # 파지 높이
            (np.array([red[0],  red[1],  red[2]  + 0.25]), rpy_down),  # 상승
            (np.array([blue[0], blue[1], blue[2] + 0.25]), rpy_down),  # 이송 상공
            (np.array([blue[0], blue[1], blue[2] + 0.20]), rpy_down),  # 드롭 높이
            (np.array([blue[0], blue[1], blue[2] + 0.25]), rpy_down),  # 상승
            (np.array([0.3, 0.0, 0.5]), rpy_down),                     # 중앙 상공
        ]
        return wp

    def pick_and_place(self) -> bool:
        print("\n Pick & Place 작업 시작...")
        self.initialize_viewer()

        # 1) Mobility 정지(팔이 mj_step 소유)
        self.stop_mobility_control(zero_on_stop=False)

        # 2) 현재 베이스 자세 유지(목표=현재)
        with self.base_lock:
            self.base_cmd_ref[:] = self.data.qpos[:3]
            self.data.ctrl[:3] = self.base_cmd_ref

        # 3) Arm/Grasp 준비(항상 새로)
        self.arm_controller = ArmController(
            self.model, self.data, self.joint_idx,
            self.joint_idx, self.shared_gripper_ctrl,
            viewer=self.viewer,
        )
        self.grasp_checker = GraspChecker(
            self.model, self.data, self.ee_site_id,
            self.left_pad_body_id, self.right_pad_body_id,
            viewer=self.viewer,
        )

        # 4) 웨이포인트 실행
        wp = self._waypoints()
        try:
            # 픽업
            self.move_to_pose(*wp[0], gripper_state=0)
            self.move_to_pose(*wp[1], gripper_state=0)

            if not self.grasp_object():
                print(" 파지 실패. 작업 중단.")
                return False

            # 이동/드롭
            self.move_to_pose(*wp[2], gripper_state=255)
            self.move_to_pose(*wp[3], gripper_state=255)
            self.move_to_pose(*wp[4], gripper_state=255)
            self.release_object()

            # 복귀
            self.move_to_pose(*wp[5], gripper_state=0)
            self.move_to_pose(*wp[6], gripper_state=0)

            # 홈 복귀
            print("\n 팔을 홈 자세로 복귀합니다...")
            self.return_arm_home(gripper_state=0)

            print("\n Pick & Place 작업 완료!")

            # 5) Mobility 재개 후, 창이 닫힐 때까지 유지
            self.start_mobility_control()
            print("\n베이스 텔레옵 재개. (ESC로 창 닫기, Space는 main에서 처리)")
            while self.viewer is not None and self.viewer.is_running():
                time.sleep(0.01)

            return True

        finally:
            self.close_viewer()