# ik_solver.py
# 역기구학(Inverse Kinematics) 해결 클래스

import numpy as np
import mujoco
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

class InverseKinematicsSolver:
    """
    목표 위치와 회전(roll-pitch-yaw)을 기반으로 MuJoCo 로봇 관절의 역기구학(Inverse Kinematics, IK)을 해결하는 클래스
    """
    def __init__(self, model, data, joint_idx, bounds, ee_site_id):
        self.model = model
        self.data = data
        self.joint_idx = joint_idx           # IK를 수행할 관절 인덱스 리스트
        self.bounds = bounds                 # 관절별 위치 제한 범위
        self.ee_site_id = ee_site_id         # 엔드이펙터 site ID

    def solve(self, target_pos, target_rpy):
        """
        목표 위치와 회전을 위한 관절 값을 계산

        Args:
            target_pos: 목표 위치 [x, y, z]
            target_rpy: 목표 회전 [roll, pitch, yaw]

        Returns:
            최적의 관절 값 또는 실패 시 현재 관절 값
        """
        target_rot_mat = R.from_euler('xyz', target_rpy).as_matrix()  # 목표 RPY를 회전행렬로 변환

        def ik_cost(q):  # 비용 함수 정의 (위치 + 회전 오차)
            q_backup = np.copy(self.data.qpos)  # 현재 관절값 백업
            self.data.qpos[self.joint_idx] = q  # q를 현재 관절값으로 설정
            mujoco.mj_forward(self.model, self.data)  # forward 계산으로 위치 및 회전 업데이트
            ee_pos = self.data.site_xpos[self.ee_site_id]  # 엔드이펙터 위치 추출
            ee_rot = self.data.site_xmat[self.ee_site_id].reshape(3, 3)  # 엔드이펙터 회전행렬 추출
            self.data.qpos[:] = q_backup  # 시뮬 상태 원복
            return np.linalg.norm(target_pos - ee_pos) + 0.5 * np.linalg.norm(ee_rot - target_rot_mat, ord='fro')
            # 위치 오차 + 회전 오차(Frobenius norm)를 비용으로 반환

        result = minimize(  # 최적화 수행 (SLSQP 방식)
            ik_cost,                                # 비용 함수
            np.copy(self.data.qpos[self.joint_idx]),  # 초기값: 현재 관절 위치
            bounds=self.bounds,                     # 관절 제한 적용
            method='SLSQP',                          # 제한조건 최적화 방법
            options={'ftol': 1e-6, 'maxiter': 500}   # 종료 조건 설정
        )
        return result.x if result.success else self.data.qpos[self.joint_idx]  # 성공 시 최적 관절값 반환, 실패 시 기존값 유지