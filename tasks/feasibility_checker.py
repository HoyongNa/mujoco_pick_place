"""Pick and Place 실행 가능성 체크"""

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

class FeasibilityChecker:
    """Pick and Place 작업의 실행 가능성을 체크하는 클래스"""
    
    def __init__(self, model, data, ik_solver, config):
        self.model = model
        self.data = data
        self.ik_solver = ik_solver
        self.config = config
        
        # 로봇 팔의 도달 범위 (tidybot 기준)
        # 베이스 중심으로부터의 거리
        self.MIN_REACH = 0.2  # 최소 도달 거리 (너무 가까우면 충돌)
        self.MAX_REACH = 0.8  # 최대 도달 거리 (팔 길이 한계)
        self.MAX_HEIGHT = 0.6  # 최대 도달 높이
        self.MIN_HEIGHT = -0.1  # 최소 도달 높이 (바닥 근처)
        
        # 안전 마진
        self.SAFETY_MARGIN = 0.05
        
    def check_pick_and_place_feasibility(self, pick_pos, place_pos):
        """Pick and Place 작업의 전체 실행 가능성 체크
        
        Args:
            pick_pos: 픽업할 물체의 위치
            place_pos: 놓을 위치
            
        Returns:
            tuple: (가능 여부, 메시지)
        """
        # 1. Pick 위치 체크
        pick_feasible, pick_msg = self.check_position_reachability(pick_pos, "Pick")
        if not pick_feasible:
            return False, pick_msg
            
        # 2. Place 위치 체크
        place_feasible, place_msg = self.check_position_reachability(place_pos, "Place")
        if not place_feasible:
            return False, place_msg
            
        # 3. IK 솔루션 체크 (실제 역기구학 해가 존재하는지)
        ik_feasible, ik_msg = self.check_ik_solutions(pick_pos, place_pos)
        if not ik_feasible:
            return False, ik_msg
            
        return True, "Pick & Place 작업 실행 가능"
    
    def check_position_reachability(self, target_pos, operation_name="작업"):
        """단일 위치의 도달 가능성 체크
        
        Args:
            target_pos: 목표 위치
            operation_name: 작업 이름 (디버깅용)
            
        Returns:
            tuple: (도달 가능 여부, 메시지)
        """
        # 현재 베이스 위치 가져오기
        base_pos = self.data.qpos[:2]  # x, y 위치
        
        # 베이스로부터 목표까지의 상대 위치
        rel_x = target_pos[0] - base_pos[0]
        rel_y = target_pos[1] - base_pos[1]
        rel_z = target_pos[2]
        
        # 수평 거리 계산
        horizontal_distance = np.sqrt(rel_x**2 + rel_y**2)
        
        # 거리 체크
        if horizontal_distance < self.MIN_REACH:
            return False, f"{operation_name} 위치가 너무 가까움 (거리: {horizontal_distance:.3f}m < {self.MIN_REACH}m)"
        
        if horizontal_distance > self.MAX_REACH:
            return False, f"{operation_name} 위치가 너무 멀음 (거리: {horizontal_distance:.3f}m > {self.MAX_REACH}m)"
        
        # 높이 체크
        if rel_z < self.MIN_HEIGHT:
            return False, f"{operation_name} 위치가 너무 낮음 (높이: {rel_z:.3f}m < {self.MIN_HEIGHT}m)"
        
        if rel_z > self.MAX_HEIGHT:
            return False, f"{operation_name} 위치가 너무 높음 (높이: {rel_z:.3f}m > {self.MAX_HEIGHT}m)"
        
        return True, f"{operation_name} 위치 도달 가능"
    
    def check_ik_solutions(self, pick_pos, place_pos):
        """IK 솔루션 존재 여부 체크
        
        Args:
            pick_pos: 픽업 위치
            place_pos: 놓을 위치
            
        Returns:
            tuple: (IK 해 존재 여부, 메시지)
        """
        # Pick 위치용 IK 테스트
        pick_rpy = [np.pi, 0, 0]  # 그리퍼 아래 향함
        pick_ik = self._test_ik_solution(pick_pos, pick_rpy)
        
        if not pick_ik['success']:
            return False, f"Pick 위치에 대한 IK 솔루션 없음 (에러: {pick_ik['error']:.4f})"
        
        # Place 위치용 IK 테스트
        place_rpy = [np.pi, 0, 0]  # 그리퍼 아래 향함
        place_ik = self._test_ik_solution(place_pos, place_rpy)
        
        if not place_ik['success']:
            return False, f"Place 위치에 대한 IK 솔루션 없음 (에러: {place_ik['error']:.4f})"
        
        # 관절 한계 체크
        if not self._check_joint_limits(pick_ik['solution']):
            return False, "Pick 위치의 IK 솔루션이 관절 한계를 벗어남"
            
        if not self._check_joint_limits(place_ik['solution']):
            return False, "Place 위치의 IK 솔루션이 관절 한계를 벗어남"
        
        return True, "IK 솔루션 존재"
    
    def _test_ik_solution(self, target_pos, target_rpy):
        """IK 솔루션 테스트
        
        Args:
            target_pos: 목표 위치
            target_rpy: 목표 회전 (roll, pitch, yaw)
            
        Returns:
            dict: {'success': bool, 'solution': array, 'error': float}
        """
        # 현재 상태 백업
        q_backup = np.copy(self.data.qpos)
        
        # IK 풀기 시도
        target_rot_mat = R.from_euler('xyz', target_rpy).as_matrix()
        
        def ik_cost(q):
            self.data.qpos[self.ik_solver.joint_idx] = q
            mujoco.mj_forward(self.model, self.data)
            ee_pos = self.data.site_xpos[self.ik_solver.ee_site_id]
            ee_rot = self.data.site_xmat[self.ik_solver.ee_site_id].reshape(3, 3)
            
            pos_error = np.linalg.norm(target_pos - ee_pos)
            rot_error = 0.5 * np.linalg.norm(ee_rot - target_rot_mat, ord='fro')
            return pos_error + rot_error
        
        # 최적화
        from scipy.optimize import minimize
        result = minimize(
            ik_cost,
            self.data.qpos[self.ik_solver.joint_idx],
            bounds=self.ik_solver.bounds,
            method='SLSQP',
            options={'ftol': 1e-4, 'maxiter': 100}
        )
        
        # 상태 복원
        self.data.qpos[:] = q_backup
        mujoco.mj_forward(self.model, self.data)
        
        # 결과 반환
        return {
            'success': result.success and result.fun < 0.05,  # 에러 임계값
            'solution': result.x if result.success else None,
            'error': result.fun
        }
    
    def _check_joint_limits(self, joint_values):
        """관절 한계 체크
        
        Args:
            joint_values: 관절 값 배열
            
        Returns:
            bool: 모든 관절이 한계 내에 있으면 True
        """
        if joint_values is None:
            return False
            
        for i, (val, (low, high)) in enumerate(zip(joint_values, self.ik_solver.bounds)):
            # 안전 마진 고려
            if val < low + self.SAFETY_MARGIN or val > high - self.SAFETY_MARGIN:
                return False
        return True
    
    def suggest_base_position(self, target_pos):
        """목표 위치에 도달하기 위한 적절한 베이스 위치 제안
        
        Args:
            target_pos: 목표 위치
            
        Returns:
            tuple: (제안된 베이스 위치, 메시지)
        """
        # 현재 베이스 위치
        current_base = self.data.qpos[:2]
        
        # 목표까지의 방향
        direction = target_pos[:2] - current_base
        distance = np.linalg.norm(direction)
        
        if distance < 0.001:
            return current_base, "베이스가 이미 적절한 위치에 있음"
        
        # 정규화된 방향
        direction_normalized = direction / distance
        
        # 최적 거리 (도달 범위의 중간)
        optimal_distance = (self.MIN_REACH + self.MAX_REACH) / 2
        
        # 제안된 위치
        suggested_pos = target_pos[:2] - direction_normalized * optimal_distance
        
        # 이동 거리
        move_distance = np.linalg.norm(suggested_pos - current_base)
        
        return suggested_pos, f"베이스를 ({suggested_pos[0]:.2f}, {suggested_pos[1]:.2f})로 이동 권장 (거리: {move_distance:.2f}m)"
