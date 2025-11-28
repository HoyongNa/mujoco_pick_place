"""
ACADOS MPC Wrapper for MuJoCo Integration
런타임에서 사용할 MPC 컨트롤러 클래스
"""
import os
import sys
import numpy as np

# ACADOS 환경 설정
ACADOS_SOURCE_DIR = os.path.expanduser('~/acados')
os.environ['ACADOS_SOURCE_DIR'] = ACADOS_SOURCE_DIR
os.environ['LD_LIBRARY_PATH'] = f"{ACADOS_SOURCE_DIR}/lib:" + os.environ.get('LD_LIBRARY_PATH', '')
sys.path.insert(0, os.path.join(ACADOS_SOURCE_DIR, 'interfaces', 'acados_template'))

from acados_template import AcadosOcpSolver


class MPCController:
    """MuJoCo 로봇용 MPC 컨트롤러"""
    
    def __init__(self, json_file='robot_mpc.json'):
        """
        Args:
            json_file: ACADOS OCP 설정 파일
        """
        self.solver = AcadosOcpSolver.create_cython_solver(json_file)
        self.N = self.solver.acados_ocp.solver_options.N_horizon
        
        # 상태/제어 차원
        self.nx = 3  # [x, y, theta]
        self.nu = 2  # [v, omega]
        
        # 목표 위치
        self.goal = np.zeros(3)
        
        print(f"✓ MPC Controller 초기화 완료 (N={self.N})")
    
    def set_goal(self, goal_x, goal_y, goal_theta=0.0):
        """목표 위치 설정"""
        self.goal = np.array([goal_x, goal_y, goal_theta])
        
        # 모든 stage에 reference 설정
        yref = np.array([goal_x, goal_y, goal_theta, 0.0, 0.0])
        yref_e = np.array([goal_x, goal_y, goal_theta])
        
        for i in range(self.N):
            self.solver.set(i, 'yref', yref)
        self.solver.set(self.N, 'yref', yref_e)
    
    def compute_control(self, current_state):
        """
        현재 상태에서 최적 제어 입력 계산
        
        Args:
            current_state: [x, y, theta] 현재 로봇 상태
            
        Returns:
            control: [v, omega] 최적 제어 입력
            status: solver 상태 (0 = 성공)
        """
        # 초기 상태 설정
        self.solver.set(0, 'lbx', current_state)
        self.solver.set(0, 'ubx', current_state)
        
        # MPC 풀기
        status = self.solver.solve()
        
        # 최적 제어 입력 추출
        control = self.solver.get(0, 'u')
        
        return control, status
    
    def get_predicted_trajectory(self):
        """예측된 전체 경로 반환 (시각화용)"""
        trajectory = np.zeros((self.N + 1, self.nx))
        for i in range(self.N + 1):
            trajectory[i] = self.solver.get(i, 'x')
        return trajectory
    
    def get_distance_to_goal(self, current_state):
        """목표까지의 거리 계산"""
        return np.linalg.norm(current_state[:2] - self.goal[:2])
    
    def is_goal_reached(self, current_state, threshold=0.1):
        """목표 도달 여부 확인"""
        return self.get_distance_to_goal(current_state) < threshold


class MPCNavigator:
    """A* + MPC 통합 네비게이터"""
    
    def __init__(self, mpc_controller):
        self.mpc = mpc_controller
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.3  # waypoint 도달 판정 거리
    
    def set_path(self, waypoints):
        """
        A* 등에서 생성된 경로 설정
        
        Args:
            waypoints: [[x1,y1], [x2,y2], ...] 형태의 경로점 리스트
        """
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        
        if len(waypoints) > 0:
            self.mpc.set_goal(waypoints[0][0], waypoints[0][1])
    
    def update(self, current_state):
        """
        현재 상태에서 제어 입력 계산 및 waypoint 업데이트
        
        Args:
            current_state: [x, y, theta]
            
        Returns:
            control: [v, omega]
            done: 모든 waypoint 도달 여부
        """
        if self.current_waypoint_idx >= len(self.waypoints):
            return np.zeros(2), True
        
        # 현재 waypoint까지의 거리 확인
        wp = self.waypoints[self.current_waypoint_idx]
        dist = np.linalg.norm(current_state[:2] - np.array(wp[:2]))
        
        # Waypoint 도달 시 다음으로 이동
        if dist < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx < len(self.waypoints):
                next_wp = self.waypoints[self.current_waypoint_idx]
                self.mpc.set_goal(next_wp[0], next_wp[1])
            else:
                return np.zeros(2), True
        
        # MPC 제어 계산
        control, status = self.mpc.compute_control(current_state)
        
        return control, False


# 테스트 코드
if __name__ == '__main__':
    print("MPC Controller 테스트...")
    
    # 컨트롤러 생성
    mpc = MPCController()
    
    # 목표 설정
    mpc.set_goal(2.0, 3.0, 0.0)
    
    # 시뮬레이션 테스트
    state = np.array([0.0, 0.0, 0.0])
    
    print(f"초기 상태: {state}")
    print(f"목표: {mpc.goal}")
    
    for i in range(10):
        control, status = mpc.compute_control(state)
        print(f"Step {i}: state={state}, control={control}, status={status}")
        
        # 간단한 상태 업데이트 (dt=0.1)
        dt = 0.1
        state[0] += control[0] * np.cos(state[2]) * dt
        state[1] += control[0] * np.sin(state[2]) * dt
        state[2] += control[1] * dt
    
    print(f"\n최종 상태: {state}")
    print(f"목표까지 거리: {mpc.get_distance_to_goal(state):.3f}m")
