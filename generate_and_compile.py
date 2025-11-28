"""
ACADOS MPC 생성 및 컴파일 스크립트 - Linux/MuJoCo 통합 버전
Ubuntu + Anaconda 환경용
"""
import os
import sys
import subprocess
import shutil
import numpy as np

# ============================================================================
# 환경 설정 (Linux/Ubuntu용)
# ============================================================================
ACADOS_SOURCE_DIR = os.path.expanduser('~/acados')

# 환경 변수 설정
os.environ['ACADOS_SOURCE_DIR'] = ACADOS_SOURCE_DIR
os.environ['LD_LIBRARY_PATH'] = f"{ACADOS_SOURCE_DIR}/lib:" + os.environ.get('LD_LIBRARY_PATH', '')

# Python 경로에 acados_template 추가
sys.path.insert(0, os.path.join(ACADOS_SOURCE_DIR, 'interfaces', 'acados_template'))

# OpenMP 스레드 설정 (CPU 코어 수에 맞게 조정)
os.environ['OMP_NUM_THREADS'] = '4'

print("="*70)
print("ACADOS MPC 생성기 - Linux/MuJoCo 통합 버전")
print("="*70)
print(f"ACADOS 경로: {ACADOS_SOURCE_DIR}")
print(f"OpenMP 스레드: {os.environ['OMP_NUM_THREADS']}")
print("="*70 + "\n")

# ============================================================================
# 기존 파일 정리
# ============================================================================
def clean_previous_build():
    """기존 빌드 파일 삭제"""
    if os.path.exists('c_generated_code'):
        shutil.rmtree('c_generated_code')
        print("✓ 기존 c_generated_code 삭제")
    
    if os.path.exists('robot_mpc.json'):
        os.remove('robot_mpc.json')
        print("✓ 기존 robot_mpc.json 삭제")
    
    # 기존 .so 파일 삭제
    for f in os.listdir('.'):
        if f.endswith('.so') and 'acados' in f:
            os.remove(f)
            print(f"✓ 기존 {f} 삭제")

# ============================================================================
# ACADOS MPC 모델 정의
# ============================================================================
def create_mpc_model():
    """모바일 로봇 MPC 모델 생성 (Unicycle model)"""
    from acados_template import AcadosModel
    import casadi as ca
    
    model = AcadosModel()
    model.name = 'robot_navigation_mpc'
    
    # 상태 변수: [x, y, theta]
    x = ca.SX.sym('x', 3)
    
    # 제어 입력: [v (선속도), omega (각속도)]
    u = ca.SX.sym('u', 2)
    
    # 상태 미분
    xdot = ca.SX.sym('xdot', 3)
    
    # Unicycle 모델 dynamics
    # x_dot = v * cos(theta)
    # y_dot = v * sin(theta)
    # theta_dot = omega
    f_expl = ca.vertcat(
        u[0] * ca.cos(x[2]),  # x_dot
        u[0] * ca.sin(x[2]),  # y_dot
        u[1]                   # theta_dot
    )
    
    # Implicit/Explicit 표현
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    
    return model

# ============================================================================
# OCP (Optimal Control Problem) 설정
# ============================================================================
def create_ocp(model):
    """MPC 최적화 문제 설정"""
    from acados_template import AcadosOcp
    import casadi as ca
    
    ocp = AcadosOcp()
    ocp.model = model
    
    # Horizon 설정
    N = 30          # Prediction horizon steps
    dt = 0.1        # Time step (seconds)
    
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * dt  # Total prediction time: 5초
    
    # ========================================
    # Cost function 설정 (Nonlinear Least Squares)
    # ========================================
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'  # Terminal cost
    
    # Cost 표현식: [x, y, theta, v, omega]
    x = model.x
    u = model.u
    ocp.model.cost_y_expr = ca.vertcat(x, u)      # Stage cost: 5차원
    ocp.model.cost_y_expr_e = x                    # Terminal cost: 3차원
    
    # Weight 행렬
    # [x, y, theta, v, omega] 가중치
    Q_x = 10.0      # x 위치 추적
    Q_y = 10.0      # y 위치 추적
    Q_theta = 5.0   # 방향 추적
    R_v = 1.0       # 선속도 페널티
    R_omega = 1.0   # 각속도 페널티
    
    ocp.cost.W = np.diag([Q_x, Q_y, Q_theta, R_v, R_omega])
    ocp.cost.W_e = np.diag([Q_x, Q_y, Q_theta]) * 10  # Terminal cost (더 큰 가중치)
    
    # Reference (초기값, 런타임에서 업데이트)
    ocp.cost.yref = np.zeros(5)
    ocp.cost.yref_e = np.zeros(3)
    
    # ========================================
    # 제약 조건
    # ========================================
    # 제어 입력 제한
    v_max = 3      # 최대 선속도 (m/s) - MuJoCo TidyBot에 맞게 조정
    v_min = 0     # 최소 선속도 (후진)
    omega_max = 4  # 최대 각속도 (rad/s)
    
    ocp.constraints.lbu = np.array([v_min, -omega_max])
    ocp.constraints.ubu = np.array([v_max, omega_max])
    ocp.constraints.idxbu = np.array([0, 1])
    
    # 초기 상태 제약 (런타임에서 업데이트)
    ocp.constraints.x0 = np.zeros(3)
    
    # ========================================
    # Solver 옵션
    # ========================================
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'  # Explicit Runge-Kutta
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # Real-Time Iteration
    ocp.solver_options.nlp_solver_max_iter = 1  # RTI는 1회 반복
    ocp.solver_options.print_level = 0
    
    # QP solver 설정
    ocp.solver_options.qp_solver_iter_max = 100
    
    return ocp

# ============================================================================
# MPC Wrapper 클래스 생성
# ============================================================================
def create_mpc_wrapper():
    """MuJoCo와 연동할 MPC wrapper 클래스 파일 생성"""
    
    wrapper_code = '''"""
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
    
    print(f"\\n최종 상태: {state}")
    print(f"목표까지 거리: {mpc.get_distance_to_goal(state):.3f}m")
'''
    
    with open('mpc_controller.py', 'w') as f:
        f.write(wrapper_code)
    
    print("✓ mpc_controller.py 생성 완료")

# ============================================================================
# 메인 실행
# ============================================================================
def main():
    # 1. 기존 빌드 정리
    print("\n[1/4] 기존 빌드 파일 정리...")
    clean_previous_build()
    
    # 2. 모델 생성
    print("\n[2/4] MPC 모델 생성...")
    model = create_mpc_model()
    print(f"✓ 모델 생성: {model.name}")
    
    # 3. OCP 설정 및 solver 생성
    print("\n[3/4] OCP 설정 및 ACADOS Solver 생성...")
    ocp = create_ocp(model)
    
    from acados_template import AcadosOcpSolver
    
    try:
        ocp_solver = AcadosOcpSolver(ocp, json_file='robot_mpc.json')
        print("✓ ACADOS Solver 생성 성공!")
        
        # Solver 정보 출력
        print(f"  - Horizon: N={ocp.solver_options.N_horizon}")
        print(f"  - Time step: dt={ocp.solver_options.tf/ocp.solver_options.N_horizon:.2f}s")
        print(f"  - Total time: {ocp.solver_options.tf:.1f}s")
        
    except Exception as e:
        print(f"❌ Solver 생성 실패: {e}")
        return False
    
    # 4. MPC Wrapper 생성
    print("\n[4/4] MPC Controller wrapper 생성...")
    create_mpc_wrapper()
    
    # 완료 메시지
    print("\n" + "="*70)
    print("✅ ACADOS MPC 생성 완료!")
    print("="*70)
    print("\n생성된 파일:")
    print("  - c_generated_code/     : ACADOS C 코드")
    print("  - robot_mpc.json        : OCP 설정 파일")
    print("  - mpc_controller.py     : MuJoCo 연동용 Python wrapper")
    print("\n사용 예시:")
    print("  from mpc_controller import MPCController")
    print("  mpc = MPCController()")
    print("  mpc.set_goal(2.0, 3.0)")
    print("  control, status = mpc.compute_control(current_state)")
    print("="*70)
    
    # 간단한 테스트
    print("\n[테스트] MPC Solver 동작 확인...")
    test_state = np.array([0.0, 0.0, 0.0])
    
    # 목표 설정
    goal = np.array([1.0, 1.0, 0.0, 0.0, 0.0])
    for i in range(ocp.solver_options.N_horizon):
        ocp_solver.set(i, 'yref', goal)
    ocp_solver.set(ocp.solver_options.N_horizon, 'yref', goal[:3])
    
    # 초기 상태 설정
    ocp_solver.set(0, 'lbx', test_state)
    ocp_solver.set(0, 'ubx', test_state)
    
    # 풀기
    status = ocp_solver.solve()
    control = ocp_solver.get(0, 'u')
    
    print(f"  초기 상태: {test_state}")
    print(f"  목표: [1.0, 1.0, 0.0]")
    print(f"  계산된 제어: v={control[0]:.3f} m/s, omega={control[1]:.3f} rad/s")
    print(f"  Solver 상태: {status} (0=성공)")
    
    if status == 0:
        print("\n✅ 모든 테스트 통과!")
    else:
        print(f"\n⚠️ Solver 상태 코드: {status}")
    
    return True


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)