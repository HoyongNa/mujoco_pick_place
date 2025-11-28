"""
ACADOS 완전 통합 스크립트 - OpenMP 활성화 (경로 수정 버전)
"""

import os
import sys
import subprocess
import shutil

# 환경 설정 (실제 ACADOS 경로로 수정됨)
os.environ['ACADOS_SOURCE_DIR'] = r'C:\acados_new\acados'
os.environ['PATH'] = r'C:\msys64\mingw64\bin;C:\acados_new\acados\bin;' + os.environ.get('PATH', '')
sys.path.insert(0, r'C:\acados_new\acados\interfaces\acados_template')

# OpenMP 스레드 개수 설정 (멀티스레드 MPC를 위해)
os.environ['OMP_NUM_THREADS'] = '4'  # CPU 코어 개수에 맞춰 조정

# 1. 기존 파일 삭제
if os.path.exists('c_generated_code'):
    shutil.rmtree('c_generated_code')
    print("✓ 기존 c_generated_code 삭제")

if os.path.exists('robot_mpc.json'):
    os.remove('robot_mpc.json')
    print("✓ 기존 robot_mpc.json 삭제")

print("\n" + "="*70)
print("ACADOS C 코드 생성 중... (OpenMP 지원)")
print("="*70)

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np

# 모델 정의 (파라미터 없이)
model = AcadosModel()
model.name = 'robot_navigation_mpc'

x = ca.SX.sym('x', 3)
u = ca.SX.sym('u', 2)
xdot = ca.SX.sym('xdot', 3)

f_expl = ca.vertcat(
    u[0] * ca.cos(x[2]),
    u[0] * ca.sin(x[2]),
    u[1]
)

model.f_impl_expr = xdot - f_expl
model.f_expl_expr = f_expl
model.x = x
model.xdot = xdot
model.u = u
# model.p 없음 - 파라미터 제거!

# OCP 설정
ocp = AcadosOcp()
ocp.model = model

N = 50
dt = 0.1
ocp.solver_options.N_horizon = N
ocp.solver_options.tf = N * dt

ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.cost.cost_type_e = 'NONLINEAR_LS'

ocp.model.cost_y_expr = ca.vertcat(x, u)
ocp.model.cost_y_expr_e = x

ocp.cost.W = np.diag([1, 1, 1, 1, 1])
ocp.cost.W_e = np.diag([1, 1, 1])

ocp.cost.yref = np.zeros(5)
ocp.cost.yref_e = np.zeros(3)

ocp.constraints.lbu = np.array([0, -4.0])
ocp.constraints.ubu = np.array([3.0, 4.0])
ocp.constraints.idxbu = np.array([0, 1])

ocp.constraints.x0 = np.zeros(3)

ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.print_level = 0

# 코드 생성 (Makefile 컴파일은 실패하지만 C 코드는 생성됨)
try:
    ocp_solver = AcadosOcpSolver(ocp, json_file='robot_mpc.json')
except:
    pass  # 예상된 동작

print("✓ C 코드 생성 완료")

if not os.path.exists('c_generated_code'):
    print("❌ C 코드 생성 실패")
    sys.exit(1)

os.chdir('c_generated_code')

# 직접 GCC로 컴파일 (OpenMP 플래그 포함)
print("\nDLL 컴파일 중... (OpenMP 활성화)")
compile_cmd = """
gcc -shared -fPIC -fopenmp -o acados_ocp_solver_robot_navigation_mpc.dll
acados_solver_robot_navigation_mpc.c
robot_navigation_mpc_model/*.c
robot_navigation_mpc_cost/*.c
-IC:/acados_new/acados
-IC:/acados_new/acados/interfaces
-IC:/acados_new/acados/external
-IC:/acados_new/acados/external/blasfeo/include
-IC:/acados_new/acados/external/hpipm/include
-LC:/acados_new/acados/bin
-lacados -lblasfeo -lhpipm -lgomp -lm
"""

cmd = ' '.join(compile_cmd.split())
result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

if os.path.exists('acados_ocp_solver_robot_navigation_mpc.dll'):
    size = os.path.getsize('acados_ocp_solver_robot_navigation_mpc.dll') / 1024
    print(f"✅ 컴파일 성공: {size:.1f} KB")
    
    # OpenMP 정보 출력
    omp_threads = os.environ.get('OMP_NUM_THREADS', '1')
    print(f"✅ OpenMP 스레드: {omp_threads}개")
    
    os.chdir('..')
    print("\n" + "="*70)
    print("성공! (OpenMP 활성화됨)")
    print(f"멀티스레드 MPC: {omp_threads} 스레드 사용")
    print("\nmain.py를 실행하세요: python main.py")
    print("="*70)
else:
    print("❌ 컴파일 실패")
    if result.stderr:
        print("에러:")
        print(result.stderr[:1000])
    os.chdir('..')
    sys.exit(1)