"""
ACADOS 기반 초고속 MPC 하이브리드 컨트롤러
순수 경로 계획 및 추적에 집중
- ✅ Thread-safe path planning via plan_path_with_start()
- ✅ Use AsyncMPCPlanner wrapper for async operations
- ✅ Simplified architecture (removed internal queue system)
"""

import time
import numpy as np
import logging
from typing import Tuple, Optional
from path_planning.map_processor import MapProcessor
from path_planning.astar_planner import AStarPlanner
from controllers.arm.torque_controller import TorqueController
from controllers.arm.arm_holder import ArmHolder

logger = logging.getLogger(__name__)


class MPCControllerACADOS:
    """ACADOS 기반 MPC 컨트롤러
    
    Pure Path Planning and Tracking:
    - ✅ Thread-safe path planning (plan_path_with_start)
    - ✅ Use AsyncMPCPlanner wrapper for async operations
    - ✅ ACADOS RTI (Real-Time Iteration)
    - ✅ 계산 시간 1-5ms (기존 10-50ms)
    
    Note: This controller is used within AsyncMPCController's worker process
    for natural warm starting and optimal performance.
    """
    
    def __init__(self, model, data, robot_id=1):
        self.model = model
        self.data = data
        self.robot_id = robot_id
        
        # ✅ MULTIPROCESS MODE: Can work without model/data (for worker process)
        self.multiprocess_mode = (model is None and data is None)
        
        # ✅ Configure indices based on robot_id
        if robot_id == 1:
            from config.constants import ARM_Q_IDX,ARM_CTRL_IDX,BASE_CTRL_SLICE,BASE_Q_SLICE
            self.joint_idx = ARM_Q_IDX
            self.base_qpos_idx = BASE_Q_SLICE
            self.base_ctrl_idx = BASE_CTRL_SLICE
            self.arm_ctrl_idx = ARM_CTRL_IDX
            self.robot_name = "Robot1"
        else:  # robot_id == 2
            from config.constants import ROBOT2_ARM_Q_IDX, ROBOT2_BASE_CTRL_SLICE, ROBOT2_BASE_QPOS_IDX, ROBOT2_ARM_CTRL_IDX
            self.joint_idx = ROBOT2_ARM_Q_IDX
            self.base_qpos_idx = ROBOT2_BASE_QPOS_IDX
            self.base_ctrl_idx = ROBOT2_BASE_CTRL_SLICE
            self.arm_ctrl_idx = ROBOT2_ARM_CTRL_IDX
            self.robot_name = "Robot2"
        
        # MPC 상태 초기화
        self.last_u = np.array([0.0, 0.0])
        self.last_state = None
        self.prev_sol = None
        
        # 경로 계획 (정적 맵 기반)
        self.map_processor = MapProcessor()
        self.global_planner = AStarPlanner(self.map_processor)
        
        # ACADOS MPC 파라미터
        self.dt = 0.1   # 예측 스텝
        self.N = 30     # 예측 호라이즌
        
        # 로봇 파라미터
        self.max_linear_vel = 2    # m/s
        self.max_angular_vel =4.0   # rad/s
        self.max_linear_acc = 3.0
        self.max_angular_acc = 3
        
        # MPC 가중치
        self.Q = np.diag([1, 1, 1])      # 상태 가중치 [x, y, theta]
        self.R = np.diag([1, 1])            # 제어 가중치 [v, omega]
        self.Q_e = np.diag([1, 1, 1])    # 종단 가중치

        # ACADOS 솔버 설정 
        self.acados_available = self._setup_acados_solver()
        
        if not self.acados_available:
            logger.error("ACADOS 사용 불가")
            raise RuntimeError("ACADOS solver required")
        
        # 팔 홀더 (✅ Skip in multiprocess mode)
        self.torque_controller = None
        self.arm_holder = None
        if not self.multiprocess_mode:
            self.torque_controller = TorqueController(model, data, joint_idx=self.joint_idx, use_dob=False)
            self.arm_holder = ArmHolder(model, data, self.torque_controller, joint_idx=self.joint_idx)
        
        # 제어 파라미터
        self.control_frequency = 200
        self.control_dt = 1.0 / self.control_frequency
        self.goal_threshold = 0.15
        self.waypoint_threshold = 0.4  # m
        self.lookahead_distance = 1  # m
        
        # 상태 초기화
        self._reset_state()
        
        # 네비게이션
        self.map_loaded = False
        
        # 통계 추적
        self.nav_start_time = None
        self.total_distance = 0.0
        self.prev_pos = None
        self.loop_count = 0
        
        # 성능 모니터링
        self.solve_times = []
        self.debug_interval = 0.5
        self.last_debug_time = 0
    
    def _setup_acados_solver(self):
        """ACADOS RTI 솔버 설정"""
        try:
            import os
            import sys
            

            # ACADOS 경로 설정 (Linux)
            acados_path = os.path.expanduser('~/acados')

            os.environ['ACADOS_SOURCE_DIR'] = acados_path
            os.environ['LD_LIBRARY_PATH'] = f"{acados_path}/lib:" + os.environ.get('LD_LIBRARY_PATH', '')
            sys.path.insert(0, os.path.join(acados_path, 'interfaces', 'acados_template'))
            
            from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
            import casadi as ca
            
            code_export_dir = 'c_generated_code'
            json_file = 'robot_mpc.json'
            dll_path = os.path.join(code_export_dir, 'acados_ocp_solver_robot_navigation_mpc.dll')
            
            # 기존 DLL이 있으면 재사용
            if os.path.exists(dll_path) and os.path.exists(json_file):
                makefile_path = os.path.join(code_export_dir, 'Makefile')
                makefile_backup = os.path.join(code_export_dir, 'Makefile.bak')
                if os.path.exists(makefile_path):
                    os.rename(makefile_path, makefile_backup)
                
                try:
                    self.acados_solver = AcadosOcpSolver(acados_ocp=None, json_file=json_file)
                    return True
                except:
                    if os.path.exists(makefile_backup):
                        os.rename(makefile_backup, makefile_path)
            
            # 새로 생성
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
            
            ocp = AcadosOcp()
            ocp.model = model
            
            ocp.solver_options.N_horizon = self.N
            ocp.solver_options.tf = self.N * self.dt
            
            ocp.cost.cost_type = 'NONLINEAR_LS'
            ocp.cost.cost_type_e = 'NONLINEAR_LS'
            
            nx = 3
            nu = 2
            ny = nx + nu
            ny_e = nx
            
            ocp.model.cost_y_expr = ca.vertcat(x, u)
            ocp.model.cost_y_expr_e = x
            
            Vx = np.zeros((ny, nx))
            Vx[:nx, :] = np.eye(nx)
            ocp.cost.Vx = Vx
            
            Vu = np.zeros((ny, nu))
            Vu[nx:, :] = np.eye(nu)
            ocp.cost.Vu = Vu
            
            ocp.cost.Vx_e = np.eye(nx)
            
            ocp.cost.W = np.diag([100, 100, 10, 0.1, 0.1])
            ocp.cost.W_e = np.diag([200, 200, 20])
            
            ocp.cost.yref = np.zeros(ny)
            ocp.cost.yref_e = np.zeros(ny_e)
            
            ocp.constraints.lbu = np.array([0, -self.max_angular_vel])
            ocp.constraints.ubu = np.array([self.max_linear_vel, self.max_angular_vel])
            ocp.constraints.idxbu = np.array([0, 1])
            
            ocp.constraints.x0 = np.zeros(3)
            
            ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
            ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
            ocp.solver_options.integrator_type = 'ERK'
            ocp.solver_options.nlp_solver_type = 'SQP_RTI'
            ocp.solver_options.nlp_solver_max_iter = 1
            ocp.solver_options.print_level = 0
            
            self.acados_solver = AcadosOcpSolver(ocp, json_file=json_file, build=False)
            
            return True
            
        except Exception as e:
            import traceback
            logger.error(f"[{self.robot_name}] ACADOS 설정 실패: {e}")
            logger.error(traceback.format_exc())
            return False
    
    def solve_mpc(self, x0: np.ndarray, ref_traj: np.ndarray) -> Tuple[float, float]:
        """MPC 최적화 문제 해결"""
        start_time = time.perf_counter()
        
        v, omega = self._solve_acados(x0, ref_traj)
        
        # 성능 모니터링
        solve_time = (time.perf_counter() - start_time) * 1000  # ms
        self.solve_times.append(solve_time)
        if len(self.solve_times) > 100:
            self.solve_times.pop(0)
        
        if solve_time > 10:
            avg_time = np.mean(self.solve_times)
            logger.warning(f"[{self.robot_name}] 계산 시간 {solve_time:.1f}ms (평균: {avg_time:.1f}ms)")
        
        return v, omega
    
    def _solve_acados(self, x0: np.ndarray, ref_traj: np.ndarray) -> Tuple[float, float]:
        """ACADOS RTI 솔버로 해결"""
        
        # 초기 상태 설정
        self.acados_solver.set(0, 'lbx', x0)
        self.acados_solver.set(0, 'ubx', x0)
        
        # 참조 궤적 설정
        for k in range(self.N):
            if k < len(ref_traj):
                x_ref = ref_traj[k]
                v_ref = 1.0 if k < len(ref_traj) - 1 else 0.0
                omega_ref = 0.0
            else:
                x_ref = ref_traj[-1] if len(ref_traj) > 0 else x0
                v_ref = 0.0
                omega_ref = 0.0
            
            y_ref = np.hstack([x_ref, [v_ref, omega_ref]])
            self.acados_solver.set(k, 'yref', y_ref)
        
        # 종단 참조
        if len(ref_traj) > 0:
            self.acados_solver.set(self.N, 'yref', ref_traj[-1])
        else:
            self.acados_solver.set(self.N, 'yref', x0)
        
        # RTI 수행
        status = self.acados_solver.solve()
        
        # 해 추출
        u_opt = self.acados_solver.get(0, 'u')
        
        # Warm start를 위해 해를 shift
        for k in range(self.N-1):
            x_k = self.acados_solver.get(k+1, 'x')
            self.acados_solver.set(k, 'x', x_k)
            if k < self.N-1:
                u_k = self.acados_solver.get(k+1, 'u')
                self.acados_solver.set(k, 'u', u_k)
        
        x_N = self.acados_solver.get(self.N, 'x')
        self.acados_solver.set(self.N-1, 'x', x_N)
        self.acados_solver.set(self.N, 'x', x_N)
        self.acados_solver.set(self.N-1, 'u', np.array([0.0, 0.0]))
        
        v = float(u_opt[0])
        omega = float(u_opt[1])
        
        # 안전 체크
        v = np.clip(v, 0, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        max_omega = self.max_angular_vel

        # 코사인 기반 부드러운 속도 프로파일
        if len(ref_traj) > 0:
            dist_to_target = np.linalg.norm(ref_traj[-1, :2] - x0[:2])
            
            decel_distance = 2.5
            min_speed = 0.05
            max_speed = self.max_linear_vel
            
            if dist_to_target < decel_distance:
                angle = (1.0 - dist_to_target / decel_distance) * (np.pi / 2.0)
                speed_ratio = np.cos(angle)
                target_speed = min_speed + (max_speed - min_speed) * speed_ratio
                
                target_omega = max_omega * speed_ratio
                if abs(omega) > target_omega:
                    omega = np.sign(omega) * target_omega

                v = min(v, target_speed)
                
                if v < 0.05:
                    v = 0.0
                if abs(omega) < 0.01:
                    omega = 0.0
            else:
                v = max(v, 0)
        
        self.last_u = np.array([v, omega])
        return v, omega
    
    def _normalize_angle(self, angle):
        """각도 정규화"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def _reset_state(self):
        """상태 초기화"""
        self.navigation_active = False
        self.global_path = []
        self.current_goal = None
        self.current_v = 0.0
        self.current_omega = 0.0
        self.current_waypoint_idx = 0
        self.last_u = np.array([0.0, 0.0])
        self.last_state = None
        self.prev_sol = None
        self.solve_times = []
        self.current_request_id = None
    
    def _generate_reference_trajectory(self, robot_x: float, robot_y: float, robot_theta: float) -> Optional[np.ndarray]:
        """MPC용 참조 궤적 생성 (순수 경로 추종)"""
        if not self.global_path or self.current_waypoint_idx >= len(self.global_path):
            return None
        
        ref_traj = np.zeros((self.N+1, 3))
        
        self._update_current_waypoint(robot_x, robot_y)
        
        current_idx = self.current_waypoint_idx
        path_length = len(self.global_path)
        
        lookahead_points = max(1, int(self.lookahead_distance / (self.dt * 1.0)))
        
        # 경로점 설정
        for k in range(self.N+1):
            if k == 0:
                wp_idx = min(current_idx, path_length - 1)
            else:
                step_size = max(1, lookahead_points // self.N)
                wp_idx = min(current_idx + k * step_size, path_length - 1)
            
            wp = self.global_path[wp_idx]
            ref_traj[k, 0] = wp[0]
            ref_traj[k, 1] = wp[1]
            
            if k == 0:
                dx = wp[0] - robot_x
                dy = wp[1] - robot_y
            else:
                dx = ref_traj[k, 0] - ref_traj[k-1, 0]
                dy = ref_traj[k, 1] - ref_traj[k-1, 1]
            
            if np.sqrt(dx**2 + dy**2) > 0.01:
                ref_traj[k, 2] = np.arctan2(dy, dx)
            else:
                ref_traj[k, 2] = ref_traj[k-1, 2] if k > 0 else robot_theta
        
        return ref_traj
    
    def _update_current_waypoint(self, robot_x: float, robot_y: float):
        """현재 웨이포인트 업데이트"""
        if not self.global_path:
            return
        
        min_dist = float('inf')
        closest_idx = self.current_waypoint_idx
        
        search_range = min(10, len(self.global_path) - self.current_waypoint_idx)
        for i in range(search_range):
            idx = self.current_waypoint_idx + i
            if idx >= len(self.global_path):
                break
                
            wp = self.global_path[idx]
            dist = np.sqrt((wp[0] - robot_x)**2 + (wp[1] - robot_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
        
        if min_dist < self.waypoint_threshold:
            self.current_waypoint_idx = min(closest_idx + 1, len(self.global_path) - 1)
            
            if self.current_waypoint_idx % 5 == 0 or self.current_waypoint_idx == len(self.global_path) - 1:
                progress = (self.current_waypoint_idx / len(self.global_path)) * 100
                avg_time = np.mean(self.solve_times) if self.solve_times else 0
                logger.debug(f"[{self.robot_name}] 진행률: {progress:.1f}% | 평균 계산: {avg_time:.1f}ms")
    
    def get_mpc_velocity(self, robot_x: float, robot_y: float, robot_yaw: float) -> Tuple[float, float]:
        """MPC 기반 속도 명령 계산"""
        x0 = np.array([robot_x, robot_y, robot_yaw])
        
        if self.last_state is not None:
            dt = self.control_dt
            dx = robot_x - self.last_state[0]
            dy = robot_y - self.last_state[1]
            self.current_v = np.sqrt(dx**2 + dy**2) / dt
            self.current_omega = (robot_yaw - self.last_state[2]) / dt
        
        self.last_state = x0.copy()
        
        ref_traj = self._generate_reference_trajectory(robot_x, robot_y, robot_yaw)
        
        if ref_traj is None:
            return 0.0, 0.0
        
        # Synchronous MPC solve (used within AsyncMPCController worker)
        v, omega = self.solve_mpc(x0, ref_traj)
        
        return v, omega
    
    def load_map(self, map_file: str) -> bool:
        """맵 파일 로드"""
        if map_file:
            success = self.map_processor.load_map(map_file, auto_dilate=True)
            if success:
                self.global_planner.map_processor = self.map_processor
                self.map_loaded = True
                
                robot_radius = 0.3
                safety_margin = 0.3
                total_radius = robot_radius + safety_margin
                dilation_cells = int(total_radius / self.map_processor.resolution)
                self.map_processor.dilate_obstacles(radius=dilation_cells)
                
                return True
            else:
                return False
        else:
            return True
    
    def _get_current_position(self):
        """현재 로봇 위치 가져오기 (헬퍼 메서드)"""
        if self.robot_id == 2:
            body_name = "robot2_base_link"
            body_id = self.model.body(body_name).id
            body_xpos = self.data.xpos[body_id]
            current_x = body_xpos[0]
            current_y = body_xpos[1]
            base_pos = self.data.qpos[self.base_qpos_idx]
            current_theta = base_pos[2]
        else:
            body_name = "robot1_base_link"
            body_id = self.model.body(body_name).id
            body_xpos = self.data.xpos[body_id]
            current_x = body_xpos[0]
            current_y = body_xpos[1]
            base_pos = self.data.qpos[self.base_qpos_idx]
            current_theta = base_pos[2]
        
        return (current_x, current_y), current_theta
    
    def update_control(self):
        """제어 업데이트 (중앙 step 관리자에서 호출)"""
        # ✅ 경로 계획 완료 시 자동으로 네비게이션 시작
        if not self.navigation_active and self.current_request_id is not None:
            # Try to start navigation if planning is complete
            if not self.check_and_start_navigation():
                # Still planning or failed - wait
                return
        
        if not self.navigation_active:
            return
        
        self.loop_count += 1
        
        # 팔 홀더 토크 적용
        torque = self.arm_holder.compute_hold_torque()
        self.data.ctrl[self.arm_ctrl_idx] = torque
        
        # Get robot position
        if self.robot_id == 2:
            body_id = self.model.body("robot2_base_link").id
            body_xpos = self.data.xpos[body_id]
            x = body_xpos[0]
            y = body_xpos[1]
            base_pos = self.data.qpos[self.base_qpos_idx]
            yaw = base_pos[2]
        else:
            body_id = self.model.body("robot1_base_link").id
            body_xpos = self.data.xpos[body_id]
            x = body_xpos[0]
            y = body_xpos[1]
            base_pos = self.data.qpos[self.base_qpos_idx]
            yaw = base_pos[2]
        
        # 이동 거리 추적
        if self.prev_pos is not None:
            dist = np.sqrt((x - self.prev_pos[0])**2 + (y - self.prev_pos[1])**2)
            self.total_distance += dist
        self.prev_pos = (x, y)
        
        if self._check_goal_reached(x, y):
            avg_time = np.mean(self.solve_times) if self.solve_times else 0
            logger.info(f"[{self.robot_name}] 🎉 목표 도달! (평균 계산 시간: {avg_time:.1f}ms)")
            self.navigation_active = False
            self.data.ctrl[self.base_ctrl_idx] = 0.0
            return
        
        v, omega = self.get_mpc_velocity(x, y, yaw)
        
        self._apply_velocity(v, omega)
        
        current_time = time.time()
        if current_time - self.last_debug_time > self.debug_interval:
            self.last_debug_time = current_time
            if self.global_path:
                progress = (self.current_waypoint_idx / len(self.global_path)) * 100
                avg_time = np.mean(self.solve_times) if self.solve_times else 0
                logger.debug(f"[{self.robot_name}] {progress:.1f}% | v={v:.2f} ω={omega:.2f} | 계산={avg_time:.1f}ms")
    
    def _check_goal_reached(self, x: float, y: float) -> bool:
        """목표 도달 확인"""
        if self.current_goal is None:
            return False
        
        dist = np.sqrt((x - self.current_goal[0])**2 + 
                      (y - self.current_goal[1])**2)
        return dist < self.goal_threshold
    
    def _apply_velocity(self, v: float, omega: float):
        """속도 적용"""
        base_pos = self.data.qpos[self.base_qpos_idx]
        yaw = base_pos[2]
        
        vx_world = v * np.cos(yaw)
        vy_world = v * np.sin(yaw)
        
        base_ctrl = self.data.ctrl[self.base_ctrl_idx]
        base_ctrl[0] = vx_world
        base_ctrl[1] = vy_world
        base_ctrl[2] = omega
        self.data.ctrl[self.base_ctrl_idx] = base_ctrl
    
    def start_navigation(self):
        """네비게이션 초기화"""
        if not self.navigation_active:
            logger.warning(f"[{self.robot_name}] 네비게이션이 활성화되지 않음")
            return
        
        logger.info(f"[{self.robot_name}] 🚀 네비게이션 시작")
        
        # 팔 홀더 초기화 (✅ Skip in multiprocess mode)
        if not self.multiprocess_mode and self.arm_holder is not None:
            current_q = np.copy(self.data.qpos[self.joint_idx])
            self.arm_holder._init_ruckig(current_q)
        
        # 통계 초기화
        self.nav_start_time = time.time()
        self.total_distance = 0.0
        base_pos = self.data.qpos[self.base_qpos_idx]
        self.prev_pos = (base_pos[0], base_pos[1])
        self.loop_count = 0
        
        logger.info(f"[{self.robot_name}] ✅ 네비게이션 초기화 완료")
    
    def stop(self):
        """정지"""
        # 통계 출력
        if self.navigation_active and self.nav_start_time is not None:
            total_time = time.time() - self.nav_start_time
            avg_speed = self.total_distance / total_time if total_time > 0 else 0
            avg_freq = self.loop_count / total_time if total_time > 0 else 0
            avg_solve_time = np.mean(self.solve_times) if self.solve_times else 0
            
            logger.info(f"\n[{self.robot_name}] 📊 네비게이션 완료")
            logger.info(f"  - 이동 거리: {self.total_distance:.2f}m")
            logger.info(f"  - 소요 시간: {total_time:.1f}초")
            logger.info(f"  - 평균 속도: {avg_speed:.2f}m/s")
            logger.info(f"  - 제어 주파수: {avg_freq:.1f}Hz")
            logger.info(f"  - 평균 계산 시간: {avg_solve_time:.1f}ms")
            logger.info(f"  - 총 루프 횟수: {self.loop_count}")
        
        logger.info(f"[{self.robot_name}] 🛑 정지")
        
        self.navigation_active = False
        self.current_v = 0.0
        self.current_omega = 0.0
        
        # ✅ Safe for multiprocess mode
        if self.data is not None:
            self.data.ctrl[self.base_ctrl_idx] = 0.0
        
        self.global_path = []
        self.current_goal = None
        self.current_waypoint_idx = 0
        self.prev_sol = None
        self.current_request_id = None
        
        # 통계 초기화
        self.nav_start_time = None
        self.total_distance = 0.0
        self.prev_pos = None
        self.loop_count = 0
        
        # ACADOS warm start 초기화
        try:
            for k in range(self.N):
                self.acados_solver.set(k, 'x', np.zeros(3))
                if k < self.N:
                    self.acados_solver.set(k, 'u', np.zeros(2))
        except:
            pass
    
    def is_navigation_complete(self) -> bool:
        return not self.navigation_active
