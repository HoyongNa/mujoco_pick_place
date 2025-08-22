"""Pick and Place 태스크 실행"""

import mujoco
import time
import numpy as np
from controllers.arm.arm_controller import ArmController
from controllers.gripper.grasp_checker import GraspChecker
from tasks.feasibility_checker import FeasibilityChecker
from config.constants import SETTLE_STEPS_GRASP, SETTLE_STEPS_RELEASE

class PickAndPlaceTask:
    """Pick and Place 태스크 로직"""
    
    def __init__(self, sim_manager):
        self.sim = sim_manager
        self.model = self.sim.model
        self.data = self.sim.data

    def execute(self):
        """태스크 실행 - 스페이스바로 각 작업 개별 실행"""
        print("\n Pick & Place 작업 시작...")
        
        # 초기화
        self.sim.initialize_viewer()
        self.sim.stop_mobility_control()
        
        # 컨트롤러 생성
        self._setup_controllers()
        
        # 실행 가능성 체커 생성
        self.feasibility_checker = FeasibilityChecker(
            self.sim.model, self.sim.data,
            self.sim.ik_solver, self.sim.config
        )
        
        try:
            # 첫 번째 작업: 빨간색 박스 -> 파란색 박스
            task1_completed = False
            while not task1_completed and self.sim.viewer_manager.is_running():
                print("\n" + "="*50)
                print(" 작업 1/2: 빨간색 박스 → 파란색 박스")
                print(" [Space] 키를 누르면 시작합니다.")
                print("="*50)
                
                if not self._wait_for_space(with_mobility=False):
                    return False
                
                # 모빌리티 정지 후 작업 실행 (현재 위치 유지)
                self.sim.stop_mobility_control(maintain_position=True)
                
                # 작업 실행 시도
                if self._execute_task_1():
                    task1_completed = True
                    print("\n" + "-"*50)
                    print(" ✅ 작업 1/2 완료!")
                    print("-"*50)
                else:
                    # 실패 시 모빌리티 재개하여 베이스 조정 가능
                    print("\n" + "-"*50)
                    print(" ⚠️  베이스를 조정한 후 다시 시도하세요.")
                    print("-"*50)
                    self.sim.start_mobility_control()
            
            # 첫 번째 작업 후 모빌리티 재개
            self.sim.start_mobility_control()
            
            # 두 번째 작업: 초록색 박스 -> 노란색 박스
            task2_completed = False
            while not task2_completed and self.sim.viewer_manager.is_running():
                print("\n" + "="*50)
                print(" 작업 2/2: 초록색 박스 → 노란색 박스")
                print("="*50)
                
                if not self._wait_for_space(with_mobility=True):
                    return False
                
                # 모빌리티 정지 후 작업 실행 (현재 위치 유지)
                self.sim.stop_mobility_control(maintain_position=True)
                
                # 작업 실행 시도
                if self._execute_task_2():
                    task2_completed = True
                else:
                    # 실패 시 모빌리티 재개하여 베이스 조정 가능
                    print("\n" + "-"*50)
                    print(" ⚠️  베이스를 조정한 후 다시 시도하세요.")
                    print("-"*50)
                    self.sim.start_mobility_control()
            
            # 모든 작업 완료
            print("\n" + "="*50)
            print(" ✅ 모든 Pick & Place 작업 완료!")
            print(" 베이스 텔레옵 재개. ESC로 창 닫기")
            print("="*50)
            
            # Mobility 재개
            self.sim.start_mobility_control()
            
            # 대기
            while self.sim.viewer_manager.is_running():
                time.sleep(0.01)
                
            return True
            
        finally:
            self.sim.viewer_manager.close()
    
    def _wait_for_space(self, with_mobility=False):
        """스페이스바 입력 대기"""
        import keyboard
        space_armed = True
        
        if with_mobility:
            print(" 🕹️  로봇 베이스를 이동할 수 있습니다 (숫자패드 사용)")
            print(" ▶ 준비가 되면 [Space] 키를 누르세요.")
        
        while self.sim.viewer_manager.is_running():
            if keyboard.is_pressed('space'):
                if space_armed:
                    print(" ▶ 작업을 시작합니다...")
                    return True
                space_armed = False
            else:
                space_armed = True
            time.sleep(0.01)
        print(" 취소: 뷰어가 닫혔습니다.")
        return False
    
    def _execute_task_1(self):
        """첫 번째 작업 실행: 빨간색 -> 파란색"""
        print("\n 작업 진행: 빨간색 박스 → 파란색 박스")
        
        # 박스 위치 가져오기
        red_pos = self._get_red_box_position()
        blue_pos = self._get_blue_box_position()
        
        # 실행 가능성 체크
        print("\n ℹ️  Pick & Place 가능성 체크 중...")
        feasible, msg = self.feasibility_checker.check_pick_and_place_feasibility(
            red_pos, blue_pos
        )
        
        if not feasible:
            print(f" ❌ Pick & Place 수행 불가능: {msg}")
            
            # 베이스 위치 제안
            suggested_pos, suggest_msg = self.feasibility_checker.suggest_base_position(red_pos)
            print(f" 💡 제안: {suggest_msg}")
            print(" 베이스를 이동한 후 다시 시도해주세요.")
            return False
        
        print(f" ✅ {msg}")
        
        # 웨이포인트 생성 및 실행
        waypoints = self.sim.waypoint_gen.generate_pick_place_waypoints(
            red_pos, blue_pos
        )
        
        if not self._execute_waypoints(waypoints, "빨간색 박스"):
            return False
        
        # 홈 복귀
        self._return_home()
        time.sleep(0.5)
        return True
    
    def _execute_task_2(self):
        """두 번째 작업 실행: 초록색 -> 노란색"""
        print("\n 작업 진행: 초록색 박스 → 노란색 박스")
        
        # 박스 위치 가져오기
        green_pos = self._get_green_box_position()
        yellow_pos = self._get_yellow_box_position()
        
        # 실행 가능성 체크
        print("\n ℹ️  Pick & Place 가능성 체크 중...")
        feasible, msg = self.feasibility_checker.check_pick_and_place_feasibility(
            green_pos, yellow_pos
        )
        
        if not feasible:
            print(f" ❌ Pick & Place 수행 불가능: {msg}")
            
            # 베이스 위치 제안
            suggested_pos, suggest_msg = self.feasibility_checker.suggest_base_position(green_pos)
            print(f" 💡 제안: {suggest_msg}")
            print(" 베이스를 이동한 후 다시 시도해주세요.")
            return False
        
        print(f" ✅ {msg}")
        
        # 웨이포인트 생성 및 실행
        waypoints = self.sim.waypoint_gen.generate_pick_place_waypoints(
            green_pos, yellow_pos
        )
        
        if not self._execute_waypoints(waypoints, "초록색 박스"):
            return False
        
        # 홈 복귀
        self._return_home()
        time.sleep(0.5)
        return True
            
    def _setup_controllers(self):
        """컨트롤러 초기화"""
        self.sim.arm_controller = ArmController(
            self.sim.model, self.sim.data,
            list(range(3, 10)), list(range(3, 10)),
            self.sim.shared_gripper_ctrl,
            viewer=self.sim.viewer_manager.viewer,
            base_cmd_ref=self.sim.base_cmd_ref,
            base_lock=self.sim.base_lock
        )
        
        self.sim.grasp_checker = GraspChecker(
            self.sim.model, self.sim.data,
            self.sim.config.ee_site_id,
            self.sim.config.left_pad_body_id,
            self.sim.config.right_pad_body_id,
            viewer=self.sim.viewer_manager.viewer
        )
        
    def _execute_waypoints(self, waypoints, object_name="객체"):
        """웨이포인트 실행"""
        print(f" {object_name} 작업 시작...")
        
        # 접근
        self._move_to_pose(*waypoints[0], gripper_state=0)
        self._move_to_pose(*waypoints[1], gripper_state=0)
        
        # 파지
        if not self._grasp_object(object_name):
            print(f" {object_name} 파지 실패. 작업 중단.")
            return False
            
        # 이동
        self._move_to_pose(*waypoints[2], gripper_state=255)
        self._move_to_pose(*waypoints[3], gripper_state=255)
        self._move_to_pose(*waypoints[4], gripper_state=255)
        
        # 릴리즈
        self._release_object(object_name)
        
        # 복귀
        self._move_to_pose(*waypoints[5], gripper_state=0)
        self._move_to_pose(*waypoints[6], gripper_state=0)
        
        print(f" ✓ {object_name} Pick & Place 완료!")
        return True
        
    def _move_to_pose(self, pos, rpy, gripper_state):
        """목표 자세로 이동"""
        q_opt = self.sim.ik_solver.solve(pos, rpy)
        self.sim.shared_gripper_ctrl[0] = gripper_state
        self.sim.arm_controller.track_with_ruckig(q_opt)
        
    def _grasp_object(self, object_name="객체"):
        """물체 파지"""
        print(f"  {object_name} 파지 시도...")
        self.sim.shared_gripper_ctrl[0] = 255
        self.settle(SETTLE_STEPS_GRASP)
        return self.sim.grasp_checker.wait_until_grasped(
            threshold=0.05, max_time=3.0
        )
        
    def _release_object(self, object_name="객체"):
        """물체 릴리즈"""
        print(f"  {object_name} 놓기...")
        self.sim.shared_gripper_ctrl[0] = 0
        self.settle(SETTLE_STEPS_RELEASE)
        
    def _return_home(self):
        """홈 자세로 복귀"""
        print("\n 팔을 홈 자세로 복귀합니다...")
        self.sim.shared_gripper_ctrl[0] = 0
        self.sim.arm_controller.track_with_ruckig(self.sim.arm_home_q)
        
    def _get_red_box_position(self):
        """빨간 상자 위치"""
        return self.sim.data.xpos[self.sim.config.red_box_id]
        
    def _get_blue_box_position(self):
        """파란 상자 위치"""
        return self.sim.data.xpos[self.sim.config.blue_box_id]
    
    def _get_yellow_box_position(self):
        """노란 상자 위치"""
        return self.sim.data.xpos[self.sim.config.yellow_box_id]
    
    def _get_green_box_position(self):
        """초록 상자 위치"""
        return self.sim.data.xpos[self.sim.config.green_box_id]
    
    def settle(self, steps, sleep_dt=0.002):
        """정착을 위한 여러 스텝 실행"""
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            time.sleep(sleep_dt)