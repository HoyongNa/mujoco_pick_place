"""Pick and Place 태스크 실행 - 8개 박스, 4번 작업"""

import mujoco
import time
import numpy as np
from controllers.arm.arm_controller import ArmController
from controllers.gripper.grasp_checker import GraspChecker
from tasks.feasibility_checker import FeasibilityChecker
from config.constants import SETTLE_STEPS_GRASP, SETTLE_STEPS_RELEASE

class PickAndPlaceTask:
    """Pick and Place 태스크 로직 - 4번 작업"""
    
    def __init__(self, sim_manager):
        self.sim = sim_manager
        self.model = self.sim.model
        self.data = self.sim.data
        
        # 4개 방별 작업 쌍 정의
        self.tasks = [
            {
                "pick": "red_box", 
                "place": "blue_box", 
                "name": "빨간색 → 파란색",
                "room": "방 1 (북서)",
                "location": "(-2.5, 2.5)"
            },
            {
                "pick": "green_box", 
                "place": "yellow_box", 
                "name": "초록색 → 노란색",
                "room": "방 2 (북동)",
                "location": "(2.5, 2.5)"
            },
            {
                "pick": "orange_box", 
                "place": "purple_box", 
                "name": "주황색 → 보라색",
                "room": "방 3 (남서)",
                "location": "(-2.5, -2.5)"
            },
            {
                "pick": "cyan_box", 
                "place": "pink_box", 
                "name": "청록색 → 분홍색",
                "room": "방 4 (남동)",
                "location": "(2.5, -2.5)"
            }
        ]

    def execute(self):
        """태스크 실행 - 스페이스바로 각 작업 개별 실행"""
        print("\n Pick & Place 작업 시작 (8개 박스, 4번 작업)...")
        
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
            completed_tasks = 0
            
            # 4개 작업 순차 실행
            for i, task in enumerate(self.tasks, 1):
                task_completed = False
                
                while not task_completed and self.sim.viewer_manager.is_running():
                    print("\n" + "="*50)
                    print(f" 작업 {i}/4: {task['room']}")
                    print(f" 위치: {task['location']}")
                    print(f" 내용: {task['name']}")
                    print(" [Space] 키를 누르면 시작합니다.")
                    print("="*50)
                    
                    if not self._wait_for_space(with_mobility=(i > 1)):
                        return False
                    
                    # 모빌리티 정지 후 작업 실행 (현재 위치 유지)
                    self.sim.stop_mobility_control(maintain_position=True)
                    
                    # 작업 실행 시도
                    if self._execute_single_task(task["pick"], task["place"], task["name"]):
                        task_completed = True
                        completed_tasks += 1
                        print("\n" + "-"*50)
                        print(f" ✅ 작업 {i}/4 완료!")
                        print("-"*50)
                    else:
                        # 실패 시 모빌리티 재개하여 베이스 조정 가능
                        print("\n" + "-"*50)
                        print(" ⚠️  베이스를 조정한 후 다시 시도하세요.")
                        print("-"*50)
                        self.sim.start_mobility_control()
                
                # 작업 완료 후 모빌리티 재개
                if task_completed and i < len(self.tasks):
                    self.sim.start_mobility_control()
            
            # 모든 작업 완료
            print("\n" + "="*50)
            print(f" ✅ 모든 Pick & Place 작업 완료! ({completed_tasks}/4)")
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

    def _execute_single_task(self, pick_name, place_name, task_name):
        """단일 Pick & Place 작업 실행"""
        print(f"\n {task_name} 작업 시작...")
        
        # 박스 위치 가져오기
        try:
            pick_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, pick_name)
            place_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, place_name)
        except Exception as e:
            print(f" ❌ 박스를 찾을 수 없습니다: {e}")
            return False
        
        pick_pos = self.data.xpos[pick_body_id]
        place_pos = self.data.xpos[place_body_id]
        
        # 실행 가능성 체크
        print("\n ℹ️  Pick & Place 가능성 체크 중...")
        feasible, msg = self.feasibility_checker.check_pick_and_place_feasibility(
            pick_pos, place_pos
        )
        
        if not feasible:
            print(f" ❌ Pick & Place 수행 불가능: {msg}")
            
            # 베이스 위치 제안
            suggested_pos, suggest_msg = self.feasibility_checker.suggest_base_position(pick_pos)
            print(f" 💡 제안: {suggest_msg}")
            
            # 더 나은 베이스 위치로 이동 제안
            base_pos = self.data.qpos[:2]
            distance = np.linalg.norm(suggested_pos - base_pos)
            print(f" 💡 현재 베이스 위치: ({base_pos[0]:.2f}, {base_pos[1]:.2f})")
            print(f" 💡 제안 베이스 위치: ({suggested_pos[0]:.2f}, {suggested_pos[1]:.2f})")
            print(f" 💡 이동 거리: {distance:.2f}m")
            return False
        
        print(f" ✅ {msg}")
        
        # Waypoint 생성
        waypoints = self.sim.waypoint_gen.generate_pick_place_waypoints(
            pick_pos, place_pos
        )
        
        # Waypoint 실행
        if not self._execute_waypoints(waypoints, pick_name):
            return False
        
        # 홈 복귀
        self._return_home()
        time.sleep(0.5)
        
        return True

    def _wait_for_space(self, with_mobility=False):
        """스페이스 키 대기"""
        if with_mobility:
            print(" 🕹️  숫자패드로 베이스를 이동할 수 있습니다.")
            print("     8/5: 전진/후진, 4/6: 좌/우 이동, 7/9: 좌/우 회전")
        
        print(" ▶  준비되면 [Space]를 누르세요. [ESC]로 종료.")
        
        try:
            import keyboard
            space_armed = True
            
            while self.sim.viewer_manager.is_running():
                if keyboard.is_pressed('space'):
                    if space_armed:
                        time.sleep(0.2)  # 키 릴리즈 대기
                        return True
                    space_armed = False
                else:
                    space_armed = True
                
                if keyboard.is_pressed('escape'):
                    return False
                    
                time.sleep(0.01)
        except ImportError:
            # keyboard 모듈이 없으면 2초 대기
            print(" keyboard 모듈이 없습니다. 2초 후 자동 시작...")
            time.sleep(2)
            return True
        
        return False

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
        print(" 팔을 홈 자세로 복귀합니다...")
        self.sim.shared_gripper_ctrl[0] = 0
        self.sim.arm_controller.track_with_ruckig(self.sim.arm_home_q)
    
    def settle(self, steps, sleep_dt=0.002):
        """정착을 위한 여러 스텝 실행"""
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            time.sleep(sleep_dt)
