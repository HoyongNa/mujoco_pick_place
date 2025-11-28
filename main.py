"""
MuJoCo Navigation & Pick & Place System with LLM Integration (Thread-Safe 버전)
 LLM executor를 메인 스레드에서 실행하여 MuJoCo thread-safety 문제 해결
 모든 물리 스텝이 메인 루프에서 실행 (Centralized Stepping)
- 입력 스레드: 사용자 명령만 받아서 큐에 저장
- 메인 스레드: 명령을 큐에서 꺼내서 처리 (동기 실행)
- 물리 step은 오직 메인 스레드에서만 호출
"""
import os
import time
import threading
import logging
from typing import Optional
from pynput import keyboard
from llm_planner.planner import LLMPlanner
from llm_planner.planner.task_types import TaskPlan
from llm_planner.executor import TaskExecutor
from simulation.simulation_manager import SimulationManager
from config.constants import DEFAULT_XML_PATH, ARM_Q_IDX, ARM_CTRL_IDX, \
      ROBOT2_ARM_Q_IDX, ROBOT2_ARM_CTRL_IDX

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

# --- 환경 설정 ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_FILE = os.path.join(BASE_DIR, "lidar_mapping", "maps", "lidar_map_20250826_215447.npz")


class NavigationPickPlaceSystem:
    """통합 시스템 - LLM을 메인 스레드에서 실행하여 thread-safety 보장"""
    def __init__(self):
        self.sim_manager = SimulationManager(DEFAULT_XML_PATH)

        # 상태
        #  REMOVED: shared current_mode (was causing cross-robot interference)
        self.llm_mode_active = False
        self.saved_arm_position = None
        self.saved_gripper_position = None
        
        #  Robot2 상태 관리
        self.saved_arm_position_robot2 = None
        self.saved_gripper_position_robot2 = None
        
        #  팔 제어권 관리 (독립 제어)
        self.is_arm_busy_robot1 = False  # Robot1 executor가 팔을 사용 중인지
        self.is_arm_busy_robot2 = False  # Robot2 executor가 팔을 사용 중인지
        
        #  Backward compatibility property for old executor.py
        self._is_arm_busy_compat = False

        #  LLM 명령 큐
        self.pending_llm_command: Optional[str] = None
        self._command_lock = threading.Lock()
        
        #  Step counter for debugging
        self._step_count = 0  # Total steps executed

        #  LLM (Dual Executors for parallel operation)
        self.llm_planner: Optional["LLMPlanner"] = None
        self.llm_executor_robot1: Optional["TaskExecutor"] = None  # Robot1 executor
        self.llm_executor_robot2: Optional["TaskExecutor"] = None  # Robot2 executor
        self.llm_input_thread: Optional[threading.Thread] = None
        self._init_llm_components()

        # 입력
        self.pressed_keys = set()
        self._key_lock = threading.Lock()
        self.last_key_time = {}
        self.key_debounce_time = 0.3
        self.keyboard_listener = keyboard.Listener(on_press=self._on_key_change(True),
                                                   on_release=self._on_key_change(False))
        self.keyboard_listener.start()

    # ---------- LLM ----------
    def _init_llm_components(self):
        api_key = os.environ.get("OPENAI_API_KEY", "")
        if not api_key:
            logger.info("OPENAI_API_KEY 미설정: LLM 비활성")
            return

        scene_xml_path = os.path.join(BASE_DIR, "model", "stanford_tidybot", "scene.xml")
        self.llm_planner = LLMPlanner(
            api_key=api_key, 
            model="gpt-4o",
            scene_xml=scene_xml_path if os.path.exists(scene_xml_path) else None,
        )
        # ✅ CHANGED: Create separate executors for parallel operation
        self.llm_executor_robot1 = TaskExecutor(robot_system=self, robot_id=1)
        self.llm_executor_robot2 = TaskExecutor(robot_system=self, robot_id=2)


    # ---------- 입력 ----------
    def _on_key_change(self, pressed: bool):
        def handler(key):
            name = None
            if hasattr(key, "char") and key.char == "-":
                name = "minus"
            elif key == keyboard.Key.esc:
                name = "escape"
            elif key == keyboard.Key.f8:
                name = "f8"
            if not name:
                return
            with self._key_lock:
                if pressed:
                    self.pressed_keys.add(name)
                else:
                    self.pressed_keys.discard(name)
        return handler

    def _is_key_pressed(self, key_name: str) -> bool:
        with self._key_lock:
            if key_name not in self.pressed_keys:
                return False
            now = time.time()
            last = self.last_key_time.get(key_name, 0.0)
            if now - last <= self.key_debounce_time:
                return False
            self.last_key_time[key_name] = now
            return True

    # ---------- 상태 저장/복원 ----------
    def _save_arm_gripper_state(self):
        # Robot 1 상태 저장
        self.sim_manager.saved_arm_position = self.sim_manager.data.qpos[ARM_Q_IDX].copy()
        self.sim_manager.saved_gripper_state = float(self.sim_manager.data.ctrl[10])
        # Robot 2 상태 저장
        self.sim_manager.saved_arm_position_robot2 = self.sim_manager.data.qpos[ROBOT2_ARM_Q_IDX].copy()
        self.sim_manager.saved_gripper_state_robot2 = float(self.sim_manager.data.ctrl[21])


    def _restore_mobility_with_saved_state(self):
        self.sim_manager.stop_path_control()
        self.sim_manager.stop_mobility_control()

        #  Start Robot2 mobility control
        self.sim_manager.start_mobility_control()
        self.sim_manager.start_mobility_control_robot2()

    # ---------- 모드 전환 ----------
    def _toggle_llm_mode(self):
        self._save_arm_gripper_state()

        # LLM -> 키보드
        if self.llm_mode_active:
            print("\n🔄 키보드 모드로 전환...")
            self.llm_mode_active = False
            with self._command_lock:
                self.pending_llm_command = None
            self._restore_mobility_with_saved_state()
            return

        # 키보드 -> LLM
        print("\n🔄 LLM 모드로 전환...")
        self.sim_manager.stop_mobility_control()
        self.sim_manager.stop_mobility_control_robot2()
        self.llm_mode_active = True
        
        #  입력 스레드 시작 (입력만 받음)
        if not self.llm_input_thread or not self.llm_input_thread.is_alive():
            self.llm_input_thread = threading.Thread(target=self._llm_input_handler, daemon=True)
            self.llm_input_thread.start()
        

    # ---------- LLM 입력 처리 (별도 스레드, 입력만 받음) ----------
    def _llm_input_handler(self):
        print("\n💬 자연어로 명령을 입력하세요.")
        
        while self.llm_mode_active:
            try:
                cmd = input("\n💬 명령> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            
            if not self.llm_mode_active:
                break
                
            if not cmd:
                continue
            
            if cmd.lower() in {"q", "quit", "exit"}:
                self.llm_mode_active = False
                break
            
            #  SIMPLIFIED: Just queue the raw command
            with self._command_lock:
                self.pending_llm_command = cmd

    # ---------- LLM 명령 실행 (메인 스레드에서만 실행) ----------
    def _process_pending_llm_commands(self):
        """ 메인 루프: 대기 중인 명령 처리"""
        command_to_execute = None
        
        with self._command_lock:
            if self.pending_llm_command:
                command_to_execute = self.pending_llm_command
                self.pending_llm_command = None
        
        if command_to_execute:
            self._execute_llm_command(command_to_execute)
            self._save_arm_gripper_state()

    def _execute_llm_command(self, command: str) -> bool:
            """ Execute LLM command using planner's analysis methods"""
            # Use Robot1's position as default reference (LLM will determine actual robot)
            cur = (self.sim_manager.data.qpos[0], self.sim_manager.data.qpos[1])

            # 1) Parse command (LLM determines robot_id)
            plan = self.llm_planner.parse_command(command, cur)

            # 2)  Use planner's method to check for multi-robot plan
            if self.llm_planner.is_multi_robot_plan(plan):
                robot_ids = self.llm_planner.get_robot_ids_in_plan(plan)
                print(f"\n🤖 혼합 로봇 계획 감지! {robot_ids} - 계획 분리 중...")
                
                #  Use planner's method to split the plan
                split_plans = self.llm_planner.split_plan_by_robot(plan)
                
                # Display and execute each sub-plan
                for robot_id_str, sub_plan in split_plans.items():
                    robot_num = 1 if robot_id_str == "robot1" else 2
                    executor = self.llm_executor_robot1 if robot_num == 1 else self.llm_executor_robot2
                    robot_name = f"Robot{robot_num}"
                    
                    # Show sub-plan
                    explanation = self.llm_planner.explain_plan(sub_plan)
                    print(f"\n📘 [{robot_name}] 분리된 작업 계획:")
                    print("-" * 50)
                    print(explanation)
                    print("-" * 50)
                    
                    # Execute sub-plan
                    print(f"📋 [{robot_name}] 작업 {len(sub_plan.subtasks)}개 실행 시작...")
                    executor.execute_plan(sub_plan, async_mode=True)
                
                print(f"\n 두 로봇의 계획이 동시에 실행됩니다!")
                return True
            
            # 3) Single robot plan - execute normally
            else:
                robot_ids = self.llm_planner.get_robot_ids_in_plan(plan)
                robot_id_str = list(robot_ids)[0] if robot_ids else "robot1"
                robot_num = 1 if robot_id_str == "robot1" else 2
                executor = self.llm_executor_robot1 if robot_num == 1 else self.llm_executor_robot2
                robot_name = f"Robot{robot_num}"
                
                if not executor:
                    print(f"[{robot_name}] Executor가 없습니다.")
                    return False
                
                # Show plan
                explanation = self.llm_planner.explain_plan(plan)
                print(f"\n📘 [{robot_name}] 생성된 작업 계획:")
                print("-" * 50)
                print(explanation)
                print("-" * 50)
                
                # Execute plan
                print(f"📋 [{robot_name}] 작업 {len(plan.subtasks)}개 실행 시작...")
                ok = executor.execute_plan(plan, async_mode=True)
                return ok

    # ---------- 초기화/루프 ----------
    def _print_header(self):
        print("\n" + "="*60)
        print(" 🤖 MuJoCo Navigation & Pick & Place System")
        print("="*60)

    def _print_controls(self):
        print("\n조작:")
        print("  [Robot1]")
        print("    • W/A/S/D: 이동 (전/좌/후/우)")
        print("    • Q/E: 회전 (좌/우)")
        print("    • C: 정지")
        print("  [Robot2]")
        print("    • Numpad 8/4/5/6: 이동 (전/좌/후/우)")
        print("    • Numpad 7/9: 회전 (좌/우)")
        print("    • Numpad 2: 정지")
        print("  [공통]")
        print("    • F8 : LLM 모드 토글")
        print("    • ESC: 종료")

    def initialize(self):
        self._print_header()
        self._print_controls()

        self.sim_manager.initialize_viewer()
    
        self.sim_manager.initialize_arm_controllers()
        
        # ✅ Initialize general arm holders (for LLM/manual modes)
        self.sim_manager.initialize_arm_holder(robot_id=1)
        self.sim_manager.initialize_arm_holder(robot_id=2)
        
        self.sim_manager.initialize_path_controller(MAP_FILE)
        self.sim_manager.initialize_path_controller_robot2(MAP_FILE)
        
        self.sim_manager.start_mobility_control()
        self.sim_manager.start_mobility_control_robot2()
        
    
    def execute(self):
        """ 메인 실행 루프 - State Machine 기반"""
        
        try:
            while self.sim_manager.viewer_manager.is_running():
                # 0) 키 입력
                self._handle_keyboard_input()

                # 1) LLM mode
                if self.llm_mode_active:
                    # LLM command handling
                    self._process_pending_llm_commands()

                    # Hold arm - Use GENERAL arm holder (not path controller)
                    if not self.is_arm_busy_robot1:
                        if self.sim_manager.arm_holder is not None:
                            torque = self.sim_manager.arm_holder.compute_hold_torque()
                            self.sim_manager.data.ctrl[ARM_CTRL_IDX] = torque
                    if not self.is_arm_busy_robot2:
                        if self.sim_manager.arm_holder_robot2 is not None:
                            torque = self.sim_manager.arm_holder_robot2.compute_hold_torque()
                            self.sim_manager.data.ctrl[ROBOT2_ARM_CTRL_IDX] = torque
                    
                    # Executor update
                    if self.llm_executor_robot1:
                        self.llm_executor_robot1.update_execution()
                    if self.llm_executor_robot2:
                        self.llm_executor_robot2.update_execution()
                        
                    # ✅ Path controller update (async velocity computation)
                    self.sim_manager.update_path_control(self)  # Pass robot_system
                    self.sim_manager.update_path_control_robot2(self)  # Pass robot_system

                # 2) mobility update (in only keyboard mode)
                if not self.llm_mode_active:
                    self.sim_manager.update_mobility_control()

                # 3) physical stepping
                self._step_count += 1
                self.sim_manager.step()
        finally:
            logger.info(f" Simulation complete. Total steps: {self._step_count}")
            self.cleanup()

    # ---------- 키 처리/네비 ----------
    def _handle_keyboard_input(self):
        if self._is_key_pressed("escape"):
            self.llm_mode_active = False
            self.sim_manager.viewer_manager.close()
            return

        # LLM Mode toggle (both '-' and F8 keys)
        if self._is_key_pressed("f8"):
            self._toggle_llm_mode()
            return

        if self.llm_mode_active:
            return
        
    def cleanup(self):
        print("\n시스템 종료...")
        self.llm_mode_active = False
        
        # Stop keyboard listener
        try:
            self.keyboard_listener.stop()
        except Exception:
            pass
        
        # Stop mobility control
        self.sim_manager.stop_mobility_control()
        self.sim_manager.stop_mobility_control_robot2()
        
        # ✅ Shutdown all async workers gracefully
        self.sim_manager.shutdown_all_workers()
        
        # Close viewer
        self.sim_manager.viewer_manager.close()
        
        print("종료 완료")

def main():
    system = NavigationPickPlaceSystem()
    system.initialize()
    system.execute()


if __name__ == "__main__":
    main()
