"""
TaskExecutor: 실행 루프/콜백/결과 수집 (State Machine 방식)
 메인 스레드에서 작업을 state로 관리하여 blocking 없이 실행
 Fully non-blocking for LLM mode
"""
import time
import threading
import logging
from typing import Optional, Callable, Any, List
from enum import Enum

from .status import ExecutionStatus
from .result import ExecutionResult

logger = logging.getLogger(__name__)


class ExecutorState(Enum):
    """실행 상태"""
    IDLE = "idle"
    NAVIGATING_WAITING_FOR_PLAN = "navigating_waiting_for_plan" 
    NAVIGATING = "navigating"  #
    #  Pick 상태 세분화 (non-blocking, async IK)
    PICK_WAITING_FOR_IK = "pick_waiting_for_ik"  #
    PICK_OPENING_GRIPPER = "pick_opening_gripper"
    PICK_APPROACHING = "pick_approaching"
    PICK_GRASPING = "pick_grasping"
    PICK_CLOSING_GRIPPER = "pick_closing_gripper"
    PICK_LIFTING = "pick_lifting"
    #  Place 상태 세분화 (non-blocking, async IK)
    PLACE_WAITING_FOR_IK = "place_waiting_for_ik"  
    PLACE_APPROACHING = "place_approaching"
    PLACE_OPENING_GRIPPER = "place_opening_gripper"
    PLACE_RETREATING = "place_retreating"
    COMPLETED = "completed"
    FAILED = "failed"


def _task_key(task_type) -> str:
    """TaskType/Enum/str 모두 소문자 문자열로 정규화"""
    if hasattr(task_type, "name"):
        return str(task_type.name).lower()
    if hasattr(task_type, "value"):
        return str(task_type.value).lower()
    return str(task_type).lower()


class TaskExecutor:
    """작업 실행 관리자 (State Machine 기반)"""
    on_task_start: Optional[Callable[[Any], None]] = None
    on_task_complete: Optional[Callable[[Any], None]] = None
    on_task_failed: Optional[Callable[[Any, str], None]] = None
    on_progress: Optional[Callable[[int, int, str], None]] = None
    on_plan_complete: Optional[Callable[[Any], None]] = None
    on_plan_failed: Optional[Callable[[Any, str], None]] = None

    def __init__(self, robot_system=None, robot_id: int = 1):
        self.robot_system = robot_system
        self.robot_id = robot_id  
        self.is_executing = False
        self.current_plan: Optional[Any] = None
        self.current_task_index = 0
        self.execution_thread: Optional[threading.Thread] = None
        self.cancel_requested = False
        self.execution_results: List[ExecutionResult] = []
        
        #  State Machine
        self.executor_state = ExecutorState.IDLE
        self.current_task = None
        self.task_start_time = 0.0

        if robot_system and hasattr(robot_system, "sim_manager"):
            logger.info("TaskExecutor 초기화 (sim_manager OK)")
        elif robot_system:
            logger.warning("TaskExecutor 초기화 (sim_manager 없음)")
        else:
            logger.info("TaskExecutor 초기화: 로봇 시스템 없음")

    # ---- 외부 API ----
    def execute_plan(self, plan: Any, async_mode: bool = True) -> bool:
        """ State machine 방식으로 실행"""
        if self.is_executing:
            logger.warning("이미 작업이 실행 중입니다")
            return False

        self.current_plan = plan
        self.current_task_index = 0
        self.execution_results.clear()
        self.cancel_requested = False
        self.is_executing = True
        self.executor_state = ExecutorState.IDLE

        logger.info(f"작업 계획 실행 시작: '{getattr(plan, 'original_command', '')}'")
        logger.info(f" State Machine 모드: 메인 루프에서 update_execution() 호출 필요")
        
        return True

    def update_execution(self) -> bool:
        """
        메인 루프에서 매 프레임 호출
        현재 작업 상태를 업데이트하고 다음 작업으로 진행
        
        Returns:
            bool: 실행이 계속 진행 중이면 True, 완료/실패하면 False
        """

        
        # Removed excessive NAV DETAIL logging
        
        # FIX: 항상 모든 arm controller 업데이트 (상태와 무관)
        self._update_all_arm_controllers()
        
        if not self.is_executing:
            # Rate limit this warning to once per 5 seconds
            if not hasattr(self, '_last_warning_time'):
                self._last_warning_time = 0
            # if time.time() - self._last_warning_time > 5.0:
                # logger.warning(f" [Executor {self.robot_id}] update_execution() called but is_executing=False! Returning False.")
                self._last_warning_time = time.time()
            return False
        
        if self.cancel_requested:
            logger.info("작업 실행 취소됨")
            self.is_executing = False
            return False
        
        # 현재 작업이 없으면 다음 작업 시작
        if self.executor_state == ExecutorState.IDLE:
            if self.current_task_index >= len(self.current_plan.subtasks):
                # 모든 작업 완료
                self._finish_execution(success=True)
                return False
            
            # 다음 작업 시작
            self._start_next_task()
            return True
        
        # 현재 작업 상태 확인
        #  Navigation 상태들 처리 ( NAVIGATING_WAITING_FOR_PLAN, NAVIGATING)
        if self.executor_state in [ExecutorState.NAVIGATING_WAITING_FOR_PLAN, ExecutorState.NAVIGATING]:
            if self._check_navigate_state_complete():
                # 다음 navigate 단계 또는 완료로 진행
                pass
            return True
        
        #  Pick 상태들 처리 ( PICK_WAITING_FOR_IK 추가)
        if self.executor_state in [ExecutorState.PICK_WAITING_FOR_IK, ExecutorState.PICK_OPENING_GRIPPER, 
                                    ExecutorState.PICK_APPROACHING, ExecutorState.PICK_GRASPING, 
                                    ExecutorState.PICK_CLOSING_GRIPPER, ExecutorState.PICK_LIFTING]:
            if self._check_pick_state_complete():
                # 다음 pick 단계 또는 완료로 진행
                pass
            return True
        
        #  Place 상태들 처리 ( PLACE_WAITING_FOR_IK 추가)
        if self.executor_state in [ExecutorState.PLACE_WAITING_FOR_IK, ExecutorState.PLACE_APPROACHING, 
                                    ExecutorState.PLACE_OPENING_GRIPPER, ExecutorState.PLACE_RETREATING]:
            if self._check_place_state_complete():
                # 다음 place 단계 또는 완료로 진행
                pass
            return True
        
        return True

    def _start_next_task(self):
        """다음 작업 시작"""
        if self.current_task_index >= len(self.current_plan.subtasks):
            return
        
        task = self.current_plan.subtasks[self.current_task_index]
        self.current_task = task
        self.task_start_time = time.time()
        
        #  DEBUG: Log task details
        task_robot_id = task.parameters.get("robot_id", "unknown")
        logger.info(f"====== STARTING NEW TASK [Executor {self.robot_id}] ======")
        logger.info(f"작업 시작: {task.description}")
        logger.info(f"Task type: {task.task_type}")
        logger.info(f"Task robot_id: {task_robot_id}")
        logger.info(f"Executor robot_id: {self.robot_id}")
        logger.info(f"Task parameters: {task.parameters}")
        logger.info(f"=====================================================")
        
        if self.on_task_start:
            self.on_task_start(task)
        
        if self.on_progress:
            self.on_progress(self.current_task_index, len(self.current_plan.subtasks), task.description)
        
        #  CRITICAL FIX: 작업 타입에 따라 STATE MACHINE 메서드 호출 (핸들러 직접 호출 금지!)
        key = _task_key(task.task_type)
        
        logger.info(f" [Executor {self.robot_id}] Dispatching task type: {key}")
        
        if key == "navigate":
            logger.info(f" [Executor {self.robot_id}] Using STATE MACHINE: _start_navigation()")
            self._start_navigation(task)
        elif key == "pick":
            logger.info(f" [Executor {self.robot_id}] Using STATE MACHINE: _execute_pick()")
            self._execute_pick(task)
        elif key == "place":
            logger.info(f" [Executor {self.robot_id}] Using STATE MACHINE: _execute_place()")
            self._execute_place(task)
        else:
            logger.error(f" [Executor {self.robot_id}] Unsupported task type: {key}")
            self._complete_current_task(success=False, error=f"지원하지 않는 작업: {key}")

    def _start_navigation(self, task):
        """ Navigation 실행 (non-blocking) - 첫 단계 시작"""
        from .handlers.navigate import start_navigate_sequence
        
        # Navigation 시퀀스 초기화
        success = start_navigate_sequence(self, task.parameters)
        
        if not success:
            self._complete_current_task(success=False, data={"error": "Navigation 시퀀스 시작 실패"})
            return
        
        # 첫 단계로 전환 ( NAVIGATING_WAITING_FOR_PLAN로 시작)
        self.executor_state = ExecutorState.NAVIGATING_WAITING_FOR_PLAN
        self.navigate_start_time = time.time()
    
    def _check_navigate_state_complete(self) -> bool:
        """ Navigation 상태 전환 검사"""
        from .handlers.navigate import check_and_advance_navigate_state
        return check_and_advance_navigate_state(self)

    def _update_all_arm_controllers(self):
        """
         모든 로봇의 arm controller를 업데이트
        - 활성 로봇: 궤적 추종
        - 비활성 로봇: 현재 위치 유지(Passive Hold)
        
        이 메서드는 매 프레임 호출되어 모든 로봇의 팔이 제어됨을 보장합니다.
        """
        if not self.robot_system or not hasattr(self.robot_system, 'sim_manager'):
            return
        
        sim = self.robot_system.sim_manager
        
        #  Robot1 arm controller 업데이트
        if sim.arm_controller:
            sim.arm_controller.update()
        
        #  Robot2 arm controller 업데이트
        if sim.arm_controller_robot2:
            sim.arm_controller_robot2.update()
    
    def _execute_pick(self, task):
        """ Pick 실행 (non-blocking) - 첫 단계 시작"""
        from .handlers.pick import start_pick_sequence
        
        # Pick 시퀀스 초기화
        success = start_pick_sequence(self, task.parameters)
        
        if not success:
            self._complete_current_task(success=False, data={"error": "Pick 시퀀스 시작 실패"})
            return
        
        # 첫 단계로 전환 ( PICK_WAITING_FOR_IK로 시작)
        self.executor_state = ExecutorState.PICK_WAITING_FOR_IK
        self.pick_start_time = time.time()

    def _execute_place(self, task):
        """ Place 실행 (non-blocking) - 첫 단계 시작"""
        from .handlers.place import start_place_sequence
        
        # Place 시퀀스 초기화
        success = start_place_sequence(self, task.parameters)
        
        if not success:
            self._complete_current_task(success=False, data={"error": "Place 시퀀스 시작 실패"})
            return
        
        # 첫 단계로 전환 ( PLACE_WAITING_FOR_IK로 시작)
        self.executor_state = ExecutorState.PLACE_WAITING_FOR_IK
        self.place_start_time = time.time()

    def _check_pick_state_complete(self) -> bool:
        """ Pick 상태 전환 검사"""
        from .handlers.pick import check_and_advance_pick_state
        return check_and_advance_pick_state(self)
    
    def _check_place_state_complete(self) -> bool:
        """ Place 상태 전환 검사"""
        from .handlers.place import check_and_advance_place_state
        return check_and_advance_place_state(self)
    

    def _complete_current_task(self, success: bool, data: dict = None, error: str = None):
        """현재 작업 완료 처리"""
        logger.info(f" [Executor {self.robot_id}] _complete_current_task() called: success={success}, current_task={self.current_task.description if self.current_task else None}, error={error}")
        
        if not self.current_task:
            logger.warning(f" [Executor {self.robot_id}] _complete_current_task() called but no current_task!")
            return
        
        end_time = time.time()
        
        if success:
            if self.on_task_complete:
                self.on_task_complete(self.current_task)
            result = ExecutionResult(
                task=self.current_task,
                status=ExecutionStatus.SUCCESS,
                start_time=self.task_start_time,
                end_time=end_time,
                data=data or {},
            )
        else:
            err_msg = error or (data or {}).get("error", "알 수 없는 오류")
            if self.on_task_failed:
                self.on_task_failed(self.current_task, err_msg)
            result = ExecutionResult(
                task=self.current_task,
                status=ExecutionStatus.FAILED,
                start_time=self.task_start_time,
                end_time=end_time,
                error_message=err_msg,
                data=data or {},
            )
        
        self.execution_results.append(result)
        
        if not success:
            # 실패 시 전체 계획 중단
            self._finish_execution(success=False)
        else:
            # 다음 작업으로
            self.current_task_index += 1
            self.current_task = None
            self.executor_state = ExecutorState.IDLE

    def _finish_execution(self, success: bool):
        """전체 계획 실행 완료"""
        logger.info(f"🏁🏁 [Executor {self.robot_id}] _finish_execution() called: success={success}, task_index={self.current_task_index}, total_tasks={len(self.current_plan.subtasks) if self.current_plan else 0}")
        
        #  Print execution results summary
        self._print_execution_summary(success)
        
        if success:
            total_time = time.time() - self.task_start_time
            logger.info(f"작업 계획 완료: {total_time:.1f}초 소요")
            if self.on_plan_complete:
                self.on_plan_complete(self.current_plan)
        else:
            if self.on_plan_failed:
                last_result = self.execution_results[-1] if self.execution_results else None
                error = last_result.error_message if last_result else "Unknown error"
                self.on_plan_failed(self.current_plan, error)
        
        logger.critical(f" [Executor {self.robot_id}] EXECUTION STOPPED: is_executing set to False in _finish_execution()")
        self.is_executing = False
        self.executor_state = ExecutorState.IDLE
        self.current_plan = None
        self.current_task = None

    def _print_execution_summary(self, overall_success: bool):
        """실행 결과 요약 출력"""
        if not self.execution_results:
            logger.info("=" * 80)
            logger.info(f" [Executor {self.robot_id}] EXECUTION SUMMARY: No tasks executed")
            logger.info("=" * 80)
            return
        
        # Count successes and failures
        success_count = sum(1 for r in self.execution_results if r.status == ExecutionStatus.SUCCESS)
        failure_count = sum(1 for r in self.execution_results if r.status == ExecutionStatus.FAILED)
        total_count = len(self.execution_results)
        
        # Calculate total time
        total_duration = sum(r.duration for r in self.execution_results)
        
        # Print summary header
        logger.info("=" * 80)
        logger.info(f" [Executor {self.robot_id}] EXECUTION SUMMARY")
        logger.info("=" * 80)
        logger.info(f"Overall Status: {' SUCCESS' if overall_success else ' FAILED'}")
        logger.info(f"Total Tasks: {total_count} | Success: {success_count} | Failed: {failure_count}")
        logger.info(f"Total Duration: {total_duration:.2f}s")
        logger.info("-" * 80)
        
        # Print individual task results
        for idx, result in enumerate(self.execution_results, 1):
            status_icon = "" if result.status == ExecutionStatus.SUCCESS else ""
            task_desc = result.task.description if hasattr(result.task, 'description') else str(result.task)
            task_type = result.task.task_type if hasattr(result.task, 'task_type') else "unknown"
            
            logger.info(f"{status_icon} Task {idx}/{total_count}: {task_desc}")
            logger.info(f"   Type: {task_type} | Duration: {result.duration:.2f}s | Status: {result.status.value}")
            
            if result.error_message:
                logger.info(f"    Error: {result.error_message}")
            
            if result.data:
                # Print relevant data (avoid printing too much)
                data_summary = {}
                for key, value in result.data.items():
                    if isinstance(value, (int, float, str, bool)):
                        data_summary[key] = value
                    elif isinstance(value, (list, tuple)) and len(value) <= 3:
                        data_summary[key] = value
                if data_summary:
                    logger.info(f"    Data: {data_summary}")
        
        logger.info("=" * 80)
