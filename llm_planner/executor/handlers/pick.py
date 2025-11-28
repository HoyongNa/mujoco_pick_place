"""
Pick Handler - Non-blocking State Machine
Async IK integration with multi-robot support
"""
import time
import logging
import numpy as np
from kinematics.async_ik_solver import IKStatus
from ..Executor import ExecutorState

logger = logging.getLogger(__name__)


# ==================== Helper Functions ====================

def _get_robot_resources(executor, robot_id):
    """Get robot-specific controllers and indices"""
    sim = executor.robot_system.sim_manager
    return {
        'sim': sim,
        'arm_controller': sim.get_arm_controller(robot_id),
        'gripper_ctrl': sim.get_gripper_control(robot_id),
        'arm_indices': sim.get_arm_indices(robot_id),
        'ik_solver': sim.get_ik_solver(robot_id),
        'grasp_checker': sim.get_grasp_checker(robot_id)
    }


def _run_trajectory(arm_controller, q_target, state_data, traj_key):
    """
    Run trajectory with automatic start/completion tracking
    Returns: completed (bool)
    """
    started_key = f"{traj_key}_trajectory_started"
    
    # Start trajectory if not yet started
    if not state_data.get(started_key):
        if not arm_controller.start_trajectory(q_target, max_step=100000):
            return False  # Failed to start
        state_data[started_key] = True
        return False  # Started, not completed yet
    
    # Check completion
    if not arm_controller.is_tracking:
        state_data[started_key] = False
        return True  # Completed successfully
    
    return False  # Still in progress


# ==================== Main Functions ====================

def start_pick_sequence(executor, parameters: dict) -> bool:
    """Initialize pick sequence (non-blocking)"""
    robot_id = parameters.get("robot_id", "robot1")
    object_name = parameters.get("object_name")
    
    if not object_name:
        logger.error(f"[PICK] [{robot_id}] No object specified")
        return False
    
    # Get robot resources
    res = _get_robot_resources(executor, robot_id)
    
    # # ✅ PAUSE MPC WORKERS - FREE CPU FOR ARM CONTROL
    # logger.info(f"[PICK] [{robot_id}] ⏸️  Pausing MPC workers...")
    # try:
    #     if robot_id == 'robot2':
    #         if res['sim'].path_controller_robot2:
    #             res['sim'].path_controller_robot2.pause_worker()
    #     else:  # robot1
    #         if res['sim'].path_controller:
    #             res['sim'].path_controller.pause_worker()
    # except Exception as e:
    #     logger.warning(f"[PICK] [{robot_id}] ⚠️ MPC pause error: {e}")
    
    #  Get FeasibilityChecker from SimulationManager
    checker = res['sim'].get_feasibility_checker(robot_id)
    
    #  Use checker to resolve pick position
    pick_pos, box_name = checker.resolve_pick_position(object_name)
    
    #  Feasibility check
    feasible, msg = checker.check_position_reachability(pick_pos, "Pick")
    if not feasible:
        logger.error(f"[PICK] [{robot_id}] Not reachable: {msg}")
        return False
    
    safe, safety_msg, _ = checker.check_surrounding_boxes_safety(box_name, pick_pos, 0.3)
    if not safe:
        logger.error(f"[PICK] [{robot_id}] Unsafe: {safety_msg}")
        return False
    
    # Acquire control
    res['sim'].set_arm_busy(executor.robot_system, robot_id, True)
    res['sim'].clear_navigation(executor.robot_system, robot_id)
    
    # Submit async IK requests
    rpy = [np.pi, 0.0, -np.pi / 2.0]
    approach_pos = pick_pos.copy()
    approach_pos[2] += 0.20
    grasp_pos = pick_pos.copy()
    grasp_pos[2] = pick_pos[2] + 0.01
    lift_pos = grasp_pos.copy()
    lift_pos[2] += 0.4
    
    approach_req = res['ik_solver'].submit_request(approach_pos, rpy, max_iter=500)
    grasp_req = res['ik_solver'].submit_request(grasp_pos, rpy, max_iter=500)
    lift_req = res['ik_solver'].submit_request(lift_pos, rpy, max_iter=500)
    
    # Store state
    executor.pick_state_data = {
        "robot_id": robot_id,
        "object_name": object_name,
        "pick_pos": pick_pos,
        "box_name": box_name,
        "grasp_pos": grasp_pos,
        "ik_requests": {"approach": approach_req, "grasp": grasp_req, "lift": lift_req},
        "ik_results": {},
        "state_start_time": time.time(),
        "simulated": False,
        **res  # Store all resources
    }
    
    return True


def check_and_advance_pick_state(executor) -> bool:
    """Check and advance pick state machine (non-blocking)"""
    
    if not hasattr(executor, 'pick_state_data'):
        executor._complete_current_task(success=False, error="No pick state")
        return True
    
    state_data = executor.pick_state_data
    robot_id = state_data.get("robot_id", "robot1")

    
    # State machine dispatch
    state = executor.executor_state
    handlers = {
        ExecutorState.PICK_WAITING_FOR_IK: _handle_wait_for_ik,
        ExecutorState.PICK_OPENING_GRIPPER: _handle_opening_gripper,
        ExecutorState.PICK_APPROACHING: _handle_approaching,
        ExecutorState.PICK_GRASPING: _handle_grasping,
        ExecutorState.PICK_CLOSING_GRIPPER: _handle_closing_gripper,
        ExecutorState.PICK_LIFTING: _handle_lifting
    }
    
    handler = handlers.get(state)
    if handler:
        return handler(executor, state_data, robot_id)
    
    return False


# ==================== State Handlers ====================

def _handle_wait_for_ik(executor, state_data, robot_id) -> bool:
    """Wait for async IK results"""
    
    ik_solver = state_data['ik_solver']
    requests = state_data["ik_requests"]
    results = state_data["ik_results"]
    
    # Check ready results (non-blocking)
    for name, req_id in requests.items():
        if name not in results and ik_solver.is_ready(req_id):
            results[name] = ik_solver.get_result(req_id)
    
    # All ready?
    if len(results) == len(requests):
        if all(r.status == IKStatus.SUCCESS for r in results.values()):
            state_data["approach_q"] = results["approach"].solution
            state_data["grasp_q"] = results["grasp"].solution
            state_data["lift_q"] = results["lift"].solution
            
            # ✅ Keep IK solver alive - needed for place operations!
            # IK workers are idle after solving, minimal CPU impact
            
            executor.executor_state = ExecutorState.PICK_OPENING_GRIPPER
            state_data["state_start_time"] = time.time()
            return True
        else:
            failed = [n for n, r in results.items() if r.status != IKStatus.SUCCESS]
            logger.error(f"[PICK] [{robot_id}] IK failed: {failed}")
            _cleanup_pick(executor, state_data, robot_id)
            executor._complete_current_task(success=False, error=f"IK failed: {failed}")
            return True
    
    return False


def _handle_opening_gripper(executor, state_data, robot_id) -> bool:
    """Open gripper instantly"""
    
    state_data["gripper_ctrl"][0] = 0
    executor.executor_state = ExecutorState.PICK_APPROACHING
    state_data["state_start_time"] = time.time()
    return True


def _handle_approaching(executor, state_data, robot_id) -> bool:
    """Handle approach trajectory"""
    
    completed = _run_trajectory(
        state_data['arm_controller'], 
        state_data["approach_q"],
        state_data, 
        "approach"
    )
    
    if completed:
        executor.executor_state = ExecutorState.PICK_GRASPING
        state_data["state_start_time"] = time.time()
        return True
    
    return False


def _handle_grasping(executor, state_data, robot_id) -> bool:
    """Handle grasp trajectory"""
    
    completed = _run_trajectory(
        state_data['arm_controller'],
        state_data["grasp_q"],
        state_data,
        "grasp"
    )
    
    if completed:
        executor.executor_state = ExecutorState.PICK_CLOSING_GRIPPER
        state_data["state_start_time"] = time.time()
        state_data["grasp_check_start_time"] = None
        return True
    
    return False


def _handle_closing_gripper(executor, state_data, robot_id) -> bool:
    """Close gripper and verify grasp"""
    
    if state_data.get("grasp_check_start_time") is None:
        state_data["gripper_ctrl"][0] = 255
        state_data["grasp_check_start_time"] = time.time()
    
    # Quick grasp check
    is_grasping, max_force, _ = state_data['grasp_checker'].check_grasp_state(
        force_threshold=0.5, min_pad_distance=0.025
    )
    
    elapsed = time.time() - state_data["grasp_check_start_time"]
    
    if is_grasping or elapsed > 0.2:
        executor.executor_state = ExecutorState.PICK_LIFTING
        state_data["state_start_time"] = time.time()
        return True
    
    return False


def _handle_lifting(executor, state_data, robot_id) -> bool:
    """Handle lift trajectory"""
    if state_data["lift_q"] is None:
        _complete_pick_success(executor, state_data, robot_id)
        return True
    
    completed = _run_trajectory(
        state_data['arm_controller'],
        state_data["lift_q"],
        state_data,
        "lift"
    )
    
    if completed:
        _complete_pick_success(executor, state_data, robot_id)
        return True
    
    return False


def _complete_pick_success(executor, state_data, robot_id):
    """Complete pick successfully"""
    # Update held object
    if robot_id == "robot2":
        executor.robot_system.held_object_robot2 = state_data["object_name"]
    else:
        executor.robot_system.held_object_robot1 = state_data["object_name"]
    
    # ✅ IK solver will be recreated on next use (lazy initialization)
    # No need to recreate explicitly - AsyncIKSolver handles this
    
    _cleanup_pick(executor, state_data, robot_id)
    executor._complete_current_task(success=True, data={
        "object": state_data["object_name"],
        "pick_position": list(state_data["pick_pos"]),
        "robot_id": robot_id
    })


def _cleanup_pick(executor, state_data, robot_id):
    """Clean up pick operation"""
    
    # # ✅ RESUME MPC WORKERS - RESTORE NAVIGATION
    # logger.info(f"[PICK] [{robot_id}] ▶️  Resuming MPC workers...")

    # try:
    #     if robot_id == 'robot2':
    #         if state_data['sim'].path_controller_robot2:
    #             state_data['sim'].path_controller_robot2.resume_worker()
    #     else:
    #         if state_data['sim'].path_controller:
    #             state_data['sim'].path_controller.resume_worker()
    # except Exception as e:
    #     logger.warning(f"[PICK] [{robot_id}] ⚠️ MPC resume error: {e}")
    
    state_data['sim'].set_arm_busy(executor.robot_system, robot_id, False)
    if hasattr(executor, 'pick_state_data'):
        del executor.pick_state_data