"""
Place Handler - Non-blocking State Machine
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
        'ik_solver': sim.get_ik_solver(robot_id)
    }


def _get_held_object(executor, robot_id):
    """Get robot-specific held object"""
    attr = f"held_object_{robot_id}"
    return getattr(executor.robot_system, attr, None) if hasattr(executor.robot_system, attr) else None


def _run_trajectory(arm_controller, q_target, state_data, traj_key):
    """
    Run trajectory with automatic start/completion tracking
    Returns: completed (bool)
    """
    started_key = f"{traj_key}_trajectory_started"
    
    # Start trajectory if not yet started
    if not state_data.get(started_key):
        if not arm_controller.start_trajectory(q_target, max_step=100000):
            return False
        state_data[started_key] = True
        return False
    
    # Check completion
    if not arm_controller.is_tracking:
        state_data[started_key] = False
        return True
    
    return False


# ==================== Main Functions ====================

def start_place_sequence(executor, parameters: dict) -> bool:
    """Initialize place sequence (non-blocking)"""
    robot_id = parameters.get("robot_id", "robot1")
    target = parameters.get("target_position")
    
    if not target:
        logger.error(f"[PLACE] [{robot_id}] No target specified")
        return False
    
    # Get resources
    res = _get_robot_resources(executor, robot_id)
    held_object = _get_held_object(executor, robot_id)
    
    # Validate
    if not held_object:
        logger.error(f"[PLACE] [{robot_id}] No object held")
        return False
    
    #  Get FeasibilityChecker from SimulationManager
    checker = res['sim'].get_feasibility_checker(robot_id)
    
    #  Use checker to resolve place position
    place_pos, distance = checker.resolve_place_position(target, held_object)
    
    if place_pos is None:
        logger.error(f"[PLACE] [{robot_id}] Place position failed")
        return False
    
    #  Feasibility check
    feasible, msg = checker.check_position_reachability(place_pos, "Place")
    if not feasible:
        logger.error(f"[PLACE] [{robot_id}] Not reachable: {msg}")
        return False
    
    # Acquire control
    res['sim'].set_arm_busy(executor.robot_system, robot_id, True)
    res['sim'].clear_navigation(executor.robot_system, robot_id)
    
    # Submit async IK requests
    rpy = [np.pi, 0.0, -np.pi / 2.0]
    
    approach_pos = place_pos.copy()
    approach_pos[2] += 0.03
    retreat_pos = approach_pos.copy()
    retreat_pos[2] += 0.30
    
    approach_req = res['ik_solver'].submit_request(approach_pos, rpy, max_iter=500)
    retreat_req = res['ik_solver'].submit_request(retreat_pos, rpy, max_iter=500)
    
    # Store state
    executor.place_state_data = {
        "robot_id": robot_id,
        "target": target,
        "held_object": held_object,
        "place_pos": place_pos,
        "approach_pos": approach_pos,
        "ik_requests": {"approach": approach_req, "retreat": retreat_req},
        "ik_results": {},
        "distance": distance,
        "is_stack": isinstance(target, str),
        "state_start_time": time.time(),
        "simulated": False,
        **res
    }
    
    return True


def check_and_advance_place_state(executor) -> bool:
    """Check and advance place state machine (non-blocking)"""
    
    if not hasattr(executor, 'place_state_data'):
        executor._complete_current_task(success=False, error="No place state")
        return True
    
    state_data = executor.place_state_data
    robot_id = state_data.get("robot_id", "robot1")
    
    # State machine dispatch
    state = executor.executor_state
    handlers = {
        ExecutorState.PLACE_WAITING_FOR_IK: _handle_wait_for_ik,
        ExecutorState.PLACE_APPROACHING: _handle_approaching,
        ExecutorState.PLACE_OPENING_GRIPPER: _handle_opening_gripper,
        ExecutorState.PLACE_RETREATING: _handle_retreating
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
        if results["approach"].status == IKStatus.SUCCESS:
            state_data["approach_q"] = results["approach"].solution
            state_data["retreat_q"] = results["retreat"].solution if results["retreat"].status == IKStatus.SUCCESS else None
            executor.executor_state = ExecutorState.PLACE_APPROACHING
            state_data["state_start_time"] = time.time()
            return True
        else:
            logger.error(f"[PLACE] [{robot_id}] Approach IK failed")
            _cleanup_place(executor, state_data, robot_id)
            executor._complete_current_task(success=False, error="IK failed")
            return True
    
    return False


def _handle_approaching(executor, state_data, robot_id) -> bool:
    """Handle approach trajectory"""
    
    completed = _run_trajectory(
        state_data['arm_controller'],
        state_data["approach_q"],
        state_data,
        "approach"
    )
    
    if completed:
        executor.executor_state = ExecutorState.PLACE_OPENING_GRIPPER
        state_data["state_start_time"] = time.time()
        return True
    
    return False


def _handle_opening_gripper(executor, state_data, robot_id) -> bool:
    """Open gripper instantly"""
    
    state_data["gripper_ctrl"][0] = 0
    gripper_idx = 21 if robot_id == "robot2" else 10
    state_data['sim'].data.ctrl[gripper_idx] = 0
    
    executor.executor_state = ExecutorState.PLACE_RETREATING
    state_data["state_start_time"] = time.time()
    return True


def _handle_retreating(executor, state_data, robot_id) -> bool:
    """Handle retreat trajectory"""
    if state_data["retreat_q"] is None:
        _complete_place_success(executor, state_data, robot_id)
        return True
    
    completed = _run_trajectory(
        state_data['arm_controller'],
        state_data["retreat_q"],
        state_data,
        "retreat"
    )
    
    if completed:
        _complete_place_success(executor, state_data, robot_id)
        return True
    
    return False


def _complete_place_success(executor, state_data, robot_id):
    """Complete place successfully"""
    # Release held object
    attr = f"held_object_{robot_id}"
    if hasattr(executor.robot_system, attr):
        setattr(executor.robot_system, attr, None)
    
    _cleanup_place(executor, state_data, robot_id)
    executor._complete_current_task(success=True, data={
        "position": list(state_data["place_pos"]),
        "object": state_data["held_object"],
        "distance_from_robot": float(state_data["distance"]),
        "robot_id": robot_id
    })


def _cleanup_place(executor, state_data, robot_id):
    """Clean up place operation"""
    sim = state_data['sim']
    
    if sim:
        gripper_ctrl = sim.get_gripper_control(robot_id)
        if gripper_ctrl is not None:
            gripper_ctrl[0] = 0
    
    sim.set_arm_busy(executor.robot_system, robot_id, False)
    
    if hasattr(executor, 'place_state_data'):
        del executor.place_state_data