"""
Navigate Handler - Non-blocking State Machine
Async path planning with multi-robot support
"""
import time
import logging
import numpy as np
from path_planning.async_mpc_planner import PlanStatus
from ..Executor import ExecutorState

logger = logging.getLogger(__name__)


# ==================== Helper Functions ====================

def _get_robot_resources(executor, robot_id):
    """Get robot-specific navigation resources"""
    sim = executor.robot_system.sim_manager
    is_robot2 = (robot_id == "robot2")
    
    return {
        'sim': sim,
        'async_planner': sim.async_planner_robot2 if is_robot2 else sim.async_planner,
        'path_controller': sim.path_controller_robot2 if is_robot2 else sim.path_controller,
        'initialize_func': sim.initialize_path_controller_robot2 if is_robot2 else sim.initialize_path_controller
    }


def _initialize_path_controller(res, robot_id):
    """Initialize path controller if needed"""
    if res['path_controller'] is not None:
        return res['path_controller']
    
    import os
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    candidates = [
        os.path.join(base_dir, "lidar_map_20250826_215447.npz"),
        os.path.join(base_dir, "lidar_mapping", "maps", "lidar_map_20250826_215447.npz"),
    ]
    map_file = next((p for p in candidates if os.path.exists(p)), None)
    
    if map_file:
        res['initialize_func'](map_file)
    else:
        res['initialize_func']()
    
    # Update reference after initialization
    sim = res['sim']
    return sim.path_controller_robot2 if robot_id == "robot2" else sim.path_controller


def _stop_path_control(sim, robot_id):
    """Stop path control for specified robot"""
    if robot_id == "robot2":
        sim.stop_path_control_robot2()
    else:
        sim.stop_path_control()


# ==================== Main Functions ====================

def start_navigate_sequence(executor, parameters: dict) -> bool:
    """Initialize navigation sequence (non-blocking)"""
    robot_id = parameters.get("robot_id", "robot1")
    target_position = parameters.get("target_position")
    
    logger.info(f"[NAVIGATE] [{robot_id}] Starting: {target_position}")
    
    if not target_position:
        logger.error(f"[NAVIGATE] [{robot_id}] No target specified")
        return False

    # Get resources
    res = _get_robot_resources(executor, robot_id)
    path_controller = _initialize_path_controller(res, robot_id)
    
    # Store state
    executor.navigate_state_data = {
        "robot_id": robot_id,
        "target": target_position,
        "path_controller": path_controller,
        "planning_future": None,
        "planning_start_time": None,
        "state_start_time": time.time(),
        **res  # Store all resources (sim, async_planner, etc.)
    }
    
    logger.info(f"[NAVIGATE] [{robot_id}] Initialized")
    return True


def check_and_advance_navigate_state(executor) -> bool:
    """Check and advance navigation state machine (non-blocking)"""
    
    if not hasattr(executor, 'navigate_state_data'):
        executor._complete_current_task(success=False, error="No navigate state")
        return True
    
    state_data = executor.navigate_state_data
    robot_id = state_data.get("robot_id", "robot1")
    
    
    # State machine dispatch
    state = executor.executor_state
    
    if state == ExecutorState.NAVIGATING_WAITING_FOR_PLAN:
        return _handle_waiting_for_plan(executor, state_data, robot_id)
    
    elif state == ExecutorState.NAVIGATING:
        return _handle_navigating(executor, state_data, robot_id)
    
    else:
        logger.error(f"[NAVIGATE] [{robot_id}] Invalid state: {state}")
        executor._complete_current_task(success=False, error=f"Invalid state: {state}")
        return True


# ==================== State Handlers ====================

def _handle_waiting_for_plan(executor, state_data, robot_id) -> bool:
    """Handle path planning phase"""
    
    # Submit planning request if not yet submitted
    if state_data.get("planning_future") is None:
        path_controller = state_data["path_controller"]
        async_planner = state_data["async_planner"]
        target = state_data["target"]
        
        # Get current position
        current_pos, _ = path_controller._get_current_position()
        
        logger.info(f"[NAVIGATE] [{robot_id}] Planning: ({current_pos[0]:.2f}, {current_pos[1]:.2f}) "
                   f"→ ({target[0]:.2f}, {target[1]:.2f})")
        
        # Submit async planning (non-blocking)
        future = async_planner.submit_plan_request(
            start_pos=np.array(current_pos[:2]),
            goal_pos=np.array(target),
            visualize=False
        )
        
        state_data["planning_future"] = future
        state_data["planning_start_time"] = time.time()
        return False
    
    # Poll for planning completion
    async_planner = state_data["async_planner"]
    future = state_data["planning_future"]    
    
    # Check if ready
    if not async_planner.is_ready(future):
        return False  # Still planning
    
    # Get result
    result = async_planner.get_result(future)
    
    if result.status == PlanStatus.SUCCESS:
        logger.info(f"[NAVIGATE] [{robot_id}] Plan ready: {result.num_waypoints} waypoints "
                   f"({result.plan_time:.3f}s)")
        
        # ✅ Send pre-computed path to worker (no re-planning!)
        path_controller = state_data["path_controller"]
        path_list = result.path.tolist()  # Convert numpy to list
        goal_pos = tuple(result.path[-1])
        
        # Send set_path command to worker (efficient - no re-planning)
        success = path_controller.set_path(path_list, goal_pos)
        if not success:
            logger.error(f"[NAVIGATE] [{robot_id}] Failed to send path to worker")
            executor._complete_current_task(success=False, error="Failed to send path to worker")
            return True
        
        # ✅ Set navigation attributes in main process (for is_navigation_complete check)
        path_controller.global_path = path_list
        path_controller.current_goal = goal_pos
        path_controller.current_waypoint_idx = 0
        
        # ✅ Send start_navigation command to worker
        success = path_controller.start_navigation()
        if not success:
            logger.error(f"[NAVIGATE] [{robot_id}] Failed to start navigation in worker")
            executor._complete_current_task(success=False, error="Failed to start navigation")
            return True
        
        # ✅ Set navigation_active in main process (for state tracking)
        path_controller.navigation_active = True
        
        # Transition to navigating
        executor.executor_state = ExecutorState.NAVIGATING
        state_data["navigation_start_time"] = time.time()
        return False
    
    else:
        logger.error(f"[NAVIGATE] [{robot_id}] Planning failed: {result.error_message}")
        executor._complete_current_task(success=False, error=result.error_message)
        return True


def _handle_navigating(executor, state_data, robot_id) -> bool:
    """Handle navigation execution phase"""
    path_controller = state_data["path_controller"]
    waiting_start = state_data.get("waiting_start_time")
    
    # Check if in settling phase
    if waiting_start is not None:
        wait_elapsed = time.time() - waiting_start
        SETTLE_TIME = 4.0
        
        if wait_elapsed < SETTLE_TIME:
            return False  # Still settling
        
        # Settling complete
        logger.info(f"[NAVIGATE] [{robot_id}] Complete")
        executor._complete_current_task(success=True, data={
            "target": state_data["target"], "robot_id": robot_id
        })
        return True
    
    # Check completion
    if path_controller.is_navigation_complete():
        logger.info(f"[NAVIGATE] [{robot_id}] Target reached, settling...")
        _stop_path_control(state_data['sim'], robot_id)
        state_data["waiting_start_time"] = time.time()
        return False
    
    return False  # Still navigating
