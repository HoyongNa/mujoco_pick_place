"""
Async MPC Path Planner - Multiprocessing Version (Pure Async)
Focuses exclusively on asynchronous path planning

Author: Hoyon
Date: November 26, 2025
"""

import numpy as np
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, Future
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple
import time
import logging

logger = logging.getLogger(__name__)


class PlanStatus(Enum):
    """Path planning status"""
    SUCCESS = "success"
    FAILED = "failed"
    IN_PROGRESS = "in_progress"
    TIMEOUT = "timeout"


@dataclass
class PlanResult:
    """Path planning result"""
    status: PlanStatus
    path: Optional[np.ndarray]  # Shape: (N, 2) for 2D path
    plan_time: float
    error_message: Optional[str] = None
    num_waypoints: int = 0


# ==================== Worker Function (Runs in Separate Process) ====================

def _compute_path_worker(
    request_id: int,
    start_pos: np.ndarray,
    goal_pos: np.ndarray,
    visualize: bool,
    map_config: dict
) -> PlanResult:
    """
    Worker function: Compute path (runs in separate PROCESS)
    """
    start_time = time.time()
    
    try:
        print(f"[AsyncMPCPlanner-Process-{request_id}] 🚀 Computing path from {start_pos} to {goal_pos}...")
        print(f"[AsyncMPCPlanner-Process-{request_id}] Running in SEPARATE PROCESS (no GIL contention!)")
        
        # ✅ Recreate MapProcessor and AStarPlanner in worker process
        from path_planning.map_processor import MapProcessor
        from path_planning.astar_planner import AStarPlanner
        
        # Validate map data
        if 'occupancy_grid' not in map_config or map_config['occupancy_grid'] is None:
            print(f"[AsyncMPCPlanner-Process-{request_id}] ⚠️ No map data in config!")
            return PlanResult(
                status=PlanStatus.FAILED,
                path=None,
                plan_time=time.time() - start_time,
                error_message="No map data available"
            )
        
        # Recreate MapProcessor with serialized data
        map_processor = MapProcessor()
        map_processor.occupancy_grid = map_config['occupancy_grid']
        map_processor.dilated_map = map_config.get('dilated_grid')
        map_processor.dilated_grid = map_config.get('dilated_grid')
        map_processor.resolution = map_config['resolution']
        map_processor.origin = map_config['origin']
        map_processor.width = map_config['width']
        map_processor.height = map_config['height']
        map_processor.map_shape = (map_config['height'], map_config['width'])
        map_processor.loaded = True
        
        print(f"[AsyncMPCPlanner-Process-{request_id}] Map restored: {map_processor.width}x{map_processor.height}")
        
        # Create AStarPlanner with MapProcessor
        planner = AStarPlanner(map_processor)
        
        # Compute path using A*
        path = planner.plan(
            start=tuple(start_pos),
            goal=tuple(goal_pos),
            use_dilated=True,
            simplify=True,
            use_interpolation=True
        )
        
        plan_time = time.time() - start_time
        
        if path is not None and len(path) > 0:
            print(f"[AsyncMPCPlanner-Process-{request_id}] ✅ Path computed "
                  f"({len(path)} waypoints, {plan_time:.3f}s)")
            
            return PlanResult(
                status=PlanStatus.SUCCESS,
                path=np.array(path),
                plan_time=plan_time,
                num_waypoints=len(path)
            )
        
        print(f"[AsyncMPCPlanner-Process-{request_id}] ❌ Planning failed - no valid path")
        return PlanResult(
            status=PlanStatus.FAILED,
            path=None,
            plan_time=plan_time,
            error_message="Path planning failed - no valid path found"
        )
        
    except Exception as e:
        plan_time = time.time() - start_time
        print(f"[AsyncMPCPlanner-Process-{request_id}] ❌ Error during planning: {e}")
        import traceback
        traceback.print_exc()
        
        return PlanResult(
            status=PlanStatus.FAILED,
            path=None,
            plan_time=plan_time,
            error_message=str(e)
        )


# ==================== Main Planner Class ====================

class AsyncMPCPlanner:
    """
    Asynchronous MPC Path Planner - PURE ASYNC MODE
    
    - Tries multiprocessing first (best performance)
    - Falls back to threading if map serialization fails
    - All operations are asynchronous (non-blocking)
    
    Usage:
        planner = AsyncMPCPlanner(mpc_controller, max_workers=2)
        future = planner.submit_plan_request(start_pos, goal_pos)
        
        # Check if ready (non-blocking)
        if planner.is_ready(future):
            result = planner.get_result(future)
        
        # Or wait for result (blocking with timeout)
        result = planner.wait_for_result(future, timeout=2.0)
    """
    
    def __init__(self, mpc_controller, max_workers: int = 2):
        """
        Initialize async MPC planner with automatic mode selection
        
        Args:
            mpc_controller: The MPC controller (MPCHybridControllerACODOS)
            max_workers: Number of persistent planning workers
        """
        self.mpc_controller = mpc_controller
        self.max_workers = max_workers
        self._shutdown = False
        self._request_counter = 0
        
        # ✅ Try to extract map config for multiprocessing
        self.map_config, self.can_use_multiprocessing = self._try_extract_map_config(mpc_controller)
        
        # ✅ Use multiprocessing (TRUE parallelism, no GIL!)
        print(f"[AsyncMPCPlanner] ✅ Using MULTIPROCESSING mode")
        print(f"[AsyncMPCPlanner] 🚀 TRUE parallel execution - NO GIL contention!")
        self.executor = ProcessPoolExecutor(max_workers=max_workers)
        self.use_multiprocessing = True

    
    def _try_extract_map_config(self, mpc_controller) -> tuple[dict, bool]:
        """
        Try to extract MapProcessor configuration
        
        Returns:
            (config_dict, can_use_multiprocessing)
        """
        config = {}
        
        try:
            if not hasattr(mpc_controller, 'map_processor'):
                print(f"[AsyncMPCPlanner] ⚠️ No map_processor attribute found")
                return config, False
            
            map_proc = mpc_controller.map_processor
            
            # Check if map is loaded
            if not hasattr(map_proc, 'loaded') or not map_proc.loaded:
                print(f"[AsyncMPCPlanner] ⚠️ Map not loaded yet (loaded={getattr(map_proc, 'loaded', False)})")
                return config, False
            
            # Check if occupancy grid exists
            if not hasattr(map_proc, 'occupancy_grid') or map_proc.occupancy_grid is None:
                print(f"[AsyncMPCPlanner] ⚠️ No occupancy_grid found")
                return config, False
            
            # Extract all MapProcessor attributes
            config['resolution'] = getattr(map_proc, 'resolution', 0.1)
            config['origin'] = getattr(map_proc, 'origin', [0.0, 0.0])
            config['width'] = getattr(map_proc, 'width', 0)
            config['height'] = getattr(map_proc, 'height', 0)
            
            # Extract grids (numpy arrays are serializable)
            config['occupancy_grid'] = np.array(map_proc.occupancy_grid)
            
            if hasattr(map_proc, 'dilated_grid') and map_proc.dilated_grid is not None:
                config['dilated_grid'] = np.array(map_proc.dilated_grid)
            else:
                config['dilated_grid'] = None
            
            print(f"[AsyncMPCPlanner] ✅ Map config extracted: {config['width']}x{config['height']}, "
                  f"res={config['resolution']}, grid_shape={config['occupancy_grid'].shape}")
            
            return config, True
            
        except Exception as e:
            print(f"[AsyncMPCPlanner] ⚠️ Error extracting map config: {e}")
            import traceback
            traceback.print_exc()
            return config, False
    
    def submit_plan_request(
        self, 
        start_pos: np.ndarray, 
        goal_pos: np.ndarray,
        visualize: bool = False
    ) -> Future:
        """
        Submit a planning request (ASYNC - non-blocking)
        
        Works with both multiprocessing and threading modes.
        Returns immediately with a Future object.
        
        Args:
            start_pos: Starting position (x, y)
            goal_pos: Goal position (x, y)
            visualize: Whether to visualize the path (currently ignored in async mode)
        
        Returns:
            Future object for tracking request status
        """
        if self._shutdown:
            raise RuntimeError("AsyncMPCPlanner has been shut down")
        
        self._request_counter += 1
        request_id = self._request_counter
        
        print(f"[AsyncMPCPlanner] Submitting async plan request #{request_id}: "
              f"{start_pos} → {goal_pos}")
        
        if self.use_multiprocessing:
            # Multiprocessing mode
            future = self.executor.submit(
                _compute_path_worker,
                request_id,
                start_pos.copy(),
                goal_pos.copy(),
                visualize,
                self.map_config
            )
        else:
            # Threading mode (fallback)
            future = self.executor.submit(
                self._compute_path_thread,
                request_id,
                start_pos.copy(),
                goal_pos.copy(),
                visualize
            )
        
        return future
    
    def is_ready(self, future: Future) -> bool:
        """
        Check if planning is complete (non-blocking)
        
        Args:
            future: Future object from submit_plan_request
        
        Returns:
            True if result is available, False otherwise
        """
        return future.done()
    
    def get_result(self, future: Future, timeout: Optional[float] = None) -> PlanResult:
        """
        Get planning result (blocking if not ready)
        
        Args:
            future: Future object from submit_plan_request
            timeout: Maximum wait time in seconds (None = wait forever)
        
        Returns:
            PlanResult with status and path
        """
        try:
            result = future.result(timeout=timeout)
            return result
        except TimeoutError:
            return PlanResult(
                status=PlanStatus.TIMEOUT,
                path=None,
                plan_time=timeout if timeout else 0.0,
                error_message="Planning timeout"
            )
        except Exception as e:
            print(f"[AsyncMPCPlanner] ❌ Error getting result: {e}")
            import traceback
            traceback.print_exc()
            return PlanResult(
                status=PlanStatus.FAILED,
                path=None,
                plan_time=0.0,
                error_message=f"Error getting result: {e}"
            )
    
    def wait_for_result(self, future: Future, timeout: float = 5.0) -> PlanResult:
        """
        Wait for result with timeout (blocking)
        
        Convenience method that wraps get_result with explicit timeout.
        
        Args:
            future: Future object from submit_plan_request
            timeout: Maximum wait time in seconds
        
        Returns:
            PlanResult when available, or timeout result
        """
        return self.get_result(future, timeout=timeout)
    
    def shutdown(self, timeout: float = 5.0):
        """
        Gracefully shutdown workers
        
        Args:
            timeout: Maximum time to wait for workers to finish
        """
        if self._shutdown:
            return
        
        mode = "processes" if self.use_multiprocessing else "threads"
        print(f"[AsyncMPCPlanner] Shutting down worker {mode}...")
        self._shutdown = True
        
        if self.executor:
            self.executor.shutdown(wait=True, cancel_futures=True)
            print(f"[AsyncMPCPlanner] ✅ Worker {mode} terminated")
    
    def __del__(self):
        """Cleanup on deletion"""
        try:
            self.shutdown(timeout=1.0)
        except:
            pass
