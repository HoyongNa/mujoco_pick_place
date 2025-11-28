"""
Async IK Solver - Multiprocessing Version (FIXED v2)
Proper MuJoCo model serialization with validation and fallback to threading if needed

Author: Hoyon
Date: November 26, 2025
"""

import numpy as np
import mujoco
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, Future
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import time
import logging
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)


class IKStatus(Enum):
    """Status of an IK request"""
    SUCCESS = "success"
    FAILED = "failed"
    IN_PROGRESS = "in_progress"
    TIMEOUT = "timeout"


@dataclass
class IKResult:
    """IK solve result"""
    status: IKStatus
    solution: Optional[np.ndarray]  # Joint angles solution
    error: Optional[float]  # Final error value
    iterations: int
    solve_time: float
    message: str = ""


# ==================== Worker Function (Runs in Separate Process) ====================

def _solve_ik_worker(
    request_id: int,
    target_pos: np.ndarray,
    target_rpy: np.ndarray,
    initial_q: np.ndarray,
    current_qpos: np.ndarray,
    current_qvel: np.ndarray,
    model_config: dict,
    max_iter: int,
    ftol: float
) -> IKResult:
    """
    Worker function: Solve IK (runs in separate PROCESS)
    """
    start_time = time.time()
    
    try:
        print(f"[AsyncIKSolver-Process-{request_id}] 🚀 Solving IK for target {target_pos}...")
        print(f"[AsyncIKSolver-Process-{request_id}] Running in SEPARATE PROCESS (no GIL contention!)")
        
        # ✅ CRITICAL: Set working directory to model directory
        # This allows the worker to find asset files (STL, textures, etc.)
        if 'model_dir' in model_config and model_config['model_dir']:
            import os
            original_cwd = os.getcwd()
            os.chdir(model_config['model_dir'])
            print(f"[AsyncIKSolver-Process-{request_id}] Set working directory: {model_config['model_dir']}")
        
        # ✅ Validate model data
        if 'model_xml' not in model_config or model_config['model_xml'] is None:
            print(f"[AsyncIKSolver-Process-{request_id}] ⚠️ No model XML in config!")
            return IKResult(
                status=IKStatus.FAILED,
                solution=None,
                error=None,
                iterations=0,
                solve_time=time.time() - start_time,
                message="No model XML available"
            )
        
        # ✅ Recreate MuJoCo model and data in worker process
        model = mujoco.MjModel.from_xml_string(model_config['model_xml'])
        data = mujoco.MjData(model)
        
        # Restore simulation state
        data.qpos[:] = current_qpos
        data.qvel[:] = current_qvel
        
        # Extract parameters
        joint_idx = model_config['joint_idx']
        ee_site_id = model_config['ee_site_id']
        bounds = model_config['bounds']
        
        print(f"[AsyncIKSolver-Process-{request_id}] Model restored: {model.nq} DoFs")
        
        # Compute target rotation matrix
        target_rot_mat = R.from_euler('xyz', target_rpy).as_matrix()
        
        iteration_count = [0]  # Mutable counter for callback
        
        def ik_cost(q):
            """Cost function for IK optimization"""
            iteration_count[0] += 1
            
            # Set joint positions
            data.qpos[joint_idx] = q
            
            # Forward kinematics
            mujoco.mj_forward(model, data)
            
            # Get end-effector pose
            ee_pos = data.site_xpos[ee_site_id].copy()
            ee_rot = data.site_xmat[ee_site_id].reshape(3, 3)
            
            # Compute errors
            pos_error = np.linalg.norm(target_pos - ee_pos)
            rot_error = 0.5 * np.linalg.norm(ee_rot - target_rot_mat, ord='fro')
            
            return pos_error + rot_error
        
        # Run optimization
        result = minimize(
            ik_cost,
            initial_q,
            bounds=bounds,
            method='SLSQP',
            options={'ftol': ftol, 'maxiter': max_iter}
        )
        
        solve_time = time.time() - start_time
        
        if result.success:
            print(f"[AsyncIKSolver-Process-{request_id}] ✅ IK solved "
                  f"({iteration_count[0]} iterations, error={result.fun:.6f}, {solve_time:.3f}s)")
            
            return IKResult(
                status=IKStatus.SUCCESS,
                solution=result.x.copy(),
                error=result.fun,
                iterations=iteration_count[0],
                solve_time=solve_time,
                message="Solution found"
            )
        else:
            print(f"[AsyncIKSolver-Process-{request_id}] ❌ IK failed: {result.message}")
            return IKResult(
                status=IKStatus.FAILED,
                solution=None,
                error=result.fun if hasattr(result, 'fun') else None,
                iterations=iteration_count[0],
                solve_time=solve_time,
                message=f"Optimization failed: {result.message}"
            )
        
    except Exception as e:
        solve_time = time.time() - start_time
        print(f"[AsyncIKSolver-Process-{request_id}] ❌ Error during IK solve: {e}")
        import traceback
        traceback.print_exc()
        
        return IKResult(
            status=IKStatus.FAILED,
            solution=None,
            error=None,
            iterations=0,
            solve_time=solve_time,
            message=str(e)
        )


# ==================== Main IK Solver Class ====================

class AsyncIKSolver:
    """
    Asynchronous IK Solver with SMART MODE SELECTION
    
    - Tries multiprocessing first (best performance)
    - Falls back to threading if model serialization fails
    
    Usage (same API regardless of mode):
        solver = AsyncIKSolver(model, data, joint_idx, bounds, ee_site_id, max_workers=2)
        future = solver.submit_request(target_pos, target_rpy)
        if solver.is_ready(future):
            result = solver.get_result(future)
    """
    
    def __init__(self, model, data, joint_idx, bounds, ee_site_id, max_workers: int = 2, model_xml_path: str = None):
        """
        Initialize async IK solver with automatic mode selection
        
        Args:
            model: MuJoCo model
            data: MuJoCo data (reference for state copying)
            joint_idx: Joint indices for IK
            bounds: Joint bounds
            ee_site_id: End-effector site ID
            max_workers: Number of persistent solver workers
            model_xml_path: Path to original XML file (needed for asset directory)
        """
        self.model = model
        self.main_data = data
        self.joint_idx = np.array(joint_idx)
        self.bounds = bounds
        self.ee_site_id = ee_site_id
        self.max_workers = max_workers
        self._shutdown = False
        self._request_counter = 0
        self.model_xml_path = model_xml_path  # ✅ Store for directory extraction
        
        # ✅ Try to extract model config for multiprocessing
        self.model_config, self.can_use_multiprocessing = self._try_extract_model_config(
            model, joint_idx, bounds, ee_site_id, model_xml_path
        )
        
        if self.can_use_multiprocessing:
            # ✅ Use multiprocessing (TRUE parallelism, no GIL!)
            print(f"[AsyncIKSolver] ✅ Using MULTIPROCESSING mode")
            print(f"[AsyncIKSolver] 🚀 TRUE parallel execution - NO GIL contention!")
            self.executor = ProcessPoolExecutor(max_workers=max_workers)
            self.use_multiprocessing = True
            self.worker_data = None  # Not needed in multiprocessing mode
        else:
            # ⚠️ Fall back to threading (some GIL contention, but works)
            print(f"[AsyncIKSolver] ⚠️ Model not serializable - using THREADING mode")
            print(f"[AsyncIKSolver] ℹ️  Will have some GIL contention")
            self.executor = ThreadPoolExecutor(
                max_workers=max_workers,
                thread_name_prefix="IKSolver"
            )
            self.use_multiprocessing = False
            # Create separate data instances for threading mode
            self.worker_data = [mujoco.MjData(model) for _ in range(max_workers)]
    
    def _try_extract_model_config(self, model, joint_idx, bounds, ee_site_id, model_xml_path=None) -> tuple[dict, bool]:
        """
        Try to extract MuJoCo model configuration for serialization
        
        CRITICAL: Must use mj_saveLastXML to compile all <include> directives!
        Otherwise worker processes can't find included files.
        
        Args:
            model: MuJoCo model
            joint_idx: Joint indices
            bounds: Joint bounds
            ee_site_id: End-effector site ID
            model_xml_path: Path to original XML file (for asset directory)
        
        Returns:
            (config_dict, can_use_multiprocessing)
        """
        config = {}
        
        try:
            # ✅ MUST use mj_saveLastXML to compile all <include> directives
            # Direct file reading doesn't work because:
            # 1. XML may have <include> tags with relative paths
            # 2. Worker process runs in different directory
            # 3. Included files won't be found
            
            import tempfile
            import os
            
            print(f"[AsyncIKSolver] Compiling model XML (resolving all includes)...")
            
            # ✅ Get directory of the XML file (where assets are relative to)
            if model_xml_path and os.path.isfile(model_xml_path):
                model_dir = os.path.dirname(os.path.abspath(model_xml_path))
                print(f"[AsyncIKSolver] Model XML path: {model_xml_path}")
                print(f"[AsyncIKSolver] Model directory (for assets): {model_dir}")
            else:
                # Fallback to current directory
                model_dir = os.getcwd()
                print(f"[AsyncIKSolver] ⚠️ No model path provided, using working directory: {model_dir}")
            
            # Create temporary file for compiled XML
            with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
                temp_filename = f.name
            
            try:
                # Save COMPILED model (all includes resolved)
                mujoco.mj_saveLastXML(temp_filename, model)
                
                # Read compiled XML
                with open(temp_filename, 'r', encoding='utf-8') as f:
                    model_xml = f.read()
                
                if model_xml is None or len(model_xml) == 0:
                    print(f"[AsyncIKSolver] ⚠️ Failed to extract model XML")
                    return config, False
                
                config['model_xml'] = model_xml
                config['joint_idx'] = joint_idx
                config['bounds'] = bounds
                config['ee_site_id'] = ee_site_id
                config['model_dir'] = model_dir  # ✅ Add working directory for assets
                
                print(f"[AsyncIKSolver] ✅ Model config extracted: {model.nq} DoFs, "
                      f"XML size={len(model_xml)} bytes (all includes compiled)")
                print(f"[AsyncIKSolver] ✅ Model directory set: {model_dir}")
                
                return config, True
                
            finally:
                # Clean up temporary file
                try:
                    os.unlink(temp_filename)
                except:
                    pass
            
        except Exception as e:
            print(f"[AsyncIKSolver] ⚠️ Error extracting model config: {e}")
            print(f"[AsyncIKSolver] Falling back to THREADING mode")
            import traceback
            traceback.print_exc()
            return config, False
    
    def submit_request(
        self,
        target_pos: np.ndarray,
        target_rpy: np.ndarray,
        max_iter: int = 100,
        ftol: float = 1e-5,
        initial_q: Optional[np.ndarray] = None
    ) -> Future:
        """
        Submit an IK solve request (non-blocking)
        
        Works with both multiprocessing and threading modes
        
        Args:
            target_pos: Target end-effector position [x, y, z]
            target_rpy: Target orientation as RPY angles [roll, pitch, yaw]
            max_iter: Maximum optimization iterations
            ftol: Optimization tolerance
            initial_q: Initial joint configuration (uses current if None)
            
        Returns:
            Future object for tracking request
        """
        if self._shutdown:
            raise RuntimeError("AsyncIKSolver has been shut down")
        
        self._request_counter += 1
        request_id = self._request_counter
        
        # Get current joint configuration as initial guess
        if initial_q is None:
            initial_q = self.main_data.qpos[self.joint_idx].copy()
        else:
            initial_q = np.array(initial_q).copy()
        
        # Copy current simulation state
        current_qpos = self.main_data.qpos.copy()
        current_qvel = self.main_data.qvel.copy()
        
        print(f"[AsyncIKSolver] Submitting IK request #{request_id}: "
              f"target={target_pos}, rpy={target_rpy}")
        
        if self.use_multiprocessing:
            # Multiprocessing mode
            future = self.executor.submit(
                _solve_ik_worker,
                request_id,
                target_pos.copy(),
                target_rpy.copy(),
                initial_q,
                current_qpos,
                current_qvel,
                self.model_config,
                max_iter,
                ftol
            )
        else:
            # Threading mode (fallback)
            future = self.executor.submit(
                self._solve_ik_thread,
                request_id,
                target_pos.copy(),
                target_rpy.copy(),
                initial_q,
                max_iter,
                ftol
            )
        
        return future
    
    def _solve_ik_thread(
        self,
        request_id: int,
        target_pos: np.ndarray,
        target_rpy: np.ndarray,
        initial_q: np.ndarray,
        max_iter: int,
        ftol: float
    ) -> IKResult:
        """
        Worker function for threading mode (fallback)
        Can access self.model and self.main_data directly
        """
        start_time = time.time()
        
        try:
            print(f"[AsyncIKSolver-Thread-{request_id}] Computing IK for target {target_pos}...")
            
            # Get a worker data instance (simplified - use worker_data[0])
            worker_data = self.worker_data[0]
            
            # Copy main simulation state
            worker_data.qpos[:] = self.main_data.qpos[:]
            worker_data.qvel[:] = self.main_data.qvel[:]
            
            # Compute target rotation matrix
            target_rot_mat = R.from_euler('xyz', target_rpy).as_matrix()
            
            iteration_count = [0]
            
            def ik_cost(q):
                """Cost function for IK optimization"""
                iteration_count[0] += 1
                
                # Set joint positions
                worker_data.qpos[self.joint_idx] = q
                
                # Forward kinematics
                mujoco.mj_forward(self.model, worker_data)
                
                # Get end-effector pose
                ee_pos = worker_data.site_xpos[self.ee_site_id].copy()
                ee_rot = worker_data.site_xmat[self.ee_site_id].reshape(3, 3)
                
                # Compute errors
                pos_error = np.linalg.norm(target_pos - ee_pos)
                rot_error = 0.5 * np.linalg.norm(ee_rot - target_rot_mat, ord='fro')
                
                return pos_error + rot_error
            
            # Run optimization
            result = minimize(
                ik_cost,
                initial_q,
                bounds=self.bounds,
                method='SLSQP',
                options={'ftol': ftol, 'maxiter': max_iter}
            )
            
            solve_time = time.time() - start_time
            
            if result.success:
                print(f"[AsyncIKSolver-Thread-{request_id}] ✅ IK solved successfully "
                      f"({iteration_count[0]} iterations, {solve_time:.3f}s)")
                
                return IKResult(
                    status=IKStatus.SUCCESS,
                    solution=result.x.copy(),
                    error=result.fun,
                    iterations=iteration_count[0],
                    solve_time=solve_time,
                    message="Solution found"
                )
            else:
                print(f"[AsyncIKSolver-Thread-{request_id}] ❌ IK failed")
                return IKResult(
                    status=IKStatus.FAILED,
                    solution=None,
                    error=result.fun if hasattr(result, 'fun') else None,
                    iterations=iteration_count[0],
                    solve_time=solve_time,
                    message=f"Optimization failed: {result.message}"
                )
            
        except Exception as e:
            solve_time = time.time() - start_time
            print(f"[AsyncIKSolver-Thread-{request_id}] ❌ Error during IK solve: {e}")
            import traceback
            traceback.print_exc()
            
            return IKResult(
                status=IKStatus.FAILED,
                solution=None,
                error=None,
                iterations=0,
                solve_time=solve_time,
                message=f"Error during solve: {e}"
            )
    
    def is_ready(self, future: Future) -> bool:
        """Check if IK solve is complete (non-blocking)"""
        return future.done()
    
    def get_result(self, future: Future, timeout: Optional[float] = None) -> IKResult:
        """Get IK solve result (blocking if not ready)"""
        try:
            result = future.result(timeout=timeout)
            return result
        except TimeoutError:
            return IKResult(
                status=IKStatus.TIMEOUT,
                solution=None,
                error=None,
                iterations=0,
                solve_time=timeout if timeout else 0.0,
                message="IK solve timeout"
            )
        except Exception as e:
            print(f"[AsyncIKSolver] ❌ Error getting result: {e}")
            import traceback
            traceback.print_exc()
            return IKResult(
                status=IKStatus.FAILED,
                solution=None,
                error=None,
                iterations=0,
                solve_time=0.0,
                message=f"Error getting result: {e}"
            )
    
    def wait_for_result(self, future: Future, timeout: float = 5.0) -> IKResult:
        """
        Wait for result with timeout (blocking)
        
        Args:
            future: Future object from submit_request
            timeout: Maximum wait time in seconds
            
        Returns:
            IKResult when available, or timeout result
        """
        return self.get_result(future, timeout=timeout)
    
    def shutdown(self, timeout: float = 5.0):
        """Gracefully shutdown workers"""
        if self._shutdown:
            return
        
        mode = "processes" if self.use_multiprocessing else "threads"
        print(f"[AsyncIKSolver] Shutting down worker {mode}...")
        self._shutdown = True
        
        if self.executor:
            self.executor.shutdown(wait=True, cancel_futures=True)
            print(f"[AsyncIKSolver] ✅ Worker {mode} terminated")
    
    def __del__(self):
        """Cleanup on deletion"""
        try:
            self.shutdown(timeout=1.0)
        except:
            pass


# ==================== Backward Compatibility ====================
# Keep old class names for compatibility with existing code

IKRequest = None  # Deprecated - no longer used with Future-based API
