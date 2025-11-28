"""Pick and Place 실행 가능성 체크 - 간소화 및 실용적 개선"""

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

class FeasibilityChecker:
    """Pick and Place 작업의 실행 가능성을 체크하는 클래스"""
    
    # 별칭 → 실제 바디명 매핑 (class-level constant)
    _NAME_MAP = {
        "빨간색 박스": "red_box", "빨간 박스": "red_box", "red box": "red_box",
        "파란색 박스": "blue_box", "파란 박스": "blue_box", "blue box": "blue_box",
        "초록색 박스": "green_box", "초록 박스": "green_box", "green box": "green_box",
        "노란색 박스": "yellow_box", "노란 박스": "yellow_box", "yellow box": "yellow_box",
        "주황색 박스": "orange_box", "주황 박스": "orange_box", "orange box": "orange_box",
        "보라색 박스": "purple_box", "보라 박스": "purple_box", "purple box": "purple_box",
        "청록색 박스": "cyan_box", "청록 박스": "cyan_box", "cyan box": "cyan_box",
        "분홍색 박스": "pink_box", "분홍 박스": "pink_box", "pink box": "pink_box",
    }
    
    def __init__(self, model, data, ik_solver, config, robot_id=1):
        self.model = model
        self.data = data
        self.ik_solver = ik_solver
        self.config = config
        self.robot_id = robot_id
        self.robot_base_link = f"robot{robot_id}_base_link"
        
        # 로봇 팔의 도달 범위 - 실용적으로 완화
        self.MIN_REACH = 0.05  # 0.1 → 0.05 (충돌 가능성 낮음)
        self.MAX_REACH = 0.9   # 0.8 → 0.9 (그리퍼 길이 고려)
        self.MAX_HEIGHT = 0.7  # 0.6 → 0.7
        self.MIN_HEIGHT = -0.15  # -0.1 → -0.15
        
        self.SAFETY_MARGIN = 0.05
    
    def _get_box_info(self, name: str):
        """MuJoCo에서 박스의 (중심 위치, 높이) 반환
        
        Args:
            name: Box name (supports aliases)
            
        Returns:
            tuple: (position, height) or (None, 0.1) if not found
        """
        mapped = self._NAME_MAP.get(name.lower(), name)
        
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, mapped)
        if body_id < 0:
            return None, 0.1  # Default height
        
        pos = self.data.xpos[body_id].copy()
        
        # Calculate height: first geom's size[2]*2 (for box). Default 0.1 if not found
        height = 0.1
        for gid in range(self.model.ngeom):
            if self.model.geom_bodyid[gid] != body_id:
                continue
            size = self.model.geom_size[gid]
            if size.shape[0] >= 3:
                height = float(size[2] * 2.0)
                break
        
        return pos, height
    
    def _get_gripper_position(self):
        """Get current gripper position
        
        Returns:
            np.ndarray: Gripper position [x, y, z]
        """
        # Use robot-specific end-effector site
        if self.robot_id == 2:
            ee_site_name = "robot2/end_effector"
        else:
            ee_site_name = "end_effector"
        
        ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)
        
        if ee_site_id >= 0:
            return self.data.site_xpos[ee_site_id].copy()
        else:
            # Fallback to robot base position
            base_body_id = self.model.body(self.robot_base_link).id
            return self.data.xpos[base_body_id].copy()
    
    def resolve_pick_position(self, object_name: str):
        """MuJoCo에서 바로 박스 실좌표를 가져와 픽업 위치 반환
        
        Args:
            object_name: Object name to pick
            
        Returns:
            tuple: (pick_position, box_name)
            
        Raises:
            RuntimeError: If object not found in simulation
        """
        import logging
        logger = logging.getLogger(__name__)
        
        # Use shared name mapping
        box_name = self._NAME_MAP.get(object_name.lower(), object_name)
        
        # MuJoCo에서 body id 조회 → 없으면 에러
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, box_name)
        if body_id < 0:
            raise RuntimeError(f"박스를 찾을 수 없음: {object_name}")
        
        # 실좌표 반환 (world frame)
        pick_pos = self.data.xpos[body_id].copy()
        
        # Debug output
        robot_id_str = f"robot{self.robot_id}"
        if self.robot_id == 2:
            # Get Robot2's current positions
            base_body_id = self.model.body(self.robot_base_link).id
            robot_base_pos = self.data.xpos[base_body_id]
            
            # Get Robot2's end-effector position
            robot_ee_pos = self._get_gripper_position()
            
            logger.info(f"[PICK_DEBUG] [{robot_id_str}] Box: {box_name}")
            logger.info(f"[PICK_DEBUG] [{robot_id_str}] Box world position: {pick_pos}")
            logger.info(f"[PICK_DEBUG] [{robot_id_str}] Robot2 base position: {robot_base_pos[:2]}")
            logger.info(f"[PICK_DEBUG] [{robot_id_str}] Robot2 EE position: {robot_ee_pos}")
            logger.info(f"[PICK_DEBUG] [{robot_id_str}] Distance base->box: {np.linalg.norm(pick_pos[:2] - robot_base_pos[:2]):.3f}m")
            logger.info(f"[PICK_DEBUG] [{robot_id_str}] Distance EE->box: {np.linalg.norm(pick_pos - robot_ee_pos):.3f}m")
        
        return pick_pos, box_name
    
    def resolve_place_position(self, target, held_object: str):
        """목표 위치와 보유 물체 정보로 place 포즈와 그리퍼까지 XY거리 계산
        
        Args:
            target: Target position - can be:
                    - str: object name to stack on top of
                    - [x, y]: coordinates (z calculated from held object height)
                    - [x, y, z]: full coordinates
            held_object: Name of the object currently being held
            
        Returns:
            tuple: (place_position, distance_from_gripper)
                   Returns (None, 999.0) if target object not found
        """
        import logging
        logger = logging.getLogger(__name__)
        
        # Get held object height
        _, held_h = self._get_box_info(held_object)
        
        def _xy_dist(p, r):
            """Calculate XY distance between two 3D points"""
            return float(np.linalg.norm(np.array(p[:2]) - np.array(r[:2])))
        
        # Case 1: String target - stack on top of another object
        if isinstance(target, str):
            tgt_pos, tgt_h = self._get_box_info(target)
            if tgt_pos is None:
                logger.error(f"[PLACE] Target object not found: {target}")
                return None, 999.0
            
            place_pos = tgt_pos.copy()
            place_pos[2] = tgt_pos[2] + tgt_h  # Place on top of target object
            
            gripper_pos = self._get_gripper_position()
            dist = _xy_dist(place_pos, gripper_pos)
            
            logger.debug(f"[PLACE] Stacking on {target}: pos={place_pos}, dist={dist:.3f}m")
            return place_pos, dist
        
        # Case 2: Coordinate target
        tp = np.array(target, dtype=float).flatten()
        if tp.shape[0] == 2:
            # [x, y] - calculate z from held object height
            place_pos = np.array([tp[0], tp[1], held_h / 2.0])
        else:
            # [x, y, z] - use provided z or default to held object height
            z = float(tp[2]) if tp.shape[0] >= 3 else (held_h / 2.0)
            place_pos = np.array([tp[0], tp[1], z])
        
        gripper_pos = self._get_gripper_position()
        dist = _xy_dist(place_pos, gripper_pos)
        
        logger.debug(f"[PLACE] Coordinates: pos={place_pos}, dist={dist:.3f}m")
        return place_pos, dist
        
    def check_pick_and_place_feasibility(self, pick_pos, place_pos):
        """Pick and Place 작업의 전체 실행 가능성 체크
        
        Args:
            pick_pos: 픽업할 물체의 위치
            place_pos: 놓을 위치
            
        Returns:
            tuple: (가능 여부, 메시지)
        """
        # 1. Pick 위치 체크
        pick_feasible, pick_msg = self.check_position_reachability(pick_pos, "Pick")
        if not pick_feasible:
            return False, pick_msg
            
        # 2. Place 위치 체크
        place_feasible, place_msg = self.check_position_reachability(place_pos, "Place")
        if not place_feasible:
            return False, place_msg
        
        return True, "Pick & Place 작업 실행 가능"
    
    def check_position_reachability(self, target_pos, operation_name="작업"):
        """단일 위치의 도달 가능성 체크 - 실용적으로 완화
        
        Args:
            target_pos: 목표 위치
            operation_name: 작업 이름 (디버깅용)
            
        Returns:
            tuple: (도달 가능 여부, 메시지)
        """
        # 현재 베이스 위치
        body_id = self.model.body(self.robot_base_link).id
        robot_base_pos = self.data.xpos[body_id]
        base_pos = robot_base_pos[:2]  # x, y
        
        # 베이스로부터 목표까지의 상대 위치
        rel_x = target_pos[0] - base_pos[0]
        rel_y = target_pos[1] - base_pos[1]
        rel_z = target_pos[2]
        
        # 수평 거리
        horizontal_distance = np.sqrt(rel_x**2 + rel_y**2)
        
        # 거리 체크 - 완화된 기준
        if horizontal_distance < self.MIN_REACH:
            return False, f"{operation_name} 위치 너무 가까움 ({horizontal_distance:.3f}m < {self.MIN_REACH}m)"
        
        if horizontal_distance > self.MAX_REACH:
            return False, f"{operation_name} 위치 너무 멀음 ({horizontal_distance:.3f}m > {self.MAX_REACH}m)"
        
        # 높이 체크 - 완화
        if rel_z < self.MIN_HEIGHT:
            return False, f"{operation_name} 높이 너무 낮음 ({rel_z:.3f}m < {self.MIN_HEIGHT}m)"
        
        if rel_z > self.MAX_HEIGHT:
            return False, f"{operation_name} 높이 너무 높음 ({rel_z:.3f}m > {self.MAX_HEIGHT}m)"
        
        return True, f"{operation_name} 도달 가능 (거리: {horizontal_distance:.3f}m, 높이: {rel_z:.3f}m)"
    
    def check_surrounding_boxes_safety(self, target_box_name, target_pos, min_safe_distance=0.3):
        """Pick 작업 시 주변 박스 안전거리 체크 - 로봇 위치 고려
        
        Args:
            target_box_name: 픽업할 박스 이름
            target_pos: 픽업할 박스 위치
            min_safe_distance: 최소 안전거리 (기본 0.3m)
            
        Returns:
            tuple: (안전 여부, 메시지, 정보 딕셔너리)
        """
        box_names = [
            "red_box", "blue_box", "green_box", "yellow_box",
            "orange_box", "purple_box", "cyan_box", "pink_box"
        ]
        
        # 현재 로봇 위치
        body_id = self.model.body(self.robot_base_link).id
        robot_base_pos = self.data.xpos[body_id]
        robot_pos = robot_base_pos[:2]  # x, y만 사용
        
        nearby_boxes = []
        closest_distance = float('inf')
        closest_box = None
        
        for box_name in box_names:
            if box_name == target_box_name:
                continue
            
            try:
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, box_name)
                box_pos = self.data.xpos[body_id].copy()
                
                # 로봇 위치에서 다른 박스까지의 거리 (2D)
                robot_to_box = np.sqrt(
                    (box_pos[0] - robot_pos[0])**2 + 
                    (box_pos[1] - robot_pos[1])**2
                )
                
                # 목표 박스와 다른 박스 사이 거리
                box_to_box = np.sqrt(
                    (box_pos[0] - target_pos[0])**2 + 
                    (box_pos[1] - target_pos[1])**2
                )
                
                # 박스 크기 고려
                box_radius = 0.075
                effective_distance = robot_to_box - box_radius
                
                box_info = {
                    'name': box_name,
                    'position': box_pos,
                    'robot_to_box': robot_to_box,
                    'box_to_box': box_to_box,
                    'effective_distance': effective_distance
                }
                
                # 로봇과 가까운 박스만 체크
                if robot_to_box < 1.0:  # 1m 이내만
                    if effective_distance < min_safe_distance:
                        nearby_boxes.append(box_info)
                
                if effective_distance < closest_distance:
                    closest_distance = effective_distance
                    closest_box = box_info
                    
            except:
                continue
        
        # 결과 판단 - 완화된 기준
        if not nearby_boxes:
            return True, f"주변 안전 (가장 가까운 박스: {closest_distance:.3f}m)", {
                'nearby_boxes': nearby_boxes,
                'closest_box': closest_box,
                'safe': True
            }
        
        # 경고만 발생 (실패 아님)
        warning_msg = f" 주변에 {len(nearby_boxes)}개 박스가 가까움:\n"
        for box in nearby_boxes:
            warning_msg += f"  - {box['name']}: 로봇으로부터 {box['robot_to_box']:.3f}m\n"
        warning_msg += "주의하여 진행하세요."
        
        # 매우 가까울 때만 실패 (0.05m 미만)
        if closest_distance < 0.05:
            return False, f" 박스가 너무 가까움 ({closest_distance:.3f}m < 0.05m)", {
                'nearby_boxes': nearby_boxes,
                'closest_box': closest_box,
                'safe': False
            }
        
        # 경고지만 진행 가능
        return True, warning_msg, {
            'nearby_boxes': nearby_boxes,
            'closest_box': closest_box,
            'safe': True,
            'warning': True
        }
    