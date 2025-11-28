"""
RoboCasa Integration - FINAL Working Version
Creates full kitchen with fixtures and merges TidyBot robots
"""

import mujoco
import numpy as np
from typing import Tuple
import logging
import os
from xml.etree import ElementTree as ET
import copy

logger = logging.getLogger(__name__)

# Layout and style mappings
LAYOUT_MAPPING = {
    "one-wall-small": 0, "one-wall-large": 1,
    "L-shaped-small": 2, "L-shaped-large": 3,
    "galley": 4,
    "U-shaped-small": 5, "U-shaped-large": 6,
    "G-shaped-small": 7, "G-shaped-large": 8,
    "G-shaped": 7, "U-shaped": 5, "L-shaped": 2,
    "wraparound": 9,
}

STYLE_MAPPING = {
    "industrial": 0, "scandinavian": 1, "coastal": 2,
    "modern": 3, "traditional": 5, "farmhouse": 7,
    "rustic": 8, "mediterranean": 9,
}


def create_robocasa_kitchen(layout: str = "G-shaped",
                            style: str = "modern",
                            robot1_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                            robot2_pos: Tuple[float, float, float] = (2.0, 0.0, 0.0),
                            physics_mode: str = "fast") -> mujoco.MjModel:
    """
    Create RoboCasa kitchen and merge with TidyBot robots
    
    Args:
        layout: Kitchen layout (e.g., "G-shaped", "L-shaped", "U-shaped")
        style: Kitchen style (e.g., "modern", "industrial", "farmhouse")
        robot1_pos: Robot1 starting position (x, y, z)
        robot2_pos: Robot2 starting position (x, y, z)
        physics_mode: Physics optimization mode
            - "fast": Low iterations, PGS solver (fast, less accurate)
            - "balanced": Medium iterations, Newton solver (good tradeoff)
            - "accurate": High iterations, Newton solver (slow, accurate)
    """
    
    layout_id = LAYOUT_MAPPING.get(layout.lower(), 7)
    style_id = STYLE_MAPPING.get(style.lower(), 3)
    
    logger.info(f"Creating RoboCasa kitchen: layout={layout} (id={layout_id}), style={style} (id={style_id})")
    
    kitchen_xml_str = None
    approach_used = None
    
    # Approach 1: Direct Kitchen class with render_camera=None
    try:
        logger.info("Approach 1: Direct Kitchen class with proper config...")
        from robocasa.environments.kitchen.kitchen import Kitchen
        
        env = Kitchen(
            robots="Panda",
            layout_ids=layout_id,
            style_ids=style_id,
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            render_camera=None,  # KEY FIX: Avoid camera issues
            control_freq=20,
        )
        
        logger.info(f"✓ Kitchen created with {env.sim.model.nbody} bodies")
        kitchen_xml_str = env.sim.model.get_xml()
        env.close()
        approach_used = "Kitchen class"
        logger.info("✓ Approach 1 successful!")
        
    except Exception as e1:
        logger.warning(f"Approach 1 failed: {e1}")
        
        # Approach 2: Try with make() function
        try:
            logger.info("Approach 2: Using robosuite.environments.base.make()...")
            from robosuite.environments.base import make
            
            env = make(
                env_name="Kitchen",
                robots="Panda",
                layout_ids=layout_id,
                style_ids=style_id,
                has_renderer=False,
                has_offscreen_renderer=False,
                use_camera_obs=False,
                render_camera=None,
                control_freq=20,
                horizon=1000,
            )
            
            kitchen_xml_str = env.sim.model.get_xml()
            env.close()
            approach_used = "robosuite.make"
            logger.info("✓ Approach 2 successful!")
            
        except Exception as e2:
            logger.warning(f"Approach 2 failed: {e2}")
            
            # Approach 3: Use basic KitchenArena
            try:
                logger.info("Approach 3: Using basic KitchenArena...")
                from robocasa.models.scenes.kitchen_arena import KitchenArena
                
                arena = KitchenArena(
                    layout_id=layout_id,
                    style_id=style_id
                )
                
                kitchen_xml_str = arena.get_xml()
                approach_used = "KitchenArena (basic)"
                logger.warning("⚠ Using basic kitchen (no fixtures)")
                
            except Exception as e3:
                logger.error(f"All approaches failed!")
                logger.error(f"Approach 1: {e1}")
                logger.error(f"Approach 2: {e2}")
                logger.error(f"Approach 3: {e3}")
                raise RuntimeError("Failed to create kitchen with all approaches")
    
    if kitchen_xml_str is None:
        raise RuntimeError("Failed to create kitchen")
    
    logger.info(f"Kitchen created using: {approach_used}")
    
    # Parse XML
    kitchen_root = ET.fromstring(kitchen_xml_str)
    
    # Remove RoboCasa robot
    logger.info("Removing RoboCasa robots from kitchen...")
    _remove_robocasa_robots(kitchen_root)
    
    # Load TidyBot XMLs
    logger.info("Loading TidyBot robot XMLs...")
    base_dir = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.join(base_dir, "model", "stanford_tidybot")
    assets_dir = os.path.join(model_dir, "assets")
    
    tidybot1_xml = os.path.join(model_dir, "tidybot.xml")
    tidybot2_xml = os.path.join(model_dir, "tidybot_robot2.xml")
    
    if not os.path.exists(tidybot1_xml):
        raise FileNotFoundError(f"Robot1 XML not found: {tidybot1_xml}")
    if not os.path.exists(tidybot2_xml):
        raise FileNotFoundError(f"Robot2 XML not found: {tidybot2_xml}")
    
    robot1_root = ET.parse(tidybot1_xml).getroot()
    robot2_root = ET.parse(tidybot2_xml).getroot()
    
    # Fix mesh paths
    logger.info("Fixing mesh paths...")
    _fix_mesh_paths(robot1_root, assets_dir)
    _fix_mesh_paths(robot2_root, assets_dir)
    
    # Merge robots into kitchen
    logger.info("Merging TidyBot robots into kitchen...")
    _merge_robots_into_kitchen(kitchen_root, robot1_root, robot2_root, robot1_pos, robot2_pos)
    
    # ✅ Apply physics settings based on mode
    logger.info(f"✅ Applying physics settings (mode: {physics_mode})...")
    if physics_mode == "fast":
        _fix_physics_settings_fast(kitchen_root)
    elif physics_mode == "balanced":
        _fix_physics_settings_balanced(kitchen_root)
    elif physics_mode == "accurate":
        _fix_physics_settings_accurate(kitchen_root)
    else:
        logger.warning(f"Unknown physics_mode '{physics_mode}', using 'fast'")
        _fix_physics_settings_fast(kitchen_root)
    
    _increase_base_damping(kitchen_root)
    _fix_floor_friction(kitchen_root)
    
    # ⭐ Apply performance optimizations (optional - can be disabled for more stability)
    # logger.info("⭐ Applying performance optimizations...")
    # _optimize_floor_wall_solref(kitchen_root)  # ⭐1: 바닥/벽 solref 최적화
    # _unify_object_friction(kitchen_root)       # ⭐2: 모든 오브젝트 friction 통일
    # _optimize_contact_computation(kitchen_root) # ⭐3: contact 계산 단순화
    
    # Convert to string
    merged_xml = ET.tostring(kitchen_root, encoding='unicode')
    
    # Save debug XML
    try:
        from xml.dom import minidom
        dom = minidom.parseString(merged_xml)
        pretty_xml = dom.toprettyxml(indent="  ")
        pretty_xml = '\n'.join([line for line in pretty_xml.split('\n') if line.strip()])
        
        debug_path = os.path.join(model_dir, "robocasa_merged_debug.xml")
        with open(debug_path, 'w', encoding='utf-8') as f:
            f.write(pretty_xml)
        logger.info(f"✓ Debug XML saved: {debug_path}")
    except Exception as e:
        logger.warning(f"Could not save pretty debug XML: {e}")
    
    # Create MuJoCo model
    logger.info("Creating MuJoCo model from merged XML...")
    try:
        model = mujoco.MjModel.from_xml_string(merged_xml)
        
        logger.info(f"✓ RoboCasa model successfully created!")
        logger.info(f"   Bodies:    {model.nbody:4d}")
        logger.info(f"   Joints:    {model.njnt:4d}")
        logger.info(f"   Geoms:     {model.ngeom:4d}")
        logger.info(f"   Actuators: {model.nu:4d}")
        
        if model.nbody > 100:
            logger.info(f"🎉 FULL KITCHEN with fixtures!")
        else:
            logger.warning(f"⚠ Basic kitchen (only {model.nbody} bodies)")
        
        return model
        
    except Exception as e:
        logger.error(f"Failed to create MuJoCo model: {e}")
        
        error_path = os.path.join(model_dir, "robocasa_error.xml")
        with open(error_path, 'w', encoding='utf-8') as f:
            f.write(merged_xml)
        logger.error(f"Error XML saved to: {error_path}")
        
        raise RuntimeError(f"Failed to create MuJoCo model from merged XML: {e}")


def _remove_robocasa_robots(kitchen_root):
    """Remove RoboCasa's robot components including sensors"""
    
    worldbody = kitchen_root.find('worldbody')
    if worldbody is None:
        return
    
    robot_keywords = [
        'robot', 'panda', 'gripper', 'eef', 'mount', 
        'base_link', 'arm', 'link', 'manipulator', 'hand',
        'finger', 'ft_frame', 'force', 'torque'
    ]
    
    # Track removed body/site names for sensor cleanup
    removed_names = set()
    
    # Remove robot bodies
    bodies_to_remove = []
    for body in worldbody.findall('body'):
        name = body.get('name', '')
        if any(keyword in name.lower() for keyword in robot_keywords):
            bodies_to_remove.append(body)
            removed_names.add(name)
            # Track all child bodies and sites
            for child in body.findall('.//*'):
                if 'name' in child.attrib:
                    removed_names.add(child.attrib['name'])
    
    for body in bodies_to_remove:
        worldbody.remove(body)
        logger.debug(f"  Removed body: {body.get('name')}")
    
    # Remove robot actuators
    actuator_section = kitchen_root.find('actuator')
    actuators_to_remove = []
    if actuator_section is not None:
        for actuator in actuator_section:
            name = actuator.get('name', '').lower()
            joint = actuator.get('joint', '').lower()
            if any(keyword in name or keyword in joint for keyword in robot_keywords):
                actuators_to_remove.append(actuator)
        
        for actuator in actuators_to_remove:
            actuator_section.remove(actuator)
            logger.debug(f"  Removed actuator: {actuator.get('name')}")
    
    # Remove robot sensors (CRITICAL!)
    sensor_section = kitchen_root.find('sensor')
    sensors_to_remove = []
    if sensor_section is not None:
        for sensor in sensor_section:
            name = sensor.get('name', '')
            site = sensor.get('site', '')
            body = sensor.get('body', '')
            
            # Check if sensor name contains robot keywords
            if any(keyword in name.lower() for keyword in robot_keywords):
                sensors_to_remove.append(sensor)
            # Check if sensor references removed bodies/sites
            elif site in removed_names or body in removed_names:
                sensors_to_remove.append(sensor)
        
        for sensor in sensors_to_remove:
            sensor_section.remove(sensor)
            logger.debug(f"  Removed sensor: {sensor.get('name')}")
    
    logger.info(f"  Removed {len(bodies_to_remove)} bodies, {len(actuators_to_remove)} actuators, {len(sensors_to_remove)} sensors")


def _fix_mesh_paths(root, assets_dir):
    """Convert relative mesh paths to absolute paths"""
    
    compiler = root.find('compiler')
    if compiler is not None:
        compiler.set('meshdir', assets_dir)
    
    asset = root.find('asset')
    if asset is not None:
        for mesh in asset.findall('.//mesh'):
            if 'file' in mesh.attrib:
                file_path = mesh.attrib['file']
                if not os.path.isabs(file_path):
                    abs_path = os.path.join(assets_dir, file_path)
                    mesh.set('file', abs_path)


def _merge_robots_into_kitchen(kitchen_root, robot1_root, robot2_root, robot1_pos, robot2_pos):
    """Merge TidyBot robot XMLs into kitchen XML"""
    
    _merge_defaults(kitchen_root, robot1_root)
    _merge_defaults(kitchen_root, robot2_root)
    
    kitchen_worldbody = kitchen_root.find('worldbody')
    if kitchen_worldbody is None:
        kitchen_worldbody = ET.SubElement(kitchen_root, 'worldbody')
    
    robot1_worldbody = robot1_root.find('worldbody')
    if robot1_worldbody is not None:
        for elem in robot1_worldbody:
            cloned = copy.deepcopy(elem)
            if elem.tag == 'body' and 'pos' in elem.attrib:
                _adjust_position(cloned, robot1_pos)
            kitchen_worldbody.append(cloned)
    
    robot2_worldbody = robot2_root.find('worldbody')
    if robot2_worldbody is not None:
        for elem in robot2_worldbody:
            cloned = copy.deepcopy(elem)
            if elem.tag == 'body' and 'pos' in elem.attrib:
                _adjust_position(cloned, robot2_pos)
            kitchen_worldbody.append(cloned)
    
    sections = ['asset', 'actuator', 'sensor', 'tendon', 'equality', 'contact']
    for section in sections:
        _merge_section(kitchen_root, robot1_root, section)
        _merge_section(kitchen_root, robot2_root, section)


def _merge_defaults(kitchen_root, robot_root):
    """Merge default classes"""
    
    kitchen_default = kitchen_root.find('default')
    robot_default = robot_root.find('default')
    
    if robot_default is None:
        return
    
    if kitchen_default is None:
        kitchen_root.insert(0, copy.deepcopy(robot_default))
        return
    
    for elem in robot_default:
        if elem.tag == 'default' and 'class' in elem.attrib:
            class_name = elem.attrib['class']
            
            existing = None
            for existing_elem in kitchen_default:
                if existing_elem.tag == 'default' and existing_elem.get('class') == class_name:
                    existing = existing_elem
                    break
            
            if existing is None:
                kitchen_default.append(copy.deepcopy(elem))
        else:
            kitchen_default.append(copy.deepcopy(elem))


def _adjust_position(element, position: Tuple[float, float, float]):
    """Adjust element position"""
    
    if 'pos' in element.attrib:
        current = [float(x) for x in element.attrib['pos'].split()]
        new_pos = [
            current[0] + position[0],
            current[1] + position[1],
            current[2] + position[2]
        ]
        element.attrib['pos'] = f"{new_pos[0]} {new_pos[1]} {new_pos[2]}"


def _merge_section(kitchen_root, robot_root, section_name: str):
    """Merge XML section"""
    
    kitchen_section = kitchen_root.find(section_name)
    robot_section = robot_root.find(section_name)
    
    if robot_section is None:
        return
    
    if kitchen_section is None:
        kitchen_section = ET.SubElement(kitchen_root, section_name)
    
    for elem in robot_section:
        kitchen_section.append(copy.deepcopy(elem))


def _fix_physics_settings_fast(kitchen_root):
    """⚡ FAST physics settings - for development/testing
    
    Uses PGS solver with low iterations for maximum speed.
    May be less accurate for complex manipulation tasks.
    """
    option = kitchen_root.find('option')
    if option is None:
        option = ET.Element('option')
        kitchen_root.insert(0, option)
    
    option.set('timestep', '0.002')  # 2ms (500 Hz)
    option.set('solver', 'PGS')  # ⚡ Fast solver (was Newton)
    option.set('iterations', '20')  # ⚡ Much lower (was 100)
    option.set('noslip_iterations', '10')  # ⚡ Much lower (was 100)
    option.set('tolerance', '1e-6')  # ⚡ Relaxed (was 1e-10)
    option.set('ls_tolerance', '0.1')
    option.set('cone', 'elliptic')
    option.set('impratio', '10')
    
    logger.info("  ⚡ FAST physics settings applied (PGS solver, 20 iterations)")


def _fix_physics_settings_balanced(kitchen_root):
    """⚖️ BALANCED physics settings - good tradeoff
    
    Uses Newton solver with moderate iterations.
    Good balance between speed and accuracy.
    """
    option = kitchen_root.find('option')
    if option is None:
        option = ET.Element('option')
        kitchen_root.insert(0, option)
    
    option.set('timestep', '0.002')  # 2ms (500 Hz)
    option.set('solver', 'Newton')  # Newton for stability
    option.set('iterations', '50')  # ⚖️ Moderate (was 100)
    option.set('noslip_iterations', '20')  # ⚖️ Moderate (was 100)
    option.set('tolerance', '1e-7')  # ⚖️ Slightly relaxed
    option.set('ls_tolerance', '0.05')
    option.set('cone', 'elliptic')
    option.set('impratio', '10')
    
    logger.info("  ⚖️ BALANCED physics settings applied (Newton solver, 50 iterations)")


def _fix_physics_settings_accurate(kitchen_root):
    """🎯 ACCURATE physics settings - for precise manipulation
    
    Uses Newton solver with high iterations.
    Slower but more stable for delicate tasks.
    """
    option = kitchen_root.find('option')
    if option is None:
        option = ET.Element('option')
        kitchen_root.insert(0, option)
    
    option.set('timestep', '0.002')  # 2ms (500 Hz)
    option.set('solver', 'Newton')  # Newton solver
    option.set('iterations', '100')  # High iteration count
    option.set('noslip_iterations', '50')  # High but not excessive
    option.set('tolerance', '1e-8')  # Tight convergence
    option.set('ls_tolerance', '0.01')
    option.set('cone', 'elliptic')
    option.set('impratio', '10')
    
    logger.info("  🎯 ACCURATE physics settings applied (Newton solver, 100 iterations)")


def _fix_physics_settings(kitchen_root):
    """✅ [DEPRECATED] Use _fix_physics_settings_fast/balanced/accurate instead"""
    _fix_physics_settings_balanced(kitchen_root)


def _optimize_floor_wall_solref(kitchen_root):
    """⭐1: 바닥 및 모든 벽 geom의 solref/solimp 최적화
    
    딱딱한 충돌 반응으로 더 빠른 동작 가능
    scene_dual_robot 수준:
    - solref="0.001 1" (기존 "0.02 1"에서 20배 더 딱딱함)
    - solimp="0.9 0.95 0.002" (높은 강성)
    """
    
    worldbody = kitchen_root.find('worldbody')
    if worldbody is None:
        return
    
    floor_wall_count = 0
    
    # Find all floor and wall geoms
    for geom in worldbody.findall('.//geom'):
        name = geom.get('name', '').lower()
        
        # Check if this is a floor or wall geom
        if any(keyword in name for keyword in ['floor', 'ground', 'wall', 'ceiling', 'boundary']):
            # Set stiff contact parameters
            geom.set('solref', '0.001 1')        # 
            geom.set('solimp', '0.9 0.95 0.002')  # High stiffness, high damping
            floor_wall_count += 1
            logger.debug(f"    Floor/Wall '{name}': solref=0.001 1, solimp=0.9 0.95 0.002")
    
    if floor_wall_count > 0:
        logger.info(f"  ⭐ Floor/Wall solref optimized ({floor_wall_count} geoms)")
    else:
        logger.warning("  ⚠ No floor/wall geoms found for solref optimization")


def _unify_object_friction(kitchen_root):
    """⭐2: 모든 기타 오브젝트의 friction 통일
    
    Robocasa의 극단적인 friction="2 2 2" 값들을 제거하고
    합리적인 friction="1 0.005 0.0001"로 통일
    """
    
    worldbody = kitchen_root.find('worldbody')
    if worldbody is None:
        return
    
    object_count = 0
    extreme_fixed = 0
    
    # Find all geoms (excluding robot parts)
    for geom in worldbody.findall('.//geom'):
        name = geom.get('name', '').lower()
        
        # Skip robot geoms
        if any(keyword in name for keyword in ['robot', 'panda', 'gripper', 'link', 'wheel']):
            continue
        
        # Get current friction
        current_friction = geom.get('friction', '')
        
        # Check for extreme values (>= 2.0)
        if current_friction:
            try:
                friction_vals = [float(x) for x in current_friction.split()]
                if any(val >= 2.0 for val in friction_vals):
                    extreme_fixed += 1
                    logger.debug(f"    Fixed extreme friction '{name}': {current_friction} -> 1 0.005 0.0001")
            except ValueError:
                pass
        
        # Unify friction for all objects
        geom.set('friction', '1 0.005 0.0001')  # [sliding, torsional, rolling]
        object_count += 1
    
    logger.info(f"  ⭐ Object friction unified: {object_count} geoms (fixed {extreme_fixed} extreme values)")


def _optimize_contact_computation(kitchen_root):
    """⭐3: Contact 계산 단순화로 속도 향상
    
    cone="pyramidal" 사용: 더 단순한 마찰 원뿔 모델
    또는 impratio="1" 사용: 충격-제약 비율 단순화
    """
    
    option = kitchen_root.find('option')
    if option is None:
        option = ET.Element('option')
        kitchen_root.insert(0, option)
    
    option.set('cone', 'elliptic')    # ← Changed from 'pyramidal'
    option.set('impratio', '10')      # ← Changed from '1'
    
    logger.info("  ⭐ Contact computation optimized: cone=pyramidal, impratio=1")


def _increase_base_damping(kitchen_root):
    """✅ Increase damping on robot base joints to prevent drifting"""
    
    worldbody = kitchen_root.find('worldbody')
    if worldbody is None:
        return
    
    # Find all robot base bodies and increase joint damping
    for body in worldbody.findall('.//body'):
        body_name = body.get('name', '')
        
        # Check if this is a robot base
        if 'base_link' in body_name.lower():
            for joint in body.findall('joint'):
                joint_name = joint.get('name', '')
                
                # Increase damping significantly for base joints
                if any(keyword in joint_name.lower() for keyword in ['joint_x', 'joint_y', 'joint_th']):
                    old_damping = joint.get('damping', '0')
                    joint.set('damping', '100')  
    
    logger.info("  ✅ Base joint damping increased for stability")


def _fix_floor_friction(kitchen_root):
    """✅ Set proper floor friction to prevent robot sliding"""
    
    worldbody = kitchen_root.find('worldbody')
    if worldbody is None:
        return
    
    floor_count = 0
    
    # Find floor geoms and set friction
    for geom in worldbody.findall('.//geom'):
        name = geom.get('name', '').lower()
        
        # Check if this is a floor geom
        if 'floor' in name or 'ground' in name:
            # Set friction: [sliding, torsional, rolling]
            # Higher sliding friction (1.0-1.5) prevents robot base from sliding
            geom.set('friction', '1.2 0.005 0.0001')
            floor_count += 1
            logger.debug(f"    Floor geom '{geom.get('name')}': friction set to 1.2 0.005 0.0001")
    
    if floor_count > 0:
        logger.info(f"  ✅ Floor friction configured ({floor_count} floor geoms)")
    else:
        logger.warning("  ⚠ No floor geoms found to set friction")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="RoboCasa Kitchen Integration")
    parser.add_argument('--physics-mode', type=str, default='fast',
                        choices=['fast', 'balanced', 'accurate'],
                        help='Physics optimization mode (default: fast)')
    parser.add_argument('--layout', type=str, default='G-shaped',
                        help='Kitchen layout (default: G-shaped)')
    parser.add_argument('--style', type=str, default='modern',
                        help='Kitchen style (default: modern)')
    args = parser.parse_args()
    
    print("="*70)
    print("RoboCasa Kitchen Integration - FINAL VERSION")
    print("="*70)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)s: %(message)s'
    )
    
    print(f"\n🔧 Creating RoboCasa kitchen with TidyBot robots...")
    print(f"   Layout: {args.layout}")
    print(f"   Style: {args.style}")
    print(f"   Physics mode: {args.physics_mode}")
    print("   (This may take 10-30 seconds)\n")
    
    try:
        model = create_robocasa_kitchen(
            layout=args.layout,
            style=args.style,
            robot1_pos=(0.0, 0.0, 0.0),
            robot2_pos=(2.0, 0.0, 0.0),
            physics_mode=args.physics_mode
        )
        
        print("\n" + "="*70)
        print("✅ SUCCESS!")
        print("="*70)
        print(f"\nModel Statistics:")
        print(f"  Bodies:      {model.nbody:4d}")
        print(f"  Joints:      {model.njnt:4d}")
        print(f"  Geoms:       {model.ngeom:4d}")
        print(f"  Actuators:   {model.nu:4d}")
        
        print(f"\nPhysics Settings:")
        print(f"  timestep:           {model.opt.timestep}s ({1/model.opt.timestep:.0f} Hz)")
        print(f"  solver:             {['PGS', 'CG', 'Newton'][model.opt.solver]}")
        print(f"  iterations:         {model.opt.iterations}")
        print(f"  noslip_iterations:  {model.opt.noslip_iterations}")
        print(f"  tolerance:          {model.opt.tolerance}")
        
        if model.nbody > 100:
            print(f"\n🎉 Full kitchen with fixtures loaded!")
            print(f"   Includes cabinets, counters, appliances, etc.")
        else:
            print(f"\n✓ Basic kitchen loaded")
            print(f"  (Suitable for testing navigation and control)")
        
        print(f"\n📝 Next steps:")
        print(f"  1. Check robocasa_merged_debug.xml in model/stanford_tidybot/")
        print(f"  2. Set USE_ROBOCASA = True in config/constants.py")
        print(f"  3. Update simulation_manager.py to use this integration")
        print(f"  4. Use create_robocasa_kitchen() function to create models")
        print("="*70)
        
    except Exception as e:
        print(f"\n" + "="*70)
        print(f"❌ FAILED")
        print("="*70)
        print(f"\nError: {e}")
        
        import traceback
        print("\nFull traceback:")
        traceback.print_exc()
        print("="*70)
