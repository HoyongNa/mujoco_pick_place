"""프로젝트 전체에서 사용되는 상수 정의"""

# ==================== 환경 선택 ====================
# 환경 전환: True = RoboCasa 주방, False = 기존 4방 씬
USE_ROBOCASA = False  # ← 이것을 True로 바꾸면 RoboCasa 사용

# RoboCasa 설정 (USE_ROBOCASA=True일 때만 사용)
ROBOCASA_LAYOUT = "G-shaped"  # G-shaped, U-shaped, L-shaped, parallel, one-wall, island, peninsula
ROBOCASA_STYLE = "modern"     # modern, rustic, industrial, scandinavian, traditional, coastal, mediterranean

# ⚡ Physics mode for simulation speed optimization
# Options: "fast", "balanced", "accurate"
# - "fast": PGS solver, 20 iterations (개발/테스트용, 가장 빠름)
# - "balanced": Newton solver, 50 iterations (기본 추천, 속도/정확도 균형)
# - "accurate": Newton solver, 100 iterations (정밀 조작 작업용, 느림)
PHYSICS_MODE = "balanced"

# 로봇 시작 위치 (환경에 따라 다름)z
if USE_ROBOCASA:
    # RoboCasa 주방 중앙에 배치
    ROBOT1_START_POS = (2.5, -2.0, 0.0)   # Robot 1: 중앙 왼쪽
    ROBOT2_START_POS = (1.5, -4.5, 0.0)   # Robot 2: 중앙 오른쪽

# ==================== XML 경로 ====================
if USE_ROBOCASA:
    # RoboCasa는 런타임에 생성되므로 DEFAULT_XML_PATH는 사용하지 않음
    # robocasa_integration.py의 create_robocasa_kitchen() 사용
    DEFAULT_XML_PATH = None  
else:
    # 기본 4방 씬
    DEFAULT_XML_PATH = "/home/hoyongna/mujoco_pick_place/model/stanford_tidybot/scene_dual_robot.xml"

# ==================== 로봇 인덱스 (환경별 분리) ====================
# ⚠️  WARNING: RoboCasa 모드로 전환 시 반드시 verify_robot_indices.py를 실행하여
#     실제 인덱스를 확인하고 아래 값들을 업데이트해야 합니다!
#     RoboCasa는 수백 개의 주방 오브젝트를 추가하여 인덱스가 크게 변경됩니다.

if USE_ROBOCASA:
    # Heading (theta) index is the 3rd element of base qpos
    ROBOT1_HEADING_IDX = 44  # BASE_Q_SLICE.stop - 1
    ROBOT2_HEADING_IDX = 62  # ROBOT2_BASE_QPOS_IDX.stop - 1
else:
    ROBOT1_HEADING_IDX = 2   # For simple scene
    ROBOT2_HEADING_IDX = 20  # For simple scene

if USE_ROBOCASA:
    # ==================== RoboCasa 환경 인덱스 ====================
    # ⚠️  이 값들은 robocasa_merged_debug.xml 기준입니다
    # 새로운 RoboCasa 씬을 생성한 후 verify_robot_indices.py를 실행하여 확인 필요!
    
    BASE_Q_SLICE = slice(42, 45)  # ✅ Robot1 base at qpos 42-44
    BASE_CTRL_SLICE = slice(0, 3)
    
    # Robot 1 Arm
    ARM_Q_IDX = list(range(45, 52))  # ✅ Robot1 arm at qpos 45-51
    ARM_CTRL_IDX = list(range(3, 10))  # ✅ ctrl 3-9
    GRIP_CTRL_IDX = 10
    
    # Robot 2 Base
    ROBOT2_BASE_QPOS_IDX = slice(60, 63)  # ✅ Robot2 base qpos 60-62
    ROBOT2_BASE_Q_SLICE = ROBOT2_BASE_QPOS_IDX  # ✅ Alias for consistency
    ROBOT2_BASE_CTRL_SLICE = slice(11, 14)  # ✅ ctrl 11-13
    
    # Robot 2 Arm
    ROBOT2_ARM_Q_IDX = list(range(63, 70))  # ✅ Robot2 arm at qpos 63-69
    ROBOT2_ARM_CTRL_IDX = list(range(14, 21))  # ✅ ctrl 14-20
    ROBOT2_GRIP_CTRL_IDX = 21
    
else:
    # ==================== 기본 4방 씬 인덱스 ====================
    # Robot 1 Base
    BASE_Q_SLICE = slice(0, 3)  # ✅ Robot1 base at qpos 42-44
    BASE_CTRL_SLICE = slice(0, 3)
    
    # Robot 1 Arm
    ARM_Q_IDX = list(range(3, 10))  # ✅ Robot1 arm at qpos 45-51
    ARM_CTRL_IDX = list(range(3, 10))  # ✅ ctrl 3-9
    GRIP_CTRL_IDX = 10
    

    # Robot 2 Base
    ROBOT2_BASE_QPOS_IDX = slice(18, 21)  # ✅ Robot2 base qpos 18-20
    ROBOT2_BASE_Q_SLICE = ROBOT2_BASE_QPOS_IDX  # ✅ Alias for consistency
    ROBOT2_BASE_CTRL_SLICE = slice(11, 14)  # ✅ ctrl 11-13
    
    # Robot 2 Arm
    ROBOT2_ARM_Q_IDX = list(range(21, 28))  # ✅ Robot2 arm at qpos 63-69
    ROBOT2_ARM_CTRL_IDX = list(range(14, 21))  # ✅ ctrl 14-20
    ROBOT2_GRIP_CTRL_IDX = 21

# ==================== 공통 제어 파라미터 ====================
ARM_KP_HOLD = 2000.0  # P 게인 증가 (중력 보상 강화)
ARM_KD_HOLD = 200.0    # D 게인 증가 (안정-성 향상)
ARM_KI_HOLD = 100  # I 게인 증가 (정상 상태 오차 제거)

# ===== 중요: 토크 스케일 수정 =====
# XML에서 actuator gear=100이므로:
# - data.ctrl에 1 넣으면 → 실제 토크 100 Nm
# - 따라서 SCALE = 1.0이면 계산된 토크를 gear로 나눈 값이 ctrl에 들어감
ARM_TORQUE_SCALE = 0.01  # gear=100 고려, 실제로는 gear*ctrl이 토크
ARM_TORQUE_LIMIT = 100.0  # ctrl 기준 제한 (실제 토크는 gear*100 = 10000 Nm)

# Ruckig 파라미터 
RUCKIG_MAX_V = 1  
RUCKIG_MAX_A = 3  
RUCKIG_MAX_J = 5 

# 카메라 설정
CAM_AZIMUTH = 135
CAM_ELEVATION = -30
CAM_DISTANCE = 3.0
CAM_LOOKAT = [0.3, 0.0, 0.3]

# # 타이밍 파라미터
# SETTLE_STEPS_GRASP = 200  # 10에서 100으로 증가하여 그리퍼가 충분히 닫히도록 함
# SETTLE_STEPS_RELEASE = 50  # 25에서 50으로 증가
# SETTLE_DT = 0.002

# Robot1 키보드 맵핑
KEY_FORWARD = 'w'
KEY_BACKWARD = 's'
KEY_LEFT = 'a'
KEY_RIGHT = 'd'
KEY_ROTATE_LEFT = 'q'
KEY_ROTATE_RIGHT = 'e'
KEY_STOP = 'c'
KEY_START = 'space'


# Robot2 키보드 맵핑
KEY_FORWARD_robot2 = '8'
KEY_BACKWARD_robot2 = '5'
KEY_LEFT_robot2 = '4'
KEY_RIGHT_robot2 = '6'
KEY_ROTATE_LEFT_robot2 = '7'
KEY_ROTATE_RIGHT_robot2 = '9'
KEY_STOP_robot2 = '2'