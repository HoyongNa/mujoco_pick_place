"""프로젝트 전체에서 사용되는 상수 정의"""

# 경로 설정
DEFAULT_XML_PATH = "./model/stanford_tidybot/scene.xml"

# 베이스 제어 파라미터
BASE_CTRL_SLICE = slice(0, 3)
BASE_LIN_STEP = 0.002
BASE_YAW_STEP = 0.002

# 팔 제어 파라미터
ARM_Q_IDX = list(range(3, 10))
ARM_CTRL_IDX = list(range(3, 10))
ARM_KP_HOLD = 400.0
ARM_KD_HOLD = 25.0
ARM_TORQUE_SCALE = 0.01
ARM_TORQUE_LIMIT = 800.0

# Ruckig 파라미터
RUCKIG_MAX_V = 3.0
RUCKIG_MAX_A = 7.0
RUCKIG_MAX_J = 150.0

# 카메라 설정
CAM_AZIMUTH = 135
CAM_ELEVATION = -30
CAM_DISTANCE = 3.0
CAM_LOOKAT = [0.3, 0.0, 0.3]

# 타이밍 파라미터
SETTLE_STEPS_GRASP = 10
SETTLE_STEPS_RELEASE = 25
SETTLE_DT = 0.002

# 키보드 맵핑
KEY_FORWARD = '8'
KEY_BACKWARD = '5'
KEY_LEFT = '4'
KEY_RIGHT = '6'
KEY_ROTATE_LEFT = '7'
KEY_ROTATE_RIGHT = '9'
KEY_STOP = '2'
KEY_START = 'space'