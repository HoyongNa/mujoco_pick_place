"""메인 실행 파일"""

import sys
import time
import keyboard
from config.constants import DEFAULT_XML_PATH, KEY_START
from simulation.simulation_manager import SimulationManager
from tasks.pick_and_place import PickAndPlaceTask

def wait_for_start(sim_manager):
    """시작 키 대기"""
    sim_manager.initialize_viewer()
    print("\n" + "="*60)
    print(" 🤖 MuJoCo Pick & Place 시뮬레이션 준비 완료")
    print("="*60)
    print("\n 🏠 4개 방 구조 (10m x 10m):")
    print("   ┌───────┬───────┐")
    print("   │ 방 1  │ 방 2  │  (북쪽)")
    print("   │ 북서  │ 북동  │")
    print("   ├───────┼───────┤")
    print("   │ 방 3  │ 방 4  │  (남쪽)")
    print("   │ 남서  │ 남동  │")
    print("   └───────┴───────┘")
    print("\n 📦 각 방의 작업:")
    print("   방 1 (북서): 빨간색 박스 → 파란색 박스")
    print("   방 2 (북동): 초록색 박스 → 노란색 박스")
    print("   방 3 (남서): 주황색 박스 → 보라색 박스")
    print("   방 4 (남동): 청록색 박스 → 분홍색 박스")
    print("\n 🎮 조작 방법:")
    print("   1. [Space] 키: 시뮬레이션 시작 / 다음 작업 진행")
    print("   2. 숫자패드: 베이스 이동 (로봇 헤딩 기준)")
    print("      - 8: 전진 (로봇이 바라보는 방향)")
    print("      - 5: 후진 (뒤로)")
    print("      - 4: 좌측 이동 (로봇 기준 왼쪽)")
    print("      - 6: 우측 이동 (로봇 기준 오른쪽)")
    print("      - 7/9: 좌/우 회전")
    print("      - 2: 원점복귀")
    print("   3. [ESC] 키: 종료")
    print("\n" + "="*60)
    print(f" ▶ [{KEY_START}] 키를 누르면 Pick & Place 를 시작합니다.")
    print("="*60)
    
    space_armed = True
    while sim_manager.viewer_manager.is_running():
        if keyboard.is_pressed(KEY_START):
            if space_armed:
                print("\n ▶ 시뮬레이션을 시작합니다!\n")
                return True
            space_armed = False
        else:
            space_armed = True
        time.sleep(0.01)
        
    print("취소: 뷰어가 닫혔습니다.")
    return False

def main():
    print("\n" + "="*60)
    print(" MuJoCo Pick & Place Simulation v3.0")
    print(" 4-Room Environment (10m x 10m)")
    print("="*60)
    
    sim_manager = SimulationManager(DEFAULT_XML_PATH)
    
    if wait_for_start(sim_manager):
        task = PickAndPlaceTask(sim_manager)
        task.execute()
    else:
        print("시뮬레이션이 취소되었습니다.")
        sim_manager.viewer_manager.close()
    
    print("\n시뮬레이션이 종료되었습니다.")

if __name__ == "__main__":
    main()
