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
    print("\n 📦 박스 배치:")
    print("   - 빨간색 박스 (작은 크기)")
    print("   - 파란색 박스 (큰 크기)")
    print("   - 노란색 박스 (큰 크기)")
    print("   - 초록색 박스 (작은 크기)")
    print("\n 🎮 조작 방법:")
    print("   1. [Space] 키: 시뮬레이션 시작 / 다음 작업 진행")
    print("   2. 숫자패드: 베이스 이동 (로봇 헤딩 기준)")
    print("      - 8: 전진 (로봇이 바라보는 방향)")
    print("      - 5: 후진 (뒤로)")
    print("      - 4: 좌측 이동 (로봇 기준 왼쪽)")
    print("      - 6: 우측 이동 (로봇 기준 오른쪽)")
    print("      - 7/9: 좌/우 회전")
    print("      - 2: 정지")
    print("   3. [ESC] 키: 종료")
    print("\n" + "="*60)
    print(f" ▶ [{KEY_START}] 키를 누르면 시작합니다.")
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
    print("      MuJoCo Pick & Place 시뮬레이션 v2.1")
    print("      - 두 개의 Pick & Place 작업 수행")
    print("      - 실행 가능성 체크 기능 포함")
    print("="*60)
    print("\n 🔧 시뮬레이션 초기화 중...")
    print(f" - 모델 경로: {DEFAULT_XML_PATH}")
    print(" - 작업 1: 빨간색 박스 → 파란색 박스")
    print(" - 작업 2: 초록색 박스 → 노란색 박스")
    print("\n ℹ️  작업 실행 전 도달 가능성을 체크합니다.")
    print("    불가능한 경우 베이스 위치 조정을 제안합니다.")
    
    try:
        # 시뮬레이션 매니저 생성
        sim_manager = SimulationManager(DEFAULT_XML_PATH)
        
        # 시작 대기
        if not wait_for_start(sim_manager):
            return 1
            
        # 태스크 실행
        task = PickAndPlaceTask(sim_manager)
        success = task.execute()
        
        # 결과 출력
        print("\n" + "="*60)
        if success:
            print(" 🎉 모든 Pick & Place 작업이 성공적으로 완료되었습니다!")
            print(" ✓ 작업 1: 빨간색 → 파란색 [완료]")
            print(" ✓ 작업 2: 초록색 → 노란색 [완료]")
        else:
            print(" ❌ Pick & Place 작업이 중단되었습니다.")
        print("="*60 + "\n")
        
        return 0 if success else 1
        
    except FileNotFoundError:
        print(f"\n 오류: XML 파일을 찾을 수 없습니다: {DEFAULT_XML_PATH}")
        return 1
    except Exception as e:
        print(f"\n 예외 발생: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())