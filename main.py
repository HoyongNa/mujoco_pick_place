import sys
import time
import keyboard  # pip install keyboard
from simulation import PickAndPlaceSimulation

# 경로 하드코딩 (필요 시 여기만 수정)
XML_PATH = "./model/stanford_tidybot/scene.xml"

def wait_for_space(sim):
    """뷰어를 띄우고 Mobility 텔레옵 상태에서 [Space]가 눌릴 때까지 대기."""
    sim.initialize_viewer()  # 내부에서 Mobility 시작
    print("\n=== 준비 완료 ===")
    print("▶ GUI에서 [Space] 키를 누르면 Pick & Place가 시작됩니다. (ESC로 창 닫기)")

    # 간단한 디바운스
    space_armed = True
    while sim.viewer is not None and sim.viewer.is_running():
        if keyboard.is_pressed("space"):
            if space_armed:
                print("▶ Space 입력 감지: 시작합니다.")
                return True
            space_armed = False
        else:
            space_armed = True
        time.sleep(0.01)

    print("취소: 뷰어가 닫혔습니다.")
    return False


def main():
    print("\n========================================")
    print(" MuJoCo Pick & Place 시뮬레이션 (GUI)")
    print("========================================")
    print(" 시뮬레이션 초기화 중...")
    print(f" - 모델 경로: {XML_PATH}")

    try:
        sim = PickAndPlaceSimulation(XML_PATH)

        # ▶ Space 대기
        if not wait_for_space(sim):
            return 1

        # ▶ Pick & Place 실행
        ok = sim.pick_and_place()

        print("\n========================================")
        if ok:
            print(" ✓ Pick & Place 작업이 성공적으로 완료되었습니다!")
        else:
            print(" ✗ Pick & Place 작업이 실패했습니다.")
        print("========================================\n")

        return 0 if ok else 1

    except FileNotFoundError:
        print("\n 오류: XML 파일을 찾을 수 없습니다.")
        print(f" 경로를 확인하세요: {XML_PATH}")
        return 1
    except Exception as e:
        print(f"\n 예외 발생: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())