# -*- coding: utf-8 -*-
'''"""메인 실행 파일 - 경로 계획 기능 추가(F8~F11, 방 이름=1번방~4번방) 버전"""'''

import sys
import time
import os
from pynput import keyboard
import numpy as np
import threading

from config.constants import DEFAULT_XML_PATH, KEY_START
from simulation.simulation_manager import SimulationManager
from tasks.pick_and_place import PickAndPlaceTask
# from path_planning.map_processor import MapProcessor  # 필요 시 활성화

# 각 방의 목표 위치 정의 (10m x 10m 환경)
ROOM_POSITIONS = {
    '1번방': (-2.0,  2.0),   # 북서
    '2번방': ( 2.0,  2.0),   # 북동
    '3번방': (-2.0, -2.0),   # 남서
    '4번방': ( 2.0, -2.0)    # 남동
}

ROOM_LABELS = {
    '1번방': '1번방 (북서)',
    '2번방': '2번방 (북동)',
    '3번방': '3번방 (남서)',
    '4번방': '4번방 (남동)'
}

# Pick&Place tasks 인덱스 매핑 (tasks[0]~tasks[3]과 1번방~4번방의 순서를 매칭)
ROOM_TO_TASK_IDX = {'1번방': 0, '2번방': 1, '3번방': 2, '4번방': 3}


class NavigationPickPlaceSystem:
    """경로 계획과 Pick & Place를 통합한 시스템 (F8~F11 함수키 사용, 방 이름=1번방~4번방)"""

    def __init__(self):
        self.sim_manager = SimulationManager(DEFAULT_XML_PATH)
        self.pick_place_task = PickAndPlaceTask(self.sim_manager)
        self.is_navigating = False
        self.current_mode = "teleop"  # "teleop", "navigation", "pick_place"
        self.map_initialized = False
        
        # pynput 키보드 상태 추적
        self.pressed_keys = set()
        self._key_lock = threading.Lock()
        self.last_key_time = {}  # 키 디바운싱용
        self.key_debounce_time = 0.3  # 300ms 디바운싱
        
        # 키보드 리스너 설정
        self.keyboard_listener = keyboard.Listener(
            on_press=self._on_key_press,
            on_release=self._on_key_release
        )
        self.keyboard_listener.start()

    def _on_key_press(self, key):
        """키 눌림 이벤트 처리"""
        try:
            with self._key_lock:
                # F8-F11 키 처리
                if key == keyboard.Key.f8:
                    self.pressed_keys.add('f8')
                elif key == keyboard.Key.f9:
                    self.pressed_keys.add('f9')
                elif key == keyboard.Key.f10:
                    self.pressed_keys.add('f10')
                elif key == keyboard.Key.f11:
                    self.pressed_keys.add('f11')
                # Space 키
                elif key == keyboard.Key.space:
                    self.pressed_keys.add('space')
                # ESC 키
                elif key == keyboard.Key.esc:
                    self.pressed_keys.add('escape')
        except AttributeError:
            pass

    def _on_key_release(self, key):
        """키 릴리즈 이벤트 처리"""
        try:
            with self._key_lock:
                # F8-F11 키 처리
                if key == keyboard.Key.f8:
                    self.pressed_keys.discard('f8')
                elif key == keyboard.Key.f9:
                    self.pressed_keys.discard('f9')
                elif key == keyboard.Key.f10:
                    self.pressed_keys.discard('f10')
                elif key == keyboard.Key.f11:
                    self.pressed_keys.discard('f11')
                # Space 키
                elif key == keyboard.Key.space:
                    self.pressed_keys.discard('space')
                # ESC 키
                elif key == keyboard.Key.esc:
                    self.pressed_keys.discard('escape')
        except AttributeError:
            pass

    def _is_key_pressed(self, key_name):
        """특정 키가 눌렸는지 확인 (디바운싱 포함)"""
        with self._key_lock:
            if key_name in self.pressed_keys:
                current_time = time.time()
                last_time = self.last_key_time.get(key_name, 0)
                
                # 디바운싱 체크
                if current_time - last_time > self.key_debounce_time:
                    self.last_key_time[key_name] = current_time
                    return True
            return False

    def initialize(self):
        """시스템 초기화"""
        print("\n" + "=" * 60)
        print(" 🤖 MuJoCo Navigation & Pick & Place 시뮬레이션")
        print("=" * 60)

        print("\n 🏠 4개 방 구조 (10m x 10m):")
        print("   ┌────────┬────────┐")
        print("   │ 1번방  │ 2번방  │  (북쪽)")
        print("   │  북서  │  북동  │")
        print("   ├────────┼────────┤")
        print("   │ 3번방  │ 4번방  │  (남쪽)")
        print("   │  남서  │  남동  │")
        print("   └────────┴────────┘")

        print("\n 📦 각 방의 작업:")
        print("   1번방(북서): 빨간색 박스 → 파란색 박스")
        print("   2번방(북동): 초록색 박스 → 노란색 박스")
        print("   3번방(남서): 주황색 박스 → 보라색 박스")
        print("   4번방(남동): 청록색 박스 → 분홍색 박스")

        print("\n 🎮 조작 방법:")
        print("   [F8–F11] 키: 해당 방으로 자동 이동 (F8=1번방, F9=2번방, F10=3번방, F11=4번방)")
        print("   [Space] 키: Pick & Place 작업 실행")
        print("   [숫자패드]: 베이스 수동 이동 (teleop 모드)")
        print("      - 8: 전진, 5: 후진")
        print("      - 4: 왼쪽, 6: 오른쪽")
        print("      - 7/9: 좌/우 회전")
        print("      - 2: 정지")
        print("   [ESC] 키: 종료")
        print("\n" + "=" * 60)

        # 뷰어 초기화
        self.sim_manager.initialize_viewer()

        # 맵 생성 또는 로드 시도
        self._initialize_map()

        print("\n ▶ 준비 완료! 키보드로 조작하세요.")
        print("=" * 60)

    def _initialize_map(self):
        """맵 초기화 - 기존 맵이 없으면 비활성화"""
        print("\n 🗺️  맵 초기화 중...")

        map_files = [f for f in os.listdir('.') if f.endswith('.npz') and 'map' in f.lower()]
        if not map_files:
            print("  ⚠️  맵 파일(.npz)을 찾지 못했습니다. 경로 계획 기능 비활성화.")
            self.map_initialized = False
            return

        # 가장 최근(수정시각) 맵 파일 사용
        map_files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        map_path = map_files[0]

        # 경로 추종 컨트롤러 초기화
        if self.sim_manager.initialize_path_controller(map_path):
            self.map_initialized = True
            print(f"  ✅ 맵 초기화 완료: {map_path}")
        else:
            print("  ⚠️  맵 초기화 실패 - 경로 계획 기능 비활성화")
            self.map_initialized = False

    def execute(self):
        """메인 실행 루프"""
        try:
            while self.sim_manager.viewer_manager.is_running():
                # 키 입력 처리
                self._handle_keyboard_input()

                # 네비게이션 완료 체크
                if self.current_mode == "navigation" and self.sim_manager.is_navigation_complete():
                    print("\n ✅ 목표 위치 도착! Teleop 모드로 전환")
                    self.current_mode = "teleop"

                    # 부드러운 전환을 위해 현재 위치 저장
                    current_base_pos = self.sim_manager.data.qpos[:3].copy()

                    # PathController 정지 (현재 위치 유지)
                    self.sim_manager.stop_path_control()

                    # 전환 중 물리 시뮬레이션 계속 실행 (제어 공백 방지)
                    with self.sim_manager.base_lock:
                        self.sim_manager.base_cmd_ref[:] = current_base_pos
                        # 시스템 구조에 따라 ctrl 사용 여부는 달라질 수 있습니다.
                        self.sim_manager.data.ctrl[:3] = current_base_pos  # 기존 코드 유지

                    # 전환 중 직접 물리 시뮬레이션 실행 (10 steps = 0.02초)
                    time.sleep(1)  # 주석과 실제 값 불일치 있었으나 원래 코드 유지

                    # MobilityController 시작 (현재 위치로)
                    with self.sim_manager.base_lock:
                        self.sim_manager.base_cmd_ref[:] = current_base_pos
                    self.sim_manager.start_mobility_control()

                    self.is_navigating = False

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n종료 중...")
        finally:
            self.cleanup()

    def _handle_keyboard_input(self):
        """키보드 입력 처리 (F8~F11)"""
        # ESC - 종료
        if self._is_key_pressed('escape'):
            self.sim_manager.viewer_manager.close()
            return

        # F8~F11: 방으로 이동
        fkey_to_room = {'f8': '1번방', 'f9': '2번방', 'f10': '3번방', 'f11': '4번방'}
        for fkey, room in fkey_to_room.items():
            if self._is_key_pressed(fkey) and not self.is_navigating:
                self._navigate_to_room(room)
                break

        # Space: Pick & Place 실행
        if self._is_key_pressed('space') and not self.is_navigating:
            if self.current_mode != "pick_place":
                self._start_pick_place()

    def _navigate_to_room(self, room_key: str):
        """지정된 방으로 자동 이동 (room_key는 '1번방'~'4번방')"""
        if not self.map_initialized:
            print("\n ⚠️  맵이 초기화되지 않아 자동 이동할 수 없습니다.")
            print("     숫자패드로 수동 이동하세요.")
            return

        if room_key not in ROOM_POSITIONS:
            print(f"\n ❌ 알 수 없는 방 키: {room_key}")
            return

        target = ROOM_POSITIONS[room_key]

        # 현재 위치 확인
        current_pos = self.sim_manager.data.qpos[:2]
        distance = np.linalg.norm(np.array(target) - current_pos)

        print(f"\n 🚗 {ROOM_LABELS.get(room_key, room_key)}으로 이동 시작")
        print(f"    현재 위치: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
        print(f"    목표 위치: ({target[0]:.2f}, {target[1]:.2f})")
        print(f"    이동 거리: {distance:.2f}m")

        # 기존 컨트롤러 정지
        self.sim_manager.stop_mobility_control(maintain_position=True)

        # 경로 계획 및 추종 시작
        self.sim_manager.start_path_control()

        if self.sim_manager.navigate_to(target, visualize=False):
            self.current_mode = "navigation"
            self.is_navigating = True
            print("    경로 계획 완료, 이동 중...")
        else:
            print("    ❌ 경로를 찾을 수 없습니다.")
            # 실패 시 teleop 모드로 복귀
            self.sim_manager.stop_path_control()
            self.sim_manager.start_mobility_control()
            self.current_mode = "teleop"

    def _start_pick_place(self):
        """Pick & Place 작업 시작"""
        print("\n 📦 Pick & Place 모드 시작")

        # 현재 위치에서 가장 가까운 방 찾기
        current_pos = self.sim_manager.data.qpos[:2]
        closest_room = None
        min_distance = float('inf')

        for room_key, room_pos in ROOM_POSITIONS.items():
            dist = np.linalg.norm(np.array(room_pos) - current_pos)
            if dist < min_distance:
                min_distance = dist
                closest_room = room_key

        print(f"    현재 위치에서 가장 가까운 방: {ROOM_LABELS[closest_room]}")
        print(f"    거리: {min_distance:.2f}m")

        if min_distance > 2.0:
            print("    ⚠️  작업 위치에서 너무 멉니다. 먼저 해당 방으로 이동하세요.")
            return

        # Pick & Place 실행
        self.current_mode = "pick_place"

        # 모든 네비게이션 정지
        if self.is_navigating:
            self.sim_manager.stop_path_control()
            self.is_navigating = False

        # 가장 가까운 방에 해당하는 태스크 선택
        task_index = ROOM_TO_TASK_IDX[closest_room]
        task = self.pick_place_task.tasks[task_index]

        print(f"\n 작업 시작: {task['name']}")

        # 기존 mobility 정지
        self.sim_manager.stop_mobility_control(maintain_position=True)

        # 컨트롤러 설정
        self.pick_place_task._setup_controllers()

        # 실행 가능성 체커 생성
        from tasks.feasibility_checker import FeasibilityChecker
        self.pick_place_task.feasibility_checker = FeasibilityChecker(
            self.sim_manager.model, self.sim_manager.data,
            self.sim_manager.ik_solver, self.sim_manager.config
        )

        # 단일 작업 실행
        success = self.pick_place_task._execute_single_task(
            task["pick"], task["place"], task["name"]
        )

        if success:
            print("\n ✅ Pick & Place 작업 완료!")
        else:
            print("\n ⚠️  Pick & Place 작업 실패. 베이스 위치를 조정하고 다시 시도하세요.")

        # Teleop 모드로 복귀
        self.current_mode = "teleop"
        self.sim_manager.start_mobility_control()
        print("    Teleop 모드로 복귀")

    def cleanup(self):
        """정리 작업"""
        print("\n시뮬레이션 종료 중...")

        # 키보드 리스너 정지
        if hasattr(self, 'keyboard_listener'):
            self.keyboard_listener.stop()

        # 컨트롤러 정지
        if self.is_navigating:
            self.sim_manager.stop_path_control()

        self.sim_manager.stop_mobility_control()
        self.sim_manager.viewer_manager.close()

        print("시뮬레이션이 종료되었습니다.")


def main():
    """메인 함수"""
    print("\n" + "=" * 60)
    print(" MuJoCo Navigation & Pick & Place Simulation v4.0")
    print(" Integrated Path Planning and Manipulation System (F8~F11, 방=1번방~4번방)")
    print("=" * 60)

    # 시스템 생성 및 실행
    system = NavigationPickPlaceSystem()
    system.initialize()
    system.execute()


if __name__ == "__main__":
    main()
