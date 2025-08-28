"""메인 실행 파일 - 경로 계획 기능 추가 버전"""

import sys
import time
import keyboard
import numpy as np
from config.constants import DEFAULT_XML_PATH, KEY_START
from simulation.simulation_manager import SimulationManager
from tasks.pick_and_place import PickAndPlaceTask
from path_planning.map_processor import MapProcessor

# 각 방의 목표 위치 정의 (10m x 10m 환경)
ROOM_POSITIONS = {
    '1': (-2.0, 2.0),   # 방 1: 북서
    '2': (2.0, 2.0),    # 방 2: 북동  
    '3': (-2.0, -2.0),  # 방 3: 남서
    '4': (2.0, -2.0)    # 방 4: 남동
}

class NavigationPickPlaceSystem:
    """경로 계획과 Pick & Place를 통합한 시스템"""
    
    def __init__(self):
        self.sim_manager = SimulationManager(DEFAULT_XML_PATH)
        self.pick_place_task = PickAndPlaceTask(self.sim_manager)
        self.is_navigating = False
        self.current_mode = "teleop"  # "teleop", "navigation", "pick_place"
        self.map_initialized = False
        
    def initialize(self):
        """시스템 초기화"""
        print("\n" + "="*60)
        print(" 🤖 MuJoCo Navigation & Pick & Place 시뮬레이션")
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
        print("   [1-4] 키: 해당 번호 방으로 자동 이동")
        print("   [Space] 키: Pick & Place 작업 실행")
        print("   [숫자패드]: 베이스 수동 이동 (teleop 모드)")
        print("      - 8: 전진, 5: 후진")
        print("      - 4: 왼쪽, 6: 오른쪽")
        print("      - 7/9: 좌/우 회전")
        print("      - 2: 정지")
        print("   [ESC] 키: 종료")
        print("\n" + "="*60)
        
        # 뷰어 초기화
        self.sim_manager.initialize_viewer()
        
        # 맵 생성 또는 로드 시도
        self._initialize_map()
        
        print("\n ▶ 준비 완료! 키보드로 조작하세요.")
        print("="*60)
        
    def _initialize_map(self):
        """맵 초기화 - 기존 맵이 없으면 생성"""
        print("\n 🗺️  맵 초기화 중...")
        
        # 기존 맵 파일 찾기
        import os
        map_files = [f for f in os.listdir('.') if f.endswith('.npz') and 'map' in f.lower()]
        

        map_path = map_files[0]
      
        
        # 경로 추종 컨트롤러 초기화
        if self.sim_manager.initialize_path_controller(map_path):
            self.map_initialized = True
            print("  ✅ 맵 초기화 완료")
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
                        self.sim_manager.data.ctrl[:3] = current_base_pos
                    
                    # 전환 중 직접 물리 시뮬레이션 실행 (10 steps = 0.02초)
                    
                        
                    time.sleep(1)  # 2ms
                    
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
        """키보드 입력 처리"""
        # ESC - 종료
        if keyboard.is_pressed('escape'):
            self.sim_manager.viewer_manager.close()
            return
            
        # 숫자 1-4: 방으로 이동
        for room_num in ['1', '2', '3', '4']:
            if keyboard.is_pressed(room_num) and not self.is_navigating:
                self._navigate_to_room(room_num)
                time.sleep(0.3)  # 키 디바운싱
                break
                
        # Space: Pick & Place 실행
        if keyboard.is_pressed('space') and not self.is_navigating:
            if self.current_mode != "pick_place":
                self._start_pick_place()
                time.sleep(0.3)  # 키 디바운싱
                
    def _navigate_to_room(self, room_num):
        """지정된 방으로 자동 이동"""
        if not self.map_initialized:
            print(f"\n ⚠️  맵이 초기화되지 않아 자동 이동할 수 없습니다.")
            print("     숫자패드로 수동 이동하세요.")
            return
            
        target = ROOM_POSITIONS[room_num]
        
        # 현재 위치 확인
        current_pos = self.sim_manager.data.qpos[:2]
        distance = np.linalg.norm(np.array(target) - current_pos)
        
        print(f"\n 🚗 방 {room_num}로 이동 시작")
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
        
        # 현재 위치 확인하여 가장 가까운 방 찾기
        current_pos = self.sim_manager.data.qpos[:2]
        closest_room = None
        min_distance = float('inf')
        
        for room_num, room_pos in ROOM_POSITIONS.items():
            dist = np.linalg.norm(np.array(room_pos) - current_pos)
            if dist < min_distance:
                min_distance = dist
                closest_room = room_num
                
        room_names = {
            '1': "방 1 (북서)",
            '2': "방 2 (북동)",
            '3': "방 3 (남서)",
            '4': "방 4 (남동)"
        }
        
        print(f"    현재 위치에서 가장 가까운 방: {room_names[closest_room]}")
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
        
        # Pick & Place 태스크 실행 (단일 작업)
        task_index = int(closest_room) - 1
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
        
        # 컨트롤러 정지
        if self.is_navigating:
            self.sim_manager.stop_path_control()
        
        self.sim_manager.stop_mobility_control()
        self.sim_manager.viewer_manager.close()
        
        print("시뮬레이션이 종료되었습니다.")

def main():
    """메인 함수"""
    print("\n" + "="*60)
    print(" MuJoCo Navigation & Pick & Place Simulation v4.0")
    print(" Integrated Path Planning and Manipulation System")
    print("="*60)
    
    # 시스템 생성 및 실행
    system = NavigationPickPlaceSystem()
    system.initialize()
    system.execute()

if __name__ == "__main__":
    main()
