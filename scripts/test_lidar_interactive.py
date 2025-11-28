"""
Interactive Lidar Mapping Demo
- 키보드로 로봇을 주행시키며 실시간 매핑
- 물리 시뮬레이션/베이스 제어는 MobilityController 스레드가 담당
- 맵핑(Occupancy Grid)은 LidarMappingSystem의 백그라운드 스레드가 담당
- 메인 스레드는 시각화만 주기적으로 업데이트
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import threading
from typing import Optional

import numpy as np
import mujoco
import mujoco.viewer

from config.constants import DEFAULT_XML_PATH
from lidar_mapping.mapping_system import LidarMappingSystem
from lidar_mapping.visualizer import MapVisualizer
from controllers.base.mobility_controller import MobilityController  # 주행 컨트롤러


class InteractiveMappingDemo:
    """인터랙티브 매핑 데모 (MobilityController 통합)"""

    def __init__(self, xml_path: str = DEFAULT_XML_PATH):
        # ---------- MuJoCo ----------
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # ---------- Mapping ----------
        self.mapping_system = LidarMappingSystem(self.model, self.data)
        self.mapping_system.lidar.set_range_limit(6.0)  # 라이다 최대 거리 6m 설정

        # ---------- Visualizer ----------
        # (model/data를 생략해도 mapping_system에서 자동 참조)
        self.visualizer = MapVisualizer(self.mapping_system)

        # ---------- Mobility Controller (주행/스텝) ----------
        # MobilityController 는 외부 공유 버퍼/락을 통해 베이스 명령을 읽습니다.
        self.base_cmd_ref = np.zeros(3, dtype=float)  # [vx, vy, yaw_rate]
        self.base_lock = threading.Lock()
        self.mobility: Optional[MobilityController] = None

        # ---------- Viewer ----------
        self.mj_viewer = None  # mujoco.viewer.launch_passive(...)

        # ---------- 상태 ----------
        self.running = True
        self.mapping_active = False
        self.viz_active = True

        print("\n" + "=" * 60)
        print("INTERACTIVE LIDAR MAPPING DEMO (with MobilityController)")
        print("=" * 60)
        self.print_controls()

    # ----------------------------- 안내 -----------------------------
    def print_controls(self):
        print("\n📋 Controls (MuJoCo Viewer 창에서 입력):")
        print("  Space    - Start/Stop mapping")
        print("  V        - Toggle visualization")
        print("  C        - Clear map")
        print("  R        - Reset robot pose")
        print("  M        - Save map")
        print("  ESC      - Exit")
        print("\n💡 주행(전/후/좌/우/회전) 키는 프로젝트의 KeyboardHandler/Constants에 따릅니다.")
        print("   (MobilityController가 전역 키보드 입력을 읽어 베이스를 구동합니다.)\n")

    # ----------------------------- 뷰어 -----------------------------
    def initialize_mujoco_viewer(self):
        """MuJoCo viewer 초기화 및 키콜백 등록(맵핑/시각화 토글 전용)"""
        self.mj_viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self.key_callback,
        )

        # 카메라 기본값(필요시 조정)
        self.mj_viewer.cam.azimuth = 90
        self.mj_viewer.cam.elevation = -45
        self.mj_viewer.cam.distance = 10
        self.mj_viewer.cam.lookat[:] = [0, 0, 0]

        print("✅ MuJoCo viewer initialized")

    def key_callback(self, key: int):
        """뷰어에서 들어오는 키(토글/관리용) 처리. 주행 키는 MobilityController가 처리."""
        import glfw

        if key == glfw.KEY_SPACE:
            self.toggle_mapping()
        elif key == glfw.KEY_V:
            self.toggle_visualization()
        elif key == glfw.KEY_C:
            self.clear_map()
        elif key == glfw.KEY_R:
            self.reset_robot()
        elif key == glfw.KEY_M:
            self.save_map()
        elif key == glfw.KEY_ESCAPE:
            self.running = False

    # ----------------------------- Mobility -----------------------------
    def start_mobility(self):
        """MobilityController 시작 (물리 스텝/주행/뷰어 동기화)"""
        if self.mobility is not None:
            return
        self.mobility = MobilityController(
            model=self.model,
            data=self.data,
            base_cmd_ref=self.base_cmd_ref,
            base_lock=self.base_lock,
            viewer=self.mj_viewer,
        )
        self.mobility.start()
        print("[Demo] MobilityController started")

    def stop_mobility(self):
        """MobilityController 정지"""
        if self.mobility is None:
            return
        self.mobility.stop(timeout=1.0, zero_on_stop=True)
        self.mobility = None
        print("[Demo] MobilityController stopped")

    # ----------------------------- 로봇/맵 관리 -----------------------------
    def toggle_mapping(self):
        if self.mapping_active:
            self.mapping_system.stop_mapping()
            self.mapping_active = False
            print("⏸️  Mapping paused")
        else:
            self.mapping_system.start_mapping(update_rate=10)
            self.mapping_active = True
            print("▶️  Mapping started")

    def toggle_visualization(self):
        self.viz_active = not self.viz_active
        print(f"👁️  Visualization: {'ON' if self.viz_active else 'OFF'}")

    def clear_map(self):
        self.mapping_system.clear_map()
        print("🗑️  Map cleared")

    def reset_robot(self):
        """로봇 포즈 초기화 (qpos/qvel)"""
        self.data.qpos[:] = 0.0
        if self.data.qvel is not None and self.data.qvel.size > 0:
            self.data.qvel[:] = 0.0
        # 이동 명령도 리셋
        with self.base_lock:
            self.base_cmd_ref[:] = 0.0
        print("🔄 Robot reset to origin")

    def save_map(self):
        fname = self.mapping_system.save_map()
        print(f"💾 Map saved to: {fname}")

    # ----------------------------- 메인 루프 -----------------------------
    def run(self):
        # 1) 뷰어 시작
        self.initialize_mujoco_viewer()

        # 2) MobilityController 시작(주행/스텝 담당)
        self.start_mobility()

        # 3) 시각화 준비
        if self.viz_active:
            self.visualizer.setup_figure()

        print("🎯 Press SPACE to start mapping!")

        # 4) 메인 루프: 시각화만 주기적으로 업데이트
        last_viz = time.time()
        viz_dt = 0.10  # 10 Hz

        while self.running:
            # 뷰어가 닫히면 종료
            if self.mj_viewer and (not self.mj_viewer.is_running()):
                break

            if self.viz_active and (time.time() - last_viz) > viz_dt:
                try:
                    self.visualizer.update_once()
                except Exception as e:
                    print(f"[Visualizer] Update error: {e}")
                last_viz = time.time()

            time.sleep(0.01)

        # 종료 처리
        self.cleanup()

    # ----------------------------- 정리 -----------------------------
    def cleanup(self):
        print("\n🔚 Shutting down...")

        # 주행/스텝 중지
        self.stop_mobility()

        # 맵핑 중지
        if self.mapping_active:
            self.mapping_system.stop_mapping()

        # 시각화 종료
        if self.visualizer:
            self.visualizer.close()

        # 뷰어 종료
        if self.mj_viewer:
            try:
                self.mj_viewer.close()
            except Exception:
                pass

        # 통계 출력
        stats = self.mapping_system.get_statistics()
        print("\n" + "=" * 60)
        print("FINAL STATISTICS")
        print("=" * 60)
        print(f"  Total scans: {stats.get('total_scans', 0)}")
        print(f"  Occupied cells: {stats.get('occupied_cells', 0)}")
        print(f"  Free cells: {stats.get('free_cells', 0)}")
        print(f"  Coverage: {stats.get('coverage_percent', 0.0):.1f}%")

        if stats.get("occupied_cells", 0) > 200:
            print("\n✅ Excellent mapping results!")
        elif stats.get("occupied_cells", 0) > 100:
            print("\n⚠️ Partial mapping achieved")
        else:
            print("\n❌ Limited mapping results")

        print("\n👋 Goodbye!")


def main():
    try:
        demo = InteractiveMappingDemo()
        demo.run()
    except KeyboardInterrupt:
        print("\n\n⚠️ Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
