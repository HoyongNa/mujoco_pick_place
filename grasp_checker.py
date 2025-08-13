# grasp_checker.py
# 파지 성공 여부 판단 클래스 (GUI 지원)

import mujoco
import numpy as np
import time

class GraspChecker:
    """
    MuJoCo 시뮬레이션 상에서 로봇 그리퍼의 파지 성공 여부를 판단하는 클래스 (GUI 지원)
    """
    def __init__(self, model, data, ee_site_id, left_pad_body_id, right_pad_body_id, viewer=None):
        self.model = model  # MuJoCo 모델 객체 저장
        self.data = data  # MuJoCo 데이터 객체 저장
        self.ee_site_id = ee_site_id  # 엔드이펙터 site ID
        self.left_pad_body_id = left_pad_body_id  # 왼쪽 패드 body ID
        self.right_pad_body_id = right_pad_body_id  # 오른쪽 패드 body ID
        self.viewer = viewer  # MuJoCo viewer 객체

    def update_viewer(self):
        """viewer가 있으면 업데이트"""
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def wait_until_grasped(self, threshold=1.0, max_time=2.0, min_pad_distance=0.02):
        """
        일정 시간 동안 시뮬레이션을 반복하면서 파지 여부를 실시간 확인

        Args:
            threshold: 파지를 감지할 최소 접촉력
            max_time: 파지를 기다리는 최대 시간 (초)
            min_pad_distance: 너무 닫힌 상태로 간주할 최소 패드 간 거리

        Returns:
            파지 성공 여부 (True/False)
        """
        steps = int(max_time / self.model.opt.timestep)  # 최대 시간에 대응하는 시뮬레이션 스텝 수 계산
        viewer_update_interval = 5  # GUI 업데이트 주기
        last_print_time = time.time()
        
        for step in range(steps):  # 일정 시간 동안 반복 수행
            mujoco.mj_step(self.model, self.data)  # 시뮬레이션 한 스텝 진행
            
            # GUI viewer 업데이트
            if step % viewer_update_interval == 0:
                self.update_viewer()

            pad_left = self.data.xipos[self.left_pad_body_id]  # 왼쪽 패드의 3D 위치
            pad_right = self.data.xipos[self.right_pad_body_id]  # 오른쪽 패드의 3D 위치
            pad_distance = np.linalg.norm(pad_left - pad_right)  # 패드 간 거리 계산

            if pad_distance < min_pad_distance:  # 너무 닫혀 있는 경우는 무시
                continue

            # 접촉 검사
            for i in range(self.data.ncon):  # 모든 접촉점 확인
                force = np.zeros(6)  # 접촉력 (force + torque) 초기화
                mujoco.mj_contactForce(self.model, self.data, i, force)  # i번째 접촉의 힘 계산
                
                # 접촉력이 임계값 이상이면 파지 성공
                if force[0] > threshold:  # x방향 힘이 임계값 이상이면 파지 성공
                    print(f"파지 성공 (force: {force[0]:.2f}N, pad_dist: {pad_distance:.3f}m)")
                    # 성공 시 viewer 업데이트
                    self.update_viewer()
                    return True  # 파지 성공 반환
                    
            # 진행 상황 출력 (GUI 모드에서 1초마다)
            if self.viewer is not None and time.time() - last_print_time > 1.0:
                print(f"  파지 확인 중... (pad_dist: {pad_distance:.3f}m)")
                last_print_time = time.time()
                
            # ESC 키 체크 (viewer가 닫혔는지 확인)
            if self.viewer is not None and not self.viewer.is_running():
                print(" 사용자가 시뮬레이션을 중단했습니다.")
                return False

        print("파지 실패 (시간 초과)")  # 시간 초과 또는 조건 불충족
        return False  # 파지 실패 반환