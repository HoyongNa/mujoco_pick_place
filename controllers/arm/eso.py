"""Extended State Observer (ESO) 구현
총 외란(모델 불확실성 + 외부 힘)을 추정하여 보상

✅ 중요: ESO는 가속도를 추정합니다!
- z3 단위: rad/s² (가속도)
- torque_controller에서 토크로 변환: τ = M × z3

✨ 새로운 기능: 가속도 직접 입력 모드
- update_with_acceleration() 메서드 사용 권장
- 명목 가속도를 직접 입력하여 b0 파라미터 불확실성 제거
- 더 정확한 외란 추정 가능
"""

import numpy as np


class LinearESO:
    """선형 확장 상태 관측기 (Linear Extended State Observer)
    
    상태 공간 모델:
        x1 = q      (위치, rad)
        x2 = qd     (속도, rad/s)
        x3 = f(x,t) (외란 가속도, rad/s²) ⭐ 주의: 가속도!
    
    관측 방정식:
        x1̇ = x2
        x2̇ = u_acc + x3  (여기서 u_acc는 명목 가속도, x3는 외란 가속도)
        x3̇ = h(t)  (외란의 시간 변화율 ≈ 0)
    
     올바른 사용법 (가속도 입력 모드 - 권장):
    1. torque_controller에서 명목 가속도 계산: qdd_nominal = M_inv @ (tau_nominal - C - g)
    2. ESO에서 z3를 외란 가속도로 추정: z3 = eso.update_with_acceleration(q, qdd_nominal)
    3. 외란 보상 토크 계산: disturbance_torque = M @ z3
    4. 최종 토크: τ = τ_nominal - disturbance_torque
    
    ⚠️ 기존 토크 입력 모드 (호환성을 위해 유지):
    - update(y, u, use_acceleration_input=False) 사용
    - b0 파라미터로 토크를 가속도로 변환 (b0 ≈ 1/M)
    """
    
    def __init__(self, num_joints, dt, omega_o=6.0, b0=0.1, gain_scaling=1):
        """
        Args:
            num_joints: 관절 개수
            dt: 시간 간격 (model.opt.timestep)
            omega_o: 관측기 대역폭 (Hz)
                    - 높을수록 빠른 수렴하지만 노이즈에 민감하고 발산 위험
                    - 권장: 4~6 Hz (실제 로봇), 6~8 Hz (시뮬레이션)
                    - 기본값 6.0 Hz (안정성 최우선)
            b0: 제어 게인 (1/M의 근사값)
                - b0 ≈ 1/M (M은 관성)
                - 기본값 0.1은 M ≈ 10 kg·m² 가정
                -  가속도 입력 모드에선 미사용
            gain_scaling: 게인 스케일링 팩터 (0.5~1.0)
                        - 1.0: 표준 Butterworth (공격적)
                        - 0.8: 권장값 (균형)
                        - 0.6: 보수적 (노이즈 많음)
        """
        self.num_joints = num_joints
        self.dt = dt
        self.omega_o = omega_o
        self.gain_scaling = gain_scaling
        
        # 상태 추정값
        self.z1 = np.zeros(num_joints)  # 위치 추정 (rad)
        self.z2 = np.zeros(num_joints)  # 속도 추정 (rad/s)
        self.z3 = np.zeros(num_joints)  #  외란 가속도 추정 (rad/s²)
        
        # 관측기 게인 (3차 Butterworth pole placement + scaling)
        # ✨ gain_scaling으로 노이즈 민감도 조절
        self.beta1 = gain_scaling * 3 * omega_o
        self.beta2 = gain_scaling * 3 * omega_o**2
        self.beta3 = gain_scaling * omega_o**3
        
        # 제어 게인 b0
        if isinstance(b0, (list, tuple, np.ndarray)):
            self.b0 = np.array(b0)
        else:
            self.b0 = np.ones(num_joints) * b0
        
        # 통계
        self.max_disturbance = np.zeros(num_joints)
        self.disturbance_history = []
        
        # print(f"✅ Linear ESO 초기화: omega_o={omega_o} Hz, gain_scaling={gain_scaling:.2f}")
        # print(f"   게인: beta1={self.beta1:.2f}, beta2={self.beta2:.2f}, beta3={self.beta3:.2f}")
        # if gain_scaling < 1.0:
        #     print(f"   🚨 보수적 게인 사용 (안정성 향상)")
    
    def update(self, y, u, use_acceleration_input=False):
        """표준 ESO 상태 업데이트
        
        Args:
            y: 측정된 위치 (rad)
            u: 제어 입력
               - use_acceleration_input=False: 토크 (Nm) - 기존 방식
               - use_acceleration_input=True: 가속도 (rad/s²) - 새로운 방식
            use_acceleration_input: True이면 u를 가속도로 해석
            
        Returns:
            z3: 추정된 외란 가속도 (rad/s²) ⭐ 주의: 가속도 단위!
                torque_controller에서 M를 곱해 토크로 변환해야 함
        """
        # 관측 오차
        e = y - self.z1
        
        # ===== 표준 ESO 업데이트 =====
        # z1̇ = z2 + β1·e
        self.z1 += self.dt * (self.z2 + self.beta1 * e)
        
        # z2̇ = z3 + β2·e + u_acc
        # - 가속도 입력 모드: u_acc = u (직접 사용)
        # - 토크 입력 모드: u_acc = b0 * u (b0로 변환)
        if use_acceleration_input:
            u_acc = u
        else:
            u_acc = self.b0 * u
        
        self.z2 += self.dt * (self.z3 + self.beta2 * e + u_acc)
        
        # z3̇ = β3·e
        self.z3 += self.dt * (self.beta3 * e)
        
        # ===== ✅ 안전장치: z3 Saturation (보수적) =====
        # z3는 가속도 단위 (rad/s²)
        # ✨ gain_scaling에 따라 적응적 조정
        z3_limit = 15.0 if self.gain_scaling >= 0.8 else 10.0  # rad/s²
        self.z3 = np.clip(self.z3, -z3_limit, z3_limit)
        
        # 통계 업데이트
        self.max_disturbance = np.maximum(self.max_disturbance, np.abs(self.z3))
        
        return self.z3.copy()
    
    def update_with_acceleration(self, y, qdd):
        """가속도를 직접 입력받는 ESO 업데이트 (권장)
        
        Args:
            y: 측정된 위치 (rad)
            qdd: 명목 가속도 (rad/s²) - 제어기가 원하는 가속도
            
        Returns:
            z3: 추정된 외란 가속도 (rad/s²)
            
        사용 예시:
            # torque_controller에서 명목 가속도 계산
            qdd_nominal = M_inv @ (tau_nominal - C - g)
            
            # ESO 업데이트 (명목 가속도 사용)
            disturbance_acc = eso.update_with_acceleration(q, qdd_nominal)
            
            # 외란 보상 토크 계산
            disturbance_torque = M @ disturbance_acc
            tau = tau_nominal - disturbance_torque
        """
        return self.update(y, qdd, use_acceleration_input=True)
    
    def update_with_measured_acceleration(self, y, qdd_measured):
        """실제 측정된 가속도를 사용하는 ESO 업데이트 (외란 감지 최적화) ✨
        
        ⭐ 핵심 아이디어:
        외부 힘이 작용하면 → 즉시 가속도로 나타남 → ESO가 바로 감지
        
        Args:
            y: 측정된 위치 (rad)
            qdd_measured: 실제 측정된 가속도 (rad/s²) - data.qacc에서 얻음
            
        Returns:
            z3: 추정된 외란 가속도 (rad/s²)
            
        장점:
            1. 외란의 즉각적 감지 (한 스텝 지연 없음)
            2. b0 파라미터 불필요 (변환 과정 제거)
            3. 모델 불확실성 영향 최소화
            
        사용 예시:
            # MuJoCo에서 실제 가속도 가져오기
            qdd_actual = data.qacc[joint_idx]
            
            # ESO 업데이트 (실제 가속도 직접 사용)
            disturbance_acc = eso.update_with_measured_acceleration(q, qdd_actual)
            
            # 외란 보상 토크 계산
            disturbance_torque = M @ disturbance_acc
            tau = tau_nominal - disturbance_torque
            
        비교:
            - update_with_acceleration(): 명목 가속도 사용 (계획 기반)
            - update_with_measured_acceleration(): 실제 가속도 사용 (측정 기반) ← 외란 감지에 더 효과적
        """
        # 관측 오차 계산
        e = y - self.z1
        
        # ===== 실제 가속도 기반 ESO 업데이트 =====
        # z1̇ = z2 + β1·e
        self.z1 += self.dt * (self.z2 + self.beta1 * e)
        
        # z2̇ = z3 + β2·e + qdd_measured
        # ✨ 핵심: 실제 측정된 가속도를 직접 사용
        # 외란이 있으면 qdd_measured에 즉시 반영됨
        self.z2 += self.dt * (self.z3 + self.beta2 * e + qdd_measured)
        
        # z3̇ = β3·e
        # z3는 "시스템의 실제 동역학과 명목 모델의 차이"를 추정
        self.z3 += self.dt * (self.beta3 * e)
        
        # ===== ✅ 안전장치: z3 Saturation =====
        z3_limit = 15.0 if self.gain_scaling >= 0.8 else 10.0  # rad/s²
        self.z3 = np.clip(self.z3, -z3_limit, z3_limit)
        
        # 통계 업데이트
        self.max_disturbance = np.maximum(self.max_disturbance, np.abs(self.z3))
        
        return self.z3.copy()
    
    def reset(self, q_init, qd_init=None):
        """ESO 상태 초기화
        
        Args:
            q_init: 초기 위치
            qd_init: 초기 속도 (None이면 0으로 설정)
        """
        self.z1 = q_init.copy()
        self.z2 = qd_init.copy() if qd_init is not None else np.zeros(self.num_joints)
        self.z3 = np.zeros(self.num_joints)
        self.max_disturbance = np.zeros(self.num_joints)
    
    def get_disturbance_estimate(self):
        """현재 외란 추정값 반환 (rad/s²) ⭐ 가속도 단위!"""
        return self.z3.copy()
    


class NonlinearESO:
    """비선형 확장 상태 관측기 (Nonlinear Extended State Observer)
    
    LESO의 개선 버전:
    - 큰 오차에서 더 빠른 수렴
    - 작은 오차에서 노이즈 억제
    - 비선형 함수 fal() 사용
    
    ⭐ z3는 가속도 단위 (rad/s²)
    ✨ 가속도 직접 입력 모드 지원 (update_with_acceleration)
    """
    
    def __init__(self, num_joints, dt, omega_o=6.0, b0=0.1, 
                 alpha=0.5, delta=0.01, gain_scaling=0.8):
        """
        Args:
            alpha: 비선형성 파라미터 (0.5~1.0)
                  - 작을수록 빠른 수렴 vs 노이즈 민감
            delta: 선형 구간 폭
                  - 작을수록 비선형성 강함
            gain_scaling: 게인 스케일링 팩터 (0.5~1.0)
                        - LinearESO와 동일하게 적용
        """
        self.num_joints = num_joints
        self.dt = dt
        self.omega_o = omega_o
        self.alpha = alpha
        self.delta = delta
        self.gain_scaling = gain_scaling
        
        # 상태
        self.z1 = np.zeros(num_joints)
        self.z2 = np.zeros(num_joints)
        self.z3 = np.zeros(num_joints)  # ⭐ 가속도 (rad/s²)
        
        # 게인 (scaling 적용)
        self.beta1 = self.gain_scaling * 3 * omega_o
        self.beta2 = self.gain_scaling * 3 * omega_o**2
        self.beta3 = self.gain_scaling * omega_o**3
        
        # b0
        if isinstance(b0, (list, tuple, np.ndarray)):
            self.b0 = np.array(b0)
        else:
            self.b0 = np.ones(num_joints) * b0
        
        # 통계
        self.max_disturbance = np.zeros(num_joints)
        
        print(f"✅ Nonlinear ESO 초기화: omega_o={omega_o} Hz, gain_scaling={self.gain_scaling:.2f}, alpha={alpha}")
        print(f"   게인: beta1={self.beta1:.2f}, beta2={self.beta2:.2f}, beta3={self.beta3:.2f}")
    
    def fal(self, e, alpha, delta):
        """비선형 함수 (fast nonlinear function)
        
        fal(e, α, δ) = {
            |e|^α · sign(e),  if |e| > δ  (큰 오차: 비선형 수렴)
            e / δ^(1-α),      if |e| ≤ δ  (작은 오차: 선형 안정화)
        }
        """
        result = np.zeros_like(e)
        mask = np.abs(e) > delta
        
        # 큰 오차: 비선형 (빠른 수렴)
        result[mask] = np.power(np.abs(e[mask]), alpha) * np.sign(e[mask])
        
        # 작은 오차: 선형 (노이즈 억제)
        result[~mask] = e[~mask] / (delta ** (1 - alpha))
        
        return result
    
    def update(self, y, u, use_acceleration_input=False):
        """NESO 상태 업데이트
        
        Args:
            y: 측정된 위치 (rad)
            u: 제어 입력
               - use_acceleration_input=False: 토크 (Nm) - 기존 방식
               - use_acceleration_input=True: 가속도 (rad/s²) - 새로운 방식
            use_acceleration_input: True이면 u를 가속도로 해석
        
        Returns:
            z3: 외란 가속도 (rad/s²) ⭐
        """
        e = y - self.z1
        
        # 비선형 함수 적용 (각 상태마다 다른 alpha)
        fe1 = self.fal(e, self.alpha, self.delta)
        fe2 = self.fal(e, 0.25, self.delta)
        fe3 = self.fal(e, 0.125, self.delta)
        
        # 가속도 입력 처리
        if use_acceleration_input:
            u_acc = u
        else:
            u_acc = self.b0 * u
        
        # 상태 업데이트
        self.z1 += self.dt * (self.z2 + self.beta1 * fe1)
        self.z2 += self.dt * (self.z3 + self.beta2 * fe2 + u_acc)
        self.z3 += self.dt * (self.beta3 * fe3)
        
        # ✅ Saturation (보수적)
        z3_limit = 15.0 if self.gain_scaling >= 0.8 else 10.0  # rad/s²
        self.z3 = np.clip(self.z3, -z3_limit, z3_limit)
        
        # 통계
        self.max_disturbance = np.maximum(self.max_disturbance, np.abs(self.z3))
        
        return self.z3.copy()
    
    def update_with_acceleration(self, y, qdd):
        """가속도를 직접 입력받는 NESO 업데이트 (권장)
        
        Args:
            y: 측정된 위치 (rad)
            qdd: 명목 가속도 (rad/s²)
            
        Returns:
            z3: 추정된 외란 가속도 (rad/s²)
        """
        return self.update(y, qdd, use_acceleration_input=True)
    
    def update_with_measured_acceleration(self, y, qdd_measured):
        """실제 측정된 가속도를 사용하는 NESO 업데이트 (외란 감지 최적화) ✨
        
        ⭐ 핵심 아이디어:
        외부 힘이 작용하면 → 즉시 가속도로 나타남 → ESO가 바로 감지
        비선형 함수로 더 빠른 수렴 + 노이즈 억제
        
        Args:
            y: 측정된 위치 (rad)
            qdd_measured: 실제 측정된 가속도 (rad/s²) - data.qacc에서 얻음
            
        Returns:
            z3: 추정된 외란 가속도 (rad/s²)
            
        장점 (LinearESO 대비):
            1. 더 빠른 수렴 (fal 함수의 비선형성)
            2. 노이즈 억제 (작은 오차에서 선형 영역)
            3. 외란 감지 반응 속도 향상
            
        사용 예시:
            # MuJoCo에서 실제 가속도 가져오기
            qdd_actual = data.qacc[joint_idx]
            
            # NESO 업데이트 (실제 가속도 직접 사용)
            disturbance_acc = neso.update_with_measured_acceleration(q, qdd_actual)
            
            # 외란 보상 토크 계산
            disturbance_torque = M @ disturbance_acc
            tau = tau_nominal - disturbance_torque
        """
        e = y - self.z1
        
        # 비선형 함수 적용 (각 상태마다 다른 alpha)
        fe1 = self.fal(e, self.alpha, self.delta)
        fe2 = self.fal(e, 0.25, self.delta)
        fe3 = self.fal(e, 0.125, self.delta)
        
        # 상태 업데이트 (실제 가속도 사용)
        self.z1 += self.dt * (self.z2 + self.beta1 * fe1)
        self.z2 += self.dt * (self.z3 + self.beta2 * fe2 + qdd_measured)
        self.z3 += self.dt * (self.beta3 * fe3)
        
        # ✅ Saturation (보수적)
        z3_limit = 15.0 if self.gain_scaling >= 0.8 else 10.0  # rad/s²
        self.z3 = np.clip(self.z3, -z3_limit, z3_limit)
        
        # 통계
        self.max_disturbance = np.maximum(self.max_disturbance, np.abs(self.z3))
        
        return self.z3.copy()
    
    def reset(self, q_init, qd_init=None):
        """상태 초기화"""
        self.z1 = q_init.copy()
        self.z2 = qd_init.copy() if qd_init is not None else np.zeros(self.num_joints)
        self.z3 = np.zeros(self.num_joints)
        self.max_disturbance = np.zeros(self.num_joints)
    
    def get_disturbance_estimate(self):
        """현재 외란 추정값 반환 (rad/s²) ⭐"""
        return self.z3.copy()
    