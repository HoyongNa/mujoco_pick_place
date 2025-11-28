"""토크 제어기 - 순수 PID + DOB + ESO 토크 계산
Holding 기능은 arm_holder.py로 분리
"""

import numpy as np
import mujoco
from config.constants import (
    ARM_Q_IDX,
    ARM_KP_HOLD, ARM_KD_HOLD, ARM_KI_HOLD,
    ARM_TORQUE_LIMIT, ARM_TORQUE_SCALE
)
from .eso import LinearESO, NonlinearESO

class TorqueController:
    """순수 토크 제어기 - 기본적인 토크 계산만 담당
    
    Note:
        Holding 기능(Ruckig + set_current_as_target 등)은 
        controllers/arm/arm_holder.py의 ArmHolder 클래스를 사용
    """
    
    def __init__(self, model, data, joint_idx=None, use_dob=True, use_eso=True, eso_type='linear', eso_omega=20.0, grasp_checker=None, use_velocity_filter=True, velocity_filter_alpha=0.3):
        """
        Args:
            model: MuJoCo model
            data: MuJoCo data
            joint_idx: 제어할 관절 인덱스 (None이면 ARM_Q_IDX 사용)
            use_dob: DOB (Disturbance Observer) 사용 여부
            use_eso: ESO (Extended State Observer) 사용 여부
            eso_type: ESO 타입 ('linear' or 'nonlinear')
            eso_omega: ESO 대역폭 (Hz, 기본값 6.0)
            grasp_checker: GraspChecker 인스턴스 (파지 상태 자동 감지용)
            use_velocity_filter: 속도 신호에 저역통과필터 적용 여부
            velocity_filter_alpha: 필터 계수 (0~1, 작을수록 강한 필터링)
        """
        self.model = model
        self.data = data
        self.joint_idx = joint_idx if joint_idx is not None else ARM_Q_IDX
        self.num_joints = len(self.joint_idx)
        
        # ===== 제어 게인 =====
        self.Kp = np.eye(self.num_joints) * ARM_KP_HOLD
        self.Kd = np.eye(self.num_joints) * ARM_KD_HOLD
        self.Ki = np.eye(self.num_joints) * ARM_KI_HOLD
        
        # ===== 적분 항 =====
        self.integral_error = np.zeros(self.num_joints)
        self.max_integral = 150  # Anti-windup (클램핑)
        
        # ===== ✅ 속도 저역통과 필터 =====
        self.use_velocity_filter = use_velocity_filter
        self.velocity_filter_alpha = velocity_filter_alpha  # 필터 계수 (0~1)
        self.filtered_velocity = np.zeros(self.num_joints)  # 필터링된 속도
        self.velocity_filter_initialized = False
        
        # 필터 파라미터 설명
        if self.use_velocity_filter:
            cutoff_freq = self._alpha_to_cutoff_freq(velocity_filter_alpha)
            # print(f"✅ Velocity Low-Pass Filter 활성화")
        self.grasp_checker = grasp_checker  # GraspChecker 인스턴스
            # print(f"   - Alpha: {velocity_filter_alpha:.3f}")
            # print(f"   - Cutoff frequency: ~{cutoff_freq:.1f} Hz")
            # print(f"   - 속도 신호 노이즈 감소를 위한 필터")
        
        # 파지 상태
        self.is_grasping = False
        
        # ===== DOB (Disturbance Observer) =====
        self.use_dob = use_dob
        self.dob_gain = 5.0
        self.qd_prev = np.zeros(self.num_joints)
        self.filtered_disturbance = np.zeros(self.num_joints)
        
        # ===== ESO (Extended State Observer) =====
        self.use_eso = use_eso
        self.eso = None
        self.max_disturbance_torque = np.zeros(self.num_joints)  # ESO 통계용
        
        if self.use_eso:
            dt = self.model.opt.timestep
            if eso_type == 'linear':
                self.eso = LinearESO(
                    num_joints=self.num_joints,
                    dt=dt,
                    omega_o=eso_omega,
                    gain_scaling=1  
                )
            elif eso_type == 'nonlinear':
                self.eso = NonlinearESO(
                    num_joints=self.num_joints,
                    dt=dt,
                    omega_o=eso_omega,
                    gain_scaling=1,
                    alpha=0.5
                )
            else:
                raise ValueError(f"Unknown ESO type: {eso_type}")
            
            # ESO 초기화
            current_q = np.copy(self.data.qpos[self.joint_idx])
            current_qd = np.copy(self.data.qvel[self.joint_idx])
            self.eso.reset(current_q, current_qd)
    
    def _alpha_to_cutoff_freq(self, alpha):
        """α 계수를 컷오프 주파수로 변환 (근사치)
        
        First-order low-pass filter: alpha = dt/(dt + 1/ωc)
        where ωc = 2πfc (cutoff angular frequency)
        
        Args:
            alpha: 필터 계수 (0~1)
            
        Returns:
            Approximate cutoff frequency in Hz
        """
        dt = self.model.opt.timestep
        if alpha >= 1.0 or alpha <= 0.0:
            return float('inf') if alpha >= 1.0 else 0.0
        
        # alpha = dt/(dt + 1/ωc) => ωc = 1/(dt/alpha - dt)
        omega_c = 1.0 / (dt/alpha - dt)
        return omega_c / (2.0 * np.pi)
    
    def get_current_position(self):
        """현재 팔 관절 위치 반환"""
        return np.copy(self.data.qpos[self.joint_idx])
    
    def compute_torque(self, q_des, qd_des, qdd_des):
        """PID + 피드포워드 + DOB + ESO 토크 계산
        
        Args:
            q_des: 목표 위치
            qd_des: 목표 속도
            qdd_des: 목표 가속도
            
        Returns:
            토크 명령 (scaled)
        """
        q = self.data.qpos[self.joint_idx]
        qd = self.data.qvel[self.joint_idx]
        dt = self.model.opt.timestep
        
        # ✅ 속도 저역통과 필터 적용
        if self.use_velocity_filter:
            # 초기화: 처음 호출 시 현재 속도로 초기화
            if not self.velocity_filter_initialized:
                self.filtered_velocity = np.copy(qd)
                self.velocity_filter_initialized = True
            
            # First-order low-pass filter (Exponential Moving Average)
            # filtered = α * current + (1 - α) * previous
            self.filtered_velocity = (self.velocity_filter_alpha * qd + 
                                     (1.0 - self.velocity_filter_alpha) * self.filtered_velocity)
            
            # 필터링된 속도 사용
            qd_for_control = self.filtered_velocity
        else:
            # 필터 사용 안 함: 원본 속도 사용
            qd_for_control = qd
        
        # 오차 계산 (필터링된 속도 사용)
        pos_err = q_des - q
        vel_err = qd_des - qd_for_control
        
        # 적분 항 업데이트 (Anti-windup 클램핑)
        self.integral_error += pos_err * dt
        self.integral_error = np.clip(
            self.integral_error, -self.max_integral, self.max_integral
        )
        
        # 동역학 계산
        M_full = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, M_full, self.data.qM)
        M = M_full[np.ix_(self.joint_idx, self.joint_idx)]
        
        # Coriolis + Gravity
        mujoco.mj_rnePostConstraint(self.model, self.data)
        bias = self.data.qfrc_bias[self.joint_idx]
        
        # ========== 외란 보상 선택 ==========
        
        # 모드 1: ESO만 사용 (추천)
        if self.use_eso and not self.use_dob:
            # 1. 명목 가속도 계산 (제어 법칙: PID)
            qdd_nominal = qdd_des + self.Kp @ pos_err + self.Kd @ vel_err + self.Ki @ self.integral_error
            
            # 2. ESO 업데이트 (외란 가속도 추정)
            disturbance_acc = self.eso.update_with_acceleration(q, qdd_nominal)
            
            # 3. 명목 토크 계산
            torque_nominal = M @ qdd_nominal + bias
            
            # 4. 외란 토크 변환 및 보상
            disturbance_torque = M @ disturbance_acc
            torque = torque_nominal - disturbance_torque
            
            # 5. 통계 업데이트
            self.max_disturbance_torque = np.maximum(
                self.max_disturbance_torque,
                np.abs(disturbance_torque)
            )
        
        # 모드 2: DOB만 사용
        elif self.use_dob and not self.use_eso:
            # 기본 토크 계산 (Computed Torque Control)
            torque_nominal = M @ (
                qdd_des + 
                self.Kp @ pos_err + 
                self.Kd @ vel_err + 
                self.Ki @ self.integral_error
            ) + bias
            torque = self._apply_dob(torque_nominal, qd, M, bias, dt)
        
        # 모드 3: ESO + DOB 동시 사용 (실험적)
        elif self.use_eso and self.use_dob:
            # 경고: 중복 보상 가능성
                        
            # 1. 명목 가속도 계산
            qdd_nominal = qdd_des + self.Kp @ pos_err + self.Kd @ vel_err + self.Ki @ self.integral_error
            
            # 2. ESO 업데이트
            disturbance_acc = self.eso.update_with_acceleration(q, qdd_nominal)
            
            # 3. 명목 토크 계산
            torque_nominal = M @ qdd_nominal + bias
            
            # 4. ESO 보상
            disturbance_torque_eso = M @ disturbance_acc
            torque_after_eso = torque_nominal - disturbance_torque_eso
            
            # 5. DOB 보상 (잔여 외란)
            torque = self._apply_dob(torque_after_eso, qd, M, bias, dt)
            
            # 통계
            self.max_disturbance_torque = np.maximum(
                self.max_disturbance_torque,
                np.abs(disturbance_torque_eso)
            )
            
        else:
            # 기본 토크 계산 (Computed Torque Control)
            torque = M @ (
                qdd_des + 
                self.Kp @ pos_err + 
                self.Kd @ vel_err + 
                self.Ki @ self.integral_error
            ) + bias
        
        # 토크 제한 및 스케일링
        torque = np.clip(torque, -ARM_TORQUE_LIMIT, ARM_TORQUE_LIMIT)
        return torque * ARM_TORQUE_SCALE
    
    def _apply_dob(self, torque_nominal, qd, M, bias, dt):
        """외란 관측기 (Disturbance Observer) 적용"""
        # 가속도 추정 (차분)
        qdd_est = (qd - self.qd_prev) / dt
        self.qd_prev = np.copy(qd)
        
        # 외란 추정
        disturbance = torque_nominal - (M @ qdd_est + bias)
        
        # Low-pass filter
        self.filtered_disturbance += self.dob_gain * dt * (
            disturbance - self.filtered_disturbance
        )
        
        # 외란 보상
        return torque_nominal + self.filtered_disturbance
    
    def reset_integral(self):
        """적분 항 리셋"""
        self.integral_error = np.zeros(self.num_joints)
    
    def reset_dob(self):
        """DOB 상태 리셋"""
        self.qd_prev = np.zeros(self.num_joints)
        self.filtered_disturbance = np.zeros(self.num_joints)
    
    def reset_eso(self):
        """ESO 상태 리셋"""
        if self.eso is not None:
            current_q = np.copy(self.data.qpos[self.joint_idx])
            current_qd = np.copy(self.data.qvel[self.joint_idx])
            self.eso.reset(current_q, current_qd)
            self.max_disturbance_torque = np.zeros(self.num_joints)
            print("✅ ESO 상태 리셋")
    
    def reset_velocity_filter(self):
        """✅ 속도 필터 상태 리셋"""
        self.filtered_velocity = np.zeros(self.num_joints)
        self.velocity_filter_initialized = False
        if self.use_velocity_filter:
            print("✅ Velocity filter 상태 리셋")
    
    def reset_all(self):
        """모든 상태 리셋"""
        self.reset_integral()
        self.reset_dob()
        self.reset_eso()
        self.reset_velocity_filter()
        print("✅ 모든 제어기 상태 리셋 완료")
    
    def get_eso_statistics(self):
        """ESO 통계 반환 (디버깅용)
        
        Returns:
            dict: ESO 통계 정보
                - current_disturbance: 현재 외란 추정값 (rad/s²)
                - max_disturbance: 최대 외란 가속도 (rad/s²)
                - disturbance_norm: 외란 크기 (L2 norm)
                - max_disturbance_torque: 최대 외란 토크 (Nm)
        """
        if self.eso is not None:
            stats = self.eso.get_statistics()
            stats['max_disturbance_torque'] = self.max_disturbance_torque.copy()
            return stats
        return None
    
    def get_eso_disturbance(self):
        """현재 ESO 외란 추정값 반환 (rad/s²)"""
        if self.eso is not None:
            return self.eso.get_disturbance_estimate()
        return np.zeros(self.num_joints)
    
    def set_gains(self, kp=None, kd=None, ki=None):
        """게인 동적 변경"""
        if kp is not None:
            self.Kp = np.eye(self.num_joints) * kp
        if kd is not None:
            self.Kd = np.eye(self.num_joints) * kd
        if ki is not None:
            self.Ki = np.eye(self.num_joints) * ki
    
    def set_velocity_filter_params(self, alpha=None, enable=None):
        """✅ 속도 필터 파라미터 동적 조정
        
        Args:
            alpha: 필터 계수 (0~1, 작을수록 강한 필터링)
            enable: 필터 활성화 여부
        """
        if enable is not None:
            self.use_velocity_filter = enable
            if enable:
                print("✅ Velocity filter 활성화")
            else:
                print("❌ Velocity filter 비활성화")
                self.reset_velocity_filter()
        
        if alpha is not None and 0.0 < alpha <= 1.0:
            self.velocity_filter_alpha = alpha
            cutoff_freq = self._alpha_to_cutoff_freq(alpha)
            print(f"✅ Velocity filter alpha = {alpha:.3f} (cutoff ~{cutoff_freq:.1f} Hz)")
    
    def get_velocity_filter_status(self):
        """✅ 속도 필터 상태 반환 (디버깅용)
        
        Returns:
            dict: 필터 상태 정보
                - enabled: 필터 활성화 여부
                - alpha: 현재 필터 계수
                - cutoff_freq: 컷오프 주파수 (Hz)
                - current_velocity: 현재 속도
                - filtered_velocity: 필터링된 속도
                - filter_lag: 필터 지연 (차이)
        """
        qd = self.data.qvel[self.joint_idx]
        
        status = {
            'enabled': self.use_velocity_filter,
            'alpha': self.velocity_filter_alpha,
            'cutoff_freq': self._alpha_to_cutoff_freq(self.velocity_filter_alpha) if self.use_velocity_filter else None,
            'current_velocity': qd.copy(),
            'filtered_velocity': self.filtered_velocity.copy() if self.use_velocity_filter else qd.copy(),
            'filter_lag': np.linalg.norm(qd - self.filtered_velocity) if self.use_velocity_filter else 0.0,
            'initialized': self.velocity_filter_initialized
        }
        
        return status
    