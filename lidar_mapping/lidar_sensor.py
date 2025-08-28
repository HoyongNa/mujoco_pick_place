"""
Lidar Sensor Handler (MuJoCo rangefinder)

- 빔 축: 사이트의 ±Z (기본 -Z). BEAM_SIGN 으로 토글.
- 히트 좌표: 3D 단위벡터의 XY 성분 * distance  (XY를 따로 정규화하지 않음)
- no-hit(히트 없음): 센서 cutoff까지를 free 전용 레이로만 사용(끝점 occupied 금지)

강건화:
- 가능한 경우 model.sensor_cutoff[sid] 사용
- 미노출/0/NaN이면 관측 거리로 cutoff를 자동 추정(노히트시 거리=실제 cutoff)
- 필요 시 set_range_limit() 으로 수동 오버라이드
"""

from typing import Dict, List, Optional
import mujoco
import numpy as np

# 장면에 따라 +Z/-Z 중 하나가 레이 방향입니다.
BEAM_SIGN = 1.0  # 반대로 보이면 -1.0 -> +1.0 로 바꾸세요.

# XML에서 설정한 기본 cutoff 값
DEFAULT_CUTOFF = 6.0


class LidarSensor:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

        # 실내 기본값
        self.min_range: float = 0.25
        self.nohit_margin: float = 0.02  # cutoff 근처 값은 no-hit 처리

        # rangefinder 목록 + 센서별 cutoff 수집
        self.sensors: List[Dict[str, int]] = []
        self._obs_max: List[float] = []     # 관측된 최대 거리(자동 추정용)
        self._override: Optional[float] = None  # 수동 오버라이드 (전 센서 공통)

        for sid in range(self.model.nsensor):
            if self.model.sensor_type[sid] == mujoco.mjtSensor.mjSENS_RANGEFINDER:
                site_id = int(self.model.sensor_objid[sid])
                adr = int(self.model.sensor_adr[sid])
                dim = int(self.model.sensor_dim[sid])

                cutoff = None
                # MuJoCo가 제공하는 센서별 cutoff 확인
                if hasattr(self.model, "sensor_cutoff"):
                    try:
                        v = float(self.model.sensor_cutoff[sid])
                        if np.isfinite(v) and v > 0:
                            cutoff = v
                    except:
                        pass
                
                # cutoff를 못 읽었으면 DEFAULT_CUTOFF 사용
                if cutoff is None or cutoff <= 0:
                    cutoff = DEFAULT_CUTOFF

                self.sensors.append({
                    "sensor_id": sid, 
                    "site_id": site_id, 
                    "adr": adr, 
                    "dim": dim, 
                    "cutoff": cutoff
                })
                self._obs_max.append(0.0)

        # 전역 최대 사거리(표시/통계용)
        self.global_max_range: float = self._compute_global_max()

    def _compute_global_max(self) -> float:
        """전역 최대 사거리 계산"""
        if self._override is not None:
            return float(self._override)
        
        vals = []
        for i, s in enumerate(self.sensors):
            if s["cutoff"] is not None:
                vals.append(float(s["cutoff"]))
            elif self._obs_max[i] > 0:
                vals.append(float(self._obs_max[i]))
        
        return float(min(vals)) if vals else DEFAULT_CUTOFF

    def set_range_limit(self, max_range: float):
        """수동 오버라이드(전 센서 공통). 예: set_range_limit(6.0)"""
        self._override = float(max_range)
        self.global_max_range = float(max_range)

    def clear_override(self):
        """오버라이드 해제"""
        self._override = None
        self.global_max_range = self._compute_global_max()

    def _read_distance(self, info: Dict[str, int]) -> float:
        """센서 거리 읽기"""
        adr, dim = info["adr"], info["dim"]
        if dim <= 0:
            return np.nan
        return float(self.data.sensordata[adr : adr + dim][0])

    def get_scan(self):
        """
        Returns dict with:
          points       : (N,2)  ← hit endpoints (occupied 후보)
          origins      : (N,2)
          free_points  : (M,2)  ← no-hit endpoints (free 전용; 끝점 occupied 금지)
          free_origins : (M,2)
          angles       : (K,)
          ranges       : (K,)   ← 각 빔의 측정거리(히트면 해당 거리, 노히트면 cutoff/추정치)
          valid_mask   : (K,)   ← 히트 여부
          num_beams, num_valid, num_nohit, min_range, max_range
        """
        points, origins = [], []
        free_points, free_origins = [], []
        angles, ranges, valids = [], [], []

        for idx, info in enumerate(self.sensors):
            site_id = info["site_id"]

            # 이 센서의 유효 cutoff 결정
            if self._override is not None:
                cutoff_eff = float(self._override)
            elif info["cutoff"] is not None:
                cutoff_eff = float(info["cutoff"])
            elif self._obs_max[idx] > 0:
                cutoff_eff = float(self._obs_max[idx])
            else:
                cutoff_eff = DEFAULT_CUTOFF

            cutoff_eff = max(self.min_range + 1e-6, cutoff_eff)

            # 센서 월드 위치/자세
            pos3 = self.data.site_xpos[site_id]
            mat = self.data.site_xmat[site_id].reshape(3, 3)

            # 빔 축: 사이트의 ±Z
            dir3 = mat[:, 2] * BEAM_SIGN
            n3 = np.linalg.norm(dir3)
            if n3 < 1e-9:
                ranges.append(np.nan)
                angles.append(0.0)
                valids.append(False)
                continue
            
            dir3 = dir3 / n3
            xy = dir3[:2]
            
            # XY가 거의 0이면 (수직 빔) 스킵
            if np.linalg.norm(xy) < 1e-6:
                ranges.append(np.nan)
                angles.append(0.0)
                valids.append(False)
                continue

            theta = float(np.arctan2(xy[1], xy[0]))
            origin2 = pos3[:2].copy()
            dist = self._read_distance(info)

            # 관측 최대치 갱신(자동 추정용)
            if np.isfinite(dist) and dist > self._obs_max[idx]:
                self._obs_max[idx] = float(dist)

            # 히트/노히트 판정: cutoff 바로 근처는 no-hit
            nohit_thr = cutoff_eff - self.nohit_margin
            if (np.isfinite(dist) and dist >= self.min_range and dist < nohit_thr):
                # hit
                hit_xy = origin2 + xy * dist
                points.append([hit_xy[0], hit_xy[1]])
                origins.append([origin2[0], origin2[1]])
                ranges.append(dist)
                angles.append(theta)
                valids.append(True)
            else:
                # no hit → free만 누적, 끝점은 occupied 금지
                far = max(self.min_range, 
                         min(cutoff_eff, dist if np.isfinite(dist) else cutoff_eff) - self.nohit_margin)
                far_xy = origin2 + xy * far
                free_points.append([far_xy[0], far_xy[1]])
                free_origins.append([origin2[0], origin2[1]])
                ranges.append(cutoff_eff if not np.isfinite(dist) else dist)
                angles.append(theta)
                valids.append(False)

        def arr(x, shape): 
            return np.asarray(x, dtype=float) if x else np.empty(shape)
        
        return {
            "points":       arr(points, (0, 2)),
            "origins":      arr(origins, (0, 2)),
            "free_points":  arr(free_points, (0, 2)),
            "free_origins": arr(free_origins, (0, 2)),
            "angles":       np.asarray(angles, dtype=float) if angles else np.empty((0,)),
            "ranges":       np.asarray(ranges, dtype=float) if ranges else np.empty((0,)),
            "valid_mask":   np.asarray(valids, dtype=bool) if valids else np.empty((0,), bool),
            "num_beams":    int(len(valids)),
            "num_valid":    int(np.count_nonzero(valids) if valids else 0),
            "num_nohit":    int(len(free_points)),
            "min_range":    self.min_range,
            "max_range":    self.global_max_range,
        }

    def get_scan_statistics(self, scan: Optional[dict] = None) -> dict:
        """스캔 통계 반환"""
        if scan is None:
            scan = self.get_scan()
        ranges, valid = scan["ranges"], scan["valid_mask"]
        valid_ranges = ranges[valid] if ranges.size and valid.size else np.array([])
        
        return {
            "num_beams": int(scan["num_beams"]),
            "num_valid": int(scan["num_valid"]),
            "num_nohit": int(scan["num_nohit"]),
            "min_range": float(np.min(valid_ranges)) if valid_ranges.size else 0.0,
            "max_range": float(np.max(valid_ranges)) if valid_ranges.size else 0.0,
            "mean_range": float(np.mean(valid_ranges)) if valid_ranges.size else 0.0,
        }
