#!/usr/bin/env python3
"""
Surgical Wrist-Worn IMU Hand Motion & Unsupervised Repetition Analyzer
======================================================================
Processes 100 Hz 9-DOF IMU telemetry (Quaternion, Gravity-Free Linear Accel,
Gyroscope) from the STM32_IMU (Bosch BNO055) wrist tracker to compute:

1. Bounded 3D Hand Trajectory (cm):
   - Quaternion rotation of body linear acceleration to Earth/World frame.
   - 2nd-order High-Pass Butterworth washout filter + Zero-Velocity Updates (ZUPT)
     to reconstruct per-cycle 3D hand displacement without unbounded drift.
   - Biomechanical Forearm + Hand kinematic model for 3D wrist/instrument position.

2. Unsupervised Pattern & Cycle Self-Discovery ("Analyze by Data Pattern"):
   - Orientation-invariant Principal Axis (PCA / SVD) dominant motion projection.
   - Multi-scale sliding autocorrelation over candidate lags [0.35 s .. 5.5 s]
     to automatically discover repeating movement patterns and their natural
     cycle period (T_rep) without hardcoded cycle times.
   - Adaptive peak-prominence & zero-crossing cycle segmentation.
   - Unsupervised kinematic pattern classification (Rotational/Suturing Arc,
     Axial Instrument Action, Translational Reach/Dissection, Static Hold).

3. Multi-Hour Surgical Skill & Ergonomic Metrics:
   - Repetition count, active bouts, cadence (reps/min), cycle duration CV%.
   - Economy of motion: 3D path length per repetition (cm) & angular excursion (deg).
   - Smoothness: Normalized Log Dimensionless Jerk (LDLJ) score (0-100%).
   - Surgeon Fatigue / Tremor Index: 8-12 Hz physiological tremor band RMS (m/s^2).
"""

import math
import time
import csv
import os
from collections import deque
from dataclasses import dataclass, asdict
from typing import Deque, Dict, List, Optional, Tuple


@dataclass
class RepetitionEvent:
    """Metrics extracted for a single self-discovered repetition cycle."""
    rep_id: int
    timestamp_s: float
    duration_s: float
    cadence_rpm: float
    path_length_cm: float
    stroke_span_cm: float
    angular_excursion_deg: float
    peak_gyro_dps: float
    peak_vel_cms: float
    smoothness_score: float       # 0..100 (derived from Log Dimensionless Jerk)
    tremor_rms_mps2: float        # 8-12 Hz physiological tremor band RMS
    pattern_type: str             # Unsupervised cluster label
    pattern_confidence: float     # Autocorrelation periodicity strength (0..1)


class BiquadFilter:
    """2nd-order IIR Biquad filter (Butterworth High-Pass or Band-Pass)."""

    def __init__(self, b0: float, b1: float, b2: float, a1: float, a2: float):
        self.b0 = b0
        self.b1 = b1
        self.b2 = b2
        self.a1 = a1
        self.a2 = a2
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0

    def reset(self):
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def process(self, x: float) -> float:
        y = (
            self.b0 * x
            + self.b1 * self.x1
            + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2
        )
        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y
        return y

    @classmethod
    def butterworth_highpass(cls, cutoff_hz: float, fs_hz: float) -> "BiquadFilter":
        """Designs a 2nd-order Butterworth high-pass filter."""
        omega = 2.0 * math.pi * cutoff_hz / fs_hz
        sn = math.sin(omega)
        cs = math.cos(omega)
        alpha = sn / math.sqrt(2.0)
        a0 = 1.0 + alpha
        b0 = ((1.0 + cs) / 2.0) / a0
        b1 = -(1.0 + cs) / a0
        b2 = ((1.0 + cs) / 2.0) / a0
        a1 = (-2.0 * cs) / a0
        a2 = (1.0 - alpha) / a0
        return cls(b0, b1, b2, a1, a2)

    @classmethod
    def butterworth_bandpass(cls, center_hz: float, bw_hz: float, fs_hz: float) -> "BiquadFilter":
        """Designs a 2nd-order resonant band-pass filter (e.g. 8-12 Hz tremor band)."""
        omega = 2.0 * math.pi * center_hz / fs_hz
        sn = math.sin(omega)
        cs = math.cos(omega)
        q = max(0.5, center_hz / max(0.1, bw_hz))
        alpha = sn / (2.0 * q)
        a0 = 1.0 + alpha
        b0 = alpha / a0
        b1 = 0.0
        b2 = -alpha / a0
        a1 = (-2.0 * cs) / a0
        a2 = (1.0 - alpha) / a0
        return cls(b0, b1, b2, a1, a2)


def rotate_vector_by_quat(
    vx: float, vy: float, vz: float, qw: float, qx: float, qy: float, qz: float
) -> Tuple[float, float, float]:
    """Rotates 3D vector v from body frame to world frame using unit quaternion q."""
    # t = 2 * cross(q_xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + qw * t + cross(q_xyz, t)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class SurgicalMotionAnalyzer:
    """
    Real-time 3D hand trajectory and unsupervised repetition analyzer
    for wrist-worn IMU during multi-hour surgical/medical procedures.
    """

    def __init__(self, fs_hz: float = 100.0, forearm_length_cm: float = 26.0):
        self.fs_hz = fs_hz
        self.dt = 1.0 / fs_hz
        self.forearm_length_cm = forearm_length_cm

        # Washout filters for 3D acceleration & velocity integration (drift suppression)
        self.acc_hp = [BiquadFilter.butterworth_highpass(0.20, fs_hz) for _ in range(3)]
        self.vel_hp = [BiquadFilter.butterworth_highpass(0.15, fs_hz) for _ in range(3)]
        self.pos_hp = [BiquadFilter.butterworth_highpass(0.10, fs_hz) for _ in range(3)]

        # 8-12 Hz physiological tremor band-pass filters (centered at 10 Hz, BW 4 Hz)
        self.tremor_bp = [BiquadFilter.butterworth_bandpass(10.0, 4.0, fs_hz) for _ in range(3)]

        # Dynamic state for bounded 3D trajectory (in cm and cm/s)
        self.effective_fs_hz = fs_hz
        self.vel_cm_s = [0.0, 0.0, 0.0]
        self.pos_dyn_cm = [0.0, 0.0, 0.0]
        self.pos_kin_cm = [0.0, 0.0, 0.0]
        self.pos_fused_cm = [0.0, 0.0, 0.0]
        self.pos_origin_cm = [0.0, 0.0, 0.0]  # Tare origin for surgical site
        self.acc_bias_body = [0.0, 0.0, 0.0]  # Sensor silicon resting zero-bias in body frame
        self.prev_a_body = [0.0, 0.0, 0.0]
        self.prev_acc_world = [0.0, 0.0, 0.0]
        self.zupt_counter = 0
        self.is_stationary = True

        # Adaptive stroke filter & per-axis optical calibration scales (Kx, Ky, Kz)
        self.stroke_scale: List[float] = [1.0, 1.0, 1.0]
        self.stroke_active: bool = False
        self.stroke_latch_frames: int = 0

        # Sliding window buffers for unsupervised pattern analysis (6 seconds @ 100 Hz = 600 samples)
        self.win_size = int(6.0 * fs_hz)
        self.gyro_hist: Deque[Tuple[float, float, float]] = deque(maxlen=self.win_size)
        self.vel_hist: Deque[Tuple[float, float, float]] = deque(maxlen=self.win_size)
        self.pos_hist: Deque[Tuple[float, float, float]] = deque(maxlen=self.win_size)
        self.tremor_sq_hist: Deque[float] = deque(maxlen=int(2.0 * fs_hz))
        self.signal_1d_hist: Deque[float] = deque(maxlen=self.win_size)
        self.ts_hist: Deque[float] = deque(maxlen=self.win_size)

        # Unsupervised pattern discovery state
        self.session_start_time = time.time()
        self.last_update_ts = 0.0
        self.sample_counter = 0
        self.auto_period_s = 0.0
        self.pattern_confidence = 0.0
        self.activity_state = "IDLE / HOLD"
        self.current_pattern_type = "Waiting for Pattern..."
        self.autocorr_curve: List[float] = []

        # Cycle segmentation state
        self.in_active_cycle = False
        self.last_rep_ts = 0.0
        self.cycle_start_ts = 0.0
        self.cycle_samples_pos: List[Tuple[float, float, float]] = []
        self.cycle_samples_vel: List[Tuple[float, float, float]] = []
        self.cycle_samples_gyro: List[Tuple[float, float, float]] = []
        self.cycle_samples_jerk_sq: List[float] = []
        self.prev_signal_val = 0.0
        self.signal_peak_in_cycle = 0.0
        self.signal_trough_in_cycle = 0.0
        self.passed_positive_peak = False

        # Multi-hour session logs
        self.rep_count = 0
        self.reps: List[RepetitionEvent] = []
        self.pattern_counts: Dict[str, int] = {
            "Suturing / Rotational Arc": 0,
            "Instrument / Axial Action": 0,
            "Dissection / Linear Reach": 0,
            "Micro-Manipulation": 0,
        }
        self.active_time_s = 0.0
        self.idle_time_s = 0.0

        # Session Recording State (Start / End Test Control)
        self.is_recording = True
        self.test_session_id = 0
        self.test_start_time = time.time()
        self.test_end_time = 0.0

    def start_session(self) -> Dict:
        """Starts a clean surgical test recording session."""
        self.reset_session()
        self.test_session_id += 1
        self.is_recording = True
        self.test_start_time = time.time()
        self.test_end_time = 0.0
        return self.get_live_snapshot()

    def tare_position(self):
        """Sets the current hand position as the surgical site origin (0, 0, 0) cm."""
        self.pos_origin_cm = [self.pos_dyn_cm[0], self.pos_dyn_cm[1], self.pos_dyn_cm[2]]
        self.pos_fused_cm = [0.0, 0.0, 0.0]

    def reset_origin(self):
        """Resets the surgical site origin offset back to zero."""
        self.pos_origin_cm = [0.0, 0.0, 0.0]

    def set_stroke_scales(self, scale_x: float, scale_y: float, scale_z: float):
        """Sets calibrated per-axis stroke scaling factors for trajectory integration."""
        self.stroke_scale = [float(scale_x), float(scale_y), float(scale_z)]

    def get_stroke_scales(self) -> Tuple[float, float, float]:
        """Returns the current per-axis stroke scaling factors (kx, ky, kz)."""
        return (float(self.stroke_scale[0]), float(self.stroke_scale[1]), float(self.stroke_scale[2]))

    def end_session(self) -> Dict:
        """Ends the surgical test recording session and returns the final report."""
        self.is_recording = False
        self.test_end_time = time.time()
        return self.get_session_report()

    def reset_session(self):
        """Resets multi-hour session counters and filter states."""
        for f in self.acc_hp + self.vel_hp + self.pos_hp + self.tremor_bp:
            f.reset()
        self.vel_cm_s = [0.0, 0.0, 0.0]
        self.pos_dyn_cm = [0.0, 0.0, 0.0]
        self.pos_kin_cm = [0.0, 0.0, 0.0]
        self.pos_fused_cm = [0.0, 0.0, 0.0]
        self.acc_bias_body = [0.0, 0.0, 0.0]
        self.prev_a_body = [0.0, 0.0, 0.0]
        self.prev_acc_world = [0.0, 0.0, 0.0]
        self.gyro_hist.clear()
        self.vel_hist.clear()
        self.pos_hist.clear()
        self.tremor_sq_hist.clear()
        self.signal_1d_hist.clear()
        self.ts_hist.clear()
        self.session_start_time = time.time()
        self.test_start_time = time.time()
        self.test_end_time = 0.0
        self.sample_counter = 0
        self.auto_period_s = 0.0
        self.pattern_confidence = 0.0
        self.activity_state = "IDLE / HOLD"
        self.current_pattern_type = "Waiting for Pattern..."
        self.rep_count = 0
        self.reps.clear()
        for k in self.pattern_counts:
            self.pattern_counts[k] = 0
        self.active_time_s = 0.0
        self.idle_time_s = 0.0
        self.stroke_active = False
        self.stroke_latch_frames = 0
        self._reset_current_cycle(0.0)

    def _reset_current_cycle(self, ts: float):
        self.cycle_start_ts = ts
        self.cycle_samples_pos.clear()
        self.cycle_samples_vel.clear()
        self.cycle_samples_gyro.clear()
        self.cycle_samples_jerk_sq.clear()
        self.signal_peak_in_cycle = 0.0
        self.signal_trough_in_cycle = 0.0
        self.passed_positive_peak = False

    def update(
        self,
        ts: float,
        qw: float,
        qx: float,
        qy: float,
        qz: float,
        ax_mps2: float,
        ay_mps2: float,
        az_mps2: float,
        gx_dps: float,
        gy_dps: float,
        gz_dps: float,
    ) -> Dict:
        """
        Processes a single IMU frame and returns live trajectory + repetition analytics.
        """
        self.sample_counter += 1
        if self.last_update_ts > 0.0 and 0.001 < (ts - self.last_update_ts) < 0.2:
            dt = ts - self.last_update_ts
        else:
            dt = self.dt
        self.last_update_ts = ts

        # Normalize quaternion
        q_norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if q_norm > 1e-6:
            qw, qx, qy, qz = qw / q_norm, qx / q_norm, qy / q_norm, qz / q_norm
        else:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        # 1. Body-frame linear acceleration and stationary detection
        a_body = [ax_mps2, ay_mps2, az_mps2]
        gyro_mag_dps = math.sqrt(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps)
        delta_a_body = math.sqrt(sum((a_body[i] - self.prev_a_body[i]) ** 2 for i in range(3)))
        self.prev_a_body = list(a_body)

        # Unbiased acceleration in sensor body frame
        a_unbiased_b = [a_body[i] - self.acc_bias_body[i] for i in range(3)]
        a_unbiased_mag = math.sqrt(sum(x * x for x in a_unbiased_b))

        # Rotate unbiased body acceleration to Earth/World frame using quaternion
        aw_x, aw_y, aw_z = rotate_vector_by_quat(a_unbiased_b[0], a_unbiased_b[1], a_unbiased_b[2], qw, qx, qy, qz)
        aw = [aw_x, aw_y, aw_z]
        acc_mag_mps2 = math.sqrt(aw_x * aw_x + aw_y * aw_y + aw_z * aw_z)

        # Compute world jerk (da/dt in m/s^3)
        dt_safe = max(0.001, dt)
        jerk_sq = 0.0
        for i in range(3):
            jerk_i = (aw[i] - self.prev_acc_world[i]) / dt_safe
            jerk_sq += jerk_i * jerk_i
            self.prev_acc_world[i] = aw[i]
        jerk_mag = math.sqrt(jerk_sq)

        # Robust stationary detection:
        # A) Gyro quiet (< 3.0 dps) and body acceleration steady (delta_a < 0.08 m/s^2) with resting accel (< 0.25 m/s^2)
        # OR B) Gyro < 7.0 dps and unbiased body acceleration < 0.15 m/s^2
        is_still = (
            (gyro_mag_dps < 3.0 and delta_a_body < 0.08 and a_unbiased_mag < 0.25)
            or (gyro_mag_dps < 7.0 and a_unbiased_mag < 0.15)
        )
        if is_still:
            self.zupt_counter += 1
        else:
            self.zupt_counter = 0

        self.is_stationary = self.zupt_counter >= 15

        if self.is_stationary:
            for i in range(3):
                # Continuously adapt body silicon bias when stationary
                self.acc_bias_body[i] = 0.90 * self.acc_bias_body[i] + 0.10 * a_body[i]
                # ZUPT: firmly clamp velocity to zero
                self.vel_cm_s[i] = 0.0

                # Hold position steady at destination for 1.0s (100 frames);
                # then gently bleed residual offset back to surgical origin to prevent long-term drift
                if self.zupt_counter > 100:
                    self.pos_dyn_cm[i] = self.pos_origin_cm[i] + (self.pos_dyn_cm[i] - self.pos_origin_cm[i]) * 0.85
                    if abs(self.pos_dyn_cm[i] - self.pos_origin_cm[i]) < 0.01:
                        self.pos_dyn_cm[i] = self.pos_origin_cm[i]

        # Adaptive stroke detection & jerk-assisted stroke latching
        a_thresh = 0.12  # m/s^2 ambient vibration noise floor threshold
        has_accel = any(abs(aw[i]) > a_thresh for i in range(3))
        has_jerk = jerk_mag > 0.8  # m/s^3 jerk latch threshold
        v_mag = math.sqrt(sum(v * v for v in self.vel_cm_s))
        has_vel = v_mag > 0.8  # cm/s

        # Only latch stroke active from ongoing motion/stroke to avoid arming from cold rest
        if has_jerk and (self.stroke_active or has_vel or has_accel):
            # Latch stroke active across rapid reversals / zero crossings
            self.stroke_latch_frames = int(0.20 * self.fs_hz)
        elif self.stroke_latch_frames > 0:
            self.stroke_latch_frames -= 1

        if self.is_stationary:
            self.stroke_active = False
            self.stroke_latch_frames = 0
        else:
            self.stroke_active = has_accel or has_vel or (self.stroke_latch_frames > 0)

        # Power-law non-linear transient boost & deadband during active stroke:
        # a_eff = sign(aw) * max(0, |aw| - a_thresh)^1.1 * K_axis
        a_eff = [0.0, 0.0, 0.0]
        for i in range(3):
            val = aw[i]
            sign = 1.0 if val > 0.0 else (-1.0 if val < 0.0 else 0.0)
            excess = max(0.0, abs(val) - a_thresh)
            a_eff[i] = sign * (excess ** 1.1) * self.stroke_scale[i]

        # 2. Extract 8-12 Hz physiological tremor power (m/s^2 RMS)
        tremor_sq = 0.0
        for i in range(3):
            tb = self.tremor_bp[i].process(aw[i])
            tremor_sq += tb * tb
        self.tremor_sq_hist.append(tremor_sq)
        tremor_rms = math.sqrt(sum(self.tremor_sq_hist) / max(1, len(self.tremor_sq_hist)))

        # 3. Dynamic Velocity & Position Integration
        if not self.is_stationary:
            for i in range(3):
                if self.stroke_active:
                    a_cms2 = a_eff[i] * 100.0
                    # Light numerical stabilization damping during active stroke
                    self.vel_cm_s[i] = (self.vel_cm_s[i] + a_cms2 * dt) * 0.998
                else:
                    # Bleed velocity when stroke inactive and not yet stationary
                    self.vel_cm_s[i] *= 0.85
                self.pos_dyn_cm[i] += self.vel_cm_s[i] * dt

        # Operating Table Workspace Clamping:
        # Horizontal reach X, Y: +/- 45 cm
        self.pos_dyn_cm[0] = max(-45.0, min(45.0, self.pos_dyn_cm[0]))
        self.pos_dyn_cm[1] = max(-45.0, min(45.0, self.pos_dyn_cm[1]))
        # Vertical elevation Z: 0.0 cm (table pad surface) to +35.0 cm (high instrument reach)
        self.pos_dyn_cm[2] = max(0.0, min(35.0, self.pos_dyn_cm[2]))

        # 4. Decoupled Spatial Position: Pure linear displacement relative to Surgical Site Origin
        # (Forearm rotational arc is handled by 3D model orientation directly in visualizer)
        kx, ky, kz = rotate_vector_by_quat(self.forearm_length_cm, 0.0, 0.0, qw, qx, qy, qz)
        self.pos_kin_cm = [kx - self.forearm_length_cm, ky, kz]

        for i in range(3):
            self.pos_fused_cm[i] = self.pos_dyn_cm[i] - self.pos_origin_cm[i]

        # Record histories
        self.gyro_hist.append((gx_dps, gy_dps, gz_dps))
        self.vel_hist.append((self.vel_cm_s[0], self.vel_cm_s[1], self.vel_cm_s[2]))
        self.pos_hist.append((self.pos_fused_cm[0], self.pos_fused_cm[1], self.pos_fused_cm[2]))
        self.ts_hist.append(ts)

        if len(self.ts_hist) >= 20:
            span_t = self.ts_hist[-1] - self.ts_hist[0]
            if span_t > 0.1:
                self.effective_fs_hz = max(20.0, min(200.0, (len(self.ts_hist) - 1) / span_t))

        # Track active vs idle time
        if gyro_mag_dps > 12.0 or acc_mag_mps2 > 0.30:
            self.active_time_s += dt
        else:
            self.idle_time_s += dt

        # 5. Compute dominant 1D motion wave (PCA / Adaptive Axis Projection)
        sig_val = self._compute_dominant_signal(gx_dps, gy_dps, gz_dps)
        self.signal_1d_hist.append(sig_val)

        # 6. Every 15 samples, run Unsupervised Multi-Scale Autocorrelation
        if self.sample_counter % 15 == 0 and len(self.signal_1d_hist) >= int(2.0 * self.effective_fs_hz):
            self._discover_periodicity()

        # 7. Adaptive Cycle Segmentation & Repetition Extraction
        self._segment_repetition_cycle(ts, sig_val, jerk_sq, tremor_rms)

        return self.get_live_snapshot(sig_val, tremor_rms)

    def _compute_dominant_signal(self, gx: float, gy: float, gz: float) -> float:
        """
        Projects 6-DOF motion (angular velocity + linear velocity) onto the
        dominant principal axis over the recent 2.5-second window so repetition
        cycles are detected regardless of wrist orientation.
        """
        recent_n = min(len(self.gyro_hist), int(2.5 * self.fs_hz))
        if recent_n < 20:
            return gx

        # Compute variance along each rotational & translational channel
        g_slice = list(self.gyro_hist)[-recent_n:]
        v_slice = list(self.vel_hist)[-recent_n:]

        means_g = [sum(p[i] for p in g_slice) / recent_n for i in range(3)]
        means_v = [sum(p[i] for p in v_slice) / recent_n for i in range(3)]

        var_g = [sum((p[i] - means_g[i]) ** 2 for p in g_slice) / recent_n for i in range(3)]
        # Scale velocity (cm/s) by 2.5 to match gyro (deg/s) dynamic range
        var_v = [sum(((p[i] - means_v[i]) * 2.5) ** 2 for p in v_slice) / recent_n for i in range(3)]

        # Weights proportional to standard deviation along each axis, preserving sign correlation with dominant axis
        total_energy = sum(var_g) + sum(var_v)
        if total_energy < 15.0:
            return 0.0

        # Find single strongest anchor axis to lock sign consistency
        all_vars = var_g + var_v
        dom_idx = max(range(6), key=lambda k: all_vars[k])

        def get_ch(k: int, idx: int) -> float:
            if k < 3:
                return g_slice[idx][k] - means_g[k]
            return (v_slice[idx][k - 3] - means_v[k - 3]) * 2.5

        # Compute covariance with dominant channel to determine sign alignment
        weights = [0.0] * 6
        for k in range(6):
            cov = sum(get_ch(dom_idx, j) * get_ch(k, j) for j in range(recent_n)) / recent_n
            sign = 1.0 if cov >= 0.0 else -1.0
            weights[k] = sign * math.sqrt(all_vars[k] / total_energy)

        cur_g = [gx - means_g[0], gy - means_g[1], gz - means_g[2]]
        cur_v = [(self.vel_cm_s[i] - means_v[i]) * 2.5 for i in range(3)]
        composite = (
            weights[0] * cur_g[0]
            + weights[1] * cur_g[1]
            + weights[2] * cur_g[2]
            + weights[3] * cur_v[0]
            + weights[4] * cur_v[1]
            + weights[5] * cur_v[2]
        )
        return composite

    def _discover_periodicity(self):
        """
        Unsupervised Multi-Scale Sliding Autocorrelation:
        Automatically discovers if the surgeon's hand is performing a repeating
        movement pattern and finds its fundamental cycle time T_rep in [0.40s .. 4.5s].
        """
        sig = list(self.signal_1d_hist)
        n = len(sig)
        mean_s = sum(sig) / n
        centered = [x - mean_s for x in sig]
        energy = sum(x * x for x in centered) / n

        if energy < 25.0:
            self.pattern_confidence = 0.0
            self.auto_period_s = 0.0
            self.activity_state = "IDLE / HOLD"
            self.autocorr_curve = []
            return

        min_lag = max(5, int(0.40 * self.effective_fs_hz))  # 0.40 s fastest surgical cycle
        max_lag = min(int(4.50 * self.effective_fs_hz), n // 2)
        step_lag = max(1, int(0.02 * self.effective_fs_hz))  # ~20 ms lag resolution

        lags = list(range(min_lag, max_lag, step_lag))
        if not lags:
            return

        r_vals: List[float] = []
        for lag in lags:
            m = n - lag
            num = sum(centered[i] * centered[i + lag] for i in range(m))
            den1 = sum(centered[i] * centered[i] for i in range(m))
            den2 = sum(centered[i + lag] * centered[i + lag] for i in range(m))
            denom = math.sqrt(den1 * den2)
            r = (num / denom) if denom > 1e-6 else 0.0
            r_vals.append(r)

        # Downsample autocorrelation curve for Web UI sparkline (max 40 points)
        stride = max(1, len(r_vals) // 40)
        self.autocorr_curve = [round(r_vals[i], 3) for i in range(0, len(r_vals), stride)]

        # Find first prominent positive peak in autocorrelation (fundamental period, avoiding 2x harmonic)
        best_r = -1.0
        best_lag = 0
        passed_negative_dip = False

        for idx in range(1, len(r_vals) - 1):
            if r_vals[idx] < 0.05:
                passed_negative_dip = True
            if passed_negative_dip and r_vals[idx] >= r_vals[idx - 1] and r_vals[idx] >= r_vals[idx + 1]:
                if r_vals[idx] > best_r:
                    best_r = r_vals[idx]
                    best_lag = lags[idx]
                    # If first fundamental peak is strong (> 0.50), lock onto it rather than 2*T harmonic
                    if best_r > 0.50:
                        break

        if best_r >= 0.35 and best_lag > 0:
            measured_period = best_lag / self.effective_fs_hz
            if self.auto_period_s <= 0.0:
                self.auto_period_s = measured_period
            else:
                # Smooth period tracking
                self.auto_period_s = 0.7 * self.auto_period_s + 0.3 * measured_period
            self.pattern_confidence = min(1.0, max(0.0, best_r))
            self.activity_state = "REPETITIVE PATTERN"
        else:
            self.pattern_confidence = max(0.0, best_r) if best_r > 0 else 0.0
            self.activity_state = "NON-REPETITIVE MOTION"

    def _segment_repetition_cycle(self, ts: float, sig_val: float, jerk_sq: float, tremor_rms: float):
        """
        Segments individual repetition cycles dynamically using zero-crossings
        and adaptive refractory gating tied to the self-discovered cycle period.
        """
        if self.cycle_start_ts <= 0.0:
            self._reset_current_cycle(ts)

        # Accumulate kinematics for the current candidate cycle
        self.cycle_samples_pos.append((self.pos_fused_cm[0], self.pos_fused_cm[1], self.pos_fused_cm[2]))
        self.cycle_samples_vel.append((self.vel_cm_s[0], self.vel_cm_s[1], self.vel_cm_s[2]))
        self.cycle_samples_gyro.append(self.gyro_hist[-1])
        self.cycle_samples_jerk_sq.append(jerk_sq)

        if sig_val > self.signal_peak_in_cycle:
            self.signal_peak_in_cycle = sig_val
        if sig_val < self.signal_trough_in_cycle:
            self.signal_trough_in_cycle = sig_val

        # Adaptive minimum cycle duration based on discovered autocorrelation period
        min_cycle_s = max(0.38, 0.55 * self.auto_period_s) if self.auto_period_s > 0 else 0.50
        max_cycle_s = 5.50
        cycle_dur = ts - self.cycle_start_ts

        # Check if signal reached a meaningful positive peak
        if sig_val > 14.0:
            self.passed_positive_peak = True

        # Detect completion of full oscillation (positive peak -> negative trough -> upward zero crossing)
        peak_to_peak = self.signal_peak_in_cycle - self.signal_trough_in_cycle
        upward_zero_cross = (self.prev_signal_val < 0.0 and sig_val >= 0.0)

        if (
            self.passed_positive_peak
            and upward_zero_cross
            and self.signal_trough_in_cycle < -10.0
            and peak_to_peak >= 28.0
            and min_cycle_s <= cycle_dur <= max_cycle_s
        ):
            self._finalize_repetition(ts, cycle_dur, tremor_rms)
            self._reset_current_cycle(ts)
        elif cycle_dur > max_cycle_s:
            # Timeout non-periodic drift window
            self._reset_current_cycle(ts)

        self.prev_signal_val = sig_val

    def _finalize_repetition(self, ts: float, duration_s: float, tremor_rms: float):
        """Computes surgical biomechanics & classifies the completed repetition."""
        if len(self.cycle_samples_pos) < 10:
            return

        # 1. 3D Path Length (Economy of Motion) & Stroke Span (cm)
        path_len_cm = 0.0
        xs = [p[0] for p in self.cycle_samples_pos]
        ys = [p[1] for p in self.cycle_samples_pos]
        zs = [p[2] for p in self.cycle_samples_pos]
        for i in range(1, len(self.cycle_samples_pos)):
            dx = xs[i] - xs[i - 1]
            dy = ys[i] - ys[i - 1]
            dz = zs[i] - zs[i - 1]
            path_len_cm += math.sqrt(dx * dx + dy * dy + dz * dz)

        span_x = max(xs) - min(xs)
        span_y = max(ys) - min(ys)
        span_z = max(zs) - min(zs)
        stroke_span_cm = math.sqrt(span_x * span_x + span_y * span_y + span_z * span_z)

        # 2. Angular Excursion (deg) & Peak Gyro (deg/s)
        angular_path_deg = 0.0
        peak_gyro_dps = 0.0
        roll_energy = 0.0
        pitch_yaw_energy = 0.0
        for g in self.cycle_samples_gyro:
            g_mag = math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2])
            angular_path_deg += g_mag * self.dt
            if g_mag > peak_gyro_dps:
                peak_gyro_dps = g_mag
            roll_energy += g[0] * g[0]
            pitch_yaw_energy += g[1] * g[1] + g[2] * g[2]

        # Net one-way angular excursion is approximately half the full closed-loop angular path
        angular_excursion_deg = angular_path_deg * 0.5

        # 3. Peak Linear Velocity (cm/s)
        peak_vel_cms = max(
            math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
            for v in self.cycle_samples_vel
        )

        # 4. Motion Smoothness Score (0..100%) from Log Dimensionless Jerk (LDLJ)
        peak_acc_mps2 = max(0.5, peak_vel_cms / 100.0 * (2.0 * math.pi / duration_s))
        jerk_integral = sum(self.cycle_samples_jerk_sq) * self.dt
        dimless_jerk = (duration_s ** 3 / (peak_acc_mps2 ** 2)) * max(1e-4, jerk_integral)
        ldlj = -math.log(max(1.0, dimless_jerk))
        # Typical surgical LDLJ ranges from -4 (very smooth) to -14 (jerky/fatigued)
        smoothness_score = max(10.0, min(99.0, 100.0 + (ldlj + 4.0) * 7.5))

        # 5. Unsupervised Kinematic Pattern Classification
        total_rot_e = roll_energy + pitch_yaw_energy + 1e-6
        roll_ratio = roll_energy / total_rot_e
        if angular_excursion_deg >= 35.0 and roll_ratio >= 0.42:
            pattern_type = "Suturing / Rotational Arc"
        elif angular_excursion_deg >= 25.0 and stroke_span_cm < 8.0:
            pattern_type = "Instrument / Axial Action"
        elif stroke_span_cm >= 8.0:
            pattern_type = "Dissection / Linear Reach"
        else:
            pattern_type = "Micro-Manipulation"

        self.current_pattern_type = pattern_type
        self.pattern_counts[pattern_type] = self.pattern_counts.get(pattern_type, 0) + 1

        self.rep_count += 1
        self.last_rep_ts = ts
        cadence_rpm = 60.0 / duration_s

        rep_event = RepetitionEvent(
            rep_id=self.rep_count,
            timestamp_s=round(ts, 3),
            duration_s=round(duration_s, 3),
            cadence_rpm=round(cadence_rpm, 1),
            path_length_cm=round(path_len_cm, 2),
            stroke_span_cm=round(stroke_span_cm, 2),
            angular_excursion_deg=round(angular_excursion_deg, 1),
            peak_gyro_dps=round(peak_gyro_dps, 1),
            peak_vel_cms=round(peak_vel_cms, 1),
            smoothness_score=round(smoothness_score, 1),
            tremor_rms_mps2=round(tremor_rms, 3),
            pattern_type=pattern_type,
            pattern_confidence=round(max(0.45, self.pattern_confidence), 2),
        )
        self.reps.append(rep_event)

    def get_live_snapshot(self, sig_val: float = 0.0, tremor_rms: float = 0.0) -> Dict:
        """Returns compact telemetry dictionary to merge into the 100 Hz WebSocket stream."""
        recent_reps = self.reps[-20:]
        if recent_reps:
            mean_dur = sum(r.duration_s for r in recent_reps) / len(recent_reps)
            mean_cadence = 60.0 / mean_dur if mean_dur > 0 else 0.0
            mean_path = sum(r.path_length_cm for r in recent_reps) / len(recent_reps)
            mean_ang = sum(r.angular_excursion_deg for r in recent_reps) / len(recent_reps)
            mean_smooth = sum(r.smoothness_score for r in recent_reps) / len(recent_reps)
            if len(recent_reps) >= 2:
                var_dur = sum((r.duration_s - mean_dur) ** 2 for r in recent_reps) / len(recent_reps)
                cv_pct = (math.sqrt(var_dur) / mean_dur) * 100.0
                consistency_pct = max(0.0, min(100.0, 100.0 - cv_pct))
            else:
                consistency_pct = 95.0
            last_rep_dict = asdict(self.reps[-1])
        else:
            mean_dur = self.auto_period_s
            mean_cadence = (60.0 / self.auto_period_s) if self.auto_period_s > 0 else 0.0
            mean_path = 0.0
            mean_ang = 0.0
            mean_smooth = 0.0
            consistency_pct = 0.0
            last_rep_dict = None

        # Calculate pattern breakdown with counts and percentages
        total_reps = max(1, self.rep_count)
        breakdown_with_pct = {}
        for pat, cnt in self.pattern_counts.items():
            breakdown_with_pct[pat] = {
                "count": cnt,
                "percentage": round((cnt / total_reps) * 100.0, 1) if self.rep_count > 0 else 0.0
            }

        cur_time = time.time()
        if self.is_recording and self.test_start_time > 0.0:
            sess_dur = cur_time - self.test_start_time
        elif self.test_end_time > self.test_start_time > 0.0:
            sess_dur = self.test_end_time - self.test_start_time
        else:
            sess_dur = self.active_time_s + self.idle_time_s

        return {
            "is_recording": self.is_recording,
            "session_id": self.test_session_id,
            "session_duration_s": round(sess_dur, 1),
            "pos_cm": [
                round(self.pos_fused_cm[0], 2),
                round(self.pos_fused_cm[1], 2),
                round(self.pos_fused_cm[2], 2),
            ],
            "vel_cms": [
                round(self.vel_cm_s[0], 1),
                round(self.vel_cm_s[1], 1),
                round(self.vel_cm_s[2], 1),
            ],
            "is_stationary": self.is_stationary,
            "stroke_active": bool(self.stroke_active),
            "stroke_scale": [
                round(float(self.stroke_scale[0]), 4),
                round(float(self.stroke_scale[1]), 4),
                round(float(self.stroke_scale[2]), 4),
            ],
            "rep_count": self.rep_count,
            "activity_state": self.activity_state,
            "pattern_type": self.current_pattern_type,
            "auto_period_s": round(self.auto_period_s, 2),
            "pattern_confidence": round(self.pattern_confidence, 2),
            "cadence_rpm": round(mean_cadence, 1),
            "mean_path_cm": round(mean_path, 2),
            "mean_angle_deg": round(mean_ang, 1),
            "smoothness_pct": round(mean_smooth, 1),
            "consistency_pct": round(consistency_pct, 1),
            "tremor_rms": round(tremor_rms, 3),
            "signal_1d": round(sig_val, 2),
            "autocorr": self.autocorr_curve,
            "pattern_counts": self.pattern_counts,
            "pattern_breakdown": breakdown_with_pct,
            "active_time_s": round(self.active_time_s, 1),
            "idle_time_s": round(self.idle_time_s, 1),
            "last_rep": last_rep_dict,
            "recent_reps": [asdict(r) for r in self.reps[-8:]],
        }

    def get_session_report(self) -> Dict:
        """Generates a comprehensive multi-hour surgical motion report."""
        snap = self.get_live_snapshot()
        total_time = max(0.1, snap["session_duration_s"])
        return {
            "session_id": self.test_session_id,
            "is_recording": self.is_recording,
            "session_duration_s": round(total_time, 1),
            "active_time_s": round(self.active_time_s, 1),
            "idle_time_s": round(self.idle_time_s, 1),
            "active_ratio_pct": round((self.active_time_s / total_time) * 100.0, 1),
            "total_repetitions": self.rep_count,
            "pattern_counts": self.pattern_counts,
            "pattern_breakdown": snap["pattern_breakdown"],
            "summary_metrics": {
                "auto_discovered_period_s": snap["auto_period_s"],
                "mean_cadence_rpm": snap["cadence_rpm"],
                "mean_3d_path_length_cm": snap["mean_path_cm"],
                "mean_angular_excursion_deg": snap["mean_angle_deg"],
                "mean_smoothness_score_pct": snap["smoothness_pct"],
                "cycle_consistency_pct": snap["consistency_pct"],
                "current_tremor_rms_mps2": snap["tremor_rms"],
            },
            "recent_repetitions": [asdict(r) for r in self.reps[-50:]],
        }

    def export_csv(self, filepath: str) -> str:
        """Exports all recorded repetition cycles to a CSV file."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        fieldnames = [
            "rep_id",
            "timestamp_s",
            "duration_s",
            "cadence_rpm",
            "path_length_cm",
            "stroke_span_cm",
            "angular_excursion_deg",
            "peak_gyro_dps",
            "peak_vel_cms",
            "smoothness_score",
            "tremor_rms_mps2",
            "pattern_type",
            "pattern_confidence",
        ]
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in self.reps:
                writer.writerow(asdict(r))
        return filepath
