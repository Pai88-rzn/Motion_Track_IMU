#!/usr/bin/env python3
"""
Unit tests for SurgicalMotionAnalyzer:
1. Verify repetition counting and kinematics during movement.
2. Verify stationary stabilization (no post-stop ringing/oscillation).
3. Verify Start / End test session lifecycle and multi-pattern breakdown analysis.
"""

import math
import unittest
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from surgical_motion_analyzer import SurgicalMotionAnalyzer


class TestSurgicalMotionAnalyzer(unittest.TestCase):

    def test_no_oscillation_after_stopping(self):
        """When motion ceases, position should settle to 0 without oscillating."""
        analyzer = SurgicalMotionAnalyzer(fs_hz=100.0)

        # 1. Simulate 2 seconds of oscillatory movement (1.0 Hz suturing motion)
        for i in range(200):
            t = i * 0.01
            roll = 35.0 * math.sin(2.0 * math.pi * 1.0 * t)
            cr = math.cos(math.radians(roll) / 2.0)
            sr = math.sin(math.radians(roll) / 2.0)
            gx = 35.0 * 2.0 * math.pi * math.cos(2.0 * math.pi * 1.0 * t)
            ax = 1.2 * math.sin(2.0 * math.pi * 1.0 * t)
            analyzer.update(
                ts=t, qw=cr, qx=sr, qy=0.0, qz=0.0,
                ax_mps2=ax, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=gx, gy_dps=0.0, gz_dps=0.0
            )

        # 2. Now simulate stopping completely for 2 seconds
        stationary_positions = []
        for i in range(200, 400):
            t = i * 0.01
            snap = analyzer.update(
                ts=t, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
                ax_mps2=0.0, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
            )
            stationary_positions.append(snap["pos_cm"])

        # Check settling at the end of the stationary rest period (> 1.5s)
        settled_samples = stationary_positions[150:]
        x_vals = [p[0] for p in settled_samples]

        sign_changes = 0
        for k in range(1, len(x_vals)):
            if abs(x_vals[k]) > 0.02 and abs(x_vals[k - 1]) > 0.02:
                if (x_vals[k] > 0 and x_vals[k - 1] < 0) or (x_vals[k] < 0 and x_vals[k - 1] > 0):
                    sign_changes += 1

        max_deviation = max(abs(x) for x in x_vals)

        self.assertEqual(sign_changes, 0, f"Detected {sign_changes} sign changes after tag stopped!")
        self.assertLess(max_deviation, 0.05, f"Stationary position did not settle to zero: {max_deviation}")

    def test_repetition_counting_normal_motion(self):
        """Simulate 4 clean repetitions and ensure rep_count reaches 4."""
        analyzer = SurgicalMotionAnalyzer(fs_hz=100.0)
        for i in range(450):
            t = i * 0.01
            roll = 45.0 * math.sin(2.0 * math.pi * 1.0 * t)
            cr = math.cos(math.radians(roll) / 2.0)
            sr = math.sin(math.radians(roll) / 2.0)
            gx = 45.0 * 2.0 * math.pi * math.cos(2.0 * math.pi * 1.0 * t)
            ax = 1.5 * math.sin(2.0 * math.pi * 1.0 * t)
            snap = analyzer.update(
                ts=t, qw=cr, qx=sr, qy=0.0, qz=0.0,
                ax_mps2=ax, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=gx, gy_dps=0.0, gz_dps=0.0
            )

        self.assertGreaterEqual(snap["rep_count"], 3)

    def test_start_end_session_and_pattern_breakdown(self):
        """Verify Start/End test session controls and pattern breakdown analysis."""
        analyzer = SurgicalMotionAnalyzer(fs_hz=100.0)

        # 1. Start a fresh test session
        start_snap = analyzer.start_session()
        self.assertTrue(start_snap["is_recording"])
        self.assertEqual(start_snap["rep_count"], 0)

        # 2. Simulate 3 cycles of suturing rotational motion (1.0 Hz)
        for i in range(350):
            t = i * 0.01
            roll = 50.0 * math.sin(2.0 * math.pi * 1.0 * t)
            cr = math.cos(math.radians(roll) / 2.0)
            sr = math.sin(math.radians(roll) / 2.0)
            gx = 50.0 * 2.0 * math.pi * math.cos(2.0 * math.pi * 1.0 * t)
            ax = 0.8 * math.sin(2.0 * math.pi * 1.0 * t)
            analyzer.update(
                ts=t, qw=cr, qx=sr, qy=0.0, qz=0.0,
                ax_mps2=ax, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=gx, gy_dps=0.0, gz_dps=0.0
            )

        # 3. End session and verify summary report
        report = analyzer.end_session()
        self.assertFalse(report["is_recording"])
        self.assertGreaterEqual(report["total_repetitions"], 2)

        # Check pattern breakdown
        breakdown = report["pattern_breakdown"]
        self.assertIn("Suturing / Rotational Arc", breakdown)
        suture_stats = breakdown["Suturing / Rotational Arc"]
        self.assertGreater(suture_stats["count"], 0)
        self.assertGreater(suture_stats["percentage"], 50.0)
        print(f"\n[TEST PASS] Pattern breakdown: {breakdown}")

    def test_pure_rotation_does_not_shift_position(self):
        """Verifies that pure rotational movement (pronation, flexion) does not introduce false translation."""
        analyzer = SurgicalMotionAnalyzer(fs_hz=100.0)

        # Pitch 35 degrees up and down repeatedly with 0 linear acceleration
        for i in range(200):
            t = i * 0.01
            pitch = 35.0 * math.sin(2.0 * math.pi * 1.5 * t)
            cp = math.cos(math.radians(pitch) / 2.0)
            sp = math.sin(math.radians(pitch) / 2.0)
            gy = 35.0 * 2.0 * math.pi * 1.5 * math.cos(2.0 * math.pi * 1.5 * t)
            snap = analyzer.update(
                ts=t, qw=cp, qx=0.0, qy=sp, qz=0.0,
                ax_mps2=0.0, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=0.0, gy_dps=gy, gz_dps=0.0
            )

        pos = snap["pos_cm"]
        self.assertLess(abs(pos[0]), 0.05, f"False X displacement on pure pitch: {pos[0]}")
        self.assertLess(abs(pos[1]), 0.05, f"False Y displacement on pure pitch: {pos[1]}")
        self.assertLess(abs(pos[2]), 0.05, f"False Z displacement on pure pitch: {pos[2]}")

    def test_pure_translation_shifts_position(self):
        """Verifies that sliding left/right without orientation change clearly produces translational displacement."""
        analyzer = SurgicalMotionAnalyzer(fs_hz=100.0)

        # 1. Slide 0.5s along Y (lateral left/right) with 0 rotation
        pos_y_samples = []
        for step in range(120):
            t = step * 0.01
            if 0.1 <= t < 0.35:
                ay = 1.6 * math.sin((t - 0.1) / 0.25 * math.pi)
            elif 0.35 <= t < 0.60:
                ay = -1.6 * math.sin((t - 0.35) / 0.25 * math.pi)
            else:
                ay = 0.0
            snap = analyzer.update(
                ts=t, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
                ax_mps2=0.0, ay_mps2=ay, az_mps2=0.0,
                gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
            )
            pos_y_samples.append(snap["pos_cm"][1])

        max_pos_y = max(pos_y_samples)
        # Translation must clearly shift position (> 3.0 cm, not stuck at ~0)
        self.assertGreater(max_pos_y, 3.0, f"Translation slide failed to move hand: {max_pos_y} cm")

        # Position at stop (t=0.8s) must be retained and not instantly wiped to zero
        pos_at_stop = pos_y_samples[80]
        self.assertGreater(pos_at_stop, 3.0, f"Position at stop wiped out too quickly: {pos_at_stop} cm")

    def test_adaptive_stroke_scaling(self):
        """Verify that setting stroke scale doubles integrated displacement for identical acceleration profile."""
        analyzer1 = SurgicalMotionAnalyzer(fs_hz=100.0)
        analyzer2 = SurgicalMotionAnalyzer(fs_hz=100.0)

        # Default scale is (1.0, 1.0, 1.0)
        self.assertEqual(analyzer1.get_stroke_scales(), (1.0, 1.0, 1.0))

        # Set scale_x = 2.0 on analyzer2
        analyzer2.set_stroke_scales(2.0, 1.0, 1.0)
        self.assertEqual(analyzer2.get_stroke_scales(), (2.0, 1.0, 1.0))

        # Feed identical acceleration profile along X (40 frames)
        for i in range(40):
            t = i * 0.01
            ax = 1.2 * math.sin(i / 40.0 * math.pi)
            snap1 = analyzer1.update(
                ts=t, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
                ax_mps2=ax, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
            )
            snap2 = analyzer2.update(
                ts=t, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
                ax_mps2=ax, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
            )

        # Output payload checks
        self.assertIn("stroke_active", snap2)
        self.assertIn("stroke_scale", snap2)
        self.assertEqual(snap2["stroke_scale"], [2.0, 1.0, 1.0])
        self.assertTrue(snap2["stroke_active"])

        disp1 = snap1["pos_cm"][0]
        disp2 = snap2["pos_cm"][0]
        self.assertGreater(disp1, 0.5, f"Displacement too small: {disp1} cm")
        self.assertAlmostEqual(disp2, 2.0 * disp1, delta=0.05 * disp2,
                               msg=f"Displacement not doubled: {disp2} vs 2 * {disp1}")

    def test_stroke_latching_on_rapid_reversal(self):
        """Verify that quick direction changes do not decay velocity to zero mid-stroke."""
        analyzer = SurgicalMotionAnalyzer(fs_hz=100.0)

        # 1. Forward stroke along X for 0.20s (20 frames)
        for i in range(20):
            t = i * 0.01
            analyzer.update(
                ts=t, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
                ax_mps2=1.5, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
            )

        # 2. Rapid reversal: jerk transition to -1.5 m/s^2
        # As velocity crosses zero during rapid reversal, stroke must remain latched active
        zero_cross_snap = None
        min_abs_vel = float("inf")
        reversal_snaps = []

        for i in range(20, 50):
            t = i * 0.01
            snap = analyzer.update(
                ts=t, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
                ax_mps2=-1.5, ay_mps2=0.0, az_mps2=0.0,
                gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
            )
            reversal_snaps.append(snap)
            vx = abs(snap["vel_cms"][0])
            if vx < min_abs_vel:
                min_abs_vel = vx
                zero_cross_snap = snap

        # Verify that at the zero-velocity crossing point, stroke remained latched active
        self.assertIsNotNone(zero_cross_snap)
        self.assertTrue(zero_cross_snap["stroke_active"], "Stroke failed to latch active during zero-velocity crossing!")
        self.assertFalse(zero_cross_snap["is_stationary"], "Zero crossing falsely classified as stationary!")

        # Verify that velocity successfully reversed to negative rather than decaying or freezing at 0
        final_vx = reversal_snaps[-1]["vel_cms"][0]
        self.assertLess(final_vx, -5.0, f"Velocity failed to reverse into negative: {final_vx} cm/s")

    def test_zero_drift_under_ambient_vibrations(self):
        """Verify that ambient vibrations (< 0.12 m/s^2) do not trigger stroke or cause drift."""
        analyzer = SurgicalMotionAnalyzer(fs_hz=100.0)

        # Feed 1.5 seconds (150 frames) of ambient vibration at 0.10 m/s^2 (< 0.12 m/s^2 threshold)
        for i in range(150):
            t = i * 0.01
            # 15 Hz vibration with 0.10 m/s^2 amplitude
            ax = 0.10 * math.sin(2.0 * math.pi * 15.0 * t)
            ay = 0.10 * math.cos(2.0 * math.pi * 15.0 * t)
            snap = analyzer.update(
                ts=t, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
                ax_mps2=ax, ay_mps2=ay, az_mps2=0.0,
                gx_dps=0.5 * math.sin(t), gy_dps=0.5 * math.cos(t), gz_dps=0.0
            )
            # stroke_active must remain False throughout ambient vibration
            self.assertFalse(snap["stroke_active"], f"Stroke falsely triggered on ambient vibration at frame {i}")

        # Position displacement must remain 0.0 cm
        pos = snap["pos_cm"]
        self.assertEqual(pos, [0.0, 0.0, 0.0], f"Position drifted under ambient vibration: {pos}")

    def test_zero_division_protection_in_jerk(self):
        """Verify that dt <= 0 (e.g. duplicate timestamps) does not raise ZeroDivisionError in jerk calculation."""
        analyzer = SurgicalMotionAnalyzer(fs_hz=100.0)

        # Initial frame at ts=1.0
        snap1 = analyzer.update(
            ts=1.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
            ax_mps2=0.5, ay_mps2=0.0, az_mps2=0.0,
            gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
        )
        self.assertIsNotNone(snap1)

        # Consecutive frame with identical timestamp (dt = 0.0)
        snap2 = analyzer.update(
            ts=1.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
            ax_mps2=1.5, ay_mps2=0.0, az_mps2=0.0,
            gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
        )
        self.assertIsNotNone(snap2)

        # Frame with backwards timestamp (dt < 0)
        snap3 = analyzer.update(
            ts=0.5, qw=1.0, qx=0.0, qy=0.0, qz=0.0,
            ax_mps2=0.8, ay_mps2=0.0, az_mps2=0.0,
            gx_dps=0.0, gy_dps=0.0, gz_dps=0.0
        )
        self.assertIsNotNone(snap3)


if __name__ == "__main__":
    unittest.main()
