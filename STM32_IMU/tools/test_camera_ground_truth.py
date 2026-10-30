#!/usr/bin/env python3
"""
Unit tests for Optical Ground-Truth Engine & ArUco Pose Tracker (camera_ground_truth.py).
"""

import os
import sys
import numpy as np
import cv2
import pytest

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from camera_ground_truth import generate_aruco_marker_png, CameraGroundTruthTracker


def test_generate_aruco_marker_png():
    """Verify ArUco marker image generation returns valid PNG bytes of requested size."""
    png_bytes = generate_aruco_marker_png(marker_id=0, size_px=200)
    assert len(png_bytes) > 0
    img = cv2.imdecode(np.frombuffer(png_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert img is not None
    assert img.shape == (200, 200)


def test_detect_marker_pose_synthetic():
    """Verify detection and 6DOF pose estimation on a synthetic frame."""
    tracker = CameraGroundTruthTracker(video_source=-1, marker_size_cm=4.0)

    # Draw marker using tracker's dictionary
    if hasattr(cv2.aruco, "drawMarker"):
        marker_img = cv2.aruco.drawMarker(tracker.dictionary, 0, 200)
    else:
        marker_img = cv2.aruco.generateImageMarker(tracker.dictionary, 0, 200)

    canvas = np.ones((480, 640, 3), dtype=np.uint8) * 255
    canvas[140:340, 220:420] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

    annotated, data = tracker.process_frame(canvas)
    assert data["detected"] is True
    assert len(data["pos_cm"]) == 3
    assert data["pos_cm"][2] > 0.0  # Positive depth Z
    assert "delta_cm" in data
    assert len(data["delta_cm"]) == 3
    assert annotated.shape == (480, 640, 3)


def test_tare_and_displacement():
    """Verify tare zeroing and relative displacement tracking."""
    tracker = CameraGroundTruthTracker(video_source=-1, marker_size_cm=4.0)

    if hasattr(cv2.aruco, "drawMarker"):
        marker_img = cv2.aruco.drawMarker(tracker.dictionary, 0, 200)
    else:
        marker_img = cv2.aruco.generateImageMarker(tracker.dictionary, 0, 200)

    # Frame 1: Center position
    canvas1 = np.ones((480, 640, 3), dtype=np.uint8) * 255
    canvas1[140:340, 220:420] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    _, data1 = tracker.process_frame(canvas1)
    assert data1["detected"] is True

    # Tare at current position
    tracker.tare()
    _, data_tared = tracker.process_frame(canvas1)
    # Displacement relative to tare origin should now be zero (or near 0.0)
    for delta in data_tared["delta_cm"]:
        assert abs(delta) < 0.1

    # Frame 2: Shifted right by 100 pixels
    canvas2 = np.ones((480, 640, 3), dtype=np.uint8) * 255
    canvas2[140:340, 320:520] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    _, data2 = tracker.process_frame(canvas2)
    assert data2["detected"] is True
    # delta_x should be positive (moved right in camera view)
    assert data2["delta_cm"][0] > 0.5


def test_undetected_marker():
    """Verify handling when no marker is present in the frame."""
    tracker = CameraGroundTruthTracker(video_source=-1, marker_size_cm=4.0)
    canvas = np.ones((480, 640, 3), dtype=np.uint8) * 255

    annotated, data = tracker.process_frame(canvas)
    assert data["detected"] is False
    assert data["pos_cm"] == [0.0, 0.0, 0.0]
    assert data["delta_cm"] == [0.0, 0.0, 0.0]
    assert annotated.shape == (480, 640, 3)


def test_jpeg_frame_encoding():
    """Verify JPEG frame encoding returns valid JPEG bytes."""
    tracker = CameraGroundTruthTracker(video_source=-1, marker_size_cm=4.0)
    canvas = np.zeros((480, 640, 3), dtype=np.uint8)
    tracker.process_frame(canvas)
    jpeg_bytes = tracker.get_jpeg_frame()
    assert isinstance(jpeg_bytes, (bytes, bytearray))
    assert len(jpeg_bytes) > 0
    # Standard JPEG header starts with 0xFF, 0xD8
    assert jpeg_bytes[:2] == b"\xff\xd8"


def test_tare_when_occluded_arms_next_detection():
    """Verify tare requested while marker is occluded does not use stale pose, but tares on next detection."""
    tracker = CameraGroundTruthTracker(video_source=-1, marker_size_cm=4.0)

    if hasattr(cv2.aruco, "drawMarker"):
        marker_img = cv2.aruco.drawMarker(tracker.dictionary, 0, 200)
    else:
        marker_img = cv2.aruco.generateImageMarker(tracker.dictionary, 0, 200)

    # 1. Detect at position 1 (Center)
    canvas1 = np.ones((480, 640, 3), dtype=np.uint8) * 255
    canvas1[140:340, 220:420] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    _, data1 = tracker.process_frame(canvas1)
    assert data1["detected"] is True

    # 2. Occlude marker completely (blank frame)
    blank = np.ones((480, 640, 3), dtype=np.uint8) * 255
    _, data_blank = tracker.process_frame(blank)
    assert data_blank["detected"] is False

    # 3. Call tare while occluded
    tracker.tare()
    assert tracker.tare_origin is None
    assert tracker._tare_requested is True

    # 4. Marker reappears at a NEW position (shifted right)
    canvas2 = np.ones((480, 640, 3), dtype=np.uint8) * 255
    canvas2[140:340, 320:520] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    _, data2 = tracker.process_frame(canvas2)
    assert data2["detected"] is True
    assert tracker._tare_requested is False
    assert tracker.tare_origin is not None
    # Because tare was re-armed, new position canvas2 should be zeroed out (delta near 0.0)
    for delta in data2["delta_cm"]:
        assert abs(delta) < 0.1


def test_concurrent_tare_and_process_frame():
    """Verify thread safety when tare() and process_frame() are called concurrently."""
    import threading

    tracker = CameraGroundTruthTracker(video_source=-1, marker_size_cm=4.0)
    if hasattr(cv2.aruco, "drawMarker"):
        marker_img = cv2.aruco.drawMarker(tracker.dictionary, 0, 200)
    else:
        marker_img = cv2.aruco.generateImageMarker(tracker.dictionary, 0, 200)

    canvas = np.ones((480, 640, 3), dtype=np.uint8) * 255
    canvas[140:340, 220:420] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

    stop_flag = False
    errors = []

    def run_process():
        while not stop_flag:
            try:
                tracker.process_frame(canvas)
            except Exception as e:
                errors.append(e)

    def run_tare():
        while not stop_flag:
            try:
                tracker.tare()
            except Exception as e:
                errors.append(e)

    threads = [
        threading.Thread(target=run_process),
        threading.Thread(target=run_process),
        threading.Thread(target=run_tare),
        threading.Thread(target=run_tare),
    ]

    for t in threads:
        t.start()

    import time
    time.sleep(0.3)
    stop_flag = True

    for t in threads:
        t.join()

    assert len(errors) == 0

