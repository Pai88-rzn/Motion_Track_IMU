#!/usr/bin/env python3
"""
Unit and integration tests for dual-view optical ground-truth streaming
and telemetry fusion endpoints in imu_web_server.py.
"""

import os
import sys
import json
import pytest
from fastapi.testclient import TestClient

# Ensure tools directory is on sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from imu_web_server import app, analyzer, tracker, latest_imu_data


@pytest.fixture
def client():
    with TestClient(app) as test_client:
        yield test_client


def test_aruco_marker_png_endpoint(client):
    """Verify /aruco_marker.png returns 200, image/png media type, and valid PNG magic bytes."""
    response = client.get("/aruco_marker.png")
    assert response.status_code == 200
    assert response.headers["content-type"] == "image/png"
    assert len(response.content) > 0
    # Check PNG signature: 89 50 4E 47 0D 0A 1A 0A
    assert response.content.startswith(b"\x89PNG\r\n\x1a\n")


def test_aruco_marker_custom_params(client):
    """Verify /aruco_marker.png supports custom marker_id and size parameters."""
    response = client.get("/aruco_marker.png?marker_id=3&size=200")
    assert response.status_code == 200
    assert response.headers["content-type"] == "image/png"
    assert response.content.startswith(b"\x89PNG\r\n\x1a\n")


def test_api_tare_both(client):
    """Verify POST /api/tare_both zeroes both IMU and Camera ground-truth origins."""
    response = client.post("/api/tare_both")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "ok"
    assert "origins tared" in data["message"].lower()


def test_api_set_stroke_scales(client):
    """Verify POST /api/set_stroke_scales updates per-axis stroke scaling factors."""
    scales_payload = {"scale_x": 1.25, "scale_y": 0.85, "scale_z": 1.10}
    response = client.post("/api/set_stroke_scales", json=scales_payload)
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "ok"
    assert data["stroke_scales"] == [1.25, 0.85, 1.10]
    assert analyzer.get_stroke_scales() == (1.25, 0.85, 1.10)


def test_api_set_stroke_scales_validation(client):
    """Verify POST /api/set_stroke_scales rejects values <= 0.01 or > 10.0 with HTTP 422."""
    # Under lower bound (<= 0.01)
    res_low = client.post("/api/set_stroke_scales", json={"scale_x": 0.005, "scale_y": 1.0, "scale_z": 1.0})
    assert res_low.status_code == 422

    # Over upper bound (> 10.0)
    res_high = client.post("/api/set_stroke_scales", json={"scale_x": 1.0, "scale_y": 12.0, "scale_z": 1.0})
    assert res_high.status_code == 422


def test_video_feed_streaming(client):
    """Verify GET /video_feed returns multipart/x-mixed-replace with JPEG frames."""
    response = client.get("/video_feed?max_frames=1")
    assert response.status_code == 200
    assert "multipart/x-mixed-replace" in response.headers["content-type"]
    assert "boundary=frame" in response.headers["content-type"]
    assert b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" in response.content
    assert b"\xff\xd8" in response.content  # JPEG SOI marker


def test_api_status_includes_optical_and_scales(client):
    """Verify GET /api/status includes stroke_scales and optical ground truth status fields."""
    response = client.get("/api/status")
    assert response.status_code == 200
    data = response.json()
    assert "stroke_scales" in data
    assert "camera_detected" in data
    assert "camera_status" in data
    assert "camera_fps" in data
    assert len(data["stroke_scales"]) == 3


def test_websocket_telemetry_fusion(client):
    """Verify WebSocket /ws broadcasts fused telemetry containing both IMU and camera objects."""
    with client.websocket_connect("/ws") as ws:
        if latest_imu_data.get("ts", 0.0) > 0:
            _ = ws.receive_json()
        # Send a simulated IMU sample
        sim_cmd = {
            "action": "sim_sample",
            "ts": 123.456,
            "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
            "ax": 0.2, "ay": -0.1, "az": 9.81,
            "gx": 1.0, "gy": 0.0, "gz": -1.0
        }
        ws.send_json(sim_cmd)
        msg = ws.receive_json()

        assert msg.get("type") == "imu"
        assert "analytics" in msg
        assert "camera" in msg
        camera_data = msg["camera"]
        assert "detected" in camera_data
        assert "pos_cm" in camera_data
        assert "delta_cm" in camera_data
        assert "status" in camera_data


def test_websocket_tare_both_action(client):
    """Verify WebSocket tare_both action zeroes both IMU and Camera origins."""
    with client.websocket_connect("/ws") as ws:
        if latest_imu_data.get("ts", 0.0) > 0:
            _ = ws.receive_json()
        ws.send_json({"action": "tare_both"})
        msg = ws.receive_json()
        assert "analytics" in msg
        assert "camera" in msg
        assert msg["camera"]["delta_cm"] == [0.0, 0.0, 0.0]


def test_websocket_set_stroke_scales_action(client):
    """Verify WebSocket set_stroke_scales action updates calibration scales."""
    with client.websocket_connect("/ws") as ws:
        if latest_imu_data.get("ts", 0.0) > 0:
            _ = ws.receive_json()
        ws.send_json({
            "action": "set_stroke_scales",
            "scale_x": 1.08,
            "scale_y": 0.92,
            "scale_z": 1.01
        })
        msg = ws.receive_json()
        assert "analytics" in msg
        assert msg["analytics"]["stroke_scale"] == [1.08, 0.92, 1.01]
        assert analyzer.get_stroke_scales() == (1.08, 0.92, 1.01)


def test_websocket_set_stroke_scales_clamping(client):
    """Verify WebSocket set_stroke_scales action clamps out-of-range values to [0.01, 10.0]."""
    with client.websocket_connect("/ws") as ws:
        if latest_imu_data.get("ts", 0.0) > 0:
            _ = ws.receive_json()
        ws.send_json({
            "action": "set_stroke_scales",
            "scale_x": -2.0,   # Should clamp to 0.01
            "scale_y": 25.0,   # Should clamp to 10.0
            "scale_z": 0.001   # Should clamp to 0.01
        })
        msg = ws.receive_json()
        assert "analytics" in msg
        assert msg["analytics"]["stroke_scale"] == [0.01, 10.0, 0.01]
        assert analyzer.get_stroke_scales() == (0.01, 10.0, 0.01)


def test_graceful_degradation_offline_camera():
    """Verify tracker serves a valid placeholder frame and detected=False when camera offline."""
    frame_bytes = tracker.get_jpeg_frame()
    assert len(frame_bytes) > 0
    assert frame_bytes.startswith(b"\xff\xd8")  # Valid JPEG SOI marker
    telemetry = tracker.get_latest_data()
    assert isinstance(telemetry["detected"], bool)
    assert len(telemetry["pos_cm"]) == 3
    assert len(telemetry["delta_cm"]) == 3
