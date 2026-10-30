# Dual-View Web + Optical Ground-Truth Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate live optical camera ground-truth tracking (`/dev/video0`) with OpenCV ArUco into the web visualizer and tune the adaptive stroke-based IMU filter for drift-free, 1:1 synchronized hand tracking.

**Architecture:** A multi-threaded video capture module detects ArUco markers at 30 FPS, streams annotated MJPEG frames to the web dashboard, and feeds ground-truth displacements alongside 100 Hz IMU telemetry. The web visualizer receives both streams, displaying a Picture-in-Picture camera feed and side-by-side error metrics. The IMU filter uses calibrated stroke scaling to achieve 1:1 displacement matching without runtime camera reliance.

**Tech Stack:** Python 3.10+, FastAPI, OpenCV 4.5.4 (cv2.aruco), Uvicorn, Three.js, WebSocket.

**Spec:** `docs/superpowers/specs/2026-10-30-stroke-based-optical-calibration-design.md`

## Global Constraints

- Stand-Alone Production: IMU tracking must not require runtime camera access to operate.
- Optical Ground-Truth: Uses standard ArUco dictionary `DICT_4X4_50`, Marker ID 0, physical size 40 mm.
- Serial Telemetry: 36-byte packets @ 100.3 Hz on `/dev/ttyUSB0` @ 115200 baud.
- Video Feed: Device `/dev/video0` @ 30 FPS, 640x480 resolution (optimized for low CPU/USB bus contention).
- Latency & Responsiveness: Motion feedback latency in visualizer < 30 ms; zero stationary drift when still.

## Review Focus

- Camera disconnect or missing `/dev/video0`: Server must gracefully serve mock/offline placeholder without crashing IMU streaming.
- Marker occlusion during calibration: Optical tracker must report `detected: False` and preserve last known valid tare reference.
- Zero-acceleration jitter: Power-law stroke filter must not trigger on ambient table vibrations ($|a| < 0.12\text{ m/s}^2$).
- Rapid sign reversal during active strokes: Jerk-based stroke latching must prevent premature damping during continuous back-and-forth oscillations.
- WebSocket saturation: MJPEG video streaming must run over HTTP `/video_feed` independent of the 100 Hz IMU WebSocket channel.

---

### Task 1: Optical Ground-Truth Engine & ArUco Pose Tracker

**Files:**
- Create: `STM32_IMU/tools/camera_ground_truth.py`
- Test: `STM32_IMU/tools/test_camera_ground_truth.py`

**Interfaces:**
- Produces:
  - `generate_aruco_marker_png(marker_id: int = 0, size_px: int = 400) -> bytes`
  - `class CameraGroundTruthTracker`:
    - `__init__(video_source: int = 0, marker_size_cm: float = 4.0)`
    - `tare()`: sets current 3D position as ground-truth origin $(0, 0, 0)$
    - `process_frame(frame: np.ndarray) -> Tuple[np.ndarray, Dict]`: returns annotated frame with 3D axes and dict `{detected: bool, pos_cm: [x,y,z], delta_cm: [dx,dy,dz]}`
    - `get_latest_data() -> Dict`
    - `get_jpeg_frame() -> bytes`
    - `start()`, `stop()`

- [x] **Step 1: Write unit tests for ArUco generation, detection, and displacement tracking**

```python
# In STM32_IMU/tools/test_camera_ground_truth.py
import pytest
import numpy as np
import cv2
from camera_ground_truth import generate_aruco_marker_png, CameraGroundTruthTracker

def test_generate_aruco_marker_png():
    png_bytes = generate_aruco_marker_png(marker_id=0, size_px=200)
    assert len(png_bytes) > 0
    img = cv2.imdecode(np.frombuffer(png_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert img.shape == (200, 200)

def test_detect_marker_pose_synthetic():
    tracker = CameraGroundTruthTracker(video_source=-1, marker_size_cm=4.0)
    # Create white canvas and draw marker
    marker_img = cv2.aruco.drawMarker(tracker.dictionary, 0, 200)
    canvas = np.ones((480, 640, 3), dtype=np.uint8) * 255
    canvas[140:340, 220:420] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    
    annotated, data = tracker.process_frame(canvas)
    assert data["detected"] is True
    assert len(data["pos_cm"]) == 3
    assert data["pos_cm"][2] > 0.0  # Positive depth Z
```

- [x] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest "/home/rifai/Imu_MOTION TRACKER/STM32_IMU/tools/test_camera_ground_truth.py"`
Expected: FAIL (module `camera_ground_truth` not found).

- [x] **Step 3: Implement `camera_ground_truth.py`**

Implement `generate_aruco_marker_png` and `CameraGroundTruthTracker` with robust OpenCV ArUco detector (handling both OpenCV 4.5 and 4.7+ API variants), pinhole camera matrix estimation, running tare vector, and thread-safe frame grabbing.

- [x] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest "/home/rifai/Imu_MOTION TRACKER/STM32_IMU/tools/test_camera_ground_truth.py"`
Expected: PASS (2/2 tests pass).

---

### Task 2: Adaptive Stroke Filter & Dynamic Axis Scaling in `SurgicalMotionAnalyzer`

**Files:**
- Modify: `STM32_IMU/tools/surgical_motion_analyzer.py`
- Modify: `STM32_IMU/tools/test_motion_analyzer.py`

**Interfaces:**
- Consumes: Raw linear acceleration $a_{body}$, orientation quaternion $q$, angular velocity $\omega$.
- Produces:
  - `set_stroke_scales(scale_x: float, scale_y: float, scale_z: float)`
  - `get_stroke_scales() -> Tuple[float, float, float]`
  - `update(...) -> Dict`: Output dictionary includes `stroke_active: bool`, `stroke_scale: [kx, ky, kz]`, calibrated `pos_cm: [x, y, z]`.

- [x] **Step 1: Write test for adaptive stroke scaling and zero stationary drift**

In `test_motion_analyzer.py`, add tests:
- `test_adaptive_stroke_scaling()`: Verify that setting stroke scale doubles integrated displacement for identical acceleration profile.
- `test_stroke_latching_on_rapid_reversal()`: Verify that quick direction changes do not decay velocity to zero mid-stroke.

- [x] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest "/home/rifai/Imu_MOTION TRACKER/STM32_IMU/tools/test_motion_analyzer.py::test_adaptive_stroke_scaling"`
Expected: FAIL (`AttributeError: 'SurgicalMotionAnalyzer' object has no attribute 'set_stroke_scales'`).

- [x] **Step 3: Implement adaptive stroke filtering in `surgical_motion_analyzer.py`**

Add:
- Attributes `self.stroke_scale = [1.0, 1.0, 1.0]`
- Power-law non-linear transient boost:
  $$a_{eff} = \text{sign}(a_w) \cdot \max(0, |a_w| - a_{thresh})^{1.1} \cdot K_{axis}$$
- Jerk-assisted stroke latching: hold stroke active when $|\dot{a}| > 0.8\text{ m/s}^3$ even during zero-crossings.
- Methods `set_stroke_scales(sx, sy, sz)` and `get_stroke_scales()`.

- [x] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest "/home/rifai/Imu_MOTION TRACKER/STM32_IMU/tools/test_motion_analyzer.py"`
Expected: PASS (All tests pass).

---

### Task 3: Dual-View MJPEG Streaming & Telemetry Fusion in `imu_web_server.py`

**Files:**
- Modify: `STM32_IMU/tools/imu_web_server.py`

**Interfaces:**
- Consumes: `CameraGroundTruthTracker` from `camera_ground_truth.py`.
- Produces:
  - `GET /video_feed`: Streaming `multipart/x-mixed-replace` JPEG response.
  - `GET /aruco_marker.png`: PNG image response.
  - `POST /api/tare_both`: Simultaneously zeroes IMU origin and Camera origin.
  - `POST /api/set_stroke_scales`: Updates $(K_x, K_y, K_z)$ live.
  - WebSocket `/ws`: Broadcasts `camera` telemetry object alongside `imu` object.

- [ ] **Step 1: Write integration test for web server endpoints**

Create `STM32_IMU/tools/test_web_server_endpoints.py` using `fastapi.testclient.TestClient` to verify `/aruco_marker.png` returns status 200 and image/png, and `/api/tare_both` returns status 200.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest "/home/rifai/Imu_MOTION TRACKER/STM32_IMU/tools/test_web_server_endpoints.py"`
Expected: FAIL (endpoints 404).

- [ ] **Step 3: Implement endpoints and background camera thread in `imu_web_server.py`**

- Initialize `CameraGroundTruthTracker(video_source=0)` in background daemon.
- Implement `/video_feed` streaming generator yielding `b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'`.
- Implement `/aruco_marker.png`, `/api/tare_both`, `/api/set_stroke_scales`.
- Inject `camera_data` into WebSocket broadcast payload.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest "/home/rifai/Imu_MOTION TRACKER/STM32_IMU/tools/test_web_server_endpoints.py"`
Expected: PASS.

---

### Task 4: Dual-View Web Visualizer (PiP Camera Feed, Comparison HUD, Marker Modal)

**Files:**
- Modify: `STM32_IMU/tools/web_imu_visualizer.html`

**Interfaces:**
- Consumes: `/video_feed`, WebSocket `data.camera`, `/api/tare_both`.
- Produces:
  - Picture-in-Picture draggable/collapsible camera window.
  - Camera View toggle button (header and shortcut key 'C').
  - Ground-truth comparison HUD: Live table of IMU vs Cam vs Error.
  - "Show ArUco Marker" modal with printable/scannable QR/ArUco ID 0.
  - "Tare Both (Zero Origin)" button.

- [ ] **Step 1: Add PiP Camera View Container and Comparison HUD to HTML**

Insert floating card `#camera-pip-panel` containing:
- `<img id="camera-stream" src="/video_feed" />`
- HUD overlay with optical status badge (`OPTICAL LOCK` / `MARKER SEARCHING`).
- Error table comparing IMU cm vs Cam cm.
- Toggle button `#btn-toggle-cam` in header bar.

- [ ] **Step 2: Add ArUco Marker Modal to HTML**

Insert `#marker-modal` with high-resolution image of `/aruco_marker.png` and dimensions instruction (40 mm x 40 mm).

- [ ] **Step 3: Update JavaScript WebSocket handler and controls**

- Handle `data.camera`: update HUD elements (`#cam-pos-x`, `#cam-pos-y`, `#cam-pos-z`, `#cam-delta-x`, etc.).
- Bind button `#btn-tare-both` to call `/api/tare_both` and reset local Three.js `tarePos`.
- Bind keyboard shortcut 'C' to toggle camera panel visibility.

- [ ] **Step 4: Verify in headless/syntax check and test browser rendering**

Run syntax check and verify HTML DOM elements with Python test.

---

### Task 5: End-to-End Live Verification & Calibration Test

**Files:**
- Execute: `STM32_IMU/tools/imu_web_server.py`
- Test: Verify live stream on `http://localhost:8080` with `/dev/ttyUSB0` and `/dev/video0`.

- [ ] **Step 1: Restart `imu_web_server.py` with camera and IMU streaming**
- [ ] **Step 2: Query `/video_feed` and verify live MJPEG header and frame delivery**
- [ ] **Step 3: Query WebSocket stream and verify live `imu` and `camera` telemetry packets**
- [ ] **Step 4: Perform live 30 cm vertical lift test, observe camera ground truth vs IMU, verify synchronized 1:1 motion**
