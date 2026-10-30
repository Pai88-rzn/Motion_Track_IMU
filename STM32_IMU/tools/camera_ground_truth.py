#!/usr/bin/env python3
"""
Optical Ground-Truth Engine & ArUco Pose Tracker.

Tracks physical 3D displacement (6DOF pose) using ArUco fiducial markers (DICT_4X4_50)
to serve as an optical ground-truth reference for surgical IMU filter calibration.
Supports both OpenCV 4.5.x and OpenCV 4.7+ API variants.
"""

import sys
import time
import threading
from typing import Dict, Optional, Tuple, Union

import cv2
import numpy as np


def _get_aruco_dictionary(dict_id: int = cv2.aruco.DICT_4X4_50):
    """Retrieve ArUco dictionary compatible with OpenCV 4.5 and 4.7+."""
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def _get_detector_parameters():
    """Retrieve ArUco detector parameters compatible with OpenCV 4.5 and 4.7+."""
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    elif hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return None


def generate_aruco_marker_png(marker_id: int = 0, size_px: int = 400) -> bytes:
    """
    Generate an ArUco marker image encoded as PNG bytes.

    Args:
        marker_id: ArUco marker ID (default 0).
        size_px: Image width and height in pixels (default 400).

    Returns:
        bytes: Raw PNG image data.
    """
    dictionary = _get_aruco_dictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "drawMarker"):
        marker_img = cv2.aruco.drawMarker(dictionary, marker_id, size_px)
    elif hasattr(cv2.aruco, "generateImageMarker"):
        marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    else:
        raise RuntimeError("No compatible ArUco marker generator found in cv2.aruco.")

    # Encode to PNG
    success, buffer = cv2.imencode(".png", marker_img)
    if not success:
        raise RuntimeError("Failed to encode ArUco marker to PNG.")
    return buffer.tobytes()


class CameraGroundTruthTracker:
    """
    Thread-safe optical pose tracker using ArUco fiducial markers.

    Provides real-time 6DOF position estimation [X, Y, Z] relative to camera
    and tared displacement [dX, dY, dZ] in centimeters.
    """

    def __init__(
        self,
        video_source: Union[int, str] = 0,
        marker_size_cm: float = 4.0,
        target_fps: int = 30,
        width: int = 640,
        height: int = 480,
        marker_id: int = 0,
    ):
        """
        Initialize the ground truth tracker.

        Args:
            video_source: Camera index (int e.g. 0) or device path (str e.g. '/dev/video0').
                          Use -1 for manual / test mode without starting camera thread.
            marker_size_cm: Physical side length of ArUco marker in centimeters (default 4.0 cm).
            target_fps: Target frame rate for camera capture thread (default 30).
            width: Expected frame width in pixels (default 640).
            height: Expected frame height in pixels (default 480).
            marker_id: Target ArUco marker ID to track (default 0).
        """
        self.video_source = video_source
        self.marker_size_cm = float(marker_size_cm)
        self.target_fps = int(target_fps)
        self.width = int(width)
        self.height = int(height)
        self.target_marker_id = int(marker_id)

        # ArUco setup
        self.dictionary = _get_aruco_dictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = _get_detector_parameters()

        # OpenCV 4.7+ ArucoDetector if available
        self.aruco_detector = None
        if hasattr(cv2.aruco, "ArucoDetector") and self.parameters is not None:
            try:
                self.aruco_detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            except Exception:
                self.aruco_detector = None

        # Camera intrinsics (pinhole approximation if not calibrated)
        self._init_camera_matrix(self.width, self.height)

        # Pose & Tare tracking state
        self.tare_origin: Optional[np.ndarray] = None
        self._tare_requested: bool = False
        self._last_pos: Optional[np.ndarray] = None

        # Threading and synchronization
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cap: Optional[cv2.VideoCapture] = None

        # Telemetry & Frame caching
        self.latest_data: Dict = {
            "detected": False,
            "pos_cm": [0.0, 0.0, 0.0],
            "delta_cm": [0.0, 0.0, 0.0],
            "fps": 0.0,
            "marker_id": self.target_marker_id,
            "timestamp": 0.0,
            "status": "ready",
        }
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_jpeg: bytes = b""

        # FPS calculation state
        self._last_frame_time = 0.0
        self._measured_fps = 0.0

    def _init_camera_matrix(self, width: int, height: int):
        """Initialize pinhole camera matrix with standard 60-degree FOV approximation."""
        fx = float(width)
        fy = float(width)
        cx = float(width) / 2.0
        cy = float(height) / 2.0
        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)

    def tare(self):
        """
        Set current 3D position as ground-truth origin (0, 0, 0).
        If marker is not currently detected, tare is applied on next valid detection.
        """
        with self._lock:
            if self._last_pos is not None:
                self.tare_origin = np.copy(self._last_pos)
                self._tare_requested = False
            else:
                self._tare_requested = True
                self.tare_origin = None

            self.latest_data["delta_cm"] = [0.0, 0.0, 0.0]

    def _detect_markers(self, gray_frame: np.ndarray):
        """Detect markers handling differences between OpenCV 4.5 and 4.7+."""
        if self.aruco_detector is not None:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray_frame)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray_frame, self.dictionary, parameters=self.parameters
            )
        return corners, ids, rejected

    def _estimate_pose(self, corner: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate 6DOF rvec and tvec for a single detected marker corner array."""
        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corner], self.marker_size_cm, self.camera_matrix, self.dist_coeffs
            )
            return rvecs[0][0], tvecs[0][0]
        else:
            # Fallback to solvePnP
            half_s = self.marker_size_cm / 2.0
            obj_points = np.array([
                [-half_s,  half_s, 0.0],
                [ half_s,  half_s, 0.0],
                [ half_s, -half_s, 0.0],
                [-half_s, -half_s, 0.0]
            ], dtype=np.float32)
            img_points = corner.reshape(4, 2).astype(np.float32)
            _, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            return rvec.flatten(), tvec.flatten()

    def _draw_axes(self, image: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, length: float):
        """Draw 3D coordinate triad on image."""
        if hasattr(cv2, "drawFrameAxes"):
            cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, length)
        elif hasattr(cv2.aruco, "drawAxis"):
            cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, length)

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """
        Process a single image frame: detects marker, estimates pose, draws overlay.

        Args:
            frame: BGR input image (numpy.ndarray).

        Returns:
            Tuple[np.ndarray, Dict]: (annotated_frame, telemetry_dict)
        """
        if frame is None or frame.size == 0:
            return frame, self.get_latest_data()

        # Update camera matrix if frame dimensions differ from initialized
        h, w = frame.shape[:2]
        if w != self.width or h != self.height:
            self.width = w
            self.height = h
            self._init_camera_matrix(w, h)

        annotated = frame.copy()
        if len(annotated.shape) == 2:
            annotated = cv2.cvtColor(annotated, cv2.COLOR_GRAY2BGR)

        gray = cv2.cvtColor(annotated, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)

        detected = False
        pos_cm = [0.0, 0.0, 0.0]
        delta_cm = [0.0, 0.0, 0.0]

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            target_indices = np.where(ids_flat == self.target_marker_id)[0]

            if len(target_indices) > 0:
                idx = target_indices[0]
                target_corner = corners[idx]

                try:
                    rvec, tvec = self._estimate_pose(target_corner)
                    pos = np.array([tvec[0], tvec[1], tvec[2]], dtype=float)
                    self._last_pos = np.copy(pos)

                    # Auto-tare on first detection or if tare was requested
                    if self.tare_origin is None or self._tare_requested:
                        self.tare_origin = np.copy(pos)
                        self._tare_requested = False

                    delta = pos - self.tare_origin

                    pos_cm = [round(float(pos[0]), 2), round(float(pos[1]), 2), round(float(pos[2]), 2)]
                    delta_cm = [round(float(delta[0]), 2), round(float(delta[1]), 2), round(float(delta[2]), 2)]
                    detected = True

                    # 1. Draw marker outline
                    cv2.aruco.drawDetectedMarkers(annotated, [target_corner], np.array([[self.target_marker_id]]))

                    # 2. Draw 3D axes at marker center (axis length = 75% marker size)
                    axis_length = max(1.0, self.marker_size_cm * 0.75)
                    self._draw_axes(annotated, rvec, tvec, axis_length)

                    # 3. HUD overlay with pose and displacement
                    overlay = annotated.copy()
                    cv2.rectangle(overlay, (8, 8), (340, 85), (20, 20, 20), -1)
                    cv2.addWeighted(overlay, 0.75, annotated, 0.25, 0, annotated)
                    cv2.rectangle(annotated, (8, 8), (340, 85), (0, 220, 100), 1)

                    cv2.putText(
                        annotated,
                        f"OPTICAL GROUND TRUTH (ArUco ID #{self.target_marker_id})",
                        (16, 26),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        (0, 255, 128),
                        1,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        annotated,
                        f"POS: X:{pos_cm[0]:+6.1f}  Y:{pos_cm[1]:+6.1f}  Z:{pos_cm[2]:+6.1f} cm",
                        (16, 48),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        annotated,
                        f"REL: dX:{delta_cm[0]:+5.1f}  dY:{delta_cm[1]:+5.1f}  dZ:{delta_cm[2]:+5.1f} cm",
                        (16, 70),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 240, 255),
                        1,
                        cv2.LINE_AA,
                    )

                except Exception as e:
                    detected = False

        if not detected:
            overlay = annotated.copy()
            cv2.rectangle(overlay, (8, 8), (320, 48), (20, 20, 20), -1)
            cv2.addWeighted(overlay, 0.75, annotated, 0.25, 0, annotated)
            cv2.rectangle(annotated, (8, 8), (320, 48), (60, 60, 200), 1)
            cv2.putText(
                annotated,
                f"Searching for ArUco Marker #{self.target_marker_id}...",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (100, 150, 255),
                1,
                cv2.LINE_AA,
            )

        # Update FPS
        now = time.time()
        if self._last_frame_time > 0:
            dt = now - self._last_frame_time
            if dt > 0:
                inst_fps = 1.0 / dt
                self._measured_fps = 0.9 * self._measured_fps + 0.1 * inst_fps if self._measured_fps > 0 else inst_fps
        self._last_frame_time = now

        # Encode JPEG representation
        _, enc_jpg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        jpeg_bytes = enc_jpg.tobytes()

        data_dict = {
            "detected": detected,
            "pos_cm": pos_cm,
            "delta_cm": delta_cm,
            "fps": round(self._measured_fps, 1),
            "marker_id": self.target_marker_id,
            "timestamp": now,
            "status": "active" if detected else "searching",
        }

        with self._lock:
            self.latest_frame = annotated
            self.latest_jpeg = jpeg_bytes
            self.latest_data = data_dict

        return annotated, data_dict

    def get_latest_data(self) -> Dict:
        """Return a thread-safe copy of the latest optical tracking telemetry."""
        with self._lock:
            return dict(self.latest_data)

    def get_jpeg_frame(self) -> bytes:
        """
        Return the latest annotated frame encoded as JPEG bytes.
        If no frame is available yet, generates a placeholder frame.
        """
        with self._lock:
            if self.latest_jpeg:
                return self.latest_jpeg

        # Generate fallback placeholder frame
        placeholder = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            "Waiting for Camera Stream...",
            (40, self.height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (180, 180, 180),
            2,
            cv2.LINE_AA,
        )
        _, enc = cv2.imencode(".jpg", placeholder, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return enc.tobytes()

    def _capture_loop(self):
        """Background thread loop capturing frames and updating pose."""
        cap = None
        frame_interval = 1.0 / max(1, self.target_fps)

        while self._running:
            loop_start = time.time()

            if cap is None or not cap.isOpened():
                if isinstance(self.video_source, int) and self.video_source < 0:
                    time.sleep(0.1)
                    continue

                try:
                    # Prefer V4L2 on Linux for low latency
                    cap = cv2.VideoCapture(self.video_source, cv2.CAP_V4L2)
                    if not cap.isOpened():
                        cap = cv2.VideoCapture(self.video_source)
                except Exception:
                    cap = None

                if cap is not None and cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    cap.set(cv2.CAP_PROP_FPS, self.target_fps)
                else:
                    # Create offline placeholder
                    offline_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    cv2.putText(
                        offline_img,
                        f"Camera Offline ({self.video_source})",
                        (30, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 200),
                        2,
                        cv2.LINE_AA,
                    )
                    _, enc = cv2.imencode(".jpg", offline_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    with self._lock:
                        self.latest_frame = offline_img
                        self.latest_jpeg = enc.tobytes()
                        self.latest_data = {
                            "detected": False,
                            "pos_cm": [0.0, 0.0, 0.0],
                            "delta_cm": [0.0, 0.0, 0.0],
                            "fps": 0.0,
                            "marker_id": self.target_marker_id,
                            "timestamp": time.time(),
                            "status": "offline",
                        }
                    time.sleep(0.5)
                    continue

            ret, frame = cap.read()
            if not ret or frame is None:
                cap.release()
                cap = None
                time.sleep(0.2)
                continue

            self.process_frame(frame)

            elapsed = time.time() - loop_start
            sleep_time = max(0.001, frame_interval - elapsed)
            time.sleep(sleep_time)

        if cap is not None and cap.isOpened():
            cap.release()

    def start(self):
        """Start the background camera capture thread."""
        if self._running:
            return

        self._running = True
        # If video_source is negative, do not spin up camera capture thread
        if isinstance(self.video_source, int) and self.video_source < 0:
            return

        self._thread = threading.Thread(target=self._capture_loop, daemon=True, name="CameraTrackerThread")
        self._thread.start()

    def stop(self):
        """Stop the background camera capture thread and release camera."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Optical Ground-Truth Camera Tracker")
    parser.add_argument("--device", default=0, help="Camera device index or path (default: 0)")
    parser.add_argument("--marker-id", type=int, default=0, help="ArUco marker ID (default: 0)")
    parser.add_argument("--save-marker", type=str, default=None, help="Save marker image to file")
    args = parser.parse_args()

    if args.save_marker:
        png = generate_aruco_marker_png(marker_id=args.marker_id)
        with open(args.save_marker, "wb") as f:
            f.write(png)
        print(f"Saved marker ID {args.marker_id} to {args.save_marker}")
        sys.exit(0)

    try:
        dev = int(args.device)
    except ValueError:
        dev = args.device

    print(f"Starting Ground Truth Tracker on device {dev}...")
    tracker = CameraGroundTruthTracker(video_source=dev, marker_id=args.marker_id)
    tracker.start()

    try:
        while True:
            data = tracker.get_latest_data()
            print(
                f"\rStatus: {data['status']:10s} | Detected: {data['detected']} | "
                f"Pos: {data['pos_cm']} cm | Delta: {data['delta_cm']} cm | FPS: {data['fps']:.1f}",
                end="",
                flush=True,
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping tracker...")
    finally:
        tracker.stop()
