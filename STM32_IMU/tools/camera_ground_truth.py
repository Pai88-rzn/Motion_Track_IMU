#!/usr/bin/env python3
"""
Optical Ground-Truth Engine & ArUco Pose Tracker.

Tracks physical 3D displacement (6DOF pose) using ArUco fiducial markers (DICT_4X4_50)
to serve as an optical ground-truth reference for surgical IMU filter calibration.
Supports both OpenCV 4.5.x and OpenCV 4.7+ API variants.
"""

import sys
import os
import glob
import time
import logging
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


def draw_analyzer_box(
    image: np.ndarray,
    bbox: Tuple[int, int, int, int],
    label: str = "IMU BALOK HITAM",
    color: Tuple[int, int, int] = (0, 240, 255),
    is_locked: bool = True,
):
    """
    Draws a high-visibility futuristic analyzer tracking box with corner brackets,
    center crosshair, and label pill directly on the camera frame.
    """
    x, y, w, h = [int(v) for v in bbox]
    h_img, w_img = image.shape[:2]
    x = max(0, min(w_img - 1, x))
    y = max(0, min(h_img - 1, y))
    w = max(4, min(w_img - x, w))
    h = max(4, min(h_img - y, h))

    cx, cy = x + w // 2, y + h // 2
    arm = max(8, min(24, min(w, h) // 4))
    thickness = 2 if is_locked else 1

    # Corner brackets (L-shaped corners)
    cv2.line(image, (x, y), (x + arm, y), color, thickness)
    cv2.line(image, (x, y), (x, y + arm), color, thickness)
    cv2.line(image, (x + w, y), (x + w - arm, y), color, thickness)
    cv2.line(image, (x + w, y), (x + w, y + arm), color, thickness)
    cv2.line(image, (x, y + h), (x + arm, y + h), color, thickness)
    cv2.line(image, (x, y + h), (x, y + h - arm), color, thickness)
    cv2.line(image, (x + w, y + h), (x + w - arm, y + h), color, thickness)
    cv2.line(image, (x + w, y + h), (x + w, y + h - arm), color, thickness)

    # Subtle bounding box outline
    cv2.rectangle(image, (x, y), (x + w, y + h), color, 1, cv2.LINE_AA)

    # Center crosshair
    ch = 8
    cv2.line(image, (cx - ch, cy), (cx + ch, cy), color, 1)
    cv2.line(image, (cx, cy - ch), (cx, cy + ch), color, 1)
    cv2.circle(image, (cx, cy), 3, (0, 255, 128) if is_locked else (0, 200, 255), -1)

    # Label tag banner
    tag_text = f"[{label}]"
    (tw, th), _ = cv2.getTextSize(tag_text, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
    tag_y = max(th + 6, y - 6)
    cv2.rectangle(image, (x, tag_y - th - 4), (x + tw + 8, tag_y + 2), (18, 22, 32), -1)
    cv2.rectangle(image, (x, tag_y - th - 4), (x + tw + 8, tag_y + 2), color, 1)
    cv2.putText(image, tag_text, (x + 4, tag_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1, cv2.LINE_AA)


class IMUEnclosureDetector:
    """
    Trained multi-view template and feature detector for the physical
    black IMU enclosure ('IMU CPRP').
    Trained on multi-view reference images:
      - Front textured face
      - Vertical face with embossed 'IMU CPRP' text
      - Side edge with horizontal ridges & green LED
      - End cross-section face
      - Back plate with mounting ears
    """

    def __init__(self, template_dir: Optional[str] = None):
        if template_dir is None:
            curr_dir = os.path.dirname(os.path.abspath(__file__))
            template_dir = os.path.join(curr_dir, "models", "imu_templates")

        self.template_dir = template_dir
        self.templates: Dict[str, Dict] = {}
        self.load_templates()

    def load_templates(self):
        """Loads and pre-processes trained reference templates."""
        if not os.path.exists(self.template_dir):
            return

        for f in glob.glob(os.path.join(self.template_dir, "*.png")):
            if "test_" in f or "_crop" in f:
                continue
            base = os.path.basename(f).replace(".png", "")
            img = cv2.imread(f)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.templates[base] = {
                "gray": gray,
                "w": img.shape[1],
                "h": img.shape[0],
                "aspect": float(img.shape[1]) / max(1, img.shape[0]),
            }

    def detect(
        self, frame: np.ndarray, min_score: float = 0.80
    ) -> Tuple[Optional[Tuple[int, int, int, int]], Optional[str], float]:
        """Scans the frame against the trained multi-view template bank."""
        if not self.templates or frame is None or frame.size == 0 or np.mean(frame) < 8:
            return None, None, 0.0

        h_f, w_f = frame.shape[:2]
        gray_f = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        scale_scan = 360.0 / max(h_f, 1)
        sw_f, sh_f = int(w_f * scale_scan), int(h_f * scale_scan)
        small_gray = cv2.resize(gray_f, (sw_f, sh_f))

        best_match = None
        best_val = -1.0
        best_box = None

        for tname, tinfo in self.templates.items():
            tmpl_gray = tinfo["gray"]
            th, tw = tmpl_gray.shape[:2]
            for scale in [0.35, 0.5, 0.7, 0.9, 1.1]:
                stw, sth = int(tw * scale_scan * scale), int(th * scale_scan * scale)
                if stw >= sw_f - 10 or sth >= sh_f - 10 or stw < 20 or sth < 20:
                    continue
                small_tmpl = cv2.resize(tmpl_gray, (stw, sth))
                res = cv2.matchTemplate(small_gray, small_tmpl, cv2.TM_CCOEFF_NORMED)
                _, max_v, _, max_loc = cv2.minMaxLoc(res)
                if max_v > best_val:
                    best_val = max_v
                    best_match = tname
                    orig_x = int(max_loc[0] / scale_scan)
                    orig_y = int(max_loc[1] / scale_scan)
                    orig_w = int(stw / scale_scan)
                    orig_h = int(sth / scale_scan)
                    best_box = (orig_x, orig_y, orig_w, orig_h)

        if best_val >= min_score and best_box is not None:
            bx, by, bw, bh = best_box
            bx = max(0, min(w_f - 10, bx))
            by = max(0, min(h_f - 10, by))
            bw = max(10, min(w_f - bx, bw))
            bh = max(10, min(h_f - by, bh))
            roi = frame[by:by + bh, bx:bx + bw]
            if roi.size > 0:
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mean_v = np.mean(hsv_roi[:, :, 2])
                mean_s = np.mean(hsv_roi[:, :, 1])
                # Physical IMU block is dark matte black (mean V < 125, mean S < 140)
                if mean_v < 125 and mean_s < 140:
                    clean_name = (
                        best_match.split("_", 1)[-1].replace("_", " ")
                        if "_" in best_match
                        else best_match
                    )
                    return (bx, by, bw, bh), clean_name, best_val

        return None, None, best_val


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

        # Optical object tracker for physical IMU enclosure (black block)
        self.object_tracker = None
        self.object_bbox: Optional[Tuple[int, int, int, int]] = None
        self.tracking_mode: str = "auto"  # "aruco", "block", "reticle"
        self._pending_roi: Optional[Tuple[int, int, int, int]] = None
        self.enclosure_detector = IMUEnclosureDetector()
        self.last_detected_view: str = "IMU CPRP"

        # Threading and synchronization
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cap: Optional[cv2.VideoCapture] = None

        # Telemetry & Frame caching
        self.latest_data: Dict = {
            "detected": False,
            "mode": "searching",
            "pos_cm": [0.0, 0.0, 0.0],
            "delta_cm": [0.0, 0.0, 0.0],
            "fps": 0.0,
            "marker_id": self.target_marker_id,
            "timestamp": 0.0,
            "status": "ready",
            "bbox": None,
        }
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_jpeg: bytes = b""

        # FPS calculation state
        self._last_frame_time = 0.0
        self._measured_fps = 0.0

        # Pre-encode placeholder frame for offline / waiting state
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
        _, enc_placeholder = cv2.imencode(".jpg", placeholder, [cv2.IMWRITE_JPEG_QUALITY, 80])
        self._cached_placeholder_jpeg: bytes = enc_placeholder.tobytes()

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
            if self.latest_data.get("detected", False) and self._last_pos is not None:
                self.tare_origin = np.copy(self._last_pos)
                self._tare_requested = False
            else:
                self._tare_requested = True
                self.tare_origin = None

            self.latest_data["delta_cm"] = [0.0, 0.0, 0.0]

    def set_tracking_roi(self, bbox: Tuple[int, int, int, int]):
        """Sets or locks the tracking ROI box to track the physical black block."""
        with self._lock:
            self._pending_roi = (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
            self._tare_requested = True

    def reset_tracker(self):
        """Resets object tracking back to searching/center target reticle."""
        with self._lock:
            self.object_tracker = None
            self.object_bbox = None
            self._pending_roi = None
            self.tare_origin = None
            self._tare_requested = True
            self.last_detected_view = "IMU CPRP"

    def auto_detect_black_block(
        self, frame: np.ndarray, require_center: bool = False
    ) -> Optional[Tuple[int, int, int, int]]:
        """Detects prominent dark rectangular block of the IMU enclosure in the frame."""
        if frame is None or frame.size == 0 or np.mean(frame) < 8:
            return None

        # 1. First scan using trained multi-view template detector
        if hasattr(self, "enclosure_detector") and self.enclosure_detector is not None:
            box, view_name, score = self.enclosure_detector.detect(frame, min_score=0.80)
            if box is not None:
                self.last_detected_view = view_name or "IMU CPRP"
                return box

        # 2. Geometric dark contour fallback
        h, w = frame.shape[:2]
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        margin_x, margin_y = int(w * 0.05), int(h * 0.05)
        roi_mask[margin_y:h - margin_y, margin_x:w - margin_x] = 255

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 95]))
        mask = cv2.bitwise_and(mask, roi_mask)

        # Exclude skin color
        skin = cv2.inRange(hsv, np.array([0, 30, 55]), np.array([28, 200, 255]))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(skin))

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        max_dist = min(w, h) * 0.35 if require_center else 999999.0

        for c in contours:
            area = cv2.contourArea(c)
            if 400 < area < 350000:
                bx, by, bw, bh = cv2.boundingRect(c)
                aspect = float(bw) / max(1, bh)
                if 0.20 < aspect < 6.5:
                    cx = bx + bw / 2.0
                    cy = by + bh / 2.0
                    dist = np.hypot(cx - w / 2.0, cy - h / 2.0)
                    if not require_center or dist <= max_dist:
                        candidates.append((dist, (bx, by, bw, bh), area))

        if candidates:
            candidates.sort(key=lambda item: item[0])
            self.last_detected_view = "IMU CPRP"
            return candidates[0][1]
        return None

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

        # 0. Check if a new tracking ROI was requested by user click
        with self._lock:
            pending_roi = self._pending_roi
            self._pending_roi = None

        if pending_roi is not None:
            try:
                self.object_tracker = cv2.TrackerCSRT_create()
                self.object_tracker.init(frame, pending_roi)
                self.object_bbox = pending_roi
            except Exception as e:
                logging.debug(f"Failed to initialize CSRT tracker: {e}")
                self.object_tracker = None

        corners, ids, _ = self._detect_markers(gray)

        detected = False
        tracking_mode = "searching"
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

                    with self._lock:
                        self._last_pos = np.copy(pos)

                        # Auto-tare on first detection or if tare was requested
                        if self.tare_origin is None or self._tare_requested:
                            self.tare_origin = np.copy(pos)
                            self._tare_requested = False

                        delta = pos - self.tare_origin
                        delta_cm = [round(float(delta[0]), 2), round(float(delta[1]), 2), round(float(delta[2]), 2)]

                    pos_cm = [round(float(pos[0]), 2), round(float(pos[1]), 2), round(float(pos[2]), 2)]
                    detected = True
                    tracking_mode = "aruco"

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
                    import logging
                    logging.debug(f"Optical pose estimation failed: {e}")
                    detected = False

        # If ArUco not found, use Hardware Analyzer Tracking Box (Balok Hitam IMU)
        if not detected:
            if self.object_tracker is not None:
                try:
                    success, box = self.object_tracker.update(frame)
                    if success:
                        bx, by, bw, bh = [int(v) for v in box]
                        self.object_bbox = (bx, by, bw, bh)
                        cx = bx + bw / 2.0
                        cy = by + bh / 2.0
                        # Pinhole depth estimate from known physical dimension (4.0 cm)
                        est_z = (self.camera_matrix[0, 0] * self.marker_size_cm) / max(10, max(bw, bh))
                        est_x = (cx - self.camera_matrix[0, 2]) * est_z / self.camera_matrix[0, 0]
                        est_y = (cy - self.camera_matrix[1, 2]) * est_z / self.camera_matrix[1, 1]
                        pos = np.array([est_x, est_y, est_z], dtype=float)

                        with self._lock:
                            self._last_pos = np.copy(pos)
                            if self.tare_origin is None or self._tare_requested:
                                self.tare_origin = np.copy(pos)
                                self._tare_requested = False
                            delta = pos - self.tare_origin
                            delta_cm = [round(float(delta[0]), 2), round(float(delta[1]), 2), round(float(delta[2]), 2)]

                        pos_cm = [round(float(pos[0]), 2), round(float(pos[1]), 2), round(float(pos[2]), 2)]
                        detected = True
                        tracking_mode = "block"

                        label_name = f"ANALYZER: {self.last_detected_view.upper()} (LOCKED)"
                        draw_analyzer_box(
                            annotated,
                            (bx, by, bw, bh),
                            label=label_name,
                            color=(0, 255, 128),
                            is_locked=True,
                        )

                        overlay = annotated.copy()
                        cv2.rectangle(overlay, (8, 8), (340, 85), (20, 20, 20), -1)
                        cv2.addWeighted(overlay, 0.75, annotated, 0.25, 0, annotated)
                        cv2.rectangle(annotated, (8, 8), (340, 85), (0, 255, 128), 1)

                        cv2.putText(
                            annotated,
                            f"OPTICAL TRACKER: {self.last_detected_view.upper()}",
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
                    else:
                        self.object_tracker = None
                except Exception as e:
                    logging.debug(f"Object tracking error: {e}")
                    self.object_tracker = None

            if not detected:
                auto_box = self.auto_detect_black_block(frame)
                if auto_box is not None:
                    try:
                        self.object_tracker = cv2.TrackerCSRT_create()
                        self.object_tracker.init(frame, auto_box)
                        self.object_bbox = auto_box
                        bx, by, bw, bh = auto_box
                        cx = bx + bw / 2.0
                        cy = by + bh / 2.0
                        est_z = (self.camera_matrix[0, 0] * self.marker_size_cm) / max(10, max(bw, bh))
                        est_x = (cx - self.camera_matrix[0, 2]) * est_z / self.camera_matrix[0, 0]
                        est_y = (cy - self.camera_matrix[1, 2]) * est_z / self.camera_matrix[1, 1]
                        pos = np.array([est_x, est_y, est_z], dtype=float)

                        with self._lock:
                            self._last_pos = np.copy(pos)
                            if self.tare_origin is None or self._tare_requested:
                                self.tare_origin = np.copy(pos)
                                self._tare_requested = False
                            delta = pos - self.tare_origin
                            delta_cm = [round(float(delta[0]), 2), round(float(delta[1]), 2), round(float(delta[2]), 2)]

                        pos_cm = [round(float(pos[0]), 2), round(float(pos[1]), 2), round(float(pos[2]), 2)]
                        detected = True
                        tracking_mode = "block"

                        label_name = f"ANALYZER: {self.last_detected_view.upper()} (ACQUIRED)"
                        draw_analyzer_box(
                            annotated,
                            auto_box,
                            label=label_name,
                            color=(0, 240, 255),
                            is_locked=True,
                        )
                    except Exception as e:
                        logging.debug(f"Auto-detect init failed: {e}")

                if not detected:
                    # Draw Center Analyzer Target Reticle
                    cw = self.width // 2
                    ch = self.height // 2
                    target_w = 150
                    target_h = 150
                    center_box = (cw - target_w // 2, ch - target_h // 2, target_w, target_h)
                    draw_analyzer_box(
                        annotated,
                        center_box,
                        label="ANALYZER TARGET (ARAHKAN BALOK HITAM)",
                        color=(0, 200, 255),
                        is_locked=False,
                    )

                    overlay = annotated.copy()
                    cv2.rectangle(overlay, (8, 8), (380, 48), (20, 20, 20), -1)
                    cv2.addWeighted(overlay, 0.75, annotated, 0.25, 0, annotated)
                    cv2.rectangle(annotated, (8, 8), (380, 48), (60, 160, 255), 1)
                    cv2.putText(
                        annotated,
                        "Arahkan Balok Hitam ke Kotak Tengah / Klik Video",
                        (16, 32),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.40,
                        (100, 220, 255),
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
            "mode": tracking_mode,
            "view": self.last_detected_view if detected else None,
            "pos_cm": pos_cm,
            "delta_cm": delta_cm,
            "fps": round(self._measured_fps, 1),
            "marker_id": self.target_marker_id,
            "timestamp": now,
            "status": "active" if detected else "searching",
            "bbox": list(self.object_bbox) if (detected and self.object_bbox) else None,
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
        If no frame is available yet, returns pre-encoded placeholder frame.
        """
        with self._lock:
            if self.latest_jpeg:
                return self.latest_jpeg

        return self._cached_placeholder_jpeg

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
                            "mode": "offline",
                            "pos_cm": [0.0, 0.0, 0.0],
                            "delta_cm": [0.0, 0.0, 0.0],
                            "fps": 0.0,
                            "marker_id": self.target_marker_id,
                            "timestamp": time.time(),
                            "status": "offline",
                            "bbox": None,
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
