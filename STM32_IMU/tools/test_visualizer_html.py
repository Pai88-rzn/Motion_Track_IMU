#!/usr/bin/env python3
"""
DOM structure and script verification test for Dual-View Web Visualizer in web_imu_visualizer.html.
Verifies Picture-in-Picture camera feed, Comparison HUD, Header Controls, and ArUco Marker Modal.
"""

import os
import re
import pytest
from bs4 import BeautifulSoup

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(SCRIPT_DIR, "web_imu_visualizer.html")


@pytest.fixture(scope="module")
def html_soup():
    """Load and parse web_imu_visualizer.html with BeautifulSoup."""
    assert os.path.exists(HTML_PATH), f"File not found: {HTML_PATH}"
    with open(HTML_PATH, "r", encoding="utf-8") as f:
        content = f.read()
    soup = BeautifulSoup(content, "html.parser")
    return soup, content


def test_header_controls_exist(html_soup):
    """Verify header bar contains Camera View toggle, Tare Both, and ArUco Marker buttons."""
    soup, _ = html_soup
    
    # Toggle camera panel button
    btn_toggle_cam = soup.find(id="btn-toggle-cam")
    assert btn_toggle_cam is not None, "Missing #btn-toggle-cam in header"
    assert "Camera" in btn_toggle_cam.get_text()

    # Tare Both button
    btn_tare_both = soup.find(id="btn-tare-both")
    assert btn_tare_both is not None, "Missing #btn-tare-both in header"
    assert "Tare Both" in btn_tare_both.get_text()

    # ArUco Marker button
    btn_show_marker = soup.find(id="btn-show-marker")
    assert btn_show_marker is not None, "Missing #btn-show-marker in header"
    assert "Marker" in btn_show_marker.get_text()


def test_camera_pip_panel_structure(html_soup):
    """Verify Picture-in-Picture floating panel with stream img, status badge, and HUD."""
    soup, _ = html_soup

    pip_panel = soup.find(id="camera-pip-panel")
    assert pip_panel is not None, "Missing #camera-pip-panel container"

    # Live video stream img element
    img_stream = pip_panel.find("img", id="camera-stream")
    assert img_stream is not None, "Missing #camera-stream img in #camera-pip-panel"
    assert img_stream.get("src") == "/video_feed", f"Expected /video_feed src, got {img_stream.get('src')}"

    # Optical status indicator badge
    badge = pip_panel.find(id="optical-status-badge")
    assert badge is not None, "Missing #optical-status-badge in #camera-pip-panel"

    # Minimizing or toggle button
    btn_min = pip_panel.find(id="pip-minimize-btn") or pip_panel.find(id="pip-toggle-btn")
    assert btn_min is not None, "Missing minimize/toggle button in #camera-pip-panel"


def test_ground_truth_comparison_hud(html_soup):
    """Verify HUD comparison elements for IMU Pos, Camera Ground Truth Pos, and Tracking Error/Delta."""
    soup, _ = html_soup

    # Camera ground-truth coordinate display elements
    cam_x = soup.find(id="cam-pos-x")
    cam_y = soup.find(id="cam-pos-y")
    cam_z = soup.find(id="cam-pos-z")
    assert cam_x is not None and cam_y is not None and cam_z is not None, \
        "Missing #cam-pos-x, #cam-pos-y, or #cam-pos-z elements in HUD"

    # Camera relative delta or tracking error elements
    cam_dx = soup.find(id="cam-delta-x")
    cam_dy = soup.find(id="cam-delta-y")
    cam_dz = soup.find(id="cam-delta-z")
    assert cam_dx is not None and cam_dy is not None and cam_dz is not None, \
        "Missing #cam-delta-x, #cam-delta-y, or #cam-delta-z elements in HUD"

    # Camera mounting selector
    mount_select = soup.find("select", id="camera-mount-select")
    assert mount_select is not None, "Missing #camera-mount-select in HUD header"
    options = [opt.get("value") for opt in mount_select.find_all("option")]
    assert "frontal" in options and "overhead" in options, \
        f"Expected 'frontal' and 'overhead' options, got {options}"


def test_aruco_marker_modal(html_soup):
    """Verify ArUco Marker modal with printable/scannable marker image and instructions."""
    soup, _ = html_soup

    modal = soup.find(id="marker-modal")
    assert modal is not None, "Missing #marker-modal container"

    img = modal.find("img")
    assert img is not None, "Missing img in #marker-modal"
    assert "/aruco_marker.png" in img.get("src"), f"Expected /aruco_marker.png in src, got {img.get('src')}"

    modal_text = modal.get_text()
    assert "DICT_4X4_50" in modal_text or "40x40" in modal_text or "40 mm" in modal_text, \
        "Missing ArUco dictionary or physical dimension instructions in #marker-modal"

    close_btn = modal.find(id="marker-modal-close") or modal.find(id="btn-close-marker")
    assert close_btn is not None, "Missing close button in #marker-modal"


def test_javascript_handlers_present(html_soup):
    """Verify JavaScript includes websocket handling for data.camera, tare_both action, and 'C' shortcut."""
    _, content = html_soup

    # Shortcut 'c' or 'C' key handler
    assert re.search(r"['\"]c['\"]|['\"]C['\"]", content), "Missing 'c' / 'C' shortcut key listener in JS"

    # tare_both websocket/api call
    assert "tare_both" in content, "Missing 'tare_both' WebSocket action or API call in script"

    # data.camera handling
    assert "data.camera" in content or "latestData.camera" in content, \
        "Missing handling for data.camera in script"

    # Camera-to-world axis mapping
    assert "delta_world" in content, "Missing delta_world camera-to-world axis mapping in script"
    assert "frontal" in content and "overhead" in content, "Missing frontal and overhead mount branches in script"


def test_camera_analyzer_tracking_box_elements(html_soup):
    """Verify hardware black block tracking controls (#btn-kunci-balok, #btn-reset-balok) and click handler."""
    soup, content = html_soup

    # Toolbar action buttons
    btn_kunci = soup.find(id="btn-kunci-balok")
    assert btn_kunci is not None, "Missing #btn-kunci-balok in camera analyzer toolbar"
    assert "Kunci Balok" in btn_kunci.get_text()

    btn_reset = soup.find(id="btn-reset-balok")
    assert btn_reset is not None, "Missing #btn-reset-balok in camera analyzer toolbar"
    assert "Reset Box" in btn_reset.get_text()

    # JS click-to-lock and endpoints
    assert "/api/set_tracking_roi" in content, "Missing /api/set_tracking_roi call in visualizer script"
    assert "/api/reset_tracker" in content, "Missing /api/reset_tracker call in visualizer script"
