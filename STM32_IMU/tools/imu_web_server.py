#!/usr/bin/env python3
"""
Surgical Hand Motion & Repetition Tracker — Direct UART Web Analytics Server
=============================================================================
Communicates directly with STM32_IMU over USART1 via USB-Serial (/dev/ttyUSB0 @ 115200 baud).
Decodes 36-byte binary telemetry frames with CRC16, feeds the real-time
SurgicalMotionAnalyzer (3D bounded hand trajectory + repetition self-discovery),
and broadcasts live telemetry & analytics via WebSocket to the 3D Web Dashboard.
"""

import os
import sys
import math
import json
import time
import struct
import asyncio
import threading
from contextlib import asynccontextmanager
from typing import Set, Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse, StreamingResponse, Response
from pydantic import BaseModel, Field

# Ensure local imports from tools/ work regardless of current working directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from surgical_motion_analyzer import SurgicalMotionAnalyzer
from camera_ground_truth import CameraGroundTruthTracker, generate_aruco_marker_png

try:
    import serial
except ImportError:
    print("[ERROR] pyserial is required. Install via: pip install pyserial")
    sys.exit(1)

HTML_PATH = os.path.join(SCRIPT_DIR, "web_imu_visualizer.html")
LOG_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "logs")

# Global state
analyzer = SurgicalMotionAnalyzer(fs_hz=100.0, forearm_length_cm=26.0)
tracker = CameraGroundTruthTracker(video_source=0, marker_size_cm=4.0)
camera_tracker = tracker  # Interop alias


@asynccontextmanager
async def lifespan(app: FastAPI):
    tracker.start()
    yield
    tracker.stop()


app = FastAPI(
    title="Surgical Wrist IMU Hand Motion & Repetition Tracker",
    lifespan=lifespan
)

active_websockets: Set[WebSocket] = set()
loop: Optional[asyncio.AbstractEventLoop] = None
packet_count = 0
msg_rate_hz = 0.0
last_rate_calc_time = time.time()
last_rate_count = 0
serial_connected = False
serial_port_name = "/dev/ttyUSB0"
serial_baud_rate = 115200

latest_imu_data = {
    "type": "imu",
    "seq": 0,
    "ts": 0.0,
    "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "ax": 0.0, "ay": 0.0, "az": 0.0,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "rate_hz": 0.0,
    "analytics": analyzer.get_live_snapshot(),
    "camera": tracker.get_latest_data(),
}


def quat_to_euler_deg(w: float, x: float, y: float, z: float):
    """Calculates Roll, Pitch, Yaw in degrees from Quaternion (w, x, y, z)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def compute_crc16(data: bytes) -> int:
    """CRC-16-CCITT calculation matching STM32 firmware."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def process_imu_sample(
    ts: float,
    qw: float, qx: float, qy: float, qz: float,
    ax: float, ay: float, az: float,
    gx_deg: float, gy_deg: float, gz_deg: float,
    source: str = "uart"
) -> dict:
    """Runs sample through SurgicalMotionAnalyzer and updates global state."""
    global packet_count, msg_rate_hz, last_rate_calc_time, last_rate_count, latest_imu_data

    packet_count += 1
    now = time.time()
    if (now - last_rate_calc_time) >= 1.0:
        msg_rate_hz = (packet_count - last_rate_count) / (now - last_rate_calc_time)
        last_rate_count = packet_count
        last_rate_calc_time = now

    roll_deg, pitch_deg, yaw_deg = quat_to_euler_deg(qw, qx, qy, qz)

    analytics = analyzer.update(
        ts=ts,
        qw=qw, qx=qx, qy=qy, qz=qz,
        ax_mps2=ax, ay_mps2=ay, az_mps2=az,
        gx_dps=gx_deg, gy_dps=gy_deg, gz_dps=gz_deg
    )

    latest_imu_data.update({
        "type": "imu",
        "source": source,
        "seq": packet_count,
        "ts": ts,
        "qw": qw,
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "roll": roll_deg,
        "pitch": pitch_deg,
        "yaw": yaw_deg,
        "ax": ax,
        "ay": ay,
        "az": az,
        "gx": gx_deg,
        "gy": gy_deg,
        "gz": gz_deg,
        "rate_hz": round(msg_rate_hz, 1),
        "analytics": analytics,
        "camera": tracker.get_latest_data(),
    })
    return latest_imu_data


_broadcast_in_progress = False

async def broadcast_imu(msg_json: str):
    global _broadcast_in_progress
    if _broadcast_in_progress:
        return  # Drop frame if websocket busy to eliminate queue lag
    _broadcast_in_progress = True
    try:
        dead_sockets = set()
        for ws in list(active_websockets):
            try:
                await ws.send_text(msg_json)
            except Exception:
                dead_sockets.add(ws)
        for ws in dead_sockets:
            active_websockets.discard(ws)
    finally:
        _broadcast_in_progress = False


def serial_telemetry_worker(port: str = "/dev/ttyUSB0", baud: int = 115200):
    """Background worker continuously reading 36-byte packets directly over UART."""
    global loop, serial_connected
    buffer = bytearray()
    PACKET_SIZE = 36
    PACKET_FORMAT = "<2s I 4h 3h 3h 3h B b H"
    SCALE_QUAT = 1.0 / 16384.0

    print(f"[*] Serial Worker starting on {port} @ {baud} baud...")

    while True:
        try:
            if not os.path.exists(port):
                serial_connected = False
                time.sleep(0.5)
                continue

            with serial.Serial(port, baud, timeout=0.1) as ser:
                serial_connected = True
                print(f"[✓] Serial Port Connected: {port} @ {baud} baud (Streaming Active)")
                buffer.clear()

                while True:
                    waiting = ser.in_waiting
                    chunk = ser.read(waiting if waiting > 0 else 1)
                    if not chunk:
                        continue
                    buffer.extend(chunk)

                    # Align with sync header (0x55, 0xAA) while printing any STM32 ASCII diagnostic logs
                    ascii_bytes = bytearray()
                    while len(buffer) >= 2 and (buffer[0] != 0x55 or buffer[1] != 0xAA):
                        b = buffer.pop(0)
                        if b == 0x0A:
                            line = ascii_bytes.decode('utf-8', errors='ignore').strip()
                            if line:
                                print(f"[STM32] {line}")
                            ascii_bytes.clear()
                        elif (0x20 <= b <= 0x7E) or b == 0x0D:
                            ascii_bytes.append(b)

                    while len(buffer) >= PACKET_SIZE:
                        if buffer[0] != 0x55 or buffer[1] != 0xAA:
                            buffer.pop(0)
                            continue

                        raw = bytes(buffer[:PACKET_SIZE])
                        del buffer[:PACKET_SIZE]

                        calc_crc = compute_crc16(raw[:PACKET_SIZE - 2])
                        rx_crc = struct.unpack_from("<H", raw, PACKET_SIZE - 2)[0]
                        if calc_crc != rx_crc:
                            continue  # CRC error, discard

                        unpacked = struct.unpack(PACKET_FORMAT, raw)
                        _, ts_ms, qw_raw, qx_raw, qy_raw, qz_raw, ax_raw, ay_raw, az_raw, \
                            gx_raw, gy_raw, gz_raw, yaw_raw, roll_raw, pitch_raw, calib_b, temp_c, _ = unpacked

                        qw = qw_raw * SCALE_QUAT
                        qx = qx_raw * SCALE_QUAT
                        qy = qy_raw * SCALE_QUAT
                        qz = qz_raw * SCALE_QUAT

                        ax = ax_raw / 100.0
                        ay = ay_raw / 100.0
                        az = az_raw / 100.0

                        gx = gx_raw / 16.0
                        gy = gy_raw / 16.0
                        gz = gz_raw / 16.0

                        ts = ts_ms * 1e-3
                        if ts <= 0:
                            ts = time.time()

                        payload = process_imu_sample(ts, qw, qx, qy, qz, ax, ay, az, gx, gy, gz, source="uart")

                        if active_websockets and loop is not None and loop.is_running():
                            msg_json = json.dumps(payload)
                            asyncio.run_coroutine_threadsafe(broadcast_imu(msg_json), loop)

        except Exception as e:
            serial_connected = False
            time.sleep(1.0)


@app.get("/")
async def get_index():
    if os.path.exists(HTML_PATH):
        return FileResponse(HTML_PATH, media_type="text/html")
    return HTMLResponse("<h1>Error: web_imu_visualizer.html not found</h1>", status_code=404)


@app.get("/api/status")
async def get_status():
    cam_data = tracker.get_latest_data()
    return {
        "status": "online",
        "transport": "uart",
        "serial_connected": serial_connected,
        "serial_port": serial_port_name,
        "rate_hz": round(msg_rate_hz, 1),
        "total_packets": packet_count,
        "connected_clients": len(active_websockets),
        "rep_count": analyzer.rep_count,
        "activity_state": analyzer.activity_state,
        "pattern_type": analyzer.current_pattern_type,
        "auto_period_s": round(analyzer.auto_period_s, 2),
        "stroke_scales": list(analyzer.get_stroke_scales()),
        "camera_detected": cam_data.get("detected", False),
        "camera_status": cam_data.get("status", "unknown"),
        "camera_fps": cam_data.get("fps", 0.0),
    }


@app.get("/api/session_report")
async def get_session_report():
    return JSONResponse(analyzer.get_session_report())


@app.post("/api/start_session")
async def api_start_session():
    analyzer.start_session()
    snap = analyzer.get_live_snapshot()
    latest_imu_data["analytics"] = snap
    await broadcast_imu(json.dumps(latest_imu_data))
    return {"status": "started", "session_id": analyzer.test_session_id}


@app.post("/api/end_session")
async def api_end_session():
    report = analyzer.end_session()
    snap = analyzer.get_live_snapshot()
    latest_imu_data["analytics"] = snap
    await broadcast_imu(json.dumps(latest_imu_data))
    return JSONResponse(report)


@app.post("/api/reset_session")
async def api_reset_session():
    analyzer.reset_session()
    return {"status": "reset", "rep_count": 0}


@app.post("/api/tare_position")
async def api_tare_position():
    analyzer.tare_position()
    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
    latest_imu_data["camera"] = tracker.get_latest_data()
    return {"status": "ok", "origin": analyzer.pos_origin_cm}


@app.post("/api/reset_origin")
async def api_reset_origin():
    analyzer.reset_origin()
    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
    latest_imu_data["camera"] = tracker.get_latest_data()
    return {"status": "ok", "origin": [0.0, 0.0, 0.0]}


class StrokeScalesRequest(BaseModel):
    scale_x: float = Field(default=1.0, gt=0.01, le=10.0)
    scale_y: float = Field(default=1.0, gt=0.01, le=10.0)
    scale_z: float = Field(default=1.0, gt=0.01, le=10.0)


class TrackingRoiRequest(BaseModel):
    x: float = Field(default=0.38, ge=0.0, le=1.0)
    y: float = Field(default=0.38, ge=0.0, le=1.0)
    width: float = Field(default=0.24, gt=0.0, le=1.0)
    height: float = Field(default=0.24, gt=0.0, le=1.0)
    normalized: bool = True


@app.post("/api/set_tracking_roi")
async def api_set_tracking_roi(payload: TrackingRoiRequest):
    """Locks optical tracking box to specified region of interest."""
    w_cam = tracker.width if tracker.width > 0 else 640
    h_cam = tracker.height if tracker.height > 0 else 480
    if payload.normalized:
        bx = int(payload.x * w_cam)
        by = int(payload.y * h_cam)
        bw = int(payload.width * w_cam)
        bh = int(payload.height * h_cam)
    else:
        bx = int(payload.x)
        by = int(payload.y)
        bw = int(payload.width)
        bh = int(payload.height)

    bx = max(0, min(w_cam - 10, bx))
    by = max(0, min(h_cam - 10, by))
    bw = max(10, min(w_cam - bx, bw))
    bh = max(10, min(h_cam - by, bh))

    tracker.set_tracking_roi((bx, by, bw, bh))
    analyzer.tare_position()
    latest_imu_data["camera"] = tracker.get_latest_data()
    if active_websockets:
        await broadcast_imu(json.dumps(latest_imu_data))
    return {
        "status": "ok",
        "message": "Tracking ROI locked",
        "roi": [bx, by, bw, bh],
        "normalized": payload.normalized,
    }


@app.post("/api/reset_tracker")
async def api_reset_tracker():
    """Resets optical tracking box back to search mode / center target reticle."""
    tracker.reset_tracker()
    latest_imu_data["camera"] = tracker.get_latest_data()
    if active_websockets:
        await broadcast_imu(json.dumps(latest_imu_data))
    return {"status": "ok", "message": "Tracker reset to search mode"}


@app.post("/api/tare_both")
async def api_tare_both():
    """Tares both IMU origin and Camera optical ground-truth origin."""
    analyzer.tare_position()
    tracker.tare()
    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
    latest_imu_data["camera"] = tracker.get_latest_data()
    if active_websockets:
        await broadcast_imu(json.dumps(latest_imu_data))
    return {"status": "ok", "message": "IMU and Camera origins tared"}


@app.post("/api/set_stroke_scales")
async def api_set_stroke_scales(payload: StrokeScalesRequest):
    """Sets per-axis stroke scaling factors for trajectory integration."""
    analyzer.set_stroke_scales(payload.scale_x, payload.scale_y, payload.scale_z)
    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
    latest_imu_data["camera"] = tracker.get_latest_data()
    if active_websockets:
        await broadcast_imu(json.dumps(latest_imu_data))
    return {
        "status": "ok",
        "message": "Stroke scales updated",
        "stroke_scales": [payload.scale_x, payload.scale_y, payload.scale_z],
    }


@app.get("/aruco_marker.png")
async def get_aruco_marker(marker_id: int = 0, size: int = 400):
    """Generates and serves an ArUco fiducial marker as PNG bytes."""
    png_bytes = generate_aruco_marker_png(marker_id=marker_id, size_px=size)
    return Response(content=png_bytes, media_type="image/png")


@app.get("/video_feed")
async def video_feed(max_frames: Optional[int] = None):
    """Streams live MJPEG camera frames with optical ground-truth annotations."""
    async def video_generator():
        count = 0
        try:
            while True:
                if max_frames is not None and count >= max_frames:
                    break
                jpeg_bytes = tracker.get_jpeg_frame()
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + jpeg_bytes + b"\r\n"
                )
                count += 1
                await asyncio.sleep(0.033)  # ~30 FPS
        except (asyncio.CancelledError, GeneratorExit):
            pass

    return StreamingResponse(
        video_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/api/export_csv")
async def export_csv():
    os.makedirs(LOG_DIR, exist_ok=True)
    csv_path = os.path.join(LOG_DIR, "surgical_repetitions_session.csv")
    analyzer.export_csv(csv_path)
    return FileResponse(
        csv_path,
        media_type="text/csv",
        filename="surgical_repetitions_session.csv"
    )


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_websockets.add(websocket)
    try:
        latest_imu_data["camera"] = tracker.get_latest_data()
        if latest_imu_data["ts"] > 0:
            await websocket.send_text(json.dumps(latest_imu_data))
        while True:
            data = await websocket.receive_text()
            try:
                cmd = json.loads(data)
                action = cmd.get("action")
                if action == "ping":
                    await websocket.send_text(json.dumps({"type": "pong"}))
                elif action == "start_session":
                    analyzer.start_session()
                    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "end_session":
                    report = analyzer.end_session()
                    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await websocket.send_text(json.dumps({"type": "session_report", "report": report}))
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "reset_session":
                    analyzer.reset_session()
                    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "tare_position":
                    analyzer.tare_position()
                    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "reset_origin":
                    analyzer.reset_origin()
                    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "tare_both":
                    analyzer.tare_position()
                    tracker.tare()
                    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "set_stroke_scales":
                    sx = max(0.01, min(10.0, float(cmd.get("scale_x", 1.0))))
                    sy = max(0.01, min(10.0, float(cmd.get("scale_y", 1.0))))
                    sz = max(0.01, min(10.0, float(cmd.get("scale_z", 1.0))))
                    analyzer.set_stroke_scales(sx, sy, sz)
                    latest_imu_data["analytics"] = analyzer.get_live_snapshot()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "set_tracking_roi":
                    w_cam = tracker.width if tracker.width > 0 else 640
                    h_cam = tracker.height if tracker.height > 0 else 480
                    norm = cmd.get("normalized", True)
                    x = float(cmd.get("x", 0.38))
                    y = float(cmd.get("y", 0.38))
                    w_box = float(cmd.get("width", 0.24))
                    h_box = float(cmd.get("height", 0.24))
                    if norm:
                        bx = int(x * w_cam)
                        by = int(y * h_cam)
                        bw = int(w_box * w_cam)
                        bh = int(h_box * h_cam)
                    else:
                        bx = int(x)
                        by = int(y)
                        bw = int(w_box)
                        bh = int(h_box)
                    bx = max(0, min(w_cam - 10, bx))
                    by = max(0, min(h_cam - 10, by))
                    bw = max(10, min(w_cam - bx, bw))
                    bh = max(10, min(h_cam - by, bh))
                    tracker.set_tracking_roi((bx, by, bw, bh))
                    analyzer.tare_position()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "reset_tracker":
                    tracker.reset_tracker()
                    latest_imu_data["camera"] = tracker.get_latest_data()
                    await broadcast_imu(json.dumps(latest_imu_data))
                elif action == "sim_sample":
                    payload = process_imu_sample(
                        ts=float(cmd.get("ts", time.time())),
                        qw=float(cmd.get("qw", 1.0)),
                        qx=float(cmd.get("qx", 0.0)),
                        qy=float(cmd.get("qy", 0.0)),
                        qz=float(cmd.get("qz", 0.0)),
                        ax=float(cmd.get("ax", 0.0)),
                        ay=float(cmd.get("ay", 0.0)),
                        az=float(cmd.get("az", 0.0)),
                        gx_deg=float(cmd.get("gx", 0.0)),
                        gy_deg=float(cmd.get("gy", 0.0)),
                        gz_deg=float(cmd.get("gz", 0.0)),
                        source="surgical_sim"
                    )
                    await websocket.send_text(json.dumps(payload))
            except Exception:
                pass
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        active_websockets.discard(websocket)



def main():
    global loop
    port = 8080
    host = "0.0.0.0"

    print("=" * 68)
    print(" Surgical Wrist-Worn IMU Hand Motion & Repetition Tracker Server")
    print(f" Direct UART   : {serial_port_name} @ {serial_baud_rate} baud")
    print(f" Web Dashboard : http://localhost:{port}")
    print(f" Video Feed    : http://localhost:{port}/video_feed")
    print(f" ArUco Marker  : http://localhost:{port}/aruco_marker.png")
    print(f" Session Report: http://localhost:{port}/api/session_report")
    print(f" Export CSV    : http://localhost:{port}/api/export_csv")
    print(f" WebSocket     : ws://localhost:{port}/ws")
    print("=" * 68)

    # Launch Direct UART reader in daemon background thread
    uart_thread = threading.Thread(
        target=serial_telemetry_worker,
        args=(serial_port_name, serial_baud_rate),
        daemon=True
    )
    uart_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    config = uvicorn.Config(app=app, host=host, port=port, loop="asyncio", log_level="warning")
    server = uvicorn.Server(config)

    try:
        loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        print("Server shutdown complete.")


if __name__ == "__main__":
    main()
