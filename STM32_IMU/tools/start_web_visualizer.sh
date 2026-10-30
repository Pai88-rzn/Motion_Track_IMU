#!/usr/bin/env bash
# Quick launcher for Surgical Wrist IMU Hand Motion & Repetition Tracker Web Server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 Humble if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[WARN] /opt/ros/humble/setup.bash not found — starting in standalone Web/Sim mode."
fi

echo "===================================================================="
echo " Starting Surgical Wrist IMU Motion & Repetition Tracker Server..."
echo " Web UI        : http://localhost:8080"
echo " LAN UI        : http://$(hostname -I | awk '{print $1}'):8080"
echo " Session Report: http://localhost:8080/api/session_report"
echo " CSV Export    : http://localhost:8080/api/export_csv"
echo "===================================================================="

exec python3 "${SCRIPT_DIR}/imu_web_server.py"
