# Surgical Wrist-Worn IMU Hand Motion & Repetition Tracker

An embedded + host analytics platform for tracking **3D surgical hand motion**, **unsupervised repetition patterns**, **economy of motion**, and **surgeon hand fatigue/tremor** over multi-hour procedures using a wrist-worn **Bosch BNO055 9-DOF IMU**.

---

## 1. Project Structure

```text
Imu_MOTION TRACKER/
├── README.md                              # Project documentation & hardware/DSP guide
└── STM32_IMU/                             # STM32F103 + Bosch BNO055 100 Hz Firmware & Host Tools
    ├── App/
    │   ├── imu_app.c / imu_app.h          # 100 Hz BNO055 NDOF supervisory state machine
    │   ├── microros_imu.c / .h            # micro-ROS publisher (/imu @ 100 Hz)
    │   └── serial_protocol.c / .h         # 36-byte CRC16 binary telemetry framing
    ├── Drivers/BSP/
    │   └── bno055.c / bno055.h            # Bosch BNO055 9-DOF I2C driver (Quat, Linear Acc, Gyro)
    └── tools/
        ├── surgical_motion_analyzer.py    # Core 3D Trajectory + Unsupervised Pattern/Cycle DSP Engine
        ├── imu_web_server.py              # ROS 2 (/imu) + FastAPI WebSocket & CSV/JSON Report Server
        ├── web_imu_visualizer.html        # 3D Forearm/Wristband/Surgical Hand & Live Repetition UI
        ├── imu_receiver.py                # Direct USB-UART Serial CLI Receiver + Repetition Counter
        └── start_web_visualizer.sh        # One-command launcher for the Web Dashboard
```

---

## 2. Quick Start: Launch the 3D Surgical Hand & Repetition Dashboard

### Option A: Live with `STM32_IMU` Hardware (via ROS 2 / micro-ROS) or Built-In Surgical Simulator
```bash
cd "/home/rifai/Imu_MOTION TRACKER/STM32_IMU/tools"
./start_web_visualizer.sh
```
Then open **`http://localhost:8080`** in your browser:
* **With `STM32_IMU` connected**: The dashboard automatically receives 100 Hz `/imu` telemetry, rotates the 3D gloved surgical hand + wristband, traces the live 3D trajectory ribbon, and counts repetitions.
* **Without hardware attached (Simulation Mode)**: Select a surgical profile in the top bar (*Suturing Needle Arc [2.0s]*, *Fast Knot Tying [0.9s]*, *Linear Dissection Reach [1.5s]*, or *Fatigued Suturing + 10Hz Tremor*) and click **`▶ Start Surgical Sim`** to watch the Python `SurgicalMotionAnalyzer` self-discover the cycle period and classify each maneuver in real time.

### Option B: Direct Serial USB-UART CLI Mode (Without ROS 2)
```bash
python3 "/home/rifai/Imu_MOTION TRACKER/STM32_IMU/tools/imu_receiver.py" -p /dev/ttyUSB0 -b 115200
```

---

## 3. How the Unsupervised Pattern & 3D Trajectory Engine Works

Implemented in [`STM32_IMU/tools/surgical_motion_analyzer.py`](STM32_IMU/tools/surgical_motion_analyzer.py):

1. **Bounded 3D Hand Location (`pos_cm`)**:
   * Rotates gravity-free linear acceleration (`linear_acc`) from the wrist frame into the fixed Earth/Operating Table frame using the BNO055 unit quaternion (`quat`).
   * Applies **Zero-Velocity Updates (ZUPT)** during micro-pauses ($>60\text{ ms}$) combined with **2nd-order Butterworth High-Pass Washout Filters** ($f_c = 0.18\text{–}0.25\text{ Hz}$) and a **Biomechanical Forearm Kinematic Model** ($L_{\text{forearm}} = 26\text{ cm}$).
   * Reconstructs the 3D loop ($\Delta X, \Delta Y, \Delta Z$ in cm) of every repetition cycle without unbounded quadratic drift over multi-hour procedures.
2. **Unsupervised Pattern & Cycle Self-Discovery ("Analyze by Data Pattern")**:
   * **Orientation-Invariant Principal Axis Projection**: Projects 6-DOF angular velocity and linear velocity onto the dominant eigenvector over a sliding window to form a clean 1D motion signal $s(t)$.
   * **Multi-Scale Autocorrelation $R(\tau)$**: Scans candidate lags $\tau \in [0.40\text{ s}, 4.50\text{ s}]$ every 150 ms to automatically lock onto the natural cycle period $T_{\text{rep}}$ (`auto_period_s`) and periodicity confidence (`pattern_confidence`) without any preset cycle duration.
   * **Unsupervised Maneuver Classification**: Classifies each segmented cycle into:
     * `Suturing / Rotational Arc` (dominant forearm pronation/supination roll)
     * `Instrument / Axial Action` (compact rotation with minimal translation)
     * `Dissection / Linear Reach` (translational excursion $\ge 8\text{ cm}$)
     * `Micro-Manipulation`
3. **Surgical Skill & Fatigue Metrics**:
   * **Economy of Motion**: 3D path length per repetition ($\text{cm/rep}$) and angular excursion ($^\circ/\text{rep}$).
   * **Smoothness Score (LDLJ %)**: Derived from Log Dimensionless Jerk.
   * **Physiological Tremor Index ($8\text{–}12\text{ Hz}$)**: Band-pass filter centered at $10\text{ Hz}$ measuring high-frequency hand tremor RMS ($\text{m/s}^2$) to track surgeon muscle fatigue across multi-hour operations.

---

## 4. REST API Endpoints for Multi-Hour Logging & Export

While `imu_web_server.py` is running on port `8080`:
* **`GET http://localhost:8080/api/status`** — Live connection status, repetition count, and auto-discovered cycle period.
* **`GET http://localhost:8080/api/session_report`** — Full JSON report of the multi-hour session (active vs. idle ratio, pattern breakdown, smoothness, tremor, and recent repetitions).
* **`GET http://localhost:8080/api/export_csv`** — Downloads `surgical_repetitions_session.csv` containing every segmented repetition cycle and its biomechanical metrics.
* **`POST http://localhost:8080/api/reset_session`** — Resets session counters for a new surgical procedure.
