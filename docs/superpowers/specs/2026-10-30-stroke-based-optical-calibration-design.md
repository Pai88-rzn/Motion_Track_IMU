# Design Document: Synchronized 3D Free-Floating Hand Tracking via Adaptive Stroke Filtering & Optical Ground-Truth Calibration (Dual Web + Camera View)

## 1. Intent and Why
During surgical operations, an operator's hand moves freely in 3D space above the operating table (0–35 cm elevation, 0–50 cm lateral reach). The motion includes fine-motor strokes (incisions, suture passes, instrument positioning) and inter-stroke micro-pauses.

Naive double integration of linear acceleration ($s = \iint a \, dt^2$) fails on MEMS IMUs (Bosch BNO055) due to quadratic drift ($s \propto t^2$), where a $0.05\text{ m/s}^2$ bias causes tens of centimeters of false movement in seconds. Overly aggressive deadbands eliminate drift but destroy responsiveness, creating perceived lag and desynchronization.

The user has explicitly requested:
1. **Stand-Alone Production**: In the final deployment, the device must operate purely on IMU without requiring external optical cameras.
2. **Interactive Calibration & Real-time Dual View**: During training/tuning, the user must be able to view **both the 3D Web View and the Live Camera View** simultaneously in the browser dashboard. The camera tracks physical ground-truth displacement in real time to calibrate the stroke-based IMU filter.

---

## 2. Goals, Non-Goals, and Anti-Goals

### Goals
1. **Unified Dual Web + Camera Dashboard**:
   - Web interface (`http://localhost:8080`) displays both the Three.js 3D hand visualizer AND a real-time Picture-in-Picture / side-by-side Camera View from `/dev/video0`.
   - Real-time optical overlay on the camera stream: detects ArUco marker (`DICT_4X4_50`, ID 0), draws 3D pose axes, and calculates physical displacement $(X_{cam}, Y_{cam}, Z_{cam})$ in cm.
   - Live telemetry comparison table: displays IMU position vs Optical ground-truth position side-by-side with instantaneous error $(\Delta X, \Delta Y, \Delta Z)$ in cm.
2. **Stand-Alone Real-Time Operation**: 3D position $(x, y, z)$ tracking driven entirely by the STM32 + BNO055 100 Hz UART stream without runtime camera dependency in production.
3. **Synchronous 1:1 Response**: Hand translation (elevating 30 cm, sliding 20 cm) reflects in the 3D visualizer with sub-30ms latency and proportional displacement.
4. **Zero Stationary Drift**: When the hand is held still in mid-air or resting on the drape, the 3D model remains locked without floating or sinking.
5. **Calibrated Parameter Optimization**: An automated calibration routine synchronizes camera ground truth with IMU packets to determine optimal dynamic stroke scale factors ($K_x, K_y, K_z$) and jerk thresholds.
6. **On-Screen ArUco Display**: Web endpoint `/aruco_marker.png` and modal dialog in the UI allowing the user to display the calibration marker directly on a phone screen or print it on paper.

### Non-Goals
* Continuous optical tracking in production (the camera is not used during actual surgery).
* Finger or joint articulation (the tracker is a single rigid body on the wrist/dorsum).
* Absolute magnetic compass orientation dependence (heading is desk-relative / tared).

### Anti-Goals
* Allowing runaway quadratic position drift when the hand pauses mid-air.
* Heavy smoothing filters (> 60 ms group delay) that make the visualizer feel sluggish.
* Forcing the user to switch between separate desktop windows (OpenCV GUI vs Web browser).

---

## 3. Constraints
* **MCU & Sensor**: STM32F103C8T6 streaming 36-byte binary packets @ 100.3 Hz over UART (`/dev/ttyUSB0` @ 115200 baud).
* **IMU Data**: Bosch BNO055 fused orientation quaternion ($w, x, y, z$) and gravity-compensated linear acceleration ($a_x, a_y, a_z$ in $\text{m/s}^2$).
* **Host Platform**: Linux (Ubuntu), Python 3.10+, OpenCV 4.5.4, `/dev/video0` HD webcam (1280x720 @ 30 FPS).
* **Operating Volume**: Operating table area: $X \in [-40, +40]\text{ cm}$, $Y \in [-30, +30]\text{ cm}$, $Z \in [0, 35]\text{ cm}$ (elevation bounded at table surface).

---

## 4. Approaches Considered

### Approach 1: Adaptive Stroke-Based Filter with Optical Ground-Truth Calibration (Selected)
* **Mechanics**: Decompose hand motion into active stroke phases and zero-velocity intervals (ZUPT). Integrate linear acceleration using a non-linear stroke response curve ($v = \int \text{sign}(a) \cdot (|a| - a_{thresh})^\gamma \cdot K_{gain} \, dt$). An optical ArUco marker tracked by the webcam provides ground-truth distance ($d_{cam}$) during reference motions (e.g., 30 cm lift, 20 cm slide) to optimize $K_{gain}$, $\gamma$, and threshold parameters.
* **Trade-offs**: Near-zero computational latency, fully deterministic, easy to port into STM32 C firmware later. Very slow drift below sensor noise threshold is clamped to zero.
* **Decision**: Selected by user for direct synchrony, transparency, and stand-alone embedded viability.

### Approach 2: Learned Displacement Estimator (Machine Learning / Kinematic Regressor)
* **Mechanics**: Supervised regression model (1D-CNN or Ridge Regressor) trained on paired IMU time-series and webcam optical tracking trajectories to directly predict $(\Delta x, \Delta y, \Delta z)$ over sliding time windows.
* **Trade-offs**: Immune to quadratic integration drift, but requires extensive dataset collection across diverse surgical hand movements, has higher inference overhead, and is harder to deploy on Cortex-M3.
* **Decision**: Kept as an optional future enhancement if analytical stroke filtering reaches theoretical limits.

---

## 5. Detailed Architecture & Implementation

### A. Camera Video Streaming & Ground-Truth Engine (`tools/camera_ground_truth.py` & `imu_web_server.py`)
1. **Async Video Capture Thread**:
   - Background daemon thread captures frames from `/dev/video0` at 30 FPS (1280x720 or 640x480).
   - Detects `DICT_4X4_50` marker ID 0 (40 mm width).
   - Solves PnP pose to obtain translation vector $[t_x, t_y, t_z]$ relative to camera optical center.
   - Calculates relative displacement $[\Delta x, \Delta y, \Delta z]_{cam}$ from tare point in centimeters.
   - Overlays marker boundary, 3D coordinate triad, and live centimeter readings directly on video frames.
2. **FastAPI MJPEG Stream**:
   - Endpoint `GET /video_feed` streams multipart JPEG images (`multipart/x-mixed-replace; boundary=frame`) directly into `<img>` tag in the web visualizer.
3. **Data Telemetry Fusion**:
   - WebSocket message includes `camera: { detected: bool, pos_cm: [x, y, z], delta_cm: [dx, dy, dz], fps: float }`.

### B. Adaptive Stroke Integration in `surgical_motion_analyzer.py`
1. **Dynamic Bias Elimination**: Maintain running body-frame bias subtraction prior to world-frame transformation to cancel silicon Z/X/Y offsets.
2. **Stroke Segmentation**:
   - Detect start of stroke when $|a_{lin}| > a_{motion\_start}$ ($0.12\text{ m/s}^2$) or jerk $|\dot{a}| > j_{thresh}$.
   - Apply non-linear power-law acceleration weighting to preserve fast transients while suppressing noise floor.
   - When motion decelerates and energy decays below $a_{motion\_stop}$ for $> 80\text{ ms}$, engage critical damping to smoothly freeze velocity to 0.0.
3. **Calibrated Scale Vector**: Apply per-axis scale factors $(K_x, K_y, K_z)$ derived from ground-truth optical comparison.

### C. Visualizer Dual-View Integration (`web_imu_visualizer.html`)
1. **Interactive Picture-in-Picture (PiP) Window**:
   - Resizable/collapsible camera window in top-right or bottom-right corner.
   - Toggle button (or shortcut `C`) to show/hide Camera View.
   - Live stream from `/video_feed`.
2. **Live Side-by-Side Comparison HUD**:
   - Live table comparing:
     - **IMU Position**: $[X, Y, Z]$ cm
     - **Camera Ground Truth**: $[X, Y, Z]$ cm
     - **Tracking Error**: $[e_x, e_y, e_z]$ cm
3. **Calibration & Tare Controls**:
   - "Tare Both (Zero Pos)" button: zeros both IMU position and Camera position simultaneously.
   - "Auto-Calibrate Scales": records a reference motion (e.g. 30 cm vertical lift), compares $\int a \, dt^2$ with $\Delta Z_{cam}$, and updates scale gains $K_z, K_x, K_y$ on the fly.
4. **ArUco Marker Display Modal**:
   - Button "Show Calibration Marker" displays the ID 0 ArUco square directly on screen with a download/print button.

---

## 6. What's Left to the Builder
* Camera frame rate vs resolution optimization (e.g. 640x480 @ 30 FPS to minimize USB bus contention with FTDI UART).
* Marker size configuration default (40 mm x 40 mm).
* Default camera orientation alignment mode (whether camera is facing from front or side).
