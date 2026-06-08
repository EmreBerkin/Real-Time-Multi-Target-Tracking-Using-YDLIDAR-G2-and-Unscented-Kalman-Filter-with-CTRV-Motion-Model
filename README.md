# Real-Time Multi-Target Tracking Using YDLIDAR G2 and Unscented Kalman Filter with CTRV

This repository contains a real-time, multi-target tracking system implemented in MATLAB. The system processes raw polar measurements from a **YDLIDAR G2** sensor, applies **DBSCAN**-based spatial clustering for target detection, and tracks dynamic indoor targets using an **Unscented Kalman Filter (UKF)** operating with a nonlinear **Constant Turn Rate and Velocity (CTRV)** motion model.

## 🚀 Key Features

* **Real-Time Data Acquisition:** Direct serial port interface (`COM9`, 230400 baud) with YDLIDAR G2.
* **Spatial Preprocessing:** DBSCAN-based and custom 2D clustering to isolate dynamic targets while rejecting environmental clutter and sensor noise.
* **Nonlinear State Estimation:** Advanced state estimation via UKF ($x = [p_x, p_y, v, \psi, \omega]^T$) bypassing explicit Jacobian matrices.
* **Robust Track Management:** Nearest-Neighbor (NN) data association combined with strict confirmation/missed-frame logical gates to reduce false tracks.
* **Statistical Consistency Analysis:** Multi-panel visualization of **Normalized Innovation Squared (NIS)** and **Normalized Estimation Error Squared (NEES)** bounded by Chi-Square ($\chi^2$) 95% confidence intervals to monitor filter health.

---

## 📊 System Architecture

The pipeline consists of four main sequential stages as described in the accompanying project paper:
1. **LiDAR Data Acquisition:** Reading range and scan angle values.
2. **Preprocessing & Clustering:** Converting polar representations to Cartesian space, range filtering, and filtering out static geometries using target height/width constraints.
3. **State Estimation & Association:** Prediction and correction routines of UKF paired with Euclidean distance gating.
4. **Performance Profiling:** Continuous logging of RMSE, track loss, and computational frame durations.

---

## 💻 Hardware & Software Requirements

* **Sensor:** YDLIDAR G2 (or equivalent 2D LiDAR scanning unit).
* **Environment:** MATLAB R2021a or newer.
* **Toolboxes:** *Instrument Control Toolbox* (for Serial communication), *Statistics and Machine Learning Toolbox* (Optional, falls back to native `simpleCluster2D` if missing).

---

## 🛠️ Configuration & Usage

1. Connect your YDLIDAR G2 to your system.
2. Open `src/main_tracking.m`.
3. Modify the **Parametre Bloğu** section to match your hardware mapping:
   ```matlab
   COM_PORT   = "COM9";   % Change to your specific port (e.g., "/dev/ttyUSB0" on Linux)
   BAUDRATE   = 230400;
