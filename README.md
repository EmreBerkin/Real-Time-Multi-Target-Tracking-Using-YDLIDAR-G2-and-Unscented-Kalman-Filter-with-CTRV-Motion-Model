# 🎯 Real-Time Multi-Target Tracking with YDLIDAR G2 + UKF-CTRV

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.9%2B-blue?style=flat-square&logo=python" />
  <img src="https://img.shields.io/badge/MATLAB-R2022a%2B-orange?style=flat-square&logo=mathworks" />
  <img src="https://img.shields.io/badge/Sensor-YDLIDAR%20G2-green?style=flat-square" />
  <img src="https://img.shields.io/badge/Filter-UKF%20%7C%20CTRV-purple?style=flat-square" />
  <img src="https://img.shields.io/badge/License-MIT-lightgrey?style=flat-square" />
</p>

<p align="center">
  A real-time multi-target tracking system for indoor environments using a 2D LiDAR sensor, <br/>
  DBSCAN-based clustering, and an Unscented Kalman Filter with a Constant Turn Rate and Velocity (CTRV) motion model.
</p>

---

## 📑 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Theory](#-theory)
  - [CTRV Motion Model](#ctrv-motion-model)
  - [Unscented Kalman Filter](#unscented-kalman-filter)
  - [DBSCAN Clustering](#dbscan-clustering)
  - [Data Association & Track Management](#data-association--track-management)
  - [NIS Consistency Analysis](#nis-consistency-analysis)
- [Repository Structure](#-repository-structure)
- [Requirements](#-requirements)
- [Installation](#-installation)
- [Usage](#-usage)
  - [Running with Live YDLIDAR G2](#running-with-live-ydlidar-g2)
  - [Running with Recorded Data](#running-with-recorded-data)
  - [MATLAB Version](#matlab-version)
- [Configuration](#-configuration)
- [Results](#-results)
- [Experimental Scenarios](#-experimental-scenarios)
- [Performance Metrics](#-performance-metrics)
- [Limitations & Future Work](#-limitations--future-work)
- [Authors](#-authors)
- [Citation](#-citation)
- [References](#-references)

---

## 🔍 Overview

This project implements a complete perception pipeline for **real-time tracking of multiple moving targets** (e.g., pedestrians) in indoor environments. The system ingests raw 2D LiDAR scans from a **YDLIDAR G2** sensor, extracts candidate object clusters via **DBSCAN**, associates measurements to tracks using a **nearest-neighbor gating** strategy, and estimates each target's full motion state—position, speed, heading, and turn rate—using an **Unscented Kalman Filter (UKF)** with the nonlinear **CTRV** motion model.

Filter statistical consistency is continuously monitored via the **Normalized Innovation Squared (NIS)** metric and chi-square confidence bounds, enabling systematic tuning of the process and measurement noise covariance matrices.

The entire pipeline runs in **real time on standard PC hardware**, making it suitable as a basis for indoor autonomous navigation, robot perception, and intelligent surveillance systems.

> **Academic context:** This system was developed as a semester project for the *EE4084 Kalman Filtering* course at Marmara University.

---

## ✨ Key Features

| Feature | Description |
|---|---|
| 🔄 **Real-Time Operation** | Processes live LiDAR scans at sensor frame rate on a standard laptop |
| 📡 **YDLIDAR G2 Integration** | Direct serial/USB interface with the YDLIDAR G2 2D LiDAR sensor |
| 🔵 **DBSCAN Clustering** | Density-based target extraction with geometric dimension filtering |
| 🧮 **UKF-CTRV Estimator** | Nonlinear state estimation without Jacobian computation |
| 🔗 **Multi-Target Association** | Nearest-neighbor gating with Mahalanobis distance |
| 🛡️ **Track Management** | Confirmation threshold + missed-frame deletion logic |
| 📊 **NIS Analysis** | Per-track chi-square consistency plots with 95% confidence bounds |
| 🎯 **False Track Suppression** | Geometric filtering + confirmation logic reduces spurious tracks |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         YDLIDAR G2 Sensor                           │
│                  Polar scan: (r, θ) at each frame                   │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    LiDAR Data Preprocessing                         │
│  • Polar → Cartesian  (x = r·cosθ,  y = r·sinθ)                   │
│  • Range filtering  (remove out-of-bounds measurements)             │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     DBSCAN Clustering                               │
│  • ε = 0.25 m,  MinPts = 5                                          │
│  • Geometric filter  (min/max width & height)                       │
│  • Output: cluster centroids  z_k = [px, py]                        │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                 Data Association & Track Management                 │
│  • Nearest-neighbor gating  (Mahalanobis distance)                  │
│  • New track init  (unmatched measurements)                         │
│  • Track confirmation  (N_confirm consecutive hits)                 │
│  • Track deletion  (N_miss consecutive misses)                      │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    UKF-CTRV State Estimator                         │
│  State:  x = [px, py, v, ψ, ω]ᵀ                                    │
│  Predict: sigma points → CTRV dynamics                              │
│  Update:  measurement model  z = [px, py]ᵀ + noise                 │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│             NIS Consistency Monitor  +  Visualization               │
│  • Per-track NIS plots vs. χ²(2) bounds                             │
│  • Real-time 2D trajectory display                                  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 📐 Theory

### CTRV Motion Model

The **Constant Turn Rate and Velocity (CTRV)** model captures the motion of a target moving at a constant linear speed $v$ and a constant yaw rate $\omega$. The 5-dimensional state vector is:

$$\mathbf{x} = \begin{bmatrix} p_x & p_y & v & \psi & \omega \end{bmatrix}^T$$

The discrete-time transition equations for $|\omega| > \epsilon$ are:

$$p_x(k+1) = p_x(k) + \frac{v}{\omega}\bigl(\sin(\psi + \omega\Delta t) - \sin\psi\bigr)$$

$$p_y(k+1) = p_y(k) + \frac{v}{\omega}\bigl(-\cos(\psi + \omega\Delta t) + \cos\psi\bigr)$$

$$\psi(k+1) = \psi(k) + \omega\,\Delta t$$

For $|\omega| < \epsilon$ (near-straight motion), the model degenerates to a constant velocity (CV) approximation to avoid numerical instability.

**Why CTRV?** Human pedestrians and wheeled robots frequently execute smooth turning maneuvers that a purely straight-line model cannot represent faithfully. The CTRV model captures these turns compactly with a single extra state $\omega$, without the complexity of an IMM framework.

---

### Unscented Kalman Filter

The UKF approximates the posterior distribution of a nonlinear system by propagating a minimal set of **sigma points** through the true nonlinear dynamics, bypassing the need for Jacobian matrices required by the EKF.

**Sigma point generation** (scaled unscented transform, $n = 5$):

$$\mathcal{X}_0 = \hat{\mathbf{x}}, \quad
\mathcal{X}_i = \hat{\mathbf{x}} + \left(\sqrt{(n+\lambda)P}\right)_i, \quad
\mathcal{X}_{i+n} = \hat{\mathbf{x}} - \left(\sqrt{(n+\lambda)P}\right)_i$$

where $\lambda = \alpha^2(n+\kappa) - n$ is a scaling parameter.

**Prediction step:**
1. Generate $2n+1$ sigma points from $(\hat{\mathbf{x}}_{k-1}, P_{k-1})$.
2. Propagate each through the CTRV function $f(\cdot)$.
3. Compute the weighted mean $\hat{\mathbf{x}}_k^-$ and covariance $P_k^-$ (add process noise $Q$).

**Update step:**
1. Map sigma points through the measurement function $h(\mathbf{x}) = [p_x,\, p_y]^T$.
2. Compute predicted measurement $\hat{\mathbf{z}}_k$, innovation covariance $S_k$, and cross-covariance $P_{xz}$.
3. Kalman gain: $K_k = P_{xz} S_k^{-1}$.
4. State update: $\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k(\mathbf{z}_k - \hat{\mathbf{z}}_k)$.
5. Covariance update: $P_k = P_k^- - K_k S_k K_k^T$.

---

### DBSCAN Clustering

DBSCAN (Density-Based Spatial Clustering of Applications with Noise) groups points that are densely connected while labeling sparse outliers as noise. A point $p$ is a **core point** if at least $MinPts$ neighbors lie within radius $\varepsilon$.

**Advantages for LiDAR:**
- No need to specify the number of clusters in advance.
- Robustly handles noise and irregular cluster shapes.
- Effective at separating closely spaced targets.

**Parameters used:**

| Parameter | Value | Rationale |
|---|---|---|
| `ε` (epsilon) | 0.25 m | Matches typical point spacing at indoor ranges |
| `MinPts` | 5 | Suppresses single-point noise returns |
| Min cluster width | 0.15 m | Filters sensor noise |
| Max cluster width | 1.2 m | Filters walls and background structures |

Each accepted cluster's centroid is extracted as the position measurement $\mathbf{z}_k = [p_x,\, p_y]^T$.

---

### Data Association & Track Management

Each frame, detected cluster centroids must be matched to existing tracks. The system uses **nearest-neighbor (NN) gating** with a Mahalanobis-distance gate:

$$d_{\text{gate}} = \sqrt{(\mathbf{z} - \hat{\mathbf{z}})^T S^{-1} (\mathbf{z} - \hat{\mathbf{z}})} < \gamma$$

A measurement is assigned to the closest track within the gate. Unmatched measurements start candidate new tracks.

**Track lifecycle:**

```
[Candidate] ──(N_confirm hits)──► [Active] ──(N_miss misses)──► [Deleted]
     │                                │
     └──(timeout before confirm)──► [Deleted]
```

| Parameter | Default | Effect |
|---|---|---|
| `N_confirm` | 3 frames | Prevents false track initialization |
| `N_miss` | 5 frames | Allows short occlusion survival |
| `γ` (gate threshold) | 2.5 | Tunable; larger gate → more associations |

---

### NIS Consistency Analysis

The **Normalized Innovation Squared (NIS)** tests whether the filter's uncertainty estimates are consistent with the actual innovation:

$$\text{NIS}_k = \boldsymbol{\nu}_k^T S_k^{-1} \boldsymbol{\nu}_k$$

where $\boldsymbol{\nu}_k = \mathbf{z}_k - \hat{\mathbf{z}}_k$ is the innovation vector and $S_k$ is the innovation covariance.

For a 2D measurement vector, $\text{NIS}_k \sim \chi^2(2)$ when the filter is consistent. The 95% confidence interval is approximately $[0.103,\ 5.991]$.

- **NIS consistently above upper bound** → filter is overconfident (Q or R too small).
- **NIS consistently below lower bound** → filter is underconfident (Q or R too large).

This metric is used during development to tune $Q$ and $R$ systematically.

---

## 📁 Repository Structure

```
lidar-ukf-ctrv-tracking/
│
├── matlab/                         # MATLAB implementation (alternative)
│   ├── main_tracking.m             # Top-level script
│   ├── ukf_predict.m               # UKF prediction step
│   ├── ukf_update.m                # UKF update step
│   ├── ctrv_model.m                # CTRV state transition function
│   ├── dbscan_cluster.m            # DBSCAN clustering
│   ├── nearest_neighbor_gate.m     # Data association
│   ├── track_manager.m             # Track init/confirm/delete
│   ├── nis_analysis.m              # NIS plots
│   └── visualize_tracks.m          # Live visualization
│
├── docs/
│   ├── EE4084_Project_Paper.pdf    # Final project report
│   ├── EE4084_Project_Proposal.pdf # Project proposal
│   └── system_diagram.png          # Architecture diagram
│
├── requirements.txt
├── environment.yml                  # Conda environment
└── README.md
```

---

## 📦 Requirements

### Python

| Library | Version | Purpose |
|---|---|---|
| `numpy` | ≥ 1.23 | Linear algebra, sigma point computations |
| `scipy` | ≥ 1.9 | Chi-square distribution for NIS bounds |
| `scikit-learn` | ≥ 1.1 | DBSCAN implementation |
| `matplotlib` | ≥ 3.6 | Real-time visualization and NIS plots |
| `pyserial` | ≥ 3.5 | YDLIDAR G2 serial communication |
| `ydlidar-sdk` | latest | Official YDLIDAR Python SDK (optional) |

### MATLAB

- MATLAB R2022a or later
- Statistics and Machine Learning Toolbox (for chi-square bounds)

### Hardware

- YDLIDAR G2 2D LiDAR sensor
- USB-to-Serial adapter (if not using native USB)
- Standard PC (tested on Intel Core i5, 8 GB RAM)

---

## 🔧 Installation

### 1. Clone the repository

```bash
git clone https://github.com/yourusername/lidar-ukf-ctrv-tracking.git
cd lidar-ukf-ctrv-tracking
```

### 2. Create a Python environment

**Option A — Conda (recommended):**
```bash
conda env create -f environment.yml
conda activate lidar-tracking
```

**Option B — pip:**
```bash
python -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Connect the YDLIDAR G2

On Linux, grant serial port permissions:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in, then verify:
ls /dev/ttyUSB*
```

On Windows, identify the COM port in Device Manager (e.g., `COM3`).

---

## 🚀 Usage

### Running with Live YDLIDAR G2

```bash
python python/main.py --mode live --port /dev/ttyUSB0 --baud 230400
```

| Argument | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port of the YDLIDAR G2 |
| `--baud` | `230400` | Baud rate |
| `--record` | `False` | Save scan sequence to `data/` for later replay |
| `--config` | `config.py` | Path to configuration file |

### Running with Recorded Data

Replay any of the included scenario files without needing the physical sensor:

```bash
python python/main.py --mode playback --file data/scenario_two_crossing.mat
```

### MATLAB Version

```matlab
% From the MATLAB command window:
cd matlab/
main_tracking      % runs with default config; edit config block at top of file
```

---

## ⚙️ Configuration

All tunable parameters live in `python/config.py`:

```python
# ── Sensor ────────────────────────────────────────────────
SENSOR_PORT       = "/dev/ttyUSB0"
SENSOR_BAUD       = 230_400
RANGE_MIN         = 0.10   # metres
RANGE_MAX         = 8.00   # metres

# ── DBSCAN Clustering ─────────────────────────────────────
DBSCAN_EPS        = 0.25   # neighbourhood radius (metres)
DBSCAN_MIN_PTS    = 5      # minimum points for a core point
CLUSTER_MIN_WIDTH = 0.15   # metres – below this: noise
CLUSTER_MAX_WIDTH = 1.20   # metres – above this: wall/structure

# ── UKF Parameters ────────────────────────────────────────
DT                = 0.10   # sampling interval (seconds)
ALPHA             = 1e-3   # sigma point spread
BETA              = 2.0    # prior knowledge parameter (Gaussian → 2)
KAPPA             = 0.0    # secondary scaling

# Process noise covariance Q
STD_A             = 1.5    # linear acceleration std (m/s²)
STD_YAWDD         = 0.5    # yaw acceleration std (rad/s²)

# Measurement noise covariance R
STD_LIDAR_PX      = 0.15   # position x std (metres)
STD_LIDAR_PY      = 0.15   # position y std (metres)

# ── Data Association ──────────────────────────────────────
GATE_THRESHOLD    = 2.5    # Mahalanobis distance gate

# ── Track Management ──────────────────────────────────────
N_CONFIRM         = 3      # hits needed to confirm a new track
N_MISS            = 5      # misses before track deletion

# ── NIS ───────────────────────────────────────────────────
NIS_DOF           = 2      # measurement dimensions
NIS_ALPHA         = 0.05   # significance level (95% CI)
```

**Tuning guide:**

- If tracks are lost during fast turns → increase `STD_YAWDD`.
- If position estimates are noisy/jumpy → increase `STD_LIDAR_PX/PY` (trust measurements less).
- If false tracks appear frequently → increase `N_CONFIRM` or decrease `GATE_THRESHOLD`.
- Use the NIS plots to guide noise covariance tuning: aim for ~95% of NIS samples within the chi-square bounds.

---

## 📊 Results

### Real-Time Tracking Visualization

The system displays a live 2D top-down view of the tracking scene. Each confirmed track shows:
- **Estimated trajectory** (colored path)
- **Current velocity** (arrow overlay)
- **Track ID and status** (`ACTIVE`, `missed=N`)

### NIS Consistency Plots

Six independent track slots are monitored simultaneously. The NIS plot for each slot shows the innovation sequence against the 95% chi-square bounds (red lines). A well-tuned filter keeps the vast majority of samples between the bounds.

```
NIS — 6 Track Fixed Panels
┌──────────┬──────────┬──────────┐
│ T1 OK 71%│ T3 OK 87%│ T4 OK 97%│
├──────────┼──────────┼──────────┤
│ T6 100%  │ T7 NIS   │ waiting  │
└──────────┴──────────┴──────────┘
```

---

## 🧪 Experimental Scenarios

| # | Scenario | Targets | Key Challenge |
|---|---|---|---|
| 1 | Single target, linear path | 1 | Baseline verification |
| 2 | Single target, curved path | 1 | CTRV model accuracy |
| 3 | Two targets, parallel motion | 2 | Association disambiguation |
| 4 | Two targets, crossing trajectories | 2 | Identity preservation through crossings |
| 5 | Multiple targets with occlusions | 3+ | Missed-frame track survival |

All scenarios were conducted in an indoor laboratory at Marmara University. The YDLIDAR G2 was mounted approximately **40 cm above ground level**.

---

## 📈 Performance Metrics

The system is evaluated using four quantitative metrics:

### Root Mean Square Error (RMSE)

$$\text{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}(x_i - \hat{x}_i)^2}$$

Measured separately for $p_x$ and $p_y$ where ground truth is available.

### Track Loss Ratio

Fraction of frames in which an expected active track was not maintained. Lower is better.

### NIS Consistency Rate

Percentage of NIS samples falling within the $\chi^2(2)$ 95% confidence interval $[0.103,\ 5.991]$. Target: ≥ 90%.

### Processing Time per Frame

Measured end-to-end: from raw scan ingestion to updated track states. Target: < sensor frame period to maintain real time.

---

## ⚠️ Limitations & Future Work

### Current Limitations

| Issue | Root Cause | Impact |
|---|---|---|
| Cluster merging | DBSCAN merges very close targets | Temporary identity loss |
| Abrupt maneuver degradation | CTRV assumes constant ω | Position drift during sharp turns |
| Crowded scene performance | NN gating is a greedy one-to-one method | Association errors in dense scenes |
| 2D tracking only | Single-plane LiDAR | Cannot track height variation |

### Planned Improvements

- **IMM (Interactive Multiple Model) Filtering** — switching between CV, CA, and CTRV models to better handle abrupt maneuvers.
- **JPDA (Joint Probabilistic Data Association)** — probabilistic measurement-to-track assignment for improved robustness in dense multi-target scenarios.
- **Adaptive DBSCAN** — dynamically adjust ε based on local point density to reduce cluster merging.
- **Sensor Fusion** — combine LiDAR with IMU and/or camera depth data.
- **3D Extension** — migrate to a 3D LiDAR (e.g., YDLIDAR TG30) for volumetric tracking.

---

## 👥 Authors

| Name | Institution | Contact |
|---|---|---|
| Kadir YOKUŞ | Marmara University, EEE | kadiryokus@marun.edu.tr |
| **Emre Berkin ÇETİN** | Marmara University, EEE | cetinemreberkin@gmail.com |
| Osman Eren KÖSE | Marmara University, EEE | osman.eren@marun.edu.tr |

> Developed as part of *EE4084 Kalman Filtering*, Marmara University, Spring 2025.

---

## 📄 Citation

If you use this code in your research, please cite:

```bibtex
@misc{yokus2025lidar,
  title   = {Real-Time Multi-Target Tracking Using {YDLIDAR G2} and
             Unscented {Kalman} Filter with {CTRV} Motion Model},
  author  = {Yoku{\c{s}}, Kadir and {\c{C}}etin, Emre Berkin and
             K{\"o}se, Osman Eren},
  year    = {2025},
  school  = {Marmara University},
  note    = {EE4084 Kalman Filtering Project Report}
}
```

---

## 📚 References

1. M. Sualeh and G.-W. Kim, "Dynamic multi-LiDAR based multiple object detection and tracking," *Sensors*, vol. 19, no. 6, 2019.
2. Y. Cui *et al.*, "Automatic vehicle tracking with roadside lidar data for the connected-vehicles system," *IEEE Intelligent Systems*, vol. 34, no. 3, pp. 44–51, 2019.
3. J. Choi *et al.*, "Multi-target tracking using a 3D-LiDAR sensor for autonomous vehicles," in *Proc. ITSC*, 2013, pp. 881–886.
4. D. Deng, "DBSCAN clustering algorithm based on density," in *Proc. IFEEA*, 2020, pp. 949–953.
5. A. Suresh and N. Anantharaman, "Enhanced human segmentation from 2D LiDAR data using DBSCAN clustering," in *Proc. SPCOM*, 2024.
6. S. Lee *et al.*, "Grid-based DBSCAN clustering accelerator for LiDAR's point cloud," *Electronics*, vol. 13, no. 17, 2024.

---

<p align="center">
  Made with ❤️ at Marmara University · Istanbul, Turkey
</p>
