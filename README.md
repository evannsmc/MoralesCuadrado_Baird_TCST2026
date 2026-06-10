# Lightweight Tracking Control for Computationally Constrained Aerial Systems with Newton-Raphson Flow

Code accompanying the IEEE Transactions on Control Systems Technology (TCST) 2026 paper. This repository bundles the **quadrotor** controllers (Newton-Raphson Flow and NMPC) and the **blimp** controllers (MPC, FBL, and NR Flow) used to produce the paper's hardware and simulation results.

**Paper:** "Lightweight Tracking Control for Computationally Constrained Aerial Systems with the Newton-Raphson Method" — [preprint (arXiv:2508.14185)](https://arxiv.org/abs/2508.14185)\
**Authors:** Evanns Morales-Cuadrado, Luke Baird, Yorai Wardi, Samuel Coogan\
**Venue:** IEEE Transactions on Control Systems Technology (TCST), 2026\
**Video:** flight footage [here](https://youtu.be/H0mwDMPMdxQ)

<div align="center">

---

**[<kbd> <br> Clone <br> </kbd>](#cloning-this-repository)** 
**[<kbd> <br> Setup <br> </kbd>](#environment-setup)** 
**[<kbd> <br> Mocap <br> </kbd>](#motion-capture-packages-optional)** 
**[<kbd> <br> Quadrotor <br> </kbd>](#running-the-quadrotor-controllers)** 
**[<kbd> <br> Blimp <br> </kbd>](#running-the-blimp-controllers)** 
**[<kbd> <br> Cite <br> </kbd>](#citing-this-work)** 

---

</div>

<details open>
<summary><b>📖 Table of Contents</b></summary>

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Cloning This Repository](#cloning-this-repository)
- [Environment Setup](#environment-setup)
  - [1. PX4 + ROS 2 + Micro XRCE-DDS](#1-px4--ros-2--micro-xrce-dds)
  - [2. px4_msgs (PX4 v1.16)](#2-px4_msgs-px4-v116)
  - [3. Python environment (conda)](#3-python-environment-conda)
  - [4. Docker (optional)](#4-docker-optional)
  - [5. Build](#5-build)
- [Motion Capture Packages (Optional)](#motion-capture-packages-optional)
- [Running the Quadrotor Controllers](#running-the-quadrotor-controllers)
  - [Simulation bring-up](#simulation-bring-up)
  - [Quadrotor NR Flow](#quadrotor-nr-flow)
  - [Quadrotor NMPC](#quadrotor-nmpc)
  - [CLI Options](#cli-options)
- [Running the Blimp Controllers](#running-the-blimp-controllers)
  - [Blimp simulator](#blimp-simulator)
  - [Controller keys](#controller-keys)
  - [Individual entry points](#individual-entry-points)
- [Citing this Work](#citing-this-work)
- [Authors](#authors)

</details>

## Overview

This repo is a single ROS 2 workspace `src/` that showcases three controller families from the paper:

- **Quadrotor — NR Flow** ([newton_raphson_px4](https://github.com/evannsmc/newton_raphson_px4)): lightweight Newton-Raphson Flow tracking controller with integral CBFs.
- **Quadrotor — NMPC** ([nmpc_acados_px4](https://github.com/evannsmc/nmpc_acados_px4)): acados-based nonlinear MPC, used as the comparison baseline.
- **Blimp — MPC / FBL / NR Flow** (`blimp_mpc_fbl_nr`): the three blimp controllers (model-predictive, feedback-linearization-via-CBF, and Newton-Raphson Flow) for sim and hardware.

The two quadrotor packages are included as **git submodules** that point to their standalone repositories, so they stay in sync with their own development; the blimp package lives directly in this repo.

## Repository Structure

```
MoralesCuadrado_Baird_TCST2026/
├── environment.yml                     # conda environment for all Python deps
└── src/
    ├── blimp_mpc_fbl_nr/               # blimp MPC / FBL / NR Flow (in-tree)
    ├── quad_mpc/                       # submodule -> nmpc_acados_px4   (run as `nmpc_acados_px4`)
    └── quad_newton_raphson_flow/       # submodule -> newton_raphson_px4 (run as `newton_raphson_px4`)
```

> Note: the submodule **directory** names (`quad_mpc`, `quad_newton_raphson_flow`) differ from the **ROS 2 package** names you `ros2 run` (`nmpc_acados_px4`, `newton_raphson_px4`). colcon resolves packages by their `package.xml` name, not the folder name, so always run them by package name.

## Cloning This Repository

The quadrotor packages are git submodules, so clone recursively:

```bash
git clone --recurse-submodules git@github.com:evannsmc/MoralesCuadrado_Baird_TCST2026.git
```

If you already cloned without `--recurse-submodules`, initialize the submodules:

```bash
git submodule update --init --recursive
```

## Environment Setup

Place this repository's `src/` inside a ROS 2 workspace (or use this repo as the workspace) and build. The pieces:

### 1. PX4 + ROS 2 + Micro XRCE-DDS

Follow the [PX4 ROS 2 user guide](https://docs.px4.io/main/en/ros/ros2_comm.html) to install the PX4 Autopilot stack, ROS 2, and the Micro XRCE-DDS Agent & Client.

### 2. px4_msgs (PX4 v1.16)

These controllers target **PX4 v1.16** and require the matching minimal message set. Clone the pinned fork into your workspace `src/`:

```bash
git clone -b v1.16_minimal_msgs git@github.com:evannsmc/px4_msgs.git
```

### 3. Python environment (conda)

The blimp controllers (CasADi/JAX) and the quad controllers share a conda environment:

```bash
conda env create -f environment.yml
conda activate <your_env_name>
```

For the quadrotor **NMPC** controller you must additionally install [Acados](https://docs.acados.org/installation/index.html) — see the [NMPC package README](src/quad_mpc/README.md#acados-setup) for the detailed steps.

### 4. Docker (optional)

A general-purpose PX4 + ROS 2 container is available at **[PX4-ROS2-Docker](https://github.com/evannsmc/PX4-ROS2-Docker)** (ships ROS 2, a prebuilt `px4_msgs` v1.16 overlay, and a Python venv). Mount this workspace into the container and build with its `make` targets — see that repo's README. The conda path above remains the simplest option for native setup.

### 5. Build

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Motion Capture Packages (Optional)

For hardware flights that use an external motion-capture system for state estimation, clone the bridge for your system plus its supporting packages into the workspace `src/`:

```bash
cd <your_ws>/src

# Pick the bridge for your mocap system:
git clone git@github.com:evannsmc/vicon4px4.git        # Vicon
git clone git@github.com:evannsmc/optitrack4px4.git    # OptiTrack

# Supporting packages for the mocap -> PX4 pipeline (skip px4_msgs if already cloned):
git clone -b v1.16_minimal_msgs git@github.com:evannsmc/px4_msgs.git
git clone git@github.com:evannsmc/mocap_msgs.git
git clone git@github.com:evannsmc/mocap_px4_relays.git

cd .. && colcon build --symlink-install
```

See the [vicon4px4](https://github.com/evannsmc/vicon4px4) and [optitrack4px4](https://github.com/evannsmc/optitrack4px4) READMEs for system-specific configuration (rigid-body/frame names, host/IP, and `ROS_DOMAIN_ID`).

## Running the Quadrotor Controllers

The quadrotor packages are run by their ROS 2 package names. Full details for each live in their standalone READMEs:
[newton_raphson_px4](https://github.com/evannsmc/newton_raphson_px4) · [nmpc_acados_px4](https://github.com/evannsmc/nmpc_acados_px4).

### Simulation bring-up

In separate terminals:

```bash
# 1) PX4 SITL
make px4_sitl gazebo-classic

# 2) Micro XRCE-DDS Agent (ROS 2 <-> PX4 bridge)
MicroXRCEAgent udp4 -p 8888
```

Then source the workspace and activate your conda environment in the controller terminal:

```bash
source install/setup.bash
conda activate <your_env_name>
```

### Quadrotor NR Flow

```bash
# Simulation — circle trajectory
ros2 run newton_raphson_px4 run_node --platform sim --trajectory circle_horz

# Hardware — helix trajectory with logging
ros2 run newton_raphson_px4 run_node --platform hw --trajectory helix --log

# Hover mode 3, double speed, with yaw spin
ros2 run newton_raphson_px4 run_node --platform sim --trajectory hover --hover-mode 3 --double-speed --spin
```

### Quadrotor NMPC

```bash
# Simulation — figure-8 trajectory
ros2 run nmpc_acados_px4 run_node --platform sim --trajectory fig8_horz

# Hardware — helix trajectory with logging
ros2 run nmpc_acados_px4 run_node --platform hw --trajectory helix --log
```

### CLI Options

Both quadrotor controllers share the same CLI:

| Flag | Description |
| --- | --- |
| `--platform {sim,hw}` | Target platform (required) |
| `--trajectory {hover,yaw_only,circle_horz,...}` | Trajectory type (required) |
| `--hover-mode {1..8}` | Hover sub-mode (1-4 for hardware) |
| `--log` | Enable CSV data logging |
| `--log-file NAME` | Custom log filename |
| `--double-speed` | 2x trajectory speed |
| `--short` | Short variant (fig8_vert) |
| `--spin` | Enable yaw rotation |
| `--flight-period SEC` | Custom flight duration |
| `--ff` | Feedforward marker (only valid with `fig8_contraction`) |

## Running the Blimp Controllers

The blimp package (`blimp_mpc_fbl_nr`) includes MPC, FBL (feedback linearization via CBF), and NR Flow controllers for both simulation and hardware.

### Blimp simulator

After sourcing the workspace and activating your conda environment, run a simulation via the `run_blimp_sim` entry point with a controller/trajectory key and a log filename:

```bash
ros2 run blimp_mpc_fbl_nr run_blimp_sim <controller_key> <logfile>
```

Examples:

```bash
# NR Flow on a horizontal circle
ros2 run blimp_mpc_fbl_nr run_blimp_sim hardware_nr_circle_horz log1.log

# MPC on a helix
ros2 run blimp_mpc_fbl_nr run_blimp_sim hardware_mpc_helix log1.log

# FBL on a figure-8
ros2 run blimp_mpc_fbl_nr run_blimp_sim hardware_fbl_fig8_horz log1.log
```

### Controller keys

**NR Flow (`nr`):** `hardware_nr_circle_horz`, `hardware_nr_circle_vert`, `hardware_nr_fig8_horz`, `hardware_nr_fig8_vert_short`, `hardware_nr_fig8_vert_tall`, `hardware_nr_circle_horz_spin`, `hardware_nr_helix`, `hardware_nr_helix_spin`

**MPC (`mpc`):** `hardware_mpc_circle_horz`, `hardware_mpc_circle_vert`, `hardware_mpc_fig8_horz`, `hardware_mpc_fig8_vert_short`, `hardware_mpc_fig8_vert_tall`, `hardware_mpc_circle_horz_spin`, `hardware_mpc_helix`, `hardware_mpc_helix_spin`

**FBL (`fbl`):** `hardware_fbl_circle_horz`, `hardware_fbl_circle_vert`, `hardware_fbl_fig8_horz`, `hardware_fbl_fig8_vert_short`, `hardware_fbl_fig8_vert_tall`, `hardware_fbl_circle_horz_spin`, `hardware_fbl_helix`, `hardware_fbl_helix_spin`

### Individual entry points

Each controller/trajectory combination also has a dedicated entry point:

**NR Flow:**
```bash
ros2 run blimp_mpc_fbl_nr run_wardi_circle_horz
ros2 run blimp_mpc_fbl_nr run_wardi_circle_vert
ros2 run blimp_mpc_fbl_nr run_wardi_fig8_horz
ros2 run blimp_mpc_fbl_nr run_wardi_fig8_vert_short
ros2 run blimp_mpc_fbl_nr run_wardi_fig8_vert_tall
ros2 run blimp_mpc_fbl_nr run_wardi_circle_horz_spin
ros2 run blimp_mpc_fbl_nr run_wardi_helix
ros2 run blimp_mpc_fbl_nr run_wardi_helix_spin
```

**NLMPC:**
```bash
ros2 run blimp_mpc_fbl_nr run_nlmpc_circle_horz
ros2 run blimp_mpc_fbl_nr run_nlmpc_circle_vert
ros2 run blimp_mpc_fbl_nr run_nlmpc_fig8_horz
ros2 run blimp_mpc_fbl_nr run_nlmpc_fig8_vert_short
ros2 run blimp_mpc_fbl_nr run_nlmpc_fig8_vert_tall
ros2 run blimp_mpc_fbl_nr run_nlmpc_circle_horz_spin
ros2 run blimp_mpc_fbl_nr run_nlmpc_helix
ros2 run blimp_mpc_fbl_nr run_nlmpc_helix_spin
```

**CBF (FBL):**
```bash
ros2 run blimp_mpc_fbl_nr run_cbf_circle_horz
ros2 run blimp_mpc_fbl_nr run_cbf_circle_vert
ros2 run blimp_mpc_fbl_nr run_cbf_fig8_horz
ros2 run blimp_mpc_fbl_nr run_cbf_fig8_vert_short
ros2 run blimp_mpc_fbl_nr run_cbf_fig8_vert_tall
ros2 run blimp_mpc_fbl_nr run_cbf_circle_horz_spin
ros2 run blimp_mpc_fbl_nr run_cbf_helix
ros2 run blimp_mpc_fbl_nr run_cbf_helix_spin
```

## Citing this Work

If you use this code, please cite the paper. See the [arXiv preprint](https://arxiv.org/abs/2508.14185) for the current citation.

## Authors

Evanns G. Morales-Cuadrado, Luke Baird, Yorai Wardi, Samuel Coogan
