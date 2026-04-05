# Code for Submission: "Lightweight Tracking Control for Computationally Constrained Aerial Systems with Newton-Raphson Flow"
Video footage for certain flights may be seen [here](https://youtu.be/H0mwDMPMdxQ)

**Paper:** "Lightweight Tracking Control for Computationally Constrained Aerial Systems with the Newton-Raphson Method", [preprint available](https://arxiv.org/abs/2508.14185)\
**Authors:** Evanns Morales-Cuadrado, Luke Baird, Yorai Wardi, Samuel Coogan\
**Venue:** IEEE Transactions on Control Systems Technology (TCST), 2026


# Environment Setup
1. Users of this code must use the `/src/` folder here in a ROS2 workspace and build
2. Users may also use the `environment.yml` file to set up a conda environment with the necessary packages to run the code below
3. **Docker (in development):** A Docker-based workspace is available at [ws_px4_work](https://github.com/evannsmc/ws_px4_work). Note that this is still under active development, but it aims to simplify the full PX4/ROS2 environment setup

# Cloning This Repository
This repository uses git submodules for the quadrotor packages. Clone with:
```bash
git clone --recurse-submodules git@github.com:evannsmc/MoralesCuadrado_Baird_TCST2025.git
```
If you already cloned without `--recurse-submodules`, initialize the submodules with:
```bash
git submodule update --init --recursive
```

# Motion Capture Packages (Optional)
If you are using a motion capture system for hardware flights, you may need the following packages cloned into your ROS2 workspace `src/` directory:
```bash
# Vicon
git clone git@github.com:evannsmc/vicon4px4.git

# OptiTrack
git clone git@github.com:evannsmc/optitrack4px4.git

# PX4 messages (minimal branch)
git clone -b v1.16_minimal_msgs git@github.com:evannsmc/px4_msgs.git

# Mocap messages and relay nodes
git clone git@github.com:evannsmc/mocap_msgs.git
git clone git@github.com:evannsmc/mocap_px4_relays.git
```
Then build from the workspace root:
```bash
cd ..  # back to workspace root
colcon build --symlink-install
```

---

# Quadrotor Code Instructions

The quadrotor packages (`quad_newton_raphson_flow` and `quad_mpc`) are included as git submodules pointing to their standalone repositories:
- **NR Flow**: [newton_raphson_px4](https://github.com/evannsmc/newton_raphson_px4)
- **NMPC**: [nmpc_acados_px4](https://github.com/evannsmc/nmpc_acados_px4)

## Preliminary
1. Follow the instructions ([here](https://docs.px4.io/main/en/ros/ros2_comm.html)) to set up the PX4 Autopilot Stack, ROS2, Micro XRCE-DDS Agent & Client, and build a ROS2 workspace with the necessary px4 communication repos
2. In the same workspace with the communication folders as above, go to the `/src/` folder and clone this repository (with submodules as described above)
3. Install the conda environment:
```bash
conda env create -f environment.yml
```
4. For the NMPC controller, you must also install [Acados](https://docs.acados.org/installation/index.html) — see the [NMPC package README](src/quad_mpc/README.md) for detailed Acados setup instructions
5. In the root of the workspace, build everything:
```bash
colcon build --symlink-install
```

## Simulation Setup
1. Run the SITL simulation:
```bash
make px4_sitl gazebo-classic
```
2. Run the Micro XRCE Agent as the bridge between ROS2 and PX4:
```bash
MicroXRCEAgent udp4 -p 8888
```

## Running the Quadrotor NR Flow Controller
1. In another terminal, source the workspace and activate your conda environment:
```bash
source install/setup.bash
conda activate <your_env_name>
```
2. Run the controller:
```bash
# Simulation — circle trajectory
ros2 run newton_raphson_px4 run_node --platform sim --trajectory circle_horz

# Hardware — helix trajectory with logging
ros2 run newton_raphson_px4 run_node --platform hw --trajectory helix --log

# Hover mode 3, double speed, with yaw spin
ros2 run newton_raphson_px4 run_node --platform sim --trajectory hover --hover-mode 3 --double-speed --spin
```

### NR Flow CLI Options

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

## Running the Quadrotor NMPC Controller
1. In another terminal, source the workspace and activate your conda environment:
```bash
source install/setup.bash
conda activate <your_env_name>
```
2. Run the controller:
```bash
# Simulation — figure-8 trajectory
ros2 run nmpc_acados_px4 run_node --platform sim --trajectory fig8_horz

# Hardware — helix trajectory with logging
ros2 run nmpc_acados_px4 run_node --platform hw --trajectory helix --log
```

The NMPC CLI options are the same as the NR Flow options listed above.

---

# Blimp Code Instructions

The blimp package (`blimp_mpc_fbl_nr`) includes MPC, FBL (feedback linearization via CBF), and NR Flow controllers for both simulation and hardware.

## Running the Blimp Simulator
1. In a terminal, source the workspace and activate your conda environment:
```bash
source install/setup.bash
conda activate <your_env_name>
```
2. Run a simulation using the `run_blimp_sim` entry point with a controller/trajectory key and a log filename:
```bash
ros2 run blimp_mpc_fbl_nr run_blimp_sim <controller_key> <logfile>
```

### Available Controller Keys for Simulation/Hardware

**NR Flow (`nr`):**
`hardware_nr_circle_horz`, `hardware_nr_circle_vert`, `hardware_nr_fig8_horz`, `hardware_nr_fig8_vert_short`, `hardware_nr_fig8_vert_tall`, `hardware_nr_circle_horz_spin`, `hardware_nr_helix`, `hardware_nr_helix_spin`

**MPC (`mpc`):**
`hardware_mpc_circle_horz`, `hardware_mpc_circle_vert`, `hardware_mpc_fig8_horz`, `hardware_mpc_fig8_vert_short`, `hardware_mpc_fig8_vert_tall`, `hardware_mpc_circle_horz_spin`, `hardware_mpc_helix`, `hardware_mpc_helix_spin`

**FBL (`fbl`):**
`hardware_fbl_circle_horz`, `hardware_fbl_circle_vert`, `hardware_fbl_fig8_horz`, `hardware_fbl_fig8_vert_short`, `hardware_fbl_fig8_vert_tall`, `hardware_fbl_circle_horz_spin`, `hardware_fbl_helix`, `hardware_fbl_helix_spin`

### Example
```bash
# Run NR Flow on a horizontal circle trajectory
ros2 run blimp_mpc_fbl_nr run_blimp_sim hardware_nr_circle_horz log1.log

# Run MPC on a helix trajectory
ros2 run blimp_mpc_fbl_nr run_blimp_sim hardware_mpc_helix log1.log

# Run FBL on a figure-8 trajectory
ros2 run blimp_mpc_fbl_nr run_blimp_sim hardware_fbl_fig8_horz log1.log
```

### Individual Blimp Entry Points
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

---

## Citing this Work
Please see the arxiv preprint [here](https://arxiv.org/abs/2508.14185)

## Authors
Evanns G. Morales-Cuadrado, Luke Baird, Yorai Wardi, Samuel Coogan
