# Quadrotor Trajectory Tracking Controllers

A ROS 2 monorepo containing multiple quadrotor trajectory tracking controllers, shared libraries, and experiment tooling. Each package is a git submodule and can be used independently or together.

## Repository Structure

```
src/
├── quad_trajectories/         # Shared trajectory library (9 trajectory types)
├── quad_platforms/            # Platform abstraction layer (sim & hardware)
├── nr_standard/               # Newton-Raphson standard controller
├── nr_enhanced_v2/            # Newton-Raphson enhanced v2 controller
├── nr_diff_flat/              # Differential-flatness NR controller
├── nmpc_acados_euler_err/     # Nonlinear MPC via Acados solver
├── ROS2Logger/                # Structured experiment logging & analysis
└── data_analysis/             # Generated log files and analysis notebooks
```

## Packages

### Shared Libraries

| Package | Description |
|---------|-------------|
| [quad_trajectories](quad_trajectories/) | JAX-based trajectory definitions providing position-level outputs — derivatives are computed on demand via autodiff |
| [quad_platforms](quad_platforms/) | Abstract platform interface with concrete implementations for Gazebo X500 (sim) and Holybro X500 V2 (hardware) |
| [ROS2Logger](ROS2Logger/) | Structured CSV logging with automatic Jupyter notebook generation for analysis and plotting |

### Controllers

| Package | Approach | Key Idea |
|---------|----------|----------|
| [nr_standard](nr_standard/) | Newton-Raphson | Iterative feedback linearization with optional integral CBFs |
| [nr_enhanced_v2](nr_enhanced_v2/) | Enhanced Newton-Raphson | Adds state Jacobian and reference rate terms for improved dynamic tracking |
| [nr_diff_flat](nr_diff_flat/) | Differential-Flatness NR | Exploits quadrotor flat outputs to compute thrust and body rates directly |
| [nmpc_acados_euler_err](nmpc_acados_euler_err/) | Nonlinear MPC | Acados-based optimization with error-state cost and wrapped yaw |

## Quick Start

### Prerequisites

- ROS 2 (Humble or later)
- Python 3.10+
- [JAX](https://github.com/google/jax) and jaxlib
- [px4_msgs](https://github.com/PX4/px4_msgs) ROS 2 package
- [Acados](https://docs.acados.org/) (for NMPC only)

### Clone and Build

```bash
mkdir -p ~/ws_clean_traj/src && cd ~/ws_clean_traj/src
git clone --recurse-submodules <repo-url> .

cd ~/ws_clean_traj
colcon build --symlink-install
source install/setup.bash
```

### Run a Controller

All controllers share the same CLI interface:

```bash
ros2 run <controller_pkg> run_node --platform <sim|hw> --trajectory <type> [options]
```

**Example — fly a helix in simulation with logging:**

```bash
ros2 run nr_standard run_node --platform sim --trajectory helix --log
```

### Common CLI Options

| Flag | Description |
|------|-------------|
| `--platform {sim,hw}` | Target platform (required) |
| `--trajectory {hover,yaw_only,circle_horz,circle_vert,fig8_horz,fig8_vert,helix,sawtooth,triangle}` | Trajectory type (required) |
| `--hover-mode {1..8}` | Hover sub-mode (modes 1-4 for hardware) |
| `--log` | Enable CSV data logging |
| `--log-file NAME` | Custom log filename (requires `--log`) |
| `--double-speed` | 2x trajectory speed |
| `--short` | Short variant (fig8_vert) |
| `--spin` | Enable yaw rotation during trajectory |
| `--flight-period SEC` | Custom flight duration in seconds |

## Architecture

```
quad_trajectories ──┐
                    ├──> Controller Node ──> PX4 via px4_msgs
quad_platforms ─────┘         │
                              │
                        ROS2Logger ──> data_analysis/
```

- **Trajectories** supply position-level references `[x, y, z, yaw]`; controllers compute needed derivatives via JAX autodiff.
- **Platforms** abstract away mass and thrust-throttle conversions so the same controller code runs in simulation and on hardware.
- **ROS2Logger** hooks into node shutdown to dump structured CSVs and auto-generate Jupyter analysis notebooks.

## License

MIT
