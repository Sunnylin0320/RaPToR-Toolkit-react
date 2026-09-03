# RaPToR Web — ROS 2 Bridge

The backend for RaPToR Web: a single ROS 2 node (`ros_bridge.py`) that
subscribes to and publishes on the iRobot Create 3's ROS 2 topics, acts as an
action client for its built-in behaviours, and exposes two WebSocket
connections that the React frontend (`raptor-web`) connects to.

Part of the dissertation *"From Desktop to Browser: A Web-Based Redesign of
the RaPToR Robot Teleoperation Interface"* (Yung Chia Lin, MSc Computer
Science, University of Bath).

## Architecture

```
React Frontend  <--- WebSocket (sensors, :6789) ---   ros_bridge.py  <--- ROS 2 topics/actions ---   Create 3
   (browser)     ---  WebSocket (commands, :6790) --->  (this repo)                                (sim or real)
```

Three independent processes, communicating only through these defined
protocols — no direct function calls between them.

## Requirements

- ROS 2 Humble (developed and tested on Ubuntu 22.04)
- Python 3.10.12
- Gazebo Sim (Fortress) 6.18.0, invoked via `ign gazebo`

## Setup

```bash
git clone https://github.com/Sunnylin0320/RaPToR-Toolkit-react.git
cd RaPToR-Toolkit-react
```

Install ROS 2 on the machine that will connect to the robot (simulated or
real). Make sure to install the ROS 2 distribution matching your Linux
image — this project does not use a colcon workspace; `ros_bridge.py` is
run directly with `python3`, following the same approach as the original
RaPToR Toolkit it's built on.

## Running

### Simulation (Gazebo)

This project was developed on an ARM64 Ubuntu virtual machine (Parallels
Desktop on Apple Silicon) with no GPU passthrough, which required a specific
startup sequence to get Gazebo running reliably. Use the included script:

```bash
./start_gazebo_reliable.sh
```


### The bridge itself

```bash
cd RaPToR-Toolkit-react
source /opt/ros/humble/setup.bash
python3 ros_bridge.py
```


## Files

This repository is built on top of Otto Chu's original RaPToR Toolkit. Only
one file from the original toolkit is still required:

- `ros_bridge.py` — new; the ROS 2 bridge node for the web version
- `sensor_websocket.py` — from the original toolkit, unmodified; provides
  the WebSocket server functions `ros_bridge.py` imports
  (`start_websocket_server`, `update_sensor_state`, `websocket_clients`)

The following files from the original toolkit are **not** used by the web
version and are kept in this repository only for reference to the original
project: `main.py`, `actions.py`, `getters.py`, `move.py`, `sensors.py`,
`template_generator.py`, `terminal.py`. These implement the original
Tkinter desktop GUI and are not required to run RaPToR Web.

## Development Environment

This project was developed and tested on an ARM64 Ubuntu 22.04 virtual
machine (Parallels Desktop on Apple Silicon), which has no GPU passthrough
and therefore no hardware-accelerated rendering available to Gazebo. This is
why `start_gazebo_reliable.sh` forces software rendering
(`LIBGL_ALWAYS_SOFTWARE=1`, `--render-engine ogre`) and retries the launch
automatically rather than assuming it succeeds on the first attempt.

If you're running on a machine with GPU passthrough or native Linux with a
GPU, Gazebo may launch reliably without this script — `ros2 launch
irobot_create_ignition_bringup create3_ignition.launch.py world:=depot`
directly may work fine. The retry script is included because it's necessary
in this project's actual development environment, not because it's required
in general.

## Acknowledgements

Built on top of the original RaPToR Toolkit by Otto Chu (BSc project,
University of Bath).
