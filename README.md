# aqua2_controller

`aqua2_controller` is a ROS 2 Python package that provides a high-level controller for Aqua2 robots using a **MAVROS-style interface**:

- `go2waypoint(waypoint, timeout)` – blocking waypoint navigation with ETA publishing  
- `pub_rc_override(cmd, timeout, publisher_period_sec)` – RC-style low-level control  
- Automatic calibration & mode setup on startup  

It’s designed to sit on top of existing Aqua2 navigation and autopilot nodes, using their topics/services rather than talking directly to hardware.

---

## Features

- Initializes Aqua2 into **swim mode** and **depth autopilot mode**
- Sends waypoints via `navigation/navigate_waypoint`
- Uses `navigation/distance_to_wp` to detect waypoint arrival
- Estimates forward velocity from odometry and publishes:
  - `waypoint_eta` with `[eta_seconds, distance_m, velocity_mps]`
- Provides a MAVROS-like **RC override** interface (`pub_rc_override`)
- Includes a simple demo `mission()` that visits a sequence of waypoints
- Configurable **default depth, speed, and acceptance radius** via controller constructor

---

## Dependencies

Runtime dependencies (declared in `package.xml`):

- `rclpy`
- `aqua2_interfaces`
- `nav_msgs`
- `std_msgs`
- `std_srvs`
- `robot_localization`
- `ir_utils` (for `get_namespace()` helper)

Python dependencies:

- `numpy`

Make sure these packages are installed in your ROS 2 workspace / environment.

---

## Node Overview

### Node

- **Name:** `aqua2_controller`  
- **Namespace:** determined by `ir_utils.param_tools.get_namespace()`

### Subscribed Topics

- `system/state` (`aqua2_interfaces/RobodevelState`)  
  Robot state, calibration, mode, autopilot mode.

- `navigation/distance_to_wp` (`aqua2_interfaces/Distance`)  
  Distance to the current target waypoint; used to decide when the robot is “at” the waypoint.

- `/a14/navigation/local_position` (`nav_msgs/Odometry`)  
  Local odom; used to estimate horizontal velocity for ETA.

### Published Topics

- `navigation/command_request` (`aqua2_interfaces/AutopilotCommand`)  
  Low-level control commands (surge, heave, yaw/pitch/roll, depth).  
  Used by `pub_rc_override`.

- `navigation/navigate_waypoint` (`aqua2_interfaces/Waypoint`)  
  Waypoint commands in the local frame.

- `waypoint_eta` (`std_msgs/Float32MultiArray`)  
  ETA info in the form `[eta_seconds, distance_m, velocity_mps]` while traveling to a waypoint.

### Service Clients

- `system/calibrate` (`std_srvs/Empty`)
- `system/set_mode` (`aqua2_interfaces/SetString`)
- `autopilot/set_autopilot_mode` (`aqua2_interfaces/SetInt`)
- `set_pose` (`robot_localization/SetPose`)
- `imu/zero_heading` (`std_srvs/Empty`)

These are used during the startup sequence and for pose/heading resets.

---

## Key APIs (Python)

These are methods on the `Controller` class, which you can reuse from other Python nodes if you import this package as a module.

### Constructor

```python
from aqua2_controller.controller import Controller

# Defaults shown here
controller = Controller(
    default_depth=0.5,
    default_speed=0.5,
    acceptance_radius=1.1,
)
```

- `default_depth`: depth used in `go2waypoint` when the waypoint does not specify depth  
- `default_speed`: nominal forward speed used in `go2waypoint`  
- `acceptance_radius`: distance threshold (meters) for considering the robot “at” the waypoint  

### `go2waypoint(waypoint, timeout=900.0) -> bool`

Blocking navigation to a local waypoint:

```python
# waypoint: [x, y] or [x, y, depth] in local frame
controller.go2waypoint([10.0, 0.0, 0.5])
```

- Sends a waypoint command repeatedly on `navigation/navigate_waypoint`
- Monitors `navigation/distance_to_wp` to determine arrival
- Publishes ETA on `waypoint_eta`
- Returns `True` if the waypoint is reached, `False` if it times out

If `depth` is omitted in the waypoint (only `[x, y]`), `default_depth` from the constructor is used.

### `at_waypoint(waypoint) -> bool`

Checks whether the robot is considered “at” the waypoint:

```python
if controller.at_waypoint([10.0, 0.0]):
    ...
```

Internally, this compares `distance_to_target` (from `navigation/distance_to_wp`) to `acceptance_radius`.

### `pub_rc_override(cmd, timeout=0.0, publisher_period_sec=0.03) -> bool`

MAVROS-style RC override wrapper over `AutopilotCommand`:

```python
# cmd = [Forward, Heave, Yaw, Pitch, Roll, Depth]
# Forward/Heave in [-1, 1]; others in controller-appropriate units.
controller.pub_rc_override(
    cmd=[0.5, 0.0, 0.0, 0.0, 0.0, 0.8],
    timeout=2.0,
    publisher_period_sec=0.05,
)
```

- Scales Forward & Heave from [-1, 1] → [0, 1] and sends them as `surge` and `heave`.
- Publishes the same `AutopilotCommand` repeatedly for `timeout` seconds.
- If `timeout <= 0`, publishes once and returns.

---

## Installation

From a ROS 2 workspace (e.g., `~/ros2_ws`):

```bash
cd ~/ros2_ws/src
git clone <your_repo_url> aqua2_controller

cd ..
colcon build --packages-select aqua2_controller
source install/setup.bash
```

Make sure any custom dependencies (`aqua2_interfaces`, `ir_utils`, etc.) are also in your workspace or installed system-wide.

---

## Running the Node

After building and sourcing:

```bash
ros2 run aqua2_controller controller
```

This will:

1. Zero the local pose using `set_pose`
2. Calibrate the robot via `system/calibrate`
3. Set mode to `swimmode`
4. Set autopilot mode to `depth`
5. Run the built-in `mission()` visiting a small set of example waypoints.

You can comment out or modify `mission()` in `main()` if you’d rather control the node externally.

---

## Example: External Control via ROS 2 Service / Python

If you import the controller in another Python node, you can reuse its API:

```python
from aqua2_controller.controller import Controller
import rclpy

def main():
    rclpy.init()
    controller = Controller(
        default_depth=1.0,
        default_speed=0.7,
        acceptance_radius=2.0,
    )

    try:
        controller.go2waypoint([5.0, 0.0])  # uses default_depth
        controller.pub_rc_override([0.0, 0.0, 0.5, 0.0, 0.0, 0.5], timeout=1.0)
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()
```

---

## Configuration & Parameters

Configurable constructor arguments:

- `default_depth` (default: `0.5`)  
- `default_speed` (default: `0.5`)  
- `acceptance_radius` (default: `1.1`)
