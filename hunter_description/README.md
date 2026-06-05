# hunter_description

## 1) Overview

`hunter_description` provides the robot-specific description layer for the Hunter mobile base.

It extends `romea_mobile_base_description` with the concrete configuration, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate the Hunter robot.

The Hunter mobile base uses:

| Configuration file | Mobile base architecture | Command type |
|---|---|---|
| `config/hunter.yaml` | `1FAS2RWD` | `one_axle_steering` |

This architecture has front axle steering and rear wheel speed control. It is controlled as a one-axle-steering mobile base.

## 2) Robot configuration

The `config/` directory contains:

| File | Purpose |
|---|---|
| `hunter.yaml` | full Hunter mobile base configuration |
| `teleop.yaml` | default one-axle-steering teleoperation configuration |

The robot configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture (`1FAS2RWD`);
* geometry, wheel dimensions and chassis bounding box;
* front axle steering command and feedback information;
* rear wheel speed command and feedback information;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/hunter.urdf.xacro` | main URDF entry point |
| `urdf/hunter.xacro` | Hunter mobile base macro |
| `urdf/hunter.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `urdf/visual/` | visual Xacro fragments for chassis, wheels and arms |
| `meshes/` | Hunter chassis and wheel meshes |

The Hunter macro reuses the `base1FASxxx.chassis.xacro` template from `romea_mobile_base_description` and specializes it with Hunter geometry, link names, joint names and visual meshes.

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/hunter.ros2_control.urdf.xacro` | main `ros2_control` entry point |
| `ros2_control/hunter.ros2_control.xacro` | Hunter `ros2_control` macro |

Depending on the selected mode, the `ros2_control` description selects:

| Mode | Hardware plugin |
|---|---|
| `live` | `hunter_hardware/HunterHardware` |
| `simulation`, `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface1FAS2RWD` |
| `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface1FAS2RWD` |

## 4) Python API

The installed Python module provides helper functions used by `hunter_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file()` | returns the Hunter configuration file path |
| `get_specifications_configuration()` | loads the full robot configuration |
| `get_configuration()` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the Hunter URDF description |
| `generate_ros2_control_description(...)` | generates the Hunter `ros2_control` description |

Example:

```python
from hunter_description import get_configuration

configuration = get_configuration()
```

When `mode` is set to `simulation`, the Python API maps it to `simulation_gazebo_classic` before generating the URDF or `ros2_control` description.

## 5) Relation with other packages

`hunter_description` is the Hunter specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic `1FAS2RWD` description and `ros2_control` templates;
* `hunter_description` provides the Hunter configuration, meshes and Xacro specialization;
* `hunter_hardware` provides the live `ros2_control` hardware plugin;
* `hunter_bringup` uses this package to generate configuration, URDF and `ros2_control` artifacts for live and simulation modes.
