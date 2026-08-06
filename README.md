# hunter

## Overview

`hunter` groups the ROS2 packages that describe, launch and control the Hunter mobile base in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `hunter` | Metapackage that groups the Hunter ROS2 packages. |
| `hunter_description` | Robot-specific description layer for Hunter, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `hunter_bringup` | Main integration entry point for generating Hunter configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `hunter_hardware` | Live `ros2_control` hardware plugin for the Hunter mobile base, built on the generic `1FAS2RWD` hardware abstraction. |

## Usage

In most cases, start with `hunter_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when a Hunter model is selected from a mobile base meta-description.

The Hunter stack is a robot-specific specialization of `romea_mobile_base`. The mobile base architecture is `1FAS2RWD`; `hunter_description` provides the concrete geometry and generated descriptions, `hunter_hardware` provides the live hardware implementation, and `hunter_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `hunter` project was developed by Jean Laneurit in the context of the TIRREX ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
