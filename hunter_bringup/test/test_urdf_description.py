# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import subprocess

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

import xml.etree.ElementTree as ET


def urdf_xml(mode):

    exe = get_package_prefix("hunter_bringup") + "/lib/hunter_bringup/urdf_description.py"

    return ET.fromstring(
        subprocess.check_output(
            [exe, "mode:" + mode, "base_name:base", "robot_namespace:robot"],
            encoding="utf-8",
        )
    )


def ros2_control_urdf_xml(mode):
    urdf_xml(mode)
    return ET.parse("/tmp/robot_base_ros2_control.urdf")


def test_footprint_link_name():
    assert urdf_xml("live").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert ros2_control_urdf_xml("live").find(
        "ros2_control/hardware/plugin"
    ).text == "hunter_hardware/HunterHardware"

    assert ros2_control_urdf_xml("simulation").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface1FAS2RWD"


def test_controller_filename_name():

    assert (
        urdf_xml("simulation").find("gazebo/plugin/controller_manager_config_file").text
        == get_package_share_directory("hunter_bringup") + "/config/controller_manager.yaml"
    )
