#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET


ROBOT_DESCRIPTION = """
<robot name="openarm">
  <ros2_control name="openarm_left_hardware_interface" type="system">
    <hardware>
      <plugin>openarm_hardware/OpenArmHW</plugin>
      <param name="can_interface">can0</param>
      <param name="kp1">70.0</param>
      <param name="kd1">2.75</param>
    </hardware>
  </ros2_control>
  <ros2_control name="openarm_right_hardware_interface" type="system">
    <hardware>
      <plugin>openarm_hardware/OpenArmHW</plugin>
      <param name="can_interface">can1</param>
    </hardware>
  </ros2_control>
</robot>
"""


def load_launch(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_launch(path):
    module = load_launch(path)
    root = ET.fromstring(
        module.single_motor_test_description(
            ROBOT_DESCRIPTION, "can2", "8.0", "0.8"
        )
    )
    controls = {
        control.get("name"): control.find("hardware")
        for control in root.findall("ros2_control")
    }
    left = controls["openarm_left_hardware_interface"]
    left_params = {param.get("name"): param.text for param in left.findall("param")}
    assert left.findtext("plugin") == "openarm_hardware/OpenArmHW"
    assert left_params["single_motor_test"] == "true"
    assert left_params["can_interface"] == "can2"
    assert left_params["kp1"] == "8.0"
    assert left_params["kd1"] == "0.8"

    right = controls["openarm_right_hardware_interface"]
    assert right.findtext("plugin") == "mock_components/GenericSystem"
    assert right.find("param[@name='can_interface']") is None


if __name__ == "__main__":
    root = Path(__file__).parents[2]
    check_launch(root / "openarm_bringup/launch/openarm.bimanual.launch.py")
    check_launch(root / "openarm_bimanual_moveit_config/launch/demo.launch.py")
