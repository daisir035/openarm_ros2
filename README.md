# ROS 2 packages for OpenArm robots

[简体中文](README.zh-CN.md) | English

ROS 2 (Robot Operating System 2) is a modern, open-source framework for building robotic software: https://www.ros.org/

This repository provides ROS 2 integration packages. See the [documentation](https://docs.openarm.dev/software/ros2/install) for details.

This fork keeps the existing ROS 2 interfaces and launch commands while using
the ZK V1.4 CAN FD protocol for motor communication. IDs 1-7 control the arm;
optional ID 8 controls the gripper with the same single-joint MIT mode as ID 7.
Configure SocketCAN for 1 Mbps nominal and 5 Mbps data rates before launching
real hardware.

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/ros2/install)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

[Apache License 2.0](LICENSE)

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
