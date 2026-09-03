# OpenArm ROS 2 软件包

简体中文 | [English](README.md)

本仓库提供 OpenArm 的 ROS 2、ros2_control 和 MoveIt 2 集成。本分支保留上游的关节名称、控制器和启动接口，只把真机通信后端替换为 ZK V1.4 CAN FD。

## 软件包

- `openarm_hardware`：ZK 真机的 ros2_control 硬件插件。
- `openarm_bringup`：双臂控制器、状态发布和 RViz 启动入口。
- `openarm_bimanual_moveit_config`：双臂 MoveIt 2 配置。
- `openarm`：ROS 2 元软件包。

## 电机与总线约定

- 每条机械臂总线上的 ID 1-7 对应七个关节。
- 可选 ID 8 对应夹爪，与 ID 7 一样使用单电机 MIT 模式。
- ZK 后端强制使用 CAN FD。
- 标称速率为 1 Mbps，数据速率为 5 Mbps。
- 双臂默认右臂使用 `can0`，左臂使用 `can1`。

## 获取源码并构建

以下示例以 ROS 2 Humble 为例：

```bash
mkdir -p ~/openarm_ws/src
cd ~/openarm_ws/src
git clone https://github.com/daisir035/openarm_ros2.git
vcs import < openarm_ros2/openarm.repos

cd ~/openarm_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`openarm.repos` 会拉取与本仓库配套的 ZK 版 `openarm_can`。

## 配置 CAN FD

单臂：

```bash
sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can fd on bitrate 1000000 dbitrate 5000000
```

双臂还需按相同参数配置 `can1`。

## 仿真启动

仿真不连接真机 CAN：

```bash
source ~/openarm_ws/install/setup.bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=openarm_v2.0 \
  use_fake_hardware:=true
```

## 真机启动

```bash
source ~/openarm_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=openarm_v2.0 \
  use_fake_hardware:=false \
  right_can_interface:=can0 \
  left_can_interface:=can1
```

常用参数：

- `arm_type`：`openarm_v1.0` 或 `openarm_v2.0`，也接受 `v10`、`v20` 等别名。
- `use_fake_hardware`：`true` 使用模拟硬件，`false` 使用 ZK 真机。
- `robot_controller`：`joint_trajectory_controller` 或 `forward_position_controller`。
- `right_can_interface` / `left_can_interface`：左右臂 SocketCAN 接口。
- `arm_prefix`：控制器和节点命名空间前缀。

## 硬件插件参数

URDF/xacro 中的 `OpenArmHW` 支持以下硬件参数：

- `can_interface`：SocketCAN 接口，默认 `can0`。
- `hand`：是否启用 ID 8 夹爪，默认 `true`。
- `can_fd`：必须为 `true`。
- `kp_hand`：夹爪 MIT 比例增益，默认 `5.0`。
- `kd_hand`：夹爪 MIT 微分增益，默认 `0.1`。

机械臂启动时会使能电机并缓慢回到零位。首次连接真机前，应确认机械零位、关节方向、CAN 接口和急停条件，避免在人员或障碍物附近直接启动。

## 故障排查

```bash
ip -details link show can0
ip -statistics link show can0
candump can0
```

若日志提示反馈数量不足，请检查电机 ID 是否为 1-7（夹爪为 8）、两端波特率、终端电阻和供电。CAN 2.0 配置不能用于此 ZK 后端。

## 许可证

本项目使用 Apache License 2.0，详见 [LICENSE](LICENSE)。
